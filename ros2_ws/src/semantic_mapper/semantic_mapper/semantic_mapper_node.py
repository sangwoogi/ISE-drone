"""semantic_mapper_node — detections + depth + TF -> SemanticObjectArray + RViz markers (Phase 6).

Pipeline (plan §19): Detection -> robust depth -> camera XYZ -> TF to map -> association -> publish.
Database persistence is done by semantic_database_node, which subscribes /semantic_map/objects.
"""
from __future__ import annotations

import time
from collections import Counter

import numpy as np
import rclpy
import rclpy.duration
import rclpy.parameter
import rclpy.time
from rclpy.executors import ExternalShutdownException
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from drone_interfaces.msg import SemanticObject, SemanticObjectArray

from .association import ObjectAssociator
from .projection import bbox_to_camera_point
from .relations import relations_for


class SemanticMapperNode(Node):
    def __init__(self):
        super().__init__('semantic_mapper')
        defaults = dict(detections_topic='/perception/tracks', depth_topic='/camera/depth/image_raw',
                        camera_info_topic='/camera/color/camera_info', map_frame='map', camera_frame='',
                        sync_slop_s=0.15, association_distance_m=0.5, position_alpha=0.3, min_observations=3,
                        min_confidence=0.4, depth_min_m=0.2, depth_max_m=8.0, publish_rate_hz=2.0, near_threshold_m=1.5)
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        # TF on a dedicated helper node with its own spin thread: TF keeps flowing while a detection callback
        # blocks in lookup_transform(timeout=...). (spin_thread=True on *this* node would add it to a second
        # executor and abort in rcl.)
        self.tf_node = rclpy.create_node(
            'semantic_mapper_tf',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=bool(self.get_parameter('use_sim_time').value))])
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.declare_parameter('tf_future_tolerance_s', 0.05)
        self.assoc = ObjectAssociator(float(self.p['association_distance_m']), float(self.p['position_alpha']),
                                      int(self.p['min_observations']))
        self.K = None
        self.color_size = (1, 1)
        self.depth_buf = []     # (stamp_s, np.ndarray) ring buffer
        self.stats = Counter()  # why detections were dropped / how many were projected
        self.create_timer(5.0, self._log_stats)

        self.create_subscription(CameraInfo, self.p['camera_info_topic'], self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.p['depth_topic'], self._on_depth, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, self.p['detections_topic'], self._on_dets, 10)
        self.pub_objects = self.create_publisher(SemanticObjectArray, '/semantic_map/objects', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/semantic_map/markers', 10)
        self.pub_frame = self.create_publisher(SemanticObjectArray, '/perception/objects_3d', 10)
        self.create_timer(1.0 / float(self.p['publish_rate_hz']), self._publish_map)
        self.get_logger().info('semantic_mapper ready (waiting for camera_info / depth / detections)')

    def _on_info(self, msg: CameraInfo):
        self.K = list(msg.k)
        self.color_size = (int(msg.width), int(msg.height))

    def _on_depth(self, msg: Image):
        depth = self.bridge.imgmsg_to_cv2(msg)  # 32FC1 (m) or 16UC1 (mm)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.depth_buf.append((t, depth))
        self.depth_buf = self.depth_buf[-15:]

    def _depth_for(self, t: float):
        if not self.depth_buf:
            return None
        ts, d = min(self.depth_buf, key=lambda e: abs(e[0] - t))
        return d if abs(ts - t) <= float(self.p['sync_slop_s']) else None

    def _on_dets(self, msg: Detection2DArray):
        if self.K is None:
            self.stats['no_camera_info'] += 1
            return
        if not msg.detections:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        depth = self._depth_for(t)
        if depth is None:
            self.stats['no_depth_in_slop'] += 1
            return
        cam_frame = self.p['camera_frame'] or msg.header.frame_id
        tf = self._lookup(cam_frame, msg.header.stamp)
        if tf is None:
            self.stats['no_tf'] += 1
            return
        # Detections are in colour-image pixels; the depth image may have another resolution (x500_depth:
        # colour 1280x960 vs depth 640x480, same optical axis). Scale bbox and intrinsics to the depth grid.
        # TODO Phase 3/6: use the depth camera's own CameraInfo instead of assuming identical FOV.
        sx = depth.shape[1] / float(self.color_size[0])
        sy = depth.shape[0] / float(self.color_size[1])
        K = [self.K[0] * sx, 0.0, self.K[2] * sx, 0.0, self.K[4] * sy, self.K[5] * sy, 0.0, 0.0, 1.0]

        frame_objs = SemanticObjectArray()
        frame_objs.header.stamp = msg.header.stamp
        frame_objs.header.frame_id = self.p['map_frame']
        for det in msg.detections:
            if not det.results:
                continue
            cls, conf = det.results[0].hypothesis.class_id, det.results[0].hypothesis.score
            if conf < float(self.p['min_confidence']):
                self.stats['low_confidence'] += 1
                continue
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            bbox = ((cx - det.bbox.size_x / 2) * sx, (cy - det.bbox.size_y / 2) * sy,
                    (cx + det.bbox.size_x / 2) * sx, (cy + det.bbox.size_y / 2) * sy)
            pc = bbox_to_camera_point(depth, bbox, K)
            if pc is None:
                self.stats['no_valid_depth'] += 1
                continue
            self.stats['projected'] += 1
            ps = PointStamped()
            ps.header = msg.header
            ps.point.x, ps.point.y, ps.point.z = pc
            pm = do_transform_point(ps, tf).point
            obj = self.assoc.update(cls, (pm.x, pm.y, pm.z), float(conf), t)
            frame_objs.objects.append(self._to_msg(obj))
        if frame_objs.objects:
            self.pub_frame.publish(frame_objs)

    def _lookup(self, cam_frame: str, stamp):
        """map <- cam_frame at `stamp`; a stamp slightly ahead of the newest TF (sensor vs 20 Hz odom) falls
        back to the latest available transform instead of dropping the detection."""
        map_frame = self.p['map_frame']
        try:
            return self.tf_buffer.lookup_transform(map_frame, cam_frame, stamp,
                                                   timeout=rclpy.duration.Duration(seconds=0.05))
        except tf2_ros.ExtrapolationException as e:
            try:
                latest = self.tf_buffer.lookup_transform(map_frame, cam_frame, rclpy.time.Time())
                dt = abs((rclpy.time.Time.from_msg(stamp) - rclpy.time.Time.from_msg(latest.header.stamp)).nanoseconds) * 1e-9
                if dt <= float(self.get_parameter('tf_future_tolerance_s').value):
                    return latest
                self.get_logger().warn(f'TF {map_frame}<-{cam_frame}: stamp {dt * 1e3:.0f} ms away from newest TF ({e})',
                                       throttle_duration_sec=5.0)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f'TF {map_frame}<-{cam_frame} unavailable: {e}', throttle_duration_sec=5.0)
        return None

    def _log_stats(self):
        if self.stats:
            self.get_logger().info('mapper stats: ' + ', '.join(f'{k}={v}' for k, v in sorted(self.stats.items()))
                                   + f', objects={len(self.assoc.objects)} confirmed={len(self.assoc.confirmed())}')

    def _to_msg(self, o) -> SemanticObject:
        m = SemanticObject()
        m.id, m.class_name, m.confidence = o.id, o.class_name, float(o.confidence)
        m.position.x, m.position.y, m.position.z = (float(v) for v in o.position)   # rosidl rejects ints/numpy
        m.size.x, m.size.y, m.size.z = (float(v) for v in o.size)
        m.first_seen.sec, m.first_seen.nanosec = int(o.first_seen), int((o.first_seen % 1) * 1e9)
        m.last_seen.sec, m.last_seen.nanosec = int(o.last_seen), int((o.last_seen % 1) * 1e9)
        m.observation_count = int(o.observation_count)
        m.relations = list(o.relations)
        return m

    def _publish_map(self):
        objs = self.assoc.confirmed()
        for o in objs:
            o.relations = relations_for(o, objs, float(self.p['near_threshold_m']))
        arr = SemanticObjectArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.p['map_frame']
        arr.objects = [self._to_msg(o) for o in objs]
        self.pub_objects.publish(arr)

        markers = MarkerArray()
        for i, o in enumerate(objs):
            for kind, mid in (('sphere', 0), ('text', 1)):
                m = Marker()
                m.header = arr.header
                m.ns, m.id = 'semantic_objects', i * 2 + mid
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y, m.pose.position.z = o.position
                m.pose.orientation.w = 1.0
                if kind == 'sphere':
                    m.type = Marker.SPHERE
                    m.scale.x = m.scale.y = m.scale.z = 0.3
                    m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.2, 0.8
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.pose.position.z += 0.3
                    m.scale.z = 0.25
                    m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                    m.text = f'{o.id} ({o.confidence:.2f}, n={o.observation_count})'
                markers.markers.append(m)
        self.pub_markers.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapperNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except BaseException:  # second SIGINT during teardown is harmless
            pass
        rclpy.try_shutdown()

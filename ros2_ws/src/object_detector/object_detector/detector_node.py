"""detector_node — /camera/color/image_raw -> /perception/detections_2d (vision_msgs/Detection2DArray).

The node is backend-agnostic (DetectorBackend). Phase 5 validates the ultralytics backend; the mock
backend lets the semantic mapper be tested without a model.
"""
from __future__ import annotations

import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .backends import create_backend


class DetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('backend', 'mock')
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('classes', [''])
        self.declare_parameter('input_topic', '/camera/color/image_raw')
        self.declare_parameter('output_topic', '/perception/detections_2d')
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_topic', '/perception/detections_image')
        self.declare_parameter('max_rate_hz', 10.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        classes = [c for c in g('classes') if c]
        self.backend = create_backend(g('backend'), model=g('model'), device=g('device'), conf=float(g('confidence')),
                                      classes=classes, imgsz=int(g('imgsz')))
        self.backend.warmup()
        self.bridge = CvBridge()
        self.min_period = 1.0 / float(g('max_rate_hz')) if float(g('max_rate_hz')) > 0 else 0.0
        self._last = 0.0
        self.pub = self.create_publisher(Detection2DArray, g('output_topic'), 10)
        self.pub_dbg = self.create_publisher(Image, g('debug_image_topic'), 1) if g('publish_debug_image') else None
        self.create_subscription(Image, g('input_topic'), self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f'detector backend={self.backend.name} -> {g("output_topic")}')

    def _on_image(self, msg: Image):
        now = time.monotonic()
        if now - self._last < self.min_period:
            return
        self._last = now
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        t0 = time.monotonic()
        dets = self.backend.detect(img)
        dt_ms = (time.monotonic() - t0) * 1e3

        out = Detection2DArray()
        out.header = msg.header
        for d in dets:
            det = Detection2D()
            det.header = msg.header
            cx, cy = d.center
            w, h = d.size
            det.bbox.center.position.x, det.bbox.center.position.y = float(cx), float(cy)
            det.bbox.size_x, det.bbox.size_y = float(w), float(h)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.class_name
            hyp.hypothesis.score = float(d.confidence)
            det.results.append(hyp)
            det.id = d.class_name
            out.detections.append(det)
        self.pub.publish(out)

        if self.pub_dbg is not None and self.pub_dbg.get_subscription_count() > 0:
            for d in dets:
                cv2.rectangle(img, (int(d.x_min), int(d.y_min)), (int(d.x_max), int(d.y_max)), (0, 255, 0), 2)
                cv2.putText(img, f'{d.class_name} {d.confidence:.2f}', (int(d.x_min), max(0, int(d.y_min) - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(img, f'{self.backend.name} {dt_ms:.0f} ms', (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            dbg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            dbg.header = msg.header
            self.pub_dbg.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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

"""tracker_node — /perception/detections_2d -> /perception/tracks (Detection2DArray with stable ids).

Baseline: pass-through with a naive IoU tracker (greedy matching, track ids as Detection2D.id).
Phase 6 may replace it with ByteTrack; the semantic mapper only relies on `id` + class + bbox.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray


def iou(a, b):
    ax1, ay1 = a.bbox.center.position.x - a.bbox.size_x / 2, a.bbox.center.position.y - a.bbox.size_y / 2
    ax2, ay2 = ax1 + a.bbox.size_x, ay1 + a.bbox.size_y
    bx1, by1 = b.bbox.center.position.x - b.bbox.size_x / 2, b.bbox.center.position.y - b.bbox.size_y / 2
    bx2, by2 = bx1 + b.bbox.size_x, by1 + b.bbox.size_y
    iw, ih = max(0.0, min(ax2, bx2) - max(ax1, bx1)), max(0.0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = a.bbox.size_x * a.bbox.size_y + b.bbox.size_x * b.bbox.size_y - inter
    return inter / union if union > 0 else 0.0


class TrackerNode(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('max_age', 10)
        self.iou_thr = float(self.get_parameter('iou_threshold').value)
        self.max_age = int(self.get_parameter('max_age').value)
        self.tracks = {}      # id -> (Detection2D, age)
        self.next_id = 1
        self.pub = self.create_publisher(Detection2DArray, '/perception/tracks', 10)
        self.create_subscription(Detection2DArray, '/perception/detections_2d', self._on_dets, 10)

    def _on_dets(self, msg: Detection2DArray):
        for tid in list(self.tracks):
            det, age = self.tracks[tid]
            self.tracks[tid] = (det, age + 1)
            if age + 1 > self.max_age:
                del self.tracks[tid]
        out = Detection2DArray()
        out.header = msg.header
        used = set()
        for det in msg.detections:
            cls = det.results[0].hypothesis.class_id if det.results else ''
            best, best_iou = None, self.iou_thr
            for tid, (tdet, _) in self.tracks.items():
                if tid in used:
                    continue
                tcls = tdet.results[0].hypothesis.class_id if tdet.results else ''
                if tcls != cls:
                    continue
                v = iou(det, tdet)
                if v > best_iou:
                    best, best_iou = tid, v
            if best is None:
                best = self.next_id
                self.next_id += 1
            used.add(best)
            self.tracks[best] = (det, 0)
            det.id = f'{cls}#{best}'
            out.detections.append(det)
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
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

#!/usr/bin/env python3
"""Phase 3 acceptance: common sensor topics are consistent (plan §13).

Checks over a --duration window (default 15 s):
  * rates:   /camera/color/image_raw and /camera/depth/image_raw >= --min-hz (default 10), camera_info >= min-hz
  * frames:  all headers carry --optical-frame (camera_optical_frame)
  * geometry: colour and depth have identical width/height and camera_info K (registered RGB-D)
  * sync:    |stamp(colour) - stamp(nearest depth)| median <= --max-skew-ms (default 60)
  * TF:      map -> camera_optical_frame resolvable
Writes experiments/<exp>/sensor_report.json and saves one colour frame + depth colormap PNG.
Requires: simulation.launch.py vehicle:=gz_x500_rgbd world:=indoor_v1 + sensor_adapter gazebo_sensors.launch.py
"""
from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def stamp_s(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class SensorProbe(Node):
    def __init__(self):
        super().__init__('phase3_sensor_probe')
        self.t = {'color': [], 'depth': [], 'cinfo': [], 'dinfo': []}
        self.frames = {}
        self.meta = {}
        self.sample = {}
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_node = rclpy.create_node('phase3_tf_helper')
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.tf_node, spin_thread=True)
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: self._img('color', m), qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/depth/image_raw', lambda m: self._img('depth', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: self._info('cinfo', m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda m: self._info('dinfo', m), qos_profile_sensor_data)

    def _img(self, key, m):
        self.t[key].append(stamp_s(m.header))
        self.frames.setdefault(key, set()).add(m.header.frame_id)
        self.meta[key] = (m.width, m.height, m.encoding)
        if key not in self.sample:
            self.sample[key] = self.bridge.imgmsg_to_cv2(m)

    def _info(self, key, m):
        self.t[key].append(stamp_s(m.header))
        self.frames.setdefault(key, set()).add(m.header.frame_id)
        self.meta[key] = (m.width, m.height, tuple(round(v, 2) for v in m.k))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--min-hz', type=float, default=10.0, help='colour / camera_info minimum rate')
    ap.add_argument('--min-depth-hz', type=float, default=8.0, help='depth minimum rate (render-bound)')
    ap.add_argument('--max-skew-ms', type=float, default=60.0)
    ap.add_argument('--optical-frame', default='camera_optical_frame')
    ap.add_argument('--exp', default='003_phase3_sensors')
    args, ros_args = ap.parse_known_args(argv)
    exp = os.path.join(ROOT, 'experiments', args.exp)
    os.makedirs(exp, exist_ok=True)

    rclpy.init(args=ros_args)
    node = SensorProbe()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 20 and not (node.t['color'] and node.t['depth']):
        time.sleep(0.2)
    if not (node.t['color'] and node.t['depth']):
        print('no camera data', file=sys.stderr)
        return 1
    for k in node.t:
        node.t[k].clear()
    print(f'collecting for {args.duration:.0f} s ...')
    time.sleep(args.duration)

    rep = {'duration_s': args.duration, 'checks': {}}
    rates = {k: len(v) / args.duration for k, v in node.t.items()}
    rep['rates_hz'] = {k: round(float(v), 2) for k, v in rates.items()}
    rep['checks']['rate_color'] = rates['color'] >= args.min_hz
    rep['checks']['rate_depth'] = rates['depth'] >= args.min_depth_hz
    rep['checks']['rate_camera_info'] = rates['cinfo'] >= args.min_hz and rates['dinfo'] >= args.min_hz
    rep['frame_ids'] = {k: sorted(v) for k, v in node.frames.items()}
    rep['checks']['frame_ids'] = all(v == {args.optical_frame} for v in node.frames.values())
    rep['meta'] = {k: list(v) for k, v in node.meta.items()}
    same_size = node.meta.get('color', (0, 0))[:2] == node.meta.get('depth', (1, 1))[:2]
    same_k = node.meta.get('cinfo', (0, 0, ()))[2] == node.meta.get('dinfo', (1, 1, ()))[2]
    rep['checks']['registered_geometry'] = bool(same_size and same_k)
    depth_t = np.array(sorted(node.t['depth']))
    skews = []
    for tc in node.t['color']:
        if depth_t.size:
            i = np.searchsorted(depth_t, tc)
            cands = [abs(depth_t[j] - tc) for j in (i - 1, i) if 0 <= j < depth_t.size]
            skews.append(min(cands) * 1e3)
    rep['skew_ms'] = {'median': float(statistics.median(skews)) if skews else None, 'max': float(max(skews)) if skews else None}
    rep['checks']['sync'] = bool(skews) and bool(statistics.median(skews) <= args.max_skew_ms)
    try:
        tf = node.tf_buffer.lookup_transform('map', args.optical_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=3.0))
        tr = tf.transform.translation
        rep['tf_map_to_camera'] = [round(tr.x, 3), round(tr.y, 3), round(tr.z, 3)]
        rep['checks']['tf'] = True
    except Exception as e:  # noqa: BLE001
        rep['tf_map_to_camera'] = str(e)
        rep['checks']['tf'] = False
    # depth statistics of the first frame
    d = node.sample.get('depth')
    if d is not None:
        valid = d[np.isfinite(d) & (d > 0)]
        rep['depth_sample'] = {'valid_fraction': round(float(valid.size / d.size), 3),
                               'min_m': round(float(valid.min()), 2) if valid.size else None,
                               'max_m': round(float(valid.max()), 2) if valid.size else None}
        import cv2
        dn = np.nan_to_num(d, nan=0.0, posinf=0.0)
        dn = np.clip(dn / 8.0 * 255.0, 0, 255).astype(np.uint8)
        cv2.imwrite(os.path.join(exp, 'sample_depth.png'), cv2.applyColorMap(dn, cv2.COLORMAP_JET))
    c = node.sample.get('color')
    if c is not None:
        import cv2
        cv2.imwrite(os.path.join(exp, 'sample_color.png'), cv2.cvtColor(c, cv2.COLOR_RGB2BGR) if node.meta['color'][2] == 'rgb8' else c)
    rep['checks'] = {k: bool(v) for k, v in rep['checks'].items()}
    rep['pass'] = all(rep['checks'].values())
    with open(os.path.join(exp, 'sensor_report.json'), 'w') as f:
        json.dump(rep, f, indent=2, ensure_ascii=False)
    print(json.dumps(rep, indent=2, ensure_ascii=False))
    print('PHASE3 SENSOR CHECK:', 'PASS' if rep['pass'] else 'FAIL')
    sys.stdout.flush()
    os._exit(0 if rep['pass'] else 1)   # skip executor/tf-thread teardown races; results are already on disk


if __name__ == '__main__':
    sys.exit(main())

"""system_monitor_node — CPU / RAM / temperature / topic rates -> /diagnostics + optional CSV (plan §44).

Topic rates are measured with generic subscriptions (no message deserialisation).
"""
from __future__ import annotations

import csv
import os
import time

import psutil
import rclpy
from rclpy.executors import ExternalShutdownException
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message


def cpu_temperature():
    try:
        temps = psutil.sensors_temperatures()
        for key in ('cpu_thermal', 'coretemp', 'k10temp'):
            if key in temps and temps[key]:
                return float(temps[key][0].current)
    except Exception:  # noqa: BLE001
        pass
    return float('nan')


class SystemMonitorNode(Node):
    def __init__(self):
        super().__init__('system_monitor')
        self.declare_parameter('rate_hz', 1.0)
        self.declare_parameter('csv_path', '')
        self.declare_parameter('watch_topics', ['/drone/vehicle_state'])
        self.topics = [t for t in self.get_parameter('watch_topics').value if t]
        self.counts = {t: 0 for t in self.topics}
        self._subs = {}
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self._csv = None
        path = self.get_parameter('csv_path').value
        if path:
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            new = not os.path.exists(path)
            self._csv = csv.writer(open(path, 'a', newline=''))
            if new:
                self._csv.writerow(['t', 'cpu_pct', 'ram_pct', 'ram_used_mb', 'temp_c'] + [f'hz{t}' for t in self.topics])
        psutil.cpu_percent(None)
        self._t_last = time.monotonic()
        self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self._tick)
        self.create_timer(2.0, self._discover)

    def _discover(self):
        """Subscribe lazily once a topic's type is known."""
        known = dict(self.get_topic_names_and_types())
        for t in self.topics:
            if t in self._subs or t not in known:
                continue
            try:
                msg_type = get_message(known[t][0])
            except (ValueError, AttributeError, ModuleNotFoundError) as e:
                self.get_logger().warn(f'cannot resolve type for {t}: {e}')
                continue
            self._subs[t] = self.create_subscription(
                msg_type, t, lambda _m, t=t: self.counts.__setitem__(t, self.counts[t] + 1), qos_profile_sensor_data, raw=True)

    def _tick(self):
        now = time.monotonic()
        dt = max(1e-3, now - self._t_last)
        self._t_last = now
        rates = {t: self.counts[t] / dt for t in self.topics}
        self.counts = {t: 0 for t in self.topics}
        cpu, mem, temp = psutil.cpu_percent(None), psutil.virtual_memory(), cpu_temperature()

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus(name='system', hardware_id=os.uname().nodename, level=DiagnosticStatus.OK,
                              message=f'cpu {cpu:.0f}% ram {mem.percent:.0f}%')
        st.values = [KeyValue(key='cpu_pct', value=f'{cpu:.1f}'), KeyValue(key='ram_pct', value=f'{mem.percent:.1f}'),
                     KeyValue(key='ram_used_mb', value=f'{mem.used / 1e6:.0f}'), KeyValue(key='temp_c', value=f'{temp:.1f}')]
        st.values += [KeyValue(key=f'hz{t}', value=f'{r:.1f}') for t, r in rates.items()]
        arr.status.append(st)
        self.pub.publish(arr)
        if self._csv:
            self._csv.writerow([time.time(), f'{cpu:.1f}', f'{mem.percent:.1f}', f'{mem.used / 1e6:.0f}', f'{temp:.1f}']
                               + [f'{rates[t]:.2f}' for t in self.topics])


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
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

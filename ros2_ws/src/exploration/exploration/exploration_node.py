"""exploration_node — frontier goals from /slam/map at fixed altitude (Phase 8 V3). Baseline: publishes
the best frontier as /exploration/goal (PoseStamped); mission_manager EXPLORE will consume it."""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

from .frontier import cluster_frontiers, frontier_cells


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration')
        self.declare_parameter('altitude_m', 1.2)
        self.declare_parameter('min_cluster_size', 5)
        self.pub = self.create_publisher(PoseStamped, '/exploration/goal', 10)
        self.create_subscription(OccupancyGrid, '/slam/map', self._on_map, 10)

    def _on_map(self, msg: OccupancyGrid):
        grid = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        clusters = cluster_frontiers(frontier_cells(grid), int(self.get_parameter('min_cluster_size').value))
        if not clusters:
            return
        r, c, _ = max(clusters, key=lambda k: k[2])
        goal = PoseStamped()
        goal.header = msg.header
        goal.pose.position.x = msg.info.origin.position.x + (c + 0.5) * msg.info.resolution
        goal.pose.position.y = msg.info.origin.position.y + (r + 0.5) * msg.info.resolution
        goal.pose.position.z = float(self.get_parameter('altitude_m').value)
        goal.pose.orientation.w = 1.0
        self.pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
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

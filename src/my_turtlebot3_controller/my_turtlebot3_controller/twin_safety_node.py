#!/usr/bin/env python3
"""
Twin Safety Node — Digital Twin dual-scan safety fusion for Nutrient Nexus.

Subscribes to LiDAR from BOTH the real robot (/scan) and the simulation
(/sim/scan). If EITHER sensor detects an obstacle within `stop_distance`,
forward motion is blocked on BOTH output channels.

This enables the simulation to act as a predictive safety shield: if the
digital twin detects an obstacle the real robot hasn't seen yet (e.g.,
from a different angle or through predictive path simulation), it can
pre-emptively halt the physical robot.

Topic pipeline:
    /scan     (real)  ──┐
                        ├─→ TwinSafetyNode ──→ /cmd_vel     (real robot)
    /sim/scan (sim)  ──┘                   └─→ /sim/cmd_vel (sim robot)
                  ↑
            /cmd_vel_raw (input from explorer / Nav2)
"""

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class TwinSafetyNode(Node):
    def __init__(self) -> None:
        super().__init__('twin_safety_node')

        # Declare configurable parameters
        self.declare_parameter('real_scan_topic', '/scan')
        self.declare_parameter('sim_scan_topic', '/sim/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('real_cmd_topic', '/cmd_vel')
        self.declare_parameter('sim_cmd_topic', '/sim/cmd_vel')
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('front_angle_deg', 30.0)

        self.real_scan_topic: str = self.get_parameter(
            'real_scan_topic').value
        self.sim_scan_topic: str = self.get_parameter(
            'sim_scan_topic').value
        self.input_cmd_topic: str = self.get_parameter(
            'input_cmd_topic').value
        self.real_cmd_topic: str = self.get_parameter(
            'real_cmd_topic').value
        self.sim_cmd_topic: str = self.get_parameter(
            'sim_cmd_topic').value
        self.stop_distance: float = float(
            self.get_parameter('stop_distance').value)
        self.front_angle_deg: float = float(
            self.get_parameter('front_angle_deg').value)

        # Obstacle detection state for both sources
        self.real_blocked: bool = False
        self.sim_blocked: bool = False
        self.real_min_distance: float = float('inf')
        self.sim_min_distance: float = float('inf')

        # Use BEST_EFFORT QoS for LiDAR — matches Gazebo Sim publisher QoS
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(
            LaserScan, self.real_scan_topic,
            self._real_scan_cb, scan_qos)

        self.create_subscription(
            LaserScan, self.sim_scan_topic,
            self._sim_scan_cb, scan_qos)

        self.create_subscription(
            Twist, self.input_cmd_topic,
            self._cmd_cb, 10)

        self.real_pub = self.create_publisher(
            Twist, self.real_cmd_topic, 10)
        self.sim_pub = self.create_publisher(
            Twist, self.sim_cmd_topic, 10)

        self.get_logger().info(
            f'Twin Safety Node started '
            f'(real: {self.real_scan_topic}, sim: {self.sim_scan_topic}, '
            f'stop_distance={self.stop_distance}m)')

    # ------------------------------------------------------------------ #
    #  Scan callbacks                                                      #
    # ------------------------------------------------------------------ #
    def _real_scan_cb(self, msg: LaserScan) -> None:
        self.real_min_distance, self.real_blocked = \
            self._evaluate_front_obstacle(msg)

    def _sim_scan_cb(self, msg: LaserScan) -> None:
        self.sim_min_distance, self.sim_blocked = \
            self._evaluate_front_obstacle(msg)

    def _evaluate_front_obstacle(
        self, msg: LaserScan
    ) -> Tuple[float, bool]:
        """Evaluate whether an obstacle exists in the front arc."""
        front_ranges = self._get_front_arc_distances(
            msg, self.front_angle_deg)

        valid: List[float] = [
            r for r in front_ranges
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if not valid:
            return float('inf'), False

        min_distance = min(valid)
        blocked = min_distance < self.stop_distance
        return min_distance, blocked

    def _get_front_arc_distances(
        self, scan_msg: LaserScan, front_angle_deg: float
    ) -> List[float]:
        """Extract LiDAR ranges within the front arc (±front_angle_deg)."""
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        front_angle_rad = math.radians(front_angle_deg)

        selected: List[float] = []
        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            # Normalize angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= front_angle_rad:
                selected.append(distance)

        return selected

    # ------------------------------------------------------------------ #
    #  Command velocity callback                                           #
    # ------------------------------------------------------------------ #
    def _cmd_cb(self, msg: Twist) -> None:
        """Filter velocity — block forward motion if either sensor detects obstacle."""
        safe_cmd = Twist()
        blocked: bool = self.real_blocked or self.sim_blocked
        forward_requested: bool = msg.linear.x > 0.0

        if blocked and forward_requested:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            # Allow turning so robot can navigate away
            safe_cmd.angular.z = msg.angular.z

            source = []
            if self.real_blocked:
                source.append(f'REAL ({self.real_min_distance:.2f}m)')
            if self.sim_blocked:
                source.append(f'SIM ({self.sim_min_distance:.2f}m)')

            self.get_logger().warn(
                f'TWIN SAFETY STOP: Obstacle detected by '
                f'{" & ".join(source)}. Blocking forward motion.')
        else:
            safe_cmd = msg

        self.real_pub.publish(safe_cmd)
        self.sim_pub.publish(safe_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

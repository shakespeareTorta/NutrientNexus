#!/usr/bin/env python3
"""
Safety Stop Node — LiDAR-based collision guard for Nutrient Nexus.

Sits between velocity command sources and the actual robot /cmd_vel topic.
If any obstacle is detected within `stop_distance` in the front arc,
forward motion is blocked while rotation is still allowed (so the robot
can turn away from the obstacle).

Topic pipeline:
    /cmd_vel_nav (Nav2)  ─┐
                          ├─→ /cmd_vel_raw (mux) ──→ SafetyStopNode ──→ /cmd_vel (robot)
    /cmd_vel_treatment ──┘

This provides defence-in-depth beyond Nav2's built-in collision_monitor,
and critically protects the treatment actuation phase which bypasses Nav2.
"""

import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SafetyStopNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_stop_node')

        # Declare configurable parameters
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('front_angle_deg', 30.0)

        self.scan_topic: str = self.get_parameter(
            'scan_topic').get_parameter_value().string_value
        self.input_cmd_topic: str = self.get_parameter(
            'input_cmd_topic').get_parameter_value().string_value
        self.output_cmd_topic: str = self.get_parameter(
            'output_cmd_topic').get_parameter_value().string_value
        self.stop_distance: float = self.get_parameter(
            'stop_distance').get_parameter_value().double_value
        self.front_angle_deg: float = self.get_parameter(
            'front_angle_deg').get_parameter_value().double_value

        self.wall_detected: bool = False
        self.latest_min_front_distance: float = float('inf')

        # Use BEST_EFFORT QoS for LiDAR — matches Gazebo Sim publisher QoS
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            self.input_cmd_topic,
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_cmd_topic,
            10
        )

        self.get_logger().info(
            f'Safety Stop Node started '
            f'({self.input_cmd_topic} → {self.output_cmd_topic}, '
            f'stop_distance={self.stop_distance}m, '
            f'front_arc=±{self.front_angle_deg}°)')

    def scan_callback(self, msg: LaserScan) -> None:
        """Process LiDAR scan and update obstacle detection state."""
        front_distances = self._get_front_arc_distances(
            msg, self.front_angle_deg)

        valid_ranges: List[float] = [
            r for r in front_distances
            if math.isfinite(r) and msg.range_min < r < msg.range_max
        ]

        if valid_ranges:
            self.latest_min_front_distance = min(valid_ranges)
            self.wall_detected = self.latest_min_front_distance < self.stop_distance
        else:
            self.latest_min_front_distance = float('inf')
            self.wall_detected = False

    def cmd_callback(self, msg: Twist) -> None:
        """Filter velocity commands — block forward motion if obstacle ahead."""
        safe_cmd = Twist()
        forward_requested: bool = msg.linear.x > 0.0

        if self.wall_detected and forward_requested:
            # Block forward motion but allow rotation (so robot can turn away)
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = 0.0
            safe_cmd.angular.x = 0.0
            safe_cmd.angular.y = 0.0
            safe_cmd.angular.z = msg.angular.z

            self.get_logger().warn(
                f'SAFETY STOP: Obstacle at {self.latest_min_front_distance:.2f}m '
                f'(threshold: {self.stop_distance}m). Blocking forward motion.')
        else:
            safe_cmd = msg

        self.cmd_pub.publish(safe_cmd)

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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyStopNode()
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

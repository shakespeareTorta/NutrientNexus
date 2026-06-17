#!/usr/bin/env python3
"""
Safety Stop Node — Multi-sector LiDAR collision guard for Nutrient Nexus.

Sits between velocity command sources and the actual robot /cmd_vel topic.
Uses 5-sector LiDAR scanning (adapted from OceanDeepSeek's obstacle avoider)
to provide intelligent safety filtering:

  FRONT        : ±front_angle_deg      hard-stop trigger
  FRONT_LEFT   : +front..+side deg     early warning left
  FRONT_RIGHT  : -side..-front deg     early warning right
  LEFT         : +side..+rear deg      open-side assessment
  RIGHT        : -rear..-side deg      open-side assessment

Improvements over the original binary stop/go:
  - 5-sector scanning for full forward-hemisphere awareness
  - Pre-steering nudge: gentle angular correction when diagonal sectors
    close in, before a hard stop is needed
  - Narrow object detection: per-ray threshold catches thin objects
    (chair legs, bottles) that only hit 1-3 LiDAR rays
  - Scan staleness detection: blocks all forward motion if LiDAR data
    is stale (cable disconnected, Gazebo crash, etc.)
  - All parameters externalised via declare_parameter()

Topic pipeline (unchanged):
    /cmd_vel_nav (Nav2)  ─┐
                          ├─→ /cmd_vel_raw (mux) ──→ SafetyStopNode ──→ /cmd_vel (robot)
    /cmd_vel_treatment ──┘
"""

import math
import json
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from my_turtlebot3_controller.qos import STATE_QOS
from my_turtlebot3_controller.lidar_utils import sector_min, narrow_object_in_sector


class SafetyStopNode(Node):
    def __init__(self) -> None:
        super().__init__('safety_stop_node')

        # ── Topic parameters ──────────────────────────────────────────────
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')

        # ── Distance parameters (metres) ──────────────────────────────────
        self.declare_parameter('stop_distance', 0.30)
        self.declare_parameter('narrow_obj_dist', 0.25)

        # ── Sector angle parameters (degrees, half-width from forward) ────
        self.declare_parameter('front_angle_deg', 20.0)
        self.declare_parameter('side_angle_deg', 70.0)
        self.declare_parameter('rear_angle_deg', 130.0)

        # ── Pre-steering parameters ───────────────────────────────────────
        self.declare_parameter('nudge_factor', 0.4)
        self.declare_parameter('nudge_turn_speed', 0.45)

        # ── Staleness detection ───────────────────────────────────────────
        self.declare_parameter('scan_stale_sec', 1.0)

        # ── Read all parameters ───────────────────────────────────────────
        self.scan_topic: str = self.get_parameter(
            'scan_topic').get_parameter_value().string_value
        self.input_cmd_topic: str = self.get_parameter(
            'input_cmd_topic').get_parameter_value().string_value
        self.output_cmd_topic: str = self.get_parameter(
            'output_cmd_topic').get_parameter_value().string_value

        self._base_stop_distance: float = self.get_parameter(
            'stop_distance').get_parameter_value().double_value
        self.stop_distance: float = self._base_stop_distance
        
        self._base_narrow_dist: float = self.get_parameter(
            'narrow_obj_dist').get_parameter_value().double_value
        self.narrow_dist: float = self._base_narrow_dist
        
        self.weather_status = 'sunny'

        self.front_deg: float = self.get_parameter(
            'front_angle_deg').get_parameter_value().double_value
        self.side_deg: float = self.get_parameter(
            'side_angle_deg').get_parameter_value().double_value
        self.rear_deg: float = self.get_parameter(
            'rear_angle_deg').get_parameter_value().double_value

        self.nudge_factor: float = self.get_parameter(
            'nudge_factor').get_parameter_value().double_value
        self.nudge_turn_speed: float = self.get_parameter(
            'nudge_turn_speed').get_parameter_value().double_value

        self.stale_sec: float = self.get_parameter(
            'scan_stale_sec').get_parameter_value().double_value

        # ── Sector distances (updated by scan_callback) ───────────────────
        self.d_front: float = float('inf')
        self.d_front_left: float = float('inf')
        self.d_front_right: float = float('inf')
        self.d_left: float = float('inf')
        self.d_right: float = float('inf')

        # ── Staleness tracking ────────────────────────────────────────────
        self._last_scan_time = None  # rclpy.time.Time or None

        # ── ROS 2 interfaces ──────────────────────────────────────────────
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

        self.create_subscription(
            String,
            '/weather_forecast',
            self._weather_callback,
            STATE_QOS)

        # ── Digital-twin fault override contract (Option B Req 1 & 2) ─────
        # The dashboard (digital entity) injects fault state here; this node
        # (physical execution layer) reacts. SafetyStop NEVER owns fault
        # lifecycle — it only reads the latest declared mode and enacts it.
        self.lidar_fault: str = 'ok'    # ok | degraded | failed
        self.motor_fault: str = 'ok'    # ok | stalled
        self.create_subscription(
            String,
            '/twin_fault_state',
            self._fault_callback,
            STATE_QOS)

        self.cmd_pub = self.create_publisher(
            Twist,
            self.output_cmd_topic,
            10
        )

        # ── Obstacle status mirror (Option B Req 3) ───────────────────────
        # Publishes environment-interaction events so the digital entity
        # (dashboard) reflects what the physical robot senses in the world.
        self.obstacle_pub = self.create_publisher(
            String,
            '/obstacle_status',
            STATE_QOS
        )
        self._last_obstacle_json: str = ''

        self.get_logger().info(
            f'Safety Stop Node started '
            f'({self.input_cmd_topic} → {self.output_cmd_topic})\n'
            f'  stop_distance={self.stop_distance}m  '
            f'narrow_obj={self.narrow_dist}m\n'
            f'  sectors: front=±{self.front_deg}°  '
            f'diag=±{self.side_deg}°  side=±{self.rear_deg}°\n'
            f'  scan_stale_sec={self.stale_sec}s  '
            f'nudge_factor={self.nudge_factor}')

    def _weather_callback(self, msg: String) -> None:
        self.weather_status = msg.data
        if msg.data == 'rainy':
            self.stop_distance = self._base_stop_distance * 1.6
            self.narrow_dist = self._base_narrow_dist * 1.6
        elif msg.data == 'storm':
            self.stop_distance = self._base_stop_distance * 2.0
            self.narrow_dist = self._base_narrow_dist * 2.0
        else:
            self.stop_distance = self._base_stop_distance
            self.narrow_dist = self._base_narrow_dist

    def _fault_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.lidar_fault = data.get('lidar', 'ok')
        self.motor_fault = data.get('motor', 'ok')

    def _publish_obstacle(self, blocked: bool, distance: float, sector: str) -> None:
        payload = json.dumps({
            'blocked': blocked,
            'distance': round(distance, 2) if math.isfinite(distance) else -1.0,
            'sector': sector,
        })
        if payload == self._last_obstacle_json:
            return
        self._last_obstacle_json = payload
        self.obstacle_pub.publish(String(data=payload))

    # ── Scan callback: 5-sector extraction + narrow object detection ──────

    def scan_callback(self, msg: LaserScan) -> None:
        """Update all 5 sector distances from the latest LiDAR frame."""
        fa = self.front_deg
        sa = self.side_deg
        ra = self.rear_deg

        self.d_front       = sector_min(msg, -fa,  fa)
        self.d_front_left  = sector_min(msg,  fa,  sa)
        self.d_front_right = sector_min(msg, -sa, -fa)
        self.d_left        = sector_min(msg,  sa,  ra)
        self.d_right       = sector_min(msg, -ra, -sa)

        # Narrow-object detection: widen the check to ±side_angle_deg.
        # If a thin object is detected and the FRONT sector doesn't already
        # show a problem, pull d_front down so cmd_callback reacts.
        if narrow_object_in_sector(msg, -sa, sa, self.narrow_dist):
            if self.d_front > self.stop_distance:
                self.d_front = min(self.d_front, self.narrow_dist - 0.01)

        # Record scan arrival time for staleness detection
        self._last_scan_time = self.get_clock().now()

    # ── Command callback: safety filter with staleness + pre-steering ─────

    def cmd_callback(self, msg: Twist) -> None:
        """Filter velocity commands with multi-sector intelligence."""
        # ── Motor fault: highest-priority gate, robot is frozen ───────
        if self.motor_fault == 'stalled':
            self.cmd_pub.publish(Twist())
            self._publish_obstacle(True, 0.0, 'MOTOR_FAULT')
            self.get_logger().error(
                'MOTOR FAULT injected: holding robot at zero velocity.',
                throttle_duration_sec=2.0)
            return

        # ── Apply Weather Velocity Scaling ────────────────────────────
        # (Rain -> max_vel scaled by 40% reduction, i.e., 0.6 multiplier)
        if self.weather_status == 'rainy':
            msg.linear.x *= 0.6
        elif self.weather_status == 'storm':
            msg.linear.x *= 0.4

        forward_requested: bool = msg.linear.x > 0.0

        # ── LiDAR fault: a failed sensor blinds the robot ─────────────
        # Treat a 'failed' lidar exactly like missing scan data: forward
        # motion is unsafe and blocked, rotation in place is still allowed.
        if self.lidar_fault == 'failed':
            if forward_requested:
                safe_cmd = Twist()
                safe_cmd.angular.z = msg.angular.z
                self.cmd_pub.publish(safe_cmd)
                self._publish_obstacle(True, -1.0, 'LIDAR_FAULT')
                self.get_logger().error(
                    'LIDAR FAULT injected: sensor blind, blocking forward motion.',
                    throttle_duration_sec=2.0)
                return
            self.cmd_pub.publish(msg)
            return

        # A degraded lidar drops confidence: halve forward speed and widen
        # the stop distance so the robot behaves more conservatively.
        effective_stop_distance = self.stop_distance
        if self.lidar_fault == 'degraded':
            msg.linear.x *= 0.5
            effective_stop_distance = self.stop_distance * 1.5

        # ── Staleness check ───────────────────────────────────────────
        if self._last_scan_time is not None:
            scan_age = (self.get_clock().now() - self._last_scan_time).nanoseconds / 1e9
            if scan_age > self.stale_sec:
                if forward_requested:
                    self.get_logger().error(
                        f'SAFETY STOP: LiDAR data stale ({scan_age:.1f}s > '
                        f'{self.stale_sec}s). Blocking forward motion.',
                        throttle_duration_sec=2.0)
                    safe_cmd = Twist()
                    safe_cmd.angular.z = msg.angular.z  # still allow rotation
                    self.cmd_pub.publish(safe_cmd)
                    self._publish_obstacle(True, -1.0, 'SCAN_STALE')
                    return
        elif forward_requested:
            # No scan received yet at all — block forward motion
            self.get_logger().warn(
                'SAFETY STOP: No LiDAR data received yet. '
                'Blocking forward motion.',
                throttle_duration_sec=2.0)
            safe_cmd = Twist()
            safe_cmd.angular.z = msg.angular.z
            self.cmd_pub.publish(safe_cmd)
            self._publish_obstacle(True, -1.0, 'SCAN_STALE')
            return

        # ── Front obstacle check ──────────────────────────────────────
        front_blocked = self.d_front < effective_stop_distance

        if front_blocked and forward_requested:
            safe_cmd = Twist()
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = msg.angular.z  # allow rotation to escape

            self.get_logger().warn(
                f'SAFETY STOP: Obstacle at {self.d_front:.2f}m '
                f'(threshold: {effective_stop_distance:.2f}m). '
                f'Blocking forward motion.',
                throttle_duration_sec=1.0)
            self.cmd_pub.publish(safe_cmd)
            self._publish_obstacle(True, self.d_front, 'FRONT')
            return

        # ── Pre-steering nudge ────────────────────────────────────────
        # When a diagonal sector is closing in but FRONT is still clear,
        # apply a gentle angular correction to steer the robot away
        # before a hard stop is needed. Only nudge if the upstream
        # command is not already commanding significant rotation.
        safe_cmd = Twist()
        safe_cmd.linear.x = msg.linear.x
        safe_cmd.linear.y = msg.linear.y
        safe_cmd.linear.z = msg.linear.z
        safe_cmd.angular.x = msg.angular.x
        safe_cmd.angular.y = msg.angular.y
        safe_cmd.angular.z = msg.angular.z

        nudge_sector = 'NONE'
        nudge_distance = float('inf')
        if forward_requested and abs(msg.angular.z) < 0.1:
            nudge_threshold = effective_stop_distance * 1.2

            if self.d_front_left < nudge_threshold:
                # Obstacle approaching from left → nudge right
                safe_cmd.angular.z = -self.nudge_turn_speed * self.nudge_factor
                nudge_sector = 'FRONT_LEFT'
                nudge_distance = self.d_front_left
                self.get_logger().debug(
                    f'Pre-steer nudge RIGHT (front_left={self.d_front_left:.2f}m)')

            elif self.d_front_right < nudge_threshold:
                # Obstacle approaching from right → nudge left
                safe_cmd.angular.z = self.nudge_turn_speed * self.nudge_factor
                nudge_sector = 'FRONT_RIGHT'
                nudge_distance = self.d_front_right
                self.get_logger().debug(
                    f'Pre-steer nudge LEFT (front_right={self.d_front_right:.2f}m)')

        self.cmd_pub.publish(safe_cmd)

        if nudge_sector != 'NONE':
            self._publish_obstacle(True, nudge_distance, nudge_sector)
        else:
            self._publish_obstacle(False, self.d_front, 'CLEAR')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero-twist on shutdown so robot doesn't coast
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

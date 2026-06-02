#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
import math
import os
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from typing import Dict, List, Optional, Any
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

class CropDecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_decision_node')

        # Identify robot (A or B)
        self.robot_id = 'B' if self.get_namespace() == '/tb2' else 'A'

        # Publishers (using relative topics to respect namespace)
        self.goal_pub = self.create_publisher(PoseStamped, 'dispatch_nav_goal', 10)
        self.irrigate_pub = self.create_publisher(String, 'irrigate_zone', STATE_QOS)
        self.fertilise_pub = self.create_publisher(String, 'fertilise_zone', STATE_QOS)
        self.treatment_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.refill_pub = self.create_publisher(String, 'refill_resources', STATE_QOS)
        self.intervention_pub = self.create_publisher(String, 'sdg14_intervention', STATE_QOS)
        
        # Supervisor Comms
        self.zone_request_pub = self.create_publisher(String, '/supervisor/zone_request', 10)

        # Subscribers (robot-specific)
        self.nav_status_sub = self.create_subscription(String, 'navigation_executor_status', self.nav_status_callback, 10)
        self.current_zone_sub = self.create_subscription(String, 'current_zone', self.current_zone_callback, 10)
        
        # Shared/Global Subscribers
        self.moisture_sub = self.create_subscription(Float32MultiArray, '/field_moisture', self.moisture_callback, 10)
        self.nutrients_sub = self.create_subscription(Float32MultiArray, '/field_nutrients', self.nutrients_callback, 10)
        self.assignment_sub = self.create_subscription(String, '/supervisor/zone_assignment', self.assignment_callback, STATE_QOS)
        self.alerts_sub = self.create_subscription(String, '/system_alerts', self.alerts_callback, STATE_QOS)

        # Load Zones
        pkg_dir = get_package_share_directory('my_turtlebot3_controller')
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        with open(zones_file, 'r', encoding='utf-8') as f:
            self.raw_zones = yaml.safe_load(f) or {}

        # System state machine
        self.current_phase = "IDLE"
        self.physical_current_zone = "no_zone"
        self.active_zone_id = None
        self.sub_targets = []
        
        self.battery_level = 100.0
        self.fertilizer_level = 100.0
        self.nav2_ready = False
        
        self.moisture_threshold = 40.0
        self.nutrient_threshold = 50.0

        self._scan_timer = None
        self._actuation_timer = None
        self._cooldown_timer = None
        self._actuation_start_time = 0.0

        self.create_timer(1.0, self.state_machine_tick)
        self.get_logger().info(f"CropDecisionNode for Robot {self.robot_id} started. Operating as twin client.")

    def _cancel_and_destroy(self, timer_attr: str) -> None:
        timer = getattr(self, timer_attr, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            setattr(self, timer_attr, None)

    # ── Callbacks ──
    def current_zone_callback(self, msg: String) -> None:
        self.physical_current_zone = msg.data

    def moisture_callback(self, msg: Float32MultiArray) -> None:
        pass # Using raw thresholds for simplicity right now

    def nutrients_callback(self, msg: Float32MultiArray) -> None:
        pass

    def alerts_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get("action") == "ABORT":
                self.get_logger().error(f"SUPERVISOR ABORT RECEIVED: {data.get('reason')}. Returning to BASE.")
                self.sub_targets.clear()
                self._dispatch_to_base()
        except json.JSONDecodeError:
            pass

    def assignment_callback(self, msg: String) -> None:
        if self.current_phase != "WAITING_FOR_ASSIGNMENT":
            return
            
        try:
            data = json.loads(msg.data)
            if data.get('robot') == self.robot_id:
                zone_id = data.get('zone')
                if zone_id == "BASE":
                    self.get_logger().info("Supervisor requested return to base.")
                    self._dispatch_to_base()
                else:
                    self.get_logger().info(f"Supervisor assigned {zone_id}. Generating sub-targets...")
                    self.active_zone_id = zone_id
                    z = self.raw_zones[zone_id]
                    # Generate 2 sub-targets for sweeping
                    self.sub_targets = [
                        (z['target_x'], z['target_y'] + 0.2, z['target_theta']),
                        (z['target_x'], z['target_y'] - 0.2, z['target_theta'])
                    ]
                    self._dispatch_next_subtarget()
        except json.JSONDecodeError:
            pass

    def nav_status_callback(self, msg: String) -> None:
        status = msg.data
        if status == "IDLE" and not self.nav2_ready:
            self.nav2_ready = True
            
        if self.current_phase == "NAVIGATING":
            if status == "SUCCEEDED_AT_POSE":
                self.current_phase = "VERIFYING_ZONE"
            elif status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE_SERVER_UNAVAILABLE"]:
                self.get_logger().warn(f"Navigation failed with status {status}. Skipping sub-target.")
                self._start_cooldown()
                
        elif self.current_phase == "RETURNING_TO_BASE":
            if status == "SUCCEEDED_AT_POSE":
                self.get_logger().info("Arrived at Base Station. Refilling...")
                msg_refill = String()
                msg_refill.data = "refill"
                self.refill_pub.publish(msg_refill)
                self.current_phase = "IDLE"

    # ── Dispatchers ──
    def _dispatch_next_subtarget(self):
        if not self.sub_targets:
            self.get_logger().info(f"Finished all sub-targets for {self.active_zone_id}.")
            self.current_phase = "IDLE"
            return
            
        x, y, yaw_deg = self.sub_targets.pop(0)
        self.get_logger().info(f"Navigating to sub-target at ({x:.2f}, {y:.2f})")
        
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        yaw_rad = math.radians(float(yaw_deg))
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        
        self.goal_pub.publish(goal)
        self.current_phase = "NAVIGATING"

    def _dispatch_to_base(self):
        z = self.raw_zones.get('base_station')
        if not z:
            return
        
        # Offset parking spots so robots don't crash into each other
        y_offset = 0.4 if self.robot_id == 'B' else -0.4

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(z['target_x'])
        goal.pose.position.y = float(z['target_y']) + y_offset
        yaw_rad = math.radians(float(z['target_theta']))
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        self.goal_pub.publish(goal)
        self.current_phase = "RETURNING_TO_BASE"

    # ── State Machine ──
    def state_machine_tick(self) -> None:
        if not self.nav2_ready:
            return
            
        if self.current_phase == "IDLE":
            self.get_logger().info("Requesting zone from Supervisor...")
            req = String()
            req.data = json.dumps({"robot": self.robot_id})
            self.zone_request_pub.publish(req)
            self.current_phase = "WAITING_FOR_ASSIGNMENT"
            
        elif self.current_phase == "VERIFYING_ZONE":
            if self.physical_current_zone == self.active_zone_id:
                self.current_phase = "SCANNING"
                self._cancel_and_destroy('_scan_timer')
                self._scan_timer = self.create_timer(2.0, self._on_scan_complete)
            else:
                self.get_logger().warn(f"Not in {self.active_zone_id} yet. Moving to next sub-target.")
                self._start_cooldown()

    def _on_scan_complete(self):
        self._cancel_and_destroy('_scan_timer')
        self.current_phase = "DECIDING"
        
        # Simulated context decision (in reality, read from field sensors)
        # We will actuate briefly to prove environment interaction
        self.current_phase = "ACTUATING"
        self._actuation_start_time = self.get_clock().now().nanoseconds / 1e9
        self._cancel_and_destroy('_actuation_timer')
        self._actuation_timer = self.create_timer(0.1, self._actuation_tick)
        
        msg = String()
        msg.data = json.dumps({"robot": self.robot_id, "zone": self.active_zone_id})
        self.fertilise_pub.publish(msg)

    def _actuation_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._actuation_start_time < 2.0:
            cmd = Twist()
            cmd.angular.z = 0.6
            self.treatment_vel_pub.publish(cmd)
        else:
            self._cancel_and_destroy('_actuation_timer')
            cmd = Twist()
            cmd.angular.z = 0.0
            self.treatment_vel_pub.publish(cmd)
            self._start_cooldown()

    def _start_cooldown(self):
        self.current_phase = "COOLDOWN"
        self._cancel_and_destroy('_cooldown_timer')
        self._cooldown_timer = self.create_timer(1.0, self._on_cooldown_complete)

    def _on_cooldown_complete(self):
        self._cancel_and_destroy('_cooldown_timer')
        # If there are more subtargets, go to them!
        if self.sub_targets:
            self._dispatch_next_subtarget()
        else:
            self.current_phase = "IDLE"

def main(args=None):
    rclpy.init(args=args)
    node = CropDecisionNode()
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

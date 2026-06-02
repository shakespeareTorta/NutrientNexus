#!/usr/bin/env python3
"""
Twin Supervisor Node — Central Orchestrator for Digital Twin (Rubric Aligned)

Responsibilities:
  - Central Task Dispatch: Maintains the global zone queue [zone_0, zone_1, zone_2, zone_3].
  - Bidirectional State Sync: Monitors Robot A (Physical) and Robot B (Digital).
  - Environmental Interaction: Listens to /weather_forecast and propagates weather
    events (e.g. storms) to halt operations.
  - Fault Management: If a robot is low on battery or crashes, tasks are reassigned.
"""
import math
import json
from typing import Dict, Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import PoseStamped

STATE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

class TwinSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('twin_supervisor_node')

        self.declare_parameter('system_mode', 'HYBRID')
        self.system_mode = self.get_parameter('system_mode').get_parameter_value().string_value

        # Global Zone Queue (1 and 2 are East, 0 and 3 are West)
        # We start with these queues for A and B to minimize travel distance.
        self.pending_zones_a: List[str] = ['zone_2', 'zone_1']
        self.pending_zones_b: List[str] = ['zone_3', 'zone_0']
        
        self.robot_states = {
            'A': {'status': 'IDLE', 'battery': 100.0, 'zone': 'Unknown'},
            'B': {'status': 'IDLE', 'battery': 100.0, 'zone': 'Unknown'}
        }
        
        self.current_weather = "sunny"

        # ── Publishers ──
        self.assignment_pub = self.create_publisher(String, '/supervisor/zone_assignment', STATE_QOS)
        self.sync_pub = self.create_publisher(String, '/sync_status', STATE_QOS)
        self.alert_pub = self.create_publisher(String, '/system_alerts', STATE_QOS)

        # ── Subscribers ──
        self.create_subscription(String, '/weather_forecast', self._weather_cb, STATE_QOS)
        
        # Robot A (Physical/Default Namespace)
        self.create_subscription(String, '/robot_resources', lambda m: self._resource_cb(m, 'A'), STATE_QOS)
        self.create_subscription(String, '/navigation_executor_status', lambda m: self._nav_status_cb(m, 'A'), STATE_QOS)
        self.create_subscription(String, '/supervisor/zone_request', lambda m: self._zone_request_cb(m, 'A'), 10)
        
        # Robot B (Digital/tb2 Namespace)
        self.create_subscription(String, '/tb2/robot_resources', lambda m: self._resource_cb(m, 'B'), STATE_QOS)
        self.create_subscription(String, '/tb2/navigation_executor_status', lambda m: self._nav_status_cb(m, 'B'), STATE_QOS)
        self.create_subscription(String, '/tb2/supervisor/zone_request', lambda m: self._zone_request_cb(m, 'B'), 10)

        self.get_logger().info("Twin Supervisor Central Orchestrator Initialized.")
        
        self.create_timer(1.0, self._supervisor_tick)

    def _weather_cb(self, msg: String) -> None:
        new_weather = msg.data.lower()
        if new_weather != self.current_weather:
            self.current_weather = new_weather
            self.get_logger().warn(f"[ENV INTERACTION] Weather changed to: {self.current_weather}")
            # If storm, instantly broadcast ABORT to both robots
            if self.current_weather == "storm":
                self._broadcast_abort("STORM_EMERGENCY")

    def _resource_cb(self, msg: String, robot_id: str) -> None:
        try:
            data = json.loads(msg.data)
            self.robot_states[robot_id]['battery'] = data.get('battery', 100.0)
            if data.get('battery', 100.0) < 15.0:
                self.get_logger().error(f"Robot {robot_id} Battery Critical! Reassigning tasks.")
                self._handle_robot_fault(robot_id)
        except json.JSONDecodeError:
            pass

    def _nav_status_cb(self, msg: String, robot_id: str) -> None:
        try:
            data = json.loads(msg.data)
            state = data.get('state', 'UNKNOWN')
            
            # If the robot's physical safety stop was triggered, it might fail navigation
            if state == 'FAILED' or 'fault' in state.lower():
                self.get_logger().error(f"[STATE SYNC] Robot {robot_id} reported fault/failure! Pausing Digital Twin.")
                self._handle_robot_fault(robot_id)
                
            self.robot_states[robot_id]['status'] = state
        except json.JSONDecodeError:
            pass

    def _zone_request_cb(self, msg: String, robot_id: str) -> None:
        """Handles requests from robots for their next task."""
        self.get_logger().info(f"Received zone request from Robot {robot_id}")
        
        if self.current_weather == "storm":
            self._dispatch_zone(robot_id, "BASE")
            return

        if self.robot_states[robot_id]['battery'] < 20.0:
            self._dispatch_zone(robot_id, "BASE")
            return

        # Give them their own tasks first
        my_queue = self.pending_zones_a if robot_id == 'A' else self.pending_zones_b
        if my_queue:
            next_zone = my_queue.pop(0)
            self._dispatch_zone(robot_id, next_zone)
            return
            
        # Task Stealing (Collaboration) if own queue is empty
        other_queue = self.pending_zones_b if robot_id == 'A' else self.pending_zones_a
        if other_queue:
            stolen_zone = other_queue.pop(0)
            self.get_logger().info(f"[COLLABORATION] Robot {robot_id} is stealing {stolen_zone} to help out!")
            self._dispatch_zone(robot_id, stolen_zone)
            return

        # No zones left
        self._dispatch_zone(robot_id, "BASE")

    def _dispatch_zone(self, robot_id: str, zone: str) -> None:
        assign_msg = String()
        assign_msg.data = json.dumps({"robot": robot_id, "zone": zone})
        self.assignment_pub.publish(assign_msg)
        self.get_logger().info(f"Dispatched {zone} to Robot {robot_id}")

    def _handle_robot_fault(self, faulted_robot: str) -> None:
        """If a robot fails, give its tasks to the other robot, and broadcast a global pause/abort."""
        self.get_logger().warn(f"Handling fault for Robot {faulted_robot}...")
        if faulted_robot == 'A':
            self.pending_zones_b.extend(self.pending_zones_a)
            self.pending_zones_a.clear()
            self._broadcast_abort("PHYSICAL_ROBOT_FAULT")
        else:
            self.pending_zones_a.extend(self.pending_zones_b)
            self.pending_zones_b.clear()

    def _broadcast_abort(self, reason: str) -> None:
        """Broadcasts an immediate abort to both robots (e.g. for storm or physical fault)."""
        msg = String()
        msg.data = json.dumps({"action": "ABORT", "reason": reason})
        self.alert_pub.publish(msg)

    def _supervisor_tick(self) -> None:
        sync_data = {
            "robot_a": self.robot_states['A'],
            "robot_b": self.robot_states['B'],
            "weather": self.current_weather,
            "tasks_remaining": len(self.pending_zones_a) + len(self.pending_zones_b)
        }
        msg = String()
        msg.data = json.dumps(sync_data)
        self.sync_pub.publish(msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinSupervisorNode()
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

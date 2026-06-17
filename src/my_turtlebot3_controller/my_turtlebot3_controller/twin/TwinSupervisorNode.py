#!/usr/bin/env python3
"""
Twin Supervisor Node — Central Orchestrator for the Digital Twin.

Responsibilities:
  - Central Task Dispatch: Maintains the global zone patrol queue.
  - State Sync: Monitors the robot's battery and navigation status.
  - Environmental Interaction: Listens to /weather_forecast and propagates weather
    events (e.g. storms) to halt operations.
  - Fault Management: If the robot is low on battery or crashes, operations pause.
"""
import json
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from my_turtlebot3_controller.qos import STATE_QOS



class TwinSupervisorNode(Node):
    def __init__(self) -> None:
        super().__init__('twin_supervisor_node')

        self.declare_parameter('system_mode', 'HYBRID')
        self.system_mode = self.get_parameter('system_mode').get_parameter_value().string_value

        self.declare_parameter('robot_id', 'A')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        self.pending_zones: List[str] = ['zone_2', 'zone_1', 'zone_3', 'zone_0']

        self.robot_state = {'status': 'IDLE', 'battery': 100.0, 'zone': 'Unknown', 'faulted': False}

        self.current_weather = "sunny"

        # ── Publishers ──
        self.assignment_pub = self.create_publisher(String, '/supervisor/zone_assignment', STATE_QOS)
        self.sync_pub = self.create_publisher(String, '/sync_status', STATE_QOS)
        self.alert_pub = self.create_publisher(String, '/system_alerts', STATE_QOS)

        # ── Subscribers ──
        self.create_subscription(String, '/weather_forecast', self._weather_cb, STATE_QOS)
        self.create_subscription(String, '/robot_resources', self._resource_cb, STATE_QOS)
        self.create_subscription(String, '/navigation_executor_status', self._nav_status_cb, STATE_QOS)
        self.create_subscription(String, '/supervisor/zone_request', self._zone_request_cb, 10)

        self.get_logger().info("Twin Supervisor Central Orchestrator Initialized.")

        self.create_timer(1.0, self._supervisor_tick)

    def _weather_cb(self, msg: String) -> None:
        new_weather = msg.data.lower()
        if new_weather != self.current_weather:
            self.current_weather = new_weather
            self.get_logger().warn(f"[ENV INTERACTION] Weather changed to: {self.current_weather}")
            if self.current_weather == "storm":
                self._broadcast_abort("STORM_EMERGENCY")

    def _resource_cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.robot_state['battery'] = data.get('battery', 100.0)
            if data.get('battery', 100.0) < 15.0:
                self.get_logger().error("Robot Battery Critical! Pausing operations.")
                self._handle_robot_fault()
        except json.JSONDecodeError:
            pass

    def _nav_status_cb(self, msg: String) -> None:
        state = msg.data

        if state in ('FAILED_NAVIGATION', 'ABORTED_NAVIGATION') or 'fault' in state.lower():
            self.get_logger().error("[STATE SYNC] Robot reported fault/failure! Pausing Digital Twin.")
            self._handle_robot_fault()

        self.robot_state['status'] = state

    def _zone_request_cb(self, msg: String) -> None:
        """Handles requests from the robot for its next task."""
        self.get_logger().info("Received zone request from robot")

        if self.robot_state.get('faulted', False):
            if self.robot_state['battery'] >= 20.0 and self.robot_state['status'] == 'IDLE':
                self.get_logger().info("Robot has recovered from its fault. Resetting fault state!")
                self.robot_state['faulted'] = False
            else:
                self.get_logger().warn("Robot is currently FAULTED. Ignoring zone request.")
                return

        if self.current_weather == "storm":
            self._dispatch_zone("BASE")
            return

        if self.robot_state['battery'] < 20.0:
            self._dispatch_zone("BASE")
            return

        if self.pending_zones:
            next_zone = self.pending_zones.pop(0)
            self.pending_zones.append(next_zone)  # Requeue for continuous patrol
            self._dispatch_zone(next_zone)
            return

        self._dispatch_zone("BASE")

    def _dispatch_zone(self, zone: str) -> None:
        assign_msg = String()
        assign_msg.data = json.dumps({"robot": self.robot_id, "zone": zone})
        self.assignment_pub.publish(assign_msg)
        self.get_logger().info(f"Dispatched {zone} to robot")

    def _handle_robot_fault(self) -> None:
        """If the robot fails, pause the patrol and broadcast a global abort."""
        self.get_logger().warn("Handling robot fault...")
        self.robot_state['faulted'] = True
        self._broadcast_abort("PHYSICAL_ROBOT_FAULT")

    def _broadcast_abort(self, reason: str) -> None:
        """Broadcasts an immediate abort to the robot (e.g. for storm or physical fault)."""
        msg = String()
        msg.data = json.dumps({"action": "ABORT", "reason": reason})
        self.alert_pub.publish(msg)

    def _supervisor_tick(self) -> None:
        sync_data = {
            "robot": self.robot_state,
            "weather": self.current_weather,
            "tasks_remaining": len(self.pending_zones)
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

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

from std_msgs.msg import String
from my_turtlebot3_controller.qos import STATE_QOS


class TwinSupervisorNode(Node):
    def __init__(self) -> None:
        """Set up the patrol queue, fault de-bounce state, and the publishers/
        subscribers that link the supervisor to the robot and the dashboard."""
        super().__init__('twin_supervisor_node')

        self.declare_parameter('system_mode', 'HYBRID')
        self.system_mode = self.get_parameter('system_mode').get_parameter_value().string_value

        self.declare_parameter('robot_id', 'A')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # A single Nav2 abort is a normal, locally-recoverable event: Nav2 runs its own
        # recovery behaviours and CropDecisionNode retries via cooldown. Only treat the
        # robot as physically faulted after this many *consecutive* navigation failures
        # with no successful arrival in between. This stops a transient abort from
        # triggering a global "return to base" abort storm.
        self.declare_parameter('nav_fault_threshold', 3)
        self._nav_fault_threshold = self.get_parameter(
            'nav_fault_threshold').get_parameter_value().integer_value

        self.pending_zones: List[str] = ['zone_2', 'zone_1', 'zone_3', 'zone_0']

        self.robot_state = {'status': 'IDLE', 'battery': 100.0, 'zone': 'Unknown', 'faulted': False}

        # Navigation-fault de-bounce state.
        self._last_nav_state = 'IDLE'
        self._consecutive_nav_failures = 0

        self.current_weather = 'sunny'

        # ── Publishers ──
        self.assignment_pub = self.create_publisher(String, '/supervisor/zone_assignment', STATE_QOS)
        self.sync_pub = self.create_publisher(String, '/sync_status', STATE_QOS)
        self.alert_pub = self.create_publisher(String, '/system_alerts', STATE_QOS)

        # ── Subscribers ──
        self.create_subscription(String, '/weather_forecast', self._weather_cb, STATE_QOS)
        self.create_subscription(String, '/robot_resources', self._resource_cb, STATE_QOS)
        self.create_subscription(String, '/navigation_executor_status', self._nav_status_cb, STATE_QOS)
        self.create_subscription(String, '/supervisor/zone_request', self._zone_request_cb, 10)

        self.get_logger().info('Twin Supervisor Central Orchestrator Initialized.')

        self.create_timer(1.0, self._supervisor_tick)

    def _weather_cb(self, msg: String) -> None:
        """React to a weather change from the dashboard; a 'storm' triggers an
        immediate global abort so the robot returns to base (env interaction)."""
        new_weather = msg.data.lower()
        if new_weather != self.current_weather:
            self.current_weather = new_weather
            self.get_logger().warn(f'[ENV INTERACTION] Weather changed to: {self.current_weather}')
            if self.current_weather == 'storm':
                self._broadcast_abort('STORM_EMERGENCY')

    def _resource_cb(self, msg: String) -> None:
        """Mirror the robot's battery level; below 15% pause the patrol (state
        sync: an internal robot state changing the supervisor's behaviour)."""
        try:
            data = json.loads(msg.data)
            self.robot_state['battery'] = data.get('battery', 100.0)
            if data.get('battery', 100.0) < 15.0:
                self.get_logger().error('Robot Battery Critical! Pausing operations.')
                self._handle_robot_fault()
        except json.JSONDecodeError:
            pass

    def _nav_status_cb(self, msg: String) -> None:
        """
        Track navigation status and de-bounce physical-robot faults.

        Reacts only to an actual change of state (the topic is transient-local
        and republished). A physical-robot fault is declared only after
        `nav_fault_threshold` consecutive failures with no arrival between them,
        so a single transient Nav2 abort cannot trigger an abort storm; a
        genuine arrival clears the streak.

        @param msg: std_msgs/String navigation status from the robot.
        @pre  (none).
        @post Updates the failure streak and robot_state['status']; may call
              _handle_robot_fault() when the threshold is reached.
        @return None.
        """
        state = msg.data

        # Edge-trigger: NavigationExecutorNode republishes its status at 2 Hz and the
        # topic is transient-local, so the same value is delivered repeatedly. React
        # only to an actual *change* of state, otherwise one event is counted many times.
        if state == self._last_nav_state:
            self.robot_state['status'] = state
            return
        self._last_nav_state = state

        is_failure = (state in ('FAILED_NAVIGATION', 'ABORTED_NAVIGATION')
                      or 'fault' in state.lower())

        if is_failure:
            self._consecutive_nav_failures += 1
            self.get_logger().warn(
                f"[STATE SYNC] Navigation reported '{state}' "
                f'({self._consecutive_nav_failures}/{self._nav_fault_threshold} consecutive). '
                'Letting the robot recover locally.')
            if self._consecutive_nav_failures >= self._nav_fault_threshold:
                self.get_logger().error(
                    '[STATE SYNC] Navigation failed repeatedly with no arrival — '
                    'declaring physical robot fault and pausing the Digital Twin.')
                self._handle_robot_fault()
                self._consecutive_nav_failures = 0
        elif state == 'SUCCEEDED_AT_POSE':
            # A genuine arrival proves the robot is healthy; clear the failure streak.
            if self._consecutive_nav_failures:
                self.get_logger().info(
                    '[STATE SYNC] Navigation succeeded — clearing nav-failure streak.')
            self._consecutive_nav_failures = 0
        # Other states (IDLE, NAVIGATING, ...) are neutral and leave the streak intact.

        self.robot_state['status'] = state

    def _zone_request_cb(self, msg: String) -> None:
        """
        Dispatch the robot's next task in response to a zone request.

        A faulted robot is ignored until it has recovered (battery >= 20% and
        IDLE). A storm or low battery sends it to BASE; otherwise the next zone
        is popped from the patrol queue and re-queued for continuous patrol.

        @param msg: std_msgs/String zone request from the robot.
        @pre  (none).
        @post Publishes one zone assignment (a zone id or BASE), or returns
              without dispatching if the robot is still faulted.
        @return None.
        """
        self.get_logger().info('Received zone request from robot')

        if self.robot_state.get('faulted', False):
            if self.robot_state['battery'] >= 20.0 and self.robot_state['status'] == 'IDLE':
                self.get_logger().info('Robot has recovered from its fault. Resetting fault state!')
                self.robot_state['faulted'] = False
            else:
                self.get_logger().warn('Robot is currently FAULTED. Ignoring zone request.')
                return

        if self.current_weather == 'storm':
            self._dispatch_zone('BASE')
            return

        if self.robot_state['battery'] < 20.0:
            self._dispatch_zone('BASE')
            return

        if self.pending_zones:
            next_zone = self.pending_zones.pop(0)
            self.pending_zones.append(next_zone)  # Requeue for continuous patrol
            self._dispatch_zone(next_zone)
            return

        self._dispatch_zone('BASE')

    def _dispatch_zone(self, zone: str) -> None:
        """Publish a {robot, zone} assignment on /supervisor/zone_assignment."""
        assign_msg = String()
        assign_msg.data = json.dumps({'robot': self.robot_id, 'zone': zone})
        self.assignment_pub.publish(assign_msg)
        self.get_logger().info(f'Dispatched {zone} to robot')

    def _handle_robot_fault(self) -> None:
        """If the robot fails, pause the patrol and broadcast a global abort."""
        self.get_logger().warn('Handling robot fault...')
        self.robot_state['faulted'] = True
        self._broadcast_abort('PHYSICAL_ROBOT_FAULT')

    def _broadcast_abort(self, reason: str) -> None:
        """Broadcasts an immediate abort to the robot (e.g. for storm or physical fault)."""
        msg = String()
        msg.data = json.dumps({'action': 'ABORT', 'reason': reason})
        self.alert_pub.publish(msg)

    def _supervisor_tick(self) -> None:
        """1 Hz heartbeat: publish the fused robot/weather/queue snapshot on
        /sync_status so the rest of the twin shares one consistent world view."""
        sync_data = {
            'robot': self.robot_state,
            'weather': self.current_weather,
            'tasks_remaining': len(self.pending_zones)
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

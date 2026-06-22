#!/usr/bin/env python3
import json
import math
import os

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from my_turtlebot3_controller.qos import STATE_QOS
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import yaml


class CropDecisionNode(Node):
    """
    Per-robot decision brain for one field robot.

    Asks the supervisor for a zone, drives Nav2 to it, reads the field
    telemetry, and issues irrigate / fertilise / SDG-14-halt actions. All work
    is governed by a phase state machine: IDLE -> WAITING_FOR_ASSIGNMENT ->
    NAVIGATING -> VERIFYING_ZONE -> SCANNING -> DECIDING -> ACTUATING ->
    COOLDOWN, with RETURNING_TO_BASE for low resources.

    Invariant: at most one of the scan / actuation / cooldown timers is active
    at any time; each is cancelled before the next phase's timer is created.
    """

    def __init__(self) -> None:
        """
        Wire up the node's ROS interfaces and decision parameters.

        Wires up the publishers/subscribers, loads zones.yaml, reads the tunable
        thresholds, and starts the 1 Hz state-machine tick.
        """
        super().__init__('crop_decision_node')

        # Identify robot (A or B)
        self.declare_parameter('robot_id', 'A')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # Publishers (using relative topics to respect namespace)
        self.goal_pub = self.create_publisher(PoseStamped, 'dispatch_nav_goal', 10)
        self.irrigate_pub = self.create_publisher(String, 'irrigate_zone', STATE_QOS)
        self.fertilise_pub = self.create_publisher(String, 'fertilise_zone', STATE_QOS)
        self.treatment_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.refill_pub = self.create_publisher(String, 'refill_resources', STATE_QOS)
        self.intervention_pub = self.create_publisher(String, 'sdg14_intervention', STATE_QOS)

        # Supervisor Comms
        self.zone_request_pub = self.create_publisher(String, 'supervisor/zone_request', 10)

        # Subscribers (robot-specific)
        self.nav_status_sub = self.create_subscription(
            String, 'navigation_executor_status', self.nav_status_callback, 10)
        self.current_zone_sub = self.create_subscription(
            String, 'current_zone', self.current_zone_callback, 10)

        # Shared/Global Subscribers
        self.moisture_sub = self.create_subscription(
            Float32MultiArray, '/field_moisture', self.moisture_callback, 10)
        self.nutrients_sub = self.create_subscription(
            Float32MultiArray, '/field_nutrients', self.nutrients_callback, 10)
        self.vulnerability_sub = self.create_subscription(
            Float32MultiArray, '/field_vulnerability', self.vulnerability_callback, 10)
        self.assignment_sub = self.create_subscription(
            String, '/supervisor/zone_assignment', self.assignment_callback, STATE_QOS)
        self.alerts_sub = self.create_subscription(
            String, '/system_alerts', self.alerts_callback, STATE_QOS)
        self.resource_sub = self.create_subscription(
            String, 'robot_resources', self.resource_callback, STATE_QOS)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Load Zones
        pkg_dir = get_package_share_directory('my_turtlebot3_controller')
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        with open(zones_file, 'r', encoding='utf-8') as f:
            self.raw_zones = yaml.safe_load(f) or {}

        self.ordered_zones = sorted([z for z in self.raw_zones.keys() if z != 'base_station'])
        self.latest_moisture = {}
        self.latest_nutrients = {}
        self.latest_vulnerability = {}

        # System state machine
        self.current_phase = 'IDLE'
        self.physical_current_zone = 'no_zone'
        self.active_zone_id = None
        self.sub_targets = []

        self.battery_level = 100.0
        self.fertilizer_level = 100.0
        self.nav2_ready = False
        self.current_x: float = 0.0
        self.current_y: float = 0.0

        # Externalised thresholds (configurable via nexus_params.yaml)
        self.declare_parameter('moisture_threshold', 40.0)
        self.declare_parameter('nutrient_threshold', 50.0)
        self.declare_parameter('vulnerability_halt_threshold', 70.0)
        self.declare_parameter('actuation_duration_sec', 2.0)
        self.declare_parameter('scan_duration_sec', 2.0)
        self.declare_parameter('cooldown_duration_sec', 1.0)
        self.declare_parameter('low_battery_threshold', 15.0)
        self.declare_parameter('low_fertilizer_threshold', 10.0)
        self.declare_parameter('battery_drain_per_meter', 2.0)
        self.declare_parameter('battery_safety_margin', 1.5)

        self.moisture_threshold: float = self.get_parameter(
            'moisture_threshold').get_parameter_value().double_value
        self.nutrient_threshold: float = self.get_parameter(
            'nutrient_threshold').get_parameter_value().double_value
        self.vulnerability_halt: float = self.get_parameter(
            'vulnerability_halt_threshold').get_parameter_value().double_value
        self.actuation_duration: float = self.get_parameter(
            'actuation_duration_sec').get_parameter_value().double_value
        self.scan_duration: float = self.get_parameter(
            'scan_duration_sec').get_parameter_value().double_value
        self.cooldown_duration: float = self.get_parameter(
            'cooldown_duration_sec').get_parameter_value().double_value
        self.low_battery_threshold: float = self.get_parameter(
            'low_battery_threshold').get_parameter_value().double_value
        self.low_fertilizer_threshold: float = self.get_parameter(
            'low_fertilizer_threshold').get_parameter_value().double_value
        self.battery_drain_per_meter: float = self.get_parameter(
            'battery_drain_per_meter').get_parameter_value().double_value
        self.battery_safety_margin: float = self.get_parameter(
            'battery_safety_margin').get_parameter_value().double_value

        self._scan_timer = None
        self._actuation_timer = None
        self._cooldown_timer = None
        self._actuation_start_time = 0.0
        self._zone_request_time = 0.0

        self.create_timer(1.0, self.state_machine_tick)
        self.get_logger().info(
            f'CropDecisionNode for Robot {self.robot_id} started. Operating as twin client.')

    def _cancel_and_destroy(self, timer_attr: str) -> None:
        """
        Cancel and tear down the one-shot timer named by `timer_attr`.

        Clears the attribute afterwards so phases never leave a stale timer
        running.
        """
        timer = getattr(self, timer_attr, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            setattr(self, timer_attr, None)

    # ── Callbacks ──
    def current_zone_callback(self, msg: String) -> None:
        """Record which zone the robot is physically inside (from ZoneDetector)."""
        self.physical_current_zone = msg.data

    def moisture_callback(self, msg: Float32MultiArray) -> None:
        """Cache per-zone moisture (array indexed by self.ordered_zones order)."""
        for i, val in enumerate(msg.data):
            if i < len(self.ordered_zones):
                self.latest_moisture[self.ordered_zones[i]] = val

    def nutrients_callback(self, msg: Float32MultiArray) -> None:
        """Cache per-zone nutrient levels (same indexing as moisture)."""
        for i, val in enumerate(msg.data):
            if i < len(self.ordered_zones):
                self.latest_nutrients[self.ordered_zones[i]] = val

    def vulnerability_callback(self, msg: Float32MultiArray) -> None:
        """Cache per-zone runoff vulnerability (drives the SDG-14 halt)."""
        for i, val in enumerate(msg.data):
            if i < len(self.ordered_zones):
                self.latest_vulnerability[self.ordered_zones[i]] = val

    def odom_callback(self, msg: Odometry) -> None:
        """Track current robot position for battery estimation."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def resource_callback(self, msg: String) -> None:
        """Update local battery/fertilizer levels from RobotResourceNode."""
        try:
            data = json.loads(msg.data)
            self.battery_level = data.get('battery', self.battery_level)
            self.fertilizer_level = data.get('fertilizer', self.fertilizer_level)
        except json.JSONDecodeError:
            pass

    def alerts_callback(self, msg: String) -> None:
        """
        Honour a supervisor ABORT (storm / fault).

        Drops the queue and heads home — an env/fault event propagating from the
        digital side to the robot.
        """
        try:
            data = json.loads(msg.data)
            if isinstance(data, dict) and data.get('action') == 'ABORT':
                self.get_logger().error(
                    f"SUPERVISOR ABORT RECEIVED: {data.get('reason')}. Returning to BASE.")
                self.sub_targets.clear()
                self._dispatch_to_base()
        except json.JSONDecodeError:
            pass

    def assignment_callback(self, msg: String) -> None:
        """
        Accept a zone assignment addressed to this robot.

        Ignored unless the phase is WAITING_FOR_ASSIGNMENT. A "BASE" zone or a
        failed predictive battery check diverts the robot home; otherwise the
        zone's target becomes the next navigation sub-target.

        @param msg: std_msgs/String holding JSON {"robot": id, "zone": zone_id}.
        @pre  self.raw_zones contains zone_id when it is not "BASE".
        @post On a match, either RETURNING_TO_BASE is entered or active_zone_id
              and sub_targets are set and the first sub-target is dispatched.
        @return None.
        @throws (none) malformed JSON is caught and ignored.
        """
        if self.current_phase != 'WAITING_FOR_ASSIGNMENT':
            return

        try:
            data = json.loads(msg.data)
            if data.get('robot') == self.robot_id:
                zone_id = data.get('zone')
                if zone_id == 'BASE':
                    self.get_logger().info('Supervisor requested return to base.')
                    self._dispatch_to_base()
                else:
                    # ── Predictive battery check ──────────────────────────
                    z = self.raw_zones[zone_id]
                    base = self.raw_zones.get('base_station', {})
                    if not self._can_afford_trip(z, base):
                        self.get_logger().warn(
                            f'Rejecting {zone_id}: insufficient battery for round-trip. '
                            f'Returning to base to recharge.')
                        self._dispatch_to_base()
                        return

                    self.get_logger().info(
                        f'Supervisor assigned {zone_id}. Generating sub-targets...')
                    self.active_zone_id = zone_id
                    # Generate 1 sub-target for the zone center
                    self.sub_targets = [
                        (z['target_x'], z['target_y'], z['target_theta'])
                    ]
                    self._dispatch_next_subtarget()
        except json.JSONDecodeError:
            pass

    def nav_status_callback(self, msg: String) -> None:
        """
        Advance the state machine on a navigation outcome.

        Arrival while NAVIGATING moves to VERIFYING_ZONE; a navigation failure
        starts a cooldown and skips the sub-target; arrival while
        RETURNING_TO_BASE publishes a refill and returns to IDLE. The first IDLE
        seen latches nav2_ready.

        @param msg: std_msgs/String navigation status from NavigationExecutor.
        @pre  (none).
        @post self.current_phase and self.nav2_ready reflect the new status.
        @return None.
        """
        status = msg.data
        if status == 'IDLE' and not self.nav2_ready:
            self.nav2_ready = True

        if self.current_phase == 'NAVIGATING':
            if status == 'SUCCEEDED_AT_POSE':
                self.current_phase = 'VERIFYING_ZONE'
            elif status in ['FAILED_NAVIGATION', 'ABORTED_NAVIGATION',
                            'CANCELED_NAVIGATION', 'REJECTED',
                            'IDLE_SERVER_UNAVAILABLE']:
                self.get_logger().warn(
                    f'Navigation failed with status {status}. Skipping sub-target.')
                self._start_cooldown()

        elif self.current_phase == 'RETURNING_TO_BASE':
            if status == 'SUCCEEDED_AT_POSE':
                self.get_logger().info('Arrived at Base Station. Refilling...')
                msg_refill = String()
                msg_refill.data = 'refill'
                self.refill_pub.publish(msg_refill)
                self.current_phase = 'IDLE'

    # ── Battery estimation ──
    def _can_afford_trip(self, zone: dict, base: dict) -> bool:
        """
        Estimate whether the battery can cover the round-trip to a zone.

        Distance is current_position -> zone target -> base station, costed as
        Euclidean distance x battery_drain_per_meter x safety_margin. The margin
        (default 1.5x) accounts for Nav2 paths being longer than straight lines.

        @param zone: zone definition with numeric 'target_x' / 'target_y'.
        @param base: base-station definition (defaults to the origin if absent).
        @pre  self.current_x / self.current_y hold the latest odometry position.
        @post No state is mutated (read-only estimate).
        @return True if self.battery_level exceeds the estimated drain.
        """
        zx, zy = float(zone['target_x']), float(zone['target_y'])
        bx, by = float(base.get('target_x', 0.0)), float(base.get('target_y', 0.0))

        dist_to_zone = math.sqrt(
            (self.current_x - zx) ** 2 + (self.current_y - zy) ** 2)
        dist_zone_to_base = math.sqrt(
            (zx - bx) ** 2 + (zy - by) ** 2)

        total_dist = dist_to_zone + dist_zone_to_base
        estimated_drain = total_dist * self.battery_drain_per_meter * self.battery_safety_margin

        self.get_logger().info(
            f'Battery check: {self.battery_level:.1f}% available, '
            f'trip ≈ {total_dist:.2f}m → estimated drain ≈ {estimated_drain:.1f}%')

        return self.battery_level > estimated_drain

    # ── Dispatchers ──
    def _dispatch_next_subtarget(self):
        """
        Pop the next (x, y, yaw) sub-target and send it to Nav2.

        The goal is sent in the map frame; if the list is empty the zone is done
        and the node returns to IDLE.
        """
        if not self.sub_targets:
            self.get_logger().info(f'Finished all sub-targets for {self.active_zone_id}.')
            self.current_phase = 'IDLE'
            return

        x, y, yaw_deg = self.sub_targets.pop(0)
        self.get_logger().info(f'Navigating to sub-target at ({x:.2f}, {y:.2f})')

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        yaw_rad = math.radians(float(yaw_deg))
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.goal_pub.publish(goal)
        self.current_phase = 'NAVIGATING'

    def _dispatch_to_base(self):
        """
        Send the robot to the base station to recharge/refill.

        Applies a per-robot parking offset and enters RETURNING_TO_BASE so
        arrival triggers a refill.
        """
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
        self.current_phase = 'RETURNING_TO_BASE'

    # ── State Machine ──
    def state_machine_tick(self) -> None:
        """
        Drive the phase state machine once per second.

        Enforces the low-resource return-to-base guard, then steps the phase
        machine: requests a zone when IDLE, times out a stuck assignment, and
        confirms arrival before scanning.
        """
        if not self.nav2_ready:
            return

        # ── Resource check: abort mission and return to base if low ────
        if self.current_phase not in ('RETURNING_TO_BASE', 'IDLE'):
            needs_refill = False
            reason = ''

            if self.battery_level <= self.low_battery_threshold:
                needs_refill = True
                reason = (
                    f'Battery critically low '
                    f'({self.battery_level:.1f}% ≤ {self.low_battery_threshold:.0f}%)')
            elif self.fertilizer_level <= self.low_fertilizer_threshold:
                needs_refill = True
                reason = (
                    f'Fertilizer critically low '
                    f'({self.fertilizer_level:.1f}% ≤ {self.low_fertilizer_threshold:.0f}%)')

            if needs_refill:
                self.get_logger().warn(
                    f'LOW RESOURCES: {reason}. Aborting mission, returning to base.')
                # Cancel any in-progress timers
                self._cancel_and_destroy('_scan_timer')
                self._cancel_and_destroy('_actuation_timer')
                self._cancel_and_destroy('_cooldown_timer')
                # Stop treatment actuation if active
                stop_cmd = Twist()
                self.treatment_vel_pub.publish(stop_cmd)
                # Clear remaining subtargets and head home
                self.sub_targets.clear()
                self._dispatch_to_base()
                return

        if self.current_phase == 'IDLE':
            self.get_logger().info('Requesting zone from Supervisor...')
            req = String()
            req.data = json.dumps({'robot': self.robot_id})
            self.zone_request_pub.publish(req)
            self._zone_request_time = self.get_clock().now().nanoseconds / 1e9
            self.current_phase = 'WAITING_FOR_ASSIGNMENT'

        elif self.current_phase == 'WAITING_FOR_ASSIGNMENT':
            elapsed = self.get_clock().now().nanoseconds / 1e9 - self._zone_request_time
            if elapsed > 5.0:
                self.get_logger().warn('No zone assignment received after 5s, retrying...')
                self.current_phase = 'IDLE'

        elif self.current_phase == 'VERIFYING_ZONE':
            if self.physical_current_zone == self.active_zone_id:
                self.current_phase = 'SCANNING'
                self._cancel_and_destroy('_scan_timer')
                self._scan_timer = self.create_timer(self.scan_duration, self._on_scan_complete)
            else:
                self.get_logger().warn(
                    f'Not in {self.active_zone_id} yet. Moving to next sub-target.')
                self._start_cooldown()

    def _on_scan_complete(self):
        """
        Decide and apply the treatment for the active zone after a scan.

        High runoff vulnerability halts fertilising and logs an SDG-14
        intervention; otherwise a low nutrient level triggers fertilising and a
        low moisture level triggers irrigation. Treatment in the base station is
        forbidden. Actuation runs only when an action was taken, else cooldown.

        @pre  self.active_zone_id names the zone just scanned; the latest field
              telemetry for it is available (defaults assumed healthy if not).
        @post Publishes the relevant treatment / intervention message and sets
              the phase to ACTUATING or COOLDOWN.
        @return None.
        """
        self._cancel_and_destroy('_scan_timer')
        self.current_phase = 'DECIDING'

        # Read from field sensors for the active zone
        moist = self.latest_moisture.get(self.active_zone_id, 100.0)
        nutri = self.latest_nutrients.get(self.active_zone_id, 100.0)
        vuln = self.latest_vulnerability.get(self.active_zone_id, 0.0)

        # Hard constraint: Never actuate in the base station
        if self.active_zone_id == 'base_station' or self.physical_current_zone == 'base_station':
            self.get_logger().info(
                'Zone is base station. Treatment is strictly forbidden here. Returning to IDLE.')
            self._start_cooldown()
            return

        self.get_logger().info(
            f'[{self.active_zone_id}] Stats - Moisture: {moist:.1f}%, '
            f'Nutrients: {nutri:.1f}%, Vulnerability: {vuln:.1f}%')

        action_taken = False

        if vuln > self.vulnerability_halt:
            self.get_logger().warn(
                f'[{self.active_zone_id}] High vulnerability '
                f'({vuln:.1f}% > {self.vulnerability_halt:.0f}%). '
                f'Halting fertilization to prevent runoff! (SDG-14)')
            intervention_msg = String()
            intervention_msg.data = json.dumps({
                'robot': self.robot_id,
                'zone': self.active_zone_id,
                'action': 'HALT_FERTILIZER',
                'reason': 'High runoff risk'
            })
            self.intervention_pub.publish(intervention_msg)
        elif nutri < self.nutrient_threshold:
            self.get_logger().info(
                f'[{self.active_zone_id}] Nutrients low '
                f'({nutri:.1f}% < {self.nutrient_threshold}%). Fertilizing.')
            msg = String()
            msg.data = json.dumps({'robot': self.robot_id, 'zone': self.active_zone_id})
            self.fertilise_pub.publish(msg)
            action_taken = True

        if moist < self.moisture_threshold:
            self.get_logger().info(
                f'[{self.active_zone_id}] Moisture low '
                f'({moist:.1f}% < {self.moisture_threshold}%). Irrigating.')
            irr_msg = String()
            irr_msg.data = json.dumps({'robot': self.robot_id, 'zone': self.active_zone_id})
            self.irrigate_pub.publish(irr_msg)
            action_taken = True

        if not action_taken and vuln <= self.vulnerability_halt:
            self.get_logger().info(
                f'[{self.active_zone_id}] Zone is healthy. No treatment needed.')

        if action_taken:
            self.current_phase = 'ACTUATING'
            self._actuation_start_time = self.get_clock().now().nanoseconds / 1e9
            self._cancel_and_destroy('_actuation_timer')
            self._actuation_timer = self.create_timer(0.1, self._actuation_tick)
        else:
            self._start_cooldown()

    def _actuation_tick(self):
        """
        Spin in place to mime spraying, then stop and start the cooldown.

        Runs for `actuation_duration` — the visible 'treatment' behaviour — then
        stops the robot and begins the cooldown.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._actuation_start_time < self.actuation_duration:
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
        """
        Enter a short COOLDOWN before the next sub-target.

        Prevents consecutive Nav2 goals from stomping on each other.
        """
        self.current_phase = 'COOLDOWN'
        self._cancel_and_destroy('_cooldown_timer')
        self._cooldown_timer = self.create_timer(
            self.cooldown_duration, self._on_cooldown_complete)

    def _on_cooldown_complete(self):
        """End of cooldown: go to the next sub-target, or back to IDLE if none."""
        self._cancel_and_destroy('_cooldown_timer')
        # If there are more subtargets, go to them!
        if self.sub_targets:
            self._dispatch_next_subtarget()
        else:
            self.current_phase = 'IDLE'


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

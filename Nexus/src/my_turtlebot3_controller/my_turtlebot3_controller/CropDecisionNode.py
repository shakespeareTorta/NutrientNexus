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


class CropDecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_decision_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.irrigate_pub = self.create_publisher(String, '/irrigate_zone', 10)
        self.fertilise_pub = self.create_publisher(String, '/fertilise_zone', 10)

        # Treatment actuation publishes directly to /cmd_vel so the physical robot spins.
        # (Nav2 stops publishing to /cmd_vel when it finishes navigating, so this is safe).
        self.treatment_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.refill_pub = self.create_publisher(String, '/refill_resources', 10)
        self.intervention_pub = self.create_publisher(String, '/sdg14_intervention', 10)
        self.override_req_pub = self.create_publisher(String, '/operator_override_request', 10)

        # Subscribers
        self.moisture_sub = self.create_subscription(
            Float32MultiArray, '/field_moisture', self.moisture_callback, 10)
        self.nutrients_sub = self.create_subscription(
            Float32MultiArray, '/field_nutrients', self.nutrients_callback, 10)
        self.growth_sub = self.create_subscription(
            Float32MultiArray, '/field_growth', self.growth_callback, 10)
        self.nav_status_sub = self.create_subscription(
            String, '/navigation_executor_status', self.nav_status_callback, 10)

        # Weather subscriber / simulation (e.g., Sunny, Rainy, Overcast)
        # We can also read this from a parameter, default is "sunny"
        self.declare_parameter('weather_forecast', 'sunny')
        self.weather = self.get_parameter(
            'weather_forecast').get_parameter_value().string_value

        self.weather_sub = self.create_subscription(
            String, '/weather_forecast', self.weather_callback, 10)
            
        self.resource_sub = self.create_subscription(
            String, '/robot_resources', self.resource_callback, 10)
        
        self.override_resp_sub = self.create_subscription(
            String, '/operator_override_response', self.override_resp_callback, 10)

        # Crop Zones Configuration - Loaded from zones.yaml
        pkg_dir = get_package_share_directory('my_turtlebot3_controller')
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        with open(zones_file, 'r', encoding='utf-8') as f:
            raw_zones = yaml.safe_load(f) or {}
            
        self.base_station: Optional[Dict[str, Any]] = None
        self.zones_data: List[Dict[str, Any]] = []
        for zone_id in sorted(raw_zones.keys()):
            z = raw_zones[zone_id]
            zone_info = {
                'id': zone_id,
                'location': (z['target_x'], z['target_y'], z['target_theta']),
                'runoff_risk': z['runoff_risk'],
                'moisture': 100.0,
                'nutrients': 100.0,
                'growth': 0.0
            }
            if zone_id == 'base_station':
                self.base_station = zone_info
            else:
                self.zones_data.append(zone_info)
            
        # Spatial Verification Subscriber
        self.physical_current_zone: str = "no_zone"
        self.current_zone_sub = self.create_subscription(
            String, '/current_zone', self.current_zone_callback, 10)

        # System state machine
        # Phases: IDLE, NAVIGATING, VERIFYING_ZONE, SCANNING, DECIDING, ACTUATING, COOLDOWN, RETURNING_TO_BASE
        self.current_phase: str = "IDLE"
        self.current_zone_index: int = 2
        
        # Resource tracking
        self.battery_level: float = 100.0
        self.fertilizer_level: float = 100.0

        # Treatment and telemetry thresholds
        self.moisture_threshold: float = 40.0   # % — below this, needs water
        self.nutrient_threshold: float = 50.0   # % — below this, needs fertilizer

        self.telemetry_received: bool = False
        self.nav2_ready: bool = False

        # Reusable one-shot timers (initialised as None, created on demand,
        # properly destroyed before re-creation to prevent memory leaks)
        self._scan_timer: Optional[Timer] = None
        self._cooldown_timer: Optional[Timer] = None
        self._actuation_timer: Optional[Timer] = None
        self._actuation_start_time: float = 0.0
        self.actuation_duration: float = 4.0
        self.actuation_speed: float = 0.6

        # Main cycle timer
        self.cycle_timer = self.create_timer(3.0, self.state_machine_tick)
        self.get_logger().info(
            f"Nutrient Nexus CropDecisionNode started. "
            f"Weather forecast: {self.weather.upper()}")

    # ------------------------------------------------------------------ #
    #  Timer lifecycle helpers — prevent accumulation / memory leaks       #
    # ------------------------------------------------------------------ #
    def _cancel_and_destroy(self, timer_attr: str) -> None:
        """Cancel and destroy a timer stored on `self.<timer_attr>`, then set it to None."""
        timer = getattr(self, timer_attr, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
            setattr(self, timer_attr, None)

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def weather_callback(self, msg: String) -> None:
        old_weather = self.weather
        self.weather = msg.data.lower()
        if old_weather != self.weather:
            self.get_logger().info(
                f"Weather Forecast updated: {self.weather.upper()}")

    def resource_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self.battery_level = data.get("battery", 100.0)
            self.fertilizer_level = data.get("fertilizer", 100.0)
        except json.JSONDecodeError:
            pass

    def override_resp_callback(self, msg: String) -> None:
        if self.current_phase != "WAITING_FOR_OPERATOR":
            return
            
        action = msg.data.lower()
        zone = self.zones_data[self.current_zone_index]
        
        if action == "override":
            self.get_logger().warn("HUMAN OVERRIDE RECEIVED: Forcing fertilization despite SDG-14 risks.")
            
            # Log this dangerous override
            intervention_data = {
                "zone": zone['id'],
                "reason": "HUMAN OVERRIDE of Critical SDG-14 Risk",
                "vulnerability_score": "N/A"
            }
            msg_int = String()
            msg_int.data = json.dumps(intervention_data)
            self.intervention_pub.publish(msg_int)
            
            # Force actuate
            self.current_phase = "ACTUATING"
            msg_fert = String()
            msg_fert.data = zone['id']
            self.fertilise_pub.publish(msg_fert)
            
            self.get_logger().info("Simulating physical actuation: Rotating in place to apply treatment...")
            self._actuation_start_time = self.get_clock().now().nanoseconds / 1e9
            self._cancel_and_destroy('_actuation_timer')
            self._actuation_timer = self.create_timer(0.1, self._actuation_tick)
            
        elif action == "comply":
            self.get_logger().info("Operator COMPLIED with SDG-14 abort. Skipping zone.")
            self._start_cooldown()

    def current_zone_callback(self, msg: String) -> None:
        self.physical_current_zone = msg.data

    def moisture_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i < len(self.zones_data):
                self.zones_data[i]['moisture'] = val
        self.telemetry_received = True

    def nutrients_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i < len(self.zones_data):
                self.zones_data[i]['nutrients'] = val

    def growth_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i < len(self.zones_data):
                self.zones_data[i]['growth'] = val

    def nav_status_callback(self, msg: String) -> None:
        status = msg.data
        self.get_logger().info(
            f"Nav Status: {status} (Nexus Phase: {self.current_phase})")

        if status == "IDLE" and not self.nav2_ready:
            self.nav2_ready = True
            self.get_logger().info("Nav2 executor ready. Starting crop patrol.")

        if self.current_phase == "NAVIGATING":
            if status == "SUCCEEDED_AT_POSE":
                expected_zone = self.zones_data[self.current_zone_index]['id']
                self.get_logger().info(
                    f"Nav2 reports arrival at {expected_zone}. "
                    f"Verifying physical zone bounding box via /current_zone...")
                self.current_phase = "VERIFYING_ZONE"

            elif status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION",
                            "CANCELED_NAVIGATION", "REJECTED"]:
                self.get_logger().warn(
                    f"Navigation failed to Zone {self.current_zone_index}. "
                    f"Retrying after cooldown.")
                self._start_cooldown()

        elif self.current_phase == "RETURNING_TO_BASE":
            if status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(
                    "Arrived at Base Station. Initiating refill and charge...")
                msg = String()
                msg.data = "refill"
                self.refill_pub.publish(msg)
                self._start_cooldown()
            elif status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION",
                            "CANCELED_NAVIGATION", "REJECTED"]:
                self.get_logger().warn("Navigation failed to Base Station. Retrying...")
                self._start_cooldown()

    # ------------------------------------------------------------------ #
    #  Phase transitions                                                   #
    # ------------------------------------------------------------------ #
    def _on_scan_complete(self) -> None:
        self._cancel_and_destroy('_scan_timer')
        self.get_logger().info(
            f"Scanning complete for Zone {self.current_zone_index}. "
            f"Evaluating context...")
        self.current_phase = "DECIDING"
        self._make_contextual_decision()

    def _start_cooldown(self) -> None:
        self.current_phase = "COOLDOWN"
        self._cancel_and_destroy('_cooldown_timer')
        self._cooldown_timer = self.create_timer(
            3.0, self._on_cooldown_complete)

    def _on_cooldown_complete(self) -> None:
        self._cancel_and_destroy('_cooldown_timer')
        self.current_phase = "IDLE"
        # Advance to next zone to prevent locking on one bad spot
        self.current_zone_index = (
            (self.current_zone_index + 1) % len(self.zones_data))

    # ------------------------------------------------------------------ #
    #  Context-Aware Decision Engine                                       #
    # ------------------------------------------------------------------ #
    def _make_contextual_decision(self) -> None:
        zone = self.zones_data[self.current_zone_index]
        moisture = zone['moisture']
        nutrients = zone['nutrients']
        runoff_risk_base = zone['runoff_risk']

        # 1. Calculate Dynamic Runoff Vulnerability Score (0-100)
        vulnerability_score = 0.0
        
        # Base Risk Contribution (0-40)
        if runoff_risk_base == 'High':
            vulnerability_score += 40.0
        elif runoff_risk_base == 'Medium':
            vulnerability_score += 20.0
            
        # Soil Saturation Contribution (0-30)
        if moisture > 85.0:
            vulnerability_score += 30.0
        elif moisture > 60.0:
            vulnerability_score += 15.0
            
        # Weather Contribution (0-30)
        if self.weather == 'rainy':
            vulnerability_score += 30.0
        elif self.weather == 'overcast':
            vulnerability_score += 10.0

        vulnerability_score = min(100.0, vulnerability_score)

        self.get_logger().info(f"=== Scan Telemetry for {zone['id']} ===")
        self.get_logger().info(
            f" - Soil Moisture: {moisture:.1f}% "
            f"(Threshold: {self.moisture_threshold}%)")
        self.get_logger().info(
            f" - Soil Nutrients: {nutrients:.1f}% "
            f"(Threshold: {self.nutrient_threshold}%)")
        self.get_logger().info(
            f" - Weather Condition: {self.weather.upper()}")
        self.get_logger().info(
            f" - Dynamic Runoff Vulnerability Score: {vulnerability_score:.1f}/100")

        irrigation_recommended: bool = False
        fertilisation_recommended: bool = False
        sustainability_log: List[str] = []

        # 1. Irrigation Rule
        if moisture < self.moisture_threshold:
            if self.weather == 'rainy':
                sustainability_log.append(
                    "Moisture LOW, but Irrigation SKIPPED "
                    "(Naturally irrigated by rain).")
            else:
                irrigation_recommended = True
                sustainability_log.append(
                    "Moisture LOW. Irrigation RECOMMENDED.")
        else:
            sustainability_log.append(
                "Moisture adequate. No irrigation needed.")

        # 2. Fertiliser / Nitrogen Rule
        #    (Context-Aware / Coastal Eutrophication Risk — SDG 14)
        is_safe_to_fertilize = vulnerability_score <= 60.0

        if nutrients < self.nutrient_threshold:
            if not is_safe_to_fertilize:
                reason = f"Fertilisation BLOCKED. Vulnerability Score {vulnerability_score:.1f} > 60.0 indicates high runoff risk."
                sustainability_log.append(f"CRITICAL: {reason}")
                
                # Publish intervention to the audit ledger
                intervention_data = {
                    "zone": zone['id'],
                    "reason": reason,
                    "vulnerability_score": round(vulnerability_score, 1)
                }
                msg_int = String()
                msg_int.data = json.dumps(intervention_data)
                self.intervention_pub.publish(msg_int)
                
                # HALT for Operator Override
                self.get_logger().warn("Halting and waiting for human operator override decision...")
                self.current_phase = "WAITING_FOR_OPERATOR"
                msg_req = String()
                msg_req.data = json.dumps({"zone": zone['id'], "reason": reason})
                self.override_req_pub.publish(msg_req)
                return  # Skip actuation logic for now
            else:
                fertilisation_recommended = True
                sustainability_log.append(
                    "Nutrients LOW. Fertilisation RECOMMENDED.")
        else:
            sustainability_log.append(
                "Nutrients adequate. No fertilisation needed.")

        self.get_logger().info(
            "=== Context-Aware Decision Recommendation ===")
        for log_line in sustainability_log:
            self.get_logger().info(f" * {log_line}")

        # Actuate if either treatment is recommended
        if irrigation_recommended or fertilisation_recommended:
            self.current_phase = "ACTUATING"
            
            # Dynamic Actuation calculation based on Zone Risk and Nutrient Deficit
            deficit = max(0.0, self.nutrient_threshold - nutrients)
            # Duration scales with deficit (e.g. 20 deficit = 4 seconds, 40 deficit = 6 seconds)
            self.actuation_duration = 2.0 + (deficit / 10.0)
            
            # Speed scales with runoff risk (slower for high risk to avoid splash)
            if runoff_risk_base == 'High':
                self.actuation_speed = 0.3  # Slow, careful spin
            elif runoff_risk_base == 'Medium':
                self.actuation_speed = 0.6  # Standard spin
            else:
                self.actuation_speed = 1.0  # Fast spin for Low risk

            self.get_logger().info(
                f"Simulating dynamic actuation for {zone['id']} "
                f"(Risk: {runoff_risk_base}, Deficit: {deficit:.1f}%). "
                f"Spinning at {self.actuation_speed} rad/s for {self.actuation_duration:.1f}s.")

            # Publish treatment commands so the sensor mock can replenish
            if irrigation_recommended:
                msg = String()
                msg.data = zone['id']
                self.irrigate_pub.publish(msg)
            if fertilisation_recommended:
                msg = String()
                msg.data = zone['id']
                self.fertilise_pub.publish(msg)

            # Start the actuation rotation
            self._actuation_start_time = (
                self.get_clock().now().nanoseconds / 1e9)
            self._cancel_and_destroy('_actuation_timer')
            self._actuation_timer = self.create_timer(
                0.1, self._actuation_tick)
        else:
            self.get_logger().info(
                "No physical action required or safe. Moving to next zone.")
            self._start_cooldown()

    # ------------------------------------------------------------------ #
    #  Actuation — in-place rotation proxy                                 #
    # ------------------------------------------------------------------ #
    def _actuation_tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._actuation_start_time

        if elapsed < self.actuation_duration:  # Dynamic spin duration
            cmd = Twist()
            cmd.angular.z = self.actuation_speed  # Dynamic spin speed
            self.treatment_vel_pub.publish(cmd)
        else:
            # Stop rotation
            self._cancel_and_destroy('_actuation_timer')
            cmd = Twist()
            cmd.angular.z = 0.0
            self.treatment_vel_pub.publish(cmd)
            self.get_logger().info(
                f"Actuation complete for Zone {self.current_zone_index}. "
                f"Cooldown initiated.")
            self._start_cooldown()

    # ------------------------------------------------------------------ #
    #  Main state machine tick                                             #
    # ------------------------------------------------------------------ #
    def state_machine_tick(self) -> None:
        if self.current_phase == "VERIFYING_ZONE":
            expected_zone = self.zones_data[self.current_zone_index]['id']
            if self.physical_current_zone == expected_zone:
                self.get_logger().info(
                    f"Verified physically inside {expected_zone}. "
                    f"Commencing sensor scanning...")
                self.current_phase = "SCANNING"
                self._cancel_and_destroy('_scan_timer')
                self._scan_timer = self.create_timer(4.0, self._on_scan_complete)
            else:
                self.get_logger().warn(
                    f"Nav2 arrived, but robot is NOT inside bounding box for {expected_zone}! "
                    f"Current physical zone: '{self.physical_current_zone}'. Aborting treatment for safety.")
                self._start_cooldown()
            return
            
        if self.current_phase != "IDLE":
            return

        if not self.nav2_ready:
            self.get_logger().info(
                "Waiting for Nav2 executor to become ready...",
                throttle_duration_sec=10.0)
            return

        # 1. Resource Pre-flight Check
        if self.battery_level < 30.0 or self.fertilizer_level < 20.0:
            self.get_logger().warn(
                f"Resources LOW! Battery: {self.battery_level}%, "
                f"Fertilizer: {self.fertilizer_level}%. "
                f"Aborting mission, returning to base station.")
            
            if self.base_station:
                x, y, yaw_deg = self.base_station['location']
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x = float(x)
                goal.pose.position.y = float(y)
                yaw_rad = math.radians(float(yaw_deg))
                goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
                goal.pose.orientation.w = math.cos(yaw_rad / 2.0)

                self.goal_pub.publish(goal)
                self.current_phase = "RETURNING_TO_BASE"
            return

        if not self.telemetry_received:
            self.get_logger().info(
                "Waiting on field telemetry data before dispatching "
                "crop scan...")
            return

        # Choose next zone and dispatch
        zone = self.zones_data[self.current_zone_index]
        self.get_logger().info(
            f"Navigating to crop {zone['id']} "
            f"at Location: {zone['location']}")

        x, y, yaw_deg = zone['location']
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


def main(args=None) -> None:
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

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
import math
from typing import Dict, List, Optional


class CropDecisionNode(Node):
    def __init__(self) -> None:
        super().__init__('crop_decision_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.irrigate_pub = self.create_publisher(String, '/irrigate_zone', 10)
        self.fertilise_pub = self.create_publisher(String, '/fertilise_zone', 10)

        # Treatment actuation publishes to a SEPARATE topic to avoid
        # conflicting with Nav2's velocity output on /cmd_vel.
        # A cmd_vel_mux or relay can merge them downstream if needed.
        self.treatment_vel_pub = self.create_publisher(Twist, '/cmd_vel_treatment', 10)

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

        # Crop Zones Configuration
        # Coordinates mapped to the physical simulation layout:
        # Zone 0: High land (Low runoff risk)
        # Zone 1: Sloped field (Medium runoff risk)
        # Zone 2: Near downstream water/wetland (High runoff risk!)
        self.zones_data = {
            0: {'id': 'zone_0', 'location': (0.05, 1.5, 90.0),
                'runoff_risk': 'Low', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0},
            1: {'id': 'zone_1', 'location': (1.5, 1.5, 0.0),
                'runoff_risk': 'Medium', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0},
            2: {'id': 'zone_2', 'location': (1.5, 0.0, -90.0),
                'runoff_risk': 'High', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0},
        }

        # System state machine
        # Phases: IDLE, NAVIGATING, SCANNING, DECIDING, ACTUATING, COOLDOWN
        self.current_phase: str = "IDLE"
        self.current_zone_index: int = 0

        # Treatment and telemetry thresholds
        self.moisture_threshold: float = 40.0   # % — below this, needs water
        self.nutrient_threshold: float = 50.0   # % — below this, needs fertilizer

        # Telemetry storage flag
        self.telemetry_received: bool = False

        # Reusable one-shot timers (initialised as None, created on demand,
        # properly destroyed before re-creation to prevent memory leaks)
        self._scan_timer: Optional[Timer] = None
        self._cooldown_timer: Optional[Timer] = None
        self._actuation_timer: Optional[Timer] = None
        self._actuation_start_time: float = 0.0

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

    def moisture_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['moisture'] = val
        self.telemetry_received = True

    def nutrients_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['nutrients'] = val

    def growth_callback(self, msg: Float32MultiArray) -> None:
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['growth'] = val

    def nav_status_callback(self, msg: String) -> None:
        status = msg.data
        self.get_logger().info(
            f"Nav Status: {status} (Nexus Phase: {self.current_phase})")

        if self.current_phase == "NAVIGATING":
            if status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(
                    f"Arrived at Zone {self.current_zone_index}. "
                    f"Commencing sensor scanning...")
                self.current_phase = "SCANNING"
                # 4-second one-shot timer to simulate scanning
                self._cancel_and_destroy('_scan_timer')
                self._scan_timer = self.create_timer(
                    4.0, self._on_scan_complete)

            elif status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION",
                            "CANCELED_NAVIGATION", "REJECTED"]:
                self.get_logger().warn(
                    f"Navigation failed to Zone {self.current_zone_index}. "
                    f"Retrying after cooldown.")
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
        runoff_risk = zone['runoff_risk']

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
            f" - Downstream Runoff Vulnerability: {runoff_risk}")

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
        if nutrients < self.nutrient_threshold:
            if self.weather == 'rainy':
                sustainability_log.append(
                    "Nutrients LOW, but Fertilisation BLOCKED "
                    "(CRITICAL: Heavy rain will wash nutrients downstream!).")
            elif runoff_risk == 'High' and self.weather != 'sunny':
                sustainability_log.append(
                    "Nutrients LOW, but Fertilisation BLOCKED "
                    "(CRITICAL: High risk zone near water during "
                    "non-sunny conditions).")
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
            self.get_logger().info(
                "Simulating physical actuation: "
                "Rotating in place to apply treatment...")

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

        if elapsed < 4.0:  # Spin for 4 seconds
            cmd = Twist()
            cmd.angular.z = 0.6  # 0.6 rad/s rotation
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
        if self.current_phase != "IDLE":
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

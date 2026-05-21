#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import time

class CropDecisionNode(Node):
    def __init__(self):
        super().__init__('crop_decision_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.irrigate_pub = self.create_publisher(String, '/irrigate_zone', 10)
        self.fertilise_pub = self.create_publisher(String, '/fertilise_zone', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.moisture_sub = self.create_subscription(Float32MultiArray, '/field_moisture', self.moisture_callback, 10)
        self.nutrients_sub = self.create_subscription(Float32MultiArray, '/field_nutrients', self.nutrients_callback, 10)
        self.growth_sub = self.create_subscription(Float32MultiArray, '/field_growth', self.growth_callback, 10)
        self.nav_status_sub = self.create_subscription(String, '/navigation_executor_status', self.nav_status_callback, 10)
        
        # Weather subscriber / simulation (e.g., Sunny, Rainy, Overcast)
        # We can also read this from a parameter, default is "sunny"
        self.declare_parameter('weather_forecast', 'sunny')
        self.weather = self.get_parameter('weather_forecast').get_parameter_value().string_value

        self.weather_sub = self.create_subscription(String, '/weather_forecast', self.weather_callback, 10)

        # Crop Zones Configuration
        # Coordinates mapped to the physical simulation layout:
        # Zone 0: High land (Low runoff risk)
        # Zone 1: Sloped field (Medium runoff risk)
        # Zone 2: Near downstream water/wetland (High runoff risk!)
        self.zones_data = {
            0: {'id': 'zone_0', 'location': (0.05, 1.5, 90.0), 'runoff_risk': 'Low', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0},
            1: {'id': 'zone_1', 'location': (1.5, 1.5, 0.0), 'runoff_risk': 'Medium', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0},
            2: {'id': 'zone_2', 'location': (1.5, 0.0, -90.0), 'runoff_risk': 'High', 'moisture': 100.0, 'nutrients': 100.0, 'growth': 0.0}
        }

        # System state machine
        # Phases: IDLE, DISPATCHING, NAVIGATING, SCANNING, DECIDING, ACTUATING, COOLDOWN
        self.current_phase = "IDLE"
        self.current_zone_index = 0
        
        # Treatment and telemetry thresholds
        self.moisture_threshold = 40.0 # % Needs water
        self.nutrient_threshold = 50.0 # % Needs fertilizer

        # Telemetry storage flag
        self.telemetry_received = False

        # Actuation control
        self.actuation_timer = None
        self.actuation_start_time = 0.0

        # Cycle timer
        self.cycle_timer = self.create_timer(3.0, self.state_machine_tick)
        self.get_logger().info(f"Nutrient Nexus CropDecisionNode started. Weather forecast: {self.weather.upper()}")

    def weather_callback(self, msg: String):
        old_weather = self.weather
        self.weather = msg.data.lower()
        if old_weather != self.weather:
            self.get_logger().info(f"Weather Forecast updated: {self.weather.upper()}")

    def moisture_callback(self, msg: Float32MultiArray):
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['moisture'] = val
        self.telemetry_received = True

    def nutrients_callback(self, msg: Float32MultiArray):
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['nutrients'] = val

    def growth_callback(self, msg: Float32MultiArray):
        for i, val in enumerate(msg.data):
            if i in self.zones_data:
                self.zones_data[i]['growth'] = val

    def nav_status_callback(self, msg: String):
        status = msg.data
        self.get_logger().info(f"Nav Status: {status} (Nexus Phase: {self.current_phase})")
        if self.current_phase == "NAVIGATING":
            if status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Arrived at Zone {self.current_zone_index}. Commencing sensor scanning...")
                self.current_phase = "SCANNING"
                # Set a 4-second timer to simulate scanning
                self.scan_timer = self.create_timer(4.0, self.complete_scanning)
            elif status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED"]:
                self.get_logger().warn(f"Navigation failed to Zone {self.current_zone_index}. Retrying after cooling down.")
                self.current_phase = "COOLDOWN"
                self.cooldown_timer = self.create_timer(5.0, self.complete_cooldown)

    def complete_scanning(self):
        self.scan_timer.cancel()
        self.get_logger().info(f"Scanning complete for Zone {self.current_zone_index}. Evaluating context...")
        self.current_phase = "DECIDING"
        self.make_contextual_decision()

    def complete_cooldown(self):
        self.cooldown_timer.cancel()
        self.current_phase = "IDLE"
        # Advance to next zone to prevent locking on one bad spot
        self.current_zone_index = (self.current_zone_index + 1) % len(self.zones_data)

    def make_contextual_decision(self):
        zone = self.zones_data[self.current_zone_index]
        moisture = zone['moisture']
        nutrients = zone['nutrients']
        runoff_risk = zone['runoff_risk']
        
        self.get_logger().info(f"=== Scan Telemetry for {zone['id']} ===")
        self.get_logger().info(f" - Soil Moisture: {moisture:.1f}% (Threshold: {self.moisture_threshold}%)")
        self.get_logger().info(f" - Soil Nutrients: {nutrients:.1f}% (Threshold: {self.nutrient_threshold}%)")
        self.get_logger().info(f" - Weather Condition: {self.weather.upper()}")
        self.get_logger().info(f" - Downstream Runoff Vulnerability: {runoff_risk}")

        # Decisions
        irrigation_recommended = False
        fertilisation_recommended = False
        sustainability_log = []

        # 1. Irrigation Rule
        if moisture < self.moisture_threshold:
            if self.weather == 'rainy':
                sustainability_log.append("Moisture LOW, but Irrigation SKIPPED (Naturally irrigated by rain).")
            else:
                irrigation_recommended = True
                sustainability_log.append("Moisture LOW. Irrigation RECOMMENDED.")
        else:
            sustainability_log.append("Moisture adequate. No irrigation needed.")

        # 2. Fertiliser / Nitrogen Rule (Context-Aware / Coastal Eutrophication Risk)
        if nutrients < self.nutrient_threshold:
            if self.weather == 'rainy':
                sustainability_log.append("Nutrients LOW, but Fertilisation BLOCKED (CRITICAL: Heavy rain will wash nutrients downstream!).")
            elif runoff_risk == 'High' and self.weather != 'sunny':
                sustainability_log.append("Nutrients LOW, but Fertilisation BLOCKED (CRITICAL: High risk zone close to water during non-sunny overcast).")
            else:
                fertilisation_recommended = True
                sustainability_log.append("Nutrients LOW. Fertilisation RECOMMENDED.")
        else:
            sustainability_log.append("Nutrients adequate. No fertilisation needed.")

        self.get_logger().info("=== Context-Aware Decision Recommendation ===")
        for log in sustainability_log:
            self.get_logger().info(f" * {log}")

        # Actuate if either treatment is recommended
        if irrigation_recommended or fertilisation_recommended:
            self.current_phase = "ACTUATING"
            self.get_logger().info("Simulating physical actuation: Rotating in place to apply treatment...")
            
            # Record treatments
            if irrigation_recommended:
                msg = String()
                msg.data = zone['id']
                self.irrigate_pub.publish(msg)
            if fertilisation_recommended:
                msg = String()
                msg.data = zone['id']
                self.fertilise_pub.publish(msg)

            # Start active rotation (actuation proxy)
            self.actuation_start_time = self.get_clock().now().nanoseconds / 1e9
            self.actuation_timer = self.create_timer(0.1, self.publish_rotation_tick)
        else:
            self.get_logger().info("No physical action required or safe. Moving to next zone.")
            self.current_phase = "COOLDOWN"
            self.cooldown_timer = self.create_timer(3.0, self.complete_cooldown)

    def publish_rotation_tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.actuation_start_time

        if elapsed < 4.0: # Spin for 4 seconds
            cmd = Twist()
            cmd.angular.z = 0.6  # Rotate at 0.6 rad/s
            self.cmd_vel_pub.publish(cmd)
        else:
            self.actuation_timer.cancel()
            # Stop rotation
            cmd = Twist()
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f"Actuation complete for Zone {self.current_zone_index}. Cooldown initiated.")
            self.current_phase = "COOLDOWN"
            self.cooldown_timer = self.create_timer(3.0, self.complete_cooldown)

    def state_machine_tick(self):
        if self.current_phase != "IDLE":
            return

        if not self.telemetry_received:
            self.get_logger().info("Waiting on field telemetry data before dispatching crop scan...")
            return

        # Choose next zone and dispatch
        zone = self.zones_data[self.current_zone_index]
        self.get_logger().info(f"Navigating to crop {zone['id']} at Location: {zone['location']}")
        
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

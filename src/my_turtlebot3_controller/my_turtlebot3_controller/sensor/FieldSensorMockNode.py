#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from typing import Dict, List, Any
import os
import yaml
import json
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from my_turtlebot3_controller.qos import STATE_QOS

# ── QoS profiles (Lecture Week 6) ────────────────────────────────────


class FieldSensorMockNode(Node):
    def __init__(self) -> None:
        super().__init__('field_sensor_mock_node')

        pkg_dir = get_package_share_directory('my_turtlebot3_controller')
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        
        try:
            with open(zones_file, 'r', encoding='utf-8') as f:
                raw_zones = yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().error(f"Could not find zones.yaml at {zones_file}")
            raw_zones = {}

        self.zones: List[Dict[str, Any]] = []
        for zone_id in sorted(raw_zones.keys()):
            if zone_id == 'base_station':
                continue
                
            baseline_m = raw_zones[zone_id].get('baseline_moisture', 50.0)
            baseline_n = raw_zones[zone_id].get('baseline_nutrients', 50.0)
            runoff_risk = raw_zones[zone_id].get('runoff_risk', 'Low')
            
            self.zones.append({
                'id': zone_id,
                'moisture': float(baseline_m),
                'nutrients': float(baseline_n),
                'growth': random.uniform(10.0, 40.0),
                'runoff_risk': runoff_risk,
                'vulnerability': 0.0
            })
            
        self.num_zones = len(self.zones)
        self.weather = "sunny"

        # Externalised parameters (configurable via nexus_params.yaml)
        self.declare_parameter('sim_tick_interval', 2.0)
        self.declare_parameter('irrigate_replenish_pct', 95.0)
        self.declare_parameter('fertilise_replenish_pct', 90.0)

        self.sim_tick_interval: float = self.get_parameter(
            'sim_tick_interval').get_parameter_value().double_value
        self.irrigate_replenish: float = self.get_parameter(
            'irrigate_replenish_pct').get_parameter_value().double_value
        self.fertilise_replenish: float = self.get_parameter(
            'fertilise_replenish_pct').get_parameter_value().double_value

        # Publishers
        self.moisture_pub = self.create_publisher(Float32MultiArray, '/field_moisture', STATE_QOS)
        self.nutrients_pub = self.create_publisher(Float32MultiArray, '/field_nutrients', STATE_QOS)
        self.growth_pub = self.create_publisher(Float32MultiArray, '/field_growth', STATE_QOS)
        self.vulnerability_pub = self.create_publisher(Float32MultiArray, '/field_vulnerability', STATE_QOS)

        # Subscribers to apply treatment and replenish zone metrics
        self.irrigate_sub = self.create_subscription(String, '/irrigate_zone', self.irrigate_callback, STATE_QOS)
        self.fertilise_sub = self.create_subscription(String, '/fertilise_zone', self.fertilise_callback, STATE_QOS)

        self.weather_sub = self.create_subscription(String, '/weather_forecast', self.weather_cb, STATE_QOS)

        # Single timer for environment simulation + telemetry publishing
        # (merged to avoid race conditions between depletion and publishing)
        self.sim_timer = self.create_timer(self.sim_tick_interval, self.simulation_tick)

        self.get_logger().info("Field Sensor Mock Node Initialized for Nutrient Nexus.")
        self.log_zone_states("Initial")

    def simulation_tick(self) -> None:
        """Combined tick: deplete/grow environment, then publish updated telemetry."""
        self.deplete_and_grow_tick()
        self.publish_telemetry_tick()

    def weather_cb(self, msg: String) -> None:
        self.weather = msg.data.lower()

    def deplete_and_grow_tick(self) -> None:
        """Simulates natural environment processes adapting to live weather."""
        for zone in self.zones:
            # Weather-driven moisture adaptation and nutrient leaching
            if self.weather == "rainy":
                zone['moisture'] += random.uniform(1.0, 3.0)
                zone['nutrients'] -= random.uniform(0.3, 0.8)  # Rain leaches nutrients
            elif self.weather == "storm":
                zone['moisture'] += random.uniform(3.0, 6.0)
                zone['nutrients'] -= random.uniform(1.0, 2.5)  # Severe leaching
            elif self.weather == "sunny":
                zone['moisture'] -= random.uniform(0.5, 1.5)
                zone['nutrients'] -= random.uniform(0.1, 0.4)  # Normal absorption
            elif self.weather == "overcast":
                zone['moisture'] -= random.uniform(0.1, 0.4)
                zone['nutrients'] -= random.uniform(0.1, 0.4)  # Normal absorption
                
            # Clamp limits
            zone['moisture'] = max(5.0, min(99.0, zone['moisture']))
            zone['nutrients'] = max(5.0, zone['nutrients'])

            # Crops grow if they have adequate moisture (>30%) and nutrients (>30%)
            if zone['moisture'] > 30.0 and zone['nutrients'] > 30.0:
                if zone['growth'] < 100.0:
                    zone['growth'] += random.uniform(0.05, 0.2)
                    if zone['growth'] > 100.0:
                        zone['growth'] = 100.0

            # Calculate vulnerability
            risk_map = {'Low': 0.2, 'Medium': 0.5, 'High': 0.8}
            base_risk = risk_map.get(zone.get('runoff_risk', 'Low'), 0.2)
            weather_factor = {'rainy': 1.5, 'storm': 2.0, 'overcast': 0.8, 'sunny': 0.3}.get(self.weather, 0.5)
            moisture_factor = zone['moisture'] / 100.0
            zone['vulnerability'] = min(100.0, base_risk * weather_factor * moisture_factor * 100.0)

    def publish_telemetry_tick(self) -> None:
        """Constructs and publishes array metrics for soil moisture, nutrients, and growth."""
        moisture_msg = self.make_float_array([z['moisture'] for z in self.zones])
        nutrients_msg = self.make_float_array([z['nutrients'] for z in self.zones])
        growth_msg = self.make_float_array([z['growth'] for z in self.zones])
        vulnerability_msg = self.make_float_array([z['vulnerability'] for z in self.zones])

        self.moisture_pub.publish(moisture_msg)
        self.nutrients_pub.publish(nutrients_msg)
        self.growth_pub.publish(growth_msg)
        self.vulnerability_pub.publish(vulnerability_msg)

        self.get_logger().debug("Published environment telemetry updates.")

    def make_float_array(self, data_list: List[float]) -> Float32MultiArray:
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "zones"
        msg.layout.dim[0].size = self.num_zones
        msg.layout.dim[0].stride = self.num_zones
        msg.layout.data_offset = 0
        msg.data = [float(val) for val in data_list]
        return msg

    def irrigate_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            zone_id = data.get('zone', msg.data)
        except (json.JSONDecodeError, AttributeError):
            zone_id = msg.data

        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Irrigating {zone_id}. Moisture replenished from {zone['moisture']:.1f}% to {self.irrigate_replenish:.0f}%")
                zone['moisture'] = self.irrigate_replenish
                break
        self.publish_telemetry_tick()

    def fertilise_callback(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            zone_id = data.get('zone', msg.data)
        except (json.JSONDecodeError, AttributeError):
            zone_id = msg.data

        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Fertilising {zone_id}. Nutrients replenished from {zone['nutrients']:.1f}% to {self.fertilise_replenish:.0f}%")
                zone['nutrients'] = self.fertilise_replenish
                break
        self.publish_telemetry_tick()

    def log_zone_states(self, prefix: str = "Current") -> None:
        states = []
        for z in self.zones:
            states.append(f"{z['id']}: Moisture={z['moisture']:.1f}%, Nutrients={z['nutrients']:.1f}%, Growth={z['growth']:.1f}%")
        self.get_logger().info(f"{prefix} Zone States: [ " + " | ".join(states) + " ]")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FieldSensorMockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Field Sensor Mock Node shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

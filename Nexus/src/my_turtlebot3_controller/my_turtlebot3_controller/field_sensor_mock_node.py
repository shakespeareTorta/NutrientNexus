#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
from typing import Dict, List, Any
import os
import yaml
from ament_index_python.packages import get_package_share_directory

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
            
            self.zones.append({
                'id': zone_id,
                'moisture': float(baseline_m),
                'nutrients': float(baseline_n),
                'growth': random.uniform(10.0, 40.0)
            })
            
        self.num_zones = len(self.zones)
        self.weather = "sunny"

        # Publishers
        self.moisture_pub = self.create_publisher(Float32MultiArray, '/field_moisture', 10)
        self.nutrients_pub = self.create_publisher(Float32MultiArray, '/field_nutrients', 10)
        self.growth_pub = self.create_publisher(Float32MultiArray, '/field_growth', 10)

        # Subscribers to apply treatment and replenish zone metrics
        self.irrigate_sub = self.create_subscription(String, '/irrigate_zone', self.irrigate_callback, 10)
        self.fertilise_sub = self.create_subscription(String, '/fertilise_zone', self.fertilise_callback, 10)
        self.weather_sub = self.create_subscription(String, '/weather_forecast', self.weather_cb, 10)

        # Single timer for environment simulation + telemetry publishing
        # (merged to avoid race conditions between depletion and publishing)
        self.sim_timer = self.create_timer(2.0, self.simulation_tick)

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
            # Weather-driven moisture adaptation
            if self.weather == "rainy":
                zone['moisture'] += random.uniform(1.0, 3.0)
            elif self.weather == "sunny":
                zone['moisture'] -= random.uniform(0.5, 1.5)
            elif self.weather == "overcast":
                zone['moisture'] -= random.uniform(0.1, 0.4)
                
            # Clamp moisture
            zone['moisture'] = max(5.0, min(99.0, zone['moisture']))

            # Nutrients deplete slowly as plants absorb them
            if zone['nutrients'] > 5.0:
                zone['nutrients'] -= random.uniform(0.1, 0.4)
            else:
                zone['nutrients'] = 5.0

            # Crops grow if they have adequate moisture (>30%) and nutrients (>30%)
            if zone['moisture'] > 30.0 and zone['nutrients'] > 30.0:
                if zone['growth'] < 100.0:
                    zone['growth'] += random.uniform(0.05, 0.2)
                    if zone['growth'] > 100.0:
                        zone['growth'] = 100.0

    def publish_telemetry_tick(self) -> None:
        """Constructs and publishes array metrics for soil moisture, nutrients, and growth."""
        moisture_msg = self.make_float_array([z['moisture'] for z in self.zones])
        nutrients_msg = self.make_float_array([z['nutrients'] for z in self.zones])
        growth_msg = self.make_float_array([z['growth'] for z in self.zones])

        self.moisture_pub.publish(moisture_msg)
        self.nutrients_pub.publish(nutrients_msg)
        self.growth_pub.publish(growth_msg)

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
        zone_id = msg.data
        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Irrigating {zone_id}. Moisture replenished from {zone['moisture']:.1f}% to 95.0%")
                zone['moisture'] = 95.0
                break
        self.publish_telemetry_tick()

    def fertilise_callback(self, msg: String) -> None:
        zone_id = msg.data
        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Fertilising {zone_id}. Nutrients replenished from {zone['nutrients']:.1f}% to 90.0%")
                zone['nutrients'] = 90.0
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

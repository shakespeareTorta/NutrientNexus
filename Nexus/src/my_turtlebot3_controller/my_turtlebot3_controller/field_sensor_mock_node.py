#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class FieldSensorMockNode(Node):
    def __init__(self):
        super().__init__('field_sensor_mock_node')

        # Configuration Parameters
        self.declare_parameter('num_zones', 3)
        self.declare_parameter('zone_prefix', 'zone_')
        
        self.num_zones = self.get_parameter('num_zones').get_parameter_value().integer_value
        self.zone_prefix = self.get_parameter('zone_prefix').get_parameter_value().string_value

        # Initialize mock zones with agricultural metrics
        # Zone 0: Safe agricultural zone (dry initially, low runoff risk)
        # Zone 1: Medium zone (reasonable growth, moderate runoff risk)
        # Zone 2: Close to water/stream (high nutrients, high runoff risk)
        self.zones = [
            {
                'id': f'{self.zone_prefix}0',
                'moisture': 35.0,     # Needs irrigation (< 40%)
                'nutrients': 60.0,    # Adequate
                'growth': 20.0        # Young crop
            },
            {
                'id': f'{self.zone_prefix}1',
                'moisture': 55.0,     # Adequate
                'nutrients': 40.0,    # Needs fertilisation (< 50%)
                'growth': 45.0        # Mid-growth crop
            },
            {
                'id': f'{self.zone_prefix}2',
                'moisture': 42.0,     # Borderline
                'nutrients': 30.0,    # Needs fertilisation, but high runoff risk
                'growth': 70.0        # Mature crop
            }
        ]

        # Publishers
        self.moisture_pub = self.create_publisher(Float32MultiArray, '/field_moisture', 10)
        self.nutrients_pub = self.create_publisher(Float32MultiArray, '/field_nutrients', 10)
        self.growth_pub = self.create_publisher(Float32MultiArray, '/field_growth', 10)

        # Subscribers to apply treatment and replenish zone metrics
        self.irrigate_sub = self.create_subscription(String, '/irrigate_zone', self.irrigate_callback, 10)
        self.fertilise_sub = self.create_subscription(String, '/fertilise_zone', self.fertilise_callback, 10)

        # Single timer for environment simulation + telemetry publishing
        # (merged to avoid race conditions between depletion and publishing)
        self.sim_timer = self.create_timer(2.0, self.simulation_tick)

        self.get_logger().info("Field Sensor Mock Node Initialized for Nutrient Nexus.")
        self.log_zone_states("Initial")

    def simulation_tick(self):
        """Combined tick: deplete/grow environment, then publish updated telemetry."""
        self.deplete_and_grow_tick()
        self.publish_telemetry_tick()

    def deplete_and_grow_tick(self):
        """Simulates natural environment processes: moisture evaporation, nutrient absorption, and crop growth."""
        for zone in self.zones:
            # Moisture depletes naturally (evaporation/drainage)
            if zone['moisture'] > 10.0:
                zone['moisture'] -= random.uniform(0.2, 0.8)
            else:
                zone['moisture'] = 10.0

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

    def publish_telemetry_tick(self):
        """Constructs and publishes array metrics for soil moisture, nutrients, and growth."""
        moisture_msg = self.make_float_array([z['moisture'] for z in self.zones])
        nutrients_msg = self.make_float_array([z['nutrients'] for z in self.zones])
        growth_msg = self.make_float_array([z['growth'] for z in self.zones])

        self.moisture_pub.publish(moisture_msg)
        self.nutrients_pub.publish(nutrients_msg)
        self.growth_pub.publish(growth_msg)

        self.get_logger().debug("Published environment telemetry updates.")

    def make_float_array(self, data_list):
        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "zones"
        msg.layout.dim[0].size = self.num_zones
        msg.layout.dim[0].stride = self.num_zones
        msg.layout.data_offset = 0
        msg.data = [float(val) for val in data_list]
        return msg

    def irrigate_callback(self, msg: String):
        zone_id = msg.data
        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Irrigating {zone_id}. Moisture replenished from {zone['moisture']:.1f}% to 95.0%")
                zone['moisture'] = 95.0
                break
        self.publish_telemetry_tick()

    def fertilise_callback(self, msg: String):
        zone_id = msg.data
        for zone in self.zones:
            if zone['id'] == zone_id:
                self.get_logger().info(f"Fertilising {zone_id}. Nutrients replenished from {zone['nutrients']:.1f}% to 90.0%")
                zone['nutrients'] = 90.0
                break
        self.publish_telemetry_tick()

    def log_zone_states(self, prefix="Current"):
        states = []
        for z in self.zones:
            states.append(f"{z['id']}: Moisture={z['moisture']:.1f}%, Nutrients={z['nutrients']:.1f}%, Growth={z['growth']:.1f}%")
        self.get_logger().info(f"{prefix} Zone States: [ " + " | ".join(states) + " ]")

def main(args=None):
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

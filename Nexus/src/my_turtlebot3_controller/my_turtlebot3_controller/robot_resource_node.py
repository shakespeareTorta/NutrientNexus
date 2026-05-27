#!/usr/bin/env python3
"""
Robot Resource Node

Simulates physical resource depletion on the robot:
- Battery drains based on physical distance travelled (calculated from /odom)
- Fertilizer and Water tanks drain when spray commands are issued
- Refills when a message is sent to /refill_resources
Publishes state as a JSON string to /robot_resources.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import json
from typing import Optional

class RobotResourceNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_resource_node')

        # Resource levels (0.0 to 100.0)
        self.battery: float = 100.0
        self.fertilizer: float = 100.0
        self.water: float = 100.0

        # Depletion rates
        # E.g., 2% battery per meter driven
        self.battery_drain_per_meter: float = 2.0 
        # 15% tank used per actuation
        self.fertilizer_drain_per_spray: float = 15.0 
        self.water_drain_per_spray: float = 15.0

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/fertilise_zone', self.fertilize_callback, 10)
        self.create_subscription(String, '/irrigate_zone', self.irrigate_callback, 10)
        self.create_subscription(String, '/refill_resources', self.refill_callback, 10)

        # Publisher
        self.resource_pub = self.create_publisher(String, '/robot_resources', 10)
        self.timer = self.create_timer(1.0, self.publish_resources)

        self.get_logger().info("Robot Resource Node started.")

    def odom_callback(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_x is not None and self.last_y is not None:
            distance = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            if distance > 0.001:  # Only drain if actually moving
                drain = distance * self.battery_drain_per_meter
                self.battery = max(0.0, self.battery - drain)
        
        self.last_x = x
        self.last_y = y

    def fertilize_callback(self, msg: String) -> None:
        self.get_logger().info("Fertilizer sprayed. Draining tank.")
        self.fertilizer = max(0.0, self.fertilizer - self.fertilizer_drain_per_spray)
        self.publish_resources()

    def irrigate_callback(self, msg: String) -> None:
        self.get_logger().info("Water sprayed. Draining tank.")
        self.water = max(0.0, self.water - self.water_drain_per_spray)
        self.publish_resources()

    def refill_callback(self, msg: String) -> None:
        self.get_logger().info("Base Station connected. Refilling all resources to 100%.")
        self.battery = 100.0
        self.fertilizer = 100.0
        self.water = 100.0
        self.publish_resources()

    def publish_resources(self) -> None:
        state = {
            "battery": round(self.battery, 1),
            "fertilizer": round(self.fertilizer, 1),
            "water": round(self.water, 1)
        }
        msg = String()
        msg.data = json.dumps(state)
        self.resource_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotResourceNode()
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

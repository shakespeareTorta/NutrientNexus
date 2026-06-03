#!/usr/bin/env python3
"""
Robot Resource Node

Simulates physical resource depletion on the robot:
- Battery drains based on physical distance travelled (calculated from /odom)
- Fertilizer tank drains when spray commands are issued
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

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from my_turtlebot3_controller.qos import STATE_QOS

class RobotResourceNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_resource_node')
        
        self.declare_parameter('robot_id', 'A')

        # Resource levels (0.0 to 100.0)
        self.battery: float = 100.0
        self.fertilizer: float = 100.0

        # Depletion rates
        # E.g., 2% battery per meter driven
        self.battery_drain_per_meter: float = 2.0 
        # 15% tank used per actuation
        self.fertilizer_drain_per_spray: float = 15.0 

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None

        # Subscribers (use relative topics so namespace /tb2 applies automatically)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(String, 'fertilise_zone', self.fertilize_callback, 10)
        self.create_subscription(String, 'refill_resources', self.refill_callback, 10)

        # Publisher
        self.resource_pub = self.create_publisher(String, 'robot_resources', STATE_QOS)
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
        try:
            data = json.loads(msg.data)
            robot_id = data.get("robot")
            # If the payload specifies a robot, only drain if it matches our namespace
            my_id = self.get_parameter('robot_id').get_parameter_value().string_value
            if robot_id and robot_id != my_id:
                return
        except json.JSONDecodeError:
            pass # Fallback to old behavior if just a string is sent

        self.get_logger().info("Fertilizer sprayed. Draining tank.")
        self.fertilizer = max(0.0, self.fertilizer - self.fertilizer_drain_per_spray)
        self.publish_resources()

    def refill_callback(self, msg: String) -> None:
        self.get_logger().info("Base Station connected. Refilling battery and fertilizer to 100%.")
        self.battery = 100.0
        self.fertilizer = 100.0
        self.publish_resources()

    def publish_resources(self) -> None:
        state = {
            "battery": round(self.battery, 1),
            "fertilizer": round(self.fertilizer, 1)
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

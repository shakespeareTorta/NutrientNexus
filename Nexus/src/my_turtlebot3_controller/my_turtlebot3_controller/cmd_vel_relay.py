#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay_node')
        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',         
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            Twist,
            '/my_tb3/cmd_vel',  
            10)                 
        
        self.get_logger().info('CmdVelRelayNode_PANA_DSP_PROJECT_FR1_Initialise started. Relaying /cmd_vel -> /my_tb3/cmd_vel')

    def listener_callback(self, msg):
        self.publisher.publish(msg) 

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_relay_node = CmdVelRelayNode()

    try:
        rclpy.spin(cmd_vel_relay_node)
    except KeyboardInterrupt:
        cmd_vel_relay_node.get_logger().info('CmdVelRelayNode stopped cleanly')
    finally:
        cmd_vel_relay_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
# my_turtlebot3_controller/move_turtlebot.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTurtleBot3(Node):

    def __init__(self):
        super().__init__('move_turtlebot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cmd = Twist()

    def timer_callback(self):
        self.cmd.linear.x = 0.2  # Move forward at 0.2 m/s
        self.publisher.publish(self.cmd)
        self.get_logger().info('Moving forward...')

def main(args=None):
    rclpy.init(args=args)
    node = MoveTurtleBot3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

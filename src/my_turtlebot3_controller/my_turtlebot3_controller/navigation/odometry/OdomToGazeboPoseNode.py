#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw.
    quaternion = [x, y, z, w]
    This function now uses the standard 'math' module instead of 'numpy'.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class OdomToGazeboPoseNode(Node): 
    """
    This node creates a simple feedback controller to make a simulated robot
    in Gazebo follow the odometry of a real robot.
    It subscribes to the real robot's odometry (the target) and the simulated
    robot's odometry (the current state), calculates the error, and publishes
    a cmd_vel to the simulated robot to minimize the error.
    """
    def __init__(self):
        super().__init__('OdomToGazeboPoseNode')

        self.declare_parameter('real_odom_topic', '/odom')
        self.declare_parameter('sim_odom_topic', '/my_tb3/odom')
        self.declare_parameter('sim_cmd_vel_topic', '/my_tb3/cmd_vel')
        
        self.declare_parameter('proportional_gain_linear', 0.8) 
        self.declare_parameter('proportional_gain_angular', 1.0) 
        self.declare_parameter('goal_tolerance_linear', 0.05)   
        self.declare_parameter('goal_tolerance_angular', 0.05)  

        real_odom_topic = self.get_parameter('real_odom_topic').get_parameter_value().string_value
        sim_odom_topic = self.get_parameter('sim_odom_topic').get_parameter_value().string_value
        sim_cmd_vel_topic = self.get_parameter('sim_cmd_vel_topic').get_parameter_value().string_value
        
        self.k_linear = self.get_parameter('proportional_gain_linear').get_parameter_value().double_value
        self.k_angular = self.get_parameter('proportional_gain_angular').get_parameter_value().double_value
        self.tolerance_linear = self.get_parameter('goal_tolerance_linear').get_parameter_value().double_value
        self.tolerance_angular = self.get_parameter('goal_tolerance_angular').get_parameter_value().double_value
        

        self.target_pose = None      
        self.current_sim_pose = None 
        

        self.target_odom_sub = self.create_subscription(
            Odometry, real_odom_topic, self.target_odom_callback, 10)
            
        self.sim_odom_sub = self.create_subscription(
            Odometry, sim_odom_topic, self.sim_odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, sim_cmd_vel_topic, 10)

        self.controller_timer = self.create_timer(0.1, self.controller_loop)

        self.get_logger().info("Pose Controller Node Initialized.")
        self.get_logger().info(f"Subscribing to real odom on: {real_odom_topic}")
        self.get_logger().info(f"Subscribing to sim odom on: {sim_odom_topic}")
        self.get_logger().info(f"Publishing control commands to: {sim_cmd_vel_topic}")

    def target_odom_callback(self, msg: Odometry):
        self.target_pose = msg.pose.pose

    def sim_odom_callback(self, msg: Odometry):
        self.current_sim_pose = msg.pose.pose

    def controller_loop(self):
        if self.target_pose is None or self.current_sim_pose is None:
            return


        error_x = self.target_pose.position.x - self.current_sim_pose.position.x
        error_y = self.target_pose.position.y - self.current_sim_pose.position.y
        distance_error = math.sqrt(error_x**2 + error_y**2)

        # Angular Error
        _, _, target_yaw = euler_from_quaternion(self.target_pose.orientation)
        _, _, current_yaw = euler_from_quaternion(self.current_sim_pose.orientation)
        angle_to_goal = math.atan2(error_y, error_x)
        
        # We first need to point towards the goal, then match the final orientation
        if distance_error > self.tolerance_linear:
            angle_error = angle_to_goal - current_yaw
        else:
            angle_error = target_yaw - current_yaw

        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        cmd_vel_msg = Twist()
        
        if distance_error > self.tolerance_linear:
            if abs(angle_error) < math.pi / 4:
                cmd_vel_msg.linear.x = self.k_linear * distance_error
            else:
                cmd_vel_msg.linear.x = 0.0
        else:
            cmd_vel_msg.linear.x = 0.0
        
        if abs(angle_error) > self.tolerance_angular:
            cmd_vel_msg.angular.z = self.k_angular * angle_error
        else:
            cmd_vel_msg.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToGazeboPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

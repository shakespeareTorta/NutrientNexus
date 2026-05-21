#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float32MultiArray, String
from nav_msgs.msg import Odometry 
import math
from geometry_msgs.msg import PoseStamped


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')


        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)
        self.reset_bin_publisher = self.create_publisher(String, '/set_bin_level_zero', 10)


        self.fill_level_subscriber = self.create_subscription(
            Float32MultiArray,
            '/bin_fill_levels',
            self.fill_level_callback,
            10)
        
        self.nav_status_subscriber = self.create_subscription(
            String, 
            '/navigation_executor_status',
            self.navigation_status_callback,
            10)
            
        self.sim_odom_subscriber = self.create_subscription(
            Odometry,
            '/my_tb3/odom', # Listens to the Gazebo twin's odom
            self.sim_odom_callback,
            10)

        # --- Configuration ---
        self.bins_data = {
            0: {'name': 'bin_0', 'location': (0.05, 1.5, 90.0),  'fill': 0.0},
            1: {'name': 'bin_1', 'location': (1.5,  1.5, 0.0), 'fill': 0.0},
            2: {'name': 'bin_2', 'location': (1.5, 0.0, -90.0), 'fill': 0.0},
        }
        self.fill_threshold = 50.0 
        self.required_stop_duration_sec = 10.0
        
        self.current_sim_pose = None 
        self.current_task_phase = "IDLE" # IDLE, WAITING_FOR_NAV_SUCCESS, WAITING_AT_BIN
        self.dispatched_bin_index = None
        self.at_bin_wait_timer = None 
        

        self.decision_timer = self.create_timer(5.0, self.make_decision_cycle)

        self.get_logger().info("Decision Node Initialized with Distance Priority Logic.")
        for i, data in self.bins_data.items():
            self.get_logger().info(f"  {data['name']} (Index {i}): Loc {data['location']}, Fill {data['fill']}%")

    def fill_level_callback(self, msg: Float32MultiArray):
        for i, level in enumerate(msg.data):
            if i in self.bins_data:
                self.bins_data[i]['fill'] = level

    def sim_odom_callback(self, msg: Odometry):

        self.current_sim_pose = msg.pose.pose

    def navigation_status_callback(self, msg: String):
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Executor Status: '{nav_status}' (Phase: '{self.current_task_phase}')")

        if self.current_task_phase == "WAITING_FOR_NAV_SUCCESS":
            if nav_status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Navigation to bin {self.dispatched_bin_index} succeeded. Starting {self.required_stop_duration_sec}s wait.")
                self.current_task_phase = "WAITING_AT_BIN"
                if self.at_bin_wait_timer: self.at_bin_wait_timer.cancel()
                self.at_bin_wait_timer = self.create_timer(self.required_stop_duration_sec, self.ten_second_wait_is_over_callback)

            elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE"]:
                self.get_logger().warn(f"Navigation task for bin {self.dispatched_bin_index} failed. Resetting.")
                self.reset_task_state()

    def ten_second_wait_is_over_callback(self):
        self.get_logger().info(f"10s wait at bin {self.dispatched_bin_index} complete. Commanding bin empty.")
        if self.at_bin_wait_timer: self.at_bin_wait_timer.cancel()
        
        if self.dispatched_bin_index is not None:
            bin_id_str = self.bins_data[self.dispatched_bin_index]['name']
            reset_cmd = String()
            reset_cmd.data = bin_id_str
            self.reset_bin_publisher.publish(reset_cmd)
        
        self.reset_task_state()

    def make_decision_cycle(self):
        if self.current_task_phase != "IDLE":
            return # Robot is busy


        
        # 1. Find all bins that are over the threshold
        candidate_bins = []
        for index, data in self.bins_data.items():
            if data['fill'] >= self.fill_threshold:
                candidate_bins.append(index)
        
        if not candidate_bins:
            self.get_logger().debug("Decision cycle: No bins meet dispatch criteria.")
            return

        # 2. If candidates exist, we need the robot's pose to find the closest one
        if self.current_sim_pose is None:
            self.get_logger().warn("Decision cycle: Cannot make distance-based decision, sim pose is unknown. Waiting...")
            return

        closest_bin_index = None
        min_distance = float('inf')

        self.get_logger().info("Evaluating candidate bins:")
        for bin_index in candidate_bins:
            bin_location = self.bins_data[bin_index]['location']
            dist = math.sqrt(
                (self.current_sim_pose.position.x - bin_location[0])**2 +
                (self.current_sim_pose.position.y - bin_location[1])**2
            )
            self.get_logger().info(f"  - Bin {bin_index}: Distance = {dist:.2f}m")

            if dist < min_distance:
                min_distance = dist
                closest_bin_index = bin_index
        
        # 3. Dispatch to the closest candidate bin
        if closest_bin_index is not None:
            self.dispatched_bin_index = closest_bin_index
            
            x, y, yaw_deg = self.bins_data[self.dispatched_bin_index]['location']
            
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map'
            goal_pose_msg.pose.position.x = float(x)
            goal_pose_msg.pose.position.y = float(y)

            yaw_rad = math.radians(float(yaw_deg))
            goal_pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            self.goal_publisher.publish(goal_pose_msg)
            self.current_task_phase = "WAITING_FOR_NAV_SUCCESS"
            bin_name = self.bins_data[self.dispatched_bin_index]['name']
            self.get_logger().info(f"Decision: Dispatched robot to CLOSEST full bin: {bin_name} (Index: {self.dispatched_bin_index})")


    def reset_task_state(self):
        self.get_logger().info("Resetting task state to IDLE.")
        self.current_task_phase = "IDLE"
        self.dispatched_bin_index = None

def main(args=None):
    rclpy.init(args=args)
    decision_node = DecisionNode()
    try:
        rclpy.spin(decision_node)
    except KeyboardInterrupt:
        pass
    finally:
        decision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

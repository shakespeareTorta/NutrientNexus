import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class Navigation(Node):
    def __init__(self):
        # create 'navigation' ROS node
        super().__init__("navigation")

        # create client to send '/navigate_to_pose' action
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.goal_sent = False
        # call try_send_goal() every second
        self.timer = self.create_timer(1.0, self.try_send_goal)
        self.get_logger().info("navigation: node started")

    def try_send_goal(self):
        if self.goal_sent:
            return

        # wait on nav2 server if not available
        if not self.client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("navigation: waiting on nav2 server...")
            return

        self.goal_sent = True

        # assemble action content (target location)
        goal_msg = NavigateToPose.Goal()
        # Pose here refers to a position and rotation in a coordinate system
        pose = PoseStamped()
        # goal defined in 'map' ('odom' accuracy drifts)
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        # (0,0) is the robot position when SLAM initialized
        pose.pose.position.x = 2.0
        pose.pose.position.y = 0.0
        # (0,0,0,1) is quaternion identity rotation - no change
        pose.pose.orientation.w = 1.0

        goal_msg.pose = pose
        self.get_logger().info("navigation: sending goal...")
        future = self.client.send_goal_async(goal_msg)
        future.add_done_callback(self.send_goal_callback)

    def send_goal_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("navigation: goal rejected")
            return
        self.get_logger().info("navigation: goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info("navigation: goal done")


def main(args=None):
    # init ROS comm. system (e.g. topics, actions, etc.)
    rclpy.init(args=args)

    node = Navigation()
    # keep node active, process callbacks
    rclpy.spin(node)

    # cleanup when stopping
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

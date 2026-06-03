#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import String 
from action_msgs.msg import GoalStatus as ActionGoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
from typing import Optional

from my_turtlebot3_controller.qos import STATE_QOS

class NavigationExecutorNode(Node): 
    def __init__(self) -> None:
        super().__init__('navigation_executor_node') 
        # Use relative topics so this works inside namespaces (/tb2)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') 
        
        self.goal_handle: Optional[ClientGoalHandle] = None
        self.current_nav_status: str = "IDLE"
        self.active_goal_pose_for_logging: Optional[PoseStamped] = None
        self._idle_timer = None

        # Subscriber - incoming navigation goals
        self.goal_subscriber = self.create_subscription(
            PoseStamped,
            'dispatch_nav_goal', 
            self.dispatch_goal_callback,
            10)
        
        # Publisher - navigation status
        self.status_publisher = self.create_publisher(String, 'navigation_executor_status', STATE_QOS)

        self.get_logger().info("Navigation Executor Node Initialized. Waiting for goals on dispatch_nav_goal.")
        self._publish_status("IDLE")
        self.create_timer(0.5, self._republish_status)

    def _republish_status(self) -> None:
        msg = String()
        msg.data = self.current_nav_status
        self.status_publisher.publish(msg)

    def _publish_status(self, status_str: str) -> None:
        msg = String()
        msg.data = status_str
        self.status_publisher.publish(msg)
        if status_str != self.current_nav_status:
            self.get_logger().info(f"Status: {self.current_nav_status} -> {status_str}")
        self.current_nav_status = status_str

    def dispatch_goal_callback(self, msg: PoseStamped) -> None:
        if self.current_nav_status == "NAVIGATING":
            self.get_logger().warn("Navigation already in progress. New goal ignored.")
            return

        if self._idle_timer is not None:
            self._idle_timer.cancel()
            self.destroy_timer(self._idle_timer)
            self._idle_timer = None

        self.active_goal_pose_for_logging = msg
        self.get_logger().info(f"Received dispatch goal: X={msg.pose.position.x:.2f}, Y={msg.pose.position.y:.2f}")

        if self.send_nav2_goal(msg):
            self._publish_status("NAVIGATING")


    def wait_for_action_server(self, timeout_sec: float = 2.0) -> bool:
        self.get_logger().info('Waiting for /navigate_to_pose action server...')

        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('/navigate_to_pose action server not available after waiting.')
            self._publish_status("IDLE_SERVER_UNAVAILABLE") 
            return False

        self.get_logger().info('/navigate_to_pose action server is available.')
        return True

    def send_nav2_goal(self, target_pose_stamped: PoseStamped) -> bool: 
        if not self.wait_for_action_server():
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose_stamped 

        self.get_logger().info(f"Sending goal to Nav2: Pos(X:{target_pose_stamped.pose.position.x:.2f}, Y:{target_pose_stamped.pose.position.y:.2f})")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True


    def _schedule_idle(self) -> None:
        """Cancel any pending idle transition, then schedule a new one-shot."""
        if self._idle_timer is not None:
            self._idle_timer.cancel()
            self.destroy_timer(self._idle_timer)
            self._idle_timer = None
        self._idle_timer = self.create_timer(0.2, self._transition_to_idle)

    def _transition_to_idle(self) -> None:
        """One-shot: publish IDLE then cancel self."""
        if self._idle_timer is not None:
            self._idle_timer.cancel()
            self.destroy_timer(self._idle_timer)
            self._idle_timer = None
        self._publish_status("IDLE")

    def goal_response_callback(self, future) -> None:
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server.')
            self._publish_status("REJECTED")
            self.active_goal_pose_for_logging = None
            self._schedule_idle()
            return

        self.get_logger().info('Goal accepted by Nav2 server. Waiting for result...')

        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) -> None:
        status_code = future.result().status
        final_status_str = "UNKNOWN_FAILURE"

        if status_code == ActionGoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation Succeeded!')
            final_status_str = "SUCCEEDED_AT_POSE"
        elif status_code == ActionGoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Navigation Aborted by Nav2. Status Code: {status_code}')
            final_status_str = "ABORTED_NAVIGATION"
        elif status_code == ActionGoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Navigation Canceled. Status Code: {status_code}')
            final_status_str = "CANCELED_NAVIGATION"
        else:
            self.get_logger().info(f'Navigation finished with status code: {status_code}')

        self._publish_status(final_status_str)
        self.active_goal_pose_for_logging = None
        self._schedule_idle()

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().debug(f"Navigating to goal, Dist_rem: {feedback.distance_remaining:.2f} m")

def main(args=None) -> None:
    rclpy.init(args=args)
    executor_node = NavigationExecutorNode() 

    try:
        rclpy.spin(executor_node) 
    except KeyboardInterrupt:
        executor_node.get_logger().info("Navigation Executor Node shutting down.")
    finally:
        executor_node.get_logger().info("Destroying Navigation Executor Node.")
        executor_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
Cmd-Vel Stamper Node — Twist -> TwistStamped adapter for the physical TurtleBot3.

ROS 2 Jazzy's turtlebot3_node subscribes to /cmd_vel as
geometry_msgs/TwistStamped (enable_stamped_cmd_vel: true in burger.yaml),
whereas Nav2 and the safety layer emit plain geometry_msgs/Twist. This node
bridges the two by re-publishing every incoming Twist as a stamped message
with a fresh header.

It is built into this package so the real-robot launch carries NO out-of-tree
runtime dependency. (Previously the launch relied on the external
`twist_stamper` apt package; when it was not installed the node failed to
start, /cmd_vel was never published, and the physical robot stayed silently
immobile.)

Topic pipeline (real robot):
    TwinSafetyNode --(Twist)--> /cmd_vel_unstamped --> [this node]
                   --(TwistStamped)--> /cmd_vel --> turtlebot3_node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelStamperNode(Node):
    """
    Wrap incoming geometry_msgs/Twist messages into geometry_msgs/TwistStamped.

    Class invariant (holds between every callback):
        Every message published on `output_topic` is a TwistStamped whose
        .twist equals the most recently received Twist, whose header.stamp is
        the node clock time at publish, and whose header.frame_id equals the
        configured `frame_id`.
    """

    def __init__(self) -> None:
        """
        Construct the node, read parameters, and wire one subscription and one
        publisher.

        @pre   rclpy.init() has been called (a ROS 2 context exists).
        @post  - Node 'cmd_vel_stamper_node' is registered in the ROS graph.
               - A subscription on `input_topic` (Twist) and a publisher on
                 `output_topic` (TwistStamped) are active.
        @return None
        """
        super().__init__('cmd_vel_stamper_node')

        self.declare_parameter('input_topic', '/cmd_vel_unstamped')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')

        self.input_topic: str = self.get_parameter(
            'input_topic').get_parameter_value().string_value
        self.output_topic: str = self.get_parameter(
            'output_topic').get_parameter_value().string_value
        self.frame_id: str = self.get_parameter(
            'frame_id').get_parameter_value().string_value

        self.publisher = self.create_publisher(
            TwistStamped, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Twist, self.input_topic, self._cmd_cb, 10)

        self.get_logger().info(
            f'Cmd-Vel Stamper started: {self.input_topic} (Twist) -> '
            f'{self.output_topic} (TwistStamped, frame_id="{self.frame_id}")')

    def _cmd_cb(self, msg: Twist) -> None:
        """
        Re-publish a received Twist as a TwistStamped.

        @param msg  geometry_msgs/Twist velocity command from the safety layer.
        @pre   msg is a valid Twist (guaranteed by the subscription type).
        @post  Exactly one geometry_msgs/TwistStamped is published on
               `output_topic`, with header.stamp = now,
               header.frame_id = self.frame_id and .twist = msg.
        @return None
        """
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self.frame_id
        stamped.twist = msg
        self.publisher.publish(stamped)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelStamperNode()
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

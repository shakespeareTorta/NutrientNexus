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
    """
    Tracks the robot's consumable resources (battery and fertilizer tank) and
    publishes them as telemetry on /robot_resources.
 
    Class invariant (must hold between every call):
        0.0 <= self.battery    <= 100.0
        0.0 <= self.fertilizer <= 100.0
 
    Every mutator preserves this invariant: each drain is clamped with
    max(0.0, ...) and a refill sets the value to exactly 100.0.
    """
    def __init__(self) -> None:
        """
        Construct the node: declare the robot_id parameter, initialise resource
        state, and wire up the three subscriptions, the publisher and the 1 Hz
        publish timer.
 
        @param  (none besides self)
        @pre    rclpy.init() has been called (a ROS 2 context exists) before this
                node is constructed.
        @post   - The node is registered in the ROS graph as 'robot_resource_node'.
                - self.battery == 100.0 and self.fertilizer == 100.0
                  (class invariant established).
                - self.last_x is None and self.last_y is None (no odom seen yet).
                - Subscriptions to 'odom', 'fertilise_zone' and 'refill_resources'
                  and the publisher on 'robot_resources' are active.
                - A timer is scheduled to call publish_resources() once per second.
        @return None
        """
        super().__init__('robot_resource_node')
        
        self.declare_parameter('robot_id', 'A')

        # Resource levels (0.0 to 100.0)
        self.battery: float = 100.0
        self.fertilizer: float = 100.0

        # Depletion rates (configurable via nexus_params.yaml)
        self.declare_parameter('battery_drain_per_meter', 2.0)
        self.declare_parameter('fertilizer_drain_per_spray', 15.0)

        self.battery_drain_per_meter: float = self.get_parameter(
            'battery_drain_per_meter').get_parameter_value().double_value
        self.fertilizer_drain_per_spray: float = self.get_parameter(
            'fertilizer_drain_per_spray').get_parameter_value().double_value

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
        """
        Integrate the distance travelled since the previous odometry sample and
        drain the battery proportionally to it.
 
        Client: the ROS 2 executor, which invokes this callback for every message
        received on the 'odom' topic. Because the caller is the trusted middleware
        and the message type is guaranteed by the subscription, no defensive
        precondition check is performed here.
 
        @param msg  nav_msgs/Odometry. Only msg.pose.pose.position.x and .y
                    (metres, in the odom frame) are read.
        @pre   msg is a valid Odometry message with a finite (x, y) position.
        @post  - If this is the first sample (self.last_x is None): the position is
                 recorded and the battery is left unchanged.
               - Otherwise, let d = Euclidean distance from the previous sample:
                   * if d > 0.001 m: self.battery decreases by
                     d * self.battery_drain_per_meter, clamped so that
                     self.battery >= 0.0 (class invariant preserved);
                   * if d <= 0.001 m (noise / stationary): battery unchanged.
               - self.last_x / self.last_y are updated to the current position
                 (the reference for the next call).
        @return None  (the change is broadcast later by the 1 Hz publish timer.)
        """
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
        """
        Drain the fertilizer tank when a spray command addressed to THIS robot is
        received, then publish the updated state.
 
        Client: CropDecisionNode, which publishes the spray command on
        'fertilise_zone'.
 
        @param msg  std_msgs/String. msg.data is expected to be a JSON object of
                    the form {"robot": <id>, "zone": <zone_id>}. A bare (non-JSON)
                    string is also accepted as a legacy fallback.
        @pre   msg.data is a String. If it is valid JSON and contains a "robot"
                key, that key names the intended target robot.
        @post  - If the payload names a robot whose id != this node's robot_id
                 parameter: nothing changes (early return).
               - Otherwise: self.fertilizer decreases by
                 self.fertilizer_drain_per_spray, clamped so self.fertilizer >= 0.0
                 (class invariant preserved), and the new state is published
                 immediately.
        @return None
        @throws (none) json.JSONDecodeError is caught locally and handled by the
                legacy bare-string fallback. The catch is deliberately narrow
                (only JSONDecodeError) to avoid the catch(Exception) anti-pattern.
        """
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
        """
        Reset both resources to full. Triggered when the robot docks at the base
        station.
 
        Client: CropDecisionNode, which publishes on 'refill_resources' once it
        has successfully returned to base.
 
        @param msg  std_msgs/String. The content is ignored; the mere arrival of a
                    message on 'refill_resources' is the trigger.
        @pre   (none on the client beyond publishing any message on
                'refill_resources'.)
        @post  self.battery == 100.0 and self.fertilizer == 100.0 (class invariant
               trivially holds), and the new state is published immediately.
        @return None
        """
        self.get_logger().info("Base Station connected. Refilling battery and fertilizer to 100%.")
        self.battery = 100.0
        self.fertilizer = 100.0
        self.publish_resources()

    def publish_resources(self) -> None:
        """
        Serialise the current resource state to JSON and publish it on
        'robot_resources'.
 
        Called by: the 1 Hz heartbeat timer, and directly by fertilize_callback
        and refill_callback for immediate updates.
 
        @param  (none besides self)
        @pre    self.resource_pub has been created (true after __init__).
        @post   A std_msgs/String whose data is the JSON object
                {"battery": <self.battery rounded to 1 dp>,
                 "fertilizer": <self.fertilizer rounded to 1 dp>}
                is published on 'robot_resources'. No resource value is mutated
                (this method is read-only with respect to battery / fertilizer).
        @return None
        """
        state = {
            "battery": round(self.battery, 1),
            "fertilizer": round(self.fertilizer, 1)
        }
        msg = String()
        msg.data = json.dumps(state)
        self.resource_pub.publish(msg)


def main(args=None) -> None:
    """
    ROS 2 entry point: initialise the context, spin the node until interrupted,
    then shut down cleanly.
 
    @param args  Optional command-line arguments forwarded to rclpy.init().
                 Defaults to None.
    @pre   (none) this function initialises rclpy itself.
    @post  The node has been spun until a KeyboardInterrupt or shutdown, then
           destroyed; rclpy is shut down if the context was still active.
    @return None
    """
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

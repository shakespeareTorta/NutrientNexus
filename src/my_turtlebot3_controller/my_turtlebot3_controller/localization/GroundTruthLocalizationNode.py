#!/usr/bin/env python3
"""
Ground-Truth Localization (simulation only).

slam_toolbox drifts badly in this small, symmetric room: the `map` frame rotates
off the world and the wheel `odom` slips during recovery spins, so the robot
navigates to the wrong physical places and the Gazebo zone tiles no longer line
up with where the robot actually goes.

Because this is a SIM with a fully known world, we localise from ground truth:
we read the robot's TRUE pose straight from Gazebo (by streaming the gz
`/world/<world>/pose/info` topic, which carries each entity's name) and publish
the map->odom transform so that map->base_footprint equals that true pose. The
`map` frame therefore coincides exactly with the Gazebo world frame — the robot
is always perfectly localised, so it reaches the real field zones and the
anchored tiles line up with it.

We read gz directly (not via the ros_gz bridge) because the bridge's
Pose_V -> TFMessage conversion drops the entity names (empty child_frame_id),
making the robot impossible to pick out. Until the first true pose arrives we
publish an identity map->odom (correct at spawn, where odom == world) so Nav2
can still bring up its costmaps.

Nav2's global costmap builds obstacles from /scan (no static-map layer), so
dropping SLAM costs nothing for planning.
"""
import math
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import (Buffer, TransformListener, TransformBroadcaster,
                     LookupException, ConnectivityException, ExtrapolationException)
from geometry_msgs.msg import TransformStamped


class GroundTruthLocalizationNode(Node):
    def __init__(self) -> None:
        """Set up the TF buffer/broadcaster, start the background thread that
        reads the robot's true Gazebo pose, and start the map->odom timer."""
        super().__init__('ground_truth_localization')

        self.declare_parameter('robot_model_name', 'burger')
        self.declare_parameter('world_name', 'default')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)

        self.robot = self.get_parameter('robot_model_name').get_parameter_value().string_value
        self.world = self.get_parameter('world_name').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.true_pose = None      # (x, y, yaw) of the robot in the Gazebo world
        self._got_pose = False
        self._proc = None

        # Stream the robot's true pose from Gazebo in a background thread.
        self._reader = threading.Thread(target=self._stream_true_pose, daemon=True)
        self._reader.start()

        self.create_timer(1.0 / rate, self._publish_map_to_odom)

        self.get_logger().info(
            f"Ground-truth localization active: pinning '{self.map_frame}' to the "
            f"Gazebo world via model '{self.robot}' (replaces SLAM).")

    # ── Read the robot's true Gazebo pose by streaming gz dynamic_pose/info ──
    # dynamic_pose/info carries only the *moving* entities (essentially just the
    # robot), so it is far lighter than the full pose/info (~40 static walls and
    # tiles) — important, because parsing the full set starves Nav2's planner of
    # CPU and makes it time out on long paths to the corners.
    def _stream_true_pose(self) -> None:
        """
        Stream and parse the robot's true Gazebo pose (background thread).

        Runs `gz topic -e` on dynamic_pose/info and parses the named robot
        entity's position and yaw, caching it as the (x, y, yaw) tuple
        self.true_pose for the publish timer to read.

        @pre  The 'gz' CLI is available and the Gazebo server is running.
        @post self.true_pose is updated on every frame; self._got_pose latches
              True after the first successful parse. Loops until the process ends.
        @return None.
        """
        topic = f'/world/{self.world}/dynamic_pose/info'
        try:
            self._proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', topic],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, bufsize=1)
        except FileNotFoundError:
            self.get_logger().error("'gz' CLI not found — cannot read ground truth.")
            return

        is_robot = in_pos = in_orient = False
        px = py = oz = ow = None
        target = f'"{self.robot}"'
        for raw in self._proc.stdout:
            line = raw.strip()
            if line.startswith('name:'):
                is_robot = (target in line)
                in_pos = in_orient = False
            elif is_robot and line == 'position {':
                in_pos, px, py = True, None, None
            elif is_robot and line == 'orientation {':
                in_orient, oz, ow = True, None, None
            elif in_pos and line.startswith('x:'):
                px = float(line.split(':', 1)[1])
            elif in_pos and line.startswith('y:'):
                py = float(line.split(':', 1)[1])
            elif in_orient and line.startswith('z:'):
                oz = float(line.split(':', 1)[1])
            elif in_orient and line.startswith('w:'):
                ow = float(line.split(':', 1)[1])
            elif line == '}':
                if in_orient and None not in (px, py, oz, ow):
                    yaw = math.atan2(2.0 * ow * oz, 1.0 - 2.0 * oz * oz)
                    self.true_pose = (px, py, yaw)   # atomic tuple assignment
                    if not self._got_pose:
                        self._got_pose = True
                        self.get_logger().info("Locked onto robot ground-truth pose.")
                    is_robot = False
                in_pos = in_orient = False

    def _publish_map_to_odom(self) -> None:
        """
        Broadcast the map->odom transform from ground truth.

        Computes map->odom so that map->base_footprint equals the robot's true
        pose, making the map frame coincide with the Gazebo world frame.

        @pre  An odom->base_footprint transform is available in TF.
        @post One map->odom transform is broadcast; identity is used until the
              first ground-truth pose arrives (correct at spawn).
        @return None.
        """
        try:
            ob = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        ox = ob.transform.translation.x
        oy = ob.transform.translation.y
        oq = ob.transform.rotation
        oyaw = math.atan2(2.0 * (oq.w * oq.z + oq.x * oq.y),
                          1.0 - 2.0 * (oq.y * oq.y + oq.z * oq.z))

        if self.true_pose is None:
            # Identity map->odom until ground truth arrives (correct at spawn).
            mx = my = th = 0.0
        else:
            tx, ty, wyaw = self.true_pose
            th = wyaw - oyaw
            c, s = math.cos(th), math.sin(th)
            mx = tx - (ox * c - oy * s)
            my = ty - (ox * s + oy * c)

        out = TransformStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = mx
        out.transform.translation.y = my
        out.transform.rotation.z = math.sin(th / 2.0)
        out.transform.rotation.w = math.cos(th / 2.0)
        self.br.sendTransform(out)

    def destroy_node(self):
        """Terminate the gz streaming subprocess before tearing down the node."""
        if self._proc is not None:
            try:
                self._proc.terminate()
            except OSError:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthLocalizationNode()
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

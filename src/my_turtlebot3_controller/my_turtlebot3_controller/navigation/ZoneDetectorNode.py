#!/usr/bin/env python3
"""
Zone Detector Node — report the robot's current field zone and draw markers.

Determines which agricultural zone the robot is currently in (by looking up its
map-frame pose via TF and testing it against the zone bounding boxes from
config/zones.yaml), publishes that zone name on /current_zone, and publishes
RViz markers (/zone_markers) visualising the zone boxes and their target points.
"""
import os
from typing import Any, Dict

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from my_turtlebot3_controller.qos import STATE_QOS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import (Buffer, ConnectivityException, ExtrapolationException,
                     LookupException, TransformListener)
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def _package_share_or_source_dir() -> str:
    """
    Resolve the directory that is expected to contain 'config/zones.yaml'.

    @param  (none)
    @pre    (none)
    @post   Returns the installed package share directory of
            'my_turtlebot3_controller' if it can be resolved; otherwise returns
            the parent-of-parent directory of this source file as a fallback
            (useful when running from source without a built/sourced install).
    @return str - a directory path. NOTe: the path is not guaranteed to exist;
            the caller is still responsible for handling a missing file.
    @throws (none) - the only expected failure, PackageNotFoundError (raised by
            get_package_share_directory when the package is not built/sourced),
            is caught here and handled by the source-dir fallback.
    """
    try:
        return get_package_share_directory('my_turtlebot3_controller')
    except PackageNotFoundError:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ZoneDetectorNode(Node):
    """
    Publishes the robot's current zone and the RViz visualisation of all zones.

    Class invariant (must hold between every call):
        - self.zones is a dict (possibly empty) mapping zone-name -> zone
          definition (each definition ideally provides min_x/max_x/min_y/max_y
          and target_x/target_y).
        - self.current_zone is either 'no_zone' or a key of self.zones.
        - (self.current_x, self.current_y) is the most recent map-frame position
          for which a zone was resolved; it is (0.0, 0.0) until the first
          successful TF lookup.
    """

    def __init__(self) -> None:
        """
        Construct the node and bring up its ROS interfaces.

        Loads the zone definitions from zones.yaml, sets up the TF listener, the
        two publishers (/current_zone, /zone_markers) and the two periodic
        timers.

        @param  (none besides self)
        @pre    rclpy.init() has been called (a ROS 2 context exists) before this
                node is constructed.
        @post   - The node is registered in the ROS graph as 'zone_detector_node'.
                - self.zones holds the parsed contents of config/zones.yaml, or an
                  empty dict if the file is missing (an error is logged).
                - self.current_x == 0.0, self.current_y == 0.0,
                  self.current_zone == 'no_zone' (class invariant established).
                - A TF buffer and listener are active so map->base_footprint can
                  be looked up.
                - Publishers on '/current_zone' (String, STATE_QOS) and
                  '/zone_markers' (MarkerArray) are active.
                - A 0.5 s timer calls update_and_publish_zone() and a 1.0 s timer
                  calls publish_markers().
        @return None
        """
        super().__init__('zone_detector_node')

        pkg_dir = _package_share_or_source_dir()
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')

        try:
            with open(zones_file, 'r', encoding='utf-8') as f:
                self.zones: Dict[str, Any] = yaml.safe_load(f) or {}
                self.get_logger().info(f'Loaded {len(self.zones)} zones from config.')
        except FileNotFoundError:
            self.get_logger().error(f'Could not find zones.yaml at {zones_file}')
            self.zones = {}

        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_zone: str = 'no_zone'

        # Use TF2 to get robot position in map frame (zones are defined in map frame)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(String, '/current_zone', STATE_QOS)
        self.marker_pub = self.create_publisher(MarkerArray, '/zone_markers', 10)

        self.timer = self.create_timer(0.5, self.update_and_publish_zone)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

    def update_and_publish_zone(self) -> None:
        """
        Look up the robot's pose, resolve its zone, and publish on /current_zone.

        Client: the ROS 2 executor, via the 0.5 s timer.

        @param  (none besides self)
        @pre    The TF tree should contain a map->base_footprint transform. If it
                does not yet (e.g. at start-up, before SLAM/odometry are ready),
                the method tolerates this - see @post.
        @post   - If the transform is available: self.current_x / self.current_y
                  are updated to the looked-up translation and self.current_zone
                  is set to get_zone(current_x, current_y).
                - If the transform is NOT available (LookupException /
                  ConnectivityException / ExtrapolationException): the last known
                  current_x / current_y / current_zone are kept unchanged.
                - In BOTH cases, the current value of self.current_zone is
                  published on /current_zone.
        @return None
        @throws (none) - the three TF exceptions are caught with a narrow
                multi-catch. Any other exception is intentionally NOT caught
                and would propagate as a genuine bug.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self.current_x = transform.transform.translation.x
            self.current_y = transform.transform.translation.y
            self.current_zone = self.get_zone(self.current_x, self.current_y)
        except (LookupException, ConnectivityException, ExtrapolationException):
            # TF not yet available — keep last known zone
            pass

        msg = String()
        msg.data = self.current_zone
        self.publisher.publish(msg)

    def get_zone(self, x: float, y: float) -> str:
        """
        Map a map-frame (x, y) position to the containing zone's name.

        @param x  float - x coordinate in the 'map' frame (metres).
        @param y  float - y coordinate in the 'map' frame (metres).
        @pre   Each zone in self.zones should define numeric 'min_x', 'max_x',
               'min_y', 'max_y'. A missing bound defaults to 0.0, which yields a
               degenerate (empty) box. Ideally the zone boxes do NOT overlap (see
               the tie-break documented in @post).
        @post  Returns the name of the FIRST matching zone when the keys are
               visited in ascending (sorted) order - i.e. the zone whose box
               satisfies (min_x <= x <= max_x) and (min_y <= y <= max_y). If no
               zone matches, returns 'no_zone'. self.zones is not modified
               (this method is read-only).
               Tie-break rule: if several zone boxes overlap the point, the
               alphabetically smallest zone name wins. For example, a point that
               lies inside both 'base_station' and 'zone_2' resolves to
               'base_station'.
        @return str - a zone name that is a key of self.zones, or 'no_zone'.
        """
        for zone_name in sorted(self.zones.keys()):
            zone = self.zones[zone_name] or {}
            if (
                zone.get('min_x', 0.0) <= x <= zone.get('max_x', 0.0)
                and zone.get('min_y', 0.0) <= y <= zone.get('max_y', 0.0)
            ):
                return zone_name
        return 'no_zone'

    def publish_markers(self) -> None:
        """
        Build and publish the RViz visualisation of all zones.

        Draws one translucent CUBE per zone box and one white SPHERE per zone
        target point.

        Client: the ROS 2 executor, via the 1.0 s timer.

        @param  (none besides self)
        @pre    self.zones is populated (an empty dict simply yields an empty
                MarkerArray). self.current_zone reflects the latest detection.
        @post   A MarkerArray is published on /zone_markers containing, for each
                zone in self.zones, exactly two markers:
                  - a CUBE (namespace "zone_boxes", id = index) centred on the
                    box, sized to the box extents (min 0.1 m per side), coloured
                    per the fixed palette; the cube of the zone equal to
                    self.current_zone is drawn more opaque (alpha 0.8) to
                    highlight the robot's location;
                  - a SPHERE (namespace "zone_targets", id = index + 100) at the
                    zone's (target_x, target_y).
                No node state is mutated (read-only with respect to self.zones and
                self.current_zone).
        @return None
        """
        marker_array = MarkerArray()

        colors = {
            'base_station': (0.5, 0.5, 0.5, 0.5),
            'zone_0': (0.0, 1.0, 0.0, 0.4),
            'zone_1': (0.0, 0.0, 1.0, 0.4),
            'zone_2': (1.0, 1.0, 0.0, 0.4),
            'zone_3': (1.0, 0.0, 0.0, 0.4),
        }

        for idx, (zone_name, zone) in enumerate(self.zones.items()):
            min_x = zone.get('min_x', 0.0)
            max_x = zone.get('max_x', 0.0)
            min_y = zone.get('min_y', 0.0)
            max_y = zone.get('max_y', 0.0)
            target_x = zone.get('target_x', 0.0)
            target_y = zone.get('target_y', 0.0)

            cube = Marker()
            cube.header.frame_id = 'map'
            cube.header.stamp = self.get_clock().now().to_msg()
            cube.ns = 'zone_boxes'
            cube.id = idx
            cube.type = Marker.CUBE
            cube.action = Marker.ADD

            cube.pose.position.x = (min_x + max_x) / 2.0
            cube.pose.position.y = (min_y + max_y) / 2.0
            cube.pose.position.z = 0.025

            cube.scale.x = max(0.1, abs(max_x - min_x))
            cube.scale.y = max(0.1, abs(max_y - min_y))
            cube.scale.z = 0.05

            r, g, b, a = colors.get(zone_name, (1.0, 1.0, 1.0, 0.3))
            if zone_name == self.current_zone:
                a = 0.8

            cube.color.r = r
            cube.color.g = g
            cube.color.b = b
            cube.color.a = a

            marker_array.markers.append(cube)

            target = Marker()
            target.header.frame_id = 'map'
            target.header.stamp = self.get_clock().now().to_msg()
            target.ns = 'zone_targets'
            target.id = idx + 100
            target.type = Marker.SPHERE
            target.action = Marker.ADD

            target.pose.position.x = float(target_x)
            target.pose.position.y = float(target_y)
            target.pose.position.z = 0.1

            target.scale.x = 0.2
            target.scale.y = 0.2
            target.scale.z = 0.2

            target.color.r = 1.0
            target.color.g = 1.0
            target.color.b = 1.0
            target.color.a = 0.8

            marker_array.markers.append(target)

        self.marker_pub.publish(marker_array)


def main(args=None) -> None:
    """
    Run the ROS 2 entry point for the zone detector node.

    Initialises the context, spins the node until interrupted, then shuts down
    cleanly.

    @param args  Optional command-line arguments forwarded to rclpy.init().
                 Defaults to None.
    @pre   (none) this function initialises rclpy itself.
    @post  The node has been spun until a KeyboardInterrupt or shutdown, then
           destroyed; rclpy is shut down if the context was still active.
    @return None
    """
    rclpy.init(args=args)
    node = ZoneDetectorNode()
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

#!/usr/bin/env python3
import os
import yaml
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def _package_share_or_source_dir() -> str:
    try:
        return get_package_share_directory('my_turtlebot3_controller')
    except Exception:
        # Support direct workspace execution before the package is installed in the environment.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ZoneDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('zone_detector_node')

        pkg_dir = _package_share_or_source_dir()
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        
        try:
            with open(zones_file, 'r', encoding='utf-8') as f:
                self.zones: Dict[str, Any] = yaml.safe_load(f) or {}
                self.get_logger().info(f"Loaded {len(self.zones)} zones from config.")
        except FileNotFoundError:
            self.get_logger().error(f"Could not find zones.yaml at {zones_file}")
            self.zones = {}

        self.current_x: float = 0.0
        self.current_y: float = 0.0
        self.current_zone: str = 'no_zone'

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.publisher = self.create_publisher(String, '/current_zone', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/zone_markers', 10)
        
        self.timer = self.create_timer(0.5, self.publish_current_zone)
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_zone = self.get_zone(self.current_x, self.current_y)

    def get_zone(self, x: float, y: float) -> str:
        for zone_name in sorted(self.zones.keys()):
            zone = self.zones[zone_name] or {}
            if (
                zone.get('min_x', 0.0) <= x <= zone.get('max_x', 0.0)
                and zone.get('min_y', 0.0) <= y <= zone.get('max_y', 0.0)
            ):
                return zone_name
        return 'no_zone'

    def publish_current_zone(self) -> None:
        msg = String()
        msg.data = self.current_zone
        self.publisher.publish(msg)

    def publish_markers(self) -> None:
        marker_array = MarkerArray()
        
        colors = {
            'base_station': (0.5, 0.5, 0.5, 0.5), # Gray
            'zone_0': (0.0, 1.0, 0.0, 0.4), # Green
            'zone_1': (0.0, 0.0, 1.0, 0.4), # Blue
            'zone_2': (1.0, 1.0, 0.0, 0.4), # Yellow
            'zone_3': (1.0, 0.0, 0.0, 0.4)  # Red
        }

        for idx, (zone_name, zone) in enumerate(self.zones.items()):
            min_x = zone.get('min_x', 0.0)
            max_x = zone.get('max_x', 0.0)
            min_y = zone.get('min_y', 0.0)
            max_y = zone.get('max_y', 0.0)
            target_x = zone.get('target_x', 0.0)
            target_y = zone.get('target_y', 0.0)
            
            # Cube for bounding box
            cube = Marker()
            cube.header.frame_id = "map"
            cube.header.stamp = self.get_clock().now().to_msg()
            cube.ns = "zone_boxes"
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
                a = 0.8 # Highlight current zone
                
            cube.color.r = r
            cube.color.g = g
            cube.color.b = b
            cube.color.a = a
            
            marker_array.markers.append(cube)
            
            # Sphere for target
            target = Marker()
            target.header.frame_id = "map"
            target.header.stamp = self.get_clock().now().to_msg()
            target.ns = "zone_targets"
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

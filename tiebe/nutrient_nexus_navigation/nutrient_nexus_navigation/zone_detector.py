import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
import yaml


def _package_share_or_source_dir() -> str:
    try:
        return get_package_share_directory('nutrient_nexus_navigation')
    except Exception:
        # Support direct workspace execution before the package is installed in the environment.
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class ZoneDetectorNode(Node):
    def __init__(self):
        super().__init__('zone_detector')

        pkg_dir = _package_share_or_source_dir()
        zones_file = os.path.join(pkg_dir, 'config', 'zones.yaml')
        with open(zones_file, 'r', encoding='utf-8') as f:
            self.zones = yaml.safe_load(f) or {}

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_zone = 'no_zone'

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.publisher = self.create_publisher(String, '/current_zone', 10)
        self.timer = self.create_timer(0.5, self.publish_current_zone)

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


def main(args=None):
    rclpy.init(args=args)
    node = ZoneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import math
from typing import List
from sensor_msgs.msg import LaserScan

def get_front_arc_distances(scan_msg: LaserScan, front_angle_deg: float) -> List[float]:
    """Extract LiDAR ranges within the front arc (±front_angle_deg)."""
    ranges = scan_msg.ranges
    angle_min = scan_msg.angle_min
    angle_increment = scan_msg.angle_increment
    front_angle_rad = math.radians(front_angle_deg)

    selected: List[float] = []
    for i, distance in enumerate(ranges):
        angle = angle_min + i * angle_increment
        # Normalize angle to [-pi, pi]
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= front_angle_rad:
            selected.append(distance)

    return selected

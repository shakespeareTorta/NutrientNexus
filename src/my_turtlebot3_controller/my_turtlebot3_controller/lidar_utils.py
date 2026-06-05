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


def sector_min(
    msg: LaserScan, angle_low_deg: float, angle_high_deg: float
) -> float:
    """
    Return the minimum valid range reading in the angular sector
    [angle_low_deg, angle_high_deg] (both in degrees, signed).

    Uses the same angle normalisation as get_front_arc_distances().
    Adapted from OceanDeepSeek's RobotController._sector_min.
    """
    low = math.radians(angle_low_deg)
    high = math.radians(angle_high_deg)

    best = float('inf')
    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))

        if low <= angle <= high:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                if r < best:
                    best = r

    return best


def narrow_object_in_sector(
    msg: LaserScan,
    angle_low_deg: float,
    angle_high_deg: float,
    threshold: float,
) -> bool:
    """
    Return True if ANY single ray in the sector reads closer than threshold.

    Catches thin objects (water bottles, chair legs) that only occupy
    1-3 scan rays and might be missed by a sector-minimum approach with
    a wider threshold.

    Adapted from OceanDeepSeek's RobotController._narrow_object_in_sector.
    """
    low = math.radians(angle_low_deg)
    high = math.radians(angle_high_deg)

    for i, r in enumerate(msg.ranges):
        angle = msg.angle_min + i * msg.angle_increment
        angle = math.atan2(math.sin(angle), math.cos(angle))

        if low <= angle <= high:
            if math.isfinite(r) and msg.range_min < r < threshold:
                return True

    return False

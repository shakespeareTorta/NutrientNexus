import pathlib
import sys

import pytest


PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from nutrient_nexus_navigation.zone_detector import ZoneDetectorNode


class _ZoneDetectorForTest(ZoneDetectorNode):
    def __init__(self, zones):
        self.zones = zones


@pytest.fixture
def detector():
    zones = {
        'zone_a': {'min_x': -1.8, 'max_x': -0.5, 'min_y': -1.8, 'max_y': -0.5},
        'zone_b': {'min_x': 0.5, 'max_x': 1.8, 'min_y': 0.5, 'max_y': 1.8},
        'zone_c': {'min_x': -1.8, 'max_x': -0.5, 'min_y': 0.5, 'max_y': 1.8},
    }
    return _ZoneDetectorForTest(zones)


def test_inside_zone_a(detector):
    assert detector.get_zone(-1.0, -1.0) == 'zone_a'


def test_inside_zone_b(detector):
    assert detector.get_zone(1.0, 1.0) == 'zone_b'


def test_inside_zone_c(detector):
    assert detector.get_zone(-1.0, 1.0) == 'zone_c'


def test_outside_all_zones_returns_no_zone(detector):
    assert detector.get_zone(0.0, 0.0) == 'no_zone'


def test_boundary_points_are_inclusive(detector):
    assert detector.get_zone(-1.8, -0.5) == 'zone_a'
    assert detector.get_zone(1.8, 1.8) == 'zone_b'

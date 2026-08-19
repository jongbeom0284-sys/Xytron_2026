"""LaserScan 구역 요약."""

import math

import numpy as np

from kobot_lane_drive.obstacle_lidar import (
    LidarSectorConfig,
    summarize_scan,
    side_is_occupied,
)

CONFIG = LidarSectorConfig()
COUNT = 720
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / COUNT


def _scan(default=5.0, placements=()):
    """placements 는 (정면 기준 각도[deg], 거리[m]) 목록."""
    ranges = np.full(COUNT, default, dtype=np.float64)
    for degrees, distance in placements:
        corrected = math.radians(degrees) - CONFIG.angle_offset_rad
        corrected = (corrected - ANGLE_MIN) % (2.0 * math.pi)
        index = int(round(corrected / ANGLE_INCREMENT)) % COUNT
        ranges[max(0, index - 4) : index + 5] = distance
    return ranges


def test_object_on_the_left_is_reported_on_the_left():
    ranges = _scan(placements=[(40.0, 0.9)])
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.left_m is not None
    assert sectors.left_m < 1.2
    assert sectors.nearest_side() == "left"


def test_object_on_the_right_is_reported_on_the_right():
    ranges = _scan(placements=[(-40.0, 0.8)])
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.right_m is not None
    assert sectors.right_m < 1.2
    assert sectors.nearest_side() == "right"


def test_object_straight_ahead_lands_in_the_front_sector():
    ranges = _scan(placements=[(0.0, 0.7)])
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.front_m is not None
    assert sectors.front_m < 1.0


def test_body_reflections_and_far_returns_are_dropped():
    ranges = np.full(COUNT, 0.05, dtype=np.float64)
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.front_m is None and sectors.left_m is None and sectors.right_m is None

    ranges = np.full(COUNT, 20.0, dtype=np.float64)
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.left_m is None


def test_single_noise_spike_does_not_dominate_the_sector():
    ranges = _scan(default=3.0)
    corrected = math.radians(40.0) - CONFIG.angle_offset_rad
    index = int(round(((corrected - ANGLE_MIN) % (2.0 * math.pi)) / ANGLE_INCREMENT)) % COUNT
    ranges[index] = 0.3
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 10.0, CONFIG)
    assert sectors.left_m is not None
    assert sectors.left_m > 1.0


def test_stale_scan_is_refused_as_evidence():
    ranges = _scan(placements=[(40.0, 0.6)])
    sectors = summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, 100.0, CONFIG)
    assert side_is_occupied(sectors, "left", 1.5, now=100.1, maximum_age_sec=0.5)
    assert not side_is_occupied(sectors, "left", 1.5, now=105.0, maximum_age_sec=0.5)


def test_empty_scan_is_handled():
    sectors = summarize_scan([], ANGLE_MIN, ANGLE_INCREMENT, 1.0, CONFIG)
    assert sectors.nearest_side() is None
    assert not side_is_occupied(sectors, "left", 1.5, now=1.0, maximum_age_sec=0.5)

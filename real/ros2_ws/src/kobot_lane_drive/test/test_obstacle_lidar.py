"""라이다 코리도 요약.

차량은 항상 차선 안에 있으므로, 옆 차선 장애물은 "앞으로 y m, 옆으로 x m" 상자
안에 들어온다. 벽과 연석은 그 상자 밖에 있어 후보가 되지 않아야 한다.
"""

import math

import numpy as np

from kobot_lane_drive.obstacle_lidar import (
    LidarSectorConfig,
    scan_to_vehicle_points,
    side_is_occupied,
    summarize_scan,
)

CONFIG = LidarSectorConfig()
COUNT = 720
ANGLE_MIN = -math.pi
ANGLE_INCREMENT = 2.0 * math.pi / COUNT


def _blank():
    return np.full(COUNT, float("inf"), dtype=np.float64)


def _place(ranges, x_m, y_m, spread=6):
    """차량 좌표 (x 왼쪽+, y 앞쪽+) 에 물체 한 덩이를 놓는다."""
    distance = math.hypot(x_m, y_m)
    theta = math.atan2(x_m, y_m)
    raw = theta - CONFIG.angle_offset_rad
    index = int(round(((raw - ANGLE_MIN) % (2.0 * math.pi)) / ANGLE_INCREMENT)) % COUNT
    for offset in range(-spread, spread + 1):
        ranges[(index + offset) % COUNT] = distance
    return ranges


def _summary(ranges, stamp=10.0):
    return summarize_scan(ranges, ANGLE_MIN, ANGLE_INCREMENT, stamp, CONFIG)


def test_coordinates_put_left_objects_on_the_positive_x_axis():
    x, y = scan_to_vehicle_points(
        _place(_blank(), 0.5, 1.0), ANGLE_MIN, ANGLE_INCREMENT, CONFIG
    )
    assert x.size > 0
    assert x.max() > 0.3
    assert y.max() > 0.8


def test_car_in_the_left_lane_is_detected_on_the_left():
    sectors = _summary(_place(_blank(), 0.45, 1.0))
    assert sectors.left_m is not None
    assert 0.8 < sectors.left_m < 1.2
    assert sectors.right_m is None
    assert sectors.nearest_side() == "left"


def test_car_in_the_right_lane_is_detected_on_the_right():
    sectors = _summary(_place(_blank(), -0.45, 1.0))
    assert sectors.right_m is not None
    assert sectors.left_m is None
    assert sectors.nearest_side() == "right"


def test_car_straight_ahead_lands_in_the_front_corridor():
    sectors = _summary(_place(_blank(), 0.0, 0.9))
    assert sectors.front_m is not None
    assert 0.7 < sectors.front_m < 1.1


def test_wall_beside_the_vehicle_is_outside_the_corridor():
    """도로 밖의 벽은 옆으로 너무 멀어 후보가 되지 않는다."""
    ranges = _blank()
    for forward in np.arange(0.3, 2.6, 0.05):
        _place(ranges, 1.4, float(forward), spread=1)
    sectors = _summary(ranges)
    assert sectors.left_m is None


def test_object_right_next_to_the_bumper_is_ignored():
    """앞쪽 거리가 모자란 물체는 옆 차선 차량이 아니다."""
    sectors = _summary(_place(_blank(), 0.4, 0.10))
    assert sectors.left_m is None


def test_object_beyond_the_forward_window_is_ignored():
    sectors = _summary(_place(_blank(), 0.4, 4.0))
    assert sectors.left_m is None


def test_body_reflections_are_dropped():
    sectors = _summary(np.full(COUNT, 0.05, dtype=np.float64))
    assert sectors.front_m is None and sectors.left_m is None and sectors.right_m is None


def test_single_noise_spike_does_not_create_an_obstacle():
    ranges = _blank()
    _place(ranges, 0.45, 1.0, spread=1)  # 점 3개뿐 -> minimum_hits 미달
    sectors = _summary(ranges)
    assert sectors.left_m is None


def test_stale_scan_is_refused_as_evidence():
    sectors = _summary(_place(_blank(), 0.45, 0.9), stamp=100.0)
    assert side_is_occupied(sectors, "left", 1.5, now=100.1, maximum_age_sec=0.5)
    assert not side_is_occupied(sectors, "left", 1.5, now=105.0, maximum_age_sec=0.5)


def test_empty_scan_is_handled():
    sectors = summarize_scan([], ANGLE_MIN, ANGLE_INCREMENT, 1.0, CONFIG)
    assert sectors.nearest_side() is None
    assert not side_is_occupied(sectors, "left", 1.5, now=1.0, maximum_age_sec=0.5)

"""라바콘 통로(Cone Corridor) 주행 알고리즘 단위 시험."""

import math
import numpy as np
import pytest

from kobot_lane_drive.cone_corridor import (
    ConeCorridorConfig,
    analyze_cone_corridor,
    compute_steering_pure_pursuit,
    extract_corridor_slices,
)
from kobot_lane_drive.obstacle_lidar import LidarSectorConfig


def _make_mock_scan(cones_xy: list[tuple[float, float]], angle_min: float = -math.pi, angle_inc: float = math.pi / 180.0, num_readings: int = 360) -> list[float]:
    """차량 좌표계 (x 좌+, y 앞+) 목표 점들로부터 역산한 합성 LaserScan ranges 생성 (실제 콘 반사 모델)."""
    ranges = [float("inf")] * num_readings
    cfg = LidarSectorConfig()

    for cx, cy in cones_xy:
        # 콘 직경 10cm 모델 (반지름 0.04m 호, 점 5개 반사)
        for offset in [-0.04, -0.02, 0.0, 0.02, 0.04]:
            x = cx + offset
            y = cy
            dist = math.hypot(x, y)
            theta_veh = math.atan2(x, y)
            theta_raw = theta_veh - cfg.angle_offset_rad
            theta_norm = math.atan2(math.sin(theta_raw), math.cos(theta_raw))
            idx = int(round((theta_norm - angle_min) / angle_inc))
            if 0 <= idx < num_readings:
                if dist < ranges[idx]:
                    ranges[idx] = dist
    return ranges


def test_empty_scan_returns_empty_result():
    ranges = [float("inf")] * 360
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0)
    assert not res.found
    assert res.steering_rad == 0.0
    assert res.speed_mps == 0.0
    assert res.confidence == 0.0
    assert res.primary_side == "NONE"


def test_straight_corridor_centers_at_zero():
    """폭 0.8m 직진 통로 (좌측 x=+0.4m, 우측 x=-0.4m) 주행 검증."""
    # A4 1장 간격(~0.30m)으로 y 0.3m부터 1.8m까지 콘 배치
    points = []
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((0.40, float(y)))   # 좌측 콘
        points.append((-0.40, float(y)))  # 우측 콘

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(target_lookahead_m=0.48)
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert res.found
    assert res.primary_side == "BOTH"
    assert abs(res.target_x_m) < 0.05  # 중심 x가 거의 0
    assert abs(res.steering_rad) < 0.05  # 직진 조향
    assert res.speed_mps == config.drive_speed_mps


def test_s_curve_corridor_steers_correctly():
    """S자 곡선 통로에서 중심 궤적 및 조향각 반응 검증."""
    # 좌우로 약간 휘어지는 S자 형태 (중심선 x = 0.15m 로 치우침)
    points = []
    offset = 0.15
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((0.40 + offset, float(y)))
        points.append((-0.40 + offset, float(y)))

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(target_lookahead_m=0.48)
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert res.found
    assert res.primary_side == "BOTH"
    assert res.target_x_m > 0.10  # 좌측으로 치우친 목표점
    assert res.steering_rad > 0.0  # 좌조향(+) 발생


def test_sharp_left_turn_single_wall_fallback():
    """90도 급좌회전 시 안쪽 콘이 가려지고 우측(외측) 벽만 보일 때의 단측 추종 검증."""
    # 우측 콘 벽만 x = -0.40m 에 존재
    points = []
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((-0.40, float(y)))

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(
        target_lookahead_m=0.48,
        single_wall_fallback_margin_m=0.40,
    )
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert res.found
    assert res.primary_side == "RIGHT_ONLY"
    # 우측 벽(-0.4m) + 오프셋(0.4m) = 가상 중심 0.0m 부근
    assert abs(res.target_x_m - 0.0) < 0.05


def test_abnormal_wide_width_gating_rejects_far_wall():
    """좌측에 먼 벽(x=+1.1m)이 있고 우측에 진짜 콘(x=-0.4m)이 있을 때 게이팅 검증."""
    points = []
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((1.10, float(y)))   # 먼 벽/책상
        points.append((-0.40, float(y)))  # 진짜 우측 콘

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(
        target_lookahead_m=0.48,
        corridor_width_min_m=0.55,
        corridor_width_max_m=1.05,
        single_wall_fallback_margin_m=0.40,
    )
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert res.found
    # 먼 벽(1.1m)을 버리고 우측 콘(-0.4m) 기준으로 중심(0.0m) 형성
    assert abs(res.target_x_m - 0.0) < 0.08


def test_far_points_filtered_by_lateral_max():
    """복도 벽이나 외부 장애물(x > 1.2m)이 라바콘 통로로 오인되지 않음 검증."""
    points = []
    # 통로 외부 2.0m 떨어진 벽
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((2.0, float(y)))

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(lateral_max_m=1.20)
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert not res.found


def test_speed_reduction_on_sharp_steering():
    """급커브로 조향각이 크게 꺾일 때 코너 감속 속도 적용 검증."""
    # 극단적으로 좌측으로 꺾인 목표점
    points = []
    for y in np.arange(0.3, 1.9, 0.30):
        points.append((0.70, float(y)))
        points.append((-0.10, float(y)))

    ranges = _make_mock_scan(points)
    config = ConeCorridorConfig(
        target_lookahead_m=0.48,
        drive_speed_mps=0.25,
        corner_speed_mps=0.20,
    )
    res = analyze_cone_corridor(ranges, -math.pi, math.pi / 180.0, config=config)

    assert res.found
    assert res.steering_rad > (config.max_steering_rad * 0.5)
    assert res.speed_mps == config.corner_speed_mps


def test_pure_pursuit_calculation():
    """Pure pursuit 계산 함수의 부호 및 클램프 검증."""
    # 목표가 정면 (x=0) 이면 조향 0
    assert compute_steering_pure_pursuit(0.0, 1.0, 0.33, 0.45) == 0.0

    # 목표가 좌측 (x>0) 이면 조향 양수(+)
    steer_left = compute_steering_pure_pursuit(0.3, 0.7, 0.33, 0.45)
    assert steer_left > 0.0

    # 목표가 우측 (x<0) 이면 조향 음수(-)
    steer_right = compute_steering_pure_pursuit(-0.3, 0.7, 0.33, 0.45)
    assert steer_right < 0.0

    # 극단적인 오차 시 max_steering_rad 로 클램프
    clamped = compute_steering_pure_pursuit(0.5, 0.2, 0.33, 0.45)
    assert clamped == 0.45

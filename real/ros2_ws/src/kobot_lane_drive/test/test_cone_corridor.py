"""라바콘 통로(Cone Corridor) 주행 알고리즘 단위 시험."""

import math
import numpy as np
import pytest

from kobot_lane_drive.cone_corridor import (
    ConeCorridorConfig,
    CorridorSlice,
    _select_single_boundary,
    analyze_cone_corridor,
    adaptive_cone_link_distance,
    chain_cone_boundary,
    chain_paired_boundaries,
    compute_adaptive_lookahead,
    compute_steering_pure_pursuit,
    compute_target_point,
    estimate_local_centerline_heading,
    extract_corridor_slices,
    fit_polynomial_centerline,
    interpolate_polyline_x_at_y,
    select_centerline_target,
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
        points.append((0.64, float(y)))
        points.append((-0.16, float(y)))

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


def test_lidar_target_is_translated_to_rear_axle_before_steering():
    """앞 라이다의 목표점을 뒷축 기준으로 옮겨 같은 물리점을 과조향하지 않는다."""
    points = []
    for y_m in np.arange(0.30, 1.51, 0.30):
        points.append((0.55, float(y_m)))
        points.append((-0.25, float(y_m)))
    ranges = _make_mock_scan(points)
    common = dict(
        target_lookahead_m=0.48,
        corner_target_lookahead_m=0.48,
        forward_max_m=1.60,
        lateral_max_m=1.00,
        cone_min_cluster_points=2,
        detection_min_slices=2,
        detection_min_total_points=3,
    )

    lidar_origin = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(**common, lidar_to_rear_axle_m=0.0),
    )
    rear_axle = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(**common, lidar_to_rear_axle_m=0.42),
    )

    assert lidar_origin.found and rear_axle.found
    assert rear_axle.target_x_m == pytest.approx(lidar_origin.target_x_m)
    assert rear_axle.target_y_m == pytest.approx(lidar_origin.target_y_m)
    assert lidar_origin.control_target_y_m == pytest.approx(lidar_origin.target_y_m)
    assert rear_axle.control_target_y_m == pytest.approx(
        rear_axle.target_y_m + 0.42
    )
    assert abs(rear_axle.steering_rad) < abs(lidar_origin.steering_rad)


@pytest.mark.parametrize("center_offset", [0.15, -0.15])
def test_curve_target_is_shifted_ten_centimeters_to_turn_outside(center_offset):
    points = []
    for y_m in np.arange(0.30, 1.51, 0.30):
        points.append((center_offset + 0.40, float(y_m)))
        points.append((center_offset - 0.40, float(y_m)))
    result = analyze_cone_corridor(
        _make_mock_scan(points),
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(
            target_lookahead_m=0.48,
            corner_target_lookahead_m=0.48,
            forward_max_m=1.60,
            lateral_max_m=1.00,
            cone_min_cluster_points=2,
            detection_min_slices=2,
            detection_min_total_points=3,
            lidar_to_rear_axle_m=0.42,
            curve_outward_bias_max_m=0.10,
            curve_outward_bias_start_ratio=0.0,
            curve_outward_bias_full_ratio=0.01,
            vehicle_width_m=0.35,
            minimum_boundary_clearance_m=0.05,
        ),
    )

    assert result.found
    assert result.curve_outward_bias_m == pytest.approx(0.10)
    assert abs(result.target_x_m) < abs(result.centerline_target_x_m)
    assert math.copysign(1.0, result.target_x_m) == math.copysign(
        1.0,
        result.centerline_target_x_m,
    )


def test_curve_outward_bias_respects_vehicle_clearance_in_narrow_corridor():
    points = []
    for y_m in np.arange(0.30, 1.51, 0.30):
        points.append((0.45, float(y_m)))
        points.append((-0.15, float(y_m)))
    result = analyze_cone_corridor(
        _make_mock_scan(points),
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(
            corridor_width_m=0.60,
            corridor_width_min_m=0.55,
            corridor_width_max_m=0.70,
            target_lookahead_m=0.48,
            corner_target_lookahead_m=0.48,
            forward_max_m=1.60,
            lateral_max_m=1.00,
            cone_min_cluster_points=2,
            detection_min_slices=2,
            detection_min_total_points=3,
            curve_outward_bias_max_m=0.10,
            curve_outward_bias_start_ratio=0.0,
            curve_outward_bias_full_ratio=0.01,
            vehicle_width_m=0.35,
            minimum_boundary_clearance_m=0.05,
        ),
    )

    assert result.found
    assert result.corridor_width_est_m == pytest.approx(0.60, abs=0.03)
    assert result.curve_outward_bias_m == pytest.approx(0.075, abs=0.015)


@pytest.mark.parametrize(
    ("center_offset_m", "expected_bias_m"),
    [
        (0.22, 0.20),
        (-0.22, 0.15),
    ],
)
def test_curve_outward_bias_uses_direction_specific_maximum(
    center_offset_m, expected_bias_m
):
    points = []
    for y_m in np.arange(0.30, 1.51, 0.30):
        points.append((0.40 + center_offset_m, float(y_m)))
        points.append((-0.40 + center_offset_m, float(y_m)))
    result = analyze_cone_corridor(
        _make_mock_scan(points),
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(
            target_lookahead_m=0.48,
            corner_target_lookahead_m=0.48,
            forward_max_m=1.60,
            lateral_max_m=1.00,
            cone_min_cluster_points=2,
            detection_min_slices=2,
            detection_min_total_points=3,
            curve_outward_bias_max_m=0.10,
            curve_outward_bias_left_max_m=0.20,
            curve_outward_bias_right_max_m=0.15,
            curve_outward_bias_start_ratio=0.0,
            curve_outward_bias_full_ratio=0.01,
            vehicle_width_m=0.35,
            minimum_boundary_clearance_m=0.0,
            corridor_width_m=0.80,
        ),
    )
    assert result.found
    assert result.curve_outward_bias_m == pytest.approx(expected_bias_m)


def test_straight_centerline_has_no_curve_outward_bias():
    points = []
    for y_m in np.arange(0.30, 1.51, 0.30):
        points.append((0.40, float(y_m)))
        points.append((-0.40, float(y_m)))
    result = analyze_cone_corridor(
        _make_mock_scan(points),
        -math.pi,
        math.pi / 180.0,
        config=ConeCorridorConfig(
            target_lookahead_m=0.48,
            corner_target_lookahead_m=0.48,
            forward_max_m=1.60,
            lateral_max_m=1.00,
            cone_min_cluster_points=2,
            detection_min_slices=2,
            detection_min_total_points=3,
            curve_outward_bias_max_m=0.10,
        ),
    )

    assert result.found
    assert result.curve_outward_bias_m == pytest.approx(0.0)
    assert result.target_x_m == pytest.approx(result.centerline_target_x_m)


def test_isolated_corner_cluster_cannot_create_boundary():
    """라바콘 형상의 벽 모서리 하나는 유효 슬라이스를 만들어도 경로가 아니다."""
    ranges = _make_mock_scan([(0.35, 0.55)])
    config = ConeCorridorConfig(
        detection_min_slices=1,
        detection_min_total_points=2,
        minimum_boundary_cones=2,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert not result.found
    assert result.left_candidates_count == 1
    assert result.left_cones_count == 0
    assert result.left_points_count > 0


def test_single_cross_corridor_pair_is_not_enough_to_start_path():
    ranges = _make_mock_scan([(0.40, 0.45), (-0.40, 0.45)])
    config = ConeCorridorConfig(
        target_lookahead_m=0.65,
        detection_min_slices=2,
        detection_min_total_points=2,
        minimum_boundary_cones=2,
        minimum_paired_boundary_pairs=2,
        drive_speed_mps=0.80,
        corner_speed_mps=0.45,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert not result.found
    assert result.primary_side == "NONE"


def test_two_connected_cones_allow_single_wall_fallback():
    """연결된 실제 콘 두 개는 단측 경계로 계속 추종한다."""
    ranges = _make_mock_scan([(0.40, 0.35), (0.40, 0.65)])
    config = ConeCorridorConfig(
        detection_min_slices=1,
        detection_min_total_points=2,
        minimum_boundary_cones=2,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert result.found
    assert result.primary_side == "LEFT_ONLY"
    assert result.left_cones_count == 2


def test_adaptive_link_keeps_near_limit_but_allows_sparse_far_cones():
    assert adaptive_cone_link_distance(0.20, 0.35, 0.50, 0.30, 0.75) == pytest.approx(0.35)
    assert adaptive_cone_link_distance(0.75, 0.35, 0.50, 0.30, 0.75) == pytest.approx(0.50)

    chain = chain_cone_boundary(
        [(0.40, 0.30)],
        [(0.40, 0.30), (0.40, 0.70), (0.42, 1.15)],
        max_link_distance_m=0.35,
        far_max_link_distance_m=0.50,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.75,
    )

    assert len(chain) == 3


def test_single_wall_chain_can_restart_from_first_visible_far_cone():
    ranges = _make_mock_scan([(-0.22, 0.84), (-0.07, 1.22)])
    config = ConeCorridorConfig(
        target_lookahead_m=0.90,
        root_maximum_y_m=0.95,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.50,
        detection_min_slices=2,
        detection_min_total_points=3,
        cone_min_cluster_points=2,
        cone_min_cluster_span_m=0.03,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert result.found
    assert result.primary_side == "RIGHT_ONLY"
    assert result.right_cones_count == 2


def test_adaptive_link_does_not_attach_near_wall_corner():
    chain = chain_cone_boundary(
        [(0.08, 0.12), (0.42, 0.30)],
        [(0.08, 0.12), (0.42, 0.30), (0.42, 0.70)],
        max_link_distance_m=0.35,
        far_max_link_distance_m=0.50,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.75,
    )

    assert chain[0] == (0.42, 0.30)


def test_smooth_boundary_beats_linkable_wall_corner_prefix():
    """45cm 안에서 실제 체인에 붙는 벽 모서리도 경계 시작점이 되지 않는다."""
    isolated = (0.10, 0.15)
    real_boundary = [(0.45, 0.30), (0.45, 0.60), (0.45, 0.90)]
    chain = chain_cone_boundary(
        [isolated, real_boundary[0]],
        [isolated, *real_boundary],
        max_link_distance_m=0.45,
    )

    assert chain == real_boundary


def test_paired_boundaries_keep_left_and_right_cones_disjoint_on_curve():
    """굽은 통로에서도 두 경계를 동시에 연결하며 반대편 콘을 훔치지 않는다."""
    left = [(0.40, 0.30), (0.48, 0.65), (0.65, 1.00), (0.86, 1.35)]
    right = [(-0.40, 0.30), (-0.32, 0.65), (-0.15, 1.00), (0.06, 1.35)]
    noise = [(0.08, 0.18), (0.92, 0.72)]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        max_cone_link_distance_m=0.45,
        minimum_boundary_cones=2,
    )

    left_chain, right_chain = chain_paired_boundaries(
        [noise[0], *left, noise[1], *right],
        config,
    )

    assert left_chain == left
    assert right_chain == right
    assert set(left_chain).isdisjoint(right_chain)


def test_paired_boundaries_reject_diagonal_cross_pairs():
    """다음 칸의 대각선 콘을 같은 통로 단면으로 잘못 짝짓지 않는다."""
    left = [(0.40, 0.30), (0.46, 0.70), (0.58, 1.10)]
    right = [(-0.40, 0.30), (-0.34, 0.70), (-0.22, 1.10)]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        max_cone_link_distance_m=0.45,
    )

    left_chain, right_chain = chain_paired_boundaries([*left, *right], config)

    assert left_chain == left
    assert right_chain == right


def test_staggered_photo_layout_forms_two_disjoint_boundaries():
    """사진처럼 좌우 콘 y가 엇갈리고 간격이 50cm여도 같은 통로로 연결한다."""
    left = [(0.40, 0.30), (0.46, 0.82), (0.60, 1.34), (0.82, 1.78)]
    right = [(-0.40, 0.52), (-0.34, 1.04), (-0.20, 1.56), (0.02, 1.92)]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
        pair_maximum_longitudinal_offset_m=0.38,
        root_maximum_y_m=0.75,
        minimum_paired_boundary_pairs=2,
    )

    left_chain, right_chain = chain_paired_boundaries([*left, *right], config)

    assert left_chain == left
    assert right_chain == right
    assert set(left_chain).isdisjoint(right_chain)
    for boundary in (left_chain, right_chain):
        for first, second in zip(boundary, boundary[1:]):
            midpoint_y = 0.5 * (first[1] + second[1])
            assert interpolate_polyline_x_at_y(
                boundary,
                midpoint_y,
                config.max_cone_link_distance_m,
                config.boundary_extrapolation_m,
                config.far_max_cone_link_distance_m,
                config.link_distance_far_start_m,
                config.link_distance_far_full_m,
            ) is not None


def test_current_corridor_anchor_beats_longer_same_side_clutter_rows():
    """의자 다리 두 열이 더 길어도 현재 차체를 가로지르는 실제 통로를 고른다."""
    actual_left = [(0.40, 0.30), (0.45, 0.70), (0.55, 1.10)]
    actual_right = [(-0.40, 0.30), (-0.35, 0.70), (-0.25, 1.10)]
    clutter_a = [(0.20, 0.25), (0.25, 0.65), (0.30, 1.05), (0.35, 1.45)]
    clutter_b = [(0.90, 0.25), (0.95, 0.65), (1.00, 1.05), (1.05, 1.45)]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.58,
        link_distance_far_full_m=0.60,
        minimum_paired_boundary_pairs=2,
    )

    left_chain, right_chain = chain_paired_boundaries(
        [*clutter_a, *clutter_b, *actual_left, *actual_right],
        config,
    )

    assert left_chain == actual_left
    assert right_chain == actual_right


def test_single_wall_root_rejects_far_lateral_furniture_row():
    ranges = _make_mock_scan([(0.90, 0.25), (0.92, 0.65), (0.94, 1.05)])
    config = ConeCorridorConfig(
        lateral_max_m=1.0,
        boundary_root_lateral_tolerance_m=0.25,
        detection_min_slices=1,
        detection_min_total_points=2,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert not result.found


def test_single_boundary_fallback_is_limited_to_corner_speed():
    ranges = _make_mock_scan([(0.40, 0.30), (0.42, 0.65), (0.48, 1.00)])
    config = ConeCorridorConfig(
        drive_speed_mps=0.65,
        corner_speed_mps=0.45,
        fallback_speed_mps=0.50,
        detection_min_slices=1,
    )

    result = analyze_cone_corridor(
        ranges,
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert result.found
    assert result.primary_side == "LEFT_ONLY"
    assert result.speed_mps == pytest.approx(0.50)


def _slice(y: float, x: float, interpolated: bool = False) -> CorridorSlice:
    return CorridorSlice(
        y_m=y,
        left_x_m=None if interpolated else x + 0.40,
        right_x_m=None if interpolated else x - 0.40,
        center_x_m=x,
        is_interpolated=interpolated,
    )


def test_adaptive_lookahead_stays_long_on_straight_path():
    slices = [_slice(y, 0.0) for y in np.arange(0.25, 1.10, 0.10)]

    lookahead, turn_ratio = compute_adaptive_lookahead(
        slices,
        straight_lookahead_m=0.72,
        corner_lookahead_m=0.50,
        bend_start_m=0.08,
        bend_full_m=0.28,
    )

    assert lookahead == pytest.approx(0.72)
    assert turn_ratio == pytest.approx(0.0)


def test_adaptive_lookahead_shortens_for_tight_bend():
    slices = [
        _slice(0.35, 0.00),
        _slice(0.45, 0.02),
        _slice(0.80, 0.30),
        _slice(0.90, 0.34),
    ]

    lookahead, turn_ratio = compute_adaptive_lookahead(
        slices,
        straight_lookahead_m=0.72,
        corner_lookahead_m=0.50,
        bend_start_m=0.08,
        bend_full_m=0.28,
    )

    assert lookahead == pytest.approx(0.50)
    assert turn_ratio == pytest.approx(1.0)


def test_polynomial_fit_recovers_quadratic_centerline():
    slices = [
        _slice(float(y), 0.20 * float(y) ** 2 - 0.03)
        for y in np.arange(0.25, 1.51, 0.10)
    ]

    model = fit_polynomial_centerline(slices)

    assert model is not None
    assert model.coefficient_a == pytest.approx(0.20, abs=0.01)
    assert model.coefficient_b == pytest.approx(0.0, abs=0.02)
    assert model.coefficient_c == pytest.approx(-0.03, abs=0.01)
    assert model.x_at(0.90) == pytest.approx(0.132, abs=0.01)
    assert model.rmse_m < 0.01


def test_polynomial_fit_rejects_centerline_outlier():
    slices = [
        _slice(float(y), 0.12 * float(y) ** 2)
        for y in np.arange(0.25, 1.51, 0.10)
    ]
    slices[7] = _slice(slices[7].y_m, slices[7].center_x_m + 0.45)

    model = fit_polynomial_centerline(slices)

    assert model is not None
    assert model.points_used == len(slices) - 1
    assert model.x_at(0.90) == pytest.approx(0.12 * 0.90**2, abs=0.02)


def test_polynomial_fit_ignores_single_wall_virtual_centers():
    slices = [
        _slice(float(y), 0.10, interpolated=True)
        for y in np.arange(0.25, 1.51, 0.10)
    ]

    assert fit_polynomial_centerline(slices) is None


def test_true_s_curve_keeps_local_bend_when_global_poly2_reverses_sign():
    """한 ROI 안의 다음 역커브가 현재 커브 조향 부호를 뒤집지 못한다."""
    y_values = np.arange(0.25, 1.61, 0.10)
    x_values = 0.21 * np.sin(
        2.0 * math.pi * 1.5 * (y_values - 0.25) / 1.35 - 0.14
    )
    slices = [
        _slice(float(y_m), float(x_m))
        for y_m, x_m in zip(y_values, x_values)
    ]
    polynomial = fit_polynomial_centerline(slices, maximum_rmse_m=0.08)
    local_x, _, _ = compute_target_point(slices, 0.62, 0.40, 0.15)

    assert polynomial is not None
    assert local_x > 0.08
    assert polynomial.x_at(0.62) < 0.0
    selected_x, model = select_centerline_target(
        local_x,
        0.62,
        turn_ratio=1.0,
        polynomial=polynomial,
    )

    assert model == "LOCAL"
    assert selected_x == pytest.approx(local_x)
    assert math.isfinite(estimate_local_centerline_heading(slices, 0.62))


def test_centerline_curvature_shortens_dynamic_lookahead():
    slices = [_slice(y, 0.0) for y in np.arange(0.25, 1.50, 0.10)]

    lookahead, turn_ratio = compute_adaptive_lookahead(
        slices,
        straight_lookahead_m=0.90,
        corner_lookahead_m=0.60,
        bend_start_m=0.08,
        bend_full_m=0.28,
        centerline_curvature_1pm=1.0,
        curvature_start_1pm=0.15,
        curvature_full_1pm=0.90,
    )

    assert lookahead == pytest.approx(0.60)
    assert turn_ratio == pytest.approx(1.0)


def test_extended_far_window_detects_corner_before_near_path_turns():
    slices = [
        _slice(0.35, 0.00),
        _slice(0.55, 0.01),
        _slice(1.10, 0.29),
        _slice(1.30, 0.34),
    ]

    lookahead, turn_ratio = compute_adaptive_lookahead(
        slices,
        straight_lookahead_m=0.85,
        corner_lookahead_m=0.60,
        bend_start_m=0.08,
        bend_full_m=0.28,
        near_min_m=0.30,
        near_max_m=0.60,
        far_min_m=1.05,
        far_max_m=1.45,
    )

    assert lookahead == pytest.approx(0.60)
    assert turn_ratio == pytest.approx(1.0)


def test_local_target_rejects_single_lateral_outlier():
    slices = [
        _slice(0.60, 0.04),
        _slice(0.70, 0.05),
        _slice(0.72, 0.55),
        _slice(0.80, 0.06),
    ]

    target_x, target_y, confidence = compute_target_point(
        slices,
        target_lookahead_m=0.72,
        fallback_margin_m=0.40,
        local_window_m=0.15,
    )

    assert target_x < 0.10
    assert target_y == pytest.approx(0.7101, abs=0.002)
    assert confidence > 0.0


def test_local_target_prefers_two_sided_slices_over_single_wall_fallback():
    slices = [
        _slice(0.62, 0.02),
        _slice(0.70, 0.03),
        _slice(0.74, 0.35, interpolated=True),
        _slice(0.80, 0.04),
    ]

    target_x, _, _ = compute_target_point(
        slices,
        target_lookahead_m=0.72,
        fallback_margin_m=0.40,
        local_window_m=0.15,
    )

    assert target_x < 0.10


def test_local_target_does_not_reuse_distant_path_segment():
    slices = [_slice(0.30, 0.10), _slice(0.40, 0.12)]

    _, _, confidence = compute_target_point(
        slices,
        target_lookahead_m=0.72,
        fallback_margin_m=0.40,
        local_window_m=0.15,
    )

    assert confidence == 0.0


def test_local_target_uses_actual_supported_y_for_sharp_near_curve():
    """가까운 경계에서 얻은 x를 먼 lookahead y와 조합해 조향을 약화하지 않는다."""
    slices = [_slice(0.30, 0.10), _slice(0.40, 0.12)]

    target_x, target_y, confidence = compute_target_point(
        slices,
        target_lookahead_m=0.62,
        fallback_margin_m=0.40,
        local_window_m=0.15,
    )
    supported_steering = compute_steering_pure_pursuit(
        target_x,
        target_y,
        wheelbase_m=0.33,
        max_steering_rad=0.45,
        steering_gain=1.9,
    )
    old_mismatched_steering = compute_steering_pure_pursuit(
        target_x,
        0.62,
        wheelbase_m=0.33,
        max_steering_rad=0.45,
        steering_gain=1.9,
    )

    assert confidence > 0.0
    assert target_y == pytest.approx(0.40)
    assert abs(supported_steering) > abs(old_mismatched_steering)


def test_single_boundary_does_not_step_sideways_into_near_same_y_wall_corner():
    """차량 옆 모서리가 25cm 안에 있어도 실제 전방 콘 열의 첫 점을 훔치지 않는다."""
    wall_corner = (0.16, 0.27)
    actual_boundary = [(0.40, 0.30), (0.45, 0.65), (0.55, 1.00)]

    chain = chain_cone_boundary(
        [wall_corner, actual_boundary[0]],
        [wall_corner, *actual_boundary],
        max_link_distance_m=0.35,
        far_max_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
    )

    assert chain == actual_boundary


def test_single_outer_wall_keeps_identity_after_crossing_vehicle_centerline():
    """급커브 외벽이 x=0을 넘어도 root가 정한 RIGHT 경계를 끝까지 유지한다."""
    right_outer_wall = [
        (-0.40, 0.25),
        (-0.30, 0.55),
        (-0.10, 0.80),
        (+0.15, 1.05),
        (+0.40, 1.30),
    ]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        forward_min_m=0.10,
        forward_max_m=2.00,
        lateral_max_m=1.00,
        cone_min_cluster_points=2,
        cone_min_cluster_span_m=0.03,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
        minimum_boundary_cones=2,
        root_maximum_y_m=0.75,
        detection_min_slices=2,
        detection_min_total_points=3,
        target_lookahead_m=0.72,
        corner_target_lookahead_m=0.72,
    )

    result = analyze_cone_corridor(
        _make_mock_scan(right_outer_wall),
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert result.found
    assert result.primary_side == "RIGHT_ONLY"
    assert result.boundary_mode == "RIGHT_FALLBACK"
    assert any(x_m > 0.0 for x_m, _ in result.right_boundary_xy)
    assert max(y_m for _, y_m in result.right_boundary_xy) >= 1.0
    assert result.target_x_m > 0.10


def test_single_boundary_prefers_nominal_anchor_over_long_same_side_clutter():
    """더 멀리 이어지는 같은 쪽 가구 열보다 차폭 반쪽 위치의 경계를 고른다."""
    actual_boundary = [
        (0.40, 0.30),
        (0.45, 0.65),
        (0.55, 1.00),
        (0.70, 1.35),
    ]
    same_side_clutter = [
        (0.16, 0.25),
        (0.16, 0.65),
        (0.16, 1.05),
        (0.16, 1.45),
    ]

    chain = chain_cone_boundary(
        [same_side_clutter[0], actual_boundary[0]],
        [*same_side_clutter, *actual_boundary],
        max_link_distance_m=0.35,
        far_max_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
        preferred_root_x_m=0.40,
    )

    assert chain == actual_boundary


def test_paired_root_prefers_true_cross_section_over_staggered_corner_pair():
    """같은 길이 체인이라면 폭/종방향 오차가 작은 실제 입구 단면을 유지한다."""
    actual_left = [(0.40, 0.30), (0.48, 0.80), (0.62, 1.20)]
    actual_right = [(-0.40, 0.30), (-0.32, 0.80), (-0.18, 1.20)]
    wall_corner = (-0.32, 0.55)
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
        pair_maximum_longitudinal_offset_m=0.38,
        root_maximum_y_m=0.75,
    )

    left_chain, right_chain = chain_paired_boundaries(
        [*actual_left, *actual_right, wall_corner],
        config,
    )

    assert left_chain == actual_left
    assert right_chain == actual_right


def test_turning_poly2_is_used_when_it_agrees_with_local_target():
    """코너라는 이유만으로 좋은 2차 경로를 버리지 않고, LOCAL과 일치할 때 채택한다."""
    slices = [
        _slice(float(y), 0.16 * float(y) ** 2 + 0.01)
        for y in np.arange(0.25, 1.51, 0.10)
    ]
    polynomial = fit_polynomial_centerline(slices)
    assert polynomial is not None

    lookahead = 0.62
    local_target_x = polynomial.x_at(lookahead) + 0.01
    selected_x, model = select_centerline_target(
        local_target_x,
        lookahead,
        turn_ratio=0.85,
        polynomial=polynomial,
        local_heading_rad=polynomial.heading_at(lookahead) + 0.05,
    )

    assert model == "POLY2"
    assert selected_x == pytest.approx(polynomial.x_at(lookahead))


def test_poly2_is_rejected_when_heading_disagrees_with_local_path():
    slices = [
        _slice(float(y), 0.16 * float(y) ** 2 + 0.01)
        for y in np.arange(0.25, 1.51, 0.10)
    ]
    polynomial = fit_polynomial_centerline(slices)
    assert polynomial is not None

    lookahead = 0.62
    polynomial_target = polynomial.x_at(lookahead)
    selected_x, model = select_centerline_target(
        polynomial_target + 0.01,
        lookahead,
        turn_ratio=0.85,
        polynomial=polynomial,
        local_heading_rad=polynomial.heading_at(lookahead) - 0.40,
    )

    assert model == "LOCAL"
    assert selected_x == pytest.approx(polynomial_target + 0.01)


def test_single_fallback_uses_near_root_not_curved_chain_median():
    """먼 구간에서 많이 휜 실제 벽을 median 오차 때문에 반대 clutter로 바꾸지 않는다."""
    curved_left = [(0.40, 0.30), (0.58, 0.70), (0.78, 1.10)]
    weak_right_clutter = [(-0.62, 0.25), (-0.40, 0.70), (-0.40, 1.10)]
    config = ConeCorridorConfig(single_wall_fallback_margin_m=0.40)

    left, right = _select_single_boundary(
        curved_left,
        weak_right_clutter,
        config,
    )

    assert left == curved_left
    assert right == []


def test_full_scan_s_curve_is_stable_against_same_side_clutter_rows():
    """실제 LaserScan 경로 전체에서 S자 경계와 같은 쪽 가구 열을 함께 재현한다."""

    def center_x(y_m: float) -> float:
        return 0.18 * math.sin(2.0 * math.pi * (y_m - 0.10) / 2.40)

    left = [
        (center_x(y_m) + 0.40, y_m)
        for y_m in (0.25, 0.70, 1.15, 1.60)
    ]
    right = [
        (center_x(y_m) - 0.40, y_m)
        for y_m in (0.45, 0.90, 1.35, 1.80)
    ]
    # 사진 속 의자/벽 모서리처럼 차량 왼쪽에만 있는 두 평행 열이다.
    clutter_inner = [(0.15, y_m) for y_m in (0.28, 0.68, 1.08, 1.48)]
    clutter_outer = [(0.85, y_m) for y_m in (0.28, 0.68, 1.08, 1.48)]
    config = ConeCorridorConfig(
        corridor_width_min_m=0.60,
        corridor_width_max_m=1.05,
        forward_min_m=0.10,
        forward_max_m=2.00,
        lateral_max_m=1.00,
        cone_min_cluster_points=2,
        cone_min_cluster_span_m=0.03,
        detection_min_total_points=3,
        detection_min_slices=2,
        max_cone_link_distance_m=0.35,
        far_max_cone_link_distance_m=0.58,
        link_distance_far_start_m=0.30,
        link_distance_far_full_m=0.60,
        pair_maximum_longitudinal_offset_m=0.38,
        root_maximum_y_m=0.75,
        target_lookahead_m=0.90,
        corner_target_lookahead_m=0.62,
        lookahead_near_max_m=0.60,
        lookahead_far_min_m=1.05,
        lookahead_far_max_m=1.45,
        polynomial_fit_max_y_m=1.60,
    )

    clean = analyze_cone_corridor(
        _make_mock_scan([*left, *right]),
        -math.pi,
        math.pi / 180.0,
        config=config,
    )
    cluttered = analyze_cone_corridor(
        _make_mock_scan([*left, *right, *clutter_inner, *clutter_outer]),
        -math.pi,
        math.pi / 180.0,
        config=config,
    )

    assert clean.found and cluttered.found
    assert clean.boundary_mode == "PAIRED"
    assert cluttered.boundary_mode == "PAIRED"
    assert clean.primary_side == cluttered.primary_side == "BOTH"
    assert clean.target_x_m > 0.08
    assert cluttered.target_x_m > 0.08
    assert cluttered.target_x_m == pytest.approx(clean.target_x_m, abs=0.04)
    assert cluttered.steering_rad == pytest.approx(clean.steering_rad, abs=0.05)

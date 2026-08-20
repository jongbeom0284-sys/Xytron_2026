"""라바콘 통로(Cone Corridor) 주행 알고리즘 단위 시험."""

import math
import numpy as np
import pytest

from kobot_lane_drive.cone_corridor import (
    ConeCorridorConfig,
    analyze_cone_corridor,
    chain_cone_boundary,
    compute_steering_pure_pursuit,
    extract_corridor_slices,
    select_boundary_roots,
    split_cone_walls,
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


# ---------------------------------------------------------------------------
# 좌/우 벽면 상호 배제 (90도 코너에서 외벽이 x=0 을 넘어올 때)
# ---------------------------------------------------------------------------

# 좌회전 헤어핀 실측 배치. 우측 외벽이 (0.30, 0.95) 에서 x=0 을 넘어 좌측으로 넘어오고,
# 진짜 안쪽(좌) 벽은 그보다 멀리 (0.75, 1.30) 에 하나만 남아 있다.
HAIRPIN_CONES = [
    (-0.40, 0.20),
    (-0.30, 0.60),
    (-0.05, 0.85),
    (0.30, 0.95),   # 우측 외벽이지만 x > 0.05 — 좌벽 시드 후보로도 잡힌다
    (0.75, 1.30),   # 진짜 좌측(안쪽) 벽
]
HAIRPIN_LINK_M = 0.55


def test_chain_root_must_belong_to_pool():
    """풀에 없는(이미 반대편 벽에 배정된) 콘은 체인의 시작점이 될 수 없다."""
    claimed = (0.30, 0.95)
    pool = [(0.75, 1.30)]

    chain = chain_cone_boundary([claimed], pool, HAIRPIN_LINK_M)

    assert claimed not in chain
    assert chain == []


def test_boundary_roots_prefer_near_and_default_to_no_far_fallback():
    """근거리 후보를 우선한다. 원거리 폴백은 기본 꺼짐이라 먼 콘은 시드가 되지 않는다."""
    near = (0.40, 0.60)
    far = (0.50, 1.80)

    # 근거리 후보가 있으면 그것만 쓴다 (폴백 설정과 무관)
    assert select_boundary_roots([near, far], left_side=True) == [near]

    # 기본값(폴백 꺼짐)에서는 근거리 창 밖 콘을 시드로 쓰지 않는다.
    # 원거리 잡음 클러스터 하나가 통로 벽으로 승격되는 것을 막는다.
    assert select_boundary_roots([far], left_side=True) == []

    # 명시적으로 켰을 때만 되돌린다
    assert select_boundary_roots([far], left_side=True, allow_far_fallback=True) == [far]

    # 반대편 콘은 절대 섞이지 않는다
    assert select_boundary_roots([(-0.40, 0.60)], left_side=True) == []


def test_hairpin_outer_wall_cone_is_not_shared_by_both_walls():
    """90도 코너에서 외벽 콘이 좌·우 양쪽 벽에 중복 배정되면 안 된다.

    수정 전에는 우벽 체인이 (0.30, 0.95) 를 삼킨 뒤에도 배제 전 목록으로 좌벽 시드를 골라
    같은 콘이 좌벽의 머리가 되었다. 그 결과 유령 좌벽이 생기고 중심선이 안쪽으로 밀렸다.
    """
    left_cones, right_cones = split_cone_walls(HAIRPIN_CONES, HAIRPIN_LINK_M)

    shared = set(left_cones) & set(right_cones)
    assert not shared, f"한 콘이 양쪽 벽에 중복 배정됨: {shared}"


def test_hairpin_keeps_genuine_inner_cone_when_fallback_enabled():
    """폴백을 켜면 상호 배제가 멀쩡한 안쪽 벽까지 지워버리지 않는다.

    이 폴백은 기본 꺼짐이다 — 원거리 잡음을 벽으로 승격시키는 대가가 실측으로 확인됐다
    (8/10 bag: found +37%). 코너에서 안쪽 벽 소실이 실제로 관측되면 그때 켠다.
    """
    left_cones, right_cones = split_cone_walls(
        HAIRPIN_CONES, HAIRPIN_LINK_M, allow_far_fallback=True
    )

    # 외벽은 x=0 을 넘어온 콘까지 자기 체인으로 갖는다
    assert (0.30, 0.95) in right_cones
    # 진짜 안쪽 콘은 버려지지 않고 좌벽으로 살아남는다
    assert (0.75, 1.30) in left_cones
    # 어떤 콘도 유실되지 않는다
    assert set(left_cones) | set(right_cones) == set(HAIRPIN_CONES)


def test_wall_split_is_exclusive_for_straight_corridor():
    """직선 통로에서는 좌/우가 부호대로 깔끔히 갈린다 (정상 경로 회귀 방지)."""
    cones = []
    for i in range(6):
        y = 0.25 + i * 0.30
        cones.append((0.40, y))
        cones.append((-0.40, y))

    left_cones, right_cones = split_cone_walls(cones, HAIRPIN_LINK_M)

    assert not (set(left_cones) & set(right_cones))
    assert all(c[0] > 0 for c in left_cones)
    assert all(c[0] < 0 for c in right_cones)
    assert len(left_cones) == 6 and len(right_cones) == 6


def test_root_fallback_switch_off_restores_strict_near_window():
    """폴백을 끄면 근거리 창 밖 콘은 시드가 되지 않는다 (수정 전 벽 배정과 동일)."""
    far = (0.50, 1.80)   # ROOT_NEAR_Y_M(1.20) 밖

    assert select_boundary_roots([far], left_side=True, allow_far_fallback=True) == [far]
    assert select_boundary_roots([far], left_side=True, allow_far_fallback=False) == []


def test_root_fallback_switch_does_not_reopen_the_shared_cone_bug():
    """폴백을 꺼도 시드 검사는 살아 있어 중복 배정은 여전히 막힌다."""
    for flag in (True, False):
        left_cones, right_cones = split_cone_walls(
            HAIRPIN_CONES, HAIRPIN_LINK_M, allow_far_fallback=flag
        )
        assert not (set(left_cones) & set(right_cones)), f"allow_far_fallback={flag} 에서 중복 배정"


def test_hairpin_default_drops_inner_cone_but_never_shares_it():
    """기본값(폴백 꺼짐)에서도 중복 배정은 없다 — 안쪽 콘이 버려질 뿐 양쪽 벽에 걸치지 않는다."""
    left_cones, right_cones = split_cone_walls(HAIRPIN_CONES, HAIRPIN_LINK_M)

    assert not (set(left_cones) & set(right_cones))
    assert (0.30, 0.95) in right_cones          # 외벽은 그대로
    assert (0.30, 0.95) not in left_cones       # 수정 전에는 여기 있었다 (버그)

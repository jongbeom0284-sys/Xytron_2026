"""차선 폭 측정과 주행 라인 상태기계."""

import numpy as np

from kobot_lane_drive.lane_policy import (
    CENTER,
    RETURNING,
    SHIFT_LEFT,
    SHIFT_RIGHT,
    LanePolicy,
    LanePolicyConfig,
    measure_lanes,
)
from kobot_lane_drive.obstacle_lidar import LidarSectors

WIDTH = 640
HEIGHT = 480
CONFIG = LanePolicyConfig()


def _road(block_side=None, block_from=0.45):
    """중앙선이 곧게 선 2차선 도로. block_side 쪽에 장애물 덩어리를 놓는다."""
    driving = np.zeros((HEIGHT, WIDTH), dtype=bool)
    center = WIDTH // 2
    driving[220:470, 60 : center - 6] = True
    driving[220:470, center + 7 : WIDTH - 60] = True
    if block_side == "left":
        driving[int(HEIGHT * block_from) : 470, 150 : center - 6] = False
    elif block_side == "right":
        driving[int(HEIGHT * block_from) : 470, center + 7 : WIDTH - 150] = False
    points = np.array([[center, y] for y in range(468, 222, -3)], dtype=np.int32)
    return driving, points


def _sectors(left=None, right=None, stamp=100.0):
    return LidarSectors(None, left, right, 0, 0, 0, stamp)


def test_symmetric_road_reports_no_narrow_side():
    driving, points = _road()
    occupancy = measure_lanes(driving, points, CONFIG)
    assert occupancy.narrow_side is None
    assert occupancy.measured_rows > 0
    assert abs(occupancy.left_width_px - occupancy.right_width_px) < 20


def test_blocked_left_lane_is_detected_as_narrow():
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, CONFIG)
    assert occupancy.narrow_side == "left"
    assert occupancy.right_center_px is not None


def test_missing_center_line_gives_empty_measurement():
    driving, _ = _road()
    occupancy = measure_lanes(driving, np.empty((0, 2), dtype=np.int32), CONFIG)
    assert occupancy.measured_rows == 0
    assert occupancy.narrow_side is None


def _run(policy, occupancy, sectors, now, center_x=320.0, speed=0.5, dt=0.1, enabled=True):
    return policy.update(
        center_x_px=center_x,
        occupancy=occupancy,
        sectors=sectors,
        speed_mps=speed,
        dt=dt,
        now=now,
        frame_width=WIDTH,
        maneuvers_enabled=enabled,
    )


def _warmup(policy, config, now, frames=None):
    """평상시 차선 폭을 학습시킨다. 이걸 거치지 않으면 정책이 판단을 보류한다."""
    clear = measure_lanes(*_road(), config)
    count = config.baseline_warmup_frames if frames is None else frames
    for step in range(count + 1):
        _run(policy, clear, _sectors(stamp=now), now + step * 0.01)
    return clear


def test_full_cycle_center_to_shift_and_back():
    config = LanePolicyConfig(
        enter_hold_sec=0.2, clear_hold_sec=0.2, pass_distance_m=0.3, target_rate_px_s=10_000.0
    )
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    blocked = measure_lanes(driving, points, config)
    now = 100.0
    clear = _warmup(policy, config, now)

    # 라이다가 좌측 물체를 확인해 주는 동안 유지 시간을 채운다
    decision = _run(policy, blocked, _sectors(left=1.0, stamp=now), now)
    assert decision.state == CENTER
    now += 0.3
    decision = _run(policy, blocked, _sectors(left=1.0, stamp=now), now)
    assert decision.state == SHIFT_RIGHT
    assert decision.blocked_side == "left"
    assert decision.target_x_px > 320.0
    assert decision.speed_scale < 1.0

    # 장애물이 사라져도 통과 거리가 모자라면 아직 복귀하지 않는다
    now += 0.1
    assert _run(policy, clear, _sectors(stamp=now), now, dt=0.1).state == SHIFT_RIGHT

    # 시간이 흘러 통과 거리를 채우면 복귀
    for _ in range(8):
        now += 0.1
        decision = _run(policy, clear, _sectors(stamp=now), now, dt=0.1)
    assert decision.state in (RETURNING, CENTER)

    now += 0.1
    decision = _run(policy, clear, _sectors(stamp=now), now)
    assert decision.state == CENTER
    assert abs(decision.target_x_px - 320.0) < 1.0


def test_blocked_right_lane_shifts_left():
    config = LanePolicyConfig(enter_hold_sec=0.0, target_rate_px_s=10_000.0)
    policy = LanePolicy(config)
    driving, points = _road(block_side="right")
    occupancy = measure_lanes(driving, points, config)
    now = 50.0
    _warmup(policy, config, now)
    _run(policy, occupancy, _sectors(right=1.0, stamp=now), now)
    decision = _run(policy, occupancy, _sectors(right=1.0, stamp=now), now + 0.05)
    assert decision.state == SHIFT_LEFT
    assert decision.target_x_px < 320.0


def test_camera_alone_never_confirms_occupancy():
    """라이다 확인이 없으면 차선을 넘지 않는다."""
    config = LanePolicyConfig(enter_hold_sec=0.0)
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, config)
    now = 10.0
    _warmup(policy, config, now)
    for step in range(10):
        decision = _run(policy, occupancy, _sectors(stamp=now), now + step * 0.1)
    assert decision.state == CENTER


def test_stale_lidar_is_not_used_as_evidence():
    config = LanePolicyConfig(enter_hold_sec=0.0, lidar_maximum_age_sec=0.5)
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, config)
    now = 200.0
    _warmup(policy, config, now)
    stale = _sectors(left=0.5, stamp=now - 3.0)
    for step in range(5):
        decision = _run(policy, occupancy, stale, now + step * 0.1)
    assert decision.state == CENTER


def test_disabled_maneuvers_hold_the_center_line():
    config = LanePolicyConfig(enter_hold_sec=0.0, target_rate_px_s=10_000.0)
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, config)
    now = 5.0
    _warmup(policy, config, now)
    for step in range(6):
        decision = _run(
            policy, occupancy, _sectors(left=0.8, stamp=now), now + step * 0.1, enabled=False
        )
    assert decision.state == CENTER
    assert decision.reason == "maneuvers_disabled"
    assert abs(decision.target_x_px - 320.0) < 1.0


def test_target_moves_at_a_limited_rate():
    config = LanePolicyConfig(enter_hold_sec=0.0, target_rate_px_s=100.0)
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, config)
    now = 0.0
    _warmup(policy, config, now)
    _run(policy, occupancy, _sectors(left=1.0, stamp=now), now, dt=0.1)
    first = _run(policy, occupancy, _sectors(left=1.0, stamp=now), now + 0.05, dt=0.1)
    # 한 주기 0.1초면 최대 10px 만 움직인다
    assert abs(first.target_x_px - 320.0) <= 10.0 + 1e-6


def test_permanently_narrow_side_is_not_treated_as_an_obstacle():
    """벽을 낀 구간처럼 한쪽이 늘 좁은 도로에서는 차선을 넘지 않는다.

    실차 녹화(20260818_204211_264)에서 좌 214px / 우 70px 로 우측이 상시 좁게 나왔다.
    폭 비율만 보면 계속 점유로 읽히므로, 그 차선 자신의 평상시 폭 대비 감소를 요구한다.
    """
    config = LanePolicyConfig(enter_hold_sec=0.0, baseline_warmup_frames=5)
    policy = LanePolicy(config)
    driving, points = _road(block_side="right")
    narrow = measure_lanes(driving, points, config)
    assert narrow.narrow_side == "right"

    now = 0.0
    # 처음부터 계속 좁은 상태만 보여준다 -> 평상시 폭을 배울 기회가 없다
    for step in range(40):
        decision = _run(policy, narrow, _sectors(right=0.8, stamp=now), now + step * 0.1)
    assert decision.state == CENTER


def test_curved_section_never_triggers_a_lane_change():
    """코너에서는 회피하지 않는다. 목표선이 기울면 좌우 폭이 원근으로 왜곡된다."""
    config = LanePolicyConfig(enter_hold_sec=0.0, baseline_warmup_frames=5)
    policy = LanePolicy(config)
    driving, points = _road(block_side="left")
    occupancy = measure_lanes(driving, points, config)
    now = 0.0
    _warmup(policy, config, now, frames=6)
    for step in range(10):
        decision = policy.update(
            center_x_px=320.0,
            occupancy=occupancy,
            sectors=_sectors(left=1.0, stamp=now),
            speed_mps=0.5,
            dt=0.1,
            now=now + step * 0.1,
            frame_width=WIDTH,
            heading_deg=28.0,
        )
    assert decision.state == CENTER

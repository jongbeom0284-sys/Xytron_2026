"""추월/회피 컨트롤러 검증 — 규정 p.30~p.34.

이 대회에서 가장 실격에 가까운 조항이 여기 있다:
  "방해차량이 주행하지 않는 차선 쪽으로 추월해야 함.
   만약 반대쪽으로 추월을 시도하는 경우 차선이탈로 간주하여 해당 규정을 적용함."
방향을 틀리면 회피에 성공해도 이탈 판정이다. 그래서 방향 선택만 따로 테스트한다.
"""

from conftest import moving_target, stationary_target

from kmu_race.core.target_passing import (
    PassConfig,
    PassPhase,
    TargetPassing,
)


def run(passer, target, *, t0=0.0, frames=1, step=0.1, **kw):
    """프레임을 흘려보내고 마지막 결과를 돌려준다."""
    kw.setdefault("lane_offset_px", 0.0)
    kw.setdefault("lane_lookahead_px", 0.0)
    kw.setdefault("lane_side", 0)
    kw.setdefault("left_clear", True)
    kw.setdefault("right_clear", True)
    result = None
    for i in range(frames):
        result = passer.update(target=target, now=t0 + step * i, **kw)
    return result


# ──────────────────────────────────────────────────────────────────
# 방향 선택 — 규정 p.33
# ──────────────────────────────────────────────────────────────────
def test_target_on_left_passes_right():
    """방해차량이 1차선(좌)에 있으면 오른쪽으로 추월해야 한다."""
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.0, y=+0.35))
    assert r.side == +1, "좌측 타겟은 우측으로 지나가야 한다"


def test_target_on_right_passes_left():
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.0, y=-0.35))
    assert r.side == -1


def test_head_on_target_uses_clear_side():
    """정면이라 횡위치로 못 가릴 때는 비어 있는 쪽으로."""
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.0, y=0.0), left_clear=False, right_clear=True)
    assert r.side == +1

    p2 = TargetPassing(moving=True)
    r2 = run(p2, moving_target(1.0, y=0.0), left_clear=True, right_clear=False)
    assert r2.side == -1


def test_head_on_both_clear_uses_opposite_lane():
    """둘 다 비었으면 노란 점선 건너편. 규정 p.27 상 1·2차선 구분이 없어 합법."""
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.0, y=0.0), lane_side=+1)
    assert r.side == -1

    p2 = TargetPassing(moving=True)
    r2 = run(p2, moving_target(1.0, y=0.0), lane_side=-1)
    assert r2.side == +1


def test_required_side_blocked_falls_back_to_other():
    """규정이 요구하는 쪽이 막혔으면 반대쪽. 충돌(10초)보다는 낫다."""
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.0, y=+0.35), right_clear=False, left_clear=True)
    assert r.side == -1


def test_both_sides_blocked_reports_blocked_and_slows():
    p = TargetPassing(moving=True)
    cfg = p.config
    r = run(p, moving_target(0.9, y=0.0), left_clear=False, right_clear=False)
    assert r.status == "blocked"
    assert r.side == 0
    assert r.speed <= cfg.shift_speed * 0.3, "막혔으면 붙지 말고 감속"


# ──────────────────────────────────────────────────────────────────
# 끼어들기 대응 — 규정 p.32 "1차선과 2차선을 오가면서 주행함"
# ──────────────────────────────────────────────────────────────────
def test_moving_target_cut_in_switches_side():
    """내가 가려는 쪽으로 방해차량이 넘어오면 방향을 바꾼다."""
    p = TargetPassing(moving=True)
    r = run(p, moving_target(1.2, y=+0.35))
    assert r.side == +1 and r.phase is PassPhase.SHIFT

    # 방해차량이 우측으로 넘어옴 → 좌측으로 전환
    r = run(p, moving_target(1.1, y=-0.35), t0=1.0, frames=5)
    assert r.side == -1, "끼어들면 반대쪽으로 재전환해야 한다"


def test_static_target_does_not_switch_side():
    """고장난 차량은 안 움직인다. 한번 정한 방향을 흔들지 않는다."""
    p = TargetPassing(moving=False)
    r = run(p, stationary_target(1.2, y=+0.35))
    assert r.side == +1

    r = run(p, stationary_target(1.1, y=-0.35), t0=1.0, frames=5)
    assert r.side == +1, "정지 타겟은 방향을 유지"


# ──────────────────────────────────────────────────────────────────
# 복귀 시한 — 규정 p.32 "뒷차와의 간격이 90센치 이상이면 차선이탈로 판정"
# ──────────────────────────────────────────────────────────────────
def test_return_is_more_urgent_as_gap_approaches_90cm():
    def offset_after_one_return_frame(gap_m):
        p = TargetPassing(moving=True)
        p.phase = PassPhase.RETURN
        p.side = +1
        p.offset_px = 300.0
        r = p.update(
            target=moving_target(-gap_m, y=0.4),
            lane_offset_px=0.0,
            lane_lookahead_px=0.0,
            lane_side=0,
            left_clear=True,
            right_clear=True,
            now=0.0,
        )
        assert r.phase is PassPhase.RETURN
        return abs(p.offset_px)

    near_deadline = offset_after_one_return_frame(0.85)   # 시한 임박
    plenty = offset_after_one_return_frame(0.05)          # 이제 막 지나침
    assert near_deadline < plenty, (
        "90cm 시한이 가까울수록 더 빠르게 복귀해야 한다"
    )


def test_return_completes_and_reports_done():
    p = TargetPassing(moving=True)
    p.phase = PassPhase.RETURN
    p.offset_px = 300.0
    r = None
    for i in range(60):
        r = p.update(
            target=None,
            lane_offset_px=0.0,
            lane_lookahead_px=0.0,
            lane_side=0,
            left_clear=True,
            right_clear=True,
            now=0.05 * i,
        )
        if r.done:
            break
    assert r.done is True
    assert p.phase is PassPhase.IDLE


# ──────────────────────────────────────────────────────────────────
# 추돌 방지 — 규정 p.34 (가해·피해 모두 벌초 10초)
# ──────────────────────────────────────────────────────────────────
def test_slows_down_when_too_close_before_shifting():
    p = TargetPassing(moving=True)
    cfg = p.config
    r = run(p, moving_target(0.3, y=+0.35))
    assert r.speed <= cfg.shift_speed * 0.3, "아직 못 비켰으면 붙지 않는다"


# ──────────────────────────────────────────────────────────────────
# 교착 방지 — 규정 p.21 (멈춘 후 1분 내 재개 못하면 실격)
# ──────────────────────────────────────────────────────────────────
def test_timeout_releases_back_to_lane_driving():
    p = TargetPassing(moving=True, config=PassConfig(timeout_s=1.0))
    run(p, moving_target(1.0, y=+0.35))
    r = run(p, moving_target(1.0, y=+0.35), t0=2.0)
    assert r.done is True
    assert r.phase is PassPhase.IDLE


def test_full_pass_sequence_reaches_alongside_then_returns():
    p = TargetPassing(moving=True)
    # SHIFT: 옆으로 이동
    for i in range(40):
        r = run(p, moving_target(1.2, y=+0.35), t0=0.05 * i)
        if p.phase is PassPhase.ALONGSIDE:
            break
    assert p.phase is PassPhase.ALONGSIDE

    # 타겟이 뒤로 넘어감 → RETURN
    for i in range(20):
        r = p.update(
            target=moving_target(-0.3, y=+0.4),
            lane_offset_px=0.0,
            lane_lookahead_px=0.0,
            lane_side=0,
            left_clear=True,
            right_clear=True,
            now=3.0 + 0.05 * i,
        )
        if p.phase is PassPhase.RETURN:
            break
    assert p.phase is PassPhase.RETURN


def test_wide_corridor_keeps_shifted_target_visible():
    """자기 횡이동(0.7m) 중에도 타겟이 시야에 남아야 조기 복귀가 없다 (리뷰 C2).

    기본 코리도(0.45m)는 비킨 순간 타겟을 놓치고, 기동용 넓은 코리도(1.2m)는
    유지한다. 통과(뒤로 빠짐) 판정은 x<=0 필터가 계속 담당한다.
    """
    from kmu_race.core.types import LidarReading, TrackedObject

    lidar = LidarReading(
        objects=(TrackedObject(x=1.0, y=0.6, width=0.3, track_id=1),),
        stamp=1.0,
    )
    assert lidar.nearest_front() is None                      # 경로 차단 판정용
    assert lidar.nearest_front(max_lateral=1.2) is not None   # 기동 추적용

    behind = LidarReading(
        objects=(TrackedObject(x=-0.3, y=0.6, width=0.3, track_id=1),),
        stamp=1.0,
    )
    assert behind.nearest_front(max_lateral=1.2) is None      # 통과하면 사라진다


def test_blocked_idle_still_arms_timeout():
    """양쪽 막힘(blocked)으로 IDLE 에 머물러도 타임아웃 탈출구가 생긴다 (리뷰 C3)."""
    from kmu_race.core.target_passing import PassConfig, TargetPassing
    from kmu_race.core.types import TrackedObject

    tp = TargetPassing(moving=False, config=PassConfig(timeout_s=2.0))
    target = TrackedObject(x=1.0, y=0.0, width=0.3, track_id=1)
    done = False
    for i in range(60):
        r = tp.update(
            target=target,
            lane_offset_px=0.0,
            lane_lookahead_px=0.0,
            lane_side=0,
            left_clear=False,
            right_clear=False,
            now=float(i) * 0.1,
        )
        if r.done:
            done = True
            break
    assert done, "blocked 에서도 timeout_s 후에는 빠져나와야 한다"

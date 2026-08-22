"""랩 카운팅 검증.

랩을 하나 놓치면 3바퀴를 못 채우고, 하나 더 세면 2바퀴 만에 멈춘다.
둘 다 곧바로 실격이라 이 모듈은 오검출/중복검출 양쪽 모두를 막아야 한다.
"""

from kmu_race.core.race_context import (
    TOTAL_LAPS,
    LapTracker,
    LapTrackerConfig,
    RaceContext,
)


def feed(tracker, *, span, frames, t0=0.0, step=0.05, yaw=0.0):
    """같은 조건을 여러 프레임 먹이고 발생한 목격 수를 돌려준다."""
    hits = 0
    for i in range(frames):
        if tracker.update(
            gantry_span_px=span, now=t0 + step * i, yaw_deg=yaw
        ):
            hits += 1
    return hits, t0 + step * frames


def test_single_sighting_counts_once():
    """겐트리 앞에 오래 서 있어도 한 번만 센다."""
    trk = LapTracker()
    hits, _ = feed(trk, span=40, frames=200)
    assert hits == 1
    assert trk.sightings == 1


def test_far_gantry_is_ignored():
    """임계 미만으로 작게 잡히는 것은 세지 않는다.

    0817 재산출로 기본 임계가 5px 다 (새 카메라 실측 — race_context.py 주석).
    크기 기반 근/원거리 분리는 이 카메라에선 불가능해서, 원거리 오검 억제는
    검출기의 링·구조 게이트 + min_gap_s 가 담당한다.
    """
    trk = LapTracker()
    hits, _ = feed(trk, span=4, frames=100)
    assert hits == 0
    assert trk.sightings == 0


def test_second_sighting_after_absence():
    trk = LapTracker()
    _, t = feed(trk, span=40, frames=20)          # 목격 1
    _, t = feed(trk, span=0, frames=400, t0=t)    # 한 바퀴 (20초)
    hits, _ = feed(trk, span=40, frames=20, t0=t)
    assert hits == 1
    assert trk.sightings == 2


def test_brief_flicker_does_not_recount():
    """겐트리 밑을 지나며 몇 프레임 끊겨도 다시 세지 않는다."""
    trk = LapTracker()
    _, t = feed(trk, span=40, frames=20)
    _, t = feed(trk, span=0, frames=5, t0=t)      # release_frames(12) 미만
    hits, _ = feed(trk, span=40, frames=20, t0=t)
    assert hits == 0
    assert trk.sightings == 1


def test_min_gap_blocks_rapid_recount():
    """미검출이 길어도 직전 목격에서 15초가 안 지났으면 안 센다."""
    cfg = LapTrackerConfig(min_gap_s=15.0)
    trk = LapTracker(cfg)
    _, t = feed(trk, span=40, frames=20)
    _, t = feed(trk, span=0, frames=40, t0=t)     # 2초만 경과
    hits, _ = feed(trk, span=40, frames=20, t0=t)
    assert hits == 0
    assert trk.sightings == 1


def test_lap_progression_and_finish():
    trk = LapTracker()
    t = 0.0
    for expected in (1, 2, 3):
        _, t = feed(trk, span=40, frames=20, t0=t)
        assert trk.current_lap == expected
        assert not trk.finished
        _, t = feed(trk, span=0, frames=400, t0=t)
    _, t = feed(trk, span=40, frames=20, t0=t)
    assert trk.finished, "4번째 목격 = 결승선 통과 후"
    assert trk.current_lap == TOTAL_LAPS


def test_yaw_cross_check_flags_missed_lap():
    """카메라가 겐트리를 놓쳤을 가능성을 요 적산으로 의심한다."""
    trk = LapTracker()
    feed(trk, span=40, frames=20, yaw=0.0)
    assert not trk.yaw_suggests_missed_lap(50.0)
    assert trk.yaw_suggests_missed_lap(360.0)


# ──────────────────────────────────────────────────────────────────
# 지름길 규칙 (규정 p.35)
# ──────────────────────────────────────────────────────────────────
def test_shortcut_not_allowed_on_lap1():
    ctx = RaceContext()
    ctx.lap.sightings = 1
    assert ctx.may_take_shortcut(True) is False


def test_shortcut_allowed_on_lap2_and_3():
    for lap in (2, 3):
        ctx = RaceContext()
        ctx.lap.sightings = lap
        assert ctx.may_take_shortcut(True) is True


def test_shortcut_requires_left_signal():
    ctx = RaceContext()
    ctx.lap.sightings = 2
    assert ctx.may_take_shortcut(False) is False


def test_shortcut_locked_after_use():
    ctx = RaceContext()
    ctx.lap.sightings = 2
    ctx.race_started_at = 0.0
    assert ctx.may_take_shortcut(True) is True
    ctx.commit_shortcut(10.0)
    ctx.lap.sightings = 3
    assert ctx.may_take_shortcut(True) is False


# ──────────────────────────────────────────────────────────────────
# 요 보충 랩 하이브리드 (0817 3바퀴 bag 재설계 — race_context.py 주석 참조)
# ──────────────────────────────────────────────────────────────────
def test_yaw_fills_missed_sighting():
    """통과 시 램프가 3px(임계 미만)로 목격을 놓쳐도 요 누적으로 랩이 오른다."""
    trk = LapTracker()
    _, t = feed(trk, span=40, frames=20, yaw=0.0)        # 목격 1 (출발)
    assert trk.sightings == 1
    # 한 바퀴 돌았지만 겐트리는 안 보임 (span 0, 요 -360)
    hits, t = feed(trk, span=0, frames=400, t0=t, yaw=-360.0)
    assert hits == 1, "355° 이상 회전 → 요 보충 랩"
    assert trk.sightings == 2
    assert trk.yaw_filled == 1


def test_yaw_fill_then_real_sighting_not_double_counted():
    """요 보충 직후(min_gap 내) 도착한 실제 목격은 중복 카운트되지 않는다."""
    trk = LapTracker()
    _, t = feed(trk, span=40, frames=20, yaw=0.0)
    _, t = feed(trk, span=0, frames=400, t0=t, yaw=-356.0)   # 보충 발화
    assert trk.sightings == 2
    hits, _ = feed(trk, span=40, frames=20, t0=t, yaw=-360.0)  # 5s 뒤 실제 통과
    assert hits == 0, "min_gap_s 가 중복을 막는다"
    assert trk.sightings == 2


def test_no_yaw_fill_before_first_sighting():
    """출발 기준선(목격 1)이 없으면 요 보충도 없다 — 이송 회전 오염 방지."""
    trk = LapTracker()
    hits, _ = feed(trk, span=0, frames=400, yaw=720.0)
    assert hits == 0
    assert trk.sightings == 0


def test_hold_start_baseline_caps_phantom_sightings():
    """출발 대기 캡: 대기 중 유령 목격(책상 오검 실측)이 랩을 못 채운다."""
    trk = LapTracker()
    t = 0.0
    for k in range(3):                                    # 20s 간격 재점멸 3회
        _, t = feed(trk, span=40, frames=20, t0=t, yaw=0.0)
        _, t = feed(trk, span=0, frames=400, t0=t, yaw=0.0)
    assert trk.sightings == 3, "캡 없으면 3회 쌓인다 (전제 확인)"
    trk.hold_start_baseline(yaw_deg=235.0)                # WAIT_GREEN 틱
    assert trk.sightings == 1
    # 기준선이 235 로 동기됐으므로 -120 까지 돌아야 보충 발화 (355 差)
    hits, _ = feed(trk, span=0, frames=400, t0=t, yaw=-121.0)
    assert hits == 1 and trk.sightings == 2

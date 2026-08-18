"""FSM 전이 검증 — 규정 조항을 그대로 테스트로 옮긴 것.

실차 없이 여기서 통과해야 트랙에 나갈 자격이 있다.
"""

from conftest import make_obs, moving_target, stationary_target

from kmu_race.core.race_context import RaceContext
from kmu_race.core.race_fsm import FSMConfig, Mode, RaceFSM
from kmu_race.core.safety import SafetyDecision, SafetyLevel, SafetyMonitor
from kmu_race.core.types import SignalState

READY = SafetyDecision(SafetyLevel.OK, "", True)
NOT_READY = SafetyDecision(SafetyLevel.OK, "waiting", False)
DEGRADED_NO_CAM = SafetyDecision(SafetyLevel.DEGRADED, "camera stale", False)


def drive(fsm, ctx, obs_list, safety=READY):
    """관측 시퀀스를 흘려보내고 마지막 상태를 돌려준다."""
    for obs in obs_list:
        fsm.step(obs, ctx, safety)
    return fsm.state


# ──────────────────────────────────────────────────────────────────
# 출발 (규정 p.17~p.20)
# ──────────────────────────────────────────────────────────────────
def test_init_waits_for_inputs():
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, NOT_READY)
    assert fsm.state is Mode.INIT

    fsm.step(make_obs(0.1), ctx, READY)
    assert fsm.state is Mode.WAIT_GREEN


def test_red_light_does_not_start():
    """규정 p.20: 빨간불에 출발하면 벌초 10초. 절대 나가면 안 된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, READY)

    for i in range(30):
        fsm.step(
            make_obs(0.1 * i, signal=SignalState.RED, gantry_span=40), ctx, READY
        )
    assert fsm.state is Mode.WAIT_GREEN
    assert ctx.race_started_at is None


def test_green_starts_after_debounce():
    """한 프레임 오검출로 출발하지 않는다 (기본 4프레임 + 0.12s)."""
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, READY)

    fsm.step(make_obs(0.1, signal=SignalState.GREEN, gantry_span=40), ctx, READY)
    assert fsm.state is Mode.WAIT_GREEN, "1프레임에 출발하면 안 된다"

    for i in range(2, 8):
        fsm.step(
            make_obs(0.1 * i, signal=SignalState.GREEN, gantry_span=40), ctx, READY
        )
    assert fsm.state is Mode.LANE_DRIVE
    assert ctx.race_started_at is not None
    assert ctx.lap.current_lap == 1


def test_green_flicker_resets_debounce():
    """진짜 끊김(분류도 블롭도 없음)은 카운터를 리셋한다.

    블롭까지 사라진 UNKNOWN 이어야 '끊김'이다 — 블롭이 남아 있는 UNKNOWN 은
    분류만 실패한 것이므로 go 로 인정된다 (test_start_on_green_blob 참조).
    """
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, READY)
    # 확정 직전(2프레임 — green_confirm_frames=3 미만)까지 초록을 먹인다
    for i in range(1, 3):
        fsm.step(
            make_obs(0.1 * i, signal=SignalState.GREEN, gantry_span=40), ctx, READY
        )
    assert fsm.state is Mode.WAIT_GREEN
    # 한 프레임 완전 끊김 → 카운터 리셋
    fsm.step(make_obs(0.3, signal=SignalState.UNKNOWN, gantry_span=0), ctx, READY)
    # 리셋됐으므로 초록 1프레임으로는 다시 출발 못 한다
    fsm.step(make_obs(0.4, signal=SignalState.GREEN, gantry_span=40), ctx, READY)
    assert fsm.state is Mode.WAIT_GREEN


def test_start_on_green_blob_when_classification_fails():
    """출발선 거리에선 4구 분류가 거의 안 된다 (run4 실측: GREEN 2/11,056프레임).

    빨간 구간(블롭 없음)을 충분히 본 뒤 초록 블롭이 '새로 나타나면' 분류가
    UNKNOWN 이어도 파란불로 인정하고 출발해야 한다. FSM 리플레이에서 출발
    실패로 발견된 결함의 회귀 테스트다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, READY)
    assert fsm.state is Mode.WAIT_GREEN

    # 빨간불 대기 = 블롭 없는 어두운 구간 (상승엣지 전제 충족 — 3s 이상,
    # 0817: 깜빡임발 가짜 어두운 구간 차단을 위해 1s→3s)
    for i in range(1, 36):
        fsm.step(make_obs(0.1 * i, gantry_span=0), ctx, READY)
    assert fsm.state is Mode.WAIT_GREEN

    for i in range(36, 43):
        fsm.step(
            make_obs(0.1 * i, signal=SignalState.UNKNOWN, gantry_span=12),
            ctx,
            READY,
        )
    assert fsm.state is Mode.LANE_DRIVE
    assert ctx.race_started_at is not None
    assert ctx.lap.sightings >= 1, "출발 = 목격 1 (0817 lap0 출발 회귀)"


def test_start_via_far_start_blob_registers_lap_baseline():
    """원거리 출발 경로(start_blob_px, span 0)로 출발해도 목격 1 이 선다.

    0817 실사고: lap 0/3 으로 출발 → 4바퀴를 돌아야 FINISH. 출발 = 신호등
    앞 = 목격 1 이 규정상 자명하므로 기준선을 강제로 세워야 한다.
    """
    from kmu_race.core.types import SignalReading
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0), ctx, READY)
    for i in range(1, 36):
        fsm.step(make_obs(0.1 * i, gantry_span=0), ctx, READY)
    for i in range(36, 43):
        obs = make_obs(0.1 * i, gantry_span=0)
        obs.signal = SignalReading(start_blob_px=7)     # 원거리 출발 경로만 켜짐
        fsm.step(obs, ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE
    assert ctx.lap.sightings == 1
    assert ctx.lap.current_lap == 1


def test_persistent_false_blob_never_starts():
    """대기 내내 켜져 있는 정적 오검 블롭(조명·유도등)으로는 출발하지 않는다.

    상승엣지 회귀 테스트: 블롭이 '빨간 구간 동안 없다가 새로 나타난' 것이
    아니면 파란불이 아니다 — 빨간불 출발은 벌초 10초다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    fsm.step(make_obs(0.0, gantry_span=12), ctx, READY)   # 첫 프레임부터 블롭
    for i in range(1, 40):
        fsm.step(
            make_obs(0.1 * i, signal=SignalState.UNKNOWN, gantry_span=12),
            ctx,
            READY,
        )
    assert fsm.state is Mode.WAIT_GREEN
    assert ctx.race_started_at is None


# ──────────────────────────────────────────────────────────────────
# 랩 카운팅 + 지름길 (규정 p.35, p.7)
# ──────────────────────────────────────────────────────────────────
def _start(fsm, ctx, t0=0.0):
    fsm.step(make_obs(t0), ctx, READY)
    for i in range(1, 8):
        fsm.step(
            make_obs(t0 + 0.1 * i, signal=SignalState.GREEN, gantry_span=40),
            ctx,
            READY,
        )
    assert fsm.state is Mode.LANE_DRIVE
    return t0 + 1.0


def _lap_away(fsm, ctx, t, duration=40.0, step=0.1):
    """신호등이 안 보이는 구간을 달린다."""
    n = int(duration / step)
    for i in range(n):
        fsm.step(make_obs(t + step * i, gantry_span=0), ctx, READY)
    return t + duration


def _approach_signal(fsm, ctx, t, signal, frames=6, step=0.1):
    for i in range(frames):
        fsm.step(
            make_obs(t + step * i, signal=signal, gantry_span=40), ctx, READY
        )
    return t + step * frames


def test_shortcut_taken_on_lap2():
    """규정 p.35/p.36: 2바퀴째 직좌 동시신호 → 지름길."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)

    assert ctx.lap.current_lap == 2
    assert fsm.state is Mode.SHORTCUT
    assert ctx.shortcut_used is True
    assert ctx.left_signal_seen_on_lap == 2


def test_shortcut_taken_on_lap3_when_lap2_was_green_only():
    """좌회전은 2 또는 3바퀴째 랜덤. 2바퀴에 없으면 3바퀴에 잡아야 한다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)

    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN)   # 2랩: 직진만
    assert ctx.lap.current_lap == 2
    assert ctx.shortcut_used is False
    assert fsm.state is Mode.LANE_DRIVE

    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)  # 3랩: 직좌
    assert ctx.lap.current_lap == 3
    assert fsm.state is Mode.SHORTCUT
    assert ctx.left_signal_seen_on_lap == 3
    assert ctx.shortcut_used is True


def test_shortcut_used_only_once():
    """규정 p.35: 3바퀴 중 1번만."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)

    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)
    assert fsm.state is Mode.SHORTCUT
    # 좌회전 완료
    for i in range(30):
        fsm.step(make_obs(t + 0.1 * i, yaw=-3.0 * i), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE
    t += 3.0

    # 3랩에서 또 좌회전 신호가 떠도 다시 쓰지 않는다.
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)
    assert ctx.lap.current_lap == 3
    assert fsm.state is not Mode.SHORTCUT


def test_three_laps_then_finish():
    """규정 p.38: 통과감지장치를 3번 지나면 종료."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for _ in range(3):
        t = _lap_away(fsm, ctx, t)
        t = _approach_signal(fsm, ctx, t, SignalState.GREEN)
    assert fsm.state is Mode.FINISH
    assert ctx.lap.sightings == 4          # 출발 1 + 랩마커 3


def test_far_green_left_is_not_latched():
    """겐트리가 안 보이는 곳(span=0)에서 읽힌 GREEN_LEFT 는 래치되지 않는다.

    대회장 조명 오검이 지름길 래치에 스며드는 것을 막는 근접 게이트
    회귀 테스트 — 가짜 GREEN_LEFT 직후 실제 겐트리(직진 신호)를 목격해도
    지름길로 가면 안 된다. 지름길 오발동은 곧 차선이탈이다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)

    # 겐트리 목격 직전, 래치 시간창(4s) 안의 가짜 GREEN_LEFT (블롭 없음)
    for i in range(4):
        fsm.step(
            make_obs(t + 0.1 * i, signal=SignalState.GREEN_LEFT, gantry_span=0),
            ctx,
            READY,
        )
    t += 0.4

    t = _approach_signal(fsm, ctx, t, SignalState.GREEN)   # 실제 신호는 직진
    assert ctx.lap.current_lap == 2
    assert fsm.state is not Mode.SHORTCUT
    assert ctx.shortcut_used is False


def test_shortcut_turn_completes_on_yaw():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)
    assert fsm.state is Mode.SHORTCUT

    fsm.step(make_obs(t + 0.1, yaw=-40.0), ctx, READY)
    assert fsm.state is Mode.SHORTCUT, "40도로는 아직 부족"

    fsm.step(make_obs(t + 0.2, yaw=-80.0), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE


def test_shortcut_turn_times_out():
    """요가 안 돌아도 교착되지 않는다 — 멈추면 실격(p.21)이라 반드시 빠져나온다."""
    cfg = FSMConfig(shortcut_timeout_s=1.0)
    fsm, ctx = RaceFSM(cfg), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN_LEFT)
    assert fsm.state is Mode.SHORTCUT

    fsm.step(make_obs(t + 1.5, yaw=0.0), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE


# ──────────────────────────────────────────────────────────────────
# 라바콘 (규정 p.23~p.25)
# ──────────────────────────────────────────────────────────────────
def test_cone_enter_and_exit():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)

    # 규정 p.23~25: 라바콘은 좌우 두 줄 — 진입은 게이트 성립이 조건이다.
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE

    # 콘이 사라져도 바로 나가지 않는다 (기본 14프레임)
    fsm.step(make_obs(t + 1.0, cone_px=0), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE

    for i in range(20):
        fsm.step(make_obs(t + 1.1 + 0.1 * i, cone_px=0), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE


def test_cone_pile_without_gate_stays_in_lane():
    """벽면 보관 콘 무더기(한쪽 덩어리)로는 라바콘 구간에 진입하지 않는다.

    대회장은 치울 수 없다 — run4 실측에서 orange>900 이 660프레임이었지만
    두 줄 게이트 성립은 5프레임뿐이었다. 게이트 없는 주황 덩어리는 미션이
    아니라 배경이다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(10):
        fsm.step(make_obs(t + 0.1 * i, cone_px=8000), ctx, READY)  # gate_valid=False
    assert fsm.state is Mode.LANE_DRIVE


def test_cone_section_ignores_front_target():
    """라바콘은 라이다에 장애물로 잡힌다. 게이트가 살아있는 동안(=진짜 구간
    한복판)에는 콘 무리를 전방 타겟으로 오인해 추월로 새면 안 된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE

    for i in range(10):
        fsm.step(
            make_obs(
                t + 1.0 + 0.1 * i,
                cone_px=5000,
                gate_valid=True,
                front_target=stationary_target(0.8),
            ),
            ctx,
            READY,
        )
    assert fsm.state is Mode.CONE_DRIVE


def test_cone_gate_lost_exits_and_reenables_avoidance():
    """벽면 보관 콘 무더기 고착 회귀 테스트 (리뷰 A2).

    주황 픽셀은 계속 많은데 게이트가 오래 안 생기면 구간이 끝난 것이다 —
    CONE_DRIVE 에 잔류하면 전방 타겟 회피가 꺼진 채 고정장애물로 직진한다.
    게이트 소실 이탈 후에는 전방 타겟에 정상적으로 반응해야 한다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE

    # 무더기: 주황은 많지만 게이트 없음 → 4.0s 후 이탈해야 한다
    # (0818: 슬라럼 안 게이트 성립률 37% 실측 → 1.2s 는 구간 한복판 이탈)
    for i in range(45):
        fsm.step(make_obs(t + 0.5 + 0.1 * i, cone_px=8000), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE

    # 이탈했으니 전방 고정장애물에 회피가 발동한다
    for i in range(4):
        fsm.step(
            make_obs(
                t + 5.2 + 0.1 * i,
                front_target=stationary_target(1.0, ego_mps=1.2),
                ego_mps=1.2,
            ),
            ctx,
            READY,
        )
    assert fsm.state is Mode.AVOID_STATIC


# ──────────────────────────────────────────────────────────────────
# 고정장애물 / 추월 (규정 p.30~p.34)
# ──────────────────────────────────────────────────────────────────
def test_static_obstacle_enters_avoid():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(4):
        fsm.step(
            make_obs(
                t + 0.1 * i,
                front_target=stationary_target(1.0, ego_mps=1.2),
                ego_mps=1.2,
            ),
            ctx,
            READY,
        )
    assert fsm.state is Mode.AVOID_STATIC


def test_moving_vehicle_enters_overtake():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(4):
        fsm.step(
            make_obs(
                t + 0.1 * i,
                front_target=moving_target(1.0, ego_mps=1.2),
                ego_mps=1.2,
            ),
            ctx,
            READY,
        )
    assert fsm.state is Mode.OVERTAKE


def test_static_upgrades_to_overtake_when_it_moves():
    """고장난 차량인 줄 알았는데 움직이면 추월 규정(p.33)이 적용된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(4):
        fsm.step(
            make_obs(
                t + 0.1 * i,
                front_target=stationary_target(1.0, ego_mps=1.2),
                ego_mps=1.2,
            ),
            ctx,
            READY,
        )
    assert fsm.state is Mode.AVOID_STATIC

    fsm.step(
        make_obs(
            t + 1.0, front_target=moving_target(1.0, ego_mps=1.2), ego_mps=1.2
        ),
        ctx,
        READY,
    )
    assert fsm.state is Mode.OVERTAKE


def test_maneuver_returns_to_lane_when_done():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(4):
        fsm.step(
            make_obs(t + 0.1 * i, front_target=moving_target(1.0), ego_mps=1.2),
            ctx,
            READY,
        )
    assert fsm.state is Mode.OVERTAKE

    fsm.maneuver_done = True
    fsm.step(make_obs(t + 1.0), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE


# ──────────────────────────────────────────────────────────────────
# 안전 (규정 p.21 — 멈추면 실격)
# ──────────────────────────────────────────────────────────────────
def test_camera_loss_degrades_before_stopping():
    """카메라가 끊겨도 즉시 정지하지 않는다. 정지가 실격에 더 가깝다."""
    mon = SafetyMonitor()
    obs_ok = make_obs(0.0)
    mon.evaluate(obs_ok, racing=True)

    d = mon.evaluate(make_obs(0.5, camera_ok=False), racing=True)
    assert d.level is SafetyLevel.DEGRADED
    assert not d.must_stop

    d = mon.evaluate(make_obs(20.0, camera_ok=False), racing=True)
    assert d.level is SafetyLevel.STOP


def test_lidar_loss_never_stops():
    """라이다가 죽어도 차선주행은 된다. 회피만 못 할 뿐 정지 사유가 아니다."""
    mon = SafetyMonitor()
    mon.evaluate(make_obs(0.0), racing=True)
    d = mon.evaluate(make_obs(60.0, lidar_ok=False), racing=True)
    assert not d.must_stop


def test_safety_stop_wins_over_everything():
    fsm, ctx = RaceFSM(), RaceContext()
    _start(fsm, ctx)
    stop = SafetyDecision(SafetyLevel.STOP, "camera dead", True)
    fsm.step(make_obs(10.0), ctx, stop)
    assert fsm.state is Mode.STOP
    assert "camera dead" in ctx.stop_reason


def test_stop_recovers_when_camera_returns():
    """규정 p.21: 1분 내 재개 못하면 실격 — STOP 은 터미널이 아니라 복귀한다.

    카메라 12초 사망으로 섰다가 카메라가 돌아오는 시나리오. 디바운스
    (기본 5프레임 + 0.5s) 를 채우면 차선주행으로 복귀해야 한다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    stop = SafetyDecision(SafetyLevel.STOP, "camera dead", False)
    fsm.step(make_obs(t, camera_ok=False), ctx, stop)
    assert fsm.state is Mode.STOP

    # 한두 프레임 회복으로는 복귀하지 않는다 (깜빡 회복 방지)
    fsm.step(make_obs(t + 1.0), ctx, READY)
    assert fsm.state is Mode.STOP

    for i in range(2, 10):
        fsm.step(make_obs(t + 1.0 + 0.1 * i), ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE
    assert ctx.stop_reason == ""


def test_stop_recovery_debounce_restarts_on_relapse():
    """회복 도중 카메라가 다시 죽으면 디바운스는 처음부터 다시 센다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    stop = SafetyDecision(SafetyLevel.STOP, "camera dead", False)
    fsm.step(make_obs(t, camera_ok=False), ctx, stop)
    assert fsm.state is Mode.STOP

    for i in range(3):                      # 3프레임 회복
        fsm.step(make_obs(t + 0.1 * (i + 1)), ctx, READY)
    fsm.step(make_obs(t + 0.5, camera_ok=False), ctx, stop)   # 재사망
    fsm.step(make_obs(t + 0.6), ctx, READY)                   # 1프레임 회복
    assert fsm.state is Mode.STOP, "카운트를 이어받아 조기 복귀하면 안 된다"


def test_stale_green_left_during_camera_loss_not_latched():
    """카메라가 죽어 동결된 GREEN_LEFT 프레임은 래치를 갱신하면 안 된다 (리뷰 C5).

    (실차에서는 race_node 가 신선하지 않은 신호를 빈 값으로 강등하지만,
    FSM 자체도 camera_ok 없는 프레임의 좌회전을 믿지 않아야 한다 — 이중 방어.)
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)

    # 카메라 죽은 채 동결된 GREEN_LEFT 가 계속 들어온다 (래치 시간창 안)
    for i in range(30):
        fsm.step(
            make_obs(
                t + 0.1 * i,
                signal=SignalState.GREEN_LEFT,
                gantry_span=40,
                camera_ok=False,
            ),
            ctx,
            DEGRADED_NO_CAM,
        )
    t += 3.0

    t = _approach_signal(fsm, ctx, t, SignalState.GREEN)   # 실제 신호는 직진
    assert fsm.state is not Mode.SHORTCUT
    assert ctx.shortcut_used is False


def test_stop_recovers_even_with_limping_camera():
    """~2Hz 절뚝 카메라(스트릭 0.35s 미만)에서도 STOP 에 고착되지 않는다 (리뷰 A5).

    복귀 신호는 원시 camera_ok 가 아니라 'safety 가 STOP 을 거둔 상태'다 —
    복귀 후 부실 카메라는 DEGRADED 크립이 받는다. 정지 1분 = 실격이다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    stop = SafetyDecision(SafetyLevel.STOP, "camera dead", False)
    fsm.step(make_obs(t, camera_ok=False), ctx, stop)
    assert fsm.state is Mode.STOP

    # 카메라는 여전히 절뚝(camera_ok=False)이지만 safety 는 STOP 을 거둠
    for i in range(1, 10):
        fsm.step(make_obs(t + 0.1 * i, camera_ok=False), ctx, DEGRADED_NO_CAM)
    assert fsm.state is Mode.LANE_DRIVE


def test_finish_stays_terminal():
    """FINISH 만은 터미널 — 3바퀴를 마친 차가 다시 달리면 안 된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for _ in range(3):
        t = _lap_away(fsm, ctx, t)
        t = _approach_signal(fsm, ctx, t, SignalState.GREEN)
    assert fsm.state is Mode.FINISH

    for i in range(20):
        fsm.step(make_obs(t + 0.1 * i), ctx, READY)
    assert fsm.state is Mode.FINISH

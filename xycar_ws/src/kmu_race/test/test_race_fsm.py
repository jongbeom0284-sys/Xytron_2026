"""FSM 전이 검증 — 규정 조항을 그대로 테스트로 옮긴 것.

실차 없이 여기서 통과해야 트랙에 나갈 자격이 있다.
"""

from conftest import approach, make_obs, moving_target, stationary_target

from kmu_race.core.race_context import RaceContext
from kmu_race.core.race_fsm import FSMConfig, Mode, RaceFSM
from kmu_race.core.safety import SafetyDecision, SafetyLevel, SafetyMonitor
from kmu_race.core.types import SignalState

READY = SafetyDecision(SafetyLevel.OK, "", True)
NOT_READY = SafetyDecision(SafetyLevel.OK, "waiting", False)
DEGRADED_NO_CAM = SafetyDecision(SafetyLevel.DEGRADED, "camera stale", False)


def _bend_lane(obs, slope):
    """차로가 전방에서 옆으로 휘는 관측으로 바꾼다 (차선기준 코리도 검증용)."""
    from dataclasses import replace
    return replace(obs, lane=replace(obs.lane, heading_slope=slope))


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


def _sight_then_silent(fsm, ctx, t, silent_frames=6, step=0.1):
    """겐트리를 분류로 한 번 잡은 뒤(=랩 목격), 결정 창에서는 판독이 침묵.

    2026-08-20 랩 마커 재정의 이후의 현실적 시퀀스다. 마커는 '구조 검증된
    하우징 + 점등 검증'을 요구하므로(types.SignalReading.lap_marker_px),
    블롭만 보이는 UNKNOWN 프레임으로는 랩이 서지 않는다. 실측상 접근 중
    분류 성공률은 가까울수록 높고(77%) 멀어지면 떨어지므로, '접근에서
    잡고 이후 놓친다'가 이 검출기의 실제 거동이다.
    """
    for i in range(5):
        fsm.step(
            make_obs(t + step * i, signal=SignalState.GREEN, gantry_span=40),
            ctx, READY,
        )
    t += step * 5
    for i in range(silent_frames):
        fsm.step(
            make_obs(t + step * i, signal=SignalState.UNKNOWN, gantry_span=0),
            ctx, READY,
        )
    return t + step * silent_frames


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
    for dt, tgt in approach(stationary_target):
        fsm.step(
            make_obs(
                t + 5.2 + dt,
                front_target=tgt,
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
    for dt, tgt in approach(stationary_target):
        fsm.step(make_obs(t + dt, front_target=tgt, ego_mps=1.2), ctx, READY)
    assert fsm.state is Mode.AVOID_STATIC


def test_passing_object_does_not_enter_avoid():
    """정면을 스쳐가기만 하는 물체(=코너 바깥벽·의자)로는 회피가 서면 안 된다.

    0820 실측: 옛 게이트(연속 2틱)로는 이번 주행 한 바퀴에 회피가 45~65회
    걸렸고 그 대부분이 코너 벽이었다. 진짜 장애물은 '계속 막으면서 가까워
    진다' — 그 물리를 게이트로 옮겼다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    # 1.2m 에 계속 있지만 가까워지지는 않는다 (2초 = 옛 게이트의 50배)
    for i in range(20):
        fsm.step(
            make_obs(t + 0.1 * i, front_target=stationary_target(1.2, ego_mps=1.2),
                     ego_mps=1.2),
            ctx, READY,
        )
    assert fsm.state is Mode.LANE_DRIVE, (
        f"접근하지 않는 물체로 {fsm.state.value} 로 갔다")


def test_wall_outside_lane_does_not_enter_avoid():
    """차로가 휘는 코너에서 정면에 있는 '차로 밖' 벽은 장애물이 아니다.

    FSMConfig.lane_corridor_half_m 주석의 실측 근거 — 코리도가 차로보다
    넓어 코너 바깥벽이 구조적으로 들어오던 경로의 회귀 테스트."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for dt, tgt in approach(stationary_target):
        obs = make_obs(t + dt, front_target=tgt, ego_mps=1.2)
        # 차로가 전방에서 오른쪽으로 크게 휜다 → 정면 물체는 차로 밖
        obs = _bend_lane(obs, slope=0.55)
        fsm.step(obs, ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE, (
        f"차로 밖 벽으로 {fsm.state.value} 로 갔다")


def test_moving_vehicle_enters_overtake():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for dt, tgt in approach(moving_target):
        fsm.step(make_obs(t + dt, front_target=tgt, ego_mps=1.2), ctx, READY)
    assert fsm.state is Mode.OVERTAKE


def test_static_upgrades_to_overtake_when_it_moves():
    """고장난 차량인 줄 알았는데 움직이면 추월 규정(p.33)이 적용된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for dt, tgt in approach(stationary_target):
        fsm.step(make_obs(t + dt, front_target=tgt, ego_mps=1.2), ctx, READY)
    assert fsm.state is Mode.AVOID_STATIC

    fsm.step(
        make_obs(
            t + 2.0, front_target=moving_target(0.55, ego_mps=1.2), ego_mps=1.2
        ),
        ctx,
        READY,
    )
    assert fsm.state is Mode.OVERTAKE


def test_cone_section_fills_lap_when_gantry_and_imu_are_both_dead():
    """겐트리도 IMU 도 못 쓰면 라바콘 구간 통과가 랩을 보충한다.

    0820 밤: 본선 설정 실주행에서 목격 채널(초록 램프 2프레임)과 요 채널
    (IMU 사망)이 **둘 다** 죽어 lap 이 1/3 에 고정됐다. 같은 주행에서
    라바콘 구간은 두 번 다 16.9초씩 깨끗이 잡혔다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    assert ctx.lap.sightings == 1

    # 목격도 요도 없는 채로 한참 달린다 (min_gap_s 15초를 넘긴다)
    for i in range(200):
        fsm.step(make_obs(t + 0.1 * i, gantry_span=0), ctx, READY)
    t += 20.0
    assert ctx.lap.sightings == 1, "아직 콘 구간을 안 지났다"

    # 라바콘 구간을 8초 지난다 (최소 체류 5초 초과)
    for i in range(80):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE
    t += 8.0
    # 구간을 빠져나온다
    for i in range(50):
        fsm.step(make_obs(t + 0.1 * i, cone_px=0), ctx, READY)

    assert ctx.lap.sightings == 2, f"콘 보충 랩이 안 섰다 (sightings={ctx.lap.sightings})"
    assert ctx.lap.cone_filled == 1


def test_short_cone_glitch_does_not_fill_lap():
    """0.8초짜리 가짜 콘 진입으로는 랩이 오르면 안 된다 (실주행 t41.2 실측)."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(200):
        fsm.step(make_obs(t + 0.1 * i, gantry_span=0), ctx, READY)
    t += 20.0
    before = ctx.lap.sightings
    # 0.8초만 콘 구간
    for i in range(8):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    t += 0.8
    for i in range(50):
        fsm.step(make_obs(t + 0.1 * i, cone_px=0), ctx, READY)
    assert ctx.lap.sightings == before, "짧은 오진입으로 랩이 올랐다"


def test_maneuver_timeout_does_not_relimit_cycle():
    """타임아웃 탈출은 '통과 완료'가 아니다 — 곧바로 재진입하면 리밋사이클.

    0820 감사: target_passing 이 6초 타임아웃에서 done=True 를 성공과 똑같이
    냈고, FSM 이 LANE_DRIVE 로 돌린 다음 틱에 타겟이 그대로면 재진입했다.
    이제 aborted 를 구분해 쿨다운을 건다 (FSMConfig.maneuver_cooldown_s).
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for dt, tgt in approach(stationary_target):
        fsm.step(make_obs(t + dt, front_target=tgt, ego_mps=1.2), ctx, READY)
    assert fsm.state is Mode.AVOID_STATIC

    # 타임아웃 탈출
    fsm.maneuver_done = True
    fsm.maneuver_aborted = True
    fsm.step(make_obs(t + 2.0, front_target=stationary_target(0.55), ego_mps=1.2),
             ctx, READY)
    assert fsm.state is Mode.LANE_DRIVE

    # 타겟이 그대로 있어도 쿨다운 동안은 재진입하지 않는다
    for i in range(20):
        fsm.step(
            make_obs(t + 2.1 + 0.05 * i,
                     front_target=stationary_target(0.6 - 0.002 * i), ego_mps=1.2),
            ctx, READY,
        )
    assert fsm.state is Mode.LANE_DRIVE, (
        f"타임아웃 직후 {fsm.state.value} 로 재진입했다 (리밋사이클)")


def test_maneuver_returns_to_lane_when_done():
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for dt, tgt in approach(moving_target):
        fsm.step(make_obs(t + dt, front_target=tgt, ego_mps=1.2), ctx, READY)
    assert fsm.state is Mode.OVERTAKE

    fsm.maneuver_done = True
    fsm.step(make_obs(t + 2.0), ctx, READY)
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


# ──────────────────────────────────────────────────────────────────
# 0818 저녁 — 라이다 게이트 통합 / 콘 구간 입구의 회피 오발동
# ──────────────────────────────────────────────────────────────────
def _with_lidar_gate(obs, depth_m=0.9, width_m=0.9):
    from dataclasses import replace
    return replace(
        obs,
        cone=replace(
            obs.cone, lidar_gate_valid=True, lidar_gate_depth_m=depth_m,
            lidar_gate_width_m=width_m, lidar_cones=2,
        ),
    )


def test_cone_section_survives_on_lidar_gate_alone():
    """카메라 게이트가 4s 넘게 안 서도 **라이다 게이트**가 살아 있으면 구간에
    머문다. policy 는 라이다를 1순위로 조향 중인데 FSM 이 카메라만 보고
    이탈시키면 슬라럼 한복판에서 콘에 오염된 차선 추종으로 떨어진다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE
    # 카메라 게이트 없음(gate_valid=False), 라이다 게이트만 6초 동안
    for i in range(60):
        obs = _with_lidar_gate(make_obs(t + 0.5 + 0.1 * i, cone_px=3000))
        fsm.step(obs, ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE, "라이다 게이트가 살아 있는데 구간을 이탈했다"


def test_cone_entry_accepts_lidar_gate():
    """진입 게이트 조건은 카메라 또는 라이다 어느 쪽이든 만족하면 된다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(_with_lidar_gate(make_obs(t + 0.1 * i, cone_px=5000)), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE


def test_cone_gate_suppresses_front_target_trigger_in_lane_drive():
    """콘 게이트가 보이는 동안 전방 타겟(콘!)으로 AVOID 가 서면 안 된다.

    통로 0.91m 의 콘은 nearest_front 코리도(±0.45m)·1.45m 안에 수시로 들어와
    2틱이면 회피가 서는데 콘 진입(3틱+게이트)보다 빠르다 — 라바콘 입구에서
    회피 기동으로 콘 열에 처박히는 경로."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    # 게이트는 보이지만 아직 진입 디바운스 전(첫 틱) + 콘이 전방 1.2m 코리도 안
    obs = _with_lidar_gate(
        make_obs(t, cone_px=2000, front_target=stationary_target(1.2, y=0.3))
    )
    fsm.step(obs, ctx, READY)
    fsm.step(_with_lidar_gate(
        make_obs(t + 0.02, cone_px=2000, front_target=stationary_target(1.2, y=0.3))
    ), ctx, READY)
    assert fsm.state not in (Mode.AVOID_STATIC, Mode.OVERTAKE), (
        f"콘 게이트가 보이는데 {fsm.state.value} 로 갔다")

    # 대조: 게이트가 없으면 같은 타겟에 정상 반응해야 한다
    fsm2, ctx2 = RaceFSM(), RaceContext()
    t2 = _start(fsm2, ctx2)
    for dt, tgt in approach(stationary_target, y=0.2):
        fsm2.step(make_obs(t2 + dt, front_target=tgt, ego_mps=1.2), ctx2, READY)
    assert fsm2.state in (Mode.AVOID_STATIC, Mode.OVERTAKE)


# ──────────────────────────────────────────────────────────────────
# 규정 추론 — 좌회전은 랩2·랩3 중 정확히 한 번 (규정 p.35)
# ──────────────────────────────────────────────────────────────────
def test_lap2_clear_observation_rules_the_arrow_into_lap3():
    """랩2 에서 겐트리를 또렷이 보고도 직좌가 아니었으면 좌회전은 랩3 확정."""
    ctx = RaceContext()
    ctx.lap.sightings = 2                      # 랩2 주행 중
    for _ in range(8):
        ctx.note_lap2_gantry_frame(gantry_clear=True, saw_left=False)
    ctx.lap.sightings = 3                      # 랩3 목격 — 창이 닫힌다
    ctx.conclude_lap2_observation(min_frames=8, now=1.0)
    assert ctx.arrow_ruled_out_on_lap2


def test_seeing_left_on_lap2_invalidates_the_negative_observation():
    """랩2 에 직좌를 한 프레임이라도 봤으면 음성 관측이 아니다."""
    ctx = RaceContext()
    ctx.lap.sightings = 2
    for _ in range(8):
        ctx.note_lap2_gantry_frame(gantry_clear=True, saw_left=False)
    ctx.note_lap2_gantry_frame(gantry_clear=True, saw_left=True)
    ctx.lap.sightings = 3
    ctx.conclude_lap2_observation(min_frames=8, now=1.0)
    assert not ctx.arrow_ruled_out_on_lap2


def test_blind_lap2_does_not_rule_the_arrow_into_lap3():
    """겐트리를 못 본 랩2 는 '보고서 아니었다'가 아니다 — 추론 불가."""
    ctx = RaceContext()
    ctx.lap.sightings = 2
    for _ in range(20):
        ctx.note_lap2_gantry_frame(gantry_clear=False, saw_left=False)
    ctx.lap.sightings = 3
    ctx.conclude_lap2_observation(min_frames=8, now=1.0)
    assert not ctx.arrow_ruled_out_on_lap2


def test_lap3_inference_is_sealed_by_default():
    """추론 스위치가 꺼져 있으면 검출 침묵 시 지름길로 들어가지 않는다."""
    ctx = RaceContext()
    ctx.lap.sightings = 3
    ctx.arrow_ruled_out_on_lap2 = True
    assert not ctx.may_take_shortcut(False)
    assert not ctx.may_take_shortcut(False, infer_lap3=False)


def test_lap3_inference_when_armed_enters_without_a_reading():
    ctx = RaceContext()
    ctx.lap.sightings = 3
    ctx.arrow_ruled_out_on_lap2 = True
    assert ctx.may_take_shortcut(False, infer_lap3=True)


def test_lap3_inference_never_fires_on_lap2():
    """추론은 랩3 전용 — 랩2 에서는 실제 판독만 받는다."""
    ctx = RaceContext()
    ctx.lap.sightings = 2
    ctx.arrow_ruled_out_on_lap2 = True         # (모순 상태를 일부러 만들어도)
    assert not ctx.may_take_shortcut(False, infer_lap3=True)


def test_shortcut_still_locked_to_once():
    ctx = RaceContext()
    ctx.lap.sightings = 3
    ctx.arrow_ruled_out_on_lap2 = True
    ctx.shortcut_used = True
    assert not ctx.may_take_shortcut(True, infer_lap3=True)


def test_lap2_green_only_approach_rules_arrow_into_lap3_end_to_end():
    """FSM 통합: 랩2 접근을 GREEN 으로 보내면 랩3 목격에서 결론이 선다."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN, frames=12)
    assert ctx.lap2_clear_frames >= 8, ctx.lap2_clear_frames
    assert not ctx.arrow_ruled_out_on_lap2, "랩2 창이 아직 안 닫혔다"

    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN, frames=2)
    assert ctx.lap.current_lap == 3
    assert ctx.arrow_ruled_out_on_lap2


def test_lap3_silent_detector_stays_straight_when_inference_sealed():
    """기본값(봉인): 랩3 에서 판독이 없으면 직진한다 — 현행 동작 유지."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN, frames=12)
    t = _lap_away(fsm, ctx, t)
    t = _sight_then_silent(fsm, ctx, t)

    assert ctx.lap.current_lap == 3
    assert ctx.shortcut_used is False
    assert fsm.state is Mode.LANE_DRIVE


def test_lap3_silent_detector_takes_shortcut_when_inference_armed():
    """무장 시: 랩2 음성 관측이 섰으면 랩3 판독이 침묵해도 진입한다."""
    fsm = RaceFSM(FSMConfig(lap3_infer_enabled=True))
    ctx = RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    t = _approach_signal(fsm, ctx, t, SignalState.GREEN, frames=12)
    t = _lap_away(fsm, ctx, t)
    t = _sight_then_silent(fsm, ctx, t)

    assert ctx.lap.current_lap == 3
    assert fsm.state is Mode.SHORTCUT
    assert ctx.shortcut_used is True


def test_armed_inference_does_not_fire_if_lap2_was_blind():
    """무장돼 있어도 랩2 에서 겐트리를 못 봤으면 랩3 진입은 없다."""
    fsm = RaceFSM(FSMConfig(lap3_infer_enabled=True))
    ctx = RaceContext()
    t = _start(fsm, ctx)
    t = _lap_away(fsm, ctx, t)
    # 랩2 목격은 확정되되(span 40 을 confirm_s 만큼) 이후 결정 창에서는
    # 겐트리가 곧바로 시야에서 사라진다 — '보지 못한 랩2'.
    for i in range(5):
        fsm.step(
            make_obs(t + 0.1 * i, signal=SignalState.GREEN, gantry_span=40),
            ctx, READY,
        )
    for i in range(5, 14):
        fsm.step(
            make_obs(t + 0.1 * i, signal=SignalState.UNKNOWN, gantry_span=0),
            ctx, READY,
        )
    t += 1.4
    assert ctx.lap.current_lap == 2, "랩2 목격 자체는 성립해야 하는 시나리오"
    assert ctx.lap2_clear_frames < 8, ctx.lap2_clear_frames
    t = _lap_away(fsm, ctx, t)
    t = _sight_then_silent(fsm, ctx, t)

    assert ctx.lap.current_lap == 3
    assert not ctx.arrow_ruled_out_on_lap2
    assert ctx.shortcut_used is False


def _cone_obs(t, *, orange_px, lidar_cones):
    """라바콘 구간용 관측 — 카메라 주황과 라이다 콘 개수를 따로 준다."""
    from dataclasses import replace

    obs = make_obs(t, cone_px=orange_px)
    obs.cone = replace(obs.cone, lidar_cones=lidar_cones,
                       lidar_gate_valid=False, gate_valid=False)
    return obs


def test_cone_mode_does_not_hand_over_while_lidar_sees_cones():
    """★ 라이다가 콘을 보고 있으면 차선 조향으로 넘기지 않는다.

    실측(cone_0821_0344/0323): 차선 주행 프레임의 **31~33%** 에서 라이다가
    콘을 3개 이상(많게는 14개) 보고 있었다. t=13.1s 에는 콘 11개가 보이는데
    차선을 따라 +42(풀락)로 꺾었다. 각진 벽 근처에서 차선 검출이 헛것을
    잡으면 그대로 코스를 이탈한다 — 사용자 목격 그대로다.
    """
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE

    # 주황은 사라졌지만 라이다는 콘 8개를 본다 → 나가면 안 된다
    t += 1.0
    for i in range(60):
        fsm.step(_cone_obs(t + 0.05 * i, orange_px=0, lidar_cones=8), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE, "라이다가 콘 8개를 보는데 나가버렸다"

    # 라이다도 비면 나간다
    t += 3.0
    for i in range(60):
        fsm.step(_cone_obs(t + 0.05 * i, orange_px=0, lidar_cones=0), ctx, READY)
    assert fsm.state is not Mode.CONE_DRIVE, "콘이 다 없어졌는데 안 나간다"


def test_cone_mode_cannot_get_stuck_forever():
    """라이다에 콘이 계속 잡혀도 상한 시간이 지나면 나간다 (갇힘 = 실격)."""
    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(5):
        fsm.step(make_obs(t + 0.1 * i, cone_px=5000, gate_valid=True), ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE
    limit = fsm.config.cone_max_dwell_s
    t += 1.0
    for i in range(int((limit + 5.0) / 0.1)):
        fsm.step(_cone_obs(t + 0.1 * i, orange_px=0, lidar_cones=9), ctx, READY)
    assert fsm.state is not Mode.CONE_DRIVE, (
        f"{limit}초가 넘었는데 라바콘 구간에 갇혀 있다")


def test_cone_mode_enters_on_lidar_without_camera():
    """★ 카메라 주황이 없어도 라이다가 통로를 보면 진입한다.

    옛 코드는 주황 픽셀이 병목이라 (1) 라이다 콘 0개인데 진입하거나
    (2) 진입이 늦었다. 실측(cone_0821_0356 t=32.3): 최근접 콘 47cm 에서
    진입했는데 콘은 0.69초 전 95cm 에서 이미 보였다.
    진입이 늦으면 정렬할 거리가 없다 — 실측 진입각 중앙 13.5°, 38%가 20°
    초과인데, R1.5 통로에서 20°를 지우려면 1.11m 가 필요하다(여유 18.5cm).
    """
    from dataclasses import replace

    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(8):
        obs = make_obs(t + 0.1 * i, cone_px=0)          # 카메라는 아무것도 못 봄
        obs.cone = replace(obs.cone, lidar_cones=6, lidar_gate_valid=True,
                           lidar_gate_width_m=0.82, lidar_gate_depth_m=0.9)
        fsm.step(obs, ctx, READY)
    assert fsm.state is Mode.CONE_DRIVE, "라이다가 통로를 보는데 진입하지 않는다"


def test_cone_mode_does_not_enter_on_a_lone_pile():
    """벽면 보관 무더기(게이트 없음)로는 진입하지 않는다."""
    from dataclasses import replace

    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(20):
        obs = make_obs(t + 0.1 * i, cone_px=0)
        obs.cone = replace(obs.cone, lidar_cones=9, lidar_gate_valid=False)
        fsm.step(obs, ctx, READY)
    assert fsm.state is not Mode.CONE_DRIVE, "게이트가 없는데 진입했다"


def test_cone_mode_never_enters_with_no_lidar_cones():
    """★ 라이다에 콘이 하나도 없으면 카메라가 뭘 보든 진입하지 않는다.

    실측(cone_0821_0443): CONE_DRIVE 진입 10회 중 **4회가 라이다 콘 0개**
    였다. 통로가 안 보이는데 콘 모드로 들어가면 따라갈 게 없어 조향이
    폴백(직전 목표 유지)으로 흐른다.
    """
    from dataclasses import replace

    fsm, ctx = RaceFSM(), RaceContext()
    t = _start(fsm, ctx)
    for i in range(20):
        obs = make_obs(t + 0.1 * i, cone_px=9000, gate_valid=True)  # 카메라는 확신
        obs.cone = replace(obs.cone, lidar_cones=0, lidar_gate_valid=False)
        fsm.step(obs, ctx, READY)
    assert fsm.state is not Mode.CONE_DRIVE, (
        "라이다에 콘이 0개인데 라바콘 구간으로 들어갔다")

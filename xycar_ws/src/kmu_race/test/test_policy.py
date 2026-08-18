"""정책 계층 + 통합 시뮬레이션.

FSM 이 '무엇을' 하는지는 test_race_fsm.py 가 본다. 여기서는 '그래서 바퀴가
어떻게 도는지' — 실제로 발행될 DriveCommand 를 끝단까지 검증한다.
"""

from conftest import make_obs, moving_target

from kmu_race.core.policy import DrivePolicy
from kmu_race.core.race_context import TIME_LIMIT_S, RaceContext
from kmu_race.core.race_fsm import Mode, RaceFSM
from kmu_race.core.safety import SafetyDecision, SafetyLevel
from kmu_race.core.speed_policy import SpeedPolicy
from kmu_race.core.types import SignalState

READY = SafetyDecision(SafetyLevel.OK, "", True)
DEGRADED = SafetyDecision(SafetyLevel.DEGRADED, "camera stale", True)


# ──────────────────────────────────────────────────────────────────
# 기본 동작
# ──────────────────────────────────────────────────────────────────
def test_waiting_at_light_is_stopped():
    """규정 p.19: 신호등 앞에 정차하여 대기."""
    pol = DrivePolicy()
    ctx = RaceContext()
    cmd = pol.update(Mode.WAIT_GREEN, make_obs(0.0), ctx, READY)
    assert cmd.speed == 0.0
    assert cmd.steer == 0.0


def test_init_commands_zero():
    """런치 직후(INIT)에는 무조건 정지 명령이어야 한다.

    회귀 테스트: INIT 이 기본 분기(차선주행)로 떨어져 런치 직후 1~2초간
    v=17.7 이 발행되는 결함이 실차 스모크 테스트에서 발견됐다 (2026-08-13).
    카메라가 차선을 보고 있어도 출발 전에는 바퀴가 돌면 안 된다.
    """
    pol = DrivePolicy()
    ctx = RaceContext()
    cmd = pol.update(Mode.INIT, make_obs(0.0, lane_offset=+120.0), ctx, READY)
    assert cmd.speed == 0.0
    assert cmd.steer == 0.0


def test_finish_commands_full_stop():
    pol = DrivePolicy()
    ctx = RaceContext()
    cmd = pol.update(Mode.FINISH, make_obs(0.0), ctx, READY)
    assert cmd.speed == 0.0


def test_lane_drive_steers_toward_center():
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 0.0

    # 목표선이 오른쪽에 있으면 우조향(양수)이 나와야 한다.
    # (출발 직진 홀드 1.2s 를 지난 시점으로 관측을 준다)
    cmd = pol.update(Mode.LANE_DRIVE, make_obs(5.0, lane_offset=+120.0), ctx, READY)
    assert cmd.steer > 0.0

    pol2 = DrivePolicy()
    cmd2 = pol2.update(
        Mode.LANE_DRIVE, make_obs(5.0, lane_offset=-120.0), ctx, READY
    )
    assert cmd2.steer < 0.0


def test_start_straight_hold_then_steer():
    """출발 직후엔 겐트리 지주가 차선으로 오검되기 쉽다 — 잠깐 직진을 유지한다."""
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 100.0

    held = pol.update(
        Mode.LANE_DRIVE, make_obs(100.5, lane_offset=+120.0), ctx, READY
    )
    assert held.steer == 0.0, "홀드 중에는 조향 중립"

    held2 = pol.update(
        Mode.LANE_DRIVE, make_obs(100.6, lane_offset=+120.0), ctx, READY
    )
    assert held2.steer == 0.0
    assert held2.speed > 0.0, "홀드는 조향만 잡는다 — 속도는 정상 램프를 탄다"

    after = pol.update(
        Mode.LANE_DRIVE, make_obs(105.0, lane_offset=+120.0), ctx, READY
    )
    assert after.steer > 0.0, "홀드가 끝나면 즉시 차선 추종으로 돌아온다"


def test_shortcut_steers_left():
    """지름길 진입은 좌회전. 조향 부호 규약상 음수여야 한다."""
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 0.0
    cmd = pol.update(Mode.SHORTCUT, make_obs(1.0), ctx, READY)
    assert cmd.steer < 0.0


def test_cone_gate_steering_follows_center():
    """게이트 중앙이 왼쪽이면 좌조향."""
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 0.0
    cmd = pol.update(
        Mode.CONE_DRIVE,
        make_obs(1.0, cone_px=6000, gate_center=-140.0, gate_valid=True),
        ctx,
        READY,
    )
    assert cmd.steer < 0.0

    pol2 = DrivePolicy()
    cmd2 = pol2.update(
        Mode.CONE_DRIVE,
        make_obs(1.0, cone_px=6000, gate_center=+140.0, gate_valid=True),
        ctx,
        READY,
    )
    assert cmd2.steer > 0.0


def test_degraded_mode_creeps_instead_of_stopping():
    """규정 p.21: 멈추면 실격에 가까워진다. 카메라가 상해도 기어간다."""
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 0.0
    cmd = None
    for i in range(40):
        cmd = pol.update(
            Mode.LANE_DRIVE,
            make_obs(1.0 + 0.05 * i, camera_ok=False),
            ctx,
            DEGRADED,
        )
    assert cmd.speed > 0.0, "열화 상태에서도 멈추면 안 된다"
    assert cmd.speed <= pol.config.speed.degraded_speed + 1e-6


# ──────────────────────────────────────────────────────────────────
# 속도 정책 — 4분 예산 (규정 p.37)
# ──────────────────────────────────────────────────────────────────
def test_speed_ramps_up_not_instant():
    sp = SpeedPolicy()
    v0 = sp.step(20.0, 0.0)
    v1 = sp.step(20.0, 0.02)
    assert v0 == 0.0
    assert 0.0 < v1 < 20.0, "급발진 금지 — 램프를 타야 한다"


def test_curve_slows_down():
    sp = SpeedPolicy()
    straight = sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=0.0)
    corner = sp.target_for("LANE_DRIVE", steer_deg=40.0, time_pressure=0.0)
    assert corner < straight


def test_time_pressure_raises_lane_speed():
    sp = SpeedPolicy()
    relaxed = sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=0.0)
    hurried = sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=1.0)
    assert hurried > relaxed


def test_override_bypasses_min_speed_floor():
    """추돌 방지 서행 오버라이드(3.3 등)를 min_speed=8.0 바닥이 끌어올리면
    0.55m 앞 정지 차량에 2배 속도로 접근한다 (리뷰 C1 회귀)."""
    sp = SpeedPolicy()
    assert sp.target_for("AVOID_STATIC", steer_deg=0.0, time_pressure=0.0,
                         override=3.3) == 3.3
    # 일반 주행은 여전히 바닥이 적용된다
    assert sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=0.0) >= 8.0


def test_clamped_rejects_nan():
    """NaN 은 min/max 를 통과해 최대값으로 둔갑한다 — clamped 가 걸러야 한다
    (리뷰 P1 회귀: 풀조향+풀스로틀 방지)."""
    from kmu_race.core.types import DriveCommand

    nan = float("nan")
    cmd = DriveCommand(nan, nan, "t").clamped(50.0, 50.0)
    assert cmd.steer == 0.0
    assert cmd.speed == 0.0
    inf = float("inf")
    cmd2 = DriveCommand(inf, -inf, "t").clamped(50.0, 50.0)
    assert cmd2.steer == 0.0
    assert cmd2.speed == 0.0


def test_passer_state_reset_on_reentry():
    """승격/STOP 으로 남은 기동 상태가 다음 진입에서 즉시 가짜 '통과 완료'를
    만들면 안 된다 (리뷰 C6 회귀)."""
    pol = DrivePolicy()
    ctx = RaceContext()
    ctx.race_started_at = 0.0

    # 1차 진입: SHIFT 시작 (타임아웃 시계 무장됨)
    pol.update(
        Mode.AVOID_STATIC,
        make_obs(5.0, front_target=moving_target(1.0, y=+0.3)),
        ctx, READY,
    )
    # 다른 모드로 이탈 (승격/정지 등가)
    pol.update(Mode.LANE_DRIVE, make_obs(6.0), ctx, READY)

    # 7초 뒤 재진입 — 이전 시계가 남아 있으면 첫 틱에 timeout done 이 나온다
    pol.update(
        Mode.AVOID_STATIC,
        make_obs(12.0, front_target=moving_target(1.0, y=+0.3)),
        ctx, READY,
    )
    assert pol.maneuver_done is False


def test_time_pressure_does_not_rush_missions():
    """미션 구간에서 서두르면 라바콘 3초·추돌 10초로 오히려 손해다."""
    sp = SpeedPolicy()
    relaxed = sp.target_for("CONE_DRIVE", steer_deg=0.0, time_pressure=0.0)
    hurried = sp.target_for("CONE_DRIVE", steer_deg=0.0, time_pressure=1.0)
    assert hurried == relaxed


def test_time_pressure_rises_as_clock_runs_out():
    ctx = RaceContext()
    ctx.race_started_at = 0.0
    ctx.lap.sightings = 1                      # 1바퀴째
    assert ctx.time_pressure(10.0) == 0.0      # 아직 여유

    late = ctx.time_pressure(TIME_LIMIT_S - 40.0)
    assert late > 0.5, "3바퀴 남았는데 40초면 압박이 커야 한다"


def test_remaining_time_reported():
    ctx = RaceContext()
    ctx.race_started_at = 100.0
    assert ctx.elapsed(160.0) == 60.0
    assert ctx.remaining(160.0) == TIME_LIMIT_S - 60.0


# ──────────────────────────────────────────────────────────────────
# 통합 — 3바퀴 완주 시뮬레이션
# ──────────────────────────────────────────────────────────────────
def test_full_race_never_commands_stop_while_racing():
    """규정 p.21 최중요: 주행 시작 후 정지 명령이 나가면 안 된다.

    지름길 1회 포함 3바퀴를 통째로 돌려 보고, FINISH 이전에 speed==0 이
    한 번이라도 나오는지 감시한다.
    """
    fsm, ctx, pol = RaceFSM(), RaceContext(), DrivePolicy()
    t = 0.0
    step = 0.05
    zero_speed_frames = 0
    finished_at = None

    def tick(obs):
        nonlocal zero_speed_frames
        fsm.maneuver_done = pol.maneuver_done
        fsm.step(obs, ctx, READY)
        cmd = pol.update(fsm.state, obs, ctx, READY)
        if fsm.state not in (Mode.INIT, Mode.WAIT_GREEN, Mode.FINISH, Mode.STOP):
            # 램프 초기 몇 프레임은 0에서 올라오므로 예외로 둔다.
            if cmd.speed <= 0.0 and ctx.elapsed(obs.now) > 1.0:
                zero_speed_frames += 1
        return cmd

    # 출발
    for _ in range(10):
        tick(make_obs(t, signal=SignalState.GREEN, gantry_span=40))
        t += step
    assert fsm.state is Mode.LANE_DRIVE

    for lap in range(3):
        # 미션 구간: 라바콘 → 고정장애물/추월 → 일반 주행
        for _ in range(60):
            tick(make_obs(t, cone_px=6000, gate_center=20.0, gate_valid=True))
            t += step
        for _ in range(40):
            tick(make_obs(t, cone_px=0))
            t += step
        for _ in range(60):
            tick(
                make_obs(
                    t,
                    front_target=moving_target(1.0, y=+0.35, ego_mps=1.1),
                    ego_mps=1.1,
                )
            )
            t += step
        for _ in range(200):
            tick(make_obs(t))
            t += step

        # 신호등 접근. 2랩째에 좌회전이 뜬다고 가정.
        sig = SignalState.GREEN_LEFT if lap == 0 else SignalState.GREEN
        for _ in range(8):
            tick(make_obs(t, signal=sig, gantry_span=40))
            t += step
        # 지름길 좌회전 소화
        if fsm.state is Mode.SHORTCUT:
            for i in range(40):
                tick(make_obs(t, yaw=-4.0 * i))
                t += step
        if fsm.state is Mode.FINISH:
            finished_at = ctx.elapsed(t)
            break

    assert fsm.state is Mode.FINISH, "3바퀴 후 종료해야 한다"
    assert ctx.shortcut_used is True
    assert zero_speed_frames == 0, (
        f"주행 중 정지 명령 {zero_speed_frames}프레임 — 규정 p.21 위반 위험"
    )
    assert finished_at is not None


def test_full_race_respects_speed_limits():
    fsm, ctx, pol = RaceFSM(), RaceContext(), DrivePolicy()
    t = 0.0
    max_seen = 0.0
    for _ in range(10):
        fsm.step(make_obs(t, signal=SignalState.GREEN, gantry_span=40), ctx, READY)
        pol.update(fsm.state, make_obs(t, signal=SignalState.GREEN), ctx, READY)
        t += 0.05
    for i in range(2000):
        obs = make_obs(t, lane_offset=60.0 * ((i // 50) % 3 - 1))
        fsm.maneuver_done = pol.maneuver_done
        fsm.step(obs, ctx, READY)
        cmd = pol.update(fsm.state, obs, ctx, READY)
        max_seen = max(max_seen, cmd.speed)
        assert abs(cmd.steer) <= pol.config.max_steer + 1e-6
        t += 0.05
    assert max_seen <= pol.config.speed.max_speed + 1e-6

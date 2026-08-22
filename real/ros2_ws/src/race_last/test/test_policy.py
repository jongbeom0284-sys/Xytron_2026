"""정책 계층 + 통합 시뮬레이션.

FSM 이 '무엇을' 하는지는 test_race_fsm.py 가 본다. 여기서는 '그래서 바퀴가
어떻게 도는지' — 실제로 발행될 DriveCommand 를 끝단까지 검증한다.
"""

from conftest import make_obs, moving_target

from race_last.kmu_race.core.policy import DrivePolicy
from race_last.kmu_race.core.race_context import TIME_LIMIT_S, RaceContext
from race_last.kmu_race.core.race_fsm import Mode, RaceFSM
from race_last.kmu_race.core.safety import SafetyDecision, SafetyLevel
from race_last.kmu_race.core.speed_policy import SpeedPolicy
from race_last.kmu_race.core.types import SignalState

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


def test_lookahead_does_not_jump():
    """전방주시가 프레임 사이에 튀면 안 된다 — 그게 조향 진동이었다.

    0821 실측(cone_0821_0037): 게이트가 끊겼다 붙으면 깊이가 급변해
    전방주시가 프레임간 최대 30.4cm 튀었고, 조향 점프 상위 10개 중 6개가
    그 급변과 겹쳤다. 근거는 PolicyConfig.cone_lookahead_rate_mps 주석.
    """
    pol = DrivePolicy()
    rate = pol.config.cone_lookahead_rate_mps
    t = 0.0
    for _ in range(20):                       # 깊은 게이트에서 안정
        t += 0.033
        la = pol._cone_lookahead(0.75, t)
    prev, worst = la, 0.0
    for _ in range(30):                       # 갑자기 코앞 게이트로 바뀐다
        t += 0.033
        la = pol._cone_lookahead(0.35, t)
        worst = max(worst, abs(la - prev))
        prev = la
    assert worst <= rate * 0.033 + 1e-6, (
        f"전방주시가 한 프레임에 {worst*100:.1f}cm 튀었다 "
        f"(상한 {rate*0.033*100:.1f}cm)")
    assert abs(prev - pol.config.cone_lookahead_min_m) < 0.03, (
        f"변화율 제한 때문에 하한에 도달 못 함 ({prev:.2f})")


def test_lookahead_ignores_lateral_offset():
    """★ 전방주시는 **횡편차와 무관**해야 한다.

    한때 '커브에서는 짧게' 를 넣었다가 뺐다. 게이트 횡편차는 이미 통로
    곡률을 담고 있어서(깊이 d, 통로반경 R_c 면 횡편차 = d²/2R_c),
    거기에 전방주시까지 줄이면 곡률을 **두 번 센다**.
    실측(manual_0821_0238, 사람이 통로 한가운데로 통과한 궤적):
        커브 적응  |조향| 중앙 42.0  포화 54.7%
        깊이 그대로 |조향| 중앙 35.3  포화 47.4%   (사람 조향은 중앙 0)
    """
    from race_last.kmu_race.core.race_context import RaceContext
    from race_last.kmu_race.core.types import ConeReading

    def settle(lateral_m):
        pol = DrivePolicy()
        ctx = RaceContext(); ctx.race_started_at = -10.0
        for i in range(80):
            obs = make_obs(0.033 * i)
            obs.cone = ConeReading(
                detected=True, gate_valid=False,
                lidar_gate_valid=True, lidar_gate_width_m=0.82,
                lidar_gate_depth_m=0.70, lidar_gate_lateral_m=lateral_m,
                lidar_cones=6)
            pol.update(Mode.CONE_DRIVE, obs, ctx, READY)
        return pol._cone_la

    straight = settle(0.0)
    offset = settle(0.30)          # 통로가 굽었거나 차가 치우쳤거나
    assert abs(offset - straight) < 0.01, (
        f"횡편차 30cm 를 줬더니 전방주시가 {straight*100:.0f} → {offset*100:.0f}cm "
        "로 바뀐다 — 곡률을 두 번 센다")
    assert abs(straight - 0.70) < 0.02, (
        f"게이트 깊이 70cm 인데 전방주시가 {straight*100:.0f}cm — 깊이를 안 따른다")


def test_cone_lookahead_follows_gate_depth():
    """전방주시는 관측된 게이트 깊이를 따른다 (상·하한 안에서)."""
    pol = DrivePolicy()
    cfg = pol.config
    for depth, want in ((0.30, cfg.cone_lookahead_min_m),
                        (0.60, 0.60),
                        (1.50, cfg.cone_lookahead_m)):
        got = pol._cone_lookahead(depth)          # now 없음 = 변화율 제한 우회
        assert abs(got - want) < 1e-6, f"깊이 {depth} → {got:.2f} (기대 {want:.2f})"


def test_near_cone_pushes_target_away():
    """차 옆 콘이 안전선보다 가까우면 목표를 반대쪽으로 민다.

    0821 실측(cone_0821_0025 t10.1): 게이트는 '횡편차 +36cm, 정상'이라 했는데
    같은 순간 **차 오른쪽 1cm 에 콘**이 있었다. 게이트는 앞쪽 쌍만 보고
    옆을 지나는 콘은 아무도 안 봤다. 근거는 PolicyConfig.cone_safe_clear_m.
    """
    from race_last.kmu_race.core.types import ConeReading
    pol = DrivePolicy()
    cfg = pol.config
    base = 0.0
    # 여유 있는 경우: 목표가 그대로
    far = ConeReading(lidar_near_left_m=0.40, lidar_near_right_m=0.44)
    assert abs(pol._cone_repel(far, base) - base) < 1e-9

    # 오른쪽 1cm — 왼쪽(+)으로 밀려야 한다
    tight_r = ConeReading(lidar_near_left_m=0.74, lidar_near_right_m=0.01)
    out_r = pol._cone_repel(tight_r, base)
    assert out_r > 0.0, f"우측 콘이 1cm 인데 좌로 안 민다 ({out_r:+.3f})"
    assert out_r <= cfg.cone_repel_max_m + 1e-9

    # 왼쪽 1cm — 오른쪽(−)으로
    tight_l = ConeReading(lidar_near_left_m=0.01, lidar_near_right_m=0.74)
    assert pol._cone_repel(tight_l, base) < 0.0

    # 양쪽 다 가까우면 상쇄되어 과하게 밀지 않는다
    both = ConeReading(lidar_near_left_m=0.05, lidar_near_right_m=0.05)
    assert abs(pol._cone_repel(both, base)) < 1e-9

    # 콘 정보가 없으면(inf) 아무 일도 없다
    none = ConeReading()
    assert abs(pol._cone_repel(none, 0.12) - 0.12) < 1e-9


def test_cone_section_never_falls_back_to_lane_steering():
    """라바콘 구간에서는 콘 사이로 가야지 차선을 보면 안 된다.

    0820 실측(cone_0820_2358 라바콘 525프레임): 게이트 50.3% / **차선주행
    폴백 48.6%**(최장 5.4초 연속). 그 동안 차가 통로를 가로질러 콘을 쳤다.
    이제 게이트가 끊기면 직전 콘 목표를 유지하고 오래되면 직진으로 푼다.
    근거는 PolicyConfig.cone_hold_s 주석 참조.
    """
    from race_last.kmu_race.core.race_fsm import Mode
    from race_last.kmu_race.core.race_context import RaceContext
    from race_last.kmu_race.core.safety import SafetyDecision, SafetyLevel
    from conftest import make_obs
    from dataclasses import replace as dc_replace

    READY = SafetyDecision(SafetyLevel.OK, "", True)
    pol, ctx = DrivePolicy(), RaceContext()

    # 콘이 보이는 동안: 오른쪽으로 치우친 게이트 → 우조향이 나온다
    seen = None
    for i in range(6):
        obs = make_obs(0.1 * i, cone_px=4000, gate_center=200.0, gate_valid=True)
        obs = dc_replace(obs, cone=dc_replace(obs.cone, gate_depth_px=500.0))
        seen = pol.update(Mode.CONE_DRIVE, obs, ctx, READY)
    assert seen is not None and abs(seen.steer) > 1.0, "게이트를 보고도 조향이 없다"
    held_sign = 1 if seen.steer > 0 else -1

    # 콘이 사라진다 — 차선은 반대쪽으로 크게 벗어나 있다.
    # 차선을 보면 부호가 뒤집힐 것이고, 콘 목표를 유지하면 그대로다.
    out = None
    for i in range(4):
        obs = make_obs(0.6 + 0.1 * i, cone_px=0, lane_offset=-400.0)
        out = pol.update(Mode.CONE_DRIVE, obs, ctx, READY)
    assert out is not None
    assert (1 if out.steer > 0 else -1) == held_sign or abs(out.steer) < 1e-6, (
        f"게이트 상실 후 차선 쪽으로 조향이 뒤집혔다 ({seen.steer:+.1f} → {out.steer:+.1f})")


def test_cone_section_uses_its_own_curve_slowdown():
    """라바콘 구간의 곡선감속은 차선주행보다 완만해야 한다.

    차선에서 조향이 크다 = 곡률 모르는 코너 → 줄이는 게 맞다.
    라바콘에서 조향이 크다 = 콘 사이를 누비는 것 → 같은 이유가 아니다.
    같은 감속을 쓰는 바람에 0.64m/s 로 기어갔다 (0820 실측).
    근거는 SpeedConfig.cone_steer_full_slow_deg 주석 참조.
    """
    sp = SpeedPolicy()
    STEER = 24.0                       # 실측 p90 대역
    lane = sp.target_for("LANE_DRIVE", steer_deg=STEER, time_pressure=0.0)
    sp2 = SpeedPolicy()
    cone = sp2.target_for("CONE_DRIVE", steer_deg=STEER, time_pressure=0.0)
    # 같은 조향에서 라바콘 쪽 감속 비율이 더 완만하다 (base 차이를 나눠서 비교)
    from race_last.kmu_race.core.speed_policy import SpeedConfig
    c = SpeedConfig()
    lane_ratio = lane / c.base["LANE_DRIVE"]
    cone_ratio = cone / c.base["CONE_DRIVE"]
    assert cone_ratio > lane_ratio, (
        f"라바콘 감속이 차선보다 심하다 (콘 {cone_ratio:.3f} vs 차선 {lane_ratio:.3f})")
    # 직진(조향 0)에서는 둘 다 감속이 없다
    sp3 = SpeedPolicy()
    assert sp3.target_for("CONE_DRIVE", steer_deg=0.0, time_pressure=0.0) == \
           c.base["CONE_DRIVE"]


def test_time_pressure_does_not_undo_curve_slowdown():
    """시간 압박 부스트는 꺼져 있다 (pressure_boost=1.0, 0820 밤 2차).

    base 28 = max_speed 28 이라 직선에서는 상한에 잘려 효과가 0이고,
    곱해지는 자리가 곡선·굽이침 감속 **뒤**라 남는 효과는 '코너 감속을
    되돌리는 것'뿐이다 — 정확히 원하지 않는 방향이다. 근거는
    speed_policy.SpeedConfig.pressure_boost 주석 참조.
    """
    sp = SpeedPolicy()
    # 직선: 상한에 걸려 어차피 같다
    assert sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=1.0) == \
           sp.target_for("LANE_DRIVE", steer_deg=0.0, time_pressure=0.0)
    # ★ 코너: 압박이 걸려도 감속이 되돌아가면 안 된다
    sp2 = SpeedPolicy()
    calm = sp2.target_for("LANE_DRIVE", steer_deg=40.0, time_pressure=0.0)
    sp3 = SpeedPolicy()
    rush = sp3.target_for("LANE_DRIVE", steer_deg=40.0, time_pressure=1.0)
    assert rush <= calm + 1e-6, f"코너에서 압박이 감속을 되돌렸다 ({calm} → {rush})"


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
    from race_last.kmu_race.core.types import DriveCommand

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
    ctx.lap.sightings = 2                      # 랩 마커를 한 번 세었다 = 2바퀴째
    assert ctx.time_pressure(10.0) == 0.0      # 아직 여유

    late = ctx.time_pressure(TIME_LIMIT_S - 40.0)
    assert late > 0.5, "바퀴가 남았는데 40초면 압박이 커야 한다"


def test_time_pressure_stays_zero_when_lap_counter_is_frozen():
    """랩을 한 번도 못 셌으면 '남은 바퀴'를 추정할 수 없다 → 가속 금지.

    0820 감사: 카운터가 얼면 laps_left 가 3 으로 고정돼 t=135s 부터 압박이
    1.0 에 붙고 LANE_DRIVE 속도에 ×1.30 이 후반 내내 걸린다. 근거 없는
    가속이라 이탈 손실이 따라잡기 기대이익보다 크다
    (race_context.time_pressure 주석 참조).
    """
    ctx = RaceContext()
    ctx.race_started_at = 0.0
    ctx.lap.sightings = 1                      # 출발 기준선뿐 — 마커 0회
    # 유예 안(60s)에서는 정상 계산 — 출발 직후 목격이 없는 건 당연하다
    assert ctx.time_pressure(30.0) == 0.0      # (여유 구간이라 어차피 0)
    # 유예를 넘겨도 마커가 없으면 압박은 0 을 유지한다
    assert ctx.time_pressure(TIME_LIMIT_S - 40.0) == 0.0


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


def test_cone_steer_is_in_motor_cmd_units():
    """★ 라바콘 조향은 **cmd 단위**여야 한다 (도(deg)가 아니라).

    /xycar_motor 로 나가는 값은 무차원 cmd 이고 motor_adapter 가
    `steering_angle = -(cmd × 0.0068)` 로 rad 을 만든다. 0821 이전에는
    콘 경로만 퓨어 퍼수트의 도(deg) 값을 그대로 발행해 실효 조향이
    1/2.567 이었다 — 194개 테스트 중 어느 것도 이걸 잡지 못했다.
    """
    import math
    from race_last.kmu_race.core.control import CMD_TO_RAD, WHEELBASE_M, pure_pursuit_cmd

    la, lat = 0.60, 0.18
    ld = math.hypot(la, lat)
    alpha = math.atan2(lat, la)
    delta = math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), ld)   # [rad]

    cmd = pure_pursuit_cmd(la, lat)
    # 발행값을 실제 체인에 태우면 기하가 요구한 바퀴각이 나와야 한다
    assert abs(-cmd * CMD_TO_RAD - delta) < 1e-9, (
        f"cmd {cmd:.2f} × 0.0068 = {-cmd*CMD_TO_RAD:.4f} rad 인데 "
        f"기하가 요구하는 값은 {delta:.4f} rad 이다")
    # 도(deg)를 그대로 내보내던 옛 값과는 2.5배 이상 다르다 (회귀 감지용)
    assert abs(cmd) > 2.0 * abs(math.degrees(delta)), (
        "도 단위 값이 그대로 나오고 있다 — 단위 버그 재발")


def test_cone_steer_produces_the_radius_the_gate_demands():
    """★ 발행값이 **실제로 만드는 선회반경**이 게이트 기하와 맞아야 한다.

    이게 단위 버그의 유일한 판별식이다. 조향값의 '크기'만 보면 도(deg)판도
    커 보인다 — 도 39.9 와 cmd 39.9 는 숫자가 같고 물리는 2.57배 다르다.
    그래서 발행값을 실제 체인(δ = cmd × 0.0068)에 태워 나오는 반경을
    퓨어 퍼수트가 요구한 반경 Ld/(2 sin α) 와 비교한다.

    포화가 안 걸리는 완만한 게이트를 쓴다 — 포화하면 양쪽 다 한계값이라
    구분이 안 된다.
    """
    import math
    from race_last.kmu_race.core.control import CMD_TO_RAD, WHEELBASE_M
    from race_last.kmu_race.core.policy import DrivePolicy, PolicyConfig
    from race_last.kmu_race.core.race_context import RaceContext
    from race_last.kmu_race.core.race_fsm import Mode
    from race_last.kmu_race.core.safety import SafetyDecision, SafetyLevel
    from race_last.kmu_race.core.types import ConeReading

    # PID 기여분을 꺼서 **단위만** 본다 (PID 는 별도 테스트가 본다)
    cfg = PolicyConfig(); cfg.cone_pid_kp = 0.0; cfg.cone_pid_kd = 0.0
    pol = DrivePolicy(cfg)
    ctx = RaceContext(); ctx.race_started_at = -10.0
    ready = SafetyDecision(SafetyLevel.OK, "", True)
    lat = 0.06                       # 완만 — 포화하지 않는다
    steer = 0.0
    for i in range(60):              # 스무더가 정착할 때까지
        obs = make_obs(0.05 * i)
        obs.cone = ConeReading(
            detected=True, gate_valid=False,
            lidar_gate_valid=True, lidar_gate_width_m=0.82,
            lidar_gate_depth_m=0.75, lidar_gate_lateral_m=lat, lidar_cones=6)
        steer = pol.update(Mode.CONE_DRIVE, obs, ctx, ready).steer

    la = pol._cone_la                # 정책이 실제로 쓴 전방주시
    ld = math.hypot(la, lat)
    r_geom = ld / (2.0 * math.sin(math.atan2(lat, la)))
    delta = abs(steer) * CMD_TO_RAD                    # 실제 모터 체인
    r_actual = WHEELBASE_M / math.tan(delta)
    assert abs(r_actual - r_geom) / r_geom < 0.05, (
        f"게이트는 반경 {r_geom:.2f}m 를 요구하는데 발행값 {steer:.2f} 는 "
        f"실제로 {r_actual:.2f}m 를 돈다 — 조향 단위가 틀렸다")


def test_cone_steer_can_reach_servo_limit_on_tight_gate():
    """급한 통로에서는 조향이 서보 한계까지 가야 한다.

    실측 통로 중심선 반경 중앙 1.26m (cone_0821_0037 라이다 원적합).
    R 1.0m 를 돌려면 바퀴 16.7° 가 필요한데 서보 한계가 16.5°(cmd 42.3)라
    급한 통로에서는 포화가 **정상**이다.

    ※ 이 테스트는 단위 버그를 잡지 못한다 (도 값도 숫자는 크게 나온다).
      단위 판별은 test_cone_steer_produces_the_radius_the_gate_demands 가 한다.
      여기서 보는 건 클램프·스무더가 포화까지 가는 길을 막지 않는지다.
    """
    from race_last.kmu_race.core.control import CMD_SATURATION
    from race_last.kmu_race.core.policy import DrivePolicy
    from race_last.kmu_race.core.race_context import RaceContext
    from race_last.kmu_race.core.race_fsm import Mode
    from race_last.kmu_race.core.safety import SafetyDecision, SafetyLevel
    from race_last.kmu_race.core.types import ConeReading

    pol = DrivePolicy()
    ctx = RaceContext(); ctx.race_started_at = -10.0
    ready = SafetyDecision(SafetyLevel.OK, "", True)
    peak = 0.0
    for i in range(40):
        obs = make_obs(0.1 * i)
        obs.cone = ConeReading(
            detected=True, gate_valid=False,
            lidar_gate_valid=True, lidar_gate_width_m=0.82,
            lidar_gate_depth_m=0.60, lidar_gate_lateral_m=0.28,  # 급하게 좌로 굽은 통로
            lidar_cones=6)
        peak = max(peak, abs(pol.update(Mode.CONE_DRIVE, obs, ctx, ready).steer))
    assert peak >= 0.9 * CMD_SATURATION, (
        f"급한 통로인데 최대 조향이 {peak:.1f} 뿐이다 (서보 한계 {CMD_SATURATION})")


def test_cone_guard_refuses_to_steer_into_a_cone():
    """★ 마지막 관문 — 보이는 콘을 관통하는 조향은 못 나간다.

    오늘 라바콘 결함 다섯을 고쳤는데도 주행마다 결과가 갈렸다
    (어떤 주행은 통과, 어떤 주행은 접촉). 게이트가 한 프레임만 틀려도
    그 순간 명령이 콘으로 향하기 때문이다. 이 관문은 게이트·통로 추정과
    **무관하게** 콘 좌표로 직접 막는다.
    """
    from race_last.kmu_race.core.control import path_clearance
    from race_last.kmu_race.core.policy import DrivePolicy, PolicyConfig
    from race_last.kmu_race.core.race_context import RaceContext
    from race_last.kmu_race.core.race_fsm import Mode
    from race_last.kmu_race.core.types import ConeReading

    # 왼쪽 앞에 콘 벽, 오른쪽은 열려 있다
    pts = ((0.40, 0.22), (0.80, 0.22), (1.20, 0.22),
           (0.40, -0.60), (0.80, -0.60))
    cone = ConeReading(
        detected=True, gate_valid=False,
        lidar_gate_valid=True, lidar_gate_width_m=0.82,
        lidar_gate_depth_m=0.55,
        lidar_gate_lateral_m=+0.35,      # ★ 게이트가 콘 쪽을 가리킨다 (오검)
        lidar_cones=5, lidar_cone_pts=pts)

    def drive(enabled):
        cfg = PolicyConfig(); cfg.cone_guard_enabled = enabled
        pol = DrivePolicy(cfg)
        ctx = RaceContext(); ctx.race_started_at = -10.0
        steer = 0.0
        for i in range(40):
            obs = make_obs(0.033 * i)
            obs.cone = cone
            obs.ego_speed_mps = 1.2
            steer = pol.update(Mode.CONE_DRIVE, obs, ctx, READY).steer
        return steer

    off = drive(False)
    on = drive(True)
    gap_off = path_clearance(off, pts, horizon_m=0.42)
    gap_on = path_clearance(on, pts, horizon_m=0.42)
    assert gap_off < 0.0, (
        f"이 장면은 원래 콘을 관통해야 회귀를 잡는다 (여유 {gap_off*100:+.1f}cm)")
    assert gap_on >= 0.0, (
        f"관문을 켰는데도 콘을 관통한다 (조향 {on:+.1f}, 여유 {gap_on*100:+.1f}cm)")


def test_cone_guard_leaves_safe_steering_alone():
    """안전한 조향은 손대지 않는다 (관문이 주행을 방해하면 안 된다)."""
    from race_last.kmu_race.core.control import clamp_steer_for_cones

    pts = ((0.60, 0.41), (1.00, 0.41), (0.60, -0.41), (1.00, -0.41))
    for cmd in (-10.0, 0.0, +10.0):
        out, gap = clamp_steer_for_cones(cmd, pts, speed_mps=1.2)
        assert abs(out - cmd) < 1e-6, f"통로 한가운데인데 {cmd} → {out} 로 바꿨다"
        assert gap > 0.0


def test_cone_lateral_target_cannot_teleport():
    """★ 콘 목표가 한 프레임에 순간이동하면 안 된다.

    통로 중심은 30ms 만에 30cm 움직이지 않는다. 그런데 게이트가 좌우 쌍
    → 단측 가상 게이트로 바뀌는 순간 목표가 통째로 뛰었다 (폐루프 추적,
    진입각 -20°): t=0.00 횡 +1.0cm → t=0.32 횡 -30.0cm. 그 한 번의 점프로
    차가 통로를 벗어나 돌아오지 못했다(이탈 275cm).
    """
    pol = DrivePolicy()
    rate = pol.config.cone_lat_rate_mps
    t = 0.0
    for _ in range(10):
        t += 0.033
        lat = pol._cone_lat_track(+0.01, t)
    prev, worst = lat, 0.0
    for _ in range(20):
        t += 0.033
        lat = pol._cone_lat_track(-0.30, t)     # 목표가 통째로 반대편으로
        worst = max(worst, abs(lat - prev))
        prev = lat
    assert worst <= rate * 0.033 + 1e-6, (
        f"목표가 한 프레임에 {worst*100:.1f}cm 뛰었다 (상한 {rate*0.033*100:.1f}cm)")
    assert abs(prev + 0.30) < 0.02, f"결국 목표에 도달해야 한다 ({prev:.3f})"


def test_lane_steering_untouched_when_no_corridor_ahead():
    """통로가 안 보이면 차선 조향을 건드리지 않는다."""
    from race_last.kmu_race.core.race_context import RaceContext

    pol = DrivePolicy()
    ctx = RaceContext(); ctx.race_started_at = -10.0
    obs = make_obs(1.0)
    plain = pol._lane_steer(obs)
    pol2 = DrivePolicy()
    blended = pol2._lane_with_cone_approach(obs)
    assert abs(blended - plain) < 1e-9


def test_cone_approach_blends_in_as_the_corridor_nears():
    """★ 통로가 가까워질수록 콘 조향 비중이 올라간다.

    라바콘 접촉의 편차는 통로 안이 아니라 **진입각**에서 온다. 실측
    (cone_0821_* 4개 bag): 진입각 중앙 13.5°, **38%가 20° 초과**.
    R1.5 통로에서 20°를 지우려면 1.11m 가 걸리고 19.3cm 밀리는데 여유는
    18.5cm 뿐 — 통로 안에서는 이미 늦다. 라이다는 0.7~1.9초 전에 이미
    통로를 보므로 그때부터 섞어 미리 정렬한다.
    """
    from race_last.kmu_race.core.types import ConeReading

    from race_last.kmu_race.core.policy import PolicyConfig

    cfg = PolicyConfig(); cfg.cone_approach_enabled = True   # 기본은 꺼져 있다
    pol = DrivePolicy(cfg)
    

    def w_at(depth, cones=6, valid=True):
        obs = make_obs(1.0)
        obs.cone = ConeReading(
            detected=True, lidar_gate_valid=valid, lidar_cones=cones,
            lidar_gate_width_m=0.82, lidar_gate_depth_m=depth,
            lidar_gate_lateral_m=0.05)
        return pol._cone_approach_weight(obs)

    assert w_at(2.0) == 0.0, "멀리 있는 통로에는 반응하지 않는다"
    assert w_at(cfg.cone_approach_far_m + 0.01) == 0.0
    mid = w_at(0.5 * (cfg.cone_approach_far_m + cfg.cone_approach_near_m))
    assert 0.0 < mid < cfg.cone_approach_max_w
    assert abs(w_at(cfg.cone_approach_near_m) - cfg.cone_approach_max_w) < 1e-6
    assert abs(w_at(0.3) - cfg.cone_approach_max_w) < 1e-6, "가까우면 상한 유지"
    # 통로로 안 보이면 0
    assert w_at(0.8, cones=2) == 0.0, "콘이 적으면 통로로 안 본다"
    assert w_at(0.8, valid=False) == 0.0, "게이트가 없으면 통로로 안 본다"


def test_cone_approach_never_fully_replaces_lane():
    """섞기만 하고 갈아타지 않는다 — 접근 중 통로 추정이 틀려도 차선이 받는다."""
    pol = DrivePolicy()
    assert 0.0 < pol.config.cone_approach_max_w < 1.0
    assert pol.config.cone_approach_enabled is False, (
        "조향 혼합은 실측이 나빠 기본 꺼짐이다 (설정 주석 참조)")


def test_cone_approach_slows_before_the_corridor():
    """★ 통로가 앞에 보이면 **미리** 라바콘 속도로 내린다.

    실차는 구간 직전까지 2.16 m/s 인데 진입 판정이 최근접 콘 ~50cm 에서
    나고, 모터 가속 제한 2.5 m/s² 로 2.16→1.44 감속에 0.52m 가 걸린다 —
    즉 첫 콘에 거의 2.16 m/s 로 닿는다. 빠를수록 지연(0.15초) 중 눈감고
    가는 거리가 늘어(1.0m/s 15cm vs 2.16m/s 32cm) 진입각 오차가 그대로
    횡이탈이 된다.
    """
    from race_last.kmu_race.core.types import ConeReading

    pol = DrivePolicy()
    cfg = pol.config
    lane_v = cfg.speed.base["LANE_DRIVE"]
    cone_v = cfg.speed.base["CONE_DRIVE"]

    def v_at(depth, cones=6, valid=True):
        obs = make_obs(1.0)
        obs.cone = ConeReading(
            detected=True, lidar_gate_valid=valid, lidar_cones=cones,
            lidar_gate_width_m=0.82, lidar_gate_depth_m=depth)
        return pol._cone_approach_speed(obs, lane_v)

    assert v_at(2.0) == lane_v, "멀리 있는 통로에는 안 줄인다"
    assert v_at(cfg.cone_approach_slow_far_m + 0.01) == lane_v
    mid = v_at(0.5 * (cfg.cone_approach_slow_far_m + cfg.cone_approach_slow_near_m))
    assert cone_v < mid < lane_v, f"중간에서 중간 속도여야 한다 ({mid})"
    assert abs(v_at(cfg.cone_approach_slow_near_m) - cone_v) < 1e-6
    assert v_at(0.8, cones=2) == lane_v, "콘이 적으면 통로로 안 본다"
    assert v_at(0.8, valid=False) == lane_v, "게이트가 없으면 통로로 안 본다"


def test_guard_accounts_for_car_body_not_a_point():
    """★ 차는 점도 원반도 아니다 — 돌 때 **앞 코너가 바깥으로 쓸고 나간다**.

    사용자 지적: "경로 계획은 좋은데 차 폭을 계산 못해 자꾸 라바콘을 친다."
    기준점은 뒷차축이고 차체는 앞으로 0.40m 뻗어 있다. R=1.0m 선회에서
        안쪽 뒷모서리 0.85m,  바깥 앞코너 √(1.15²+0.40²) = 1.22m
    → 실효 폭 37cm (차폭 30cm 보다 7cm 넓다).
    반지름 0.15 원반으로 보면 이 스윙아웃을 통째로 놓쳐 '여유 있다'고
    판단한 조향이 실제로는 콘을 친다.
    """
    import math

    from race_last.kmu_race.core.control import (
        CAR_FRONT_M, CAR_HALF_M, CONE_RADIUS_M, path_clearance,
    )

    # 통로 82cm 한가운데, 좌우 두 줄
    cones = [(0.5, 0.41), (0.9, 0.41), (1.3, 0.41),
             (0.5, -0.41), (0.9, -0.41), (1.3, -0.41)]
    straight = path_clearance(0.0, cones, horizon_m=0.6)
    assert abs(straight - (0.41 - CAR_HALF_M - CONE_RADIUS_M)) < 0.01, (
        "직진에서는 횡여유가 그대로여야 한다")

    # 꺾을수록 줄어야 한다 (원반 모델은 이걸 못 본다).
    # ※ 아주 급하게 꺾으면 먼 콘에서 일찍 멀어져 다시 조금 늘 수 있다 —
    #   그래서 30까지만 단조를 요구한다.
    turned = [path_clearance(c, cones, horizon_m=0.6) for c in (10, 20, 30, 42)]
    for a, b in zip([straight] + turned[:3], turned[:3]):
        assert b < a + 1e-9, f"꺾었는데 여유가 안 줄었다: {turned}"
    assert turned[-1] < 0.0, (
        f"통로 폭 82cm 에서 풀락인데 여유가 {turned[-1]*100:+.1f}cm — "
        "차체가 콘을 쓸고 지나간다는 것을 못 잡고 있다")

    # 앞 코너가 실제로 가장 위험한 지점인지 (기하 확인)
    R = 0.30 / math.tan(42.0 * 0.0068)
    outer = math.hypot(R + CAR_HALF_M, CAR_FRONT_M)
    assert outer - (R - CAR_HALF_M) > 2 * CAR_HALF_M, (
        "선회 중 실효 폭이 차폭보다 넓어야 한다")


def test_cone_pid_adds_proportional_response_but_not_derivative():
    """★ 라바콘에 P 항을 더한다. D 는 켜지 않는다 (실측에서 나빠졌다).

    폐루프 진입조건 스윕(통로 4종 × 횡위치 7 × 진입각 5):
        kp  0 / kd 0 → 80%, 여유 7.0cm
        kp 20 / kd 0 → 80%, 여유 7.3cm   ← 채택
        kp 20 / kd 6 → 76%, 여유 6.8cm
        kp 30 / kd10 → 72%, 여유 5.8cm
    이 루프는 게인이 모자란 게 아니라 포화·지연에 묶여 있다.
    """
    pol = DrivePolicy()
    cfg = pol.config
    assert cfg.cone_pid_kp > 0.0, "P 항이 꺼져 있다"
    assert cfg.cone_pid_kd == 0.0, "D 항은 실측이 나빠 꺼 둔다 (주석 참조)"

    # 목표가 왼쪽(+)이면 좌회전(cmd 음수) 방향으로 기여해야 한다
    assert pol._cone_pid(+0.20, 1.0) < 0.0
    pol2 = DrivePolicy()
    assert pol2._cone_pid(-0.20, 1.0) > 0.0
    # 상한이 걸린다
    pol3 = DrivePolicy()
    assert abs(pol3._cone_pid(+5.0, 1.0)) <= cfg.cone_pid_max + 1e-9


def test_cone_pid_derivative_uses_its_own_clock():
    """미분항이 죽지 않는지 — 초기 구현에서 dt 가 늘 0 이라 D 가 무효였다."""
    from race_last.kmu_race.core.policy import PolicyConfig

    cfg = PolicyConfig(); cfg.cone_pid_kp = 0.0; cfg.cone_pid_kd = 10.0
    pol = DrivePolicy(cfg)
    t = 0.0
    out = 0.0
    for e in (0.30, 0.25, 0.20, 0.15, 0.10):
        t += 0.033
        out = pol._cone_pid(e, t)
    assert abs(pol._cone_err_rate) > 0.1, "오차가 줄고 있는데 미분항이 0 이다"
    assert out > 0.0, "오차가 줄면 D 는 반대 방향으로 감쇠해야 한다"


def test_lidar_extrapolation_is_off_by_default_but_correct():
    """★ 라이다 외삽은 만들었지만 **기본 끔** — 실측에서 효과가 없었다.

    폐루프 진입조건 스윕(140케이스): 끔 80%/7.3cm, 100% 외삽 77%/6.4cm.
    회전항을 빼면 74%/4.5cm 로 확실히 나빠지므로 구현은 맞다 — 이 루프가
    정보 지연이 아니라 **포화와 최소 회전반경**에 묶여 있는 것이다.
    """
    import math

    from race_last.kmu_race.core.control import CMD_TO_RAD, WHEELBASE_M
    from race_last.kmu_race.core.policy import DrivePolicy, PolicyConfig
    from race_last.kmu_race.core.types import ConeReading

    assert DrivePolicy().config.cone_extrapolate_enabled is False

    # 켜면 기하가 맞는가: 직진으로 age 동안 v 만큼 갔으면 깊이가 그만큼 준다
    cfg = PolicyConfig(); cfg.cone_extrapolate_enabled = True
    pol = DrivePolicy(cfg)
    obs = make_obs(1.0)
    obs.ego_speed_mps = 1.0
    obs.cone = ConeReading(
        detected=True, lidar_gate_valid=True, lidar_cones=6,
        lidar_gate_width_m=0.82, lidar_gate_depth_m=0.90,
        lidar_gate_lateral_m=0.0, lidar_gate_age_s=0.10,
        lidar_cone_pts=((0.90, 0.41), (0.90, -0.41)))
    pol._last_steer = 0.0
    out = pol._extrapolate_cone(obs)
    assert abs(out.lidar_gate_depth_m - 0.80) < 0.01, (
        f"1.0m/s 로 0.10초면 10cm 가까워져야 한다 ({out.lidar_gate_depth_m:.3f})")
    assert out.lidar_gate_age_s == 0.0

    # 너무 낡으면 손대지 않는다
    obs.cone = replace_age(obs.cone, cfg.cone_extrapolate_max_s + 0.05)
    out2 = pol._extrapolate_cone(obs)
    assert abs(out2.lidar_gate_depth_m - 0.90) < 1e-9


def replace_age(cone, age):
    from dataclasses import replace
    return replace(cone, lidar_gate_age_s=age)

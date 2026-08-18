"""차선 조향기 회귀 — 0817 첫 자율주행 코너 실패에서 나온 계약들.

이 파일의 테스트가 깨지면 코너를 못 돈다. 각 테스트는 그날 실제로 관측된
실패 모드 하나씩에 대응한다.
"""

import math

from kmu_race.core.control import CMD_TO_RAD, WHEELBASE_M, LaneController, PixelPID

SERVO_MAX_DEG = 16.5
CMD_SAT = SERVO_MAX_DEG / math.degrees(CMD_TO_RAD)   # ≈ 42


def settle(ctl, *, offset=0.0, slope=0.0, curv=0.0, lane_w=450.0, n=40):
    out = 0.0
    for _ in range(n):
        out = ctl.update(offset, slope, curv, lane_w)
    return out


def test_straight_is_quiet():
    """직선에서 조향이 튀면 안 된다 (오차·기울기·곡률 0)."""
    assert abs(settle(LaneController())) < 1.0


def test_corner_needs_no_error_to_steer():
    """★ 핵심: 코너에서 '오차 없이' 조향이 나와야 한다.

    0817 실패: 횡오차 P 제어라 차가 바깥으로 밀려나야 조향이 생겼고, 그
    밀려난 양이 곧 차선 이탈이었다 (1랩 129샘플 최대 조향 19 / 물리 42).
    """
    ctl = LaneController()
    # 차선폭 450px = 0.8m 기준, 반경 1.2m 코너의 BEV 곡률
    s_lat = 450.0 / 0.8
    kappa_real = 1.0 / 1.2
    curv_px = kappa_real * s_lat / (s_lat * ctl.bev_aspect) ** 2 * s_lat
    steer = settle(ctl, curv=curv_px)
    assert steer > 8.0, "코너 피드포워드가 조향을 못 만든다"


def test_heading_term_corrects_misalignment():
    """차가 차선과 어긋나면(기울기) 오차가 0이어도 편다."""
    ctl = LaneController()
    right = settle(ctl, slope=0.25)
    ctl.reset()
    left = settle(ctl, slope=-0.25)
    assert right > 5.0 and left < -5.0
    assert abs(right + left) < 1.0, "좌우 대칭이어야 한다"


def test_output_never_exceeds_physical_saturation():
    """물리 포화(cmd≈42) 초과 명령은 바퀴를 못 꺾고 하류만 오해시킨다."""
    ctl = LaneController()
    out = settle(ctl, offset=900.0, slope=3.0, curv=0.05)
    assert abs(out) <= ctl.cmd_saturation + 1e-6


def test_apex_engages_only_when_saturated():
    """아펙스는 '실제로 조향이 포화됐을 때만' 켜진다.

    0817 시뮬 실패: 곡률 추정치로 발동시켰더니 BEV 축척 오차(최대 1.6배)
    탓에 멀쩡한 R=1.2m 코너에서도 켜져 0.755m 이탈했다.
    """
    gentle = LaneController()
    settle(gentle, offset=20.0, n=60)          # 포화 근처도 안 감
    assert abs(gentle._bias_px) < 1.0, "완만한 상황에서 아펙스가 켜졌다"

    hard = LaneController()
    settle(hard, offset=400.0, n=60)           # 지속 포화
    assert abs(hard._bias_px) > 30.0, "포화 지속인데 아펙스가 안 켜졌다"


def test_apex_cancels_in_slalom():
    """S자(좌우 교대)에서는 아펙스가 상쇄돼 꺼져야 한다.

    슬라럼은 안쪽에 붙는 게 아니라 가운데로 지나야 한다 — 0817 시뮬에서
    이걸 안 지키면 연속 S자에서 0.90m 이탈했다.
    """
    ctl = LaneController()
    for i in range(200):
        ctl.update(400.0 if (i // 15) % 2 == 0 else -400.0, 0.0, 0.0, 450.0)
    assert abs(ctl._bias_px) < 60.0, "교대 코너에서 아펙스가 한쪽으로 쏠렸다"


def test_apex_bias_is_rate_limited():
    """아펙스 목표가 한 프레임에 계단 이동하면 그대로 진동이 된다."""
    ctl = LaneController()
    prev = 0.0
    for _ in range(60):
        ctl.update(400.0, 0.0, 0.0, 450.0)
        assert abs(ctl._bias_px - prev) <= ctl.bias_rate_px + 1e-6
        prev = ctl._bias_px


def test_gain_self_calibrates_with_lane_width():
    """피드포워드 게인은 측정된 차선폭에서 유도된다 (px_per_m 미실측 대응)."""
    ctl = LaneController()
    assert ctl.k_curvature(450.0) > ctl.k_curvature(300.0)
    # 유도식 재현: (축거/CMD_TO_RAD) × s_lat × aspect² × ff_scale
    s_lat = 450.0 / ctl.road_width_m
    expect = (WHEELBASE_M / CMD_TO_RAD) * s_lat * ctl.bev_aspect ** 2 * ctl.ff_scale
    assert abs(ctl.k_curvature(450.0) - expect) < 1e-6


def test_physical_turn_radius_limit_is_documented():
    """이 차는 반경 1.0m 이하 코너를 중심선으로는 못 돈다 — 설계 전제."""
    max_rad = math.radians(SERVO_MAX_DEG)
    r_min = WHEELBASE_M / math.tan(max_rad)
    assert 0.95 < r_min < 1.10, f"최소 회전반경 계산이 바뀌었다: {r_min:.2f}m"
    assert CMD_SAT > 40.0

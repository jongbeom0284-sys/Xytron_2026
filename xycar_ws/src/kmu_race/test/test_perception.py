"""퍼셉션 스모크 테스트 — 합성 입력으로 '죽지 않는가'와 '방향이 맞는가'를 본다.

실차 정확도는 여기서 못 잰다. 그건 연습주행에서 bag 을 받아 맞춰야 한다.
여기서 잡으려는 건 대회 당일에 나면 끝장인 것들이다:
  · 프레임이 비었을 때 예외로 노드가 죽는가
  · 부호가 뒤집혀 있는가 (왼쪽 차선인데 우조향이 나온다든가)
  · 스캔 길이가 바뀌었을 때 인덱스 에러가 나는가
"""

import math

import pytest

cv2 = pytest.importorskip("cv2")
np = pytest.importorskip("numpy")

from kmu_race.core.types import SignalState  # noqa: E402
from kmu_race.perception.cone_detector import ConeDetector  # noqa: E402
from kmu_race.perception.lane_detector import LaneDetector  # noqa: E402
from kmu_race.perception.lidar_tracker import LidarConfig, LidarTracker  # noqa: E402

# 합성 스캔 테스트는 '라이다 0° = 차 정면' 기하를 검증한다. 실차 기본값
# angle_offset=-2.164 는 kobot 장착 실측(0817)이라 여기선 0 으로 끈다.
_SYNTH = LidarConfig(angle_offset=0.0)
from kmu_race.perception.signal_detector import SignalDetector  # noqa: E402

W, H = 640, 480


def road_frame(left_x=140, right_x=500):
    """흰 차선 두 줄이 그려진 합성 프레임."""
    img = np.full((H, W, 3), 40, np.uint8)
    cv2.line(img, (left_x, H), (left_x + 40, H // 2), (255, 255, 255), 12)
    cv2.line(img, (right_x, H), (right_x - 40, H // 2), (255, 255, 255), 12)
    return img


def signal_frame(lit_index, n_lamps=4, cx=320, cy=200, gap=46, r=15):
    """4구 신호등 합성. lit_index 에 해당하는 램프만 켠다.

    점등 램프는 실물처럼 **유채색**으로 그린다 (검출기의 채도 게이트가
    무채색 밝음 = 천장 조명을 걸러내므로, 흰색으로 그리면 실물과 다르게
    lit 판정에서 탈락하는 게 맞다). 색은 run4 bag 실측 HSV 를 BGR 로 옮긴 것.
    """
    img = np.full((H, W, 3), 30, np.uint8)
    x0 = cx - (n_lamps - 1) * gap // 2
    cv2.rectangle(
        img,
        (x0 - 32, cy - 30),
        (x0 + (n_lamps - 1) * gap + 32, cy + 30),
        (20, 20, 20),
        -1,
    )
    # 좌→우 [빨강, 노랑, 좌회전(초록화살표), 녹색] — BGR
    lamp_color = {
        0: (40, 40, 245),     # 빨강 (H≈0)
        1: (40, 215, 245),    # 노랑
        2: (120, 230, 40),    # 초록 화살표 (H≈73 — 실측 램프 색감)
        3: (120, 230, 40),    # 녹색
    }
    for i in range(n_lamps):
        x = x0 + i * gap
        on = i in (lit_index if isinstance(lit_index, (list, tuple)) else [lit_index])
        color = lamp_color.get(i, (245, 245, 245)) if on else (55, 55, 55)
        cv2.circle(img, (x, cy), r, color, -1)
    return img


# ──────────────────────────────────────────────────────────────────
# 견고성 — 죽지 않는가
# ──────────────────────────────────────────────────────────────────
def test_detectors_survive_none_input():
    assert LaneDetector().detect(None).valid is False
    assert ConeDetector().detect(None).detected is False
    assert SignalDetector().detect(None).state is SignalState.UNKNOWN


def test_detectors_survive_blank_frame():
    blank = np.zeros((H, W, 3), np.uint8)
    LaneDetector().detect(blank)
    ConeDetector().detect(blank)
    SignalDetector().detect(blank)


def test_lidar_survives_varying_scan_length():
    """스캔 길이가 바뀌어도 죽지 않는다 — 인덱스 기반 구현의 고질병 방지."""
    trk = LidarTracker(_SYNTH)
    for n in (180, 360, 505, 720):
        ranges = [3.0] * n
        r = trk.update(
            ranges,
            angle_min=-math.pi,
            angle_increment=2 * math.pi / n,
            now=float(n),
        )
        assert r.valid


def test_lidar_handles_inf_and_nan():
    trk = LidarTracker(_SYNTH)
    n = 360
    ranges = [float("inf")] * n
    ranges[10] = float("nan")
    ranges[20] = 0.0
    r = trk.update(
        ranges, angle_min=-math.pi, angle_increment=2 * math.pi / n, now=1.0
    )
    assert r.front_dist == float("inf")


# ──────────────────────────────────────────────────────────────────
# 부호 — 방향이 맞는가
# ──────────────────────────────────────────────────────────────────
def test_lane_offset_sign_follows_car_position():
    """차가 왼쪽에 치우치면 목표선은 오른쪽(+)에 보여야 한다."""
    det = LaneDetector()
    centered = det.detect(road_frame(left_x=140, right_x=500))

    det2 = LaneDetector()
    # 차선을 통째로 오른쪽으로 밀면 = 차가 왼쪽에 있는 상황
    shifted = det2.detect(road_frame(left_x=240, right_x=600))

    assert shifted.offset_px > centered.offset_px


def test_lidar_front_sector_is_angle_based():
    """정면 물체는 스캔 해상도와 무관하게 정면으로 잡혀야 한다."""
    for n in (200, 400, 800):
        trk = LidarTracker(_SYNTH)
        ranges = [5.0] * n
        # 0 rad(정면)에 해당하는 인덱스에 가까운 물체를 놓는다
        idx = int((0.0 - (-math.pi)) / (2 * math.pi / n))
        for k in range(idx - 2, idx + 3):
            ranges[k % n] = 1.1
        r = trk.update(
            ranges, angle_min=-math.pi, angle_increment=2 * math.pi / n, now=1.0
        )
        assert abs(r.front_dist - 1.1) < 0.05, f"n={n} 에서 정면 검출 실패"


def test_lidar_side_clearance():
    trk = LidarTracker(_SYNTH)
    n = 360
    inc = 2 * math.pi / n
    ranges = [5.0] * n
    # 좌측 90° 부근을 막는다
    for k in range(n):
        a = -math.pi + k * inc
        if 0.7 < a < 1.4:
            ranges[k] = 0.4
    r = trk.update(ranges, angle_min=-math.pi, angle_increment=inc, now=1.0)
    assert r.left_clear is False
    assert r.right_clear is True


def test_cone_gate_center_between_two_rows():
    """좌우 두 줄 사이의 중앙을 잡아야 한다 — 한쪽 줄을 따라가면 안 된다."""
    img = np.full((240, W, 3), 40, np.uint8)
    # BGR 로 주황 (H≈15, S 높음, V 높음)
    orange = (30, 150, 250)
    for y in (120, 170, 220):
        cv2.circle(img, (200, y), 16, orange, -1)   # 왼쪽 줄
        cv2.circle(img, (440, y), 16, orange, -1)   # 오른쪽 줄

    r = ConeDetector().detect(img)
    assert r.detected
    assert r.gate_valid, "양쪽 줄을 모두 봐야 게이트가 성립"
    # 게이트 중앙(320)이 곧 ROI 중앙(320) → 오차 거의 0
    assert abs(r.gate_center_px) < 40, f"게이트 중앙 오차 {r.gate_center_px}"


def test_cone_gate_shifts_with_rows():
    """두 줄이 함께 왼쪽으로 밀리면 게이트 중앙도 왼쪽(-)으로."""
    img = np.full((240, W, 3), 40, np.uint8)
    orange = (30, 150, 250)
    for y in (120, 170, 220):
        cv2.circle(img, (100, y), 16, orange, -1)
        cv2.circle(img, (300, y), 16, orange, -1)
    r = ConeDetector().detect(img)
    assert r.gate_valid
    assert r.gate_center_px < -40


def test_cone_single_row_reports_invalid_gate():
    """한쪽만 보이면 게이트를 만들지 않는다 (policy 가 반발 조향으로 대체)."""
    img = np.full((240, W, 3), 40, np.uint8)
    orange = (30, 150, 250)
    for y in (120, 170, 220):
        cv2.circle(img, (150, y), 16, orange, -1)
    r = ConeDetector().detect(img)
    assert r.detected
    assert r.gate_valid is False


# ──────────────────────────────────────────────────────────────────
# 4구 신호등 — 규정 p.36 의 점등 조합
# ──────────────────────────────────────────────────────────────────
def _detect(frame):
    det = SignalDetector()
    det.config.min_radius = 10
    det.config.max_radius = 22
    det.config.roi_top = 0.0
    det.config.roi_bottom = 0.45
    det.config.roi_left = 0.15
    det.config.roi_right = 0.85
    return det.detect(frame)


def test_signal_green_only_is_go_straight():
    """좌→우 [빨강, 노랑, 좌회전, 녹색] — 4번째만 켜지면 직진."""
    r = _detect(signal_frame(lit_index=3))
    assert r.state is SignalState.GREEN
    assert r.state.is_go
    assert not r.state.allows_left


def test_signal_green_plus_left_allows_shortcut():
    """3번+4번 동시 = 직좌 동시신호 → 지름길 가능."""
    r = _detect(signal_frame(lit_index=[2, 3]))
    assert r.state is SignalState.GREEN_LEFT
    assert r.state.allows_left


def test_signal_red_is_not_go():
    r = _detect(signal_frame(lit_index=0))
    assert r.state is SignalState.RED
    assert not r.state.is_go


def test_signal_reports_gantry_span_for_lap_counting():
    """분류가 되든 안 되든 겐트리 근접 지표는 나와야 한다 (랩 카운팅용)."""
    r = _detect(signal_frame(lit_index=3))
    assert r.gantry_span_px > 0


def test_achromatic_bright_circles_are_not_a_signal():
    """천장 조명 회귀 테스트 (run4 bag 실측 실패 모드).

    무채색으로 밝은 원 4개가 일렬로 있어도 — 대회장 형광등이 정확히 이렇게
    잡힌다 — 켜진 램프도, 랩 마커도 아니어야 한다.
    """
    img = np.full((H, W, 3), 30, np.uint8)
    x0 = 320 - 3 * 46 // 2
    for i in range(4):
        cv2.circle(img, (x0 + i * 46, 200), 15, (245, 245, 245), -1)
    r = _detect(img)
    assert r.state is SignalState.UNKNOWN, "무채색 밝음은 점등이 아니다"
    assert r.gantry_span_px == 0, "무채색 원은 랩 마커가 아니다"


def test_gantry_span_requires_green_lamp():
    """랩 마커는 초록 점등에서만 나온다.

    빨간불 대기(출발 전) 장면에는 span 이 없어야 하고 — '목격 1'은 초록이
    켜지는 출발 순간에 잡히는 것이 의도된 설계다 (race_context.py 주석 참조).
    """
    assert _detect(signal_frame(lit_index=0)).gantry_span_px == 0
    assert _detect(signal_frame(lit_index=[2, 3])).gantry_span_px > 0


# ──────────────────────────────────────────────────────────────────
# 과노출 심 경로 (0817 kobot 새 카메라 실측 재현 — signal_detector 주석 참조)
# ──────────────────────────────────────────────────────────────────
def washed_signal_frame(lit_index=3, ring_green=True, cx=320, cy=200, gap=46, r=4):
    """과노출 카메라 합성: 점등 램프 심이 하얗게 날아가고 초록 프린지만 남는다.

    run_0817_1211 실측을 본뜬 것 — 심 V 240~253 / S 요동(채도 게이트 탈락),
    심 가장자리 링에 초록 프린지 0.13~0.67. 전체 장면도 과노출로 밝다.
    cy=200: 목격 거리 램프의 실측 기하 대역(y_full 187~219 — washed_min_y_frac
    게이트 안쪽). 이보다 높으면(작은 y) 창문/원거리로 보고 걸러지는 게 설계다.
    """
    img = np.full((H, W, 3), 200, np.uint8)          # 과노출 실내 (V 200 < 심 220)
    x0 = cx - 3 * gap // 2
    cv2.rectangle(                                    # 하우징 패널 (어두움)
        img, (x0 - 32, cy - 2 * r - 8), (x0 + 3 * gap + 32, cy + 2 * r + 8),
        (50, 50, 50), -1,
    )
    for i in range(4):
        x = x0 + i * gap
        if i == lit_index:
            if ring_green:
                cv2.circle(img, (x, cy), r + 4, (130, 200, 30), -1)  # 초록 프린지 (H≈78, 실측 램프색)
            cv2.circle(img, (x, cy), r, (250, 255, 250), -1)         # 날아간 흰 심
        else:
            cv2.circle(img, (x, cy), r + 2, (35, 35, 35), -1)        # 소등 램프
    return img


def test_washed_core_with_green_ring_is_lap_marker():
    """흰 심 + 초록 링 + 왼쪽 소등 램프 = 겐트리 랩 마커로 인정."""
    r = _detect(washed_signal_frame(r=4))
    assert r.gantry_span_px > 0


def test_washed_core_without_ring_is_not_lap_marker():
    """초록 프린지가 없는 흰 심(조명 반사)은 랩 마커가 아니다."""
    r = _detect(washed_signal_frame(r=4, ring_green=False))
    assert r.gantry_span_px == 0


def test_fluorescent_bar_is_not_lap_marker():
    """길쭉한 밝은 막대(형광등)는 종횡비에서 걸린다."""
    img = np.full((H, W, 3), 200, np.uint8)
    cv2.rectangle(img, (250, 100), (390, 112), (255, 255, 255), -1)
    assert _detect(img).gantry_span_px == 0


def test_washed_green_classifies_as_green():
    """분류 경로도 과노출을 수용한다: 흰 심 + 초록 링 램프 → GREEN.

    (원 분류가 성립하는 근접 거리 크기 r=10 로 그린다. 원거리에서는 분류가
    안 되는 게 정상이고 출발/랩은 블롭 경로가 담당한다 — race_fsm.py 주석)
    """
    r = _detect(washed_signal_frame(lit_index=3, r=10))
    assert r.state is SignalState.GREEN


def test_washed_core_above_horizon_is_rejected():
    """창밖 수풀 회귀 (0817 실측 실패 모드).

    수평선 위(y_full < 34%)의 흰 심+초록 링 — 창문 수풀 속 하늘 구멍이
    정확히 이 모양이고 랩마다 같은 위치에서 목격을 오발화했다(76.5s/177.1s).
    목격 거리의 진짜 램프는 기하상 y_full 187~219 에만 온다.
    """
    r = _detect(washed_signal_frame(r=4, cy=110))
    assert r.gantry_span_px == 0


# ──────────────────────────────────────────────────────────────────
# 라바콘 게이트 기하 (0817 재설계 — ddm 실주행 bag 진단 반영)
# ──────────────────────────────────────────────────────────────────
def _mk(now):
    """conftest 의 make_obs 를 늦게 가져온다 (import 순환 회피)."""
    from conftest import make_obs
    return make_obs(now)


def cone_frame(pairs, w=W, h=240):
    """BEV ROI 에 좌우 콘 쌍을 그린다. pairs = [(y, xl, xr), ...]"""
    img = np.zeros((h, w, 3), np.uint8)
    for y, xl, xr in pairs:
        for x in (xl, xr):
            cv2.circle(img, (int(x), int(y)), 14, (60, 160, 255), -1)  # 주황 BGR
    return img


def test_cone_gate_reports_depth_and_width():
    """게이트의 깊이·폭을 보고해야 한다 (전방주시 클램프·가상 게이트의 입력)."""
    det = ConeDetector()
    r = det.detect(cone_frame([(200, 180, 460)]))
    assert r.gate_valid
    assert r.gate_width_px > 200
    assert r.gate_depth_px > 0, "게이트 깊이가 없으면 전방주시를 못 자른다"


def test_cone_lookahead_is_clamped_to_observed_gate():
    """★ 조준점은 실제로 본 콘보다 멀리 가면 안 된다.

    ddm 실주행 bag 진단: 관측 밖을 조준하면 코너에서 통로를 가로지른다.
    """
    from kmu_race.core.policy import DrivePolicy, PolicyConfig
    from kmu_race.core.race_context import RaceContext
    from kmu_race.core.race_fsm import Mode
    from kmu_race.core.safety import SafetyDecision, SafetyLevel
    from kmu_race.core.types import ConeReading

    pol = DrivePolicy(PolicyConfig())
    ready = SafetyDecision(SafetyLevel.OK, "", True)
    ctx = RaceContext()
    ctx.race_started_at = -10.0

    def steer_for(depth_px):
        pol.reset()
        obs = _mk(1.0)
        obs.cone = ConeReading(
            detected=True, orange_px=5000, gate_center_px=120.0,
            gate_valid=True, gate_depth_px=depth_px, gate_width_px=300.0,
        )
        return pol.update(Mode.CONE_DRIVE, obs, ctx, ready).steer

    near = abs(steer_for(60.0))     # 게이트가 코앞 → 전방주시 짧음 → 크게 꺾어야
    far = abs(steer_for(600.0))     # 게이트가 멀다 → 상한(0.75m)에 걸림
    assert near > far, "가까운 게이트인데 조향이 더 작다 (클램프 미작동)"


def test_cone_single_side_uses_virtual_gate_not_full_lock():
    """단측 구간에서 풀락이 아니라 가상 게이트 중앙으로 가야 한다."""
    from kmu_race.core.policy import DrivePolicy, PolicyConfig
    from kmu_race.core.race_context import RaceContext
    from kmu_race.core.race_fsm import Mode
    from kmu_race.core.safety import SafetyDecision, SafetyLevel
    from kmu_race.core.types import ConeReading

    pol = DrivePolicy(PolicyConfig())
    ready = SafetyDecision(SafetyLevel.OK, "", True)
    ctx = RaceContext()
    ctx.race_started_at = -10.0
    obs = _mk(1.0)
    obs.cone = ConeReading(
        detected=True, orange_px=5000, left_cone_px=4000, right_cone_px=0,
        gate_valid=False,
    )
    steer = pol.update(Mode.CONE_DRIVE, obs, ctx, ready).steer
    assert steer > 0.5, "왼쪽 콘만 보이면 오른쪽으로 가야 한다"
    assert abs(steer) < 40.0, "풀락은 반대편 콘 열로 처박는 동작이다"

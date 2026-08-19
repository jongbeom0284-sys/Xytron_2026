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
    """차가 왼쪽에 치우치면 목표선은 오른쪽(+)에 보여야 한다.

    0818 저녁: 실측 축척(≈810px/m)에서 도로 전폭(670px)은 BEV ROI(640px)보다
    넓어 '양쪽 흰 선이 나란히 보이는' 프레임은 실차에 없다 — 옛 합성 프레임
    (간격 270px)은 이제 '같은 물리 선 둘'로 병합되는 퇴화 입력이라, 실차와
    같은 **단측(오른 흰선만)** 프레임으로 부호를 본다.
    """
    def right_only(x):
        img = np.full((H, W, 3), 40, np.uint8)
        cv2.line(img, (x, H), (x - 40, H // 2), (255, 255, 255), 12)
        return img

    centered = LaneDetector().detect(right_only(500))
    # 흰 선을 오른쪽으로 밀면 = 차가 왼쪽에 있는 상황 → 목표(도로 중앙)도 오른쪽
    shifted = LaneDetector().detect(right_only(560))
    assert centered.right_found and shifted.right_found
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


# ──────────────────────────────────────────────────────────────────
# 0818 낮 실차 회귀 — 노란 중앙선 폴백 / 반사 웅덩이 / 콘 원거리 투영
# ──────────────────────────────────────────────────────────────────
def test_yellow_center_fallback_when_white_lines_gone():
    """곡선에서 흰 선이 시야 밖일 때 노란 점선만으로 목표선이 서야 한다.

    0818 bag 실측: 이 상황(노란곡선 창)에서 검출 유효율 7% → 직진 이탈.
    폴백 도입 후 87%. 흰 차선이 전혀 없고 노란 점선만 있는 합성 프레임.
    """
    from kmu_race.perception.lane_detector import LaneDetector

    img = np.full((480, 640, 3), 120, np.uint8)
    # 노란 점선 (BGR ≈ H26 S255 V255) — 화면 하단 중앙에 성기게
    for y in range(300, 460, 50):
        cv2.rectangle(img, (310, y), (335, y + 30), (0, 200, 255), -1)
    det = LaneDetector()
    r = det.detect(img)
    assert r.valid, "노란 점선만 있는 프레임에서 목표선이 서야 한다"
    assert r.center_found
    assert not r.left_found and not r.right_found


def test_glare_pool_removed_but_curved_lane_kept():
    """반사 웅덩이는 지우되 **휜 차선은 살린다** (0818 두 번 데인 지점).

    · 반사 웅덩이 = 뚱뚱하고 사각형을 꽉 채운다(fill 高) → 지운다.
    · 곡선 차선 = minAreaRect 는 뚱뚱하지만 대각선으로 가로지를 뿐이라
      성기다(fill 低) → 남긴다. 짧은 변만 보고 지우면 코너에서 차선이
      통째로 사라진다 (실측 양쪽 검출 27.1%→0.8% 붕괴).
    """
    from kmu_race.perception.lane_detector import LaneDetector

    roi = np.full((240, 640, 3), 100, np.uint8)
    cv2.rectangle(roi, (60, 40), (200, 170), (252, 252, 252), -1)   # 웅덩이
    cv2.polylines(                                                   # 휜 차선
        roi,
        [np.array([[380, 230], [430, 150], [520, 70], [610, 30]], np.int32)],
        False, (250, 250, 250), 12,
    )
    det = LaneDetector()
    binary = det._binarize(roi)
    pool = binary[40:170, 60:200]
    curve = binary[20:240, 360:640]
    assert np.count_nonzero(pool) < 0.25 * pool.size, "반사 웅덩이가 안 지워짐"
    assert np.count_nonzero(curve) > 0.02 * curve.size, "휜 차선까지 지워짐"


def test_cone_far_projection_creates_gate():
    """BEV ROI 밖(먼) 콘도 원본 프레임 바닥접점 투영으로 게이트가 된다.

    0818 실차: BEV 근접 밴드만 봐서 통로 한복판에서 게이트 성립률 0~37% →
    조향 근거 상실. 원본 프레임에 좌우 콘 열이 보이면 게이트가 서야 한다.
    """
    from kmu_race.perception.cone_detector import ConeDetector
    from kmu_race.perception.lane_detector import LaneDetector

    img = np.full((480, 640, 3), 90, np.uint8)
    # 좌우 콘 열 (주황 BGR) — 원근감 있게: 가까울수록 크고 바깥쪽
    for (cx, cy, s) in [(150, 430, 34), (200, 360, 26), (240, 310, 20),
                        (500, 430, 34), (460, 360, 26), (420, 310, 20)]:
        cv2.rectangle(img, (cx - s // 2, cy - s), (cx + s // 2, cy),
                      (0, 90, 255), -1)
    lane = LaneDetector()
    lane.detect(img)
    cone = ConeDetector()
    r = cone.detect(lane.last_bev_roi, frame=img, warp_m=lane.warp_matrix())
    assert r.detected
    assert r.gate_valid, "원거리 투영으로 게이트가 서야 한다"


# ──────────────────────────────────────────────────────────────────
# 0818 저녁 실차 프레임 감사 회귀 — 기하 상수 / 가짜 쌍 / 기하 게이트 / 접선
# ──────────────────────────────────────────────────────────────────
def _yellow_dashes(img, x, y0=300, y1=470, w=22, dash=28, gap=30):
    for y in range(y0, y1, dash + gap):
        cv2.rectangle(img, (x, y), (x + w, y + dash), (30, 215, 240), -1)


def test_center_candidate_outside_white_line_is_rejected():
    """오른 흰선 **바깥**의 노란(올리브 바닥) 조각은 중앙선이 될 수 없다.

    0818 run5 #112/#163/#220: 흰선 바깥 바닥 조각이 노란 게이트를 통과하고
    span 이 길어 목표선으로 뽑혀 +133~+247px(벽 쪽) 조향. 진짜 점선은 왼쪽.
    """
    from kmu_race.perception.lane_detector import LaneDetector

    img = np.full((H, W, 3), 120, np.uint8)
    cv2.line(img, (500, H), (470, 300), (250, 250, 250), 14)      # 오른 흰선
    _yellow_dashes(img, 240)                                       # 진짜 중앙선(왼쪽)
    cv2.rectangle(img, (522, 250), (560, 470), (30, 215, 240), -1)  # 바깥 노란 덩어리(길다)
    r = LaneDetector().detect(img)
    assert r.valid
    assert r.offset_px < -40, f"목표가 오른 흰선 바깥 조각으로 갔다 (offset={r.offset_px:+.0f})"


def test_fake_narrow_white_pair_does_not_veto_center_line():
    """이음새 같은 가짜 흰 선이 진짜 흰 선과 '쌍'을 이뤄도, 간격이 도로 전폭
    대역(520~900px)이 아니면 중점 목표·교차검증에 쓰지 않는다.

    0818 run5 #159~162: 바닥 이음새(x≈300)+진짜 오른선 → 중점 +80 으로 우측
    벽 방향 조향, 진짜 노란선은 교차검증에서 기각. 진짜 중앙은 −170.
    """
    from kmu_race.perception.lane_detector import LaneDetector

    img = np.full((H, W, 3), 120, np.uint8)
    cv2.line(img, (500, H), (470, 300), (250, 250, 250), 14)      # 오른 흰선
    cv2.line(img, (293, H), (285, 300), (235, 235, 235), 5)       # 이음새(얇은 가짜 선, BEV x≈300)
    _yellow_dashes(img, 240)                                       # 진짜 중앙선
    r = LaneDetector().detect(img)
    assert r.valid
    assert r.center_found, "가짜 쌍의 교차검증이 진짜 노란선을 기각했다"
    assert r.offset_px < -40, f"목표가 가짜 쌍의 중점으로 갔다 (offset={r.offset_px:+.0f})"


def test_heading_uses_tangent_at_nearest_observed_point_and_is_clipped():
    """헤딩 기울기는 관측 범위 끝의 **접선**이고(LaneFit.at 과 같은 규약),
    물리적으로 불가능한 값은 검출기 출력에서 잘린다.

    옛 코드는 -(2a·y_near+b) 로 관측 밖까지 2차식을 외삽했다 — 짧은 원거리
    조각에서 |Δ|×k_heading 가 p90 85cmd (0818 저녁 실측).
    """
    from kmu_race.perception.lane_detector import LaneConfig, LaneDetector, LaneFit

    det = LaneDetector()
    img = np.full((H, W, 3), 120, np.uint8)
    # 관측 범위 y 20~80 인 강한 곡률 조각: 접선(y=80) 기울기 = 2·0.01·80 − 1.5 = 0.1
    fit = LaneFit(coef=np.array([0.01, -1.5, 300.0]), y_lo=20.0, y_hi=80.0)
    det._fit_center = lambda c, lo=None, hi=None: None
    det._assign_identity = lambda fits, h, w: (None, fit)
    r = det.detect(img)
    assert r.right_found
    assert abs(r.heading_slope - (-0.1)) < 0.02, (
        f"접선이 아니라 외삽 기울기가 나왔다: {r.heading_slope:+.3f} (외삽이면 -3.28→클립 -0.6)")
    assert abs(r.curvature_px) <= LaneConfig().max_curvature_px + 1e-9, "곡률 클립이 안 걸렸다"

    # 접선 자체가 비물리적이면 클립
    fit2 = LaneFit(coef=np.array([0.0, 2.5, 100.0]), y_lo=100.0, y_hi=239.0)
    det._assign_identity = lambda fits, h, w: (None, fit2)
    r2 = det.detect(img)
    assert abs(r2.heading_slope) <= LaneConfig().max_heading_slope + 1e-9


def test_half_road_constants_match_measured_geometry():
    """0818 저녁 실측: 노란선↔흰선 335px, 흰↔흰 ≈670px (≈810 px/m).
    상수가 되돌아가면 단측 프레임(45%)의 목표가 140px 벽 쪽으로 밀린다."""
    from kmu_race.perception.lane_detector import LaneConfig

    c = LaneConfig()
    assert 300 <= c.half_road_default_px <= 370
    assert c.half_road_max_px >= 400, "max 330 은 진짜 표본의 절반을 기각한다"
    assert 600 <= c.default_lane_width_px <= 740
    assert c.pair_width_min_px > c.default_lane_width_px * 0.7

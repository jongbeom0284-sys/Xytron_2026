"""라이다 라바콘 게이트 회귀 — 0818 오후 신설 (perception/lidar_cones.py).

근거 실측 (bag auto_0818_1153 콘 구간 103스캔):
    콘 후보 평균 8.5개/스캔, 좌우 쌍 성립률 80%, 통로 폭 중앙값 0.91m.
카메라(58%)보다 높고 조명과 무관하며 미터를 직접 준다 — policy 1순위.
"""

import math

from kmu_race.core.policy import DrivePolicy
from kmu_race.core.race_context import RaceContext
from kmu_race.core.race_fsm import Mode
from kmu_race.core.safety import SafetyDecision, SafetyLevel
from kmu_race.core.types import ConeReading
from kmu_race.perception.lidar_cones import LidarConeConfig, find_cone_gate

from conftest import make_obs

READY = SafetyDecision(SafetyLevel.OK, "", True)

N = 360
ANGLE_MIN = -math.pi
INC = 2.0 * math.pi / N


def scan_with_points(points, base=10.0):
    """차량 기준 (x 전방+, y 좌측+) 점들을 하나의 스캔으로 합성한다."""
    ranges = [base] * N
    for x, y in points:
        r = math.hypot(x, y)
        a = math.atan2(y, x)
        i = int(round((a - ANGLE_MIN) / INC)) % N
        ranges[i] = min(ranges[i], r)
    return ranges


def cone_at(x, y, width=0.12, n=4):
    """콘 하나 = 좁은 각도 범위에 점 여러 개."""
    pts = []
    for k in range(n):
        off = (k / max(1, n - 1) - 0.5) * width
        pts.append((x, y + off))
    return pts


def test_left_right_pair_forms_gate():
    pts = cone_at(1.0, +0.45) + cone_at(1.0, -0.45)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert g.valid
    assert 0.7 <= g.width_m <= 1.1
    assert abs(g.lateral_m) < 0.10
    assert 0.8 <= g.depth_m <= 1.2


def test_same_side_rows_become_virtual_gate_not_row_gap():
    """같은 줄의 앞뒤 콘 **사이**를 조준하면 콘 줄로 처박는다.

    0821 밤부터는 '게이트 없음' 대신 **가상 게이트**를 낸다 — 보이는 줄에서
    차 쪽으로 반폭 옮긴 곳이 통로다. 그래도 줄 사이 틈을 조준하면 안 된다:
    그 경우 횡편차가 콘 줄 쪽(+0.4 근처)으로 나온다.
    """
    from kmu_race.perception.lidar_cones import LidarConeConfig

    cfg = LidarConeConfig()
    pts = cone_at(0.8, +0.4) + cone_at(1.4, +0.42)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert g.valid, "가상 게이트조차 안 난다 — 통로를 못 따라간다"
    want = 0.4 - cfg.corridor_half_m          # 왼쪽 줄 → 오른쪽으로 반폭
    assert abs(g.lateral_m - want) < 0.10, (
        f"횡편차 {g.lateral_m*100:+.0f}cm — 콘 줄 사이 틈을 조준하고 있다 "
        f"(가상 게이트라면 {want*100:+.0f}cm)")
    assert abs(g.width_m - 2 * cfg.corridor_half_m) < 0.05


def test_wide_cluster_is_not_a_cone():
    """사람·박스처럼 폭이 넓은 물체는 콘이 아니다."""
    wide = [(1.0, -0.3 + 0.05 * k) for k in range(14)]     # 폭 ~0.65m
    pts = cone_at(1.0, +0.45) + wide
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert not g.valid


def test_nearest_pair_wins():
    """여러 깊이의 쌍이 보이면 **지금 지나야 할(가장 가까운)** 쌍을 고른다."""
    near = cone_at(0.7, +0.4) + cone_at(0.7, -0.5)
    far = cone_at(1.8, +0.5) + cone_at(1.8, -0.4)
    g = find_cone_gate(scan_with_points(near + far), ANGLE_MIN, INC)
    assert g.valid
    assert g.depth_m < 1.0


def test_angle_offset_applied():
    """각도 보정(전진축=+124°)이 적용돼야 실차 스캔에서 전방을 본다."""
    offset = 2.164
    pts = cone_at(1.0, +0.45) + cone_at(1.0, -0.45)
    ranges = [10.0] * N
    for x, y in pts:
        r = math.hypot(x, y)
        a = math.atan2(y, x) - (-offset)     # 라이다 좌표계로 역회전
        i = int(round((a - ANGLE_MIN) / INC)) % N
        ranges[i] = min(ranges[i], r)
    g = find_cone_gate(ranges, ANGLE_MIN, INC, angle_offset=-offset)
    assert g.valid


def test_policy_prefers_lidar_gate():
    """라이다 게이트가 있으면 카메라 게이트보다 우선한다 (부호까지).

    라이다: 게이트 중앙이 왼쪽(+0.3m) → 좌조향(음수).
    카메라: 게이트 중앙이 오른쪽(+120px) → 우조향 — 이게 이기면 안 된다.
    """
    policy = DrivePolicy()
    ctx = RaceContext()
    obs = make_obs(10.0)
    obs.cone = ConeReading(
        detected=True, orange_px=5000,
        gate_valid=True, gate_center_px=120.0,
        gate_depth_px=200.0, gate_width_px=300.0,
        lidar_gate_valid=True, lidar_gate_depth_m=0.8,
        lidar_gate_width_m=0.9, lidar_gate_lateral_m=+0.30, lidar_cones=6,
    )
    cmd = policy.update(Mode.CONE_DRIVE, obs, ctx, READY)
    assert cmd.steer < -2.0, f"라이다(좌) 우선이어야 하는데 steer={cmd.steer}"


def test_policy_falls_back_to_camera_gate():
    policy = DrivePolicy()
    ctx = RaceContext()
    obs = make_obs(10.0)
    obs.cone = ConeReading(
        detected=True, orange_px=5000,
        gate_valid=True, gate_center_px=120.0,
        gate_depth_px=200.0, gate_width_px=300.0,
    )
    cmd = policy.update(Mode.CONE_DRIVE, obs, ctx, READY)
    assert cmd.steer > 2.0, f"카메라(우) 폴백이어야 하는데 steer={cmd.steer}"


def test_outer_row_only_on_curve_aims_back_into_corridor():
    """★ 통로 밖으로 밀렸을 때 게이트가 **더 바깥**을 가리키면 안 된다.

    0821 밤 폐루프 실측(cone_corridor_sim, R1.2m 좌회전 통로, 차가 중심선에서
    바깥으로 30cm). 안쪽 줄은 시야에서 사라지고 **바깥 줄(오른쪽)만** 남는데,
    그 줄이 곡률 때문에 차 축을 가로질러 올라온다:

        (0.33, -0.05) (0.83, +0.14) (1.22, +0.50) (1.48, +0.96)

    옛 코드는 y 부호로 줄을 판정해서 (0.83,+0.14) 를 '왼쪽 줄'로 읽고 통로
    중심을 그 콘의 **오른쪽** 반폭에 놓았다 — 폭 82cm·횡 **-26.7cm**.
    차는 그대로 통로 밖으로 나갔다(2m 이탈). 통로는 저 줄의 **왼쪽**이다.
    """
    row = [(0.33, -0.05), (0.83, +0.14), (1.22, +0.50), (1.48, +0.96)]
    pts = []
    for x, y in row:
        pts += cone_at(x, y)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert g.valid, "가상 게이트조차 안 선다"
    assert g.lateral_m > 0.15, (
        f"횡 {g.lateral_m*100:+.1f}cm — 통로 바깥(오른쪽)을 가리킨다. "
        f"보이는 줄은 오른쪽 줄이고 통로는 그 왼쪽이다")
    assert g.near_row_side == -1, "가장 가까운 콘의 줄이 오른쪽으로 판정돼야 한다"


def test_row_side_carries_over_when_anchor_crosses_axis():
    """기준 콘이 y=0 을 스치면 부호가 한 스캔 만에 뒤집힌다 — 직전 쪽을 잇는다.

    같은 통로에서 차가 더 밀리고 요까지 틀어지면 오른쪽 줄의 최근접 콘이
    y=+0.03 으로 넘어온다. 부호만 보면 줄 전체가 '왼쪽'이 되어 목표가 통로
    바깥으로 뒤집힌다 (실측 -38.1cm). 줄은 움직이지 않으므로 8.8Hz 사이에
    소속이 바뀔 수 없다 — 직전 스캔의 쪽을 그대로 쓴다.
    """
    row = [(0.35, +0.03), (0.74, +0.38), (1.00, +0.84)]
    pts = []
    for x, y in row:
        pts += cone_at(x, y)
    scan = scan_with_points(pts)

    flipped = find_cone_gate(scan, ANGLE_MIN, INC)          # 힌트 없음
    assert flipped.lateral_m < 0.0, (
        "이 장면은 힌트 없이는 뒤집힌다 — 회귀 감지가 성립해야 한다")

    g = find_cone_gate(scan, ANGLE_MIN, INC, prev_row_side=-1)
    assert g.valid
    assert g.lateral_m > 0.15, (
        f"횡 {g.lateral_m*100:+.1f}cm — 직전 스캔이 '오른쪽 줄'이라 했는데 "
        f"y 부호에 끌려 통로 밖을 가리킨다")
    assert g.near_row_side == -1


def test_real_pair_still_beats_row_side_on_a_curve():
    """굽은 통로라도 좌우 쌍이 보이면 그 중점이 목표다 (판정이 과하면 안 된다)."""
    pts = cone_at(0.34, -0.14) + cone_at(0.82, +0.06) + cone_at(0.53, +0.75)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert g.valid
    assert g.lateral_m > 0.20, f"횡 {g.lateral_m*100:+.1f}cm — 통로 안쪽을 못 본다"


# ── 실측 장면 (cone_0821_0651 t=11.10s) ────────────────────────────
#   차가 라바콘 통로에서 오른쪽 줄을 긁던 순간, 라이다가 실제로 본 콘 7개.
#   좌표는 bag 재생으로 뽑았다 (kmu_race/perception 의 _cone_clusters 출력).
REAL_SCENE_1110 = [
    (0.336, -0.187),    # ← 차 오른쪽 19cm. 이게 지금 긁히고 있는 콘이다.
    (0.667, +0.056),
    (0.980, +0.328),
    (1.267, +0.650),
    (0.382, +0.863),
    (0.643, +1.198),    # ← 차 왼쪽 1.2m. 82cm 통로 안에 있을 수 없다.
    (2.347, +0.500),
]


def test_real_scene_does_not_aim_across_a_cone_beside_the_car():
    """★ 실측 회귀 — 오른쪽 19cm 에 콘이 있는데 왼쪽 76cm 를 조준했다.

    cone_0821_0651 t=11.10s. 게이트는 (0.643,+1.198) 과 (0.980,+0.328) 을
    짝지어 '폭 87cm 통로, 중심 +76cm' 라고 했다. 왼쪽 1.2m 의 콘은 82cm
    통로 안에 있을 수 없고, 중심이 +76cm 라면 오른쪽 줄이 +35cm — 차의
    **왼쪽** — 이라 오른쪽 19cm 의 콘이 설명되지 않는다.
    차는 그 목표를 좌 풀락(-42)으로 2초간 쫓았고 차체 여유가 -7.5cm 까지
    파고들었다 (접촉 스캔 17개).
    """
    pts = []
    for x, y in REAL_SCENE_1110:
        pts += cone_at(x, y)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert g.valid, "이 장면에선 게이트가 서야 한다"
    assert math.isfinite(g.near_right_m) and g.near_right_m < 0.30, (
        f"오른쪽 옆 콘을 못 봤다 (우근접 {g.near_right_m}) — 장면 재현 실패")
    assert g.lateral_m < 0.40, (
        f"횡 {g.lateral_m*100:+.1f}cm — 오른쪽 {g.near_right_m*100:.0f}cm 에 콘이 "
        f"있는데 그보다 훨씬 왼쪽을 조준한다")
    assert 0.70 <= g.width_m <= 0.95, (
        f"폭 {g.width_m*100:.0f}cm — 통로 폭(82cm) 근처여야 한다")

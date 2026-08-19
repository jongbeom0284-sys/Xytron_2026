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


def test_same_side_rows_do_not_gate():
    """같은 줄의 앞뒤 콘 사이를 게이트로 오인하면 콘 줄로 조향한다."""
    pts = cone_at(0.8, +0.4) + cone_at(1.4, +0.42)
    g = find_cone_gate(scan_with_points(pts), ANGLE_MIN, INC)
    assert not g.valid


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

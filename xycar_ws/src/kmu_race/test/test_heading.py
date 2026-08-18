"""자이로 적분 헤딩 (core/heading.py) — 0817 IMU 포렌식 대응 회귀.

실측 배경: kobot IMU 는 orientation 쿼터니언이 회전을 안 담고(2바퀴 bag
오일러 3축 적산 전부 ~0°), 각속도에는 고정 바이어스(gz +2.67°/s)가 있다.
바이어스를 정지 구간에서 빼고 적분하면 gz 가 -811° ≈ 2바퀴+주차회전으로
정확히 복원된다. 이 모듈은 그 경로의 수치 계약을 잠근다.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from kmu_race.core.heading import GyroHeading, GyroHeadingConfig  # noqa: E402

BIAS = math.radians(2.67)      # 실측 gz 바이어스
HZ = 30.0


def feed(h, rate_rad_s, seconds, t0):
    t = t0
    for _ in range(int(seconds * HZ)):
        t += 1.0 / HZ
        h.update(0.0, 0.0, rate_rad_s, t)
    return t


def test_stationary_bias_integrates_to_zero():
    """정지 상태 바이어스만 있으면 (캘리브 후) 헤딩은 0 에 머문다.

    보정이 없으면 2.67°/s × 60s = 160° 가짜 회전이 쌓인다 — 실측 bag 에서
    자이로 y 가 +695° 로 보였던 것이 정확히 이 함정이었다.
    """
    h = GyroHeading()
    t = feed(h, BIAS, 5.0, 0.0)      # 캘리브 구간 포함 정지
    feed(h, BIAS, 60.0, t)           # 계속 정지
    assert h.calibrated
    assert abs(h.yaw_deg) < 1.0


def test_turn_is_recovered_on_top_of_bias():
    """바이어스 위에 실린 실제 회전이 그대로 복원된다."""
    h = GyroHeading()
    t = feed(h, BIAS, 4.0, 0.0)                       # 정지 캘리브
    turn = math.radians(-72.0)                        # 지름길 좌회전 수준
    t = feed(h, BIAS + turn / 2.0, 2.0, t)            # 2초간 -36°/s
    assert h.calibrated
    assert abs(h.yaw_deg - (-72.0)) < 3.0


def test_calibration_window_keeps_zero():
    """캘리브레이션이 끝나기 전에는 0 을 유지한다 (정지 가정 구간)."""
    h = GyroHeading(GyroHeadingConfig(calib_s=3.0))
    feed(h, BIAS, 2.0, 0.0)
    assert not h.calibrated
    assert h.yaw_deg == 0.0


def test_time_jump_is_not_integrated():
    """드랍아웃(dt 비정상)과 시간 역행은 적분하지 않는다."""
    h = GyroHeading()
    t = feed(h, 0.0, 4.0, 0.0)
    h.update(0.0, 0.0, math.radians(30.0), t + 5.0)   # 5초 구멍 — 무시
    h.update(0.0, 0.0, math.radians(30.0), t + 4.0)   # 시간 역행 — 무시
    assert abs(h.yaw_deg) < 0.1


def test_nan_input_is_ignored():
    h = GyroHeading()
    t = feed(h, 0.0, 4.0, 0.0)
    before = h.yaw_deg
    h.update(0.0, 0.0, float("nan"), t + 1 / HZ)
    assert h.yaw_deg == before


def test_axis_selection():
    """축 설정이 해당 축만 읽는다 (IMU 재장착 대비)."""
    h = GyroHeading(GyroHeadingConfig(axis="y", calib_s=1.0))
    t = feed(h, 0.0, 2.0, 0.0)
    for _ in range(int(2.0 * HZ)):
        t += 1.0 / HZ
        h.update(0.0, math.radians(45.0), math.radians(-99.0), t)
    assert abs(h.yaw_deg - 90.0) < 3.0

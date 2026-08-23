"""라이다 차로 안 정지물체 회피 (0823 밤).

실측 근거 — 214410 세션 f306~332, 원본 스캔에서 재계산:
    f316 이전 : 차로 안 아무것도 없음 (오검출 0)
    f317      : 1점  1.19m  y-0.33  우측
    f318      : 2점  1.02m  y-0.30  우측
    f319      : 4점  0.86m  y-0.29  우측
    ... f332 까지 16프레임 연속 전부 우측
같은 구간에서 YOLO 는 f324 에 car 0.48 이 처음이자 마지막이었다 (충돌 직전).
"""
import math
import types

from race_last.Cone_Perception import (OBS_HALF_W, OBS_X_MAX, OBS_X_MIN,
                                       ConePerception)


def scan(points, offset=0.0):
    """(전방x, 횡y) 목록을 그대로 담은 가짜 LaserScan 을 만든다."""
    n = 360
    ranges = [float('inf')] * n
    inc = 2.0 * math.pi / n
    for x, y in points:
        r = math.hypot(x, y)
        a = math.atan2(y, x) - offset          # _obstacle 이 offset 을 더한다
        i = int(round((a % (2.0 * math.pi)) / inc)) % n
        ranges[i] = r
    return types.SimpleNamespace(ranges=ranges, angle_min=0.0,
                                 angle_increment=inc)


def obstacle(points):
    """ConePerception._obstacle 만 떼어 부른다 (노드 초기화 없이)."""
    fake = types.SimpleNamespace(offset=0.0)
    return ConePerception._obstacle(fake, scan(points))


def test_empty_lane_reports_nothing():
    """차로가 비면 아무것도 내지 않는다 — f316 이전이 그랬다."""
    d = obstacle([])
    assert d['side'] == '' and d['n'] == 0


def test_single_point_is_enough():
    """1점이어도 잡는다. 라바콘은 원뿔이라 라이다 높이에 1점만 맞는다 (f317)."""
    d = obstacle([(1.19, -0.33)])
    assert d['side'] == 'R'
    assert abs(d['dist'] - 1.19) < 0.02
    assert d['n'] == 1


def test_left_object():
    d = obstacle([(1.0, 0.30), (1.05, 0.33)])
    assert d['side'] == 'L'


def test_outside_lane_ignored():
    """차로 폭 밖(양옆 벽·연석)은 무시한다."""
    assert obstacle([(1.0, OBS_HALF_W + 0.2)])['side'] == ''
    assert obstacle([(1.0, -(OBS_HALF_W + 0.2))])['side'] == ''


def test_range_limits():
    """너무 가깝거나(자기 범퍼) 너무 먼 점은 안 본다."""
    assert obstacle([(OBS_X_MIN - 0.1, 0.0)])['side'] == ''
    assert obstacle([(OBS_X_MAX + 0.5, 0.0)])['side'] == ''


def test_nearest_group_wins():
    """멀리 있는 큰 덩어리가 아니라 가장 가까운 것을 낸다."""
    d = obstacle([(0.8, -0.30),                       # 가까운 우측 1점
                  (2.0, 0.30), (2.05, 0.33), (2.1, 0.35)])   # 먼 좌측 3점
    assert d['side'] == 'R'
    assert abs(d['dist'] - 0.8) < 0.02
    assert d['groups'] == 2

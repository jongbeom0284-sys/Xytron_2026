"""차로 안 물체는 좁은 덩어리여야 한다 — 벽·연석을 가른다 (0823 밤).

실측 230555 (밴드 반폭 0.45 그대로, 덩어리의 횡폭만 잰 것):
    정지차량  f272 0.60m  2점 횡폭 0.008   f274 0.28m 19점 횡폭 0.077
    곡선 벽   f286 2.40m 29점 횡폭 0.868   f293 1.12m 32점 횡폭 0.618
겹치는 구간이 없다. 0.25 로 자르면 벽 정면 6프레임(f286~f291)이 전부 떨어지고
정지차량 4프레임은 전부 남는다 (실기록으로 확인).

밴드 반폭을 넓히는 것은 소용이 없었다 — 0.90 까지 올려도 f264~271 은 전부
'없음' 이고 f272 값도 그대로다. 라이다에 반사 자체가 안 온다.

주의: 점은 반드시 '호(arc)' 위에 놓는다. 실제 라이다가 그렇게 준다.
      x 를 완전히 고정해 만들면 _obstacle 의 x 정렬 순서가 뒤섞여 한 덩어리가
      1점짜리 여러 개로 쪼개진다 — 실제 데이터에서는 안 생기는 일이다.
"""
import math
import types

from race_last.Cone_Perception import OBS_MAX_WIDE, ConePerception


def scan(points, offset=0.0):
    n = 1440
    ranges = [float('inf')] * n
    inc = 2.0 * math.pi / n
    for x, y in points:
        r = math.hypot(x, y)
        a = math.atan2(y, x) - offset
        i = int(round((a % (2.0 * math.pi)) / inc)) % n
        ranges[i] = r
    return types.SimpleNamespace(ranges=ranges, angle_min=0.0,
                                 angle_increment=inc)


def obstacle(points):
    return ConePerception._obstacle(types.SimpleNamespace(offset=0.0),
                                    scan(points))


def arc(r, y_from, y_to, n):
    """거리 r 인 호 위에 점을 뿌린다 — 벽·물체가 라이다에 보이는 모양."""
    if n == 1:
        return [(math.sqrt(max(r * r - y_from * y_from, 0.0)), y_from)]
    out = []
    for k in range(n):
        y = y_from + (y_to - y_from) * k / (n - 1)
        out.append((math.sqrt(max(r * r - y * y, 0.0)), y))
    return out


def test_narrow_object_kept():
    """정지차량 — 좁은 덩어리는 통과한다 (실측 f274: 19점 횡폭 0.077)."""
    d = obstacle(arc(0.70, -0.39, -0.31, 6))
    assert d['side'] == 'R'
    assert d['wide'] <= OBS_MAX_WIDE


def test_single_point_kept():
    """1점짜리(횡폭 0)도 통과 — 라바콘은 1점만 맞는 일이 흔하다."""
    d = obstacle([(1.19, -0.33)])
    assert d['side'] == 'R'
    assert d['n'] == 1


def test_wide_wall_rejected():
    """곡선 바깥 벽 — 횡폭 0.8m 는 물체가 아니다 (실측 f286~f291)."""
    d = obstacle(arc(2.40, -0.44, 0.36, 40))
    assert d['side'] == '', '벽을 물체로 보면 반대쪽으로 조향한다'
    assert d['n'] == 0
    assert d['groups'] >= 1, '덩어리 자체는 세어 둔다 (진단용)'


def test_wall_at_threshold():
    """경계값 — 0.25m 까지는 물체, 넘으면 벽."""
    assert obstacle(arc(1.0, -0.11, 0.11, 12))['side'] != ''
    assert obstacle(arc(1.0, -0.20, 0.20, 20))['side'] == ''


def test_narrow_object_wins_over_nearer_wall():
    """벽이 더 가까워도, 좁은 덩어리를 골라야 한다."""
    pts = arc(0.95, -0.40, 0.40, 40) + arc(1.65, -0.33, -0.27, 5)
    d = obstacle(pts)
    assert d['side'] == 'R'
    assert 1.5 < d['dist'] < 1.7, '넓은 덩어리는 건너뛴다: %s' % d


def test_nearest_narrow_wins():
    """좁은 덩어리가 둘이면 가까운 쪽."""
    pts = arc(0.85, -0.33, -0.28, 4) + arc(2.05, 0.28, 0.33, 4)
    d = obstacle(pts)
    assert d['side'] == 'R'
    assert d['dist'] < 1.0, d

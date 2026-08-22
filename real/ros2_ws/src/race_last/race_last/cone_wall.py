"""라바콘 벽 추종 — 콘을 짝짓지 않고 좌·우 벽을 세워 그 중앙을 따라간다.

왜 이렇게 하나 (2026-08-23 실주행으로 하나씩 확인한 것들):

  기존 find_cone_gate 는 좌콘 하나와 우콘 하나를 '짝지어' 게이트를 만든다.
  실측 성립률이 19% 였고, 콘이 3열로 놓인 구간에서는 엉뚱한 짝을 지어
  폭 0.69m 짜리 가짜 게이트를 냈다 (실측 통로는 0.82m).

  그래서 짝짓기를 버리고 벽을 세운다:
    1) 게이트 찾기 — 차를 사이에 두고 걸치는, 통로 폭만큼 떨어진 콘 쌍
    2) 그 선의 양 끝이 좌·우 씨앗                    (부호로 좌우를 가르지 않는다)
    3) 씨앗에서 이어지는 콘만 붙여 벽을 뻗는다        (새 씨앗은 안 생긴다)
    4) 두 벽에 직선을 맞추고 그 중앙을 조준한다

  각 조건이 실주행 사고 하나씩에 대응한다:
    · 부호로 좌우를 갈랐더니, 통로가 굽어 좌벽 먼 부분이 중심선을 넘자
      우벽으로 분류돼 조준이 +29 → -58cm 로 뒤집혔다   → 게이트 씨앗
    · 거리 조건만 봤더니 콘 열과 뒤쪽 벽을 번갈아 이어 직선 잔차가 25cm
      까지 벌어졌다 (벽을 콘으로 인식)                 → MAX_OFFLINE
    · 같은 깊이의 콘 두 개를 한 벽으로 묶어 기울기가 발산, 조준이 -6169cm
      까지 튀며 조향이 즉시 포화했다                   → MIN_ADVANCE, MIN_SPAN
    · 전방주시를 속도에 비례시켰더니(1.0m/s 에서 1.10m) 모든 지표가 나빠졌다.
      통로 82cm·콘 간격 42cm 라 벽 콘 범위를 넘어 외삽하는 거리였다.
        0.55m → |조준| 중앙 2cm, 통로밖 3회, 프레임간 변화 0.5
        1.10m → |조준| 중앙 16cm, 통로밖 13회, 프레임간 변화 4.5
                                                       → LOOKAHEAD 고정

  이 구성으로 1.0 m/s 통과를 확인했다 (~/cone_go.sh 12.5 1.5 20 0).
"""
import math

from .kmu_race.perception.lidar_cones import LidarConeConfig

CFG = LidarConeConfig()

# 콘 간격은 실측 40~44cm. 아래 값들은 그 위에서 정했다.
MAX_LINK = 0.50           # 이보다 벌어지면 같은 벽으로 안 잇는다
MAX_OFFLINE = 0.12        # 지금까지의 벽 방향에서 이보다 벗어나면 다른 물체
MIN_ADVANCE = 0.15        # 벽은 앞으로 뻗는다. 같은 깊이의 쌍은 벽이 아니다
MIN_SPAN = 0.20           # 벽 앞뒤 폭이 이보다 좁으면 기울기를 믿지 않는다
MAX_AIM = 0.41            # 조준 횡오차 상한 — 통로 반폭. 그 밖은 계산 오류다

GATE_MIN = 0.65           # 게이트로 인정할 통로 폭 하한
GATE_MAX = 0.98           # 상한. 실측 통로는 0.82m
GATE_DEPTH_DIFF = 0.25    # 게이트는 진행 방향에 수직 — 실측 깊이차 3cm
GATE_MAX_OFFSET = 0.40    # 통로 중앙이 차에서 이보다 멀면 그 통로가 아니다

TRACK_W = 0.82            # 통로 실측 폭
LOOKAHEAD = 0.55          # 전방주시. 속도와 무관하게 고정 (위 주석 참조)
MAX_STEER = 40.0          # yolop_drive 와 같은 legacy 조향 상한
STEER_RATE = 6.0          # 한 틱에 바꿀 수 있는 조향량 (20Hz 기준 초당 120)


def clusters(ranges, angle_min, angle_increment, angle_offset):
    """스캔 → 콘 후보 중심. 판정 기준은 lidar_cones 와 같다."""
    pts = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < CFG.min_range_m or r > CFG.max_range_m:
            continue
        a = angle_min + i * angle_increment + angle_offset
        x, y = r * math.cos(a), r * math.sin(a)
        if x >= CFG.min_x_m and abs(y) <= CFG.max_abs_y_m:
            pts.append((x, y))
    groups, cur = [], []
    for p in pts:
        if cur and math.hypot(p[0] - cur[-1][0],
                              p[1] - cur[-1][1]) > CFG.cluster_gap_m:
            groups.append(cur)
            cur = []
        cur.append(p)
    if cur:
        groups.append(cur)
    out = []
    for g in groups:
        width = math.hypot(g[0][0] - g[-1][0], g[0][1] - g[-1][1])
        if len(g) >= CFG.min_points and width <= CFG.max_width_m:
            out.append((sum(p[0] for p in g) / len(g),
                        sum(p[1] for p in g) / len(g)))
    return out


def _offline(wall, cand):
    """지금까지의 벽 방향에서 후보가 옆으로 벗어난 거리 [m].

    콘 벽은 직선이다. 콘 열과 뒤쪽 벽을 번갈아 집으면 이 값이 커진다.
    콘이 하나뿐이면 방향을 모르므로 조건을 걸지 않는다.
    """
    if len(wall) < 2:
        return 0.0
    a, b = wall[-2], wall[-1]
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return 0.0
    ex, ey = cand[0] - b[0], cand[1] - b[1]
    return abs(ex * dy - ey * dx) / length


def _grow(seed, pool, taken):
    """씨앗에서 이어지는 콘을 앞으로 붙여 벽을 만든다.

    거리(50cm 안) · 전진(15cm 이상) · 방향(직선에서 12cm 안) 을 모두 만족해야
    이어 붙인다. 직선을 가장 잘 잇는 후보를 먼저 고른다.
    """
    wall = [seed]
    while True:
        best, best_key = None, None
        for k, cand in enumerate(pool):
            if k in taken:
                continue
            if math.hypot(cand[0] - wall[-1][0],
                          cand[1] - wall[-1][1]) > MAX_LINK:
                continue
            if cand[0] - wall[-1][0] < MIN_ADVANCE:
                continue
            off = _offline(wall, cand)
            if off > MAX_OFFLINE:
                continue
            key = (off, math.hypot(cand[0] - wall[-1][0],
                                   cand[1] - wall[-1][1]))
            if best_key is None or key < best_key:
                best, best_key = k, key
        if best is None:
            return wall
        taken.add(best)
        wall.append(pool[best])


def find_gate(pool):
    """차를 사이에 두고 걸치는 콘 쌍. 돌려주는 값: (좌 인덱스, 우 인덱스).

    한쪽에만 있는 쌍은 지나갈 통로가 아니다 — 그런 쌍이 씨앗이 되면 우측 콘이
    좌벽으로 승격되고 진짜 좌벽이 통째로 버려진다 (0823 실측).
    """
    best, best_key = None, None
    for i in range(len(pool)):
        for j in range(i + 1, len(pool)):
            a, b = pool[i], pool[j]
            if abs(a[0] - b[0]) > GATE_DEPTH_DIFF:
                continue
            if (a[1] > 0) == (b[1] > 0):
                continue
            width = math.hypot(a[0] - b[0], a[1] - b[1])
            if not GATE_MIN <= width <= GATE_MAX:
                continue
            if abs((a[1] + b[1]) / 2.0) > GATE_MAX_OFFSET:
                continue
            key = (round((a[0] + b[0]) / 2.0, 2), abs(width - TRACK_W))
            if best_key is None or key < best_key:
                best_key = key
                best = (i, j) if a[1] > b[1] else (j, i)
    return best


def build_walls(cones):
    """좌·우 벽과 어디에도 안 붙은 나머지를 돌려준다.

    씨앗은 게이트에서만 나온다. 게이트가 없으면 벽도 없다 — 없는 벽을
    지어내면 저 멀리 있는 물체가 통로 벽이 된다.
    """
    pool = sorted(cones, key=lambda c: c[0])
    gate = find_gate(pool)
    if gate is None:
        return [], [], list(pool)
    li, ri = gate
    taken = {li, ri}
    left = _grow(pool[li], pool, taken)
    right = _grow(pool[ri], pool, taken)
    used = {(round(c[0], 4), round(c[1], 4)) for c in left + right}
    rest = [c for c in pool if (round(c[0], 4), round(c[1], 4)) not in used]
    return left, right, rest


def fit(wall):
    """벽에 y = a·x + b 를 맞춘다. 못 맞추면 None.

    앞뒤 폭이 좁으면 기울기가 발산하므로(0823: 폭 0cm 에서 조준 -43m) 그때는
    기울기 0 으로 두고 평균 옆위치만 쓴다 — 차와 나란한 벽으로 본다.
    """
    if len(wall) < 2:
        return None
    span = max(p[0] for p in wall) - min(p[0] for p in wall)
    if span < MIN_SPAN:
        return 0.0, sum(p[1] for p in wall) / len(wall)
    k = len(wall)
    sx = sum(p[0] for p in wall)
    sy = sum(p[1] for p in wall)
    den = k * sum(p[0] * p[0] for p in wall) - sx * sx
    if abs(den) < 1e-9:
        return 0.0, sy / k
    a = (k * sum(p[0] * p[1] for p in wall) - sx * sy) / den
    return a, (sy - a * sx) / k


def centre_line(left, right):
    """중앙선 (기울기, 절편, 근거). 못 내면 None.

    양쪽 다 직선이 맞으면 평균. 한쪽만 맞으면 게이트 중점을 지나게 하고
    기울기만 그 벽에서 가져온다. 한쪽 벽만 보이면 통로 절반만큼 민다.
    """
    fl, fr = fit(left), fit(right)
    if fl and fr:
        return (fl[0] + fr[0]) / 2.0, (fl[1] + fr[1]) / 2.0, 'both'
    if left and right:
        gx = (left[0][0] + right[0][0]) / 2.0
        gy = (left[0][1] + right[0][1]) / 2.0
        slope = fl[0] if fl else (fr[0] if fr else 0.0)
        return slope, gy - slope * gx, 'gate'
    if fl:
        return fl[0], fl[1] - TRACK_W / 2.0, 'left'
    if fr:
        return fr[0], fr[1] + TRACK_W / 2.0, 'right'
    return None


def solve(ranges, angle_min, angle_increment, angle_offset, bias_m=0.0):
    """스캔 한 장 → 조준 결과. 벽을 못 세우면 None.

    돌려주는 값 (dict):
      aim_y   전방주시 지점의 횡오차 [m]. 양수는 왼쪽
      basis   both / gate / left / right
      left    좌벽 콘 목록, right  우벽 콘 목록
    """
    cones = clusters(ranges, angle_min, angle_increment, angle_offset)
    left, right, _rest = build_walls(cones)
    line = centre_line(left, right)
    if line is None:
        return None
    slope, icept, basis = line
    aim_y = slope * LOOKAHEAD + icept - bias_m
    # 통로가 82cm 뿐이다. 그 밖을 겨누는 값은 계산이 터진 것이다.
    aim_y = max(-MAX_AIM, min(MAX_AIM, aim_y))
    return dict(aim_y=aim_y, basis=basis, left=left, right=right,
                lookahead=LOOKAHEAD)


def steer_for(aim_y, gain, prev_steer=None):
    """조준 횡오차 → 조향값 (legacy 단위).

    이 시스템은 조향 양수가 우회전이다 (motor_adapter 가 부호를 뒤집어
    Ackermann 각으로 넘긴다). 조준점이 왼쪽이면 음수를 내야 한다.
    prev_steer 를 주면 한 틱에 STEER_RATE 넘게 안 바뀌도록 깎는다.
    """
    want = -gain * math.degrees(math.atan2(aim_y, LOOKAHEAD))
    want = max(-MAX_STEER, min(MAX_STEER, want))
    if prev_steer is None:
        return want
    return max(prev_steer - STEER_RATE, min(prev_steer + STEER_RATE, want))

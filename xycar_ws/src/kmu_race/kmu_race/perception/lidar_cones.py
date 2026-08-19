"""라이다 라바콘 게이트 — 조명·반사와 무관한 제2의 눈 (0818 오후 신설).

왜 만들었나 (0818 실측):
  카메라 콘 게이트는 조명·과노출·바닥 반사에 휘둘려 실주행 성립률이
  22~58% 였다. 반면 라바콘은 라이다에 **작고 고립된 점 클러스터**로 또렷이
  찍힌다 — 같은 bag 의 같은 구간에서 스캔당 콘 후보 8.5개, 좌우 쌍(게이트)
  성립률 **80%**, 통로 폭 중앙값 **0.91m**. 게다가 라이다는

    · 조명·노출·반사와 완전히 무관하고 (실내 형광등·유리창 반사 무영향)
    · 거리를 **미터로 직접** 준다 — 카메라 경로의 px_per_m 추정(420, 미실측)
      이 통째로 필요 없어진다. 조준점 오차의 큰 원인 하나가 사라진다.

  그래서 콘 구간의 조향 근거는 라이다를 1순위, 카메라를 보조로 쓴다.

한계:
  · 라이다는 한 평면만 본다. 콘이 낮게 잘려 보이거나(설치 높이) 사람 다리·
    의자 다리도 비슷한 크기로 잡힌다 → 크기 상한과 '좌우 쌍' 조건으로 거른다.
  · 통로가 넓게 열린 구간(콘 끝)에서는 쌍이 안 만들어진다 → 그때는 카메라
    또는 단측 가상 게이트로 자연히 넘어간다 (policy 담당).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class LidarConeConfig:
    # 유효 거리 [m] — 0.30 미만은 자기 반사(차체), 2.5 초과는 조향에 무의미
    min_range_m: float = 0.30
    max_range_m: float = 2.50
    # 전방 부채꼴 (x 전방+, y 좌측+)
    min_x_m: float = 0.15
    max_abs_y_m: float = 1.20
    # 인접점이 이보다 멀면 다른 물체 (클러스터 분리 기준)
    cluster_gap_m: float = 0.14
    # 클러스터가 콘이려면: 점 2개 이상, 폭 0.30m 이하
    # (실측 콘 밑면 ≈0.20m. 사람 다리 0.15~0.2 도 통과하지만 좌우 쌍
    #  조건에서 걸러진다 — 사람은 통로 양쪽에 대칭으로 서 있지 않다.)
    min_points: int = 2
    max_width_m: float = 0.30
    # 게이트: 좌우(y 부호 반대) + 깊이 차 0.45m 이내 + 폭 0.35~1.60m
    # 실측 통로 폭 0.91m 를 넉넉히 감싼다.
    max_depth_diff_m: float = 0.45
    min_gate_m: float = 0.35
    max_gate_m: float = 1.60


@dataclass
class LidarGate:
    valid: bool = False
    depth_m: float = 0.0        # 게이트까지 전방 거리
    width_m: float = 0.0        # 좌우 콘 사이 간격
    lateral_m: float = 0.0      # 게이트 중앙의 횡위치 (좌측 +)
    cones: int = 0              # 이번 스캔의 콘 후보 수


def _cluster(
    pts: Sequence[Tuple[float, float]], gap: float, min_points: int
) -> List[Tuple[float, float, float, int]]:
    """각도 순으로 정렬된 점들을 인접 거리로 묶는다 → (cx, cy, 폭, 점수)."""
    out: List[Tuple[float, float, float, int]] = []
    cur: List[Tuple[float, float]] = []

    def flush() -> None:
        if len(cur) < min_points:
            return
        xs = [p[0] for p in cur]
        ys = [p[1] for p in cur]
        w = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        out.append((sum(xs) / len(xs), sum(ys) / len(ys), w, len(cur)))

    for p in pts:
        if not cur:
            cur = [p]
            continue
        if math.hypot(p[0] - cur[-1][0], p[1] - cur[-1][1]) <= gap:
            cur.append(p)
        else:
            flush()
            cur = [p]
    flush()
    return out


def find_cone_gate(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    angle_offset: float = 0.0,
    config: Optional[LidarConeConfig] = None,
) -> LidarGate:
    """LaserScan 원시값 → 콘 게이트. rclpy 의존 없음(테스트 가능)."""
    cfg = config or LidarConeConfig()

    pts: List[Tuple[float, float, float]] = []      # (bearing, x, y)
    for i, r in enumerate(ranges):
        if not (cfg.min_range_m < r < cfg.max_range_m):
            continue
        if not math.isfinite(r):
            continue
        a = angle_min + i * angle_increment + angle_offset
        x = r * math.cos(a)
        y = r * math.sin(a)
        if x < cfg.min_x_m or abs(y) > cfg.max_abs_y_m:
            continue
        pts.append((math.atan2(y, x), x, y))

    if len(pts) < cfg.min_points * 2:
        return LidarGate()

    pts.sort()
    cones = [
        c
        for c in _cluster(
            [(p[1], p[2]) for p in pts], cfg.cluster_gap_m, cfg.min_points
        )
        if c[2] <= cfg.max_width_m
    ]
    if len(cones) < 2:
        return LidarGate(cones=len(cones))

    # 좌우 마주보는 쌍 중 **가장 가까운** 것 = 지금 지나야 할 게이트
    best: Optional[Tuple[float, float, float]] = None
    for i in range(len(cones)):
        for j in range(i + 1, len(cones)):
            a, b = cones[i], cones[j]
            if a[1] * b[1] >= 0.0:                       # 같은 쪽이면 게이트 아님
                continue
            if abs(a[0] - b[0]) > cfg.max_depth_diff_m:  # 깊이가 다르면 다른 쌍
                continue
            gap = abs(a[1] - b[1])
            if not (cfg.min_gate_m <= gap <= cfg.max_gate_m):
                continue
            depth = (a[0] + b[0]) / 2.0
            if best is None or depth < best[0]:
                best = (depth, gap, (a[1] + b[1]) / 2.0)

    if best is None:
        return LidarGate(cones=len(cones))
    return LidarGate(
        valid=True,
        depth_m=best[0],
        width_m=best[1],
        lateral_m=best[2],
        cones=len(cones),
    )


__all__ = ["LidarConeConfig", "LidarGate", "find_cone_gate"]

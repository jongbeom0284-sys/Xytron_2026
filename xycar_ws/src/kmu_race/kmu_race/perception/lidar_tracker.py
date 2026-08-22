"""라이다 클러스터링 + 트래킹.

두 가지를 의도적으로 다르게 만들었다.

1) **각도 기반 섹터.** 흔한 구현은 `ranges[245:255]` 처럼 인덱스로 정면을 잡는데,
   드라이버가 스캔 수를 바꾸거나 라이다를 다시 장착하면 '정면'이 엉뚱한 곳을
   가리킨다. 여기서는 `angle_min`/`angle_increment` 로 실제 각도를 계산한다.

2) **트래킹 + 속도 추정.** 규정상 고정장애물(p.30 고장난 차량)과 방해차량(p.32)은
   같은 크기의 차량 모형이라 형상으로 못 가른다. 유일한 단서가 상대속도라서
   프레임 간 대응(association)을 붙여 vx/vy 를 추정한다.

좌표계: x 전방(+), y 좌측(+). 라이다 0rad 이 차량 정면을 향한다고 가정한다
(자이카 기본 장착). 다르면 `angle_offset` 으로 보정할 것.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

from ..core.types import LidarReading, TrackedObject


@dataclass
class LidarConfig:
    # 라이다 0rad 이 정면이 아닐 때의 보정 [rad] — a_car = a_lidar + offset.
    # ★ 0817 실측 −2.164 (kobot 장착: 차 전진축 = 라이다 +124°).
    #   근거 3중 일치: ① 3바퀴 bag 직진 창 거리변화율 피크 라이다 +122°
    #   (−0.53 m/s ≈ 차속) ② 점군 흐름 벡터 +122° ③ 자기 반사(라이다 −62°,
    #   0.11m)가 보정 후 차 후방 176° = 라이다 뒤 케이블/마운트. 콘 구간
    #   재생에서 콘이 앞쪽 양옆(y ±0.4~0.6, x 0.8~1.3m)으로 정렬됨을 확인.
    #   보정 전에는 정면 섹터가 차 후방-좌측을 보고 있어 뒤에 선 사람을
    #   '0.45m 앞 방해차량'으로 오인, 첫 자율주행이 OVERTAKE 무한루프였다.
    #   ⚠️ 라이다 재장착 시 재실측: 차 정면 1m 에 박스 두고 preflight.
    angle_offset: float = -2.164

    # ── 섹터 정의 [rad] ──
    front_half_angle: float = 0.26     # ±15°
    side_near: float = 0.61            # 35°
    side_far: float = 1.57             # 90°

    # ── 유효 거리 ──
    # ★ min_range 0.10 → 0.30 (0817 첫 자율주행 실사고): 라이다 −72~−52°
    #   0.11~0.25m 에 차체 부품(마운트/케이블) 자기 반사 11점이 상시 존재 —
    #   x>0 이라 코리도 로직이 '정면 0.12m 방해차량'으로 오인, 출발 직후
    #   4분간 OVERTAKE 기동 반복. 차체 반경(≈0.25m) 안쪽 반환은 물리적으로
    #   자기 몸이다 (실제 장애물이 0.3m 안이면 이미 충돌).
    min_range: float = 0.30
    max_range: float = 6.0

    # ── 클러스터링 (adaptive breakpoint) ──
    # 인접 점 간 허용거리 = base + slope * range
    cluster_base_m: float = 0.055
    cluster_slope: float = 0.045
    min_cluster_points: int = 3
    max_cluster_width_m: float = 0.85

    # ── 통행 가능 판정 ──
    # 옆으로 비킬 때 필요한 최소 여유거리 [m]
    side_clear_m: float = 0.85

    # ── 트래킹 ──
    assoc_max_dist_m: float = 0.45     # 프레임 간 같은 물체로 볼 최대 이동거리
    max_missed_frames: int = 5
    velocity_alpha: float = 0.35       # 속도 추정 저역통과 계수


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    width: float
    vx: float = 0.0
    vy: float = 0.0
    age: int = 0
    missed: int = 0
    stamp: float = 0.0


class LidarTracker:
    def __init__(self, config: Optional[LidarConfig] = None) -> None:
        self.config = config or LidarConfig()
        self._tracks: List[_Track] = []
        self._next_id = 1

    # ── 스캔 → 점군 ───────────────────────────────────────────────
    def _to_points(
        self, ranges: Sequence[float], angle_min: float, angle_inc: float
    ) -> List[Tuple[float, float, float]]:
        """(x, y, r) 리스트. 무효 값은 버린다."""
        cfg = self.config
        pts: List[Tuple[float, float, float]] = []
        for i, r in enumerate(ranges):
            if r is None or not math.isfinite(r):
                continue
            if r < cfg.min_range or r > cfg.max_range:
                continue
            a = angle_min + i * angle_inc + cfg.angle_offset
            pts.append((r * math.cos(a), r * math.sin(a), r))
        return pts

    # ── 클러스터링 ────────────────────────────────────────────────
    def _cluster(
        self, pts: Sequence[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """반환 (cx, cy, width). adaptive breakpoint 로 인접 점을 묶는다."""
        cfg = self.config
        out: List[Tuple[float, float, float]] = []
        if not pts:
            return out

        current: List[Tuple[float, float, float]] = [pts[0]]
        for prev, cur in zip(pts, pts[1:]):
            gap = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
            limit = cfg.cluster_base_m + cfg.cluster_slope * min(prev[2], cur[2])
            if gap <= limit:
                current.append(cur)
            else:
                out.extend(self._finish_cluster(current))
                current = [cur]
        out.extend(self._finish_cluster(current))
        return out

    def _finish_cluster(
        self, pts: Sequence[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        cfg = self.config
        if len(pts) < cfg.min_cluster_points:
            return []
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        width = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
        if width > cfg.max_cluster_width_m:
            return []
        # ★ 대표점 = 중심이 아니라 **가장 가까운 점** (0820 저녁).
        #   호(arc)처럼 각도로 넓게 퍼진 클러스터의 중심은 물체가 있는 자리가
        #   아니다 — 차 양옆을 감싸는 점들의 평균은 원점 근처로 떨어져,
        #   모든 점이 min_range(0.30m) 밖인데도 중심 거리가 0.16m 로 나온다.
        #   run_0820_1821 실측: 전방 코리도 트리거의 최근접 x 가 0.15~0.28m 에
        #   몰려 있었는데, 그 거리는 차체 반경 안쪽이라 물리적으로 '전방
        #   장애물'일 수가 없다(진짜라면 이미 충돌).
        #   충돌 판단에 의미 있는 점도 최근접점이므로 대표점을 그리 옮긴다.
        near = min(pts, key=lambda p: p[2])
        return [(near[0], near[1], width)]

    # ── 트래킹 ────────────────────────────────────────────────────
    def _associate(
        self, clusters: Sequence[Tuple[float, float, float]], now: float
    ) -> None:
        cfg = self.config

        # 전역 최근접 매칭: (거리, 트랙, 클러스터) 쌍을 거리순으로 확정한다.
        # 트랙 순회 순서대로 선점하는 그리디는 두 물체가 근접 교차할 때
        # 트랙-클러스터를 스왑시켜 vx 를 오염시킨다 (리뷰 P4).
        pairs: List[Tuple[float, int, int]] = []
        for ti, tr in enumerate(self._tracks):
            for ci, c in enumerate(clusters):
                d = math.hypot(c[0] - tr.x, c[1] - tr.y)
                if d < cfg.assoc_max_dist_m:
                    pairs.append((d, ti, ci))
        pairs.sort(key=lambda p: p[0])

        used_t: set = set()
        used_c: set = set()
        for _d, ti, ci in pairs:
            if ti in used_t or ci in used_c:
                continue
            used_t.add(ti)
            used_c.add(ci)
            tr = self._tracks[ti]
            best = clusters[ci]

            # 시간 역전·중복 스탬프(dt<=0)나 극소 dt 는 속도를 갱신하지 않는다.
            # 0.01s 로 눌러 나누면 몇 cm 변위가 수 m/s 가짜 속도로 둔갑해
            # 고정장애물이 방해차량으로 오분류된다 (리뷰 P3).
            dt = now - tr.stamp if tr.stamp > 0 else 0.0
            if dt > 1e-3:
                vx = (best[0] - tr.x) / dt
                vy = (best[1] - tr.y) / dt
                a = cfg.velocity_alpha
                tr.vx = (1 - a) * tr.vx + a * vx
                tr.vy = (1 - a) * tr.vy + a * vy
            tr.x, tr.y, tr.width = best
            tr.stamp = now
            tr.age += 1
            tr.missed = 0

        for ti, tr in enumerate(self._tracks):
            if ti not in used_t:
                tr.missed += 1

        for ci, c in enumerate(clusters):
            if ci not in used_c:
                self._tracks.append(
                    _Track(self._next_id, c[0], c[1], c[2], stamp=now)
                )
                self._next_id += 1

        self._tracks = [
            t for t in self._tracks if t.missed <= cfg.max_missed_frames
        ]

    # ── 메인 ──────────────────────────────────────────────────────
    def update(
        self,
        ranges: Sequence[float],
        *,
        angle_min: float,
        angle_increment: float,
        now: float,
    ) -> LidarReading:
        cfg = self.config
        pts = self._to_points(ranges, angle_min, angle_increment)
        if not pts:
            return LidarReading(stamp=now)

        clusters = self._cluster(pts)
        self._associate(clusters, now)

        front = float("inf")
        left = float("inf")
        right = float("inf")
        for x, y, r in pts:
            a = math.atan2(y, x)
            if abs(a) <= cfg.front_half_angle:
                front = min(front, r)
            elif cfg.side_near <= a <= cfg.side_far:
                left = min(left, r)
            elif -cfg.side_far <= a <= -cfg.side_near:
                right = min(right, r)

        objects = tuple(
            TrackedObject(
                x=t.x, y=t.y, width=t.width, track_id=t.track_id,
                vx=t.vx, vy=t.vy, age=t.age,
            )
            for t in sorted(self._tracks, key=lambda t: t.x * t.x + t.y * t.y)
            if t.missed == 0
        )

        return LidarReading(
            front_dist=front,
            left_dist=left,
            right_dist=right,
            left_clear=left >= cfg.side_clear_m,
            right_clear=right >= cfg.side_clear_m,
            objects=objects,
            stamp=now,
        )


__all__ = ["LidarConfig", "LidarTracker"]

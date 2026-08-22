"""신호등 '위치' 판정 — 색을 전혀 쓰지 않는다.

왜 색을 버리는가 (0817 실측):
  대회장 카메라는 과노출이라 8~10m 출발선에서 점등 램프가 채도 12 의 흰 점이
  된다. 색으로 빨강/초록을 가르려던 시도는 청록 반사(G−B 19.9)에 속아
  **오출발**로 이어졌다. 색 정보는 이 거리에서 물리적으로 존재하지 않는다.

무엇을 대신 쓰는가 — 규정이 못박은 램프 배치:
      좌 → 우 : [ 빨강 ][ 노랑 ][ 좌회전← ][ 초록 ]
  램프 위치는 고정이고, 대기 중 차는 정지해 있다. 따라서

      **"켜져 있던 불빛이 꺼지고, 그 오른쪽에 새 불빛이 켜졌다"
        = 빨강이 꺼지고 초록(또는 좌회전)이 켜졌다 = 출발**

  색·절대밝기·거리 보정이 전혀 필요 없다. 필요한 건 '작고 둥근 밝은 점'을
  찾는 것뿐이고, 그건 과노출에서 오히려 가장 쉬운 일이다. 덤으로 이동량이
  램프 간격(px)을 알려 줘 기하가 자동 보정된다.

왜 '한 점'이 아니라 '꺼짐+켜짐 쌍'인가:
  실측 프레임에서 밝은 점 후보는 프레임당 13~37개다 (천장 조명, 창문, 노트북,
  사람 옷의 반사). 그중 하나를 기준점으로 고르는 방식은 잘못 고를 위험이 크다.
  반면 **"성숙한 트랙이 사라진 그 순간, 같은 줄 오른쪽 1.6~6.5 램프폭 지점에
  새 트랙이 태어난다"** 는 사건은 정지 반사광이 흉내 낼 수 없다. 신호등의
  물리적 동작 그 자체이기 때문이다.

한계와 방어:
  · 대기 중 빨간불을 한 번도 못 보면 사건이 성립하지 않는다 → 판정을 포기하고
    상위(FSM)의 기존 경로에 맡긴다. '모르면 출발'은 절대 없다.
  · 주행 중에는 풍경이 흘러 트랙이 유지되지 않는다 → 그때는 직좌(두 점 동시
    점등)만 '간격' 규칙으로 본다. 간격은 상대량이라 거리와 무관하다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from . import lamp_pitch


# ──────────────────────────────────────────────────────────────────
@dataclass
class LampBlob:
    """점등 램프 후보 — 작고 둥글고 밝은 점."""

    cx: float
    cy: float
    w: float
    h: float
    brightness: float

    @property
    def size(self) -> float:
        return max(self.w, self.h)


@dataclass
class LampFinderConfig:
    # 과노출 점등 심의 밝기 하한. 실측: 점등 250 / 소등 하우징 145~204.
    core_v: int = 220
    # 램프 심 크기 [px] — 실측 8~10m 7px, 4~5m 5~8px, 통과 직전 20~30px
    min_px: int = 3
    max_px: int = 34
    max_aspect: float = 2.0
    min_fill: float = 0.55
    # 세로 대역 (프레임 높이 비율) — 겐트리 램프는 카메라 피치상 이 밖으로
    # 나가지 않는다. 천장 조명·창밖 하늘 구멍을 구조적으로 배제한다.
    # ⚠️ 카메라 재장착·겐트리 이동 시 재실측 (0817 실측 y_full 187~219).
    y_min_frac: float = 0.28
    y_max_frac: float = 0.62


def find_lamp_blobs(
    frame: np.ndarray, cfg: Optional[LampFinderConfig] = None
) -> List[LampBlob]:
    """프레임 → 점등 램프 후보들. 색을 쓰지 않는다 (밝기·모양·위치만)."""
    cfg = cfg or LampFinderConfig()
    if frame is None or getattr(frame, "size", 0) == 0:
        return []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    h, w = gray.shape[:2]
    mask = (gray >= cfg.core_v).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
    n, labels, stats, cent = cv2.connectedComponentsWithStats(mask)

    out: List[LampBlob] = []
    for i in range(1, n):
        x, y, bw, bh, area = (int(stats[i][k]) for k in range(5))
        if bw < cfg.min_px or bw > cfg.max_px or area < 4:
            continue
        if max(bw / max(1, bh), bh / max(1, bw)) > cfg.max_aspect:
            continue
        if area / float(bw * bh) < cfg.min_fill:
            continue
        cx, cy = float(cent[i][0]), float(cent[i][1])
        if not (cfg.y_min_frac * h <= cy <= cfg.y_max_frac * h):
            continue
        out.append(
            LampBlob(cx, cy, float(bw), float(bh), float(gray[labels == i].mean()))
        )
    return out


# ──────────────────────────────────────────────────────────────────
@dataclass
class _Track:
    cx: float
    cy: float
    w: float
    h: float
    born: float
    last_seen: float
    hits: int = 1
    x0: float = 0.0        # 태어난 위치 (이동량 검사용)
    y0: float = 0.0
    drift: float = 0.0     # 수명 동안 최대 이동량 [px]

    @property
    def size(self) -> float:
        return max(self.w, self.h)


@dataclass
class WatcherConfig:
    # 트랙 매칭 반경 (램프 크기 배수)
    match_mult: float = 1.8
    # 이만큼 유지돼야 '성숙한' 트랙 (반사 깜빡임 배제) [s]
    # ★ 0818 실차 오출발: 차 앞 0.44m 를 지나가는 사람의 밝은 옷이 0.7s 짜리
    #   트랙을 만들고 사라진 뒤 오른쪽에 새 트랙이 태어나 '점등 이동'으로
    #   판정됐다. 진짜 빨간불은 수 초~수십 초 켜져 있으므로 3.0s 를 요구한다.
    mature_s: float = 3.0
    # 성숙 트랙이 이만큼 안 보이면 '꺼졌다' [s]
    death_s: float = 0.15
    # 새 트랙이 이만큼 유지돼야 '켜졌다' [s] — 지나가는 사람은 0.5s 를 못 버틴다
    birth_s: float = 0.5
    # 트랙 수명 동안 허용되는 최대 이동 (램프 크기 배수). 램프는 정지해
    # 있고 사람은 움직인다 — 이동한 트랙은 anchor 도, 새 점등도 될 수 없다.
    max_drift_mult: float = 1.2
    # 꺼짐과 켜짐이 이 시간 안에 함께 일어나야 한 사건이다 [s]
    pair_window_s: float = 1.2
    # 오른쪽 이동량 허용 범위 (램프폭 배수). 규정 배치상 빨강→좌회전 2칸,
    # 빨강→초록 3칸이고 실측 간격/지름 ≈ 1.25 → 2.5d / 3.75d.
    switch_min_mult: float = 1.6
    switch_max_mult: float = 6.5
    # 같은 하우징이면 세로가 거의 같다 (램프 높이 배수)
    row_tol_mult: float = lamp_pitch.PAIR_ROW_TOL_MULT
    # 같은 신호등의 램프끼리는 크기가 비슷하다
    size_ratio_max: float = lamp_pitch.PAIR_SIZE_RATIO_MAX
    # ── 직좌 동시신호 (두 점) ── 창은 perception/lamp_pitch.py 단일 출처.
    # ★ 0819: 상한 2.2 → 1.75. 2.2 는 하한 0.9 의 2.44배라 인접(1피치)뿐
    #   아니라 **2피치 떨어진 쌍**까지 통과시켰다. 직좌는 3·4번 = 인접
    #   한 칸이므로 2피치 쌍은 물리적으로 불가능한 오검 표면이었다.
    pair_gap_min_mult: float = lamp_pitch.PAIR_GAP_MIN_MULT
    pair_gap_max_mult: float = lamp_pitch.PAIR_GAP_MAX_MULT
    # 직좌 쌍의 양옆 여백 — 4번(초록)은 유닛 오른쪽 끝이고 3번 왼쪽의
    # 2번(노랑)은 직좌에서 꺼져 있다. 같은 줄 점등체 열은 신호등이 아니다.
    pair_neighbour_clear_mult: float = lamp_pitch.PAIR_NEIGHBOUR_CLEAR_MULT
    # 두 점 왼쪽에 '꺼진 램프' 자리가 있어야 한다 (하우징 구조 확인).
    # 이게 0817 책상 반사 오검을 죽인 게이트다.
    require_dark_left: bool = True
    dark_left_v_max: int = 150
    dark_left_frac_min: float = 0.15


@dataclass
class WatchResult:
    go: bool = False              # 출발해도 되는가 (오른쪽 램프로 점등 이동)
    green_left: bool = False      # 직좌 동시신호 (두 점 동시 점등)
    spacing_px: float = 0.0       # 측정된 램프 간격 (기하 자동보정 결과)
    steps_right: int = 0          # 몇 칸 오른쪽으로 옮겨갔나 (2=좌회전, 3=초록)
    tracked: int = 0
    reason: str = ""


@dataclass
class SignalWatcher:
    """램프 위치 변화를 지켜본다 (모듈 주석의 설계)."""

    config: WatcherConfig = field(default_factory=WatcherConfig)

    _tracks: List[_Track] = field(default_factory=list)
    _deaths: List[Tuple[float, _Track]] = field(default_factory=list)
    _fired: bool = False

    def reset(self) -> None:
        self._tracks.clear()
        self._deaths.clear()
        self._fired = False

    # ── 보조 ──────────────────────────────────────────────────────
    def _dark_left_ok(self, gray: Optional[np.ndarray], a: LampBlob) -> bool:
        """왼쪽에 꺼진 램프(어두운 하우징)가 있는가 — 단독 발광체 배제."""
        c = self.config
        if not c.require_dark_left or gray is None:
            return True
        s = a.size
        x0 = int(max(0, a.cx - 4.0 * s))
        x1 = int(max(0, a.cx - 0.8 * s))
        y0 = int(max(0, a.cy - 0.8 * s))
        y1 = int(min(gray.shape[0], a.cy + 0.8 * s))
        if x1 <= x0 or y1 <= y0:
            return False
        strip = gray[y0:y1, x0:x1]
        if strip.size == 0:
            return False
        frac = float(np.count_nonzero(strip < c.dark_left_v_max)) / strip.size
        return frac >= c.dark_left_frac_min

    def _pair_window(self) -> lamp_pitch.PairWindow:
        c = self.config
        return lamp_pitch.PairWindow(
            gap_min_mult=c.pair_gap_min_mult,
            gap_max_mult=c.pair_gap_max_mult,
            size_ratio_max=c.size_ratio_max,
            row_tol_mult=c.row_tol_mult,
            neighbour_clear_mult=c.pair_neighbour_clear_mult,
        )

    def _find_pair(
        self, blobs: List[LampBlob], gray: Optional[np.ndarray]
    ) -> Optional[Tuple[LampBlob, LampBlob]]:
        """직좌 동시점등 쌍. 기하는 lamp_pitch 가, 하우징 구조는 여기가 본다."""
        cands = [
            lamp_pitch.LampCandidate(cx=b.cx, cy=b.cy, size=b.size, h=b.h)
            for b in blobs
        ]
        found = lamp_pitch.find_adjacent_pair(
            cands,
            self._pair_window(),
            accept=lambda i, _j: self._dark_left_ok(gray, blobs[i]),
        )
        if found is None:
            return None
        i, j = found
        return blobs[i], blobs[j]

    # ── 메인 ──────────────────────────────────────────────────────
    def update(
        self,
        blobs: List[LampBlob],
        now: float,
        *,
        stationary: bool = True,
        gray: Optional[np.ndarray] = None,
    ) -> WatchResult:
        c = self.config
        res = WatchResult(tracked=len(self._tracks))

        pair = self._find_pair(blobs, gray)
        if pair is not None:
            res.green_left = True
            res.spacing_px = pair[1].cx - pair[0].cx

        if not stationary:
            # 주행 중에는 풍경이 흘러 트랙이 성립하지 않는다.
            self._tracks.clear()
            self._deaths.clear()
            res.reason = "moving (직좌 판정만)"
            return res

        # ── 트랙 매칭 ──
        used = set()
        for t in self._tracks:
            best, bd = None, 1e9
            for i, b in enumerate(blobs):
                if i in used:
                    continue
                d = abs(b.cx - t.cx) + abs(b.cy - t.cy)
                tol = c.match_mult * max(t.size, b.size)
                if abs(b.cx - t.cx) <= tol and abs(b.cy - t.cy) <= tol and d < bd:
                    best, bd = i, d
            if best is not None:
                b = blobs[best]
                used.add(best)
                t.cx = 0.6 * t.cx + 0.4 * b.cx
                t.cy = 0.6 * t.cy + 0.4 * b.cy
                t.w, t.h = b.w, b.h
                t.last_seen = now
                t.hits += 1
                t.drift = max(t.drift, abs(t.cx - t.x0) + abs(t.cy - t.y0))

        # ── 죽은 트랙 수확 ──
        alive: List[_Track] = []
        for t in self._tracks:
            if now - t.last_seen >= c.death_s:
                stationary_track = t.drift <= c.max_drift_mult * max(1.0, t.size)
                if t.last_seen - t.born >= c.mature_s and stationary_track:
                    self._deaths.append((t.last_seen, t))
            else:
                alive.append(t)
        self._tracks = alive
        self._deaths = [
            (dt, t) for dt, t in self._deaths if now - dt <= c.pair_window_s
        ]

        # ── 새 트랙 생성 ──
        for i, b in enumerate(blobs):
            if i in used:
                continue
            self._tracks.append(_Track(b.cx, b.cy, b.w, b.h, now, now, 1, b.cx, b.cy))

        res.tracked = len(self._tracks)

        # ── 사건 판정: 꺼진 자리의 오른쪽에 새 불빛이 태어났는가 ──
        if not self._fired:
            for dt, dead in self._deaths:
                for t in self._tracks:
                    if t.born < dt - c.pair_window_s:
                        continue          # 원래 있던 불빛
                    if now - t.born < c.birth_s:
                        continue          # 아직 확정 전
                    if t.drift > c.max_drift_mult * max(1.0, t.size):
                        continue          # 움직이는 물체(사람)는 점등이 아니다
                    dx = t.cx - dead.cx
                    if dx <= 0:
                        continue
                    size = max(dead.size, t.size)
                    small = min(dead.size, t.size)
                    if small <= 0 or size / small > c.size_ratio_max:
                        continue
                    if not (c.switch_min_mult * size <= dx <= c.switch_max_mult * size):
                        continue
                    if abs(t.cy - dead.cy) > c.row_tol_mult * max(dead.h, t.h):
                        continue
                    steps = 3 if dx > 3.0 * size else 2
                    res.go = True
                    res.steps_right = steps
                    res.spacing_px = dx / steps
                    res.reason = (
                        f"점등 이동 Δx={dx:.0f}px ({steps}칸 오른쪽) — 출발"
                    )
                    self._fired = True
                    return res

        res.reason = (
            "대기 중 (성숙 트랙 %d, 최근 소등 %d)"
            % (sum(1 for t in self._tracks if now - t.born >= c.mature_s),
               len(self._deaths))
        )
        return res


__all__ = [
    "LampBlob",
    "LampFinderConfig",
    "find_lamp_blobs",
    "WatcherConfig",
    "WatchResult",
    "SignalWatcher",
]

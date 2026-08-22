"""겐트리 램프 배치의 단일 출처 — 규정 p.35/p.36 배치 + 실측 피치비.

규정이 못박은 배치:  좌 → 우  [ 빨강 ][ 노랑 ][ 좌회전← ][ 초록 ]

네 램프는 한 하우징에 **등간격**으로 박혀 있다. 그래서 인접 램프 중심간격
(피치)을 램프 겉보기 크기로 나눈 값은 **거리와 무관한 하드웨어 상수**다.
램프가 7px 로 보이든 35px 로 보이든 같은 수가 나온다 — 이 무차원 비율이
'진짜 신호등'과 '나란한 흰 물체 두 개'를 가르는, 유일하게 스케일 불변인
특징이다.

왜 이 파일이 생겼나 (2026-08-19):
  같은 물리량이 세 곳에 서로 다르게 박혀 있었다.
    · signal_watcher `_find_pair`   : gap ∈ [0.9, 2.2]·size   ← 실측 인용본
    · signal_detector `_washed_pair`: gap ∈ [1.2, 4.5]·bw
    · signal_detector `_shape_ok`   : min_gap=3·r_min, horiz≤11·r_max 로
                                      피치/지름 ≈ [1.5, 1.83] 을 암묵 가정
  특히 두 번째는 **상한이 하한의 3.75배**라, 인접(1피치)뿐 아니라 2피치·
  3피치 떨어진 쌍까지 통과시킨다. 직좌 동시신호는 정의상 3·4번 = **인접
  한 칸**이므로 그 창은 오검 경로 그 자체였다 (0817 책상·박스 페어 오검).

여기 상수 하나를 고치면 세 경로가 같이 움직인다. 실측으로 피치비를 다시
재면 **이 파일만** 고칠 것.

실측 피치비 = 1.25:
  signal_watcher 주석 "실측 간격/지름 ≈ 1.25", 그리고 test_signal_watcher 의
  `LAMP_D = 8` / `SPACING = 10` (= 1.25) 로 두 번 확인된다.

⚠️ 미검증: 1.25 가 램프 렌즈 지름 기준인지 과노출 심 지름 기준인지는 기록에
   구분돼 있지 않다. 아래 창은 1.25 를 안에 두되 상한/하한 비를 2 미만으로
   유지하는 최소 조정이다(watcher 상한 2.2 → 1.75). 진짜 직좌 프레임이 담긴
   bag 으로 피치비를 확정하기 전까지 여기서 더 좁히지 말 것.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

# ── 인접 램프 판정 창 (겉보기 크기 배수) ────────────────────────────
# 실측 간격/지름 1.25 를 담는 창. 하한은 watcher 승계(0.9), 상한은 2.2 →
# 1.75 (1.75/0.9 = 1.94 < 2 — 아래 __post_init__ 의 인접성 조건).
# 1.25 기준 여유는 아래 −28% / 위 +40%.
PAIR_GAP_MIN_MULT: float = 0.9
PAIR_GAP_MAX_MULT: float = 1.75
# 같은 하우징의 램프끼리는 크기가 비슷하다.
PAIR_SIZE_RATIO_MAX: float = 2.4
# 같은 하우징이면 세로가 거의 같다 (램프 높이 배수).
PAIR_ROW_TOL_MULT: float = 1.8
# 직좌 쌍의 양옆 여백 (겉보기 크기 배수). 4번(초록)은 유닛의 오른쪽 끝이고
# 3번(좌회전) 왼쪽 2번(노랑)은 직좌에서 꺼져 있다 — 이 범위 안에 같은 줄·
# 비슷한 크기의 점등체가 또 있으면 신호등 하우징이 아니라 물체 열이다.
# (규정 p.35/36 배치에서 직접 나오는 구조 게이트 — 실측 상수가 아니다.)
PAIR_NEIGHBOUR_CLEAR_MULT: float = 2.5


@dataclass(frozen=True)
class LampCandidate:
    """점등 램프 후보 — 모듈마다 다른 블롭 표현을 이 형태로 맞춰 넘긴다."""

    cx: float
    cy: float
    size: float   # 겉보기 지름 [px] (블롭 bbox 의 max(w, h))
    h: float      # 세로 크기 [px] — 같은 줄 판정용


@dataclass(frozen=True)
class PairWindow:
    """인접 램프 판정 창. 노드 파라미터로 덮어쓸 수 있게 값으로 들고 다닌다."""

    gap_min_mult: float = PAIR_GAP_MIN_MULT
    gap_max_mult: float = PAIR_GAP_MAX_MULT
    size_ratio_max: float = PAIR_SIZE_RATIO_MAX
    row_tol_mult: float = PAIR_ROW_TOL_MULT
    neighbour_clear_mult: float = PAIR_NEIGHBOUR_CLEAR_MULT

    def __post_init__(self) -> None:
        # 상한이 하한의 2배를 넘으면 인접(1피치)과 2피치를 구분할 수 없다 —
        # 직좌는 반드시 인접이므로 그런 창은 정의상 잘못된 창이다.
        if self.gap_max_mult >= 2.0 * self.gap_min_mult:
            raise ValueError(
                f"pair gap window [{self.gap_min_mult}, {self.gap_max_mult}] 은 "
                "2피치 쌍까지 통과시킨다 — 직좌는 인접 한 칸이다"
            )


def adjacent_ok(a: LampCandidate, b: LampCandidate, win: PairWindow) -> bool:
    """a(왼쪽), b(오른쪽) 두 후보가 '한 하우징의 인접 램프'인가."""
    big, small = max(a.size, b.size), min(a.size, b.size)
    if small <= 0 or big / small > win.size_ratio_max:
        return False
    gap = b.cx - a.cx
    if not (win.gap_min_mult * big <= gap <= win.gap_max_mult * big):
        return False
    if abs(a.cy - b.cy) > win.row_tol_mult * max(a.h, b.h):
        return False
    return True


def _has_similar_neighbour(
    anchor: LampCandidate,
    others: Sequence[LampCandidate],
    win: PairWindow,
    *,
    to_right: bool,
) -> bool:
    """anchor 의 한쪽 옆 clear_mult 안에 같은 줄·비슷한 크기 점등체가 있는가."""
    for o in others:
        if o is anchor:
            continue
        dx = (o.cx - anchor.cx) if to_right else (anchor.cx - o.cx)
        if dx <= 0 or dx > win.neighbour_clear_mult * anchor.size:
            continue
        if abs(o.cy - anchor.cy) > win.row_tol_mult * max(o.h, anchor.h):
            continue
        big, small = max(o.size, anchor.size), min(o.size, anchor.size)
        if small > 0 and big / small <= win.size_ratio_max:
            return True
    return False


def flanks_are_clear(
    left: LampCandidate,
    right: LampCandidate,
    others: Sequence[LampCandidate],
    win: PairWindow,
) -> bool:
    """직좌 쌍의 양옆이 규정대로 비어 있는가 — 점등체 열(列)을 죽이는 게이트.

    규정 p.36 의 직좌 동시신호는 **정확히 3·4번 두 개만** 켜진다.
      · 4번(초록)은 유닛의 오른쪽 끝  → 오른쪽에 점등체가 있을 수 없다.
      · 3번(좌회전) 왼쪽은 2번(노랑) → 직좌에서 노랑은 꺼져 있다.
    그래서 '같은 줄에 같은 간격으로 늘어선 밝은 것 셋 이상'은 신호등이
    아니다. 책상 위 물건·천장 조명 열이 정확히 그 모양이고, 0817 오검의
    주된 형태였다. 놓치는 쪽으로 틀리는 게이트다 — 직진은 항상 합법이므로
    (규정 p.35 "직진할지 좌회전할지 스스로 결정") 안전한 방향이다.
    """
    if _has_similar_neighbour(right, others, win, to_right=True):
        return False
    if _has_similar_neighbour(left, others, win, to_right=False):
        return False
    return True


def find_adjacent_pair(
    cands: Sequence[LampCandidate],
    win: Optional[PairWindow] = None,
    accept: Optional[Callable[[int, int], bool]] = None,
) -> Optional[Tuple[int, int]]:
    """직좌 동시점등 후보쌍의 인덱스 (왼쪽, 오른쪽). 없으면 None.

    이 함수는 **기하만** 본다. 호출자 고유의 게이트(왼쪽 소등 검사 등)는
    `accept(i, j)` 로 넘긴다 — 스캔 중에 걸러야 한 쌍이 탈락해도 다음 쌍을
    계속 볼 수 있다 (호출 후 판정하면 첫 쌍에서 멈춰 버린다).
    """
    win = win or PairWindow()
    order: List[int] = sorted(range(len(cands)), key=lambda i: cands[i].cx)
    for k in range(len(order) - 1):
        i, j = order[k], order[k + 1]
        if not adjacent_ok(cands[i], cands[j], win):
            continue
        if not flanks_are_clear(cands[i], cands[j], cands, win):
            continue
        if accept is not None and not accept(i, j):
            continue
        return i, j
    return None


__all__ = [
    "PAIR_GAP_MIN_MULT",
    "PAIR_GAP_MAX_MULT",
    "PAIR_SIZE_RATIO_MAX",
    "PAIR_ROW_TOL_MULT",
    "PAIR_NEIGHBOUR_CLEAR_MULT",
    "LampCandidate",
    "PairWindow",
    "adjacent_ok",
    "flanks_are_clear",
    "find_adjacent_pair",
]

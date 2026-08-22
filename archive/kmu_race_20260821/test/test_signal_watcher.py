"""위치 기반 신호 판정 회귀 — 0817 오출발 사고에서 나온 계약들.

핵심 계약: **색을 쓰지 않고**, '켜져 있던 불빛이 꺼지고 그 오른쪽에 새 불빛이
켜졌다'는 사건으로만 출발을 판정한다. 정지 반사광은 이 사건을 만들 수 없다.
"""

import cv2
import numpy as np

from kmu_race.perception.signal_watcher import (
    LampBlob,
    SignalWatcher,
    find_lamp_blobs,
)

W, H = 640, 480
ROW_Y = 210          # 실측 램프 높이대 (y_full 187~219)
LAMP_D = 8           # 실측 램프 지름
SPACING = 10         # 실측 램프 간격


def frame_with_lamps(xs, y=ROW_Y, d=LAMP_D, bg=120):
    """지정한 x 위치에 점등 램프(흰 원)를 그린 프레임."""
    img = np.full((H, W, 3), bg, np.uint8)
    # 하우징: 램프 왼쪽에 어두운 판 (소등 램프 자리)
    cv2.rectangle(img, (140, y - 14), (360, y + 14), (60, 60, 60), -1)
    for x in xs:
        cv2.circle(img, (int(x), int(y)), d // 2, (250, 250, 250), -1)
    return img


def feed(w, frames, dt=1.0 / 30.0, t0=0.0, stationary=True):
    """(프레임, 반복횟수) 목록을 순서대로 흘려보내고 마지막 결과를 돌려준다."""
    t = t0
    last = None
    for img, n in frames:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        for _ in range(n):
            last = w.update(find_lamp_blobs(img), t, stationary=True, gray=gray)
            t += dt
            if last.go:
                return last, t
    return last, t


# ──────────────────────────────────────────────────────────────────
def test_finds_lit_lamp_without_color():
    """점등 램프를 색 없이(회색조 밝기만으로) 찾는다."""
    blobs = find_lamp_blobs(frame_with_lamps([300]))
    assert any(abs(b.cx - 300) < 4 and abs(b.cy - ROW_Y) < 4 for b in blobs)


def test_ignores_lamps_outside_geometric_band():
    """겐트리 기하 대역 밖(천장 조명·창밖)은 후보에서 제외한다."""
    img = np.full((H, W, 3), 120, np.uint8)
    cv2.circle(img, (300, 40), LAMP_D // 2, (250, 250, 250), -1)    # 천장
    cv2.circle(img, (300, 460), LAMP_D // 2, (250, 250, 250), -1)   # 바닥
    assert find_lamp_blobs(img) == []


def test_light_moving_right_is_go():
    """★ 핵심: 빨강이 꺼지고 오른쪽(초록)이 켜지면 출발."""
    w = SignalWatcher()
    red_x = 280
    green_x = red_x + 3 * SPACING
    res, _ = feed(
        w,
        [
            (frame_with_lamps([red_x]), 110),     # 빨간불 대기 (3.7s ≥ mature 3.0)
            (frame_with_lamps([green_x]), 30),    # 초록 점등 (1.0s ≥ birth 0.5)
        ],
    )
    assert res.go, f"출발 판정 실패: {res.reason}"
    assert res.steps_right in (2, 3)
    assert res.spacing_px > 0


def test_static_reflection_never_fires():
    """정지 반사광은 아무리 오래 봐도 출발을 만들지 않는다."""
    w = SignalWatcher()
    res, _ = feed(w, [(frame_with_lamps([280, 500]), 300)])
    assert not res.go


def test_light_moving_left_is_not_go():
    """왼쪽으로 옮겨간 불빛(초록→빨강)은 출발이 아니다."""
    w = SignalWatcher()
    res, _ = feed(
        w,
        [
            (frame_with_lamps([280 + 3 * SPACING]), 40),
            (frame_with_lamps([280]), 30),
        ],
    )
    assert not res.go


def test_blink_at_same_place_is_not_go():
    """같은 자리에서 껐다 켜는 것(깜빡임)은 출발이 아니다."""
    w = SignalWatcher()
    res, _ = feed(
        w,
        [
            (frame_with_lamps([300]), 40),
            (frame_with_lamps([]), 8),
            (frame_with_lamps([300]), 20),
        ],
    )
    assert not res.go


def test_far_jump_is_not_go():
    """너무 멀리 떨어진 새 불빛은 같은 신호등이 아니다."""
    w = SignalWatcher()
    res, _ = feed(
        w,
        [
            (frame_with_lamps([200]), 40),
            (frame_with_lamps([600]), 30),      # 60px = 7.5 램프폭 밖
        ],
    )
    assert not res.go


def test_green_left_pair_detected():
    """직좌 동시신호 = 같은 줄에 한 칸 간격 두 점 (규정 p.36)."""
    w = SignalWatcher()
    img = frame_with_lamps([300, 300 + SPACING])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = w.update(find_lamp_blobs(img), 0.0, stationary=True, gray=gray)
    assert res.green_left


def test_single_lamp_is_not_green_left():
    w = SignalWatcher()
    img = frame_with_lamps([300])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = w.update(find_lamp_blobs(img), 0.0, stationary=True, gray=gray)
    assert not res.green_left


def test_pair_without_dark_housing_rejected():
    """왼쪽에 소등 램프(어두운 하우징)가 없으면 직좌가 아니다.

    0817: 밝은 배경의 청록 반사 두 점이 직좌로 오검됐던 경로를 막는다.
    """
    img = np.full((H, W, 3), 200, np.uint8)     # 하우징 없음, 배경 밝음
    for x in (300, 300 + SPACING):
        cv2.circle(img, (x, ROW_Y), LAMP_D // 2, (250, 250, 250), -1)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    w = SignalWatcher()
    res = w.update(find_lamp_blobs(img), 0.0, stationary=True, gray=gray)
    assert not res.green_left


def test_moving_disables_anchor_logic():
    """주행 중에는 트랙이 성립하지 않으므로 출발 판정을 하지 않는다."""
    w = SignalWatcher()
    img = frame_with_lamps([280])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    for i in range(60):
        res = w.update(find_lamp_blobs(img), i / 30.0, stationary=False, gray=gray)
    img2 = frame_with_lamps([280 + 3 * SPACING])
    gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
    for i in range(60, 90):
        res = w.update(find_lamp_blobs(img2), i / 30.0, stationary=False, gray=gray2)
    assert not res.go


def test_fires_only_once():
    """한 번 발화하면 같은 대기 구간에서 다시 발화하지 않는다."""
    w = SignalWatcher()
    res, t = feed(
        w,
        [
            (frame_with_lamps([280]), 110),
            (frame_with_lamps([280 + 3 * SPACING]), 30),
        ],
    )
    assert res.go
    img = frame_with_lamps([280 + 3 * SPACING])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    again = w.update(find_lamp_blobs(img), t + 1.0, stationary=True, gray=gray)
    assert not again.go


# ──────────────────────────────────────────────────────────────────
# 램프 피치 기하 — 규정 p.35/p.36 배치에서 나오는 구조 게이트
# (0819: 같은 물리량이 detector/watcher 에 2배 어긋나 박혀 있던 것을
#  perception/lamp_pitch.py 단일 출처로 통합하며 추가한 회귀들)
# ──────────────────────────────────────────────────────────────────
from kmu_race.perception import lamp_pitch  # noqa: E402


def _cand(cx, size=8.0, cy=ROW_Y):
    return lamp_pitch.LampCandidate(cx=cx, cy=cy, size=size, h=size)


def test_measured_pitch_is_inside_the_window():
    """실측 피치비 1.25 (LAMP_D 8 / SPACING 10) 는 창 안에 있어야 한다."""
    win = lamp_pitch.PairWindow()
    a, b = _cand(300.0, LAMP_D), _cand(300.0 + SPACING, LAMP_D)
    assert lamp_pitch.adjacent_ok(a, b, win)


def test_two_pitch_pair_is_rejected():
    """2피치 떨어진 쌍은 직좌가 아니다 (직좌 = 3·4번 인접 한 칸).

    구 창(detector 1.2~4.5)이 통과시키던 바로 그 형태 — 노랑+초록처럼
    한 칸 건너뛴 조합은 규정 점등표에 존재하지 않는다.
    """
    win = lamp_pitch.PairWindow()
    a, b = _cand(300.0, LAMP_D), _cand(300.0 + 2 * SPACING, LAMP_D)
    assert not lamp_pitch.adjacent_ok(a, b, win)


def test_ambiguous_gap_window_is_refused_at_construction():
    """상한이 하한의 2배 이상인 창은 1피치와 2피치를 구분할 수 없다."""
    try:
        lamp_pitch.PairWindow(gap_min_mult=1.2, gap_max_mult=4.5)
    except ValueError:
        return
    raise AssertionError("모호한 창이 그대로 만들어졌다")


def test_row_of_three_lights_is_not_a_pair():
    """같은 간격 점등체 셋은 신호등이 아니다 (책상 위 물건·천장 조명 열).

    직좌는 정확히 두 개만 켜진다 — 4번 오른쪽은 유닛 끝이고 3번 왼쪽의
    노랑은 꺼져 있다. 양옆 여백 게이트가 이 열 전체를 죽인다.
    """
    xs = [300.0, 300.0 + SPACING, 300.0 + 2 * SPACING]
    cands = [_cand(x, LAMP_D) for x in xs]
    assert lamp_pitch.find_adjacent_pair(cands) is None


def test_true_pair_with_empty_flanks_is_found():
    cands = [_cand(300.0, LAMP_D), _cand(300.0 + SPACING, LAMP_D)]
    assert lamp_pitch.find_adjacent_pair(cands) == (0, 1)


def test_lit_object_right_of_green_vetoes_pair():
    """4번(초록) 오른쪽에 점등체가 또 있으면 신호등 하우징이 아니다."""
    cands = [
        _cand(300.0, LAMP_D),
        _cand(300.0 + SPACING, LAMP_D),
        _cand(300.0 + SPACING + int(1.5 * SPACING), LAMP_D),
    ]
    assert lamp_pitch.find_adjacent_pair(cands) is None


def test_watcher_rejects_row_of_three_lamps():
    """워처 경로 회귀 — 세 점 열은 직좌로 승격되지 않는다."""
    w = SignalWatcher()
    img = frame_with_lamps([300, 300 + SPACING, 300 + 2 * SPACING])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = w.update(find_lamp_blobs(img), 0.0, stationary=True, gray=gray)
    assert not res.green_left


def test_watcher_rejects_two_pitch_pair():
    """워처 경로 회귀 — 한 칸 건너뛴 쌍은 직좌가 아니다."""
    w = SignalWatcher()
    img = frame_with_lamps([300, 300 + 2 * SPACING])
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    res = w.update(find_lamp_blobs(img), 0.0, stationary=True, gray=gray)
    assert not res.green_left

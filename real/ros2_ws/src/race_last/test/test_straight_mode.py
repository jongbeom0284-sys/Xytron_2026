"""직진 구간 동작 회귀 — 0823 저녁 요구.

  · 초록 확정 ~ 콘 트리거 = 흰선 중앙만 따라간다 (노란선 무시)
  · 콘 트리거가 울리면 직진 구간 종료 → 노란선 우선 복귀 (오염 정상)
"""
import numpy as np
import pytest

from race_last.lane_core import LaneConfig, LaneTracker


def make_masks(h=480, w=640):
    """노란 중앙선은 왼쪽으로 치우치고, 흰선 두 줄은 화면 중앙을 감싼다.

    노란선을 따라가면 목표가 왼쪽(음수)으로, 흰선 중앙을 따라가면 0 근처로
    나온다 — 두 모드를 오차 부호로 구분할 수 있다.
    """
    lane = np.zeros((h, w), bool)
    lane[280:470, 180:196] = True    # 노란 중앙선 (왼쪽)
    lane[280:470, 250:262] = True    # 좌 흰선
    lane[280:470, 400:412] = True    # 우 흰선
    road = np.zeros((h, w), bool)
    road[270:, :] = True
    bgr = np.zeros((h, w, 3), np.uint8)
    bgr[:, :] = (60, 60, 60)
    bgr[280:470, 180:196] = (30, 190, 210)    # 노랑 (b* 양수)
    bgr[280:470, 250:262] = (235, 235, 235)   # 흰
    bgr[280:470, 400:412] = (235, 235, 235)
    return bgr, lane, road


def run(white_center_only):
    cfg = LaneConfig(white_center_only=white_center_only)
    tr = LaneTracker(cfg)
    bgr, lane, road = make_masks()
    return tr.process(bgr, lane, road, 1.0)


def test_normal_mode_follows_yellow():
    r = run(False)
    assert r.basis == 'yellow', r.basis
    assert r.error_far < -0.3, r.error_far      # 노란선 쪽(왼쪽)으로 끌린다


def test_straight_mode_uses_white_center_only():
    r = run(True)
    assert r.basis == 'white_center', r.basis
    # 흰선 두 줄의 중앙(≈331px) → 화면 중앙(320)과 거의 같다
    assert abs(r.error_far) < 0.1, r.error_far


def test_straight_mode_no_yellow_fallback_when_white_missing():
    """흰선이 없으면 노란선으로 떨어지지 않는다 (그러면 끌림이 되살아난다)."""
    cfg = LaneConfig(white_center_only=True)
    tr = LaneTracker(cfg)
    bgr, lane, road = make_masks()
    lane[:, 240:420] = False           # 흰선 두 줄 제거
    bgr[:, 240:420] = (60, 60, 60)
    r = tr.process(bgr, lane, road, 1.0)
    assert r.basis in ('none', 'hold'), r.basis


def test_avoid_offset_still_wins():
    """앞차 회피(흰선 붙기)는 직진 모드보다 우선한다."""
    cfg = LaneConfig(white_center_only=True, avoid_offset_px=90.0)
    tr = LaneTracker(cfg)
    bgr, lane, road = make_masks()
    r = tr.process(bgr, lane, road, 1.0)
    assert r.basis == 'avoid_right', r.basis

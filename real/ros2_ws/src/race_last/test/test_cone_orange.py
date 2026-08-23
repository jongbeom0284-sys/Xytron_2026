"""라바콘 주황을 노란 씨앗에서 빼는 동작 회귀 (0823 저녁).

근거: 195303/f130 실측 — 콘 화소 14797개 중 차선 마스크와 겹친 50개가
노란 씨앗이 되어 흰선 덩이 442화소를 노랑으로 물들였다.
노란 중앙선(H≈25~35)과 콘 주황(H 3~12)은 색상이 다르므로 갈라낼 수 있다.
실프레임 검증: 직진 구간 7프레임에서 씨앗 손실 0.0%, 콘 구간에서만 0.3~12.7% 제거.
"""
import numpy as np

from race_last.lane_core import LaneConfig, LaneTracker


def scene(h=480, w=640):
    """노란 중앙선 + 흰선 + 흰선 위에 얹힌 라바콘(주황)."""
    lane = np.zeros((h, w), bool)
    lane[300:460, 300:316] = True     # 노란 중앙선
    lane[300:460, 460:474] = True     # 우측 흰선
    lane[330:360, 455:480] = True     # 그 위에 얹힌 콘 (차선 마스크와 겹침)
    road = np.zeros((h, w), bool)
    road[290:, :] = True
    bgr = np.zeros((h, w, 3), np.uint8)
    bgr[:, :] = (70, 70, 70)
    bgr[300:460, 300:316] = (40, 200, 220)    # 노랑  (HSV H≈27)
    bgr[300:460, 460:474] = (240, 240, 240)   # 흰
    bgr[330:360, 455:480] = (30, 110, 240)    # 주황 콘 (HSV H≈8)
    return bgr, lane, road


def seeds(exclude):
    cfg = LaneConfig(exclude_cone_orange=exclude)
    tr = LaneTracker(cfg)
    bgr, lane, road = scene()
    seed, _ = tr._classify_pixels(bgr, lane, road)
    return seed


def test_cone_orange_is_dropped_from_seed():
    on, off = seeds(True), seeds(False)
    cone_box = (slice(330, 360), slice(455, 480))
    assert off[cone_box].sum() > 0, '기존 동작에서는 콘이 씨앗이 된다'
    assert on[cone_box].sum() == 0, '제외를 켜면 콘은 씨앗이 아니다'


def test_yellow_line_survives():
    on, off = seeds(True), seeds(False)
    line = (slice(300, 460), slice(300, 316))
    assert on[line].sum() > 0
    assert on[line].sum() == off[line].sum(), '노란 중앙선은 그대로여야 한다'


def test_white_line_not_turned_yellow():
    """콘이 얹힌 흰선 덩이가 노랑으로 넘어가지 않는다."""
    cfg = LaneConfig(exclude_cone_orange=True)
    tr = LaneTracker(cfg)
    bgr, lane, road = scene()
    r = tr.process(bgr, lane, road, 1.0)
    white_box = (slice(300, 460), slice(455, 480))
    assert r.white_mask[white_box].sum() > 0, '흰선은 흰선으로 남아야 한다'
    assert r.yellow_mask[white_box].sum() == 0, '흰선 자리가 노랑이면 안 된다'


def test_can_be_disabled():
    cfg = LaneConfig(exclude_cone_orange=False)
    assert cfg.exclude_cone_orange is False

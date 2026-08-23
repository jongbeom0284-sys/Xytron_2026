"""노란 덩이가 둘일 때 하나만 따라가는지 (0823 저녁, 좌회전이 끊기던 문제).

실측 근거: 203743/f130~154 좌회전 구간에서 왼쪽 덩이와 오른쪽 덩이를 행별로
섞어 뽑아 e_far 부호가 4번 뒤집혔다. 가장 큰 덩이만 쓰면 0번이 된다.
"""
import numpy as np

from race_last.lane_core import LaneConfig, LaneTracker, largest_blob


def two_yellow(h=480, w=640):
    """왼쪽에 길고 큰 노란 선, 오른쪽 위에 짧은 노란 선.

    선 폭은 실제 차선처럼 좁게 둔다 — max_row_run(80) 을 넘으면
    cut_horizontal_bridges 가 횡단보도로 보고 덩이를 통째로 지운다.
    오른쪽 선을 더 넓게 두어, 두 선이 함께 보이는 행에서 '행마다 가장 넓은
    run' 규칙이 오른쪽을 고르게 한다 (실주행에서 벌어진 상황과 같은 꼴).
    넓이 합은 왼쪽이 크다 (24x180=4320 vs 40x62=2480).
    """
    lane = np.zeros((h, w), bool)
    lane[290:470, 150:174] = True     # A: 왼쪽, 좁고 아래까지 길다 (큰 덩이)
    lane[290:352, 520:560] = True     # B: 오른쪽, 넓고 위쪽에만 (작은 덩이)
    road = np.zeros((h, w), bool)
    road[285:, :] = True
    bgr = np.zeros((h, w, 3), np.uint8)
    bgr[:, :] = (70, 70, 70)
    bgr[290:470, 150:174] = (40, 200, 220)    # 노랑
    bgr[290:352, 520:560] = (40, 200, 220)    # 노랑
    return bgr, lane, road


def run(single):
    cfg = LaneConfig(single_yellow_blob=single)
    tr = LaneTracker(cfg)
    bgr, lane, road = two_yellow()
    return tr.process(bgr, lane, road, 1.0)


def test_largest_blob_keeps_only_biggest():
    m = np.zeros((100, 100), bool)
    m[10:40, 10:40] = True      # 900
    m[60:70, 60:70] = True      # 100
    out = largest_blob(m)
    assert out[10:40, 10:40].all()
    assert not out[60:70, 60:70].any()


def test_largest_blob_passthrough_when_single():
    m = np.zeros((50, 50), bool)
    m[10:20, 10:20] = True
    assert largest_blob(m).sum() == m.sum()


def test_single_blob_follows_the_left_line():
    """가장 큰(왼쪽) 덩이만 따라가므로 목표가 왼쪽에 유지된다."""
    r = run(True)
    assert r.basis == 'yellow', r.basis
    assert r.error_far < -0.2, r.error_far      # 왼쪽


def test_mixed_blobs_pull_target_right():
    """끄면 오른쪽 덩이가 먼 행을 차지해 목표가 오른쪽으로 끌린다.

    실주행 203743/f139 에서 이 섞임이 e_far 를 -0.24 에서 +0.04 로 뒤집었다.
    """
    on, off = run(True), run(False)
    assert off.error_far > on.error_far + 0.05, (off.error_far, on.error_far)


def test_can_be_disabled():
    assert LaneConfig(single_yellow_blob=False).single_yellow_blob is False

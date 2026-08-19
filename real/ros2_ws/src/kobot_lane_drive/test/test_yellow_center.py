"""노란 중앙선 검출과 점선 외삽 (주행 우선순위 1단계)."""

import numpy as np
import pytest

from kobot_lane_drive.twinmixing_center import TwinMixingCenterConfig
from kobot_lane_drive.yellow_center import (
    GHOST_SOURCE,
    YELLOW_SOURCE,
    YellowCenterConfig,
    YellowCenterTracker,
    extract_yellow_center,
    yellow_mask,
)

WIDTH = 640
HEIGHT = 480
CENTER_CONFIG = TwinMixingCenterConfig(minimum_points=12)


def _scene(center_shift=0, dashed=False, paint_yellow=True):
    """노란 중앙선으로 갈라진 2차선 도로를 합성한다."""
    frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
    driving = np.zeros((HEIGHT, WIDTH), dtype=bool)
    lane = np.zeros_like(driving)
    for y in range(int(HEIGHT * 0.45), int(HEIGHT * 0.98)):
        progress = (y - HEIGHT * 0.45) / (HEIGHT * 0.53)
        center = int(WIDTH * 0.5 + center_shift * progress)
        half_gap = max(3, int(4 + 10 * progress))
        outer = int(WIDTH * (0.08 + 0.30 * (1.0 - progress)))
        driving[y, outer : center - half_gap] = True
        driving[y, center + half_gap : WIDTH - outer] = True
        lane[y, center - 2 : center + 3] = True
        if dashed and (y // 18) % 2 == 0:
            continue
        if paint_yellow:
            frame[y, center - half_gap : center + half_gap + 1] = (0, 255, 255)
    return frame, driving, lane


def test_yellow_mask_selects_painted_line_only():
    frame, _, _ = _scene()
    mask = yellow_mask(frame, YellowCenterConfig())
    assert mask.any()
    # 배경 회색은 걸리지 않는다
    assert not mask[0, 0]


def test_follows_yellow_line_when_painted():
    frame, driving, lane = _scene(center_shift=70)
    result, evidence = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert result.found
    assert result.path_source == YELLOW_SOURCE
    assert evidence.any()
    assert result.center_offset_px is not None
    assert result.center_offset_px > 15


def test_dashed_line_still_yields_a_path():
    frame, driving, lane = _scene(dashed=True)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert result.found
    assert result.path_source == YELLOW_SOURCE


def test_reports_lost_without_yellow_paint():
    frame, driving, lane = _scene(paint_yellow=False)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert not result.found


def test_tracker_extrapolates_across_a_gap_then_gives_up():
    frame, driving, lane = _scene(center_shift=40)
    valid, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert valid.found

    config = YellowCenterConfig(ghost_maximum_frames=5)
    tracker = YellowCenterTracker(config)
    for _ in range(3):
        assert tracker.update(valid, WIDTH).path_source == YELLOW_SOURCE

    blank_frame, blank_driving, blank_lane = _scene(paint_yellow=False)
    lost, _ = extract_yellow_center(
        blank_frame, blank_driving, blank_lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert not lost.found

    ghosts = [tracker.update(lost, WIDTH) for _ in range(config.ghost_maximum_frames)]
    assert all(item.path_source == GHOST_SOURCE for item in ghosts)
    assert ghosts[-1].confidence < ghosts[0].confidence

    given_up = tracker.update(lost, WIDTH)
    assert not given_up.found
    assert tracker.blind_frames == 0


@pytest.mark.parametrize("shift", [-80, 0, 80])
def test_offset_sign_follows_the_line(shift):
    frame, driving, lane = _scene(center_shift=shift)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(), CENTER_CONFIG
    )
    assert result.found
    if shift == 0:
        assert abs(result.center_offset_px) < 20
    else:
        assert np.sign(result.center_offset_px) == np.sign(shift)

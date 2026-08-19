"""노란 중앙선 검출과 점선 외삽 (주행 우선순위 1단계)."""

import numpy as np
import pytest

from kobot_lane_drive.twinmixing_center import (
    TwinMixingCenterConfig,
    extract_center_path,
)
from kobot_lane_drive.yellow_center import (
    GHOST_SOURCE,
    HSV_ABSOLUTE,
    LAB_RELATIVE,
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
    frame, driving, _ = _scene()
    mask = yellow_mask(frame, driving, YellowCenterConfig(detector=HSV_ABSOLUTE))
    assert mask.any()
    # ROI 밖 배경은 걸리지 않는다
    assert not mask[0, 0]


def test_follows_yellow_line_when_painted():
    frame, driving, lane = _scene(center_shift=70)
    result, evidence = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
    )
    assert result.found
    assert result.path_source == YELLOW_SOURCE
    assert evidence.any()
    assert result.center_offset_px is not None
    assert result.center_offset_px > 15


def test_dashed_line_still_yields_a_path():
    frame, driving, lane = _scene(dashed=True)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
    )
    assert result.found
    assert result.path_source == YELLOW_SOURCE


def test_reports_lost_without_yellow_paint():
    frame, driving, lane = _scene(paint_yellow=False)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
    )
    assert not result.found


def test_tracker_extrapolates_across_a_gap_then_gives_up():
    frame, driving, lane = _scene(center_shift=40)
    valid, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
    )
    assert valid.found

    config = YellowCenterConfig(detector=HSV_ABSOLUTE, ghost_maximum_frames=5)
    tracker = YellowCenterTracker(config)
    for _ in range(3):
        assert tracker.update(valid, WIDTH).path_source == YELLOW_SOURCE

    blank_frame, blank_driving, blank_lane = _scene(paint_yellow=False)
    lost, _ = extract_yellow_center(
        blank_frame, blank_driving, blank_lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
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
        frame, driving, lane, YellowCenterConfig(detector=HSV_ABSOLUTE), CENTER_CONFIG
    )
    assert result.found
    if shift == 0:
        assert abs(result.center_offset_px) < 20
    else:
        assert np.sign(result.center_offset_px) == np.sign(shift)


def test_candidate_must_be_anchored_to_the_road():
    """중앙선은 도로 위에 있거나 두 주행영역 사이 틈에 있다.

    색만으로는 벽의 나무 슬랫을 못 거른다. 세로 줄무늬라 top-hat 이 찾는
    "좁은 노란 줄"과 똑같이 생겼기 때문이다. 2026-08-19 12:56 주행 214~219 에서
    목표선이 도로가 아니라 오른쪽 나무 기둥 위에 얹혔다.

    구분 기준은 색이 아니라 위치다.
    """

    def scene(where):
        frame = np.full((HEIGHT, WIDTH, 3), 60, dtype=np.uint8)
        driving = np.zeros((HEIGHT, WIDTH), dtype=bool)
        lane = np.zeros_like(driving)
        for y in range(int(HEIGHT * 0.45), int(HEIGHT * 0.98)):
            if where == "off_road":
                driving[y, 180:420] = True          # 도로는 왼쪽에만
                x0, x1 = 470, 495                   # 노란 줄은 도로 밖 (나무 기둥)
            elif where == "in_gap":
                driving[y, 180:305] = True          # 도로 두 덩이
                driving[y, 335:460] = True
                x0, x1 = 312, 330                   # 그 사이 틈
            else:  # on_road
                driving[y, 180:460] = True          # 하나로 이어진 도로
                x0, x1 = 305, 330                   # 그 위에 그려진 선
            frame[y, x0:x1] = (0, 255, 255)
            lane[y, x0:x1] = True
        return frame, driving, lane

    anchored = YellowCenterConfig(detector=HSV_ABSOLUTE)
    assert anchored.require_road_anchor

    # 도로 밖의 노란 구조물은 걸러진다
    assert not extract_yellow_center(*scene("off_road"), anchored, CENTER_CONFIG)[0].found

    # 도로 사이 틈의 중앙선은 잡는다
    gap = extract_yellow_center(*scene("in_gap"), anchored, CENTER_CONFIG)[0]
    assert gap.found and gap.path_source == YELLOW_SOURCE

    # 도로 위에 그려진 중앙선도 잡는다
    on_road = extract_yellow_center(*scene("on_road"), anchored, CENTER_CONFIG)[0]
    assert on_road.found and on_road.path_source == YELLOW_SOURCE

    # 조건을 끄면 도로 밖 구조물도 통과한다 — 걸러낸 주체가 이 조건임을 확인한다
    relaxed = YellowCenterConfig(detector=HSV_ABSOLUTE, require_road_anchor=False)
    assert extract_yellow_center(*scene("off_road"), relaxed, CENTER_CONFIG)[0].found


def _wood_scene(paint_line=True):
    """누런 나무 바닥 위 도로에 노란 중앙선이 있는 장면.

    실차와 같은 구도다. 나무는 노란선과 색상(H) 축에서 겹치므로 절대 임계로는
    가를 수 없고, 노면 대비로만 갈린다.
    """
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    driving = np.zeros((HEIGHT, WIDTH), dtype=bool)
    lane = np.zeros_like(driving)
    frame[:, :] = (90, 120, 150)                  # 누런 나무 바닥 (BGR)
    for y in range(int(HEIGHT * 0.45), int(HEIGHT * 0.98)):
        frame[y, 180:460] = (140, 142, 138)       # 회색 노면
        driving[y, 180:312] = True
        driving[y, 328:460] = True
        if paint_line:
            frame[y, 313:327] = (40, 210, 235)    # 노란 중앙선
            lane[y, 313:327] = True
    return frame, driving, lane


def test_lab_relative_ignores_the_wooden_floor():
    """나무 바닥은 그 행 전체가 누렇다. 노면 대비가 없으므로 걸리지 않아야 한다."""
    frame, driving, lane = _wood_scene()
    config = YellowCenterConfig(detector=LAB_RELATIVE)
    mask = yellow_mask(frame, driving, config)

    row = int(HEIGHT * 0.8)
    assert mask[row, 313:327].any(), "중앙선을 놓쳤다"
    assert not mask[row, :100].any(), "도로 왼쪽 나무 바닥이 걸렸다"
    assert not mask[row, 540:].any(), "도로 오른쪽 나무 바닥이 걸렸다"


def test_hsv_absolute_cannot_separate_the_wooden_floor():
    """같은 장면에서 예전 방식은 나무 바닥까지 잡는다. LAB 을 쓰는 이유다."""
    frame, driving, _ = _wood_scene()
    hsv_mask = yellow_mask(frame, driving, YellowCenterConfig(detector=HSV_ABSOLUTE))
    row = int(HEIGHT * 0.8)
    assert hsv_mask[row, :100].any()


def test_lab_relative_follows_the_line_in_the_wood_scene():
    frame, driving, lane = _wood_scene()
    result, evidence = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=LAB_RELATIVE), CENTER_CONFIG
    )
    assert result.found
    assert result.path_source == YELLOW_SOURCE
    assert abs(result.center_offset_px) < 20
    # 근거는 ROI 안에만 남는다
    assert not evidence[: int(HEIGHT * 0.5)].any()


def test_lab_relative_reports_lost_without_a_line():
    frame, driving, lane = _wood_scene(paint_line=False)
    result, _ = extract_yellow_center(
        frame, driving, lane, YellowCenterConfig(detector=LAB_RELATIVE), CENTER_CONFIG
    )
    assert not result.found


def test_ghost_never_outranks_a_real_road_measurement():
    """외삽본은 실측이 없을 때만 쓴다.

    12:23 주행 231 번에서 인식 구간의 97% 가 어두워 노란선을 못 봤는데도, 도로가
    보이는 상태에서 3 프레임 전 잔상을 계속 따라갔다. 외삽 신뢰도가 20 프레임에
    걸쳐 떨어지므로 최대 1.3 초까지 실측 도로를 무시했다.

    노드의 선택 로직을 그대로 흉내 내 순서를 검증한다.
    """
    frame, driving, lane = _scene(center_shift=40)
    config = YellowCenterConfig(detector=HSV_ABSOLUTE)
    valid, _ = extract_yellow_center(frame, driving, lane, config, CENTER_CONFIG)
    assert valid.found

    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH)

    blank_frame, blank_driving, blank_lane = _scene(paint_yellow=False)
    lost, _ = extract_yellow_center(
        blank_frame, blank_driving, blank_lane, config, CENTER_CONFIG
    )
    ghost = tracker.update(lost, WIDTH)
    assert ghost.path_source == GHOST_SOURCE
    assert ghost.confidence >= 0.30, "외삽본이 임계 위에 있어야 이 상황이 재현된다"

    # 노드와 같은 순서: 실측 노란선이 아니면 도로를 먼저 본다
    road = extract_center_path(blank_driving, blank_lane, CENTER_CONFIG)
    assert road.found, "이 장면에서는 도로가 보인다"

    chosen = ghost if ghost.path_source == YELLOW_SOURCE else road
    assert chosen.path_source != GHOST_SOURCE
    assert chosen.path_source in ("CENTER_GAP", "REGION_CENTER")

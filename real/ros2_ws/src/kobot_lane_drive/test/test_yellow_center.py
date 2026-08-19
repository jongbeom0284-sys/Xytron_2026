"""노란 중앙선 검출과 점선 외삽 (주행 우선순위 1단계)."""

from dataclasses import replace

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
    # 유지 중에는 신뢰도를 깎지 않는다. 깎으면 호출자 임계에 걸려 한도 전에 끊긴다.
    assert all(item.confidence == valid.confidence for item in ghosts)

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

    # 기본은 꺼짐. 216 번의 나무 기둥은 막았지만 1 번에서 진짜 중앙선까지 걷어냈다.
    # 중앙선이 주행영역 가장자리 바로 바깥에 놓이면 나무와 기하학적으로 구분되지 않는다.
    assert not YellowCenterConfig().require_road_anchor
    anchored = YellowCenterConfig(detector=HSV_ABSOLUTE, require_road_anchor=True)

    # 도로 밖의 노란 구조물은 걸러진다
    assert not extract_yellow_center(*scene("off_road"), anchored, CENTER_CONFIG)[0].found

    # 도로 사이 틈의 중앙선은 잡는다
    gap = extract_yellow_center(*scene("in_gap"), anchored, CENTER_CONFIG)[0]
    assert gap.found and gap.path_source == YELLOW_SOURCE

    # 도로 위에 그려진 중앙선도 잡는다
    on_road = extract_yellow_center(*scene("on_road"), anchored, CENTER_CONFIG)[0]
    assert on_road.found and on_road.path_source == YELLOW_SOURCE

    # 이제는 거리 조건이 1차 방어선이다. 도로에서 40px 넘게 떨어진 구조물은
    # require_road_anchor 를 꺼도 걸러진다. 실측에서 나무 슬랫이 107px 였다.
    default = YellowCenterConfig(detector=HSV_ABSOLUTE)
    assert not default.require_road_anchor
    assert not extract_yellow_center(*scene("off_road"), default, CENTER_CONFIG)[0].found

    # 거리 조건까지 풀면 통과한다 — 걸러낸 주체가 그 조건임을 확인한다
    relaxed = YellowCenterConfig(
        detector=HSV_ABSOLUTE, maximum_road_distance_ratio=1.0
    )
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


ROAD_SOURCES = ("CENTER_GAP", "REGION_CENTER")


def _select_like_the_node(tracker, yellow_result, now, driving, lane):
    """twinmixing_center_node._select_target_line 의 순서를 그대로 흉내 낸다.

    노드는 ROS 노드라 시험에서 직접 못 부른다. 순서가 어긋나면 여기서 잡는다.
    신뢰도 판정은 트래커 안에서 끝난다 — 노드도 여기도 다시 거르지 않는다.
    """
    tracked = tracker.update(yellow_result, WIDTH, now)
    if tracked.found:
        return tracked
    return extract_center_path(driving, lane, CENTER_CONFIG)


def _hold_fixture(ghost_maximum_seconds=1.0):
    """노란선이 보이는 장면 하나와, 노란선만 지운 장면 하나를 만든다."""
    config = YellowCenterConfig(
        detector=HSV_ABSOLUTE, ghost_maximum_seconds=ghost_maximum_seconds
    )
    frame, driving, lane = _scene(center_shift=40)
    valid, _ = extract_yellow_center(frame, driving, lane, config, CENTER_CONFIG)
    assert valid.found
    assert valid.confidence >= config.minimum_confidence, "장면이 채택 임계를 넘어야 한다"

    blank_frame, blank_driving, blank_lane = _scene(paint_yellow=False)
    lost, _ = extract_yellow_center(
        blank_frame, blank_driving, blank_lane, config, CENTER_CONFIG
    )
    assert not lost.found
    road = extract_center_path(blank_driving, blank_lane, CENTER_CONFIG)
    assert road.found, "이 장면에서는 도로가 보여야 대비가 성립한다"
    return config, valid, lost, blank_driving, blank_lane


def test_hold_outranks_the_road_while_the_window_is_open():
    """노란선이 끊긴 직후에는 도로가 보여도 직전 노란선을 유지한다.

    실측 주행에서 노란선이 사라지는 프레임은 주행영역 인식도 함께 무너져 있었다.
    그 순간의 도로 중앙은 노란선의 대안이 아니라 잡음이다.
    """
    config, valid, lost, driving, lane = _hold_fixture()
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)

    chosen = _select_like_the_node(tracker, lost, 10.4, driving, lane)
    assert chosen.path_source == GHOST_SOURCE
    assert chosen.found


def test_road_takes_over_once_the_hold_window_expires():
    """1 초를 넘기면 유지를 포기하고 도로로 내려간다."""
    config, valid, lost, driving, lane = _hold_fixture()
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)

    assert _select_like_the_node(tracker, lost, 10.9, driving, lane).path_source == GHOST_SOURCE
    chosen = _select_like_the_node(tracker, lost, 11.05, driving, lane)
    assert chosen.path_source in ROAD_SOURCES
    assert tracker.blind_frames == 0, "포기했으면 트래커가 비워져 있어야 한다"


def test_hold_window_is_measured_in_seconds_not_frames():
    """추론 주기가 9~15Hz 사이에서 움직여 프레임 수로는 한도가 흔들렸다.

    0.9 초 안에 24 프레임이 들어와도 유지가 유지돼야 한다. 프레임 한도(20)로
    재던 예전 코드는 21 번째에서 끊겼다.
    """
    config, valid, lost, driving, lane = _hold_fixture()
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)

    sources = [
        _select_like_the_node(tracker, lost, 10.0 + 0.9 * (step + 1) / 24.0, driving, lane).path_source
        for step in range(24)
    ]
    assert sources == [GHOST_SOURCE] * 24


def test_hold_keeps_the_last_confidence_so_the_node_gate_stays_open():
    """유지본의 신뢰도를 깎으면 노드의 minimum_confidence 에 걸려 한도 전에 끊긴다."""
    config, valid, lost, driving, lane = _hold_fixture()
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)

    for step in range(8):
        held = _select_like_the_node(tracker, lost, 10.0 + 0.1 * (step + 1), driving, lane)
        assert held.path_source == GHOST_SOURCE
        assert held.confidence == valid.confidence


def test_a_weak_detection_does_not_reset_the_hold():
    """주행에 못 쓸 검출은 못 본 것으로 친다.

    임계 미만 검출이 타이머를 초기화하면 (1) 그 프레임은 노란선도 유지본도 아니라
    한 프레임 도로로 튀고, (2) 흐릿한 선이 좋았던 기준선을 밀어내며, (3) 1초가
    영원히 안 찬다. 셋 다 막는다.
    """
    config, valid, lost, driving, lane = _hold_fixture()
    weak = replace(valid, confidence=config.minimum_confidence - 0.05)
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)

    # 0.5초 뒤 약한 검출 — 유지가 끊기지 않고(도로로 안 튀고) 계속 유지본이어야 한다
    held = _select_like_the_node(tracker, weak, 10.5, driving, lane)
    assert held.path_source == GHOST_SOURCE, "약한 검출은 유지를 끊으면 안 된다"
    assert held.confidence == valid.confidence, "기준선은 원래 검출 것 그대로여야 한다"
    assert tracker.blind_seconds == pytest.approx(0.5), "타이머가 초기화되면 안 된다"

    # 원래 시각 기준 1초가 차면 도로로 — 약한 검출이 창을 늘려 주지 않는다
    assert _select_like_the_node(tracker, lost, 10.95, driving, lane).path_source == GHOST_SOURCE
    assert _select_like_the_node(tracker, weak, 11.05, driving, lane).path_source in ROAD_SOURCES


def test_a_weak_detection_alone_is_not_a_line():
    """직전 실측이 없으면 약한 검출은 그냥 없는 것이다 — 도로로 간다."""
    config, valid, lost, driving, lane = _hold_fixture()
    weak = replace(valid, confidence=config.minimum_confidence - 0.05)
    tracker = YellowCenterTracker(config)

    chosen = _select_like_the_node(tracker, weak, 10.0, driving, lane)
    assert chosen.path_source in ROAD_SOURCES
    assert tracker.blind_frames == 0


def test_a_new_yellow_detection_reopens_the_full_window():
    """다시 보이면 유지 시간이 처음부터 다시 시작한다."""
    config, valid, lost, driving, lane = _hold_fixture()
    tracker = YellowCenterTracker(config)
    tracker.update(valid, WIDTH, 10.0)
    assert _select_like_the_node(tracker, lost, 10.9, driving, lane).path_source == GHOST_SOURCE

    assert tracker.update(valid, WIDTH, 11.0).path_source == YELLOW_SOURCE
    assert tracker.blind_seconds == 0.0
    assert _select_like_the_node(tracker, lost, 11.9, driving, lane).path_source == GHOST_SOURCE

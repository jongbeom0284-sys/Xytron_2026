"""노란 중앙선을 주행 목표선으로 뽑는다 — 주행 우선순위 1단계.

TwinMixing 은 주행 가능 영역(driving_mask)과 차선 픽셀(lane_mask)을 낸다. 하지만
lane_mask 는 색을 구분하지 않고, driving_mask 의 틈은 그림자·얼룩·장애물로도 생긴다.
여기서 원본 프레임의 색을 직접 확인해 "그 틈이 정말 노란 중앙선인가"를 판정한다.

중앙선이 점선이면 행 사이가 비므로, 검출이 끊긴 동안 최근 추세를 연장하는 외삽을
둔다. legacy/vehicle-nodes/Lane_follow.py 가 쓰던 GHOST/PREDICT 와 같은 발상이다.
이 외삽이 없으면 점선 구간마다 1단계가 2단계로 떨어져 목표선이 계속 흔들린다.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

import cv2
import numpy as np

from .twinmixing_center import (
    CenterPathResult,
    TwinMixingCenterConfig,
    _fit_path,
    _true_runs,
)

YELLOW_SOURCE = "YELLOW_LINE"
GHOST_SOURCE = "YELLOW_LINE_GHOST"


@dataclass(frozen=True)
class YellowCenterConfig:
    # HSV 임계. legacy Lane_follow.py 의 실측 프리셋(H 25-40 / S 61-120 / V 190-255)에서
    # 출발하되 조명 변화를 견디도록 넓혔다. 현장에서는 노출과 함께 다시 맞춘다.
    hue_low: int = 15
    hue_high: int = 45
    saturation_low: int = 55
    saturation_high: int = 255
    value_low: int = 110
    value_high: int = 255

    roi_top_ratio: float = 0.46
    roi_bottom_ratio: float = 0.97
    row_step: int = 4

    # lane_mask 를 살짝 부풀려 색 마스크와 교차시킨다. 세그멘테이션 경계가 한두 픽셀
    # 어긋나도 노란 픽셀을 놓치지 않기 위해서다.
    lane_dilate_px: int = 5
    open_kernel_px: int = 3

    search_half_width_ratio: float = 0.34
    maximum_run_width_ratio: float = 0.22
    minimum_run_pixels: int = 3
    minimum_rows: int = 8

    # 후보 좌우에 주행 가능 영역을 요구하는 선택 조건. 노란 구조물을 배제할 수는
    # 있지만, 한쪽 도로만 보이는 프레임에서도 중앙선은 따라가야 하므로 기본은 끈다.
    # 실측(20260818_204211_264): 켜면 YELLOW_LINE 118 -> 42 로 떨어졌다.
    require_road_on_both_sides: bool = False
    side_support_ratio: float = 0.02
    side_search_ratio: float = 0.10

    ghost_maximum_frames: int = 20
    history_size: int = 10
    ghost_confidence_floor: float = 0.20


def yellow_mask(frame_bgr: np.ndarray, config: YellowCenterConfig) -> np.ndarray:
    """프레임에서 노란 픽셀만 남긴 bool 마스크."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([config.hue_low, config.saturation_low, config.value_low], dtype=np.uint8),
        np.array([config.hue_high, config.saturation_high, config.value_high], dtype=np.uint8),
    )
    if config.open_kernel_px > 1:
        kernel = np.ones((config.open_kernel_px, config.open_kernel_px), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask.astype(bool)


def yellow_evidence(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    lane_mask: np.ndarray | None,
    config: YellowCenterConfig,
) -> np.ndarray:
    """중앙선 후보 픽셀.

    1순위 근거는 모델이 차선이라고 한 픽셀 중 실제로 노란 것,
    보조 근거는 주행 영역 바깥(=영역 사이 틈)의 노란 픽셀이다.
    """
    yellow = yellow_mask(frame_bgr, config)
    driving = driving_mask.astype(bool)
    evidence = yellow & ~driving
    if lane_mask is not None and np.any(lane_mask):
        lane = lane_mask.astype(bool).astype(np.uint8)
        if config.lane_dilate_px > 1:
            kernel = np.ones((config.lane_dilate_px, config.lane_dilate_px), np.uint8)
            lane = cv2.dilate(lane, kernel)
        evidence = evidence | (yellow & lane.astype(bool))
    return evidence


def _has_road_on_both_sides(
    driving_row: np.ndarray,
    start: int,
    end: int,
    support_px: int,
    search_px: int,
) -> bool:
    """후보 양옆에 주행 가능 영역이 충분히 있는가.

    노란 중앙선은 도로 한가운데 있으므로 좌우 모두 도로다. 벽면의 노란 표지나
    배리어는 한쪽이 도로가 아니므로 여기서 걸러진다.
    """
    width = driving_row.size
    left_from = max(0, start - search_px)
    right_to = min(width, end + 1 + search_px)
    left_road = int(driving_row[left_from:start].sum())
    right_road = int(driving_row[end + 1 : right_to].sum())
    return left_road >= support_px and right_road >= support_px


def extract_yellow_center(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    lane_mask: np.ndarray | None,
    config: YellowCenterConfig | None = None,
    center_config: TwinMixingCenterConfig | None = None,
) -> tuple[CenterPathResult, np.ndarray]:
    """노란 중앙선 경로와 근거 마스크를 돌려준다.

    실패하면 state="LOST" 인 결과를 준다. 호출자는 그때 센터 갭으로 내려간다.
    """
    config = config or YellowCenterConfig()
    center_config = center_config or TwinMixingCenterConfig()
    height, width = driving_mask.shape[:2]
    evidence = yellow_evidence(frame_bgr, driving_mask, lane_mask, config)

    y_top = int(round(height * config.roi_top_ratio))
    y_bottom = min(height - 1, int(round(height * config.roi_bottom_ratio)))
    # ROI 밖의 노란 픽셀은 쓰지 않으므로 근거 마스크에서도 지운다.
    # 화면에 남겨 두면 벽면 반사가 검출된 것처럼 보여 오해를 부른다.
    evidence[:y_top] = False
    evidence[y_bottom + 1:] = False
    rows = range(y_bottom, y_top - 1, -max(1, config.row_step))
    attempted_rows = len(rows)
    search_half = max(12, int(round(width * config.search_half_width_ratio)))
    maximum_run = max(4, int(round(width * config.maximum_run_width_ratio)))

    side_support = max(3, int(round(width * config.side_support_ratio)))
    side_search = max(side_support * 2, int(round(width * config.side_search_ratio)))
    driving = driving_mask.astype(bool)

    expected_x = width * 0.5
    samples: list[tuple[float, float]] = []
    widths: list[float] = []
    for y in rows:
        best: tuple[float, float, float] | None = None
        for start, end in _true_runs(evidence[y]):
            run_width = end - start + 1
            if run_width < config.minimum_run_pixels or run_width > maximum_run:
                continue
            center = (start + end) * 0.5
            distance = abs(center - expected_x)
            if distance > search_half:
                continue
            if config.require_road_on_both_sides and not _has_road_on_both_sides(
                driving[y], start, end, side_support, side_search
            ):
                continue
            if best is None or distance < best[0]:
                best = (distance, center, float(run_width))
        if best is None:
            continue
        _, center, run_width = best
        samples.append((center, float(y)))
        widths.append(run_width)
        expected_x = 0.72 * expected_x + 0.28 * center

    if len(samples) < config.minimum_rows:
        return (
            _fit_path([], [], 0, attempted_rows, width, height, center_config, YELLOW_SOURCE),
            evidence,
        )
    result = _fit_path(
        samples,
        widths,
        len(samples),
        attempted_rows,
        width,
        height,
        center_config,
        YELLOW_SOURCE,
    )
    return result, evidence


class YellowCenterTracker:
    """점선 구간에서 노란 중앙선 목표를 유지하는 외삽기.

    최근 lookahead x 를 기억해 두고, 검출이 끊기면 추세를 연장한다. 연속 실패가
    ghost_maximum_frames 를 넘으면 포기하고 호출자가 다음 우선순위로 내려가게 한다.
    """

    def __init__(self, config: YellowCenterConfig | None = None) -> None:
        self._config = config or YellowCenterConfig()
        self._history: list[float] = []
        self._blind_frames = 0
        self._last_valid: CenterPathResult | None = None

    @property
    def blind_frames(self) -> int:
        return self._blind_frames

    def reset(self) -> None:
        self._history.clear()
        self._blind_frames = 0
        self._last_valid = None

    def update(self, result: CenterPathResult, width: int) -> CenterPathResult:
        if result.found and result.lookahead is not None:
            self._history.append(float(result.lookahead[0]))
            if len(self._history) > self._config.history_size:
                self._history.pop(0)
            self._blind_frames = 0
            self._last_valid = result
            return result
        return self._ghost(width)

    def _ghost(self, width: int) -> CenterPathResult:
        if self._last_valid is None or self._last_valid.lookahead is None:
            return self._last_valid if self._last_valid is not None else _lost_result()
        if self._blind_frames >= self._config.ghost_maximum_frames:
            self.reset()
            return _lost_result()

        self._blind_frames += 1
        step = 0.0
        if len(self._history) >= 3:
            recent = self._history[-min(5, len(self._history)):]
            step = (recent[-1] - recent[0]) / max(1, len(recent) - 1)

        previous = self._last_valid
        lookahead_y = previous.lookahead[1]
        lookahead_x = int(
            round(max(0.0, min(width - 1.0, self._history[-1] + step * self._blind_frames)))
        )
        decay = 1.0 - self._blind_frames / float(self._config.ghost_maximum_frames)
        confidence = max(
            self._config.ghost_confidence_floor, previous.confidence * max(0.0, decay)
        )
        anchor_x = width * 0.5
        return replace(
            previous,
            path_source=GHOST_SOURCE,
            confidence=confidence,
            lookahead=(lookahead_x, lookahead_y),
            center_offset_px=float(lookahead_x - anchor_x),
        )


def _lost_result() -> CenterPathResult:
    empty = np.empty((0, 2), dtype=np.int32)
    return CenterPathResult(
        state="LOST",
        path_source="NONE",
        confidence=0.0,
        raw_points=empty,
        fitted_points=empty,
        planned_points=empty,
        lookahead=None,
        center_offset_px=None,
        heading_deg=None,
        mean_gap_width_px=None,
        lane_support_ratio=0.0,
    )

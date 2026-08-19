"""노란 중앙선을 주행 목표선으로 뽑는다 — 주행 우선순위 1단계.

TwinMixing 은 주행 가능 영역(driving_mask)과 차선 픽셀(lane_mask)을 낸다. 하지만
lane_mask 는 색을 구분하지 않고(양쪽 흰 차선도 함께 잡는다), driving_mask 의 틈은
그림자·얼룩·장애물로도 생긴다. 여기서 원본 프레임의 색을 직접 확인한다.

검출 방식은 두 가지고 기본은 lab_relative 다.

lab_relative
    LAB 의 b* 채널은 파랑(-)과 노랑(+)을 잇는 한 축이다. 여기에 가로 방향 top-hat 을
    걸어 '주변보다 국소적으로 더 노란 좁은 줄무늬'만 남긴다.

    절대 임계로는 못 가른다. 나무는 갈색 = 어두운 주황이라 색상 축에서 노랑과 겹치고,
    2026-08-19 실측에서는 노란 점선의 V 가 108, 나무 바닥이 127 로 선이 오히려 더
    어두웠다. 도로 픽셀 기준의 상대 판정도 안 된다. 도로 밖 나무 바닥은 회색 도로보다
    노라므로 그대로 통과한다.

    실제로 가르는 것은 '넓고 균일한 누런 면'과 '좁은 노란 줄무늬'의 차이다. 가로로 긴
    구조요소로 opening 하면 좁은 줄무늬만 지워지고 넓은 면은 남는다. 원본에서 빼면
    줄무늬만 남고 나무 바닥·벽은 0 에 가깝게 사라진다. 조명이 바뀌거나 자동
    화이트밸런스가 장면 전체에 노란 캐스트를 씌워도 국소 차이는 유지된다.

hsv_absolute
    고정된 HSV 상하한으로 자르는 예전 방식. 비교용으로 남겨 둔다. 실내에서는
    갈색(=어두운 주황)이 노랑과 색상 축에서 겹쳐 나무 벽·바닥이 함께 걸린다.
    2026-08-19 실측에서 노란 점선의 V 가 108, 나무 바닥이 127 로 선이 오히려
    더 어두워, 절대 임계로는 원리적으로 가를 수 없었다.

중앙선이 점선이면 행 사이가 비므로, 검출이 끊긴 동안 최근 추세를 연장하는 외삽을
둔다. legacy/vehicle-nodes/Lane_follow.py 가 쓰던 GHOST/PREDICT 와 같은 발상이다.
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

LAB_RELATIVE = "lab_relative"
HSV_ABSOLUTE = "hsv_absolute"


@dataclass(frozen=True)
class YellowCenterConfig:
    detector: str = LAB_RELATIVE

    # --- lab_relative ---
    # OpenCV LAB 에서 b* 는 0~255 이고 128 이 중립이다.
    lab_margin: float = 8.0        # 주변 대비 최소 초과량
    lab_minimum_b: float = 130.0   # 노랑쪽이라는 약한 절대 조건
    # 중앙선 폭보다 넉넉히 넓어야 한다. 이보다 좁은 줄무늬만 남는다.
    lab_tophat_width_px: int = 81

    # --- hsv_absolute (비교용) ---
    hue_low: int = 15
    hue_high: int = 45
    saturation_low: int = 55
    saturation_high: int = 255
    value_low: int = 110
    value_high: int = 255

    roi_top_ratio: float = 0.50
    roi_bottom_ratio: float = 0.78
    row_step: int = 4

    # 세로 사다리꼴. 아래는 넓고 위는 좁다. 화면 폭 대비 반폭 비율이다.
    roi_bottom_half_width_ratio: float = 0.48
    roi_top_half_width_ratio: float = 0.30

    # lane_mask 를 살짝 부풀려 색 마스크와 교차시킨다. 세그멘테이션 경계가 한두 픽셀
    # 어긋나도 노란 픽셀을 놓치지 않기 위해서다.
    lane_dilate_px: int = 5
    open_kernel_px: int = 3

    search_half_width_ratio: float = 0.34
    maximum_run_width_ratio: float = 0.22
    minimum_run_pixels: int = 3
    minimum_rows: int = 8

    # 후보가 도로에 붙어 있을 것을 요구한다. 도로 위에 그려진 선이거나(겹침),
    # 두 주행영역 사이 틈에 있는 선이어야 한다. 벽의 나무 슬랫을 거르는 조건이다.
    require_road_anchor: bool = True
    side_support_ratio: float = 0.02
    side_search_ratio: float = 0.10

    # 예전 방식. 좌우 도로를 둘 다 요구하면 한쪽만 보이는 프레임에서 중앙선을 놓친다.
    require_road_on_both_sides: bool = False

    ghost_maximum_frames: int = 20
    history_size: int = 10
    ghost_confidence_floor: float = 0.20


def _row_bounds(height: int, config: YellowCenterConfig) -> tuple[int, int]:
    y_top = int(round(height * config.roi_top_ratio))
    y_bottom = min(height - 1, int(round(height * config.roi_bottom_ratio)))
    return y_top, y_bottom


def roi_trapezoid(height: int, width: int, config: YellowCenterConfig) -> np.ndarray:
    """화면 중앙을 축으로 한 세로 사다리꼴 마스크.

    중앙선은 늘 화면 중앙 부근에 있고 멀수록 좁은 범위로 모인다. 직사각형으로
    자르면 먼 행에서 화면 끝까지 훑어 벽과 바닥을 후보로 끌어들인다.
    """
    mask = np.zeros((height, width), dtype=bool)
    y_top, y_bottom = _row_bounds(height, config)
    if y_bottom <= y_top:
        return mask
    rows = np.arange(y_top, y_bottom + 1)
    progress = (rows - y_top) / float(y_bottom - y_top)
    half = width * (
        config.roi_top_half_width_ratio
        + progress * (config.roi_bottom_half_width_ratio - config.roi_top_half_width_ratio)
    )
    centre = width * 0.5
    columns = np.arange(width)[None, :]
    inside = (columns >= (centre - half)[:, None]) & (columns <= (centre + half)[:, None])
    mask[y_top : y_bottom + 1] = inside
    return mask


def yellow_mask_hsv(frame_bgr: np.ndarray, config: YellowCenterConfig) -> np.ndarray:
    """고정 HSV 상하한으로 자른 마스크 (예전 방식)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([config.hue_low, config.saturation_low, config.value_low], dtype=np.uint8),
        np.array([config.hue_high, config.saturation_high, config.value_high], dtype=np.uint8),
    )
    return mask.astype(bool)


def yellow_mask_lab(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    config: YellowCenterConfig,
) -> np.ndarray:
    """주변보다 국소적으로 더 노란 좁은 줄무늬만 남긴다.

    driving_mask 는 쓰지 않는다. 도로 안팎을 가리지 않고 '좁은 노란 줄'이라는
    형태 조건만으로 판정하므로, 세그멘테이션이 중앙선을 덮어도 결과가 같다.
    """
    del driving_mask  # 형태 기반 판정이라 필요 없다. 호출부 서명은 유지한다.
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    blue_yellow = lab[:, :, 2]
    width = max(3, int(config.lab_tophat_width_px) | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (width, 1))
    # opening 은 커널보다 좁은 밝은 줄무늬를 지운다. 원본에서 빼면 그 줄무늬만 남는다.
    surroundings = cv2.morphologyEx(blue_yellow, cv2.MORPH_OPEN, kernel)
    contrast = blue_yellow.astype(np.int16) - surroundings.astype(np.int16)
    return (contrast >= config.lab_margin) & (blue_yellow >= config.lab_minimum_b)


def yellow_mask(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    config: YellowCenterConfig,
) -> np.ndarray:
    """설정된 방식으로 노란 픽셀 마스크를 만든다."""
    if config.detector == HSV_ABSOLUTE:
        return yellow_mask_hsv(frame_bgr, config)
    return yellow_mask_lab(frame_bgr, driving_mask, config)


def _clean(mask: np.ndarray, config: YellowCenterConfig) -> np.ndarray:
    if config.open_kernel_px <= 1:
        return mask
    kernel = np.ones((config.open_kernel_px, config.open_kernel_px), np.uint8)
    cleaned = cv2.morphologyEx(mask.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    return cleaned.astype(bool)


def yellow_evidence(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    lane_mask: np.ndarray | None,
    config: YellowCenterConfig,
) -> np.ndarray:
    """중앙선 후보 픽셀.

    주행 영역 바깥(=영역 사이 틈)의 노란 픽셀과, 모델이 차선이라고 한 픽셀 중
    노란 것을 함께 근거로 삼는다. 세그멘테이션이 중앙선을 차선으로 덮는 프레임과
    주행영역 사이 틈으로 남기는 프레임이 둘 다 있어서 한쪽만으로는 부족하다.
    """
    yellow = _clean(yellow_mask(frame_bgr, driving_mask, config), config)
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
    """후보 양옆에 주행 가능 영역이 충분히 있는가."""
    width = driving_row.size
    left_from = max(0, start - search_px)
    right_to = min(width, end + 1 + search_px)
    left_road = int(driving_row[left_from:start].sum())
    right_road = int(driving_row[end + 1 : right_to].sum())
    return left_road >= support_px and right_road >= support_px


def _is_anchored_to_road(
    driving_row: np.ndarray,
    start: int,
    end: int,
    support_px: int,
    search_px: int,
) -> bool:
    """후보가 도로에 붙어 있는가.

    도로 위에 그려진 선이면 후보 픽셀이 주행영역과 겹친다. 두 주행영역 사이 틈에
    있는 선이면 좌우 모두에 주행영역이 있다. 벽의 구조물은 둘 다 아니다.
    """
    if int(driving_row[start : end + 1].sum()) > 0:
        return True
    return _has_road_on_both_sides(driving_row, start, end, support_px, search_px)


def extract_yellow_center(
    frame_bgr: np.ndarray,
    driving_mask: np.ndarray,
    lane_mask: np.ndarray | None,
    config: YellowCenterConfig | None = None,
    center_config: TwinMixingCenterConfig | None = None,
    seed_x: float | None = None,
) -> tuple[CenterPathResult, np.ndarray]:
    """노란 중앙선 경로와 근거 마스크를 돌려준다.

    seed_x 는 아래쪽 행 스캔을 시작할 x 다. 직전 프레임에서 확정한 경로의 바닥 x 를
    넘기면 프레임 사이 연속성이 생긴다. 없으면 화면 중앙에서 시작한다.

    매번 화면 중앙에서 시작하면, 곡선에서 중앙선이 중앙에서 멀어졌을 때 바닥 근처의
    엉뚱한 노란 조각이 더 가깝다는 이유로 시작점이 되어 거기서부터 잘못 따라간다.

    실패하면 state="LOST" 인 결과를 준다. 호출자는 그때 센터 갭으로 내려간다.
    """
    config = config or YellowCenterConfig()
    center_config = center_config or TwinMixingCenterConfig()
    height, width = driving_mask.shape[:2]
    evidence = yellow_evidence(frame_bgr, driving_mask, lane_mask, config)

    y_top, y_bottom = _row_bounds(height, config)
    # ROI 밖의 노란 픽셀은 쓰지 않으므로 근거 마스크에서도 지운다.
    # 화면에 남겨 두면 벽면 반사가 검출된 것처럼 보여 오해를 부른다.
    evidence &= roi_trapezoid(height, width, config)

    rows = range(y_bottom, y_top - 1, -max(1, config.row_step))
    attempted_rows = len(rows)
    search_half = max(12, int(round(width * config.search_half_width_ratio)))
    maximum_run = max(4, int(round(width * config.maximum_run_width_ratio)))
    side_support = max(3, int(round(width * config.side_support_ratio)))
    side_search = max(side_support * 2, int(round(width * config.side_search_ratio)))
    driving = driving_mask.astype(bool)

    expected_x = width * 0.5 if seed_x is None else float(np.clip(seed_x, 0.0, width - 1.0))
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
            if config.require_road_anchor and not _is_anchored_to_road(
                driving[y], start, end, side_support, side_search
            ):
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
            recent = self._history[-min(5, len(self._history)) :]
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

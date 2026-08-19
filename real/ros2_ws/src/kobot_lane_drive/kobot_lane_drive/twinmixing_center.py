"""TwinMixing 마스크에서 따라갈 선 하나를 뽑는다 — 주행 우선순위 2·3단계.

노란 중앙선(yellow_center.py)이 실패했을 때 쓰는 폴백 두 가지가 여기 있다.

  2단계 CENTER_GAP     주행 가능 영역이 두 덩이로 갈렸을 때 그 사이 틈의 중앙
  3단계 REGION_CENTER  덩이가 하나뿐이면 그 덩이의 중앙

행 단위로 후보 x 를 모은 뒤 `_robust_polyfit` 으로 2차 곡선을 맞춘다. MAD 기반
이상치 제거가 들어 있어 몇 행이 튀어도 선이 끌려가지 않는다. 이 피팅기는
yellow_center.py 도 그대로 재사용한다.

`fill_narrow_gaps` 는 3단계 **직전에만** 부른다. 중앙선 자국이 도로를 두 조각으로
가르면 REGION_CENTER 가 넓은 쪽 하나만 골라 중앙이 크게 치우치기 때문인데,
센터 갭 판정이 끝나기 전에 부르면 진짜로 갈라진 도로까지 메워 버린다.

`render_overlay` 는 웹 화면을 그린다. 판단에는 관여하지 않는다.

ROS 에 의존하지 않는다. 입력은 numpy 배열, 출력은 CenterPathResult 다.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class TwinMixingCenterConfig:
    roi_top_ratio: float = 0.46
    roi_bottom_ratio: float = 0.78
    row_step: int = 4
    search_half_width_ratio: float = 0.24
    minimum_gap_width_ratio: float = 0.004
    maximum_gap_width_ratio: float = 0.18
    minimum_side_support_ratio: float = 0.02
    minimum_points: int = 18
    lookahead_y_ratio: float = 0.67
    minimum_region_width_ratio: float = 0.12
    region_search_half_width_ratio: float = 0.40
    # 영역 중앙을 잡기 직전에 메울 틈의 최대 폭 (화면 폭 대비).
    # 중앙선 자국처럼 얇은 틈이 도로를 갈라 놓으면 넓은 쪽 절반만 도로로 보게 된다.
    region_fill_gap_ratio: float = 0.035


@dataclass
class CenterPathResult:
    state: str
    path_source: str
    confidence: float
    raw_points: np.ndarray
    fitted_points: np.ndarray
    planned_points: np.ndarray
    lookahead: tuple[int, int] | None
    center_offset_px: float | None
    heading_deg: float | None
    mean_gap_width_px: float | None
    lane_support_ratio: float

    @property
    def found(self) -> bool:
        return self.lookahead is not None and len(self.fitted_points) > 0


def letterbox(
    image: np.ndarray,
    target: tuple[int, int] = (384, 640),
) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    source_height, source_width = image.shape[:2]
    target_height, target_width = target
    ratio = min(target_height / source_height, target_width / source_width)
    resized_width = int(round(source_width * ratio))
    resized_height = int(round(source_height * ratio))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_LINEAR,
    )
    horizontal = target_width - resized_width
    vertical = target_height - resized_height
    left = int(round(horizontal / 2 - 0.1))
    right = int(round(horizontal / 2 + 0.1))
    top = int(round(vertical / 2 - 0.1))
    bottom = int(round(vertical / 2 + 0.1))
    padded = cv2.copyMakeBorder(
        resized,
        top,
        bottom,
        left,
        right,
        cv2.BORDER_CONSTANT,
        value=(114, 114, 114),
    )
    return padded, (top, bottom, left, right)


def restore_mask(
    mask: np.ndarray,
    padding: tuple[int, int, int, int],
    output_size: tuple[int, int],
) -> np.ndarray:
    top, bottom, left, right = padding
    height, width = mask.shape
    cropped = mask[
        top : height - bottom if bottom else height,
        left : width - right if right else width,
    ]
    return cv2.resize(
        cropped.astype(np.uint8),
        output_size,
        interpolation=cv2.INTER_NEAREST,
    ).astype(bool)


def _false_runs(row: np.ndarray) -> list[tuple[int, int]]:
    padded = np.pad((~row).astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1) - 1
    return list(zip(starts.tolist(), ends.tolist()))


def _true_runs(row: np.ndarray) -> list[tuple[int, int]]:
    padded = np.pad(row.astype(np.int8), (1, 1))
    changes = np.diff(padded)
    starts = np.flatnonzero(changes == 1)
    ends = np.flatnonzero(changes == -1) - 1
    return list(zip(starts.tolist(), ends.tolist()))


def _robust_polyfit(points: np.ndarray) -> tuple[np.ndarray, np.ndarray] | None:
    if len(points) < 3:
        return None
    y = points[:, 1].astype(np.float64)
    x = points[:, 0].astype(np.float64)
    degree = 2 if len(points) >= 8 else 1
    coefficients = np.polyfit(y, x, degree)
    residual = np.abs(x - np.polyval(coefficients, y))
    median = float(np.median(residual))
    mad = float(np.median(np.abs(residual - median)))
    threshold = max(5.0, median + 2.5 * max(1.0, 1.4826 * mad))
    inliers = residual <= threshold
    if int(inliers.sum()) < max(3, len(points) // 2):
        return None
    coefficients = np.polyfit(y[inliers], x[inliers], degree)
    return coefficients, inliers


def _fit_path(
    samples: list[tuple[float, float]],
    widths: list[float],
    lane_supported: int,
    attempted_rows: int,
    width: int,
    height: int,
    config: TwinMixingCenterConfig,
    path_source: str,
) -> CenterPathResult:
    empty = np.empty((0, 2), dtype=np.int32)
    raw_points = np.asarray(samples, dtype=np.float64).reshape((-1, 2))
    minimum_points = min(config.minimum_points, max(6, attempted_rows // 3))
    fitted = None if len(samples) < minimum_points else _robust_polyfit(raw_points)
    if fitted is None:
        return CenterPathResult(
            state="LOST",
            path_source="NONE",
            confidence=0.0,
            raw_points=raw_points.astype(np.int32),
            fitted_points=empty,
            planned_points=empty,
            lookahead=None,
            center_offset_px=None,
            heading_deg=None,
            mean_gap_width_px=float(np.mean(widths)) if widths else None,
            lane_support_ratio=lane_supported / max(1, len(samples)),
        )

    coefficients, inliers = fitted
    inlier_points = raw_points[inliers]
    fit_y_bottom = int(np.max(inlier_points[:, 1]))
    fit_y_top = int(np.min(inlier_points[:, 1]))
    path_y = np.arange(fit_y_bottom, fit_y_top - 1, -3, dtype=np.int32)
    path_x = np.clip(np.polyval(coefficients, path_y), 0, width - 1)
    fitted_points = np.column_stack((path_x, path_y)).astype(np.int32)
    lookahead_y = max(
        fit_y_top,
        min(fit_y_bottom, int(round(height * config.lookahead_y_ratio))),
    )
    lookahead_x = max(
        0,
        min(width - 1, int(round(float(np.polyval(coefficients, lookahead_y))))),
    )
    anchor_x = width * 0.5
    anchor_y = height - 1
    center_offset = lookahead_x - anchor_x
    heading = math.degrees(math.atan2(center_offset, anchor_y - lookahead_y))
    planned_x = path_x.copy()
    below = path_y >= lookahead_y
    if np.any(below):
        span = max(1.0, anchor_y - lookahead_y)
        blend = np.clip((anchor_y - path_y[below]) / span, 0.0, 1.0)
        planned_x[below] = anchor_x * (1.0 - blend) + path_x[below] * blend
    planned_points = np.vstack(
        (
            np.asarray([[int(round(anchor_x)), int(anchor_y)]], dtype=np.int32),
            np.column_stack((planned_x, path_y)).astype(np.int32),
        )
    )
    coverage = min(1.0, len(inlier_points) / max(1.0, attempted_rows * 0.72))
    inlier_ratio = len(inlier_points) / max(1, len(raw_points))
    confidence = max(0.0, min(1.0, coverage * (0.55 + 0.45 * inlier_ratio)))
    return CenterPathResult(
        state="TRACKED",
        path_source=path_source,
        confidence=confidence,
        raw_points=raw_points.astype(np.int32),
        fitted_points=fitted_points,
        planned_points=planned_points,
        lookahead=(lookahead_x, lookahead_y),
        center_offset_px=float(center_offset),
        heading_deg=float(heading),
        mean_gap_width_px=float(np.mean(widths)) if widths else None,
        lane_support_ratio=lane_supported / max(1, len(samples)),
    )


def fill_narrow_gaps(driving_mask: np.ndarray, maximum_gap_px: int) -> np.ndarray:
    """행마다 폭이 maximum_gap_px 이하인 빈틈을 주행영역으로 메운다.

    중앙선 자국이나 그림자가 만든 얇은 틈이 도로를 두 조각으로 가르면, 가장 넓은
    조각만 쓰는 계산에서 도로의 절반이 통째로 빠진다. 그 결과 한가운데가 실제 도로
    중앙에서 크게 벗어난다.

    이 함수는 마지막 폴백 직전에만 쓴다. 그 전에 센터 갭 판정이 이미 끝나 있으므로
    진짜로 갈라진 도로를 지울 위험이 없다.
    """
    if maximum_gap_px <= 0:
        return driving_mask
    filled = driving_mask.copy()
    for y in range(filled.shape[0]):
        row = filled[y]
        if not row.any():
            continue
        for start, end in _false_runs(row):
            if start == 0 or end >= row.size - 1:
                continue  # 화면 끝에 닿은 틈은 바깥이라 메우지 않는다
            if end - start + 1 <= maximum_gap_px:
                row[start : end + 1] = True
    return filled


def extract_center_path(
    driving_mask: np.ndarray,
    lane_mask: np.ndarray | None = None,
    config: TwinMixingCenterConfig | None = None,
) -> CenterPathResult:
    config = config or TwinMixingCenterConfig()
    driving = driving_mask.astype(bool)
    lane = np.zeros_like(driving, dtype=bool) if lane_mask is None else lane_mask.astype(bool)
    height, width = driving.shape
    y_top = int(round(height * config.roi_top_ratio))
    y_bottom = min(height - 1, int(round(height * config.roi_bottom_ratio)))
    rows = range(y_bottom, y_top - 1, -max(1, config.row_step))
    attempted_rows = len(rows)
    search_half = max(8, int(round(width * config.search_half_width_ratio)))
    minimum_gap = max(2, int(round(width * config.minimum_gap_width_ratio)))
    maximum_gap = max(minimum_gap, int(round(width * config.maximum_gap_width_ratio)))
    side_support = max(3, int(round(width * config.minimum_side_support_ratio)))

    expected_x = width * 0.5
    gap_samples: list[tuple[float, float]] = []
    gap_widths: list[float] = []
    gap_lane_supported = 0
    for y in rows:
        candidates: list[tuple[float, int, int, bool]] = []
        for start, end in _false_runs(driving[y]):
            gap_width = end - start + 1
            center = (start + end) * 0.5
            if not minimum_gap <= gap_width <= maximum_gap or abs(center - expected_x) > search_half:
                continue
            left_start = max(0, start - side_support * 3)
            right_end = min(width, end + 1 + side_support * 3)
            if int(driving[y, left_start:start].sum()) < side_support or int(driving[y, end + 1:right_end].sum()) < side_support:
                continue
            has_lane = bool(lane[y, max(0, start - 5):min(width, end + 6)].any())
            candidates.append((abs(center - expected_x) - (search_half * 0.12 if has_lane else 0.0), start, end, has_lane))
        if candidates:
            _, start, end, has_lane = min(candidates, key=lambda item: item[0])
            center = (start + end) * 0.5
            gap_samples.append((center, float(y)))
            gap_widths.append(float(end - start + 1))
            gap_lane_supported += int(has_lane)
            expected_x = 0.72 * expected_x + 0.28 * center
    result = _fit_path(gap_samples, gap_widths, gap_lane_supported, attempted_rows, width, height, config, "CENTER_GAP")
    if result.found:
        return result

    # 노란선도 센터 갭도 실패했다. 얇은 틈이 도로를 갈라 놓았을 수 있으므로 메운다.
    driving = fill_narrow_gaps(
        driving, int(round(width * config.region_fill_gap_ratio))
    )
    minimum_region = max(12, int(round(width * config.minimum_region_width_ratio)))
    region_search_half = max(20, int(round(width * config.region_search_half_width_ratio)))
    expected_x = width * 0.5
    region_started = False
    region_samples: list[tuple[float, float]] = []
    region_widths: list[float] = []
    region_lane_supported = 0
    for y in rows:
        candidates: list[tuple[float, int, int]] = []
        for start, end in _true_runs(driving[y]):
            region_width = end - start + 1
            center = (start + end) * 0.5
            if region_width < minimum_region:
                continue
            if not region_started and not start <= expected_x <= end:
                continue
            if region_started and abs(center - expected_x) > region_search_half:
                continue
            candidates.append((abs(center - expected_x), start, end))
        if not candidates:
            continue
        _, start, end = min(candidates, key=lambda item: item[0])
        region_started = True
        center = (start + end) * 0.5
        region_samples.append((center, float(y)))
        region_widths.append(float(end - start + 1))
        region_lane_supported += int(lane[y, max(0, int(center) - 5):min(width, int(center) + 6)].any())
        expected_x = 0.72 * expected_x + 0.28 * center
    return _fit_path(region_samples, region_widths, region_lane_supported, attempted_rows, width, height, config, "REGION_CENTER")


def _lane_side_masks(
    driving: np.ndarray, fitted_points: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """중앙선을 기준으로 주행 영역을 좌/우 차선으로 가른다 (화면 표시용)."""
    height, width = driving.shape
    center_x = np.full(height, np.nan, dtype=np.float64)
    if fitted_points is not None and len(fitted_points) >= 2:
        ys = np.clip(fitted_points[:, 1].astype(int), 0, height - 1)
        center_x[ys] = fitted_points[:, 0]
        known = np.flatnonzero(~np.isnan(center_x))
        if known.size >= 2:
            rows = np.arange(height)
            center_x = np.interp(rows, known, center_x[known], left=np.nan, right=np.nan)
    valid = ~np.isnan(center_x)
    if not valid.any():
        empty = np.zeros_like(driving)
        return empty, empty
    columns = np.arange(width)[None, :]
    boundary = np.where(valid, center_x, -1.0)[:, None]
    inside = valid[:, None]
    left = driving & inside & (columns < boundary)
    right = driving & inside & (columns > boundary)
    return left, right


def _tint(overlay: np.ndarray, mask: np.ndarray, colour, strength: float) -> None:
    if not np.any(mask):
        return
    layer = np.zeros_like(overlay)
    layer[:] = colour
    overlay[mask] = cv2.addWeighted(
        overlay[mask], 1.0 - strength, layer[mask], strength, 0
    )


def path_x_at(points: np.ndarray, y_value: float) -> float | None:
    """맞춘 경로에서 지정한 행의 x 를 선형 보간한다. 범위 밖이면 None."""
    if points is None or len(points) < 2:
        return None
    ys = points[:, 1].astype(np.float64)
    xs = points[:, 0].astype(np.float64)
    if y_value < ys.min() or y_value > ys.max():
        return None
    order = np.argsort(ys)
    return float(np.interp(y_value, ys[order], xs[order]))


def render_overlay(
    frame: np.ndarray,
    driving_mask: np.ndarray,
    lane_mask: np.ndarray,
    result: CenterPathResult,
    steering_rad: float,
    inference_ms: float,
    fps: float,
    device_name: str,
    control_mode: str | None = None,
    yellow_mask: np.ndarray | None = None,
    decision=None,
    occupancy=None,
    sectors=None,
    draw_hud: bool = False,
) -> np.ndarray:
    overlay = frame.copy()
    driving = driving_mask.astype(bool)
    lane = lane_mask.astype(bool)

    # 좌/우 차선을 서로 다른 색으로 칠하고, 지금 쓰는 차선을 진하게 한다.
    left_lane, right_lane = _lane_side_masks(driving, result.fitted_points)
    active = None if decision is None else decision.state
    left_strength = 0.58 if active == "SHIFT_LEFT" else 0.34
    right_strength = 0.58 if active == "SHIFT_RIGHT" else 0.34
    _tint(overlay, left_lane, (200, 90, 0), left_strength)
    _tint(overlay, right_lane, (150, 60, 190), right_strength)
    _tint(overlay, driving & ~left_lane & ~right_lane, (255, 0, 0), 0.45)

    _tint(overlay, lane, (0, 255, 0), 0.55)
    if yellow_mask is not None:
        _tint(overlay, yellow_mask.astype(bool), (0, 235, 255), 0.92)

    for x, y in result.raw_points[::2]:
        cv2.circle(overlay, (int(x), int(y)), 2, (0, 165, 255), -1, cv2.LINE_AA)
    if len(result.fitted_points) >= 2:
        cv2.polylines(
            overlay,
            [result.fitted_points.reshape((-1, 1, 2))],
            False,
            (0, 255, 255),
            5,
            cv2.LINE_AA,
        )
    if len(result.planned_points) >= 2:
        cv2.polylines(
            overlay,
            [result.planned_points.reshape((-1, 1, 2))],
            False,
            (255, 255, 0),
            4,
            cv2.LINE_AA,
        )
    anchor = (frame.shape[1] // 2, frame.shape[0] - 1)
    if result.lookahead is not None:
        target_x = (
            int(round(decision.target_x_px)) if decision is not None else result.lookahead[0]
        )
        target = (target_x, result.lookahead[1])
        cv2.arrowedLine(
            overlay, anchor, target, (255, 255, 255), 3, cv2.LINE_AA, tipLength=0.08
        )
        cv2.drawMarker(overlay, target, (0, 0, 255), cv2.MARKER_CROSS, 24, 4, cv2.LINE_AA)
        if decision is not None and target_x != result.lookahead[0]:
            # 중앙선 자체 위치도 같이 보여줘야 왜 비켜섰는지 읽힌다.
            cv2.drawMarker(
                overlay, result.lookahead, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 18, 2,
                cv2.LINE_AA,
            )

    if not draw_hud:
        return overlay

    direction = "CENTER"
    if result.center_offset_px is not None:
        if result.center_offset_px > 6:
            direction = "RIGHT"
        elif result.center_offset_px < -6:
            direction = "LEFT"
    # 실제 모터로 나가는 값과 같은 부호로 표시한다. 예전에는 부호가 반대라
    # 화면만 보면 조향 방향을 거꾸로 읽게 됐다.
    legacy_angle = -steering_rad / 0.0068

    drive_line = decision.state if decision is not None else "-"
    blocked = decision.blocked_side if decision is not None else None
    lidar_text = "LIDAR -"
    if sectors is not None:
        def metres(value):
            return "-" if value is None else f"{value:.2f}m"

        lidar_text = (
            f"LIDAR L {metres(sectors.left_m)}  F {metres(sectors.front_m)}  "
            f"R {metres(sectors.right_m)}"
        )
    lane_text = "LANES -"
    if occupancy is not None and occupancy.measured_rows > 0:
        lane_text = (
            f"LANES L {occupancy.left_width_px:.0f}px  R {occupancy.right_width_px:.0f}px"
            + (f"  NARROW {occupancy.narrow_side.upper()}" if occupancy.narrow_side else "")
        )

    lines = [
        f"TwinMixing CUDA | {device_name}" + (f" | {control_mode}" if control_mode else ""),
        f"SOURCE {result.path_source}  CONF {result.confidence:.2f}  "
        f"{inference_ms:.1f} ms  {fps:.1f} FPS",
        f"DRIVE LINE {drive_line}" + (f"  BLOCKED {blocked.upper()}" if blocked else ""),
        (
            f"TARGET {direction}  OFFSET {result.center_offset_px:+.1f}px  "
            f"HEADING {result.heading_deg:+.1f}deg"
            if result.center_offset_px is not None and result.heading_deg is not None
            else "TARGET LOST"
        ),
        lane_text,
        lidar_text,
        f"STEERING {steering_rad:+.3f} rad  LEGACY {legacy_angle:+.1f}",
        "YELLOW center | ORANGE left lane | PURPLE right lane | RED target",
    ]
    box_height = 22 * len(lines) + 14
    cv2.rectangle(overlay, (8, 8), (min(frame.shape[1] - 8, 760), box_height), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(
            overlay,
            text,
            (18, 31 + index * 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return overlay

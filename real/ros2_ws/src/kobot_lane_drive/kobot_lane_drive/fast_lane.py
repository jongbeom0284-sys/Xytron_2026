from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import cv2
import numpy as np


@dataclass(frozen=True)
class FastLaneConfig:
    roi_top_ratio: float = 0.43
    roi_bottom_ratio: float = 0.94
    lookahead_ratio: float = 0.70
    window_count: int = 11
    window_margin_ratio: float = 0.075
    minimum_window_pixels: int = 5
    minimum_windows: int = 4
    expected_bottom_width_ratio: float = 0.82
    expected_top_width_ratio: float = 0.30
    clahe_clip_limit: float = 1.6


@dataclass(frozen=True)
class LaneCurve:
    coefficients: np.ndarray
    support_windows: int
    residual_px: float

    def x_at(self, y_value: float) -> float:
        return float(np.polyval(self.coefficients, y_value))


@dataclass(frozen=True)
class FastLaneResult:
    state: str
    confidence: float
    center_offset_px: float | None
    lookahead_y_px: int
    left: LaneCurve | None
    right: LaneCurve | None
    yellow_center_x_px: float | None
    processing_ms: float
    debug_image: np.ndarray | None

    def status(self) -> dict[str, object]:
        return {
            "state": self.state,
            "confidence": round(self.confidence, 3),
            "center_offset_px": (
                None
                if self.center_offset_px is None
                else round(self.center_offset_px, 2)
            ),
            "lookahead_y_px": self.lookahead_y_px,
            "left_detected": self.left is not None,
            "right_detected": self.right is not None,
            "yellow_center_x_px": (
                None
                if self.yellow_center_x_px is None
                else round(self.yellow_center_x_px, 2)
            ),
            "processing_ms": round(self.processing_ms, 2),
            "metric_calibrated": False,
            "motor_output": False,
        }


def _validate_frame(frame: np.ndarray) -> None:
    if not isinstance(frame, np.ndarray):
        raise TypeError("frame must be a numpy array")
    if frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a BGR image with shape HxWx3")
    if frame.shape[0] < 120 or frame.shape[1] < 160:
        raise ValueError("frame is too small for lane detection")


def _roi_mask(height: int, width: int, y_top: int, y_bottom: int) -> np.ndarray:
    mask = np.zeros((height, width), dtype=np.uint8)
    polygon = np.asarray(
        [[
            (0, y_bottom),
            (width - 1, y_bottom),
            (int(width * 0.88), y_top),
            (int(width * 0.12), y_top),
        ]],
        dtype=np.int32,
    )
    cv2.fillPoly(mask, polygon, 255)
    return mask


def _colour_masks(
    frame: np.ndarray,
    roi: np.ndarray,
    config: FastLaneConfig,
) -> tuple[np.ndarray, np.ndarray]:
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lightness, channel_a, channel_b = cv2.split(lab)

    clahe = cv2.createCLAHE(
        clipLimit=config.clahe_clip_limit,
        tileGridSize=(8, 8),
    )
    normalized_l = clahe.apply(lightness)
    roi_values = lightness[roi > 0]
    dynamic_white_floor = max(85.0, float(np.percentile(roi_values, 62)))
    neutral = (
        (np.abs(channel_a.astype(np.int16) - 128) <= 22)
        & (np.abs(channel_b.astype(np.int16) - 128) <= 24)
    )
    white_support = neutral & (lightness >= dynamic_white_floor)
    yellow_lab = (
        (channel_b >= 148)
        & (channel_a >= 98)
        & (channel_a <= 145)
        & (lightness >= 95)
        & (
            channel_b.astype(np.int16) - channel_a.astype(np.int16)
            >= 22
        )
    )
    yellow = yellow_lab

    blurred_l = cv2.GaussianBlur(normalized_l, (5, 5), 0)
    edges = cv2.Canny(blurred_l, 35, 105)
    white_band = np.where(white_support, 255, 0).astype(np.uint8)
    white_band = cv2.dilate(
        white_band,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
        iterations=1,
    )
    white_mask = cv2.bitwise_and(edges, white_band)
    yellow_mask = np.where(yellow, 255, 0).astype(np.uint8)
    yellow_exclusion = cv2.dilate(
        yellow_mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
        iterations=1,
    )
    white_mask[yellow_exclusion > 0] = 0
    white_mask = cv2.bitwise_and(white_mask, roi)
    yellow_mask = cv2.bitwise_and(yellow_mask, roi)

    yellow_mask = cv2.morphologyEx(
        yellow_mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 7)),
    )
    return white_mask, yellow_mask


def _base_peak(
    mask: np.ndarray,
    x_min: int,
    x_max: int,
    y_min: int,
    y_max: int,
) -> int | None:
    if x_max <= x_min or y_max <= y_min:
        return None
    histogram = np.sum(mask[y_min:y_max, x_min:x_max] > 0, axis=0)
    if histogram.size == 0 or int(histogram.max()) < 3:
        return None
    kernel_width = max(9, (x_max - x_min) // 20)
    if kernel_width % 2 == 0:
        kernel_width += 1
    smooth = np.convolve(
        histogram.astype(np.float32),
        np.ones(kernel_width, dtype=np.float32) / kernel_width,
        mode="same",
    )
    return x_min + int(np.argmax(smooth))


def _sliding_curve(
    mask: np.ndarray,
    base_x: int | None,
    y_top: int,
    y_bottom: int,
    config: FastLaneConfig,
) -> LaneCurve | None:
    if base_x is None:
        return None
    height, width = mask.shape
    window_height = max(1, (y_bottom - y_top) // config.window_count)
    margin = max(18, int(width * config.window_margin_ratio))
    current_x = float(base_x)
    centers: list[tuple[float, float]] = []

    for index in range(config.window_count):
        y_high = y_bottom - index * window_height
        y_low = max(y_top, y_high - window_height)
        x_low = max(0, int(current_x - margin))
        x_high = min(width, int(current_x + margin))
        local_y, local_x = np.nonzero(mask[y_low:y_high, x_low:x_high])
        xs = local_x + x_low
        ys = local_y + y_low
        if xs.size < config.minimum_window_pixels:
            continue
        median_x = float(np.median(xs))
        median_y = float(np.median(ys))
        centers.append((median_y, median_x))
        current_x = median_x

    if len(centers) < config.minimum_windows:
        return None
    values = np.asarray(centers, dtype=np.float64)
    ys = values[:, 0]
    xs = values[:, 1]
    if float(np.ptp(ys)) < (y_bottom - y_top) * 0.32:
        return None

    degree = 2 if len(centers) >= 6 else 1
    keep = np.ones(xs.shape, dtype=bool)
    coefficients = np.polyfit(ys, xs, degree)
    for _ in range(3):
        residuals = np.abs(xs - np.polyval(coefficients, ys))
        median = float(np.median(residuals))
        threshold = max(8.0, 2.8 * median)
        next_keep = residuals <= threshold
        if int(np.count_nonzero(next_keep)) < config.minimum_windows:
            break
        keep = next_keep
        coefficients = np.polyfit(ys[keep], xs[keep], degree)
    residual = float(np.sqrt(np.mean((xs[keep] - np.polyval(coefficients, ys[keep])) ** 2)))
    return LaneCurve(coefficients, int(np.count_nonzero(keep)), residual)


def _expected_lane_width(
    y_value: float,
    y_top: int,
    y_bottom: int,
    width: int,
    config: FastLaneConfig,
) -> float:
    fraction = np.clip((y_value - y_top) / max(1, y_bottom - y_top), 0.0, 1.0)
    ratio = (
        config.expected_top_width_ratio
        + fraction
        * (config.expected_bottom_width_ratio - config.expected_top_width_ratio)
    )
    return float(width * ratio)


def _yellow_center(
    yellow_mask: np.ndarray,
    y_top: int,
    y_bottom: int,
    center_x: int,
) -> float | None:
    height, width = yellow_mask.shape
    band_top = max(y_top, int(height * 0.55))
    band_bottom = min(y_bottom, int(height * 0.88))
    ys, xs = np.nonzero(yellow_mask[band_top:band_bottom])
    if xs.size < 18:
        return None
    valid = np.abs(xs - center_x) < width * 0.30
    if int(np.count_nonzero(valid)) < 18:
        return None
    return float(np.median(xs[valid]))


def _line_points(
    curve: LaneCurve,
    y_top: int,
    y_bottom: int,
    width: int,
) -> np.ndarray:
    ys = np.linspace(y_bottom, y_top, 32)
    xs = np.clip(np.polyval(curve.coefficients, ys), 0, width - 1)
    return np.column_stack((xs, ys)).astype(np.int32)


def detect_fast_lane(
    frame: np.ndarray,
    config: FastLaneConfig | None = None,
    render_debug: bool = False,
) -> FastLaneResult:
    """Detect the track corridor in pixels without camera calibration or motor output."""
    _validate_frame(frame)
    started_at = perf_counter()
    config = config or FastLaneConfig()
    height, width = frame.shape[:2]
    y_top = int(height * config.roi_top_ratio)
    y_bottom = min(height - 1, int(height * config.roi_bottom_ratio))
    lookahead_y = int(height * config.lookahead_ratio)
    image_center_x = width // 2

    roi = _roi_mask(height, width, y_top, y_bottom)
    white_mask, yellow_mask = _colour_masks(frame, roi, config)
    base_band_top = max(y_top, int(height * 0.72))
    left_base = _base_peak(
        white_mask,
        0,
        int(width * 0.34),
        base_band_top,
        y_bottom,
    )
    right_base = _base_peak(
        white_mask,
        int(width * 0.66),
        width,
        base_band_top,
        y_bottom,
    )
    left = _sliding_curve(white_mask, left_base, y_top, y_bottom, config)
    right = _sliding_curve(white_mask, right_base, y_top, y_bottom, config)
    yellow_x = _yellow_center(yellow_mask, y_top, y_bottom, image_center_x)

    left_x = None if left is None else left.x_at(lookahead_y)
    right_x = None if right is None else right.x_at(lookahead_y)
    expected_width = _expected_lane_width(
        lookahead_y, y_top, y_bottom, width, config
    )

    if (
        left is not None
        and right is not None
        and left_x is not None
        and right_x is not None
        and yellow_x is not None
    ):
        paired_center = (left_x + right_x) * 0.5
        if abs(paired_center - yellow_x) > width * 0.10:
            left_center = left_x + expected_width * 0.5
            right_center = right_x - expected_width * 0.5
            if abs(left_center - yellow_x) <= abs(right_center - yellow_x):
                right = None
                right_x = None
            else:
                left = None
                left_x = None

    state = "LOST"
    confidence = 0.0
    center_x: float | None = None
    if left is not None and right is not None and left_x is not None and right_x is not None:
        measured_width = right_x - left_x
        width_ratio = measured_width / max(1.0, expected_width)
        if measured_width > width * 0.18 and 0.45 <= width_ratio <= 1.70:
            state = "BOTH"
            center_x = (left_x + right_x) * 0.5
            support = min(left.support_windows, right.support_windows) / config.window_count
            residual_score = np.exp(-(left.residual_px + right.residual_px) / 45.0)
            width_score = np.exp(-abs(width_ratio - 1.0) * 0.9)
            confidence = float(np.clip(0.45 + 0.30 * support + 0.15 * residual_score + 0.10 * width_score, 0.0, 1.0))
    elif left is not None and left_x is not None:
        state = "LEFT_ONLY"
        center_x = left_x + expected_width * 0.5
        confidence = float(np.clip(0.25 + 0.40 * left.support_windows / config.window_count, 0.0, 0.68))
    elif right is not None and right_x is not None:
        state = "RIGHT_ONLY"
        center_x = right_x - expected_width * 0.5
        confidence = float(np.clip(0.25 + 0.40 * right.support_windows / config.window_count, 0.0, 0.68))
    if center_x is not None and yellow_x is not None:
        disagreement = abs(center_x - yellow_x)
        if disagreement <= width * 0.08:
            center_x = 0.78 * center_x + 0.22 * yellow_x
            confidence = min(1.0, confidence + 0.06)
        elif state == "BOTH":
            confidence *= 0.82

    offset = None if center_x is None else float(center_x - image_center_x)
    debug_image = None
    if render_debug:
        debug_image = frame.copy()
        mask_preview = np.zeros_like(frame)
        mask_preview[white_mask > 0] = (255, 210, 30)
        mask_preview[yellow_mask > 0] = (0, 210, 255)
        debug_image = cv2.addWeighted(debug_image, 0.80, mask_preview, 0.35, 0)
        if left is not None:
            cv2.polylines(debug_image, [_line_points(left, y_top, y_bottom, width)], False, (255, 80, 20), 4, cv2.LINE_AA)
        if right is not None:
            cv2.polylines(debug_image, [_line_points(right, y_top, y_bottom, width)], False, (20, 180, 255), 4, cv2.LINE_AA)
        if center_x is not None:
            cv2.circle(debug_image, (int(round(center_x)), lookahead_y), 8, (0, 255, 70), -1, cv2.LINE_AA)
            cv2.line(debug_image, (image_center_x, lookahead_y - 22), (image_center_x, lookahead_y + 22), (0, 0, 255), 2, cv2.LINE_AA)
        cv2.line(debug_image, (0, lookahead_y), (width - 1, lookahead_y), (80, 80, 80), 1, cv2.LINE_AA)
        cv2.rectangle(debug_image, (10, 10), (350, 66), (12, 20, 35), -1)

    processing_ms = (perf_counter() - started_at) * 1000.0
    if debug_image is not None:
        cv2.putText(debug_image, f"{state}  conf {confidence:.2f}", (22, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (255, 255, 255), 2, cv2.LINE_AA)
        offset_text = "n/a" if offset is None else f"{offset:+.1f}px"
        cv2.putText(debug_image, f"offset {offset_text}  {processing_ms:.1f}ms", (22, 59), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (205, 225, 240), 1, cv2.LINE_AA)

    return FastLaneResult(
        state=state,
        confidence=confidence,
        center_offset_px=offset,
        lookahead_y_px=lookahead_y,
        left=left,
        right=right,
        yellow_center_x_px=yellow_x,
        processing_ms=processing_ms,
        debug_image=debug_image,
    )

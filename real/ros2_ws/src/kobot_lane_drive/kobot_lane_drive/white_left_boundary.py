from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter

import cv2
import numpy as np


DELTAS = np.arange(-3, 6, dtype=np.int16)


@dataclass(frozen=True)
class WhiteLeftConfig:
    work_width: int = 320
    work_height: int = 240
    near_y_ratio: float = 0.80
    far_y_ratio: float = 0.52
    control_y_ratio: float = 0.70
    left_search_ratio: float = 0.52
    minimum_path_score: float = 0.30
    # Approved frame 1 with this detector: center 320px - left line 76px.
    desired_left_distance_ratio: float = 244.0 / 640.0


@dataclass(frozen=True)
class WhiteLeftResult:
    state: str
    confidence: float
    path: np.ndarray | None
    control_y_px: int
    left_boundary_x_px: float | None
    desired_left_distance_px: float
    center_offset_px: float | None
    processing_ms: float
    debug_image: np.ndarray | None

    def status(self) -> dict[str, object]:
        return {
            "state": self.state,
            "confidence": round(self.confidence, 3),
            "control_y_px": self.control_y_px,
            "left_boundary_x_px": (
                None
                if self.left_boundary_x_px is None
                else round(self.left_boundary_x_px, 2)
            ),
            "desired_left_distance_px": round(self.desired_left_distance_px, 2),
            "center_offset_px": (
                None
                if self.center_offset_px is None
                else round(self.center_offset_px, 2)
            ),
            "processing_ms": round(self.processing_ms, 2),
            "control_mode": "frame90_white_left_boundary",
            "metric_calibrated": False,
            "motor_output": False,
        }


def white_ridge_score(frame: np.ndarray) -> np.ndarray:
    """Score thin, locally bright, low-saturation paint like the frame-90 code."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY).astype(np.float32)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    local = cv2.GaussianBlur(gray, (0, 0), 13.0)
    local_contrast = np.clip((gray - local) / 30.0, 0.0, 1.0)
    top_hat = cv2.morphologyEx(
        gray.astype(np.uint8),
        cv2.MORPH_TOPHAT,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (31, 31)),
    ).astype(np.float32)
    ridge = np.clip(top_hat / 34.0, 0.0, 1.0)
    low_saturation = np.clip(
        (105.0 - hsv[:, :, 1].astype(np.float32)) / 75.0,
        0.0,
        1.0,
    )
    brightness = np.clip((gray - 90.0) / 100.0, 0.0, 1.0)
    structural = 0.62 * ridge + 0.38 * local_contrast
    score = (
        structural * (0.18 + 0.82 * low_saturation)
        + 0.08 * brightness * low_saturation
    )
    return cv2.GaussianBlur(score, (0, 0), 1.2)


def _smooth_path(points: np.ndarray) -> np.ndarray:
    if len(points) < 9:
        return points
    values = points.astype(np.float64)
    window = 11
    kernel = np.ones(window, np.float64) / float(window)
    pad = window // 2
    xs = np.convolve(np.pad(values[:, 0], (pad, pad), mode="edge"), kernel, mode="valid")
    ys = np.convolve(np.pad(values[:, 1], (pad, pad), mode="edge"), kernel, mode="valid")
    return np.round(np.column_stack((xs, ys))).astype(np.int32)


def trace_left_boundary(score: np.ndarray, config: WhiteLeftConfig) -> tuple[np.ndarray, float]:
    """Direction-aware dynamic programming from the near-left paint upward."""
    height, width = score.shape
    near_y = int(round(height * config.near_y_ratio))
    far_y = int(round(height * config.far_y_ratio))
    x_limit = max(8, int(round(width * config.left_search_ratio)))
    rows = near_y - far_y + 1
    direction_count = len(DELTAS)
    inf = np.float32(np.inf)
    observation = 8.0 * np.square(1.0 - np.clip(score, 0.0, 1.0))

    previous = np.full((x_limit, direction_count), inf, np.float32)
    x_values = np.arange(x_limit, dtype=np.float32)
    for direction_index, delta in enumerate(DELTAS):
        previous[:, direction_index] = (
            observation[near_y, :x_limit]
            + 0.010 * x_values
            + 0.08 * float((int(delta) - 2) ** 2)
        )

    back_direction = np.full(
        (rows, x_limit, direction_count),
        -1,
        dtype=np.int8,
    )
    direction_axis = np.arange(direction_count, dtype=np.float32)
    for row_index in range(1, rows):
        y_value = near_y - row_index
        current = np.full_like(previous, inf)
        for direction_index, delta_value in enumerate(DELTAS):
            delta = int(delta_value)
            x_start = max(0, delta)
            x_end = min(x_limit, x_limit + delta)
            if x_end <= x_start:
                continue
            current_x = np.arange(x_start, x_end, dtype=np.int32)
            previous_x = current_x - delta
            transition = (
                previous[previous_x, :]
                + 0.22 * np.square(direction_axis - float(direction_index))[None, :]
            )
            best_direction = np.argmin(transition, axis=1).astype(np.int8)
            best_cost = transition[np.arange(len(current_x)), best_direction]
            current[current_x, direction_index] = (
                best_cost + observation[y_value, current_x]
            )
            back_direction[row_index, current_x, direction_index] = best_direction
        previous = current

    end_flat = int(np.argmin(previous))
    x_value, direction = np.unravel_index(end_flat, previous.shape)
    points: list[tuple[int, int]] = []
    for row_index in range(rows - 1, -1, -1):
        y_value = near_y - row_index
        points.append((int(x_value), int(y_value)))
        if row_index == 0:
            break
        previous_direction = int(back_direction[row_index, x_value, direction])
        if previous_direction < 0:
            raise RuntimeError("WHITE_LEFT_PATH_LOST")
        x_value -= int(DELTAS[direction])
        direction = previous_direction
    path = _smooth_path(np.asarray(points[::-1], dtype=np.int32))
    path_score = float(np.mean(score[path[:, 1], path[:, 0]]))
    return path, path_score


def _x_at_y(path: np.ndarray, y_value: float) -> float:
    order = np.argsort(path[:, 1])
    ys = path[order, 1].astype(np.float64)
    xs = path[order, 0].astype(np.float64)
    return float(np.interp(y_value, ys, xs))


def detect_white_left_boundary(
    frame: np.ndarray,
    config: WhiteLeftConfig | None = None,
    render_debug: bool = False,
) -> WhiteLeftResult:
    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a BGR HxWx3 image")
    config = config or WhiteLeftConfig()
    started_at = perf_counter()
    small = cv2.resize(
        frame,
        (config.work_width, config.work_height),
        interpolation=cv2.INTER_AREA,
    )
    score = white_ridge_score(small)
    path_small, confidence = trace_left_boundary(score, config)

    scale_x = frame.shape[1] / float(config.work_width)
    scale_y = frame.shape[0] / float(config.work_height)
    path = path_small.astype(np.float32)
    path[:, 0] *= scale_x
    path[:, 1] *= scale_y
    path = np.round(path).astype(np.int32)

    control_y = int(round(frame.shape[0] * config.control_y_ratio))
    left_x = _x_at_y(path, control_y)
    desired_distance = frame.shape[1] * config.desired_left_distance_ratio
    target_center = left_x + desired_distance
    center_offset = target_center - frame.shape[1] * 0.5
    valid = confidence >= config.minimum_path_score

    debug_image = None
    if render_debug:
        debug_image = frame.copy()
        cv2.polylines(debug_image, [path], False, (0, 255, 0), 5, cv2.LINE_AA)

    processing_ms = (perf_counter() - started_at) * 1000.0
    return WhiteLeftResult(
        state="READY" if valid else "LOST",
        confidence=confidence,
        path=path if valid else None,
        control_y_px=control_y,
        left_boundary_x_px=left_x if valid else None,
        desired_left_distance_px=desired_distance,
        center_offset_px=center_offset if valid else None,
        processing_ms=processing_ms,
        debug_image=debug_image,
    )

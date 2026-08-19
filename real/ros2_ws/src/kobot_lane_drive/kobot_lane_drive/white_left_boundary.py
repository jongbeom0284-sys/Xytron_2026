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
    far_y_ratio: float = 0.625
    control_y_ratio: float = 0.70
    left_search_ratio: float = 0.52
    minimum_path_score: float = 0.30
    continuation_minimum_score: float = 0.25
    continuation_maximum_gap_rows: int = 5
    sharp_turn_window_rows: int = 5
    maximum_turn_angle_deg: float = 60.0
    edge_gradient_threshold: float = 25.0
    stripe_minimum_width_ratio: float = 4.0 / 640.0
    stripe_maximum_width_ratio: float = 28.0 / 640.0
    stripe_minimum_contrast: float = 7.0
    stripe_maximum_saturation: float = 115.0
    ransac_iterations: int = 1200
    ransac_residual_ratio: float = 7.0 / 640.0
    ransac_minimum_inliers: int = 12
    minimum_inward_progress_ratio: float = 12.0 / 640.0
    maximum_bottom_slope_error_ratio: float = 80.0 / 640.0
    bottom_extrapolation_ratio: float = 80.0 / 640.0
    # Approved frame 1 with this detector: center 320px - left line 76px.
    desired_left_distance_ratio: float = 244.0 / 640.0
    # When only one boundary is visible, bias the vehicle toward that boundary.
    single_lane_distance_ratio: float = 180.0 / 640.0


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
            "control_mode": "edge_pair_ransac_left_boundary",
            "metric_calibrated": False,
            "motor_output": False,
        }


@dataclass(frozen=True)
class WhiteRightResult:
    state: str
    confidence: float
    path: np.ndarray | None
    control_y_px: int
    right_boundary_x_px: float | None
    desired_right_distance_px: float
    center_offset_px: float | None
    processing_ms: float

    def status(self) -> dict[str, object]:
        return {
            "right_state": self.state,
            "right_confidence": round(self.confidence, 3),
            "right_boundary_x_px": (
                None
                if self.right_boundary_x_px is None
                else round(self.right_boundary_x_px, 2)
            ),
            "desired_right_distance_px": round(
                self.desired_right_distance_px,
                2,
            ),
            "right_center_offset_px": (
                None
                if self.center_offset_px is None
                else round(self.center_offset_px, 2)
            ),
            "right_processing_ms": round(self.processing_ms, 2),
        }


def select_lane_control(
    left: WhiteLeftResult,
    right: WhiteRightResult,
    single_lane_distance_px: float | None = None,
) -> tuple[str, float | None, float]:
    """Select a frame-local center offset from both, left, right, or neither."""
    left_ready = left.state == "READY" and left.center_offset_px is not None
    right_ready = right.state == "READY" and right.center_offset_px is not None
    if left_ready and right_ready:
        return (
            "BOTH",
            0.5 * (left.center_offset_px + right.center_offset_px),
            0.5 * (left.confidence + right.confidence),
        )
    if left_ready:
        distance = (
            left.desired_left_distance_px
            if single_lane_distance_px is None
            else single_lane_distance_px
        )
        offset = left.center_offset_px - (
            left.desired_left_distance_px - distance
        )
        return "LEFT", offset, left.confidence
    if right_ready:
        distance = (
            right.desired_right_distance_px
            if single_lane_distance_px is None
            else single_lane_distance_px
        )
        offset = right.center_offset_px + (
            right.desired_right_distance_px - distance
        )
        return "RIGHT", offset, right.confidence
    return "LOST", None, 0.0


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


def _trim_forced_continuation(
    path: np.ndarray,
    score: np.ndarray,
    config: WhiteLeftConfig,
) -> np.ndarray:
    """Stop at a lost continuation instead of joining an unrelated bright edge."""
    if len(path) < 3:
        return path

    control_y = int(round(score.shape[0] * config.control_y_ratio))
    control_indices = np.flatnonzero(path[:, 1] <= control_y)
    if not len(control_indices):
        return path
    continuation_start = int(control_indices[0])
    keep = len(path)

    point_scores = score[path[:, 1], path[:, 0]]
    low_run_start: int | None = None
    maximum_gap = max(1, int(config.continuation_maximum_gap_rows))
    for index in range(continuation_start, len(path)):
        if point_scores[index] < config.continuation_minimum_score:
            if low_run_start is None:
                low_run_start = index
            if index - low_run_start + 1 >= maximum_gap:
                keep = min(keep, low_run_start)
                break
        else:
            low_run_start = None

    # A control point must be observed rather than extrapolated from a short
    # fragment above it.
    minimum_keep = continuation_start + 1
    return path[:max(minimum_keep, keep)]


def _trim_sharp_turn(
    path: np.ndarray,
    config: WhiteLeftConfig,
) -> np.ndarray:
    """Discard a continuation that bends away too sharply from the near lane."""
    window = max(2, int(config.sharp_turn_window_rows))
    if len(path) < window * 2 + 1:
        return path

    control_y = int(round(config.work_height * config.control_y_ratio))
    control_indices = np.flatnonzero(path[:, 1] <= control_y)
    minimum_keep = int(control_indices[0]) + 1 if len(control_indices) else 1

    for center in range(window, len(path) - window):
        before = path[center].astype(np.float64) - path[center - window]
        after = path[center + window].astype(np.float64) - path[center]
        before_length = float(np.linalg.norm(before))
        after_length = float(np.linalg.norm(after))
        if before_length <= 0.0 or after_length <= 0.0:
            continue
        cosine = float(np.clip(
            np.dot(before, after) / (before_length * after_length),
            -1.0,
            1.0,
        ))
        turn_angle = float(np.degrees(np.arccos(cosine)))
        if turn_angle > config.maximum_turn_angle_deg:
            return path[:max(minimum_keep, center + 1)]
    return path


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
    path = _trim_forced_continuation(path, score, config)
    path = _trim_sharp_turn(path, config)
    path_score = float(np.mean(score[path[:, 1], path[:, 0]]))
    return path, path_score


def _x_at_y(path: np.ndarray, y_value: float) -> float:
    order = np.argsort(path[:, 1])
    ys = path[order, 1].astype(np.float64)
    xs = path[order, 0].astype(np.float64)
    return float(np.interp(y_value, ys, xs))


def _edge_pair_candidates(
    frame: np.ndarray,
    config: WhiteLeftConfig,
) -> tuple[np.ndarray, np.ndarray, int, int, int]:
    """Return locally bright stripe centers in the left ROI of this frame."""
    height, width = frame.shape[:2]
    far_y = int(round(height * config.far_y_ratio))
    near_y = int(round(height * config.near_y_ratio))
    x_limit = max(8, min(width, int(round(width * config.left_search_ratio))))
    row_step = max(1, int(round(height / 240.0)))
    minimum_width = max(2, int(round(width * config.stripe_minimum_width_ratio)))
    maximum_width = max(
        minimum_width + 1,
        int(round(width * config.stripe_maximum_width_ratio)),
    )

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    gradient_x = cv2.Sobel(blurred, cv2.CV_32F, 1, 0, ksize=3)
    candidates: list[tuple[float, float]] = []
    qualities: list[float] = []
    expected_width = float(np.clip(
        round(width * 12.0 / 640.0),
        minimum_width,
        maximum_width,
    ))

    for y_value in range(far_y, near_y + 1, row_step):
        row_gradient = gradient_x[y_value, :x_limit]
        positive = np.flatnonzero(
            row_gradient > config.edge_gradient_threshold
        )
        negative = np.flatnonzero(
            row_gradient < -config.edge_gradient_threshold
        )
        gray_row = gray[y_value, :x_limit]
        saturation_row = hsv[y_value, :x_limit, 1]
        for left_edge in positive:
            right_options = negative[
                (negative >= left_edge + minimum_width)
                & (negative <= left_edge + maximum_width)
            ]
            if not len(right_options):
                continue
            right_edge = int(
                right_options[
                    np.argmin(
                        np.abs(right_options - left_edge - expected_width)
                    )
                ]
            )
            inside = gray_row[left_edge + 1 : right_edge]
            if not len(inside):
                continue
            outside = np.concatenate((
                gray_row[max(0, left_edge - 8) : left_edge],
                gray_row[
                    right_edge + 1 : min(x_limit, right_edge + 9)
                ],
            ))
            if not len(outside):
                continue
            contrast = float(np.mean(inside) - np.mean(outside))
            saturation = float(
                np.mean(saturation_row[left_edge + 1 : right_edge])
            )
            if (
                contrast < config.stripe_minimum_contrast
                or saturation > config.stripe_maximum_saturation
            ):
                continue
            center_x = 0.5 * (float(left_edge) + float(right_edge))
            edge_strength = float(
                row_gradient[left_edge] - row_gradient[right_edge]
            )
            candidates.append((center_x, float(y_value)))
            qualities.append(contrast + 0.025 * edge_strength)

    return (
        np.asarray(candidates, dtype=np.float64).reshape((-1, 2)),
        np.asarray(qualities, dtype=np.float64),
        far_y,
        near_y,
        row_step,
    )


def _edge_pair_ransac_path(
    frame: np.ndarray,
    config: WhiteLeftConfig,
    seed: int = 42,
) -> tuple[np.ndarray | None, float]:
    """Fit one smooth boundary using bottom support and row connectivity."""
    points, weights, far_y, near_y, row_step = _edge_pair_candidates(
        frame,
        config,
    )
    if len(points) < 3 or near_y <= far_y:
        return None, 0.0

    height, width = frame.shape[:2]
    x_limit = max(8, min(width, int(round(width * config.left_search_ratio))))
    normalized_y = (points[:, 1] - near_y) / float(near_y - far_y)
    rng = np.random.default_rng(seed)
    samples = rng.integers(
        0,
        len(points),
        size=(max(1, int(config.ransac_iterations)), 3),
    )
    sample_y = normalized_y[samples]
    valid_samples = (
        (samples[:, 0] != samples[:, 1])
        & (samples[:, 0] != samples[:, 2])
        & (samples[:, 1] != samples[:, 2])
        & (sample_y[:, 0] != sample_y[:, 1])
        & (sample_y[:, 0] != sample_y[:, 2])
        & (sample_y[:, 1] != sample_y[:, 2])
        & (np.ptp(sample_y, axis=1) >= 0.35)
    )
    samples = samples[valid_samples]
    sample_y = sample_y[valid_samples]
    if not len(samples):
        return None, 0.0

    systems = np.stack(
        (np.square(sample_y), sample_y, np.ones_like(sample_y)),
        axis=2,
    )
    try:
        models = np.linalg.solve(
            systems,
            points[samples, 0, None],
        )[:, :, 0]
    except np.linalg.LinAlgError:
        return None, 0.0

    control_t = (
        height * config.control_y_ratio - near_y
    ) / float(near_y - far_y)
    scale_x = width / 640.0
    bottom_x = models[:, 2]
    top_x = models[:, 0] - models[:, 1] + models[:, 2]
    control_x = (
        models[:, 0] * control_t * control_t
        + models[:, 1] * control_t
        + models[:, 2]
    )
    derivative_bottom = models[:, 1]
    derivative_top = -2.0 * models[:, 0] + models[:, 1]
    bottom_heading = np.arctan2(-derivative_bottom, near_y - far_y)
    top_heading = np.arctan2(-derivative_top, near_y - far_y)
    heading_delta = np.arctan2(
        np.sin(top_heading - bottom_heading),
        np.cos(top_heading - bottom_heading),
    )
    turn_angle_deg = np.abs(np.degrees(heading_delta))
    valid_geometry = (
        (bottom_x >= -width * config.bottom_extrapolation_ratio)
        & (bottom_x <= 150.0 * scale_x)
        & (control_x >= 20.0 * scale_x)
        & (control_x <= 260.0 * scale_x)
        & (top_x >= 0.0)
        & (top_x < x_limit)
        & (top_x - bottom_x >= width * config.minimum_inward_progress_ratio)
        & (derivative_bottom >= -360.0 * scale_x)
        & (derivative_bottom <= 60.0 * scale_x)
        & (derivative_top >= -360.0 * scale_x)
        & (derivative_top <= 60.0 * scale_x)
        & (turn_angle_deg <= config.maximum_turn_angle_deg)
    )
    models = models[valid_geometry]
    if not len(models):
        return None, 0.0

    predicted = (
        models[:, 0, None] * np.square(normalized_y)[None, :]
        + models[:, 1, None] * normalized_y[None, :]
        + models[:, 2, None]
    )
    residual = np.abs(points[None, :, 0] - predicted)
    inliers = residual <= width * config.ransac_residual_ratio
    inlier_count = np.count_nonzero(inliers, axis=1)
    valid_inliers = inlier_count >= max(3, int(config.ransac_minimum_inliers))
    models = models[valid_inliers]
    residual = residual[valid_inliers]
    inliers = inliers[valid_inliers]
    inlier_count = inlier_count[valid_inliers]
    if not len(models):
        return None, 0.0

    unique_rows = np.unique(points[:, 1])
    row_coverage = np.zeros(len(models), dtype=np.float64)
    for y_value in unique_rows:
        row_coverage += np.any(inliers[:, points[:, 1] == y_value], axis=1)
    maximum_y = np.max(
        np.where(inliers, points[None, :, 1], -np.inf),
        axis=1,
    )
    minimum_y = np.min(
        np.where(inliers, points[None, :, 1], np.inf),
        axis=1,
    )
    span = maximum_y - minimum_y
    bottom_support = np.count_nonzero(
        inliers
        & (
            points[None, :, 1]
            >= maximum_y[:, None] - 14.0 * height / 480.0
        ),
        axis=1,
    )
    bottom_mask = inliers & (
        points[None, :, 1]
        >= maximum_y[:, None] - 14.0 * height / 480.0
    )
    bottom_count = np.maximum(1, np.count_nonzero(bottom_mask, axis=1))
    bottom_mean_y = np.sum(
        bottom_mask * normalized_y[None, :], axis=1
    ) / bottom_count
    bottom_mean_x = np.sum(
        bottom_mask * points[None, :, 0], axis=1
    ) / bottom_count
    centered_y = normalized_y[None, :] - bottom_mean_y[:, None]
    centered_x = points[None, :, 0] - bottom_mean_x[:, None]
    bottom_denominator = np.sum(
        bottom_mask * np.square(centered_y), axis=1
    )
    bottom_slope = np.sum(
        bottom_mask * centered_y * centered_x, axis=1
    ) / np.maximum(bottom_denominator, 1e-9)
    bottom_slope_error = np.abs(models[:, 1] - bottom_slope)
    quality_sum = np.sum(
        inliers * np.minimum(weights[None, :], 40.0),
        axis=1,
    )
    mean_residual = np.sum(residual * inliers, axis=1) / inlier_count
    valid_bottom = (
        (bottom_support >= 3)
        & (bottom_denominator > 1e-9)
        & (
            bottom_slope_error
            <= width * config.maximum_bottom_slope_error_ratio
        )
    )
    models = models[valid_bottom]
    residual = residual[valid_bottom]
    inliers = inliers[valid_bottom]
    inlier_count = inlier_count[valid_bottom]
    row_coverage = row_coverage[valid_bottom]
    span = span[valid_bottom]
    bottom_support = bottom_support[valid_bottom]
    bottom_slope_error = bottom_slope_error[valid_bottom]
    quality_sum = quality_sum[valid_bottom]
    mean_residual = mean_residual[valid_bottom]
    if not len(models):
        return None, 0.0
    score = (
        row_coverage * 12.0
        + span * 3.0
        + bottom_support * 10.0
        + quality_sum
        - mean_residual * 8.0
        - bottom_slope_error * 2.0
    )
    selected_index = None
    model = None
    for candidate_index in np.argsort(score)[::-1]:
        candidate_inliers = inliers[candidate_index]
        refined_y = normalized_y[candidate_inliers]
        refined_x = points[candidate_inliers, 0]
        refined_weights = np.maximum(1.0, weights[candidate_inliers])
        candidate_model = np.polyfit(
            refined_y,
            refined_x,
            2,
            w=np.sqrt(refined_weights),
        )
        refined_bottom_x = float(np.polyval(candidate_model, 0.0))
        refined_top_x = float(np.polyval(candidate_model, -1.0))
        refined_control_x = float(np.polyval(candidate_model, control_t))
        refined_bottom_derivative = float(candidate_model[1])
        refined_top_derivative = float(-2.0 * candidate_model[0] + candidate_model[1])
        refined_bottom_heading = np.arctan2(
            -refined_bottom_derivative,
            near_y - far_y,
        )
        refined_top_heading = np.arctan2(
            -refined_top_derivative,
            near_y - far_y,
        )
        refined_turn = abs(float(np.degrees(np.arctan2(
            np.sin(refined_top_heading - refined_bottom_heading),
            np.cos(refined_top_heading - refined_bottom_heading),
        ))))
        if not (
            -width * config.bottom_extrapolation_ratio
            <= refined_bottom_x
            <= 150.0 * scale_x
            and 20.0 * scale_x <= refined_control_x <= 260.0 * scale_x
            and 0.0 <= refined_top_x < x_limit
            and refined_top_x - refined_bottom_x
            >= width * config.minimum_inward_progress_ratio
            and -360.0 * scale_x <= refined_bottom_derivative <= 60.0 * scale_x
            and -360.0 * scale_x <= refined_top_derivative <= 60.0 * scale_x
            and refined_turn <= config.maximum_turn_angle_deg
        ):
            continue
        selected_index = int(candidate_index)
        model = candidate_model
        break
    if selected_index is None or model is None:
        return None, 0.0

    y_values = np.arange(near_y, far_y - 1, -row_step, dtype=np.float64)
    t_values = (y_values - near_y) / float(near_y - far_y)
    x_values = np.polyval(model, t_values)
    valid_points = (x_values >= 0.0) & (x_values < x_limit)
    path = np.round(
        np.column_stack((x_values[valid_points], y_values[valid_points]))
    ).astype(np.int32)
    if len(path) < 3:
        return None, 0.0
    possible_rows = max(1, len(range(far_y, near_y + 1, row_step)))
    confidence = float(
        np.clip(row_coverage[selected_index] / possible_rows, 0.0, 1.0)
    )
    return path, confidence


def detect_white_left_boundary(
    frame: np.ndarray,
    config: WhiteLeftConfig | None = None,
    render_debug: bool = False,
    _ransac_seed: int = 42,
) -> WhiteLeftResult:
    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a BGR HxWx3 image")
    config = config or WhiteLeftConfig()
    started_at = perf_counter()
    path, confidence = _edge_pair_ransac_path(frame, config, seed=_ransac_seed)

    control_y = int(round(frame.shape[0] * config.control_y_ratio))
    left_x = None if path is None else _x_at_y(path, control_y)
    desired_distance = frame.shape[1] * config.desired_left_distance_ratio
    center_offset = (
        None
        if left_x is None
        else left_x + desired_distance - frame.shape[1] * 0.5
    )
    valid = (
        path is not None
        and left_x is not None
        and confidence >= config.minimum_path_score
    )

    debug_image = None
    if render_debug:
        debug_image = frame.copy()
        if valid:
            cv2.polylines(
                debug_image,
                [path],
                False,
                (0, 255, 0),
                5,
                cv2.LINE_AA,
            )

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


def detect_white_right_boundary(
    frame: np.ndarray,
    config: WhiteLeftConfig | None = None,
) -> WhiteRightResult:
    """Run the approved left detector on a mirrored frame for the right ROI."""
    if not isinstance(frame, np.ndarray) or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError("frame must be a BGR HxWx3 image")
    config = config or WhiteLeftConfig()
    started_at = perf_counter()
    mirrored = np.ascontiguousarray(frame[:, ::-1])
    mirrored_result = detect_white_left_boundary(
        mirrored,
        config=config,
        render_debug=False,
        _ransac_seed=42,
    )
    width = frame.shape[1]
    path = None
    if mirrored_result.path is not None:
        path = mirrored_result.path.copy()
        path[:, 0] = width - 1 - path[:, 0]
    right_x = (
        None
        if mirrored_result.left_boundary_x_px is None
        else width - 1 - mirrored_result.left_boundary_x_px
    )
    desired_distance = mirrored_result.desired_left_distance_px
    center_offset = (
        None
        if right_x is None
        else right_x - desired_distance - width * 0.5
    )
    return WhiteRightResult(
        state=mirrored_result.state,
        confidence=mirrored_result.confidence,
        path=path,
        control_y_px=mirrored_result.control_y_px,
        right_boundary_x_px=right_x,
        desired_right_distance_px=desired_distance,
        center_offset_px=center_offset,
        processing_ms=(perf_counter() - started_at) * 1000.0,
    )


def render_dual_roi_overlay(
    frame: np.ndarray,
    config: WhiteLeftConfig,
    left: WhiteLeftResult,
    right: WhiteRightResult,
) -> np.ndarray:
    """Draw symmetric ROIs, active d references, and current boundary paths."""
    output = frame.copy()
    height, width = output.shape[:2]
    far_y = int(round(height * config.far_y_ratio))
    near_y = int(round(height * config.near_y_ratio))
    left_limit = int(round(width * config.left_search_ratio))
    right_limit = width - left_limit
    center_x = width // 2
    control_y = left.control_y_px
    both_ready = left.state == "READY" and right.state == "READY"
    single_distance = int(round(width * config.single_lane_distance_ratio))
    left_distance = (
        int(round(left.desired_left_distance_px))
        if both_ready or left.state != "READY"
        else single_distance
    )
    right_distance = (
        int(round(right.desired_right_distance_px))
        if both_ready or right.state != "READY"
        else single_distance
    )
    target_left_x = center_x - left_distance
    target_right_x = center_x + right_distance

    cv2.rectangle(output, (0, far_y), (left_limit, near_y), (255, 170, 0), 2)
    cv2.rectangle(
        output,
        (right_limit, far_y),
        (width - 1, near_y),
        (0, 170, 255),
        2,
    )
    cv2.putText(
        output,
        "LEFT ROI",
        (12, far_y + 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 170, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        "RIGHT ROI",
        (max(right_limit + 12, width - 155), far_y + 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 170, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.line(output, (center_x, far_y), (center_x, near_y), (255, 255, 255), 2)
    cv2.line(
        output,
        (target_left_x, control_y),
        (target_right_x, control_y),
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    cv2.circle(output, (target_left_x, control_y), 7, (255, 170, 0), -1)
    cv2.circle(output, (target_right_x, control_y), 7, (0, 170, 255), -1)
    cv2.putText(
        output,
        f"d={left_distance}px",
        (center_x - left_distance // 2 - 36, control_y - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        output,
        f"d={right_distance}px",
        (center_x + right_distance // 2 - 36, control_y - 12),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if left.path is not None:
        cv2.polylines(output, [left.path], False, (0, 255, 0), 5, cv2.LINE_AA)
    if right.path is not None:
        cv2.polylines(output, [right.path], False, (255, 0, 255), 5, cv2.LINE_AA)
    if left.left_boundary_x_px is not None:
        cv2.circle(
            output,
            (int(round(left.left_boundary_x_px)), control_y),
            8,
            (0, 255, 0),
            -1,
        )
    if right.right_boundary_x_px is not None:
        cv2.circle(
            output,
            (int(round(right.right_boundary_x_px)), control_y),
            8,
            (255, 0, 255),
            -1,
        )
    return output

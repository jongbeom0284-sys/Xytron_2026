from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np

from .lane_config import LaneVisionConfig


@dataclass
class PolynomialFit:
    coeffs: tuple[float, float, float]
    confidence: float
    inlier_mask: np.ndarray
    rmse_m: float
    point_count: int
    inlier_count: int
    x_min: float
    x_max: float

    def y(self, x: float | np.ndarray) -> float | np.ndarray:
        a, b, c = self.coeffs
        return a * np.asarray(x) ** 2 + b * np.asarray(x) + c

    def dy_dx(self, x: float) -> float:
        a, b, _c = self.coeffs
        return 2.0 * a * x + b

    def shifted(self, dy: float, confidence_scale: float = 0.55) -> "PolynomialFit":
        a, b, c = self.coeffs
        return PolynomialFit(
            coeffs=(a, b, c + dy),
            confidence=max(0.0, min(1.0, self.confidence * confidence_scale)),
            inlier_mask=self.inlier_mask.copy(),
            rmse_m=self.rmse_m,
            point_count=self.point_count,
            inlier_count=self.inlier_count,
            x_min=self.x_min,
            x_max=self.x_max,
        )


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def finite_float(value: object, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(number):
        return default
    return number


def polyfit_quadratic(xs: np.ndarray, ys: np.ndarray) -> tuple[float, float, float] | None:
    if xs.size < 3 or ys.size < 3:
        return None
    try:
        coeffs = np.polyfit(xs, ys, 2)
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return None
    if not np.all(np.isfinite(coeffs)):
        return None
    return float(coeffs[0]), float(coeffs[1]), float(coeffs[2])


def fit_quadratic_ransac(
    points: Iterable[tuple[float, float]],
    cfg: LaneVisionConfig,
    seed: int = 17,
) -> PolynomialFit | None:
    data = np.asarray(list(points), dtype=np.float64)
    if data.ndim != 2 or data.shape[0] < cfg.fit_min_points or data.shape[1] != 2:
        return None

    xs = data[:, 0]
    ys = data[:, 1]
    valid = (
        np.isfinite(xs)
        & np.isfinite(ys)
        & (xs >= cfg.fit_min_x_m)
        & (xs <= cfg.fit_max_x_m)
    )
    xs = xs[valid]
    ys = ys[valid]
    n = xs.size
    if n < cfg.fit_min_points:
        return None

    rng = np.random.default_rng(seed)
    best_mask: np.ndarray | None = None
    best_score = -1.0
    best_rmse = float("inf")

    sample_size = 3
    for _ in range(max(1, cfg.ransac_iterations)):
        idx = rng.choice(n, sample_size, replace=False)
        coeffs = polyfit_quadratic(xs[idx], ys[idx])
        if coeffs is None:
            continue
        a, b, c = coeffs
        pred = a * xs * xs + b * xs + c
        residual = np.abs(pred - ys)
        mask = residual <= cfg.ransac_residual_m
        inliers = int(np.count_nonzero(mask))
        if inliers < max(3, int(math.ceil(cfg.ransac_min_inlier_ratio * n))):
            continue
        rmse = float(np.sqrt(np.mean((pred[mask] - ys[mask]) ** 2)))
        coverage = float(np.max(xs[mask]) - np.min(xs[mask])) if inliers else 0.0
        score = inliers + 0.4 * coverage - 10.0 * rmse
        if score > best_score or (math.isclose(score, best_score) and rmse < best_rmse):
            best_score = score
            best_rmse = rmse
            best_mask = mask

    if best_mask is None:
        coeffs = polyfit_quadratic(xs, ys)
        if coeffs is None:
            return None
        pred = coeffs[0] * xs * xs + coeffs[1] * xs + coeffs[2]
        residual = np.abs(pred - ys)
        best_mask = residual <= max(cfg.ransac_residual_m * 1.5, cfg.max_fit_rmse_m)

    if int(np.count_nonzero(best_mask)) < cfg.fit_min_points:
        return None

    coeffs = polyfit_quadratic(xs[best_mask], ys[best_mask])
    if coeffs is None:
        return None
    pred = coeffs[0] * xs[best_mask] ** 2 + coeffs[1] * xs[best_mask] + coeffs[2]
    rmse = float(np.sqrt(np.mean((pred - ys[best_mask]) ** 2)))
    if rmse > cfg.max_fit_rmse_m:
        return None

    inlier_count = int(np.count_nonzero(best_mask))
    inlier_ratio = inlier_count / max(1, n)
    x_min = float(np.min(xs[best_mask]))
    x_max = float(np.max(xs[best_mask]))
    coverage_ratio = clamp((x_max - x_min) / max(1e-6, cfg.fit_max_x_m - cfg.fit_min_x_m), 0.0, 1.0)
    rmse_score = clamp(1.0 - rmse / max(1e-6, cfg.max_fit_rmse_m), 0.0, 1.0)
    confidence = clamp(0.52 * inlier_ratio + 0.33 * coverage_ratio + 0.15 * rmse_score, 0.0, 1.0)

    full_mask = np.zeros(valid.shape[0], dtype=bool)
    valid_indices = np.flatnonzero(valid)
    full_mask[valid_indices[best_mask]] = True
    return PolynomialFit(
        coeffs=coeffs,
        confidence=confidence,
        inlier_mask=full_mask,
        rmse_m=rmse,
        point_count=int(data.shape[0]),
        inlier_count=inlier_count,
        x_min=x_min,
        x_max=x_max,
    )


def unwrap_angle(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad

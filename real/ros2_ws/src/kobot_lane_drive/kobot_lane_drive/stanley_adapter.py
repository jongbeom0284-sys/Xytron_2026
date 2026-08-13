from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class SteeringPreview:
    steering_angle_rad: float
    target_index: int


def _load_pythonrobotics(vendor_root: str | Path):
    root = str(Path(vendor_root).resolve())
    if root not in sys.path:
        sys.path.insert(0, root)
    from PathPlanning.CubicSpline import cubic_spline_planner
    from PathTracking.stanley_control import stanley_control

    return cubic_spline_planner, stanley_control


def calculate_stanley_preview(
    target_path_xy: np.ndarray,
    speed_mps: float,
    vendor_root: str | Path,
    wheelbase_m: float = 0.24,
    gain: float = 0.5,
    sample_distance_m: float = 0.05,
    maximum_steering_rad: float = 0.34,
) -> SteeringPreview:
    """Run the pinned PythonRobotics Stanley controller for the vehicle origin."""
    path = np.asarray(target_path_xy, dtype=np.float64)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        raise ValueError("target_path_xy must contain at least two XY points")
    if wheelbase_m <= 0.0 or sample_distance_m <= 0.0:
        raise ValueError("wheelbase and sample distance must be positive")
    if maximum_steering_rad <= 0.0:
        raise ValueError("maximum_steering_rad must be positive")

    planner, controller = _load_pythonrobotics(vendor_root)
    controller.L = float(wheelbase_m)
    controller.k = float(gain)
    path = path[np.argsort(path[:, 0])]
    cx, cy, cyaw, _, _ = planner.calc_spline_course(
        path[:, 0].tolist(), path[:, 1].tolist(), ds=sample_distance_m
    )
    if len(cx) < 2:
        raise ValueError("PythonRobotics spline is too short")
    state = controller.State(x=0.0, y=0.0, yaw=0.0, v=max(float(speed_mps), 0.05))
    steering, index = controller.stanley_control(state, cx, cy, cyaw, 0)
    steering = float(np.clip(steering, -maximum_steering_rad, maximum_steering_rad))
    return SteeringPreview(steering_angle_rad=steering, target_index=int(index))

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from shapely import LineString, Point, distance, offset_curve


@dataclass(frozen=True)
class MetricPath:
    left_lane_xy: np.ndarray
    target_path_xy: np.ndarray
    measured_distance_m: float
    lateral_error_m: float


def validate_camera_matrix(camera_matrix: np.ndarray) -> None:
    matrix = np.asarray(camera_matrix, dtype=np.float64).reshape((3, 3))
    if not np.isfinite(matrix).all():
        raise ValueError("camera matrix contains non-finite values")
    if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
        raise ValueError("camera focal lengths must be positive")
    if abs(float(np.linalg.det(matrix))) < 1e-9:
        raise ValueError("camera matrix is singular")


def offset_left_lane(
    ground_points_xyz: np.ndarray,
    target_distance_m: float,
    minimum_forward_m: float = 0.15,
    maximum_forward_m: float = 4.0,
) -> MetricPath:
    """Create a metric target path using Shapely's open-source offset_curve.

    Coordinates follow ROS base_link convention: x forward, y left.  The target
    vehicle path lies to the right of the left boundary, therefore the offset is
    negative.  No custom BEV or parallel-curve formula is implemented here.
    """
    if target_distance_m <= 0.0:
        raise ValueError("target_distance_m must be positive")
    points = np.asarray(ground_points_xyz, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 2:
        raise ValueError("ground_points_xyz must have shape Nx2 or Nx3")
    xy = points[:, :2]
    xy = xy[np.isfinite(xy).all(axis=1)]
    xy = xy[(xy[:, 0] >= minimum_forward_m) & (xy[:, 0] <= maximum_forward_m)]
    if len(xy) < 2:
        raise ValueError("not enough valid ground-plane lane points")
    xy = xy[np.argsort(xy[:, 0])]
    xy = xy[np.concatenate(([True], np.linalg.norm(np.diff(xy, axis=0), axis=1) > 1e-4))]
    if len(xy) < 2:
        raise ValueError("ground-plane lane points collapse to one position")

    left_line = LineString(xy)
    result = offset_curve(left_line, -target_distance_m, join_style="round")
    if result.is_empty or result.geom_type != "LineString":
        raise ValueError("Shapely could not create a single target path")
    target_xy = np.asarray(result.coords, dtype=np.float64)
    if len(target_xy) < 2:
        raise ValueError("target path is too short")
    if target_xy[0, 0] > target_xy[-1, 0]:
        target_xy = target_xy[::-1].copy()

    measured_distance_m = float(distance(Point(0.0, 0.0), left_line))
    lateral_error_m = measured_distance_m - target_distance_m
    return MetricPath(
        left_lane_xy=xy,
        target_path_xy=target_xy,
        measured_distance_m=measured_distance_m,
        lateral_error_m=lateral_error_m,
    )

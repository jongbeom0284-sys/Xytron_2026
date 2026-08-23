"""Vehicle-centre correction for the integrated first-order cone path.

The LiDAR sits near the front of the 0.60 m long vehicle.  The tested profile
uses a tuned 0.15 m longitudinal correction (half the initial 0.30 m physical
estimate), while retaining the existing wall association and linear fit.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

from . import cone_wall


Point = Tuple[float, float]


@dataclass(frozen=True)
class CenterOffsetResult:
    aim_y_m: float
    basis: str
    target_x_from_lidar_m: float
    target_x_from_center_m: float
    maximum_center_offset_m: float
    left_count: int
    right_count: int


def _finite_points(points: Iterable[Sequence[float]]) -> list[Point]:
    result = []
    for point in points:
        if len(point) < 2:
            continue
        x, y = float(point[0]), float(point[1])
        if math.isfinite(x) and math.isfinite(y):
            result.append((x, y))
    return result


def calculate_center_offset_target(
    points: Iterable[Sequence[float]],
    *,
    vehicle_length_m: float = 0.60,
    vehicle_width_m: float = 0.30,
    lidar_to_vehicle_center_m: Optional[float] = 0.15,
    lookahead_from_center_m: float = 0.55,
    boundary_clearance_m: float = 0.02,
) -> Optional[CenterOffsetResult]:
    """Return the existing linear-wall target in the vehicle-centre frame.

    The parameters match the standalone test that was validated on the car.
    Passing ``None`` for the LiDAR offset retains a physical half-length
    fallback, but the integrated default is the tuned 0.15 m value.
    """

    values = (vehicle_length_m, vehicle_width_m, lookahead_from_center_m,
              boundary_clearance_m)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('vehicle geometry must be finite')
    if vehicle_length_m <= 0.0 or vehicle_width_m <= 0.0:
        raise ValueError('vehicle dimensions must be positive')
    if lookahead_from_center_m <= 0.0 or boundary_clearance_m < 0.0:
        raise ValueError('lookahead must be positive and clearance non-negative')

    center_offset = (vehicle_length_m / 2.0
                     if lidar_to_vehicle_center_m is None
                     else float(lidar_to_vehicle_center_m))
    if not math.isfinite(center_offset) or center_offset < 0.0:
        raise ValueError('lidar_to_vehicle_center_m must be finite/non-negative')

    cones = _finite_points(points)
    left, right, _unused = cone_wall.build_walls(cones)
    line = cone_wall.centre_line(left, right)
    if line is None:
        return None

    slope, intercept, basis = line
    target_x_lidar = max(0.05, lookahead_from_center_m - center_offset)
    aim_y = slope * target_x_lidar + intercept

    corridor_half = cone_wall.TRACK_W / 2.0
    maximum_center_offset = max(
        0.0, corridor_half - vehicle_width_m / 2.0 - boundary_clearance_m)
    aim_y = max(-maximum_center_offset, min(maximum_center_offset, aim_y))

    return CenterOffsetResult(
        aim_y_m=float(aim_y),
        basis=f'CENTER_OFFSET_{basis}',
        target_x_from_lidar_m=float(target_x_lidar),
        target_x_from_center_m=float(lookahead_from_center_m),
        maximum_center_offset_m=float(maximum_center_offset),
        left_count=len(left),
        right_count=len(right),
    )

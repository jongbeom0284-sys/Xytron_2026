from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ._vendor.simple_pid import PID


@dataclass(frozen=True)
class PixelLaneReference:
    """Fixed-camera lane locations captured at the desired vehicle position."""

    near_y_ratio: float
    far_y_ratio: float
    near_x_ratio: float
    far_x_ratio: float

    def __post_init__(self) -> None:
        if not 0.0 < self.far_y_ratio < self.near_y_ratio < 1.0:
            raise ValueError("reference rows must satisfy 0 < far_y < near_y < 1")
        for name, value in (
            ("near_x_ratio", self.near_x_ratio),
            ("far_x_ratio", self.far_x_ratio),
        ):
            if not 0.05 <= value <= 0.95:
                raise ValueError(f"{name} must be in [0.05, 0.95]")


@dataclass(frozen=True)
class PixelLaneMeasurement:
    near_x_px: float
    near_y_px: float
    far_x_px: float
    far_y_px: float
    near_x_ratio: float
    far_x_ratio: float
    near_error_ratio: float
    far_error_ratio: float


@dataclass(frozen=True)
class ImageSteeringPreview:
    measurement: PixelLaneMeasurement
    near_pid_rad: float
    far_pid_rad: float
    steering_angle_rad: float


def _valid_lane_points(lane_points: np.ndarray) -> np.ndarray:
    points = np.asarray(lane_points, dtype=np.float32).reshape((-1, 2))
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 3:
        raise RuntimeError("LEFT_LANE_TOO_SHORT")
    points = points[np.argsort(points[:, 1])]
    _, unique_indices = np.unique(points[:, 1], return_index=True)
    points = points[np.sort(unique_indices)]
    if len(points) < 3:
        raise RuntimeError("LEFT_LANE_TOO_SHORT")
    return points


def _sample_x(points: np.ndarray, y_px: float, label: str) -> float:
    minimum_y = float(points[0, 1])
    maximum_y = float(points[-1, 1])
    if not minimum_y <= y_px <= maximum_y:
        raise RuntimeError(
            f"{label}_SAMPLE_UNAVAILABLE: y={y_px:.1f}, lane=[{minimum_y:.1f}, {maximum_y:.1f}]"
        )
    return float(np.interp(y_px, points[:, 1], points[:, 0]))


def measure_lane_against_reference(
    lane_points: np.ndarray,
    image_width: int,
    image_height: int,
    reference: PixelLaneReference,
) -> PixelLaneMeasurement:
    """Sample the detected lane at the two fixed rows used by public followers."""
    if image_width <= 1 or image_height <= 1:
        raise ValueError("image dimensions must be greater than one")
    points = _valid_lane_points(lane_points)
    near_y_px = reference.near_y_ratio * float(image_height - 1)
    far_y_px = reference.far_y_ratio * float(image_height - 1)
    near_x_px = _sample_x(points, near_y_px, "NEAR")
    far_x_px = _sample_x(points, far_y_px, "FAR")
    near_x_ratio = near_x_px / float(image_width - 1)
    far_x_ratio = far_x_px / float(image_width - 1)
    if not 0.0 <= near_x_ratio <= 1.0 or not 0.0 <= far_x_ratio <= 1.0:
        raise RuntimeError("LEFT_LANE_SAMPLE_OUTSIDE_IMAGE")
    return PixelLaneMeasurement(
        near_x_px=near_x_px,
        near_y_px=near_y_px,
        far_x_px=far_x_px,
        far_y_px=far_y_px,
        near_x_ratio=near_x_ratio,
        far_x_ratio=far_x_ratio,
        near_error_ratio=reference.near_x_ratio - near_x_ratio,
        far_error_ratio=reference.far_x_ratio - far_x_ratio,
    )


def capture_pixel_reference(
    lane_points: np.ndarray,
    image_width: int,
    image_height: int,
    near_y_ratio: float = 0.85,
    far_y_ratio: float = 0.60,
) -> PixelLaneReference:
    """Capture target x ratios from one correctly positioned reference image."""
    provisional = PixelLaneReference(
        near_y_ratio=near_y_ratio,
        far_y_ratio=far_y_ratio,
        near_x_ratio=0.5,
        far_x_ratio=0.5,
    )
    measurement = measure_lane_against_reference(
        lane_points, image_width, image_height, provisional
    )
    return PixelLaneReference(
        near_y_ratio=near_y_ratio,
        far_y_ratio=far_y_ratio,
        near_x_ratio=measurement.near_x_ratio,
        far_x_ratio=measurement.far_x_ratio,
    )


class ImageReferenceController:
    """DonkeyCar-style target-pixel control with near/far simple-pid loops."""

    def __init__(
        self,
        reference: PixelLaneReference,
        near_tunings: tuple[float, float, float] = (1.0, 0.0, 0.0),
        far_tunings: tuple[float, float, float] = (0.65, 0.0, 0.0),
        maximum_steering_rad: float = 0.34,
    ) -> None:
        if maximum_steering_rad <= 0.0:
            raise ValueError("maximum_steering_rad must be positive")
        self.reference = reference
        self.maximum_steering_rad = float(maximum_steering_rad)
        limits = (-self.maximum_steering_rad, self.maximum_steering_rad)
        self._near_pid = PID(
            *near_tunings,
            setpoint=reference.near_x_ratio,
            sample_time=None,
            output_limits=limits,
        )
        self._far_pid = PID(
            *far_tunings,
            setpoint=reference.far_x_ratio,
            sample_time=None,
            output_limits=limits,
        )

    def reset(self) -> None:
        self._near_pid.reset()
        self._far_pid.reset()

    def calculate(
        self,
        lane_points: np.ndarray,
        image_width: int,
        image_height: int,
        dt: float | None = None,
    ) -> ImageSteeringPreview:
        measurement = measure_lane_against_reference(
            lane_points, image_width, image_height, self.reference
        )
        near_pid_rad = float(self._near_pid(measurement.near_x_ratio, dt=dt))
        far_pid_rad = float(self._far_pid(measurement.far_x_ratio, dt=dt))
        steering = float(
            np.clip(
                near_pid_rad + far_pid_rad,
                -self.maximum_steering_rad,
                self.maximum_steering_rad,
            )
        )
        return ImageSteeringPreview(
            measurement=measurement,
            near_pid_rad=near_pid_rad,
            far_pid_rad=far_pid_rad,
            steering_angle_rad=steering,
        )


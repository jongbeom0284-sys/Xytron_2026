from __future__ import annotations

import cv2
import numpy as np

from .image_reference_controller import ImageSteeringPreview
from .lane_selector import LeftLaneSelection


def draw_perception_overlay(
    source: np.ndarray,
    lane: LeftLaneSelection | None,
    preview: ImageSteeringPreview | None,
    elapsed_ms: float,
    error: str | None,
) -> np.ndarray:
    output = source.copy()
    if lane is not None and lane.points is not None:
        points = np.round(lane.points).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(output, [points], False, (0, 0, 0), 9, cv2.LINE_AA)
        cv2.polylines(output, [points], False, (255, 255, 255), 4, cv2.LINE_AA)

    if preview is not None:
        measurement = preview.measurement
        near_point = (int(round(measurement.near_x_px)), int(round(measurement.near_y_px)))
        far_point = (int(round(measurement.far_x_px)), int(round(measurement.far_y_px)))
        target_near = (
            int(
                round(
                    (measurement.near_x_ratio + measurement.near_error_ratio)
                    * (source.shape[1] - 1)
                )
            ),
            near_point[1],
        )
        target_far = (
            int(
                round(
                    (measurement.far_x_ratio + measurement.far_error_ratio)
                    * (source.shape[1] - 1)
                )
            ),
            far_point[1],
        )
        cv2.line(output, target_far, target_near, (255, 0, 255), 2, cv2.LINE_AA)
        cv2.drawMarker(
            output, target_near, (255, 0, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA
        )
        cv2.drawMarker(
            output, target_far, (255, 0, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA
        )
        cv2.circle(output, near_point, 8, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(output, far_point, 8, (255, 200, 0), -1, cv2.LINE_AA)
        cv2.line(output, far_point, near_point, (0, 180, 255), 2, cv2.LINE_AA)

    state = "READY" if error is None else "BLOCKED"
    color = (40, 210, 40) if error is None else (0, 80, 255)
    cv2.rectangle(output, (0, 0), (source.shape[1], 87), (0, 0, 0), -1)
    cv2.putText(
        output,
        f"LEFT LANE {state} | CLRerNet {elapsed_ms:.1f} ms",
        (8, 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        color,
        1,
        cv2.LINE_AA,
    )
    pixel_text = "pixel reference unavailable"
    if preview is not None:
        measurement = preview.measurement
        pixel_text = (
            f"near={measurement.near_x_ratio:.4f} e={measurement.near_error_ratio:+.4f} "
            f"far={measurement.far_x_ratio:.4f} e={measurement.far_error_ratio:+.4f}"
        )
    cv2.putText(
        output,
        pixel_text,
        (8, 42),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    steer_text = "steering preview unavailable"
    if preview is not None:
        steer_text = (
            f"simple-pid preview={np.degrees(preview.steering_angle_rad):+.2f} deg "
            f"(near={preview.near_pid_rad:+.3f}, far={preview.far_pid_rad:+.3f})"
        )
    cv2.putText(
        output,
        steer_text,
        (8, 63),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    footer = error[:72] if error else "OUTPUT GENERATED - ACCURACY UNVERIFIED"
    cv2.putText(
        output,
        footer,
        (8, 82),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.35,
        (180, 220, 255),
        1,
        cv2.LINE_AA,
    )
    return output

from __future__ import annotations

import cv2
import numpy as np

from .lane_selector import LeftLaneSelection
from .metric_path import MetricPath


def draw_perception_overlay(
    source: np.ndarray,
    lane: LeftLaneSelection | None,
    metric_path: MetricPath | None,
    steering_angle_rad: float | None,
    elapsed_ms: float,
    error: str | None,
) -> np.ndarray:
    output = source.copy()
    if lane is not None and lane.points is not None:
        points = np.round(lane.points).astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(output, [points], False, (0, 0, 0), 9, cv2.LINE_AA)
        cv2.polylines(output, [points], False, (255, 255, 255), 4, cv2.LINE_AA)

    state = "READY" if error is None else "BLOCKED"
    color = (40, 210, 40) if error is None else (0, 80, 255)
    cv2.rectangle(output, (0, 0), (source.shape[1], 82), (0, 0, 0), -1)
    cv2.putText(output, f"LEFT LANE {state} | CLRerNet {elapsed_ms:.1f} ms", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    metric_text = "metric distance unavailable"
    if metric_path is not None:
        metric_text = (
            f"left={metric_path.measured_distance_m:.3f} m "
            f"error={metric_path.lateral_error_m:+.3f} m"
        )
    cv2.putText(output, metric_text, (8, 41), cv2.FONT_HERSHEY_SIMPLEX,
                0.43, (255, 255, 255), 1, cv2.LINE_AA)
    steer_text = "steering preview unavailable"
    if steering_angle_rad is not None:
        steer_text = f"Stanley preview={np.degrees(steering_angle_rad):+.2f} deg"
    cv2.putText(output, steer_text, (8, 61), cv2.FONT_HERSHEY_SIMPLEX,
                0.43, (255, 255, 255), 1, cv2.LINE_AA)
    if error:
        cv2.putText(output, error[:72], (8, 78), cv2.FONT_HERSHEY_SIMPLEX,
                    0.35, (180, 220, 255), 1, cv2.LINE_AA)
    return output

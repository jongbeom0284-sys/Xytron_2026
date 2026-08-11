from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class ObstacleResult:
    direction: str = "middle"
    front_distance_m: float = float("inf")
    detected: bool = False


class ObstacleDetector:
    """AuTURBO lidar obstacle direction detector."""

    def detect(self, ranges: list[float] | tuple[float, ...], angle_increment: float) -> ObstacleResult:
        if ranges is None or len(ranges) == 0 or angle_increment == 0.0:
            return ObstacleResult()

        arr = np.asarray(ranges, dtype=np.float32)
        arr = np.where(np.isfinite(arr) & (arr > 0.05), arr, 0.0)
        arr[: len(arr) // 4] = 0.0
        arr[3 * len(arr) // 4 :] = 0.0
        deg = np.arange(len(arr), dtype=np.float32) * float(angle_increment) - 252.0 * float(angle_increment)
        forward = arr * np.cos(deg)
        lateral = arr * np.sin(deg)
        front_mask = (np.abs(lateral) < 0.4) & (0.15 < forward) & (forward < 0.4)
        filtered = np.where(front_mask, arr, 0.0)
        nz = np.nonzero(filtered)[0]

        valid_front = forward[(np.abs(lateral) < 0.25) & (forward > 0.15)]
        front_distance = float(np.min(valid_front)) if valid_front.size else float("inf")
        if len(nz) <= 2:
            return ObstacleResult(direction="middle", front_distance_m=front_distance, detected=False)

        median_idx = float(np.median(nz))
        direction = "right" if median_idx > len(arr) / 2.0 else "left"
        return ObstacleResult(direction=direction, front_distance_m=front_distance, detected=True)

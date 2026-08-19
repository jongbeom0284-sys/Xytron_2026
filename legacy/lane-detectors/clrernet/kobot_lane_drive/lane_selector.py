from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class LeftLaneSelection:
    points: np.ndarray | None
    source_index: int | None
    source_lane_count: int
    bottom_x: float | None
    state: str


def _valid_points(raw_lane: np.ndarray) -> np.ndarray | None:
    points = np.asarray(raw_lane, dtype=np.float32).reshape((-1, 2))
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < 3:
        return None
    order = np.argsort(points[:, 1])
    points = points[order]
    _, unique_indices = np.unique(points[:, 1], return_index=True)
    points = points[np.sort(unique_indices)]
    return points if len(points) >= 3 else None


def select_left_lane(
    lanes: list[np.ndarray],
    image_width: int,
    image_height: int,
    minimum_bottom_ratio: float = 0.55,
) -> LeftLaneSelection:
    """Select the nearest CLRerNet lane left of the camera center.

    This function only adapts CLRerNet lane output to the left-boundary contract.
    Ground projection, metric distance, path offset, and steering are delegated to
    the pinned open-source components documented in PROVENANCE.md.
    """
    if image_width <= 0 or image_height <= 0:
        raise ValueError("image dimensions must be positive")
    if not 0.0 <= minimum_bottom_ratio < 1.0:
        raise ValueError("minimum_bottom_ratio must be in [0, 1)")

    center_x = image_width * 0.5
    minimum_y = image_height * minimum_bottom_ratio
    candidates: list[tuple[float, int, np.ndarray]] = []
    for index, raw_lane in enumerate(lanes):
        points = _valid_points(raw_lane)
        if points is None:
            continue
        near_points = points[points[:, 1] >= minimum_y]
        if len(near_points) == 0:
            continue
        bottom_x = float(near_points[np.argmax(near_points[:, 1]), 0])
        if bottom_x < center_x:
            candidates.append((bottom_x, index, points))

    if not candidates:
        return LeftLaneSelection(None, None, len(lanes), None, "LOST")

    bottom_x, source_index, points = max(candidates, key=lambda item: item[0])
    return LeftLaneSelection(
        points=points,
        source_index=source_index,
        source_lane_count=len(lanes),
        bottom_x=bottom_x,
        state="DETECTED",
    )

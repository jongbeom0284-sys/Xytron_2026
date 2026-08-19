"""LaserScan 을 "옆 차선에 차가 있는가"라는 하나의 질문으로 줄인다.

각도로 부채꼴을 자르지 않고, 차량 좌표계의 직사각 코리도로 자른다. 차량은 항상
차선 안에 있고 차선 폭은 대략 알고 있으므로, 옆 차선의 장애물은 "앞으로 y m,
옆으로 x m" 범위에 들어온다. 벽과 연석은 이 상자 밖(옆으로 너무 멀거나, 바로
옆이라 앞쪽 거리가 모자란 곳)에 있으므로 애초에 후보가 되지 않는다.

좌표 규약: x 는 왼쪽이 양수, y 는 앞쪽이 양수.
라이다가 차 앞으로 돌출 장착돼 후방 163도는 차체가 가린다. 실측으로 정면이 원시 121.6도라

ROS 에 의존하지 않는다. ranges 는 그냥 float 시퀀스다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class LidarSectorConfig:
    # 실측 기본값. 라이다가 차 앞으로 돌출 장착돼 정면이 원시 121.6도다.
    angle_offset_rad: float = -2.1217

    # 앞쪽으로 볼 구간. 너무 가까우면 차체와 노면, 너무 멀면 다음 코너의 벽이 들어온다.
    forward_minimum_m: float = 0.35
    forward_maximum_m: float = 2.50

    # 옆 차선에 해당하는 가로 범위. 안쪽 경계는 자기 차선을 제외하고,
    # 바깥쪽 경계는 도로 밖의 벽을 제외한다.
    side_lateral_minimum_m: float = 0.12
    side_lateral_maximum_m: float = 0.75

    # 정면 코리도 (자기 차선 폭)
    front_lateral_maximum_m: float = 0.20

    minimum_range_m: float = 0.15
    maximum_range_m: float = 6.0
    # 이만큼의 점이 동의해야 물체로 인정한다. 반사 한 점으로 차선을 넘지 않기 위해서다.
    minimum_hits: int = 5


@dataclass(frozen=True)
class LidarSectors:
    """각 코리도에서 가장 가까운 물체까지의 전방 거리. 없으면 None."""

    front_m: float | None
    left_m: float | None
    right_m: float | None
    front_count: int
    left_count: int
    right_count: int
    stamp: float

    def side_distance(self, side: str) -> float | None:
        if side == "left":
            return self.left_m
        if side == "right":
            return self.right_m
        return None

    def nearest_side(self) -> str | None:
        if self.left_m is None and self.right_m is None:
            return None
        if self.right_m is None:
            return "left"
        if self.left_m is None:
            return "right"
        return "left" if self.left_m < self.right_m else "right"

    def is_fresh(self, now: float, maximum_age_sec: float) -> bool:
        return self.stamp > 0.0 and (now - self.stamp) <= maximum_age_sec


EMPTY_SECTORS = LidarSectors(None, None, None, 0, 0, 0, 0.0)


def _nearest(forward: np.ndarray, minimum_hits: int) -> float | None:
    """코리도 안에서 minimum_hits 번째로 가까운 전방 거리."""
    if forward.size < minimum_hits:
        return None
    return float(np.partition(forward, minimum_hits - 1)[minimum_hits - 1])


def scan_to_vehicle_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    config: LidarSectorConfig,
) -> tuple[np.ndarray, np.ndarray]:
    """유효한 측정만 차량 좌표계 (x 왼쪽+, y 앞쪽+) 로 옮긴다."""
    array = np.asarray(ranges, dtype=np.float64)
    if array.size == 0 or angle_increment == 0.0:
        empty = np.empty(0, dtype=np.float64)
        return empty, empty
    angles = angle_min + np.arange(array.size, dtype=np.float64) * angle_increment
    angles = angles + config.angle_offset_rad
    usable = (
        np.isfinite(array)
        & (array >= config.minimum_range_m)
        & (array <= config.maximum_range_m)
    )
    distance = array[usable]
    theta = angles[usable]
    return distance * np.sin(theta), distance * np.cos(theta)


def summarize_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    stamp: float,
    config: LidarSectorConfig | None = None,
) -> LidarSectors:
    config = config or LidarSectorConfig()
    x, y = scan_to_vehicle_points(ranges, angle_min, angle_increment, config)
    if x.size == 0:
        return LidarSectors(None, None, None, 0, 0, 0, stamp)

    ahead = (y >= config.forward_minimum_m) & (y <= config.forward_maximum_m)
    lateral = np.abs(x)
    in_side_band = (
        ahead
        & (lateral >= config.side_lateral_minimum_m)
        & (lateral <= config.side_lateral_maximum_m)
    )
    left = in_side_band & (x > 0.0)
    right = in_side_band & (x < 0.0)
    front = ahead & (lateral <= config.front_lateral_maximum_m)

    return LidarSectors(
        front_m=_nearest(y[front], config.minimum_hits),
        left_m=_nearest(y[left], config.minimum_hits),
        right_m=_nearest(y[right], config.minimum_hits),
        front_count=int(front.sum()),
        left_count=int(left.sum()),
        right_count=int(right.sum()),
        stamp=stamp,
    )


def side_is_occupied(
    sectors: LidarSectors,
    side: str,
    threshold_m: float,
    now: float,
    maximum_age_sec: float,
) -> bool:
    """지정한 쪽 옆 차선 코리도에 임계 거리 이내의 물체가 있는가.

    오래된 스캔은 근거로 쓰지 않는다. 라이다 9.7 Hz, 인식 15 Hz 로 주기가 달라
    직전 값이 이미 낡았을 수 있다.
    """
    if not sectors.is_fresh(now, maximum_age_sec):
        return False
    distance = sectors.side_distance(side)
    if distance is None:
        return False
    return distance <= threshold_m

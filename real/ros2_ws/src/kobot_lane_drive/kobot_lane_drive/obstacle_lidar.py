"""LaserScan 을 전방·좌·우 세 구역 거리로 요약한다.

카메라가 "이쪽 차선이 좁아졌다"고 말할 때 그게 진짜 물체인지 확인하는 용도다.
라이다 단독으로 차선 점유를 판정하지 않는다 — 벽과 연석도 같은 거리에 잡히기 때문이다.

각도 규약: xycar_lidar 의 원시 0 rad 은 차량 후방을 본다. angle_offset_rad(기본 pi)를
더해 0 = 정면으로 맞춘 뒤, 양수를 좌측, 음수를 우측으로 쓴다. LidarAvoidance 가 쓰던
보정과 같다. 드라이버가 바뀌면 이 값만 파라미터로 고친다.

ROS 에 의존하지 않는다. ranges 는 그냥 float 시퀀스다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class LidarSectorConfig:
    angle_offset_rad: float = math.pi
    front_half_deg: float = 18.0
    side_minimum_deg: float = 12.0
    side_maximum_deg: float = 75.0
    minimum_range_m: float = 0.15
    maximum_range_m: float = 6.0
    # 구역 거리로 k번째 근접값을 쓴다. 단발 잡음 하나로 회피가 트리거되지 않으면서,
    # 실제 차량처럼 여러 점을 만드는 물체는 놓치지 않는 절충이다.
    minimum_hits: int = 5


@dataclass(frozen=True)
class LidarSectors:
    """각 구역의 대표 거리. 물체가 없으면 None."""

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
        """좌우 중 더 가까운 쪽. 양쪽 다 비었으면 None."""
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


def _representative(values: np.ndarray, minimum_hits: int) -> float | None:
    """구역을 대표하는 거리 = minimum_hits 번째로 가까운 값.

    최솟값을 그대로 쓰면 반사 하나에 회피가 걸린다. 백분위수를 쓰면 구역의 점 개수에
    따라 임계가 흔들려서, 점이 많은 넓은 구역에서 작은 물체를 놓친다. k번째 근접값은
    구역 크기와 무관하게 "그만큼의 점이 동의해야 인정"이라는 뜻이 된다.
    """
    if values.size == 0:
        return None
    if values.size < minimum_hits:
        return None
    partitioned = np.partition(values, minimum_hits - 1)
    return float(partitioned[minimum_hits - 1])


def summarize_scan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    stamp: float,
    config: LidarSectorConfig | None = None,
) -> LidarSectors:
    config = config or LidarSectorConfig()
    array = np.asarray(ranges, dtype=np.float64)
    if array.size == 0 or angle_increment == 0.0:
        return LidarSectors(None, None, None, 0, 0, 0, stamp)

    angles = angle_min + np.arange(array.size, dtype=np.float64) * angle_increment
    angles = angles + config.angle_offset_rad
    angles = (angles + math.pi) % (2.0 * math.pi) - math.pi
    degrees = np.degrees(angles)

    usable = (
        np.isfinite(array)
        & (array >= config.minimum_range_m)
        & (array <= config.maximum_range_m)
    )

    front = usable & (np.abs(degrees) <= config.front_half_deg)
    left = usable & (degrees > config.side_minimum_deg) & (degrees <= config.side_maximum_deg)
    right = usable & (degrees < -config.side_minimum_deg) & (degrees >= -config.side_maximum_deg)

    return LidarSectors(
        front_m=_representative(array[front], config.minimum_hits),
        left_m=_representative(array[left], config.minimum_hits),
        right_m=_representative(array[right], config.minimum_hits),
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
    """지정한 쪽에 임계 거리 이내의 물체가 있는가.

    오래된 스캔은 근거로 쓰지 않는다. 라이다 9.7 Hz, 인식 15 Hz 로 주기가 달라
    직전 값이 이미 낡았을 수 있다.
    """
    if not sectors.is_fresh(now, maximum_age_sec):
        return False
    distance = sectors.side_distance(side)
    if distance is None:
        return False
    return distance <= threshold_m

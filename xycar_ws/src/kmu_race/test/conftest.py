"""테스트 공용 헬퍼.

core/ 는 rclpy·cv2·numpy 에 의존하지 않으므로 ROS 환경 없이 그냥 돌아간다.

    cd xycar_ws/src/kmu_race && python3 -m pytest test -q
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kmu_race.core.types import (  # noqa: E402
    ConeReading,
    LaneReading,
    LidarReading,
    Observation,
    SignalReading,
    SignalState,
    TrackedObject,
)


def make_obs(
    now: float,
    *,
    signal: SignalState = SignalState.UNKNOWN,
    gantry_span: int = 0,
    lane_offset: float = 0.0,
    lane_side: int = 0,
    cone_px: int = 0,
    gate_center: float = 0.0,
    gate_valid: bool = False,
    front_target: TrackedObject = None,
    left_clear: bool = True,
    right_clear: bool = True,
    yaw: float = 0.0,
    ego_mps: float = 1.0,
    camera_ok: bool = True,
    lidar_ok: bool = True,
) -> Observation:
    """관측 하나를 손으로 지어낸다. 테스트 가독성을 위한 얇은 래퍼."""
    objects = (front_target,) if front_target is not None else ()
    return Observation(
        now=now,
        signal=SignalReading(state=signal, gantry_span_px=gantry_span),
        lane=LaneReading(
            offset_px=lane_offset,
            lookahead_px=lane_offset,
            left_found=True,
            right_found=True,
            lane_width_px=450.0,
            lane_side=lane_side,
        ),
        lidar=LidarReading(
            front_dist=front_target.x if front_target else float("inf"),
            left_clear=left_clear,
            right_clear=right_clear,
            objects=objects,
            stamp=now,
        ),
        cone=ConeReading(
            detected=cone_px > 0,
            orange_px=cone_px,
            gate_center_px=gate_center,
            gate_valid=gate_valid,
            left_cone_px=cone_px // 2,
            right_cone_px=cone_px // 2,
        ),
        yaw_deg=yaw,
        ego_speed_mps=ego_mps,
        camera_ok=camera_ok,
        lidar_ok=lidar_ok,
    )


def stationary_target(x: float, y: float = 0.0, ego_mps: float = 1.0) -> TrackedObject:
    """고정장애물처럼 보이는 트랙 — 상대 접근속도가 자차속도와 같다."""
    return TrackedObject(x=x, y=y, width=0.3, track_id=1, vx=-ego_mps, age=10)


def moving_target(x: float, y: float = 0.0, ego_mps: float = 1.0) -> TrackedObject:
    """방해차량처럼 보이는 트랙 — 같은 방향으로 달려 접근속도가 작다."""
    return TrackedObject(x=x, y=y, width=0.3, track_id=2, vx=-0.2 * ego_mps, age=10)

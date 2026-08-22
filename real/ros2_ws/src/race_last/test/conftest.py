"""테스트 공용 헬퍼.

core/ 는 rclpy·cv2·numpy 에 의존하지 않으므로 ROS 환경 없이 그냥 돌아간다.

    cd xycar_ws/src/kmu_race && python3 -m pytest test -q
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from race_last.kmu_race.core.types import (  # noqa: E402
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
    housing_span: int = None,
    lane_offset: float = 0.0,
    lane_side: int = 0,
    cone_px: int = 0,
    gate_center: float = 0.0,
    gate_valid: bool = False,
    lidar_cones: int = -1,      # -1 = 카메라 콘이 보이면 라이다도 본다

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
        signal=SignalReading(
            state=signal,
            gantry_span_px=gantry_span,
            # 랩 마커 2세대: '구조 검증된 하우징을 봤다'. 테스트에서 겐트리를
            # 보여 준다는 뜻은 하우징도 보인다는 뜻이므로 기본을 같이 채운다
            # (types.SignalReading.lap_marker_px 주석 참조).
            housing_span_px=gantry_span if housing_span is None else housing_span,
        ),
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
            # 실차에서 카메라가 콘을 보면 라이다도 본다 (구간 내 스캔당 7~9개).
            # FSM 은 라이다 콘이 0개면 진입하지 않으므로(0821 밤), 기본값을
            # 0 으로 두면 모든 라바콘 테스트가 진입조차 못 한다.
            lidar_cones=(6 if cone_px > 0 else 0) if lidar_cones < 0 else lidar_cones,
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


def approach(
    maker,
    *,
    x0: float = 1.40,
    x1: float = 0.55,
    n: int = 12,
    step: float = 0.1,
    y: float = 0.0,
    ego_mps: float = 1.2,
):
    """전방 타겟이 실제로 '접근하는' 궤적을 만든다.

    2026-08-20 도입 — FSM 의 진입 게이트가 '연속 2틱'에서
    **'0.8초 이상 계속 차로를 막으면서 실제로 가까워질 것'**으로 바뀌었다
    (race_fsm.FSMConfig.target_enter_seconds 주석에 실측 근거).
    옛 테스트는 x 를 고정한 채 4틱만 먹였는데, 그건 정면을 스쳐가는 벽·의자와
    구분이 안 되는 관측이라 이제는(정당하게) 회피를 발동시키지 않는다.

    반환: [(dt, TrackedObject), ...] — dt 는 시작 시각으로부터의 오프셋.
    """
    out = []
    for i in range(n):
        frac = i / max(1, n - 1)
        out.append((step * i, maker(x0 + (x1 - x0) * frac, y=y, ego_mps=ego_mps)))
    return out

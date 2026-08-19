"""조향·속도를 motor_adapter 가 받는 모터 명령 배열로 바꾼다.

`[legacy_angle, legacy_speed]` 두 값이다. legacy 라 부르는 이유는 Xycar 원본
스택이 쓰던 단위(각도 ±70, 속도 0~50 의 무차원 값)를 그대로 유지하기 때문이다.
motor_adapter 가 반대편에서 같은 배율로 되돌린다.

ROS 에 의존하지 않는다. 브리지의 안전 판단(ready/fresh)을 명령으로 옮기는
마지막 한 단계라서 단위 시험으로 따로 검증한다.
"""

from __future__ import annotations


def make_legacy_command(
    steering_rad: float | None,
    speed_mps: float,
    angle_scale: float,
    speed_scale: float,
    ready: bool,
    fresh: bool,
) -> list[float]:
    # 판단이 준비되지 않았거나(ready) 값이 낡았으면(fresh) 무조건 정지 명령이다.
    # 마지막 값을 유지하지 않는다 — 상류가 죽었을 때 차가 계속 가면 안 된다.
    if not ready or not fresh or steering_rad is None:
        return [0.0, 0.0]
    if angle_scale <= 0.0 or speed_scale <= 0.0:
        raise ValueError("motor adapter scales must be positive")
    # 부호 반전: 제어는 왼쪽을 양수로 쓰고 차량 입력은 오른쪽을 양수로 쓴다.
    # 이 한 줄이 조향 방향의 유일한 정의다. 바꾸면 차가 반대로 돈다.
    legacy_angle = max(-70.0, min(70.0, -steering_rad / angle_scale))
    legacy_speed = max(0.0, min(50.0, speed_mps / speed_scale))
    return [legacy_angle, legacy_speed]

"""모터 출력 어댑터.

실차(자이카 ROS2 미니PC Y모델)의 모터 인터페이스:

    publisher: 'xycar_motor'  std_msgs/Float32MultiArray
    data     : [angle(deg), speed]
    angle 부호: 음수 = 좌회전 / 양수 = 우회전

이 시그니처는 두 곳에서 교차 확인했다.
  · 팀 실차 코드(`segi/my_driver_*.py`) — 실제로 차를 굴린 코드
  · 자이트론 공식 워크스페이스 `study/my_motor/go.py`
    (거기서 `XycarMotor` 임포트는 주석 처리돼 있고 Float32MultiArray 로 대체됨)

주의: Unity 시뮬레이터 계통은 `xycar_msgs/XycarMotor` 에 `.angle`/`.speed` 필드를
쓴다. 참고 레포에서 코드를 가져올 때 어느 계통인지 먼저 확인할 것 —
시뮬용 퍼블리셔를 실차에 붙이면 아무 반응이 없다.
`use_xycar_msgs` 파라미터로 두 계통을 모두 지원한다.
"""

from __future__ import annotations

import math
from typing import Optional

from std_msgs.msg import Float32MultiArray


class MotorPublisher:
    """조향/속도를 자이카 모터 토픽으로 내보낸다.

    발행 직전에 한 번 더 클램프한다. 상위에서 이미 제한하지만, 모터로 나가는
    마지막 관문이라 여기서도 막는 편이 안전하다.
    """

    def __init__(
        self,
        node,
        *,
        topic: str = "xycar_motor",
        queue: int = 1,
        max_steer: float = 50.0,
        max_speed: float = 50.0,
        use_xycar_msgs: bool = False,
    ) -> None:
        self.max_steer = max_steer
        self.max_speed = max_speed
        self.use_xycar_msgs = use_xycar_msgs
        self._node = node

        if use_xycar_msgs:
            from xycar_msgs.msg import XycarMotor  # type: ignore

            self._msg_type = XycarMotor
            self._pub = node.create_publisher(XycarMotor, topic, queue)
        else:
            self._msg_type = Float32MultiArray
            self._pub = node.create_publisher(Float32MultiArray, topic, queue)

        self.last_steer = 0.0
        self.last_speed = 0.0

    def publish(self, steer: float, speed: float) -> None:
        steer = float(steer)
        speed = float(speed)
        # NaN/inf 는 파이썬 min/max 를 그대로 통과해 +max 로 둔갑한다
        # (min(50.0, nan) == 50.0 — 비교가 False 라 첫 인자 유지). 마지막
        # 관문에서 반드시 걸러 정지로 강등한다.
        if not (math.isfinite(steer) and math.isfinite(speed)):
            steer, speed = 0.0, 0.0
        steer = max(-self.max_steer, min(self.max_steer, steer))
        speed = max(-self.max_speed, min(self.max_speed, speed))
        self.last_steer, self.last_speed = steer, speed

        if self.use_xycar_msgs:
            msg = self._msg_type()
            msg.angle = steer
            msg.speed = speed
            self._pub.publish(msg)
        else:
            # 공유 버퍼 재사용은 MultiThreadedExecutor 에서 stop() 과 제어
            # 루프의 동시 publish 가 인터리브될 때 정지 명령을 주행 명령으로
            # 둔갑시킬 수 있다 (직렬화 시점에 .data 를 읽는다). 매번 새로 만든다.
            msg = Float32MultiArray()
            msg.data = [steer, speed]
            self._pub.publish(msg)

    def stop(self) -> None:
        """정지. 종료 시 확실히 서도록 여러 번 쏜다 (한 발이 유실될 수 있음)."""
        for _ in range(3):
            self.publish(0.0, 0.0)


__all__ = ["MotorPublisher"]

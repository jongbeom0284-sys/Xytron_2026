"""자동 잠금해제 — race_day.launch.py 의 마지막 조각.

race_go.sh 가 하던 일을 launch 안에서 한다: 뇌(/yolop_drive)와
근육(/kobot_motor_adapter)이 응답할 때까지 기다렸다가
max_speed_mps 설정 + armed=true + /kobot_yolop/enabled=true 를 걸고,
/kobot_yolop/status 로 실제 반영을 확인한다 (발행 유실 대비 재시도 —
0821 cone_once 교훈).

성공하면 "신호 대기" 로그를 남기고 조용히 대기한다. 이후 출발 판단은
전적으로 yolop_drive 의 신호등 로직이다 (빨간불 정지 → 초록불 자동 출발).
"""
import json

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, String

TIMEOUT_SEC = 180.0        # TensorRT 적재 등 부팅이 길어도 이만큼은 기다린다
RETRY_SEC = 2.0


class RaceStarter(Node):
    def __init__(self):
        super().__init__('race_starter')
        self.declare_parameter('speed_mps', 2.0)
        self.speed = float(self.get_parameter('speed_mps').value)

        self.pub_enable = self.create_publisher(Bool, '/kobot_yolop/enabled', 10)
        self.create_subscription(String, '/kobot_yolop/status',
                                 self._on_status, 10)
        self.cli = self.create_client(SetParameters,
                                      '/yolop_drive/set_parameters')
        self._armed_seen = False
        self._enabled_seen = False
        self._done = False
        self._elapsed = 0.0
        self._pending = None
        self.create_timer(RETRY_SEC, self._tick)
        self.get_logger().info(
            '자동 잠금해제 대기 — 목표 속도 %.2f m/s' % self.speed)

    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        self._armed_seen = bool(data.get('armed', False))
        self._enabled_seen = bool(data.get('enabled', False))

    def _param(self, name, value):
        p = Parameter(name=name)
        if isinstance(value, bool):
            p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                     bool_value=value)
        else:
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(value))
        return p

    def _tick(self):
        if self._done:
            return
        self._elapsed += RETRY_SEC
        if self._elapsed > TIMEOUT_SEC:
            self.get_logger().error(
                '%.0f초 내 잠금해제 실패 — 수동으로 ~/race_go.sh 를 실행하라'
                % TIMEOUT_SEC)
            self._done = True
            return

        # 성공 판정: status 가 armed+enabled 를 모두 보고할 때
        if self._armed_seen and self._enabled_seen:
            self._done = True
            self.get_logger().info(
                '━━ 잠금해제 완료 (%.2f m/s) — 신호 대기. '
                '빨간불 정지 → 초록불 자동 출발 ━━' % self.speed)
            return

        # 뇌가 아직 안 떴으면 조용히 다음 틱을 기다린다
        if not self.cli.service_is_ready():
            if self._elapsed % 10 < RETRY_SEC:
                self.get_logger().info(
                    'yolop_drive 대기 중... (%.0f초)' % self._elapsed)
            return

        # enabled(토픽) + armed/max_speed(파라미터) 를 매 틱 다시 건다
        self.pub_enable.publish(Bool(data=True))
        if self._pending is None or self._pending.done():
            req = SetParameters.Request()
            req.parameters = [
                self._param('max_speed_mps', self.speed),
                self._param('armed', True),
            ]
            self._pending = self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = RaceStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

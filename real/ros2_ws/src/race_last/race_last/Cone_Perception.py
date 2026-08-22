"""라바콘 인식 노드 — /scan → 콘 게이트 → /cone/state, /cone/gate.

검출은 kmu_race 의 lidar_cones.find_cone_gate 를 그대로 쓴다 (2026-08-21
작업분, 실측 튜닝 포함 — 이 노드에서 수정하지 않는다. 원본:
/home/xytron/xycar_ws/src/kmu_race/kmu_race/perception/lidar_cones.py).

켜지는 조건: 신호등이 직진(green)이 된 순간부터 signal_delay_sec(기본 1.0초)
뒤에 켜지고, 한 번 켜지면 계속 켜져 있다 (래치).
force_enabled:=true 면 신호 없이 즉시 켠다 — 콘 구간 단독 시험용.

발행:
  /cone/state  String            'O 깊이.. 폭.. 횡..' / 'X ..' — 콘솔 표시용
  /cone/gate   Float32MultiArray [valid, depth, width, lateral, cones,
                                  near_left, near_right] — 조향 연동용
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from .kmu_race.perception.lidar_cones import (
    LidarConeConfig,
    find_cone_gate,
)

# 8/17 실측: 차 전진축 = 라이다 +124°
# (출처: xycar_ws/src/kmu_race/config/params.yaml — 콘 주행이 쓰던 값)
DEFAULT_ANGLE_OFFSET = -2.164


class ConePerception(Node):
    def __init__(self):
        super().__init__('cone_perception')
        self.declare_parameter('signal_delay_sec', 1.0)
        self.declare_parameter('force_enabled', False)
        self.declare_parameter('angle_offset_rad', DEFAULT_ANGLE_OFFSET)
        self.delay = float(self.get_parameter('signal_delay_sec').value)
        self.force = bool(self.get_parameter('force_enabled').value)
        self.offset = float(self.get_parameter('angle_offset_rad').value)

        self.config = LidarConeConfig()
        self.green_at = None          # 직진 신호를 처음 본 시각
        self.enabled = self.force
        self.row_side = 0             # find_cone_gate 의 줄 판정 되먹임

        self.pub_state = self.create_publisher(String, '/cone/state', 10)
        self.pub_gate = self.create_publisher(
            Float32MultiArray, '/cone/gate', 10)
        # yolop_drive 의 cone_link 가 먹는 전체 관측 (JSON)
        self.pub_reading = self.create_publisher(
            String, '/cone/reading', 10)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_signal, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            '콘 인식 대기. 직진 신호 +%.1f초 뒤 활성 (force_enabled=%s)'
            % (self.delay, self.force))

    # ------------------------------------------------------------ 신호 게이트
    def _on_signal(self, msg):
        if self.green_at is not None:
            return
        try:
            state = json.loads(msg.data).get('state', '')
        except (ValueError, AttributeError):
            state = msg.data
        if state == 'green':
            self.green_at = time.monotonic()
            self.get_logger().info(
                '직진 신호 — %.1f초 뒤 콘 인식을 켠다' % self.delay)

    # ------------------------------------------------------------ 스캔 처리
    def _on_scan(self, msg):
        if not self.enabled:
            if (self.green_at is not None
                    and time.monotonic() - self.green_at >= self.delay):
                self.enabled = True
                self.get_logger().info('콘 인식 활성')
            else:
                self.pub_state.publish(String(data='X 직진 대기'))
                return

        gate = find_cone_gate(
            msg.ranges, msg.angle_min, msg.angle_increment,
            angle_offset=self.offset, config=self.config,
            prev_row_side=self.row_side)
        self.row_side = gate.near_row_side

        out = Float32MultiArray()
        out.data = [
            1.0 if gate.valid else 0.0,
            float(gate.depth_m),
            float(gate.width_m),
            float(gate.lateral_m),
            float(gate.cones),
            9.9 if math.isinf(gate.near_left_m) else float(gate.near_left_m),
            9.9 if math.isinf(gate.near_right_m) else float(gate.near_right_m),
        ]
        self.pub_gate.publish(out)

        if gate.valid:
            text = 'O 깊이%.2f 폭%.2f 횡%+.2f' % (
                gate.depth_m, gate.width_m, gate.lateral_m)
        else:
            text = 'X 콘%d' % gate.cones
        self.pub_state.publish(String(data=text))

        payload = dict(
            stamp=time.monotonic(),
            valid=bool(gate.valid),
            depth=round(float(gate.depth_m), 3),
            width=round(float(gate.width_m), 3),
            lateral=round(float(gate.lateral_m), 3),
            cones=int(gate.cones),
            near_left=(9.9 if math.isinf(gate.near_left_m)
                       else round(float(gate.near_left_m), 3)),
            near_right=(9.9 if math.isinf(gate.near_right_m)
                        else round(float(gate.near_right_m), 3)),
            pts=[[round(x, 3), round(y, 3)] for x, y in gate.points],
        )
        self.pub_reading.publish(String(data=json.dumps(payload)))


def main(args=None):
    rclpy.init(args=args)
    node = ConePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

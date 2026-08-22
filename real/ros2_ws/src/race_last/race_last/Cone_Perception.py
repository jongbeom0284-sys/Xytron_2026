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
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from . import cone_wall
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
        # 콘 구간 통과가 약 10초. 여유를 붙여 이만큼 열어 둔다 (0823).
        self.declare_parameter('on_window_sec', 15.0)
        self.declare_parameter('signal_delay_sec', 1.0)
        self.declare_parameter('force_enabled', False)
        self.declare_parameter('angle_offset_rad', DEFAULT_ANGLE_OFFSET)
        self.delay = float(self.get_parameter('signal_delay_sec').value)
        self.force = bool(self.get_parameter('force_enabled').value)
        self.offset = float(self.get_parameter('angle_offset_rad').value)

        self.config = LidarConeConfig()
        # 벽 추종 — 통로 중앙에서 오른쪽으로 미는 거리. 0 이면 정중앙.
        self.declare_parameter('wall_bias_cm', 0.0)
        self.green_at = None          # 직진 신호를 처음 본 시각
        self.open_until = 0.0         # 이 시각까지 콘 인식을 연다
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
        self.create_subscription(Bool, '/cone/trigger', self._on_trigger, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            '콘 인식 대기. 차선 위 주황이 잡히면 %.0f초간 활성 '
            '(force_enabled=%s)'
            % (float(self.get_parameter('on_window_sec').value), self.force))

        # 콘 인식 결과를 jsonl 로 남긴다 — 라이다 기반이라 카메라 녹화에는
        # 안 남으므로, 재생 뷰어의 콘 미니맵은 이 파일로 그린다 (2026-08-22)
        self._log_fh = None

    # ------------------------------------------------------------ 진입 트리거
    def _on_trigger(self, msg):
        """차선 위 주황이 잡혔다 — 콘 구간 진입. 창 시간만큼 연다.

        첫 콘만 차선 위에 있으므로 이 신호는 진입 순간에만 온다. 그래서
        '보이는 동안' 이 아니라 '받은 뒤 창 시간' 으로 유지한다.
        """
        if not bool(msg.data):
            return
        window = float(self.get_parameter('on_window_sec').value)
        self.open_until = time.monotonic() + window
        if not self.enabled:
            self.get_logger().warning(
                '차선 위 주황 — 콘 인식 %.0f초간 켬' % window)

    # ------------------------------------------------------------ 신호 게이트
    def _on_signal(self, msg):
        """직진(green) 상태일 때만 콘 인식을 켠다.

        2026-08-23: 예전에는 초록을 한 번 보면 영구 래치라, 몇 분 전 신호
        하나로 이후 전 구간에서 콘이 돌았다 (실주행 로그: traffic_state=none
        인데 콘 게이트가 -40 을 냈고 그대로 코스 이탈). 이제 직진이 풀리면
        (좌회전·빨강·신호없음) 콘도 함께 꺼진다.
        """
        try:
            state = json.loads(msg.data).get('state', '')
        except (ValueError, AttributeError, TypeError):
            state = msg.data or ''
        if state == 'green':
            if self.green_at is None:
                self.green_at = time.monotonic()
                self.get_logger().info(
                    '직진 신호 — %.1f초 뒤 콘 인식을 켠다' % self.delay)
        elif state in ('left', 'red'):
            # 직진이 아닌 '판정' 이 났을 때만 끈다.
            # none(신호등이 시야 밖) 은 판정이 아니라 관측 부재이므로
            # 직전 상태를 그대로 유지한다 (2026-08-23).
            if self.enabled or self.open_until:
                self.get_logger().info('직진 해제(%s) — 콘 인식 끔' % state)
            self.green_at = None
            self.open_until = 0.0
            self.enabled = self.force
        # state == 'none' / 'yellow' : 아무것도 바꾸지 않는다

    # ------------------------------------------------------------ 스캔 처리
    def _on_scan(self, msg):
        # 켜짐은 진입 트리거가 연 창 안에서만 성립한다. 창이 닫히면 꺼진다.
        if not self.force and time.monotonic() >= self.open_until:
            if self.enabled:
                self.get_logger().info('콘 인식 창 종료 — 끔')
            self.enabled = False
            self.pub_state.publish(String(data='X 진입 대기'))
            return
        self.enabled = True

        # 벽 추종 조준값. 짝지은 게이트와 별개로 매 스캔 계산해 함께 싣는다.
        wall = cone_wall.solve(
            msg.ranges, msg.angle_min, msg.angle_increment, self.offset,
            float(self.get_parameter('wall_bias_cm').value) / 100.0)

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

        if wall is not None:
            text = 'O 벽 좌%d/우%d 조준%+.0fcm (%s)' % (
                len(wall['left']), len(wall['right']),
                wall['aim_y'] * 100, wall['basis'])
        elif gate.valid:
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
            # 벽 추종 결과 — ConeLink 가 조향 근거로 쓴다
            wall_aim=(None if wall is None else round(wall['aim_y'], 3)),
            wall_basis=('' if wall is None else wall['basis']),
            wall_left=(0 if wall is None else len(wall['left'])),
            wall_right=(0 if wall is None else len(wall['right'])),
        )
        self.pub_reading.publish(String(data=json.dumps(payload)))

        if self._log_fh is None:
            os.makedirs('/home/xytron/cone_logs', exist_ok=True)
            path = time.strftime('/home/xytron/cone_logs/conelog_%Y%m%d_%H%M%S.jsonl')
            self._log_fh = open(path, 'w', buffering=1)
            self.get_logger().info('콘 기록 시작: %s' % path)
        payload['wall'] = time.time()
        # 원본 스캔도 같이 남긴다 (2026-08-23). 결과만 있으면 '설정을
        # 바꿨으면 어땠을까' 를 검증할 수 없다 — 의자다리 오인 수정 때
        # 원본이 없어 콘 좌표만으로 간접 확인해야 했다.
        # 소수 2자리(1cm)로 줄여 용량을 1/3 로 만든다.
        payload['scan'] = {
            'angle_min': round(float(msg.angle_min), 6),
            'angle_increment': round(float(msg.angle_increment), 8),
            'ranges': [(-1.0 if not math.isfinite(r) else round(float(r), 2))
                       for r in msg.ranges],
        }
        self._log_fh.write(json.dumps(payload) + chr(10))


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

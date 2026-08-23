"""라바콘 인식 노드 — /scan → 콘 게이트 → /cone/state, /cone/gate.

검출은 kmu_race 의 lidar_cones.find_cone_gate 를 그대로 쓴다 (2026-08-21
작업분, 실측 튜닝 포함 — 이 노드에서 수정하지 않는다. 원본:
/home/xytron/xycar_ws/src/kmu_race/kmu_race/perception/lidar_cones.py).

켜지는 조건: yolop_drive 가 '차선 위 주황' 을 확인하고 보내는 /cone/trigger.
받으면 on_window_sec(기본 15초) 동안 열린다. 그 창은 시간으로만 닫힌다 —
신호등 판정(red/left)으로는 끄지 않는다 (콘 구간엔 신호등이 없어서 그 판정은
오검출이고, 실제로 그 오검출이 콘 주행을 반복해서 끊었다. 0823 밤).
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
from .center_offset import calculate_center_offset_target
from .kmu_race.perception.lidar_cones import (
    LidarConeConfig,
    find_cone_gate,
)

# 8/17 실측: 차 전진축 = 라이다 +124°
# (출처: xycar_ws/src/kmu_race/config/params.yaml — 콘 주행이 쓰던 값)
DEFAULT_ANGLE_OFFSET = -2.164

# ── 차로 안 정지물체 (앞차 자리에 놓인 차+라바콘) ──
# 콘 게이트와 별개다. 게이트는 '좌우 쌍' 을 요구하지만 앞차 자리의 물체는
# 한쪽에만 있고 라이다 점이 1개뿐일 때도 있다 (실측 f317: 1점, 1.19m).
# 그래서 짝짓기·최소점수 없이 '차로 폭 안에 찍힌 점' 만 본다.
OBS_X_MIN = 0.25        # 이보다 가까우면 차체·범퍼
OBS_X_MAX = 2.5         # 이보다 멀면 아직 볼 필요 없다
OBS_HALF_W = 0.45       # 차로 반폭 0.41 + 여유
OBS_GAP = 0.15          # 이 이상 벌어지면 다른 물체
TOPIC_OBSTACLE = '/lidar/obstacle'


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
        # 차량 중심 보정. 트리거/FSM과 무관하며, 인식이 켜진 뒤 wall_aim만
        # 실차 검증본과 동일하게 바꾼다.
        self.declare_parameter('vehicle_length_m', 0.60)
        self.declare_parameter('vehicle_width_m', 0.30)
        self.declare_parameter('lidar_to_vehicle_center_m', 0.15)
        self.declare_parameter('center_lookahead_m', 0.55)
        self.declare_parameter('boundary_clearance_m', 0.02)
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
        # 차로 안 정지물체 — 콘 인식 창과 **무관하게 항상** 발행한다.
        # 앞차 구간은 콘 창이 닫힌 뒤에도 오므로 여기 묶으면 안 된다.
        self.pub_obstacle = self.create_publisher(String, TOPIC_OBSTACLE, 10)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_signal, 10)
        self.create_subscription(Bool, '/cone/trigger', self._on_trigger, 10)
        self.create_subscription(String, '/cone/command', self._on_command, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info(
            '콘 인식 대기. 차선 위 주황이 잡히면 %.0f초간 활성 '
            '(force_enabled=%s)'
            % (float(self.get_parameter('on_window_sec').value), self.force))

        # 콘 인식 결과를 jsonl 로 남긴다 — 라이다 기반이라 카메라 녹화에는
        # 안 남으므로, 재생 뷰어의 콘 미니맵은 이 파일로 그린다 (2026-08-22)
        self._log_fh = None

    # ------------------------------------------------------------ 강제 제어
    def _on_command(self, msg):
        """대시보드에서 콘 인식을 손으로 켜고 끈다.

        신호등을 안 지나고 콘 구간만 시험할 때, 또는 오진입을 손으로 끊을 때
        쓴다. 조건(초록불+주황)을 우회하므로 시험용이다.
        """
        try:
            body = json.loads(msg.data)
            cmd = str(body.get('cmd', ''))
        except (ValueError, TypeError, AttributeError):
            return
        if cmd == 'force':
            sec = float(body.get('sec', 15.0) or 15.0)
            self.open_until = time.monotonic() + max(0.0, sec)
            self.get_logger().warning('콘 인식 강제로 %.0f초 켬' % sec)
        elif cmd == 'off':
            self.open_until = 0.0
            self.green_at = None
            self.enabled = self.force
            self.get_logger().warning('콘 인식 강제 종료')

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
        # 라바콘 단독 테스트(force_enabled=true)에서는 신호등 상태가
        # 인식 게이트를 덮어쓰지 못하게 한다.
        if self.force:
            self.enabled = True
            return
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
            # ★ 0823 밤: 여기서 콘 인식을 끄지 않는다.
            #
            # 콘 구간 안에는 신호등이 없다. 그런데 이 노드는 별도 프로세스라
            # yolop_drive 의 신호 잠금(초록 확정 후 30초)을 모르고
            # /traffic_light/state 원본을 그대로 믿는다. 그래서 콘 통로
            # 한복판에서 나온 오검출 한 번에 창을 지워 버렸다:
            #     19:57  진입 0.04초 뒤 성공 → 0.77초 뒤 '직진 해제(red)' → 이탈
            #     21:29  진입 0.02초 뒤 성공 → 1.98초 뒤 '직진 해제(left)' → 이탈
            #     21:30  같은 패턴 (2.6초 뒤 left)
            # 창을 지우면 /cone/reading 이 끊기고, 0.6초 뒤 ConeLink 가
            # STALE_SEC 로 콘 구간을 빠져나온다. 콘 주행이 계속 꺼진 원인이다.
            #
            # 창은 어차피 on_window_sec(15초) 로 스스로 닫히므로, 신호로
            # 끊을 이유가 없다. 손으로 끊어야 하면 대시보드의 '콘 인식 끄기'
            # (/cone/command cmd=off) 를 쓴다.
            self.green_at = None
        # state == 'none' / 'yellow' : 아무것도 바꾸지 않는다

    # ------------------------------------------------------------ 스캔 처리
    def _obstacle(self, msg):
        """차로 폭 안에 찍힌 점을 덩어리로 묶어 가장 가까운 것을 낸다.

        짝짓기도 최소 점수도 요구하지 않는다 — 라바콘은 원뿔이라 라이다
        높이에서 1점만 맞는 일이 흔하다 (실측 10스캔 중 9번이 1~2점).
        """
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= 0.05:
                continue
            a = msg.angle_min + i * msg.angle_increment + self.offset
            x, y = r * math.cos(a), r * math.sin(a)
            if OBS_X_MIN <= x <= OBS_X_MAX and abs(y) <= OBS_HALF_W:
                pts.append((x, y))
        if not pts:
            return dict(side='', dist=None, y=None, n=0, groups=0)
        pts.sort()
        groups, cur = [], []
        for q in pts:
            if cur and math.hypot(q[0] - cur[-1][0], q[1] - cur[-1][1]) > OBS_GAP:
                groups.append(cur)
                cur = []
            cur.append(q)
        groups.append(cur)
        near = min(groups, key=lambda g: min(q[0] for q in g))
        cy = sum(q[1] for q in near) / len(near)
        return dict(side=('L' if cy > 0 else 'R'),
                    dist=round(min(q[0] for q in near), 3),
                    y=round(cy, 3), n=len(near), groups=len(groups))

    def _on_scan(self, msg):
        # 차로 안 정지물체는 콘 창과 무관하게 매 스캔 낸다 (아래 창 검사보다 먼저).
        obs = self._obstacle(msg)
        obs['stamp'] = time.monotonic()
        obs['wall'] = round(time.time(), 3)
        self.pub_obstacle.publish(String(data=json.dumps(obs)))

        # 켜짐은 진입 트리거가 연 창 안에서만 성립한다. 창이 닫히면 꺼진다.
        if not self.force and time.monotonic() >= self.open_until:
            if self.enabled:
                self.get_logger().info('콘 인식 창 종료 — 끔')
            self.enabled = False
            self.pub_state.publish(String(data='X 진입 대기'))
            return
        self.enabled = True

        # 기존 벽 조준은 차량 중심 보정을 만들 수 없는 프레임의 fallback으로
        # 남긴다. 따라서 새 보정 때문에 추가 미검출/정지가 생기지 않는다.
        original_wall = cone_wall.solve(
            msg.ranges, msg.angle_min, msg.angle_increment, self.offset,
            float(self.get_parameter('wall_bias_cm').value) / 100.0)

        gate = find_cone_gate(
            msg.ranges, msg.angle_min, msg.angle_increment,
            angle_offset=self.offset, config=self.config,
            prev_row_side=self.row_side)
        self.row_side = gate.near_row_side

        center_result = calculate_center_offset_target(
            gate.points,
            vehicle_length_m=float(
                self.get_parameter('vehicle_length_m').value),
            vehicle_width_m=float(
                self.get_parameter('vehicle_width_m').value),
            lidar_to_vehicle_center_m=float(
                self.get_parameter('lidar_to_vehicle_center_m').value),
            lookahead_from_center_m=float(
                self.get_parameter('center_lookahead_m').value),
            boundary_clearance_m=float(
                self.get_parameter('boundary_clearance_m').value),
        )
        if center_result is not None:
            wall_aim = center_result.aim_y_m
            wall_basis = center_result.basis
            wall_left = center_result.left_count
            wall_right = center_result.right_count
            center_meta = {
                'target_x_lidar_m': center_result.target_x_from_lidar_m,
                'target_x_center_m': center_result.target_x_from_center_m,
                'maximum_center_offset_m':
                    center_result.maximum_center_offset_m,
            }
        elif original_wall is not None:
            wall_aim = original_wall['aim_y']
            wall_basis = ('CENTER_OFFSET_FALLBACK_'
                          + str(original_wall['basis']))
            wall_left = len(original_wall['left'])
            wall_right = len(original_wall['right'])
            center_meta = {'fallback': True}
        else:
            wall_aim = None
            wall_basis = ''
            wall_left = 0
            wall_right = 0
            center_meta = {'fallback': True}

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

        if wall_aim is not None:
            text = 'O 벽 좌%d/우%d 조준%+.0fcm (%s)' % (
                wall_left, wall_right, wall_aim * 100, wall_basis)
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
            wall_aim=(None if wall_aim is None else round(wall_aim, 3)),
            wall_basis=wall_basis,
            wall_left=wall_left,
            wall_right=wall_right,
            center_offset=center_meta,
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

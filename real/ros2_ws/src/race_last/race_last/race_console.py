"""터미널 상황판 — ros2 launch 를 친 그 창이 심판용 화면이 된다.

부팅 단계: 필수 노드가 다 뜰 때까지 항목별 대기중/완료 를 보여준다.
주행 단계: 신호 / 차량(좌·우) / 콘 상태를 같은 자리에서 계속 갱신한다.
웹 대시보드와 달리 영상 스트림이 없어 CPU 를 거의 쓰지 않는다.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String

CLEAR = '\x1b[2J\x1b[H'

SIGNAL_KO = {'red': '빨강', 'yellow': '노랑', 'green': '직진',
             'left': '좌회전', 'none': '--'}

# 부팅 화면 순서. (표시 이름, 노드 이름)
BOOT_NODES = [
    ('카메라', 'camera'),
    ('라이다', 'xycar_lidar_node'),
    ('차선(YOLOPv2)', 'yolop_drive'),
    ('신호등', 'traffic_perception_node'),
    ('차량인식', 'vehicle_perception_node'),
    ('콘', 'cone_perception'),
]


class RaceConsole(Node):
    def __init__(self):
        super().__init__('race_console')
        self.t0 = time.time()
        self.image_at = 0.0
        self.scan_at = 0.0
        self.signal = 'none'
        self.veh_dir = 'NONE'
        self.veh_dist = float('inf')
        self.cone = None              # None = 콘 노드 침묵
        self.signal_at = 0.0
        self.veh_at = 0.0
        self.cone_at = 0.0
        self.cone_info = ''
        self.booted = False

        self.create_subscription(Image, '/image_raw', self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_signal, 10)
        self.create_subscription(String, '/yolo/obstacle_dir',
                                 self._on_dir, 10)
        self.create_subscription(Float32, '/yolo/obstacle_dist',
                                 self._on_dist, 10)
        self.create_subscription(String, '/cone/state', self._on_cone, 10)
        self.create_timer(0.2, self._draw)

    # ------------------------------------------------------------ 입력
    def _on_image(self, _msg):
        self.image_at = time.time()

    def _on_scan(self, _msg):
        self.scan_at = time.time()

    def _on_dir(self, msg):
        self.veh_dir = msg.data
        self.veh_at = time.time()

    def _on_dist(self, msg):
        self.veh_dist = float(msg.data)
        self.veh_at = time.time()

    def _on_signal(self, msg):
        try:
            self.signal = json.loads(msg.data).get('state', 'none')
            self.signal_at = time.time()
        except (ValueError, AttributeError):
            self.signal = msg.data or 'none'

    def _on_cone(self, msg):
        data = msg.data or 'X'
        self.cone = 'O' if data[:1] == 'O' else 'X'
        self.cone_info = data[1:].strip()
        self.cone_at = time.time()

    # ------------------------------------------------------------ 그리기
    def _alive(self):
        names = set(self.get_node_names())
        now = time.time()
        # vehicle_perception_node 는 살아 있어도 node list 에 안 잡힐 때가
        # 있다 (2026-08-22 실측, DDS 디스커버리 특성). 데이터 수신도 생존으로 본다.
        data_at = {'traffic_perception_node': self.signal_at,
                   'vehicle_perception_node': self.veh_at,
                   'cone_perception': self.cone_at}
        rows = []
        for label, node in BOOT_NODES:
            up = node in names or (now - data_at.get(node, 0.0)) < 2.0
            if node == 'camera':
                up = up and (now - self.image_at) < 2.0
            if node == 'xycar_lidar_node':
                up = up and (now - self.scan_at) < 2.0
            rows.append((label, up))
        return rows

    def _draw(self):
        rows = self._alive()

        if not self.booted:
            done = all(up for _, up in rows)
            lines = ['══════ RACE 부팅 ══════']
            for label, up in rows:
                lines.append(' %-14s %s' % (label, '완료' if up else '대기중...'))
            lines.append('═══════════════════════')
            if done:
                self.booted = True
                lines.append(' 전 항목 완료 — 주행 준비 끝')
            elif time.time() - self.t0 > 30.0:
                lines.append(' 30초 경과 — 안 뜨는 항목을 확인하라')
            self._flush(lines)
            return

        left = right = 'X'
        if self.veh_dist < 90.0:
            if self.veh_dir == 'LEFT':
                left = '%.1fm' % self.veh_dist
            elif self.veh_dir == 'RIGHT':
                right = '%.1fm' % self.veh_dist
        cone = self.cone if self.cone is not None else 'X'
        note = ('  (%s)' % self.cone_info) if self.cone_info else ''

        lines = [
            '═══════════════════',
            ' 신호 : %s' % SIGNAL_KO.get(self.signal, self.signal),
            '',
            ' 차량 ---',
            '   좌 / %s' % left,
            '   우 / %s' % right,
            '',
            ' 콘 : %s%s' % (cone, note),
            '═══════════════════',
        ]
        dead = [label for label, up in rows if not up]
        if dead:
            lines.append(' !! 죽은 노드: ' + ', '.join(dead))
        self._flush(lines)

    def _flush(self, lines):
        # 내용이 바뀔 때만 다시 그린다 — 0.2초마다 전면 갱신하면 CPU 와
        # systemd 로그를 낭비한다 (2026-08-22, 대시보드 병용 시 특히)
        text = chr(10).join(lines)
        if text == getattr(self, '_last_text', None):
            return
        self._last_text = text
        sys.stdout.write(CLEAR + text + chr(10))
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = RaceConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

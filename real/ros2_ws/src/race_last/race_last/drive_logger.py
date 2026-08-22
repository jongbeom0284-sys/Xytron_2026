"""주행 판단 전수 기록 — 사후 분석에서 추측을 없애기 위한 노드.

2026-08-22 교훈: 재생(run_replay)은 저장된 사진을 새 추적기로 다시 계산하므로
실주행 값과 다르다. frame120 에서 재생은 +1.5, 실제 모터는 -21.5 였고
원인을 며칠 추측해도 확정할 수 없었다. 그래서 주행 중 판단 근거를 전부 남긴다.

/kobot_yolop/status(판단) + /xycar_motor(명령) + /traffic_light/state(신호)
+ /vehicle/state(앞차) + /cone/state(콘) 을 한 CSV 에 시각과 함께 적는다.
모터 텔레메트리는 motor_logger 가 따로 남긴다 (wall 시각으로 조인 가능).

저장: ~/drive_logs/drivelog_YYYYmmdd_HHMMSS.csv
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

OUT_DIR = '/home/xytron/drive_logs'

COLS = [
    'wall', 't', 'mode', 'basis', 'steer', 'speed', 'raw_steer',
    'error_far', 'error_near', 'center_offset_px', 'fit_rows',
    'yellow_blobs', 'white_blobs', 'confidence', 'latency_ms',
    'camera_stale', 'armed', 'enabled', 'fault',
    'straight_only', 'straight_hold', 'signal_lock_s',
    'traffic_state', 'traffic_stop', 'vehicle_needed',
    'avoid_side', 'avoid_offset_px',
    'cone_active', 'cone_approach', 'cone_state',
    'vehicle_state', 'vehicle_boxes', 'cmd_steer', 'cmd_speed',
]


class DriveLogger(Node):
    def __init__(self):
        super().__init__('drive_logger')
        self._fh = None
        self._t0 = None
        self._rows = 0
        self._traffic = ''
        self._vehicle = ''
        self._vehicle_n = 0
        self._cone = ''
        self._cmd = (0.0, 0.0)
        self.create_subscription(String, '/kobot_yolop/status',
                                 self._on_status, 20)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_traffic, 10)
        self.create_subscription(String, '/vehicle/state',
                                 self._on_vehicle, 10)
        self.create_subscription(String, '/cone/state', self._on_cone, 10)
        self.create_subscription(Float32MultiArray, '/xycar_motor',
                                 self._on_cmd, 20)
        self.get_logger().info('주행 로거 대기 — 주행 상태가 오면 기록 시작')

    def _on_traffic(self, msg):
        """none 은 '신호등이 안 보임' 이지 '신호가 없음' 이 아니다.
        마지막 유효 판정(red/yellow/green/left)을 유지해 기록한다."""
        try:
            state = str(json.loads(msg.data).get('state', ''))
        except (ValueError, AttributeError, TypeError):
            return
        if state and state != 'none':
            self._traffic = state

    def _on_vehicle(self, msg):
        # 페이로드는 {"car": {"dir","dist"}, "boxes": [...], "n": N} 이다.
        # 예전에는 최상위에서 dir/dist 를 찾아 항상 빈 값이 남았고, 그래서
        # 로그만으로는 '언제 차가 보였나' 를 확인할 수 없었다 (0823).
        try:
            d = json.loads(msg.data)
            car = d.get('car') or {}
            self._vehicle = '%s/%s' % (car.get('dir', ''), car.get('dist', ''))
            self._vehicle_n = int(d.get('n', 0) or 0)
        except (ValueError, AttributeError, TypeError):
            self._vehicle = ''
            self._vehicle_n = 0

    def _on_cone(self, msg):
        self._cone = (msg.data or '').replace(',', ' ')[:40]

    def _on_cmd(self, msg):
        if len(msg.data) >= 2:
            self._cmd = (float(msg.data[0]), float(msg.data[1]))

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        now = time.monotonic()
        if self._fh is None:
            os.makedirs(OUT_DIR, exist_ok=True)
            path = os.path.join(
                OUT_DIR, time.strftime('drivelog_%Y%m%d_%H%M%S.csv'))
            self._fh = open(path, 'w', buffering=1)
            self._fh.write(','.join(COLS) + '\n')
            self._t0 = now
            self.get_logger().info('기록 시작: %s' % path)

        d['wall'] = round(time.time(), 3)
        d['t'] = round(now - self._t0, 3)
        d['traffic_state'] = self._traffic
        d['vehicle_state'] = self._vehicle
        d['vehicle_boxes'] = self._vehicle_n
        d['cone_state'] = self._cone
        d['cmd_steer'], d['cmd_speed'] = self._cmd

        out = []
        for k in COLS:
            v = d.get(k, '')
            if isinstance(v, bool):
                v = int(v)
            elif isinstance(v, float):
                v = round(v, 4)
            elif v is None:
                v = ''
            out.append(str(v).replace(',', ' '))
        self._fh.write(','.join(out) + '\n')
        self._rows += 1
        if self._rows % 500 == 0:
            self.get_logger().info('기록 %d행' % self._rows)


def main(args=None):
    rclpy.init(args=args)
    node = DriveLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._fh:
            node._fh.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

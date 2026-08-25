"""터미널 상황판 — ros2 launch 를 친 그 창이 심판용 화면이 된다.

부팅 단계: 필수 노드가 다 뜰 때까지 항목별 대기중/완료 를 보여준다.
주행 단계: 미션별로 한 줄씩, 키워드만.

두 가지 원칙이 있다.

1) 원시 토픽이 아니라 /kobot_yolop/status 를 본다.
   /traffic_light/state 만 보면 신호등을 지나는 순간 none 이 되어 '무엇을 보고
   달리는 중인지' 가 화면에서 사라진다. 확정은 노드가 하고, 노드가 그 결과를
   status.confirmed 로 실어 준다.

2) 미션은 **확정 순간의 값을 붙잡아 두고**, 그 미션이 끝날 때 지운다.
   라이다 관측은 매 스캔 바뀌고 회피는 프레임마다 흔들려서, 그대로 띄우면
   글자가 계속 바뀌어 읽을 수가 없다. 확정 시점 값 + 경과 시간만 보인다.

웹 대시보드와 달리 영상 스트림이 없어 CPU 를 거의 쓰지 않는다.
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

CLEAR = '\x1b[2J\x1b[H'

SIGNAL_KO = {'red': '빨강', 'yellow': '노랑', 'green': '초록',
             'left': '좌회전', 'none': '--', '': '--'}
BASIS_KO = {'white_center': '흰색 중앙', 'yellow': '노란 중앙선',
            'cone': '라이다 콘 벽', 'cone_approach': '차선 (콘 접근)',
            'avoid_left': '좌측 흰선 (회피)', 'avoid_right': '우측 흰선 (회피)',
            'hold': '직전 값 유지', 'none': '차선 못 찾음', '': '--'}
CONFIRM_KO = {'green': '직진확정', 'left': '좌회전확정', '': '미확정'}

BOOT_NODES = [
    ('카메라', 'camera'),
    ('라이다', 'xycar_lidar_node'),
    ('차선(YOLOPv2)', 'yolop_drive'),
    ('신호등', 'traffic_perception_node'),
    ('차량인식', 'vehicle_perception_node'),
    ('콘', 'cone_perception'),
]

# 미션 순서. 코스: 신호 -> 콘 -> 정지차 -> 이동차 -> S자
STAGES = ['신호', '콘', '정지차', '이동차', 'S자']


class Latch:
    """확정 순간의 문구를 붙잡아 둔다. 끝나면 스스로 비운다."""

    def __init__(self):
        self.text = ''
        self.at = 0.0

    def set(self, text):
        if not self.text:
            self.text = text
            self.at = time.time()

    def clear(self):
        self.text = ''

    def show(self):
        if not self.text:
            return ''
        return '%s (%.0fs)' % (self.text, time.time() - self.at)


class RaceConsole(Node):
    def __init__(self):
        super().__init__('race_console')
        self.t0 = time.time()
        self.image_at = 0.0
        self.scan_at = 0.0
        self.signal = 'none'
        self.signal_at = 0.0
        self.cone_at = 0.0
        self.st = {}                  # 마지막 /kobot_yolop/status
        self.st_at = 0.0
        self.booted = False

        self.obs_latch = Latch()      # 정지차 — 라이다가 확정한 값
        self.avoid_latch = Latch()    # 회피 — 방향이 선 순간의 값
        self.cone_latch = Latch()     # 콘 — 모드가 켜진 순간

        # 카메라 생존만 본다. 원본(921KB×30Hz)이 아니라 압축(~80KB)을 받는다.
        self.create_subscription(CompressedImage, '/image_raw/compressed',
                                 self._on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_signal, 10)
        self.create_subscription(String, '/cone/state', self._on_cone, 10)
        self.create_subscription(String, '/kobot_yolop/status',
                                 self._on_status, 10)
        self.create_timer(0.25, self._draw)

    # ------------------------------------------------------------ 입력
    def _on_image(self, _msg):
        self.image_at = time.time()

    def _on_scan(self, _msg):
        self.scan_at = time.time()

    def _on_signal(self, msg):
        try:
            self.signal = json.loads(msg.data).get('state', 'none')
        except (ValueError, AttributeError, TypeError):
            self.signal = msg.data or 'none'
        self.signal_at = time.time()

    def _on_cone(self, _msg):
        self.cone_at = time.time()

    def _on_status(self, msg):
        try:
            self.st = json.loads(msg.data)
            self.st_at = time.time()
        except (ValueError, TypeError):
            return
        self._update_latches()

    # ------------------------------------------------------------ 래치
    def _update_latches(self):
        s = self.st
        num = lambda k, d=0.0: float(s.get(k) or d)      # noqa: E731
        here = self._stage()
        track = '  '.join(('\u25b6%s' % x) if x == here else ('  %s' % x)
                          for x in STAGES)
        basis = BASIS_KO.get(s.get('basis') or '', s.get('basis') or '--')
        conf = CONFIRM_KO.get(s.get('confirmed', ''), s.get('confirmed'))

        rows = []
        if here == '신호':
            # 신호만 본다. 아직 아무 미션도 안 시작했다.
            line = SIGNAL_KO.get(self.signal, self.signal)
            if s.get('traffic_stop'):
                line += ' · 정지 중'
            rows.append(('신호등', line))
            rows.append(('확정', conf))
            if num('signal_lock_s') > 0:
                rows.append(('신호 잠금', '%.0f초 남음 (그동안 안 읽음)'
                             % num('signal_lock_s')))

        elif here == '콘':
            rows.append(('신호', conf))
            rows.append(('차선', basis))
            if s.get('cone_active'):
                rows.append(('콘 모드', self.cone_latch.show() or 'ON'))
            elif s.get('cone_triggered'):
                rows.append(('주황 감지', '트리거 발사됨 — 콘 진입 대기'))
            elif num('cone_arm_s') > 0:
                rows.append(('주황 감지', '%.1f초 뒤 시작' % num('cone_arm_s')))
            else:
                rows.append(('주황 감지', '보는 중 (%d px)'
                             % int(s.get('cone_trig_px') or 0)))
            if s.get('cone_approach'):
                rows.append(('콘', '통로 보임 — 감속'))

        elif here == '정지차':
            if s.get('obs_armed'):
                rows.append(('라이다', '감시 중 · %.0f초 남음' % num('obs_arm_s')))
            elif num('obs_hold_s') > 0:
                rows.append(('라이다', '확정 · 회피 %.1f초 남음' % num('obs_hold_s')))
            else:
                rows.append(('라이다', '끔'))
            if self.obs_latch.text:
                rows.append(('관측', self.obs_latch.show()))
            if self.avoid_latch.text:
                rows.append(('회피', self.avoid_latch.show()))
            rows.append(('차선', basis))

        elif here == '이동차':
            veh = '보는 중' if s.get('vehicle_needed') else '끔'
            if num('vehicle_window_s') > 0:
                veh += ' · %.0f초 남음' % num('vehicle_window_s')
            rows.append(('욜로', veh))
            if int(s.get('car_inside') or 0):
                rows.append(('차로 안', '%d개' % int(s['car_inside'])))
            if s.get('vehicle_passed'):
                rows.append(('추월', '완료'))
            if self.avoid_latch.text:
                rows.append(('회피', self.avoid_latch.show()))
            rows.append(('차선', basis))

        else:                                   # S자
            rows.append(('신호', conf))
            rows.append(('차선', basis))
            if self.avoid_latch.text:
                rows.append(('회피', self.avoid_latch.show()))

        lines = ['\u2550' * 8 + ' RACE ' + '\u2550' * 8, ' %s' % track, '']
        for label, value in rows:
            lines.append(' %-9s %s' % (label, value))
        lines.append('')
        lines.append(' %.2f m/s  \uc870\ud5a5 %+d  %s %s' % (
            num('speed') * 0.08, int(round(num('steer'))),
            'ARMED' if s.get('armed') else 'safe',
            'ON' if s.get('enabled') else 'off'))
        lines.append('\u2550' * 22)

        if s.get('fault'):
            lines.append(' !! \uace0\uc7a5: %s' % s['fault'])
        if s.get('camera_stale'):
            lines.append(' !! \uce74\uba54\ub77c \ub04a\uae40')
        dead = [label for label, up in rows2 if not up]
        if dead:
            lines.append(' !! \uc8fd\uc740 \ub178\ub4dc: ' + ', '.join(dead))
        self._flush(lines)

    def _flush(self, lines):
        # 내용이 바뀔 때만 다시 그린다 — 0.25초마다 전면 갱신하면 CPU 와
        # systemd 로그를 낭비한다.
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

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

        # 정지차 — 라이다가 확정하면(유지 시간이 서면) 그 값을 붙잡는다.
        # 무장만 된 상태에서는 관측이 매 스캔 바뀌므로 띄우지 않는다.
        if num('obs_hold_s') > 0.0:
            if s.get('obs_side'):
                self.obs_latch.set('%s %.2fm %s점' % (
                    '우측' if s['obs_side'] == 'R' else '좌측',
                    num('obs_dist'), s.get('obs_n')))
        elif not s.get('obs_armed'):
            self.obs_latch.clear()

        # 회피 — 방향이 선 순간 고정, 풀리면 지운다.
        if s.get('avoid_side'):
            self.avoid_latch.set('%s 흰선' % (
                '좌측' if s['avoid_side'] == 'left' else '우측'))
        else:
            self.avoid_latch.clear()

        # 콘 — 모드가 켜진 순간 고정, 꺼지면 지운다.
        if s.get('cone_active'):
            self.cone_latch.set('모드 ON')
        else:
            self.cone_latch.clear()

    # ------------------------------------------------------------ 판단
    def _stage(self):
        """지금 어느 미션인가. 코스 순서대로 앞에서부터 본다."""
        s = self.st
        if s.get('traffic_stop') or not s.get('confirmed'):
            return '신호'
        # 좌회전 바퀴는 콘·차량이 없다. 바로 S자다.
        if s.get('confirmed') == 'left':
            return 'S자'
        if s.get('cone_active') or s.get('cone_approach'):
            return '콘'
        # 아직 콘을 안 지났으면 콘을 향해 가는 중이다 (초록 확정 ~ 콘 탈출).
        if not s.get('cone_exited'):
            return '콘'
        if s.get('obs_armed') or float(s.get('obs_hold_s') or 0.0) > 0.0:
            return '정지차'
        if s.get('vehicle_needed'):
            return '이동차'
        return 'S자'

    def _alive(self):
        names = set(self.get_node_names())
        now = time.time()
        # vehicle_perception_node 는 살아 있어도 node list 에 안 잡힐 때가
        # 있다 (DDS 디스커버리 특성). 데이터 수신도 생존으로 본다.
        data_at = {'traffic_perception_node': self.signal_at,
                   'cone_perception': self.cone_at,
                   'yolop_drive': self.st_at}
        rows = []
        for label, node in BOOT_NODES:
            up = node in names or (now - data_at.get(node, 0.0)) < 2.0
            if node == 'camera':
                up = up and (now - self.image_at) < 2.0
            if node == 'xycar_lidar_node':
                up = up and (now - self.scan_at) < 2.0
            rows.append((label, up))
        return rows

    # ------------------------------------------------------------ 그리기
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

        s = self.st
        num = lambda k, d=0.0: float(s.get(k) or d)      # noqa: E731
        here = self._stage()
        track = '  '.join(('▶%s' % x) if x == here else ('  %s' % x)
                          for x in STAGES)

        # 신호 — 확정값을 계속 띄운다. 잠금은 정수 초라 자주 안 바뀐다.
        sig = [CONFIRM_KO.get(s.get('confirmed', ''), s.get('confirmed'))]
        if s.get('traffic_stop'):
            sig.append('정지')
        if num('signal_lock_s') > 0:
            sig.append('잠금 %.0fs' % num('signal_lock_s'))

        # 콘
        if self.cone_latch.text:
            cone = self.cone_latch.show()
        elif s.get('cone_approach'):
            cone = '접근'
        elif s.get('cone_exited'):
            cone = '탈출 완료'
        elif s.get('cone_triggered'):
            cone = '트리거 발사됨'
        else:
            cone = '대기 (트리거 무장)'

        # 정지차
        if self.obs_latch.text:
            obs = self.obs_latch.show()
        elif s.get('obs_armed'):
            obs = '라이다 감시중 %.0fs' % num('obs_arm_s')
        else:
            obs = '--'

        # 이동차
        if s.get('vehicle_needed'):
            veh = '욜로 감시중'
            if num('vehicle_window_s') > 0:
                veh += ' %.0fs' % num('vehicle_window_s')
        elif s.get('vehicle_passed'):
            veh = '추월 완료'
        else:
            veh = '--'

        avoid = self.avoid_latch.show() or '--'

        lines = [
            '═════════════ RACE ═════════════',
            ' %s' % track,
            '',
            ' 신호   %s' % ' · '.join(sig),
            ' 콘     %s' % cone,
            ' 정지차 %s' % obs,
            ' 이동차 %s' % veh,
            ' 회피   %s' % avoid,
            '',
            ' %.2f m/s  조향 %+d  %s %s' % (
                num('speed') * 0.08, int(round(num('steer'))),
                'ARMED' if s.get('armed') else 'safe',
                'ON' if s.get('enabled') else 'off'),
            '════════════════════════════════',
        ]
        if s.get('fault'):
            lines.append(' !! 고장: %s' % s['fault'])
        if s.get('camera_stale'):
            lines.append(' !! 카메라 끊김')
        dead = [label for label, up in rows if not up]
        if dead:
            lines.append(' !! 죽은 노드: ' + ', '.join(dead))
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

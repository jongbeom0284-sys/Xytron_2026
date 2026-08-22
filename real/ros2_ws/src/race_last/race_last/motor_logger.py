"""모터 텔레메트리 상시 기록 — 주행 진단용 (2026-08-22).

/sensors/core (VESC 실측) + /xycar_motor (명령) 를 CSV 로 남긴다.
"모터가 문제인지 전압이 문제인지 토크가 문제인지" 를 주행 후 바로 가리기 위한
데이터다. 모터 스택이 떠서 첫 메시지가 올 때 파일을 만들고, 그 전에는 조용하다.

저장: /home/xytron/motor_logs/motorlog_YYYYmmdd_HHMMSS.csv (런치당 1개)
열: t(초), erpm, mps(실제속도), duty, v_in, i_motor(토크), i_in, temp_fet,
    fault, cmd_steer, cmd_speed_units
"""
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from vesc_msgs.msg import VescStateStamped

OUT_DIR = '/home/xytron/motor_logs'
ERPM_PER_MPS = 4614.0


class MotorLogger(Node):
    def __init__(self):
        super().__init__('motor_logger')
        self._fh = None
        self._t0 = None
        self._cmd = (0.0, 0.0)
        self._rows = 0
        self.create_subscription(VescStateStamped, '/sensors/core',
                                 self._on_core, 50)
        self.create_subscription(Float32MultiArray, '/xycar_motor',
                                 self._on_cmd, 10)
        self.get_logger().info('모터 로거 대기 — 모터 스택이 뜨면 기록 시작')

    def _on_cmd(self, msg):
        if len(msg.data) >= 2:
            self._cmd = (float(msg.data[0]), float(msg.data[1]))

    def _on_core(self, msg):
        now = time.monotonic()
        if self._fh is None:
            os.makedirs(OUT_DIR, exist_ok=True)
            path = os.path.join(
                OUT_DIR,
                time.strftime('motorlog_%Y%m%d_%H%M%S.csv'))
            self._fh = open(path, 'w', buffering=1)
            self._fh.write('t,erpm,mps,duty,v_in,i_motor,i_in,'
                           'temp_fet,fault,cmd_steer,cmd_speed\n')
            self._t0 = now
            self.get_logger().info('기록 시작: %s' % path)
        s = msg.state
        erpm = float(s.speed)
        self._fh.write(
            '%.3f,%.0f,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%d,%.1f,%.1f\n'
            % (now - self._t0, erpm, erpm / ERPM_PER_MPS,
               float(s.duty_cycle), float(s.voltage_input),
               float(s.current_motor), float(s.current_input),
               float(getattr(s, 'temp_fet',
                             getattr(s, 'temperature_pcb', 0.0))),
               int(s.fault_code),
               self._cmd[0], self._cmd[1]))
        self._rows += 1
        if self._rows % 3000 == 0:
            self.get_logger().info('기록 %d행 (%.0f초)'
                                   % (self._rows, now - self._t0))


def main(args=None):
    rclpy.init(args=args)
    node = MotorLogger()
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

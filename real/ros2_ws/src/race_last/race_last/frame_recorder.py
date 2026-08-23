"""주행 프레임을 통째로 남긴다 — 문제 생긴 시각으로 사진을 찾아보려고.

대시보드가 하던 일을 떼어 온 것이다. race_day.launch.py 는 대시보드를 끄고
띄우므로 그대로 두면 사진이 한 장도 안 남고, 나중에 "그때 뭘 본 거냐" 를
확인할 방법이 없다 (0824 01:33 의 11초짜리 회피가 그랬다).

싸게 만든 이유:
  /image_raw/compressed 는 **이미 JPEG** 다. 디코드도 인코드도 하지 않고
  바이트를 그대로 파일에 쓴다. cv2 를 아예 안 부른다.

남기는 것:
  <record_root>/<세션>/000000.jpg ...
  <record_root>/<세션>/frames.csv   file,wall,rt   ← 시각으로 찾을 때 쓴다
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


class FrameRecorder(Node):
    def __init__(self):
        super().__init__('frame_recorder')
        self.declare_parameter('record_root',
                               os.path.expanduser('~/recordings/camera'))
        self.declare_parameter('topic', '/image_raw/compressed')
        # 카메라는 30Hz 지만 그대로 다 쓰면 2.4MB/s 다. 되짚어 보는 데는
        # 10Hz 로 충분하다 (기존 대시보드 기록도 8Hz 정도였다).
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('enabled', True)

        self.root = str(self.get_parameter('record_root').value)
        self.rate = max(0.5, float(self.get_parameter('rate_hz').value))
        self.min_gap = 1.0 / self.rate
        self.enabled = bool(self.get_parameter('enabled').value)

        self.session = time.strftime('%Y%m%d_%H%M%S') + '_rec'
        self.dir = os.path.join(self.root, self.session)
        self.n = 0
        self.last = 0.0
        self.t0 = None
        self.index = None
        self.writer = None

        if self.enabled:
            os.makedirs(self.dir, exist_ok=True)
            self.index = open(os.path.join(self.dir, 'frames.csv'),
                              'w', buffering=1, newline='')
            self.writer = csv.writer(self.index)
            self.writer.writerow(['file', 'wall', 'rt'])

        self.create_subscription(
            CompressedImage, str(self.get_parameter('topic').value),
            self._on_image, qos_profile_sensor_data)
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(30.0, self._report)
        self.get_logger().warning(
            '프레임 기록 %s — %s (%.0fHz)'
            % ('시작' if self.enabled else '꺼짐', self.dir, self.rate))

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                if self.enabled and self.index is None:
                    os.makedirs(self.dir, exist_ok=True)
                    self.index = open(os.path.join(self.dir, 'frames.csv'),
                                      'w', buffering=1, newline='')
                    self.writer = csv.writer(self.index)
                    self.writer.writerow(['file', 'wall', 'rt'])
                self.get_logger().warning(
                    '프레임 기록 %s' % ('켬' if self.enabled else '끔'))
            elif p.name == 'rate_hz':
                self.rate = max(0.5, float(p.value))
                self.min_gap = 1.0 / self.rate
                self.get_logger().warning('기록 %.0fHz 로 변경' % self.rate)
        return SetParametersResult(successful=True)

    def _on_image(self, message):
        if not self.enabled:
            return
        now = time.time()
        if now - self.last < self.min_gap:
            return
        self.last = now
        if self.t0 is None:
            self.t0 = now
        name = '%06d.jpg' % self.n
        try:
            # 이미 JPEG 다. 바이트를 그대로 쓴다 — 디코드/인코드 없음.
            with open(os.path.join(self.dir, name), 'wb') as fh:
                fh.write(bytes(message.data))
        except OSError as exc:
            self.get_logger().error('프레임 저장 실패: %s' % exc)
            self.enabled = False
            return
        self.writer.writerow([name, round(now, 3), round(now - self.t0, 3)])
        self.n += 1

    def _report(self):
        if self.enabled and self.n:
            self.get_logger().info(
                '프레임 %d장 (%s)' % (self.n, self.session))

    def destroy_node(self):
        if self.index is not None:
            try:
                self.index.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrameRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

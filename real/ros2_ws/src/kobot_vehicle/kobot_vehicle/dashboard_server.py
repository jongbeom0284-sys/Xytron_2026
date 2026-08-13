import csv
import datetime as dt
import json
import math
import os
import signal
import subprocess
import threading
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


class DeviceController:
    DEVICE_INFO = {
        'camera': ('/dev/videoCAM', 'usb_cam_node_exe', 'usb_cam', 'usb_cam_node_exe', 'camera.yaml', ['-r', '__node:=camera']),
        'lidar': ('/dev/ttyLIDAR', 'xycar_lidar_node', 'xycar_lidar', 'xycar_lidar_node', 'lidar.yaml', ['-r', '__node:=xycar_lidar_node']),
        'ultrasonic': ('/dev/ttySonic', 'ultrasonic_node', 'kobot_vehicle', 'ultrasonic_node', 'ultrasonic.yaml', []),
        'imu': ('/dev/ttyIMU', 'imu_node', 'kobot_vehicle', 'imu_node', 'imu.yaml', []),
    }

    def __init__(self, config_dir):
        self.config_dir = Path(config_dir)
        self.log_dir = Path.home() / '.local' / 'state' / 'kobot'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.started = {}
        self.lock = threading.Lock()

    @staticmethod
    def _processes():
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / 'cmdline').read_bytes()
                args = [part.decode(errors='replace') for part in raw.split(b'\0') if part]
            except (OSError, PermissionError):
                continue
            if args:
                yield int(entry.name), args

    def pids(self, name):
        executable = self.DEVICE_INFO[name][1]
        return [pid for pid, args in self._processes() if any(Path(arg).name == executable for arg in args)]

    def status(self):
        result = {}
        for name, info in self.DEVICE_INFO.items():
            pids = self.pids(name)
            result[name] = {'usb': os.path.exists(info[0]), 'running': bool(pids), 'pid': pids[0] if pids else None}
        return result

    def connect(self, name):
        with self.lock:
            info = self.DEVICE_INFO[name]
            if self.pids(name):
                return '이미 실행 중입니다'
            if not os.path.exists(info[0]):
                raise RuntimeError('USB 장치가 인식되지 않았습니다')
            previous = self.started.pop(name, None)
            if previous:
                previous[1].close()
            command = ['ros2', 'run', info[2], info[3], '--ros-args', *info[5], '--params-file', str(self.config_dir / info[4])]
            log_handle = open(self.log_dir / f'{name}.log', 'ab', buffering=0)
            process = subprocess.Popen(command, stdout=log_handle, stderr=subprocess.STDOUT, start_new_session=True)
            self.started[name] = (process, log_handle)
            time.sleep(0.35)
            if process.poll() is not None:
                log_handle.close()
                raise RuntimeError('장치 프로그램이 시작되지 않았습니다')
            return '연결했습니다'

    def disconnect(self, name):
        with self.lock:
            pids = self.pids(name)
            if not pids:
                return '이미 중지되어 있습니다'
            for pid in pids:
                try:
                    os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and self.pids(name):
                time.sleep(0.1)
            for pid in self.pids(name):
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            tracked = self.started.pop(name, None)
            if tracked:
                tracked[1].close()
            return '연결을 해제했습니다'

    def reconnect(self, name):
        self.disconnect(name)
        time.sleep(0.4)
        self.connect(name)
        return '재연결했습니다'

    def close(self):
        for name in list(self.started):
            try:
                self.disconnect(name)
            except Exception:
                pass


class MotorController:
    SPEED_MIN_MPS = 0.20
    # FW 6.5 motor limit is 10000 ERPM and speed_to_erpm_gain is 4614.
    SPEED_MAX_MPS = 2.16
    DURATION_MIN_SEC = 0.5
    DURATION_MAX_SEC = 300.0
    SPEED_SCALE = 0.08
    PROCESS_NAMES = {'vesc_driver_node', 'ackermann_to_vesc_node', 'motor_adapter'}
    STEERING_MODES = {'center', 'left', 'right', 'sweep'}

    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Float32MultiArray, '/xycar_motor', 10)
        self.log_dir = Path.home() / '.local' / 'state' / 'kobot'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.process = None
        self.log_handle = None
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.test_thread = None
        self.active = False
        self.speed_mps = 0.0
        self.duration_sec = 0.0
        self.started_at = 0.0
        self.steering = 'center'
        self.last_result = '대기 중'
        self.error = None

    @staticmethod
    def _processes():
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / 'cmdline').read_bytes()
                args = [part.decode(errors='replace') for part in raw.split(b'\0') if part]
            except (OSError, PermissionError):
                continue
            if args:
                yield int(entry.name), args

    def stack_pids(self):
        result = {}
        for pid, args in self._processes():
            names = {Path(arg).name for arg in args}
            for name in self.PROCESS_NAMES:
                if name in names:
                    result[name] = pid
        return result

    def stack_running(self):
        return self.PROCESS_NAMES.issubset(self.stack_pids())

    def stack_state(self):
        pids = self.stack_pids()
        if self.PROCESS_NAMES.issubset(pids):
            return 'running'
        return 'partial' if pids else 'stopped'

    def ensure_stack(self):
        if not os.path.exists('/dev/ttyMOTOR'):
            raise RuntimeError('VESC USB가 인식되지 않았습니다')
        pids = self.stack_pids()
        if self.PROCESS_NAMES.issubset(pids):
            return
        if pids:
            missing = ', '.join(sorted(self.PROCESS_NAMES - set(pids)))
            raise RuntimeError(f'모터 프로그램 일부만 실행 중입니다. 누락: {missing}')
        if self.process and self.process.poll() is None:
            raise RuntimeError('모터 프로그램을 시작하는 중입니다')
        if self.log_handle:
            self.log_handle.close()
        self.log_handle = open(self.log_dir / 'motor-dashboard.log', 'ab', buffering=0)
        self.process = subprocess.Popen(
            ['ros2', 'launch', 'kobot_vehicle', 'motor.launch.py', 'armed:=true'],
            stdout=self.log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise RuntimeError('모터 프로그램 시작에 실패했습니다')
            if self.stack_running():
                time.sleep(0.5)
                self.publish_zero(0.4)
                return
            time.sleep(0.1)
        raise RuntimeError('모터 프로그램 연결 시간이 초과되었습니다')

    def _signal_stack(self, sig):
        pids = set(self.stack_pids().values())
        if self.process and self.process.poll() is None:
            pids.add(self.process.pid)
        process_group = os.getpgrp()
        groups = set()
        direct = set()
        for pid in pids:
            try:
                group = os.getpgid(pid)
            except ProcessLookupError:
                continue
            if group == process_group:
                direct.add(pid)
            else:
                groups.add(group)
        for group in groups:
            try:
                os.killpg(group, sig)
            except ProcessLookupError:
                pass
        for pid in direct:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def _stop_stack(self):
        self._signal_stack(signal.SIGINT)
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline and self.stack_pids():
            time.sleep(0.1)
        if self.stack_pids():
            self._signal_stack(signal.SIGTERM)
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and self.stack_pids():
                time.sleep(0.1)
        if self.stack_pids():
            raise RuntimeError('기존 모터 프로그램을 종료하지 못했습니다')
        if self.process:
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                    self.process.wait(timeout=1)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
            self.process = None
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None

    def restart(self):
        with self.lock:
            if self.active:
                raise RuntimeError('시간 주행 중에는 모터 스택을 다시 실행할 수 없습니다')
            if not os.path.exists('/dev/ttyMOTOR'):
                raise RuntimeError('VESC USB가 인식되지 않았습니다')
            self.stop_event.set()
            self.publish_zero(0.4)
            self._stop_stack()
            time.sleep(0.4)
            self.ensure_stack()
            self.last_result = '모터 스택 다시 실행됨'
            self.error = None
            return self.status()

    def publish(self, angle, speed_mps):
        message = Float32MultiArray()
        message.data = [float(angle), float(speed_mps) / self.SPEED_SCALE]
        self.publisher.publish(message)

    def publish_zero(self, duration=0.6):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publish(0.0, 0.0)
            time.sleep(0.05)

    def arm(self):
        self.ensure_stack()
        self.publish_zero(0.4)
        return self.status()

    def _validate(self, speed_mps, duration_sec, steering):
        if not math.isfinite(speed_mps) or not self.SPEED_MIN_MPS <= speed_mps <= self.SPEED_MAX_MPS:
            raise ValueError(f'속도는 {self.SPEED_MIN_MPS:.2f}~{self.SPEED_MAX_MPS:.2f} m/s로 입력하세요')
        if not math.isfinite(duration_sec) or not self.DURATION_MIN_SEC <= duration_sec <= self.DURATION_MAX_SEC:
            raise ValueError(f'시간은 {self.DURATION_MIN_SEC:.1f}~{self.DURATION_MAX_SEC:.0f}초로 입력하세요')
        if steering not in self.STEERING_MODES:
            raise ValueError('알 수 없는 조향 방식입니다')

    @staticmethod
    def steering_angle(mode, elapsed):
        if mode == 'left':
            return -50.0
        if mode == 'right':
            return 50.0
        if mode == 'sweep':
            return -50.0 * math.sin((2.0 * math.pi * elapsed) / 4.0)
        return 0.0

    def start_test(self, speed_mps, duration_sec, steering):
        speed_mps = float(speed_mps)
        duration_sec = float(duration_sec)
        self._validate(speed_mps, duration_sec, steering)
        with self.lock:
            if self.active:
                raise RuntimeError('이미 시간 주행 테스트가 실행 중입니다')
            self.ensure_stack()
            self.stop_event.clear()
            self.active = True
            self.speed_mps = speed_mps
            self.duration_sec = duration_sec
            self.started_at = time.monotonic()
            self.steering = steering
            self.last_result = '실행 중'
            self.error = None
            self.test_thread = threading.Thread(target=self._run_test, daemon=True)
            self.test_thread.start()
        return self.status()

    def _run_test(self):
        try:
            self.publish_zero(0.5)
            start = time.monotonic()
            while not self.stop_event.is_set():
                elapsed = time.monotonic() - start
                if elapsed >= self.duration_sec:
                    break
                self.publish(self.steering_angle(self.steering, elapsed), self.speed_mps)
                time.sleep(0.05)
            self.last_result = '사용자 정지' if self.stop_event.is_set() else '시간 완료'
        except Exception as error:
            self.error = str(error)
            self.last_result = '실행 오류'
            self.node.get_logger().error(f'Motor test failed: {error}')
        finally:
            self.publish_zero(1.0)
            with self.lock:
                self.active = False
                self.speed_mps = 0.0

    def stop(self):
        self.stop_event.set()
        self.publish_zero(1.0)
        self.last_result = '즉시 정지'
        return self.status()

    def status(self):
        pids = self.stack_pids()
        missing = sorted(self.PROCESS_NAMES - set(pids))
        remaining = 0.0
        if self.active:
            remaining = max(0.0, self.duration_sec - (time.monotonic() - self.started_at))
        return {
            'usb': os.path.exists('/dev/ttyMOTOR'),
            'stack_running': self.PROCESS_NAMES.issubset(pids),
            'stack_state': 'running' if self.PROCESS_NAMES.issubset(pids) else ('partial' if pids else 'stopped'),
            'processes': pids,
            'missing_processes': missing,
            'active': self.active,
            'speed_mps': self.speed_mps,
            'duration_sec': self.duration_sec,
            'remaining_sec': round(remaining, 1),
            'steering': self.steering,
            'result': self.last_result,
            'error': self.error,
            'limits': {
                'speed_min_mps': self.SPEED_MIN_MPS,
                'speed_max_mps': self.SPEED_MAX_MPS,
                'duration_min_sec': self.DURATION_MIN_SEC,
                'duration_max_sec': self.DURATION_MAX_SEC,
            },
        }

    def close(self):
        self.stop()
        if self.process and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
                self.process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        if self.log_handle:
            self.log_handle.close()


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_server')
        share = Path(get_package_share_directory('kobot_vehicle'))
        self.dashboard_dir = share / 'dashboard'
        self.devices = DeviceController(share / 'config')
        self.motor = MotorController(self)
        self.record_root = Path(os.environ.get('KOBOT_RECORDINGS_DIR', str(Path.home() / 'recordings' / 'camera')))
        self.record_lock = threading.Lock()
        self.recording = False
        self.record_path = None
        self.last_path = None
        self.record_manifest = None
        self.record_writer = None
        self.frame_count = 0
        self.last_saved_at = 0.0
        self.record_error = None
        self.create_subscription(Image, '/image_raw', self.on_image, qos_profile_sensor_data)
        self.httpd = ThreadingHTTPServer(('0.0.0.0', 8000), self._handler_class())
        self.httpd.dashboard_node = self
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info('Dashboard and device API listening on port 8000')

    def _handler_class(self):
        dashboard_dir = str(self.dashboard_dir)

        class Handler(DashboardRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=dashboard_dir, **kwargs)

        return Handler

    def recording_status(self):
        with self.record_lock:
            return {'active': self.recording, 'path': str(self.record_path) if self.record_path else None,
                    'last_path': str(self.last_path) if self.last_path else None,
                    'frames': self.frame_count, 'error': self.record_error}

    def start_recording(self):
        with self.record_lock:
            if self.recording:
                return self.recording_status_unlocked()
            stamp = dt.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.record_path = self.record_root / stamp
            self.record_path.mkdir(parents=True, exist_ok=False)
            (self.record_path / 'session.json').write_text(json.dumps({
                'topic': '/image_raw', 'format': 'JPEG', 'target_fps': 10,
                'started_at': dt.datetime.now().astimezone().isoformat(),
            }, ensure_ascii=False, indent=2), encoding='utf-8')
            self.record_manifest = open(self.record_path / 'timestamps.csv', 'w', newline='', encoding='utf-8')
            self.record_writer = csv.writer(self.record_manifest)
            self.record_writer.writerow(['filename', 'ros_time_ns'])
            self.frame_count = 0
            self.last_saved_at = 0.0
            self.record_error = None
            self.recording = True
            return self.recording_status_unlocked()

    def stop_recording(self):
        with self.record_lock:
            if self.record_manifest:
                self.record_manifest.flush()
                self.record_manifest.close()
            self.record_manifest = None
            self.record_writer = None
            self.recording = False
            if self.record_path:
                self.last_path = self.record_path
            result = self.recording_status_unlocked()
            self.record_path = None
            return result

    def recording_status_unlocked(self):
        return {'active': self.recording, 'path': str(self.record_path) if self.record_path else None,
                'last_path': str(self.last_path) if self.last_path else None,
                'frames': self.frame_count, 'error': self.record_error}

    def on_image(self, message):
        with self.record_lock:
            if not self.recording or time.monotonic() - self.last_saved_at < 0.1:
                return
            self.last_saved_at = time.monotonic()
            try:
                rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
                if message.encoding == 'rgb8':
                    image = cv2.cvtColor(rows[:, :message.width * 3].reshape(message.height, message.width, 3), cv2.COLOR_RGB2BGR)
                elif message.encoding == 'bgr8':
                    image = rows[:, :message.width * 3].reshape(message.height, message.width, 3)
                elif message.encoding == 'mono8':
                    image = rows[:, :message.width]
                else:
                    raise ValueError(f'지원하지 않는 영상 형식: {message.encoding}')
                filename = f'{self.frame_count:06d}.jpg'
                if not cv2.imwrite(str(self.record_path / filename), image, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                    raise OSError('JPEG 저장 실패')
                ros_time = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
                self.record_writer.writerow([filename, ros_time])
                self.record_manifest.flush()
                self.frame_count += 1
            except Exception as error:
                self.record_error = str(error)
                self.get_logger().error(f'Camera recording failed: {error}')

    def close_services(self):
        self.stop_recording()
        self.httpd.shutdown()
        self.httpd.server_close()
        self.motor.close()
        self.devices.close()


class DashboardRequestHandler(SimpleHTTPRequestHandler):
    def _json(self, status, payload):
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(data)

    def end_headers(self):
        if not self.path.startswith('/api/'):
            self.send_header('Cache-Control', 'no-cache')
        super().end_headers()

    def _read_json(self):
        length = int(self.headers.get('Content-Length', '0'))
        if length > 16_384:
            raise ValueError('요청 데이터가 너무 큽니다')
        if not length:
            return {}
        return json.loads(self.rfile.read(length).decode('utf-8'))

    def do_GET(self):
        path = urlparse(self.path).path
        if path == '/api/devices':
            return self._json(200, {'devices': self.server.dashboard_node.devices.status()})
        if path == '/api/recording':
            return self._json(200, self.server.dashboard_node.recording_status())
        if path == '/api/motor':
            return self._json(200, self.server.dashboard_node.motor.status())
        return super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            if path == '/api/motor/arm':
                return self._json(200, self.server.dashboard_node.motor.arm())
            if path == '/api/motor/restart':
                return self._json(200, self.server.dashboard_node.motor.restart())
            if path == '/api/motor/test':
                body = self._read_json()
                result = self.server.dashboard_node.motor.start_test(
                    body.get('speed_mps'), body.get('duration_sec'), body.get('steering', 'center'))
                return self._json(200, result)
            if path == '/api/motor/stop':
                return self._json(200, self.server.dashboard_node.motor.stop())
            if path in ('/api/recording/start', '/api/recording/stop'):
                action = path.rsplit('/', 1)[-1]
                result = self.server.dashboard_node.start_recording() if action == 'start' else self.server.dashboard_node.stop_recording()
                return self._json(200, result)
            parts = path.strip('/').split('/')
            if len(parts) == 4 and parts[:2] == ['api', 'devices']:
                name, action = parts[2], parts[3]
                if name not in DeviceController.DEVICE_INFO or action not in ('connect', 'disconnect', 'reconnect'):
                    return self._json(404, {'error': '알 수 없는 장치 또는 작업입니다'})
                message = getattr(self.server.dashboard_node.devices, action)(name)
                return self._json(200, {'message': message, 'devices': self.server.dashboard_node.devices.status()})
            return self._json(404, {'error': '없는 기능입니다'})
        except Exception as error:
            return self._json(409, {'error': str(error)})

    def log_message(self, form, *args):
        self.server.dashboard_node.get_logger().debug(form % args)


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close_services()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

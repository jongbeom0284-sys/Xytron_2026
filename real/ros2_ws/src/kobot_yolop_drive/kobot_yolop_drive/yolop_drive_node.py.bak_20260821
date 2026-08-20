"""YOLOPv2 차선 세그멘테이션 기반 주행 노드.

안전 계약
  - armed 파라미터가 true 일 때만 모터 토픽에 발행한다. 기본값은 false 다.
  - armed 여도 enabled 토픽이 true 여야 실제로 nonzero 명령이 나간다. 기본 false.
  - 카메라가 끊기면 즉시 정지 명령을 보낸다.
  - 추론 오류가 나면 즉시 정지하고 다시 활성화하려면 enable 을 껐다 켜야 한다.
  - 종료 경로에서 정지 명령을 반복 발행한다.

발행 토픽
  /kobot_yolop/debug_image     오버레이 화면
  /kobot_yolop/status          JSON 상태
  /kobot_yolop/drive_preview   [steer, speed]  — armed 와 무관하게 항상 발행
  <motor_topic>                armed=true 일 때만
"""
from __future__ import annotations

import csv
import json
import math
import signal
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray, String

from kobot_yolop_drive.lane_core import (
    LaneConfig,
    LaneTracker,
    draw_overlay,
)

STOP = (0.0, 0.0)


class YolopDriveNode(Node):
    # 트랙에서 재시작 없이 조정할 수 있는 값과 허용 범위.
    # 범위를 벗어난 값은 거부한다. 안전 상한을 코드에 남겨 둔다.
    LIVE_LIMITS = {
        'steering_sign': (-1.0, 1.0),
        'kp_far': (0.0, 120.0),
        'kp_far_lo': (0.0, 60.0),
        'kp_far_hi': (0.0, 200.0),
        'knee_norm': (0.02, 0.60),
        'kp_near': (0.0, 40.0),
        'kd': (0.0, 40.0),
        'deadband_norm': (0.0, 0.30),
        'filter_alpha': (0.05, 1.0),
        'max_rate': (10.0, 300.0),
        'max_steer': (1.0, 40.0),
        'roi_top_ratio': (0.30, 0.90),
        'y_far_ratio': (0.30, 0.95),
        'y_near_ratio': (0.40, 0.99),
        'hold_seconds': (0.0, 3.0),
        'camera_timeout_sec': (0.1, 2.0),
        'max_row_run': (0.0, 640.0),
        'crosswalk_row_width': (0.0, 200.0),
        'crosswalk_aspect': (0.0, 2.0),
    }

    def __init__(self) -> None:
        super().__init__('yolop_drive')
        self._declare()
        self._read()

        self.bridge = CvBridge()
        self._lock = threading.RLock()
        self._frame_ready = threading.Condition(self._lock)
        self._frame = None
        self._frame_seq = 0
        self._processed_seq = 0
        self._frame_time = None
        self._result = None
        self._last_overlay = 0.0
        self._last_status = 0.0
        self._enabled = False
        self._fault = None      # 래치되는 고장 (추론 오류)
        self._stale = False     # 일시 조건 (카메라 끊김). 자동 회복
        self._stop_worker = False
        self._shutdown_done = False
        self._latency_ms = 0.0
        self._recording = None
        self._record_rows = 0

        self.tracker = LaneTracker(self._lane_config())
        self._load_model()

        self.pub_preview = self.create_publisher(
            Float32MultiArray, '/kobot_yolop/drive_preview', 10)
        self.pub_debug = self.create_publisher(
            Image, '/kobot_yolop/debug_image', 1)
        self.pub_status = self.create_publisher(
            String, '/kobot_yolop/status', 10)
        self.pub_motor = None
        self._apply_armed(self.armed, announce=True)
        self.add_on_set_parameters_callback(self._on_parameters)

        self.create_subscription(Image, self.camera_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/kobot_yolop/enabled',
                                 self._on_enable, 10)
        self.create_subscription(Bool, '/kobot_yolop/record',
                                 self._on_record, 10)
        # 대시보드 기록주행 카드가 쓰는 토픽. QoS 를 발행측과 맞춘다.
        dashboard_qos = QoSProfile(depth=1)
        dashboard_qos.reliability = ReliabilityPolicy.RELIABLE
        dashboard_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool,
                                 '/kobot_lane_drive/record_drive/enabled',
                                 self._on_enable, dashboard_qos)

        self._worker = threading.Thread(target=self._infer_loop, daemon=True,
                                        name='yolop-infer')
        self._worker.start()
        self.create_timer(1.0 / self.publish_rate_hz, self._on_control)
        self.get_logger().info(
            f'yolop_drive 시작. camera={self.camera_topic} '
            f'armed={self.armed} roi={self.tracker.config.roi_top_ratio:.2f}')

    # ------------------------------------------------------------ 파라미터
    def _declare(self) -> None:
        self.declare_parameter('model_path',
                               '/home/xytron/yolotl_offline_eval/models/'
                               'yolopv2.pt')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter('armed', False)
        self.declare_parameter('speed_units', 5.0)
        # 0 보다 크면 이 값이 speed_units 를 대체한다. 대시보드가 m/s 로 준다.
        self.declare_parameter('max_speed_mps', 0.0)
        self.declare_parameter('mps_per_unit', 0.08)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('camera_timeout_sec', 0.35)
        self.declare_parameter('debug_hz', 7.0)
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('record_root',
                               '/home/xytron/yolop_records')
        # 차선/조향
        self.declare_parameter('roi_top_ratio', 0.58)
        self.declare_parameter('y_far_ratio', 0.64)
        self.declare_parameter('y_near_ratio', 0.88)
        self.declare_parameter('kp_far', 26.0)
        self.declare_parameter('kp_far_lo', 12.0)
        self.declare_parameter('kp_far_hi', 90.0)
        self.declare_parameter('knee_norm', 0.20)
        self.declare_parameter('kp_near', 6.0)
        self.declare_parameter('kd', 8.0)
        self.declare_parameter('deadband_norm', 0.035)
        self.declare_parameter('max_steer', 40.0)
        self.declare_parameter('max_rate', 140.0)
        self.declare_parameter('filter_alpha', 0.45)
        self.declare_parameter('steering_sign', -1.0)
        self.declare_parameter('hold_seconds', 0.8)
        self.declare_parameter('yellow_b', 20)
        self.declare_parameter('white_dl', 10)
        self.declare_parameter('min_blob_area', 150)
        self.declare_parameter('min_yellow_px', 25)
        self.declare_parameter('use_tensorrt', True)
        self.declare_parameter('engine_path',
                               '/home/xytron/yolotl_offline_eval'
                               '/models/yolopv2_lane_fp16.engine')
        self.declare_parameter('max_row_run', 80)
        self.declare_parameter('crosswalk_row_width', 32.0)
        self.declare_parameter('crosswalk_aspect', 0.30)

    def _value(self, name):
        return self.get_parameter(name).value

    def _read(self) -> None:
        self.model_path = str(self._value('model_path'))
        self.camera_topic = str(self._value('camera_topic'))
        self.motor_topic = str(self._value('motor_topic'))
        self.armed = bool(self._value('armed'))
        self.mps_per_unit = float(self._value('mps_per_unit'))
        speed_mps = float(self._value('max_speed_mps'))
        if speed_mps > 0.0:
            self.speed_units = speed_mps / max(self.mps_per_unit, 1e-6)
        else:
            self.speed_units = float(self._value('speed_units'))
        self.use_tensorrt = bool(self._value('use_tensorrt'))
        self.engine_path = str(self._value('engine_path'))
        self.publish_rate_hz = float(self._value('publish_rate_hz'))
        self.camera_timeout_sec = float(self._value('camera_timeout_sec'))
        self.debug_hz = float(self._value('debug_hz'))
        self.use_fp16 = bool(self._value('use_fp16'))
        self.record_root = Path(str(self._value('record_root')))
        self.max_steer = float(self._value('max_steer'))
        # 61 단위 = 4.87 m/s. VESC 22500 ERPM 상한과 맞춘다.
        if not 0.0 <= self.speed_units <= 61.0:
            raise ValueError('speed_units 는 0~61 단위로 제한한다')
        if not 0.0 < self.max_steer <= 40.0:
            raise ValueError('max_steer 는 0~40 단위로 제한한다')

    def _lane_config(self) -> LaneConfig:
        return LaneConfig(
            roi_top_ratio=float(self._value('roi_top_ratio')),
            y_far_ratio=float(self._value('y_far_ratio')),
            y_near_ratio=float(self._value('y_near_ratio')),
            kp_far=float(self._value('kp_far')),
            kp_far_lo=float(self._value('kp_far_lo')),
            kp_far_hi=float(self._value('kp_far_hi')),
            knee_norm=float(self._value('knee_norm')),
            kp_near=float(self._value('kp_near')),
            kd=float(self._value('kd')),
            deadband_norm=float(self._value('deadband_norm')),
            max_steer=float(self._value('max_steer')),
            max_rate=float(self._value('max_rate')),
            filter_alpha=float(self._value('filter_alpha')),
            steering_sign=float(self._value('steering_sign')),
            hold_seconds=float(self._value('hold_seconds')),
            yellow_b=int(self._value('yellow_b')),
            white_dl=int(self._value('white_dl')),
            min_blob_area=int(self._value('min_blob_area')),
            min_yellow_px=int(self._value('min_yellow_px')),
            max_row_run=int(self._value('max_row_run')),
            crosswalk_row_width=float(self._value('crosswalk_row_width')),
            crosswalk_aspect=float(self._value('crosswalk_aspect')),
        )

    def _apply_armed(self, armed: bool, announce: bool = False) -> None:
        """armed 상태를 반영한다. false 면 모터 발행자를 아예 없앤다.

        발행자가 없으면 다른 노드의 배타 발행자 검사도 통과한다.
        """
        armed = bool(armed)
        if armed and self.pub_motor is None:
            self.pub_motor = self.create_publisher(
                Float32MultiArray, self.motor_topic, 10)
            self.get_logger().warning(
                f'ARMED: {self.motor_topic} 에 실제 명령을 발행한다. '
                f'속도 {self.speed_units:g} 단위 '
                f'({self.speed_units * self.mps_per_unit:.2f} m/s), '
                f'조향 상한 {self.max_steer:g} 단위')
        elif not armed and self.pub_motor is not None:
            for _ in range(5):
                self._publish_motor(*STOP)
            self.destroy_publisher(self.pub_motor)
            self.pub_motor = None
            self.get_logger().warning('DISARMED: 모터 발행을 중단했다')
        elif announce:
            self.get_logger().info(
                'ARMED 로 시작' if armed
                else '안전 모드로 시작: 모터 토픽에 발행하지 않는다')
        self.armed = armed
        if not armed:
            with self._lock:
                self._enabled = False

    def _on_parameters(self, params):
        """armed / max_speed_mps / speed_units 를 실행 중에 바꿀 수 있게 한다."""
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'armed':
                self._apply_armed(bool(param.value))
            elif param.name == 'max_speed_mps':
                value = float(param.value)
                if value < 0.0 or value > 4.87:
                    return SetParametersResult(
                        successful=False,
                        reason='max_speed_mps 는 0~4.87 m/s 로 제한한다')
                if value > 0.0:
                    self.speed_units = value / max(self.mps_per_unit, 1e-6)
                    self.get_logger().warning(
                        f'속도 변경: {value:.2f} m/s '
                        f'= {self.speed_units:.2f} 단위')
            elif param.name == 'speed_units':
                value = float(param.value)
                if not 0.0 <= value <= 61.0:
                    return SetParametersResult(
                        successful=False,
                        reason='speed_units 는 0~61 단위로 제한한다')
                self.speed_units = value
            elif param.name in self.LIVE_LIMITS:
                low, high = self.LIVE_LIMITS[param.name]
                value = float(param.value)
                if not low <= value <= high:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} 는 {low}~{high} 범위여야 한다')
                if param.name == 'camera_timeout_sec':
                    self.camera_timeout_sec = value
                else:
                    setattr(self.tracker.config, param.name, value)
                if param.name == 'max_steer':
                    self.max_steer = value
                self.get_logger().warning(
                    f'{param.name} = {value:g} 로 변경')
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------ 모델
    def _load_model(self) -> None:
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError('CUDA 를 쓸 수 없다. GPU 추론이 필요하다')
        self._torch = torch
        self._engine = None

        # TensorRT 엔진이 있으면 그쪽을 쓴다. 실패하면 반드시 TorchScript 로
        # 돌아간다. 어느 쪽이든 GPU 추론이고, 어느 쪽인지 로그에 남긴다.
        if self.use_tensorrt:
            try:
                from .trt_backend import TensorRTLane
                self._engine = TensorRTLane(self.engine_path, torch)
                self.get_logger().info(
                    f'YOLOPv2 TensorRT 적재 완료 — {self._engine.description} '
                    f'{torch.cuda.get_device_name(0)}')
            except Exception as error:  # noqa: BLE001
                self._engine = None
                self.get_logger().warning(
                    f'TensorRT 적재 실패, TorchScript 로 진행한다: {error}')

        model = torch.jit.load(self.model_path, map_location='cuda').eval()
        self._model = model.half() if self.use_fp16 else model
        sample = torch.zeros(1, 3, 384, 640, device='cuda')
        if self.use_fp16:
            sample = sample.half()
        with torch.no_grad():
            for _ in range(3):
                self._model(sample)
        torch.cuda.synchronize()
        if self._engine is None:
            self.get_logger().info(
                f'YOLOPv2 적재 완료 ({"fp16" if self.use_fp16 else "fp32"}) '
                f'{torch.cuda.get_device_name(0)}')

    def _infer(self, bgr):
        torch = self._torch
        # uint8 그대로 올리고 GPU 에서 나눈다. 전송량이 1/4 이고
        # 나눗셈은 여전히 fp32 라 IEEE754 상 결과가 비트 단위로 같다.
        tensor = torch.from_numpy(
            cv2.resize(bgr, (640, 384))[:, :, ::-1].copy()).cuda()
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        if self._engine is not None:
            # 엔진 버퍼가 요구하는 자료형으로만 맞춰 준다.
            drive_out, lane_out = self._engine.infer(
                tensor.to(self._engine.input_dtype))
        else:
            if self.use_fp16:
                tensor = tensor.half()
            with torch.no_grad():
                _, drive_out, lane_out = self._model(tensor)
        height, width = bgr.shape[:2]
        lane = (lane_out[0, 0].float() > 0.5).cpu().numpy()
        road = (drive_out[0].float().argmax(0) == 1).cpu().numpy()
        lane = cv2.resize(lane.astype(np.uint8), (width, height)) > 0
        road = cv2.resize(road.astype(np.uint8), (width, height)) > 0
        return lane, road

    # ------------------------------------------------------------ 콜백
    def _on_image(self, message: Image) -> None:
        try:
            bgr = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self._raise_fault(f'카메라 변환 실패: {exc}')
            return
        with self._frame_ready:
            self._frame_seq += 1
            self._frame = np.ascontiguousarray(bgr)
            self._frame_time = time.monotonic()
            self._frame_ready.notify()

    def _on_enable(self, message: Bool) -> None:
        want = bool(message.data)
        with self._lock:
            if want and self._fault is not None:
                self.get_logger().warning(
                    f'활성화 거부: 고장 상태 ({self._fault}). '
                    '먼저 비활성화했다가 다시 시도하라')
                return
            if not want:
                self._fault = None
            self._enabled = want
        self.get_logger().warning(
            f'주행 {"활성화" if want else "비활성화"}')
        if not want:
            self._publish_motor(*STOP)

    def _on_record(self, message: Bool) -> None:
        if bool(message.data):
            self._start_record()
        else:
            self._stop_record()

    # ------------------------------------------------------------ 기록
    def _start_record(self) -> None:
        with self._lock:
            if self._recording is not None:
                return
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            folder = self.record_root / stamp
            (folder / 'frame').mkdir(parents=True, exist_ok=True)
            handle = (folder / 'drive.csv').open('w', newline='',
                                                 encoding='utf-8')
            writer = csv.writer(handle)
            writer.writerow(['wall_time', 'basis', 'steer', 'speed',
                             'error_far', 'error_near', 'fit_rows',
                             'yellow_blobs', 'white_blobs', 'latency_ms',
                             'enabled', 'armed', 'frame'])
            self._recording = {'folder': folder, 'handle': handle,
                               'writer': writer, 'index': 0}
            self._record_rows = 0
        self.get_logger().warning(f'기록 시작: {folder}')

    def _stop_record(self) -> None:
        with self._lock:
            state = self._recording
            self._recording = None
        if state is None:
            return
        state['handle'].close()
        self.get_logger().warning(
            f'기록 종료: {state["folder"]} ({self._record_rows} 행)')

    def _write_record(self, result, steer, speed, overlay):
        with self._lock:
            state = self._recording
            if state is None:
                return
            index = state['index']
            state['index'] += 1
            writer, folder = state['writer'], state['folder']
            handle = state['handle']
        name = ''
        if index % 3 == 0 and overlay is not None:
            name = f'{index:06d}.jpg'
            cv2.imwrite(str(folder / 'frame' / name), overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
        writer.writerow([f'{time.time():.3f}', result.basis,
                         f'{steer:.2f}', f'{speed:.2f}',
                         f'{result.error_far:.4f}', f'{result.error_near:.4f}',
                         result.fit_rows, result.yellow_blobs,
                         result.white_blobs, f'{self._latency_ms:.1f}',
                         int(self._enabled), int(self.armed), name])
        handle.flush()
        self._record_rows += 1

    # ------------------------------------------------------------ 추론 루프
    def _infer_loop(self) -> None:
        while True:
            with self._frame_ready:
                while (not self._stop_worker
                       and (self._frame is None
                            or self._frame_seq <= self._processed_seq)):
                    self._frame_ready.wait(timeout=0.5)
                if self._stop_worker:
                    return
                if self._frame is None:
                    continue
                bgr = self._frame
                seq = self._frame_seq
                stamp = self._frame_time
                self._processed_seq = seq
            try:
                started = time.perf_counter()
                lane, road = self._infer(bgr)
                result = self.tracker.process(bgr, lane, road,
                                              time.monotonic())
                self._latency_ms = (time.perf_counter() - started) * 1000.0
            except Exception as exc:  # noqa: BLE001
                self._raise_fault(f'추론 실패: {exc}')
                continue
            with self._lock:
                self._result = (result, stamp, bgr)
            self._maybe_publish_debug(bgr, result)

    def _maybe_publish_debug(self, bgr, result) -> None:
        now = time.monotonic()
        if now - self._last_overlay < 1.0 / max(self.debug_hz, 0.1):
            return
        self._last_overlay = now
        speed = self.speed_units if self._enabled else 0.0
        overlay = draw_overlay(bgr, result, self.tracker.config,
                               speed_units=speed, armed=self.armed)
        cv2.putText(overlay, f'{self._latency_ms:.0f} ms', (540, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1,
                    cv2.LINE_AA)
        try:
            self.pub_debug.publish(
                self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8'))
        except Exception:  # noqa: BLE001
            pass
        self._last_overlay_image = overlay

    # ------------------------------------------------------------ 제어
    def _on_control(self) -> None:
        now = time.monotonic()
        with self._lock:
            payload = self._result
            enabled = self._enabled
            fault = self._fault

        steer = speed = 0.0
        basis = 'none'
        if payload is not None:
            result, stamp, bgr = payload
            basis = result.basis
            stale = stamp is None or (now - stamp) > self.camera_timeout_sec
            with self._lock:
                was_stale, self._stale = self._stale, stale
            if stale:
                if not was_stale:
                    self.get_logger().warning(
                        '카메라 프레임이 오래됨 — 정지. 프레임이 돌아오면 자동 회복한다')
            else:
                if was_stale:
                    self.get_logger().info('카메라 프레임 회복')
                if fault is None and enabled:
                    steer = float(np.clip(result.steer, -self.max_steer,
                                          self.max_steer))
                    speed = self.speed_units
        else:
            with self._lock:
                self._stale = True
            if self._recording is not None:
                self._write_record(result, steer, speed,
                                   getattr(self, '_last_overlay_image', None))

        message = Float32MultiArray()
        message.data = [float(steer), float(speed)]
        self.pub_preview.publish(message)
        self._publish_motor(steer, speed)
        self._publish_status(basis, steer, speed, enabled)

    def _publish_motor(self, steer: float, speed: float) -> None:
        if self.pub_motor is None:
            return
        if not all(math.isfinite(v) for v in (steer, speed)):
            steer = speed = 0.0
        message = Float32MultiArray()
        message.data = [float(steer), float(speed)]
        self.pub_motor.publish(message)

    def _publish_status(self, basis, steer, speed, enabled) -> None:
        now = time.monotonic()
        if now - self._last_status < 0.2:
            return
        self._last_status = now
        with self._lock:
            payload = self._result
            fault = self._fault
            recording = self._recording is not None
            stale = self._stale
            rows = self._record_rows
        result = payload[0] if payload else None
        if fault:
            mode = 'ERROR'
        elif basis in ('yellow', 'white_center'):
            mode = 'TRACKING'
        elif basis == 'hold':
            mode = 'HOLD'
        else:
            mode = 'LOST'
        self.pub_status.publish(String(data=json.dumps({
            'mode': mode,
            'basis': basis,
            'steer': round(steer, 2),
            'speed': round(speed, 2),
            'enabled': bool(enabled),
            'armed': bool(self.armed),
            'fault': fault,
            'camera_stale': bool(stale),
            'latency_ms': round(self._latency_ms, 1),
            'error_far': round(result.error_far, 4) if result else 0.0,
            'error_near': round(result.error_near, 4) if result else 0.0,
            'fit_rows': result.fit_rows if result else 0,
            'yellow_blobs': result.yellow_blobs if result else 0,
            'white_blobs': result.white_blobs if result else 0,
            'crosswalk_blobs': result.crosswalk_blobs if result else 0,
            'recording': recording,
            'record_rows': rows,
            'roi_top': self.tracker.config.roi_top_ratio,
            'confidence': (1.0 if basis in ('yellow', 'white_center')
                           else 0.0),
            'center_offset_px': round(
                (result.error_far * 320.0) if result else 0.0, 1),
            'speed_mps': round(speed * self.mps_per_unit, 3),
        }, ensure_ascii=False)))

    def _raise_fault(self, reason: str) -> None:
        with self._lock:
            changed = reason != self._fault
            self._fault = reason
            self._enabled = False
        self._publish_motor(*STOP)
        if changed:
            self.get_logger().error(f'주행 중지: {reason}')

    # ------------------------------------------------------------ 종료
    def shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        with self._frame_ready:
            self._stop_worker = True
            self._enabled = False
            self._frame_ready.notify_all()
        for _ in range(5):
            self._publish_motor(*STOP)
        self._stop_record()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_int = signal.getsignal(signal.SIGINT)
    previous_term = signal.getsignal(signal.SIGTERM)

    def request_shutdown(_signum, _frame):
        if node is not None:
            node.shutdown()
        context = rclpy.get_default_context()
        if context.ok():
            context.shutdown()

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    try:
        node = YolopDriveNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

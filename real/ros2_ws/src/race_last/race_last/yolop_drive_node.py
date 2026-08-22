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
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32MultiArray, String

import json

from .cone_link import ConeLink

from .lane_core import (
    divider_x,
    LaneConfig,
    LaneTracker,
    draw_overlay,
)

STOP = (0.0, 0.0)


# 신호등 노드는 약 15Hz 로 돈다. 아래 장수는 그 기준이다.
STOP_RELEASE_SEC = 1.0   # red 를 이 시간 넘게 못 보면 정지를 푼다
RED_CONFIRM = 3          # 빨간불 확정에 필요한 연속 장수 (약 0.2초)
GO_CONFIRM = 7           # 초록·좌회전 확정에 필요한 연속 장수 (약 0.47초)
GO_LOCK_GREEN_SEC = 30.0  # 직진 확정 뒤 신호 인식을 쉬는 시간 (빨간불 포함)
GO_LOCK_LEFT_SEC = 20.0   # 좌회전 확정 뒤 잠금 (0822 실험: 10에서 상향)
STRAIGHT_HOLD_SEC = 3.0  # 직진 확정 직후 조향을 줄여 직진만 하는 시간
STRAIGHT_HOLD_GAIN = 0.15  # 그동안 조향에 곱하는 값. 0 이면 완전 직진

# ---- 앞차 회피. 거리·유지 값은 Lane_follow.py 원본 그대로 ----
AVOID_ENTER_M = 1.30     # 이 거리 안이면 회피 시작
AVOID_CLEAR_M = 1.80     # 이만큼 멀어지면 해제
AVOID_HOLD_SEC = 0.8     # 차가 안 보여도 이 시간은 회피를 유지한다
CAR_CONFIRM = 3          # 같은 방향이 이만큼 연속이어야 인정 (오검출 방지)
AVOID_OFFSET_PX = 90.0   # 흰선에서 안쪽으로 띄울 거리. 실주행하며 맞춘다


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
        'max_rate': (10.0, 500.0),   # 0822: 조향 도달 속도 상향 여지 확보
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

        # 신호등 연동 상태. 신호등을 지나면 상태가 none 이 되므로
        # 마지막으로 본 초록/좌회전을 물고 있는다.
        self._straight = False        # 초록불에서 켜고 좌회전에서 끈다
        self._traffic_stop = False    # 빨간불에서 켜고 초록/좌회전에서 끈다
        self._traffic_state = 'none'
        self._traffic_at = 0.0
        self._last_red_at = 0.0       # 마지막으로 red 를 본 시각
        self._recent = []             # 최근 신호. 연속 장수를 센다
        self._go_locked_until = 0.0   # 이 시각까지 신호등을 아예 안 읽는다
        self._straight_hold_until = 0.0  # 이 시각까지 조향을 줄여 직진만 한다

        # 앞차 회피 상태
        self._car_recent = []        # 최근 car.dir. 연속 장수를 센다
        self._avoid_side = None      # 'right' | 'left' | None
        self._avoid_until = 0.0      # 이 시각까지 회피를 유지한다
        self._car_dist = None

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

        # 압축 스트림 구독 (원본은 900KB 라 대역폭 병목, 2026-08-22)
        if self.camera_topic.endswith('/compressed'):
            self.create_subscription(
                CompressedImage, self.camera_topic,
                self._on_image_compressed, qos_profile_sensor_data)
        else:
            self.create_subscription(Image, self.camera_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/kobot_yolop/enabled',
                                 self._on_enable, 10)
        self.create_subscription(Bool, '/kobot_yolop/record',
                                 self._on_record, 10)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_traffic, 10)
        self.create_subscription(String, '/vehicle/state',
                                 self._on_vehicle, 10)
        self.create_subscription(String, '/cone/reading',
                                 self._on_cone_reading, 10)
        # 대시보드 기록주행 카드가 쓰는 토픽. QoS 를 발행측과 맞춘다.
        dashboard_qos = QoSProfile(depth=1)
        dashboard_qos.reliability = ReliabilityPolicy.RELIABLE
        dashboard_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(Bool,
                                 '/kobot_lane_drive/record_drive/enabled',
                                 self._on_enable, dashboard_qos)

        self._cone_reading = None
        self._cone_link = (ConeLink(self.get_logger(),
                                    float(self._value('cone_approach_depth_m')))
                           if self.cone_enabled else None)
        self._worker = threading.Thread(target=self._infer_loop, daemon=True,
                                        name='yolop-infer')
        self._worker.start()
        self.create_timer(1.0 / self.publish_rate_hz, self._on_control)
        self.get_logger().info(
            f'yolop_drive 시작. camera={self.camera_topic} '
            f'armed={self.armed} roi={self.tracker.config.roi_top_ratio:.2f}')

    # ------------------------------------------------------------ 파라미터
    def _declare(self) -> None:
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
        self.declare_parameter('engine_path',
                               '/home/xytron/2026_Xytron'
                               '/models/yolopv2_lane_fp16.engine')
        self.declare_parameter('max_row_run', 80)
        self.declare_parameter('crosswalk_row_width', 32.0)
        self.declare_parameter('crosswalk_aspect', 0.30)
        # 라바콘 (kmu_race 이식 — cone_link.py)
        self.declare_parameter('cone_enabled', True)
        self.declare_parameter('cone_speed_scale', 0.7)
        # 통로가 보이면 진입 전에 이 속도로 미리 감속 (0822 벽 충돌 재발 방지:
        # 2.16 m/s 로 통로 돌진 -> 진입 후 감속으로는 늦다)
        self.declare_parameter('cone_approach_speed_units', 9.0)
        self.declare_parameter('cone_approach_depth_m', 1.2)

    def _value(self, name):
        return self.get_parameter(name).value

    def _read(self) -> None:
        self.camera_topic = str(self._value('camera_topic'))
        self.motor_topic = str(self._value('motor_topic'))
        self.armed = bool(self._value('armed'))
        self.mps_per_unit = float(self._value('mps_per_unit'))
        speed_mps = float(self._value('max_speed_mps'))
        if speed_mps > 0.0:
            self.speed_units = speed_mps / max(self.mps_per_unit, 1e-6)
        else:
            self.speed_units = float(self._value('speed_units'))
        self.engine_path = str(self._value('engine_path'))
        self.publish_rate_hz = float(self._value('publish_rate_hz'))
        self.camera_timeout_sec = float(self._value('camera_timeout_sec'))
        self.debug_hz = float(self._value('debug_hz'))
        self.use_fp16 = bool(self._value('use_fp16'))
        self.record_root = Path(str(self._value('record_root')))
        self.max_steer = float(self._value('max_steer'))
        self.cone_enabled = bool(self._value('cone_enabled'))
        self.cone_speed_scale = float(self._value('cone_speed_scale'))
        self.cone_approach_units = float(
            self._value('cone_approach_speed_units'))
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

        # TensorRT 엔진만 쓴다 (2026-08-22 폴백 제거). 적재 실패는 곧바로
        # 기동 실패다 — 느린 TorchScript 로 몰래 달리는 상황을 없앤다.
        from .trt_backend import TensorRTLane
        self._engine = TensorRTLane(self.engine_path, torch)
        sample = torch.zeros(1, 3, 384, 640, device='cuda')
        for _ in range(3):
            self._engine.infer(sample.to(self._engine.input_dtype))
        torch.cuda.synchronize()
        self.get_logger().info(
            f'YOLOPv2 TensorRT 적재 완료 — {self._engine.description} '
            f'{torch.cuda.get_device_name(0)}')

    def _infer(self, bgr):
        torch = self._torch
        # uint8 그대로 올리고 GPU 에서 나눈다. 전송량이 1/4 이고
        # 나눗셈은 여전히 fp32 라 IEEE754 상 결과가 비트 단위로 같다.
        tensor = torch.from_numpy(
            cv2.resize(bgr, (640, 384))[:, :, ::-1].copy()).cuda()
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        # 엔진 버퍼가 요구하는 자료형으로만 맞춰 준다.
        drive_out, lane_out = self._engine.infer(
            tensor.to(self._engine.input_dtype))
        height, width = bgr.shape[:2]
        lane = (lane_out[0, 0].float() > 0.5).cpu().numpy()
        road = (drive_out[0].float().argmax(0) == 1).cpu().numpy()
        lane = cv2.resize(lane.astype(np.uint8), (width, height)) > 0
        road = cv2.resize(road.astype(np.uint8), (width, height)) > 0
        return lane, road

    # ------------------------------------------------------------ 콜백
    def _on_image_compressed(self, message) -> None:
        try:
            buf = np.frombuffer(bytes(message.data), dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'압축 영상 디코드 실패: {exc}')
            return
        if bgr is None or bgr.ndim != 3:
            return
        with self._frame_ready:
            self._frame_seq += 1
            self._frame = np.ascontiguousarray(bgr)
            self._frame_time = time.monotonic()
            self._frame_ready.notify()

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

    def _on_traffic(self, message: String) -> None:
        """신호등 상태를 받아 정지와 직진 트리거를 물린다.

        빨간불   RED_CONFIRM 장 연속이면 정지. 잠금과 무관하게 항상 읽는다
        초록불   GO_CONFIRM 장 연속이면 직진 확정. 이후 30초(GO_LOCK_GREEN_SEC) 안 읽는다
        좌회전   GO_CONFIRM 장 연속이면 직진 해제. 오염을 그대로 두어 좌측을 따른다
        노란불   아무것도 바꾸지 않는다. 하던 대로 간다
        없음     아무것도 바꾸지 않는다. 신호등을 지난 뒤가 여기다
        """
        try:
            state = str(json.loads(message.data).get('state', 'none'))
        except Exception:  # noqa: BLE001
            return
        now = time.monotonic()
        self._traffic_state = state
        self._traffic_at = now

        # 잠금 중에는 신호등 인식 자체를 쉰다. 빨간불도 안 읽는다.
        # 확정한 판단을 흔들 것이 없으니 그 사이 오검출에 면역이다.
        if now < self._go_locked_until:
            return

        self._recent.append(state)
        if len(self._recent) > GO_CONFIRM:
            self._recent.pop(0)

        def run_of(want, need):
            tail = self._recent[-need:]
            return len(tail) == need and all(v == want for v in tail)

        # 빨간불. 잠금과 무관하게 언제나 본다. 언제 초록으로 바뀔지 모른다.
        if state == 'red':
            if run_of('red', RED_CONFIRM):
                if not self._traffic_stop:
                    self.get_logger().warning(
                        '빨간불 %d장 연속 — 정지' % RED_CONFIRM)
                self._traffic_stop = True
                self._last_red_at = now
            return

        if state not in ('green', 'left'):
            return
        if not run_of(state, GO_CONFIRM):
            return

        self._traffic_stop = False
        self._go_locked_until = now + (
            GO_LOCK_GREEN_SEC if state == 'green' else GO_LOCK_LEFT_SEC)
        self._recent.clear()
        if state == 'green':
            self._straight = True
            self._straight_hold_until = now + STRAIGHT_HOLD_SEC
            self.get_logger().info(
                '초록불 %d장 연속 — 직진 확정. %.0f초 조향 억제, %.0f초 신호 무시'
                % (GO_CONFIRM, STRAIGHT_HOLD_SEC, GO_LOCK_GREEN_SEC))
        else:
            self._straight = False
            self._straight_hold_until = 0.0
            self.get_logger().info(
                '좌회전 %d장 연속 — 직진 해제, %.0f초간 신호 무시'
                % (GO_CONFIRM, GO_LOCK_LEFT_SEC))

    def _on_vehicle(self, message: String) -> None:
        """앞차 위치를 받아 회피 방향을 정한다.

        왼쪽 차 → 우측 흰선에 붙는다. 오른쪽 차 → 좌측 흰선에 붙는다.
        한 프레임 오검출로 차선을 바꾸면 안 되므로 CAR_CONFIRM 장 연속을 본다.
        차가 사라져도 AVOID_HOLD_SEC 동안은 유지한다. 지나치는 중이니까.
        """
        # 콘이 2개 이상 보이는 동안은 앞차 회피를 무시한다 — 콘 군집이
        # 차량(suitcase)으로 오인돼 통로 입구에서 우측 회피 조향이 선
        # 사고가 있었다 (2026-08-22 21:39 실주행 로그).
        with self._lock:
            _cone_d = self._cone_reading
        if _cone_d and int(_cone_d.get('cones', 0)) >= 2:
            self._avoid_side = None
            return
        try:
            payload = json.loads(message.data)
            car = payload.get('car', {})
            dist = car.get('dist')
            boxes = payload.get('boxes') or []
        except Exception:  # noqa: BLE001
            return

        # 좌/우는 인식 노드가 준 값(화면 중앙 기준) 대신 여기서 다시 정한다.
        # 노란 중앙선을 기준으로 보면 카메라 장착 위치와 차의 치우침이 상쇄된다.
        side = self._side_by_lane(boxes, payload)
        if side == 'NONE':
            side = str(car.get('dir', 'NONE'))   # 차선이 안 보이면 원래 값
        now = time.monotonic()
        self._car_dist = dist

        # 2026-08-23: 라이다 융합이 실패하면 dist=None 이 되는데, 예전에는
        # 그때 near=False·far=True 라 회피가 아예 안 걸리고 즉시 해제됐다.
        # 실측: YOLO 는 dir=LEFT conf 0.73 으로 차를 정확히 잡는데 거리만 없었다.
        # 거리를 몰라도 좌/우가 나오면 반대 차선으로 붙는다.
        if dist is None:
            near = bool(boxes)          # 차가 보이면 회피 대상으로 본다
            far = not boxes             # 안 보이면 해제
        else:
            near = float(dist) <= AVOID_ENTER_M
            far = float(dist) >= AVOID_CLEAR_M

        self._car_recent.append(side if near else 'NONE')
        if len(self._car_recent) > CAR_CONFIRM:
            self._car_recent.pop(0)
        steady = (len(self._car_recent) == CAR_CONFIRM
                  and len(set(self._car_recent)) == 1)

        if steady and near and side in ('LEFT', 'RIGHT'):
            want = 'right' if side == 'LEFT' else 'left'
            if self._avoid_side != want:
                self.get_logger().warning(
                    '앞차 %s %s — %s 흰선으로 회피'
                    % (side,
                       ('%.2fm' % float(dist)) if dist is not None else '거리없음',
                       '우측' if want == 'right' else '좌측'))
            self._avoid_side = want
            self._avoid_until = now + AVOID_HOLD_SEC
        elif far and self._avoid_side and now > self._avoid_until:
            self.get_logger().info('앞차 벗어남 — 회피 해제')
            self._avoid_side = None

    def _side_by_lane(self, boxes, payload) -> str:
        """가장 가까운 앞차가 노란 중앙선의 어느 쪽인지 본다.

        비교할 행은 박스 아래쪽 y2 다. 차가 바닥에 닿는 지점이라 차선과
        같은 평면이고, 원근 왜곡의 영향이 가장 작다.
        """
        if not boxes:
            return 'NONE'
        with self._lock:
            payload_result = self._result
        result = payload_result[0] if payload_result else None
        if result is None:
            return 'NONE'

        width = float(payload.get('w') or 640)
        height = float(payload.get('h') or 480)
        # 가장 가까운 것 = 박스가 가장 큰 것
        box = max(boxes, key=lambda b: float(b.get('w', 0)) * float(b.get('h', 0)))
        cx = (float(box.get('x', 0)) + float(box.get('w', 0)) / 2.0) * width
        y2 = (float(box.get('y', 0)) + float(box.get('h', 0))) * height

        edge = divider_x(result, int(round(y2)))
        if edge is None:
            return 'NONE'
        return 'LEFT' if cx < edge else 'RIGHT'

    def _avoid_offset(self, now: float) -> float:
        """이 순간 흰선에서 띄울 거리. 0 이면 평소 주행."""
        # 직진 조향 억제 중에는 회피하지 않는다. 구간이 정해져 있어 억제가 이긴다.
        if now < self._straight_hold_until:
            return 0.0
        if not self._avoid_side or now > self._avoid_until:
            return 0.0
        return AVOID_OFFSET_PX if self._avoid_side == 'right' else -AVOID_OFFSET_PX

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
                if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3:
                    self.get_logger().warning(
                        '프레임 형식 이상 — 건너뜀 %s'
                        % (None if bgr is None else str(bgr.shape)))
                    continue
                lane, road = self._infer(bgr)
                # 직진 트리거. 꺼져 있으면 기존 동작 그대로다.
                self.tracker.config.straight_only = self._straight
                # 앞차 회피. 0 이면 기존 동작 그대로다.
                self.tracker.config.avoid_offset_px = self._avoid_offset(
                    time.monotonic())
                result = self.tracker.process(bgr, lane, road,
                                              time.monotonic())
                self._latency_ms = (time.perf_counter() - started) * 1000.0
            except Exception as exc:  # noqa: BLE001
                self._raise_fault(f'추론 실패: {exc}')
                continue
            with self._lock:
                self._result = (result, stamp, bgr)
            # 오버레이 실패로 워커 스레드가 조용히 죽는 것을 막는다
            # (2026-08-22: 이 줄이 try 밖이라 첫 예외에 스레드가 사망,
            #  차선 영상이 안 나오고 camera_stale 이 영구 true 였다)
            try:
                self._maybe_publish_debug(bgr, result)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'오버레이 생략: {exc}')

    def _vehicle_needed(self, now: float) -> bool:
        """차량 인식을 돌려야 하는가. Vehicle_Perception 이 이 값을 따른다."""
        # 좌회전/직진 신호 잠금 중에는 차량이 없다
        if now < self._go_locked_until:
            # 다만 회피를 유지 중이면 계속 봐야 한다 (지나치는 중)
            if self._avoid_side and now <= self._avoid_until:
                return True
            return False
        # 콘 구간(진입 또는 접근)에서는 콘을 차량으로 오인한다
        link = self._cone_link
        if link is not None and (link.active or link.approach):
            return False
        return True

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
    def _on_cone_reading(self, message: String) -> None:
        try:
            data = json.loads(message.data)
        except ValueError:
            return
        with self._lock:
            self._cone_reading = data

    def _on_control(self) -> None:
        now = time.monotonic()
        # red 를 STOP_RELEASE_SEC 넘게 못 보면 정지를 스스로 푼다.
        # 신호등이 시야를 벗어나거나 오검출이 한 번 났을 때 갇히지 않게 한다.
        if self._traffic_stop and (now - self._last_red_at) > STOP_RELEASE_SEC:
            self._traffic_stop = False
            self.get_logger().info(
                '빨간불이 %.1f초 안 보임 — 정지 해제' % STOP_RELEASE_SEC)
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
                    # 직진 확정 직후에는 조향을 줄여 사실상 직진만 한다.
                    # 삼거리에서 좌회전 차선으로 끌려가는 것을 막는다.
                    if now < self._straight_hold_until:
                        steer *= STRAIGHT_HOLD_GAIN
                    # 빨간불이면 속도만 0 으로 떨군다. 조향은 그대로 둔다.
                    speed = 0.0 if self._traffic_stop else self.speed_units
        else:
            with self._lock:
                self._stale = True
            if self._recording is not None:
                self._write_record(result, steer, speed,
                                   getattr(self, '_last_overlay_image', None))

        # ── 라바콘 구간: kmu_race 이식 조향(cone_link)으로 대체 ──
        if self._cone_link is not None and fault is None and enabled:
            with self._lock:
                cone_d = self._cone_reading
                cam_ok = payload is not None and not self._stale
            cone_cmd = self._cone_link.tick(
                now, cone_d,
                ego_mps=speed * self.mps_per_unit,
                camera_ok=cam_ok)
            if cone_cmd is not None:
                steer = float(np.clip(cone_cmd[0], -self.max_steer,
                                      self.max_steer))
                cone_speed = min(cone_cmd[1] * self.cone_speed_scale,
                                 self.speed_units)
                speed = 0.0 if self._traffic_stop else cone_speed
                basis = 'cone'
            elif (self._cone_link.approach
                    and speed > self.cone_approach_units):
                # 통로가 보인다 — 진입 전에 미리 줄인다 (조향은 차선 그대로)
                speed = self.cone_approach_units
                basis = 'cone_approach'

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
            'signal_lock_s': round(max(
                0.0, self._go_locked_until - now), 1),
            # 차량 인식 필요 여부 (2026-08-22):
            #  · 좌회전 잠금 중  -> 불필요 (그 구간에 차량 없음)
            #  · 콘 구간 진입/접근 -> 불필요 (콘을 차량으로 오인)
            #  · 회피 유지 중    -> 필요 (지나치는 중이라 계속 봐야 함)
            # 회피 유지가 풀리는 순간 이 값이 함께 내려간다.
            'vehicle_needed': bool(self._vehicle_needed(now)),
            # 사후 분석용 판단 근거 (2026-08-22: 재생값과 실주행값이
            # 달라 원인 추적이 불가능했던 문제. 이제 전부 남긴다)
            'straight_only': bool(self._straight),
            'straight_hold': round(max(
                0.0, self._straight_hold_until - now), 2),
            'avoid_side': self._avoid_side or '',
            'avoid_offset_px': round(self._avoid_offset(now), 1),
            'traffic_stop': bool(self._traffic_stop),
            'traffic_state': self._traffic_state,
            'cone_active': bool(
                self._cone_link.active if self._cone_link else False),
            'cone_approach': bool(
                self._cone_link.approach if self._cone_link else False),
            'raw_steer': round(
                getattr(result, 'raw_steer', 0.0), 2) if result else 0.0,
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

"""YOLOPv2 차선 세그멘테이션 및 YOLOv8n 장애물 회피 기반 주행 노드.

안전 계약
  - armed 파라미터가 true 일 때만 모터 토픽에 발행한다. 기본값은 false 다.
  - armed 여도 enabled 토픽이 true 여야 실제로 nonzero 명령이 나간다. 기본 false.
  - 카메라가 끊기면 즉시 정지 명령을 보낸다.
  - 보행자(Person, ID 0) 감지 시 비상 정지(거리 이탈 시 자동 재출발).
  - 장애물(Car, ID 2 등) 감지 시 중앙선 기준 좌/우 판정 및 흰선 회피 주행.
  - 추론 오류가 나면 즉시 정지하고 다시 활성화하려면 enable 을 껐다 켜야 한다.
  - 종료 경로에서 정지 명령을 반복 발행한다.

발행 토픽
  /kobot_yolop/debug_image     오버레이 화면
  /kobot_yolop/status          JSON 상태
  /kobot_yolop/drive_preview   [steer, speed]  — armed 와 무관하게 항상 발행
  /kobot_yolop/obstacle_dist   장애물 최단거리 (m)
  /kobot_yolop/obstacle_dir    장애물 방향 (LEFT / RIGHT / NONE)
  <motor_topic>                armed=true 일 때만
"""
from __future__ import annotations

import csv
import json
import math
import os
import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

# 가상환경 패키지 경로 우선 로드 (YOLOv8 등 의존성)
VENV_SITE_PACKAGES = [
    '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages',
    '/home/xytron/2026_Xytron/real/.venv/lib/python3.10/site-packages',
    '/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages',
]
for p in VENV_SITE_PACKAGES:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String

from kobot_vehicle.lane_core import (
    LaneConfig,
    LaneTracker,
    draw_overlay,
    far_gain,
)

STOP = (0.0, 0.0)


# 신호등 노드는 약 15Hz 로 돈다. 아래 장수는 그 기준이다.
STOP_RELEASE_SEC = 1.0   # red 를 이 시간 넘게 못 보면 정지를 푼다
RED_CONFIRM = 3          # 빨간불 확정에 필요한 연속 장수 (약 0.2초)
GO_CONFIRM = 7           # 초록·좌회전 확정에 필요한 연속 장수 (약 0.47초)
GO_LOCK_SEC = 10.0       # 확정 뒤 초록·좌회전을 더 안 읽는 시간


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
        'obstacle_avoid_dist': (0.3, 4.0),
        'person_stop_dist': (0.5, 5.0),
        'avoid_hold_frames': (1, 200),
        'avoid_speed_ratio': (0.1, 1.0),
        'white_lane_offset_px': (10.0, 300.0),
        'yolo_conf_thres': (0.05, 0.95),
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

        # LiDAR 데이터 버퍼
        self._latest_scan = None
        self._latest_scan_at = 0.0

        # 장애물 회피 상태 머신
        self._avoid_state = 'STATE_NORMAL'
        self._avoid_hold_counter = 0
        self._last_obstacle_dir = None
        self._last_obstacle_dist = float('inf')
        self._detected_boxes = []
        self._person_emergency = False
        self._person_dist = float('inf')
        self._avoid_steer = 0.0
        self._avoid_prev_error = None
        self._avoid_prev_time = None

        # 신호등 연동 상태
        self._straight = False        # 초록불에서 켜고 좌회전에서 끈다
        self._traffic_stop = False    # 빨간불에서 켜고 초록/좌회전에서 끈다
        self._traffic_state = 'none'
        self._traffic_at = 0.0
        self._last_red_at = 0.0       # 마지막으로 red 를 본 시각
        self._recent = []             # 최근 신호. 연속 장수를 센다
        self._go_locked_until = 0.0   # 이 시각까지 초록·좌회전을 안 읽는다

        self.tracker = LaneTracker(self._lane_config())
        self._load_model()

        # YOLOv8n 모델 로드
        self._yolo_model = None
        if self.enable_obstacle:
            self._load_yolo()

        self.pub_preview = self.create_publisher(
            Float32MultiArray, '/kobot_yolop/drive_preview', 10)
        self.pub_debug = self.create_publisher(
            Image, '/kobot_yolop/debug_image', 1)
        self.pub_status = self.create_publisher(
            String, '/kobot_yolop/status', 10)
        self.pub_obstacle_dist = self.create_publisher(
            Float32, '/kobot_yolop/obstacle_dist', 10)
        self.pub_obstacle_dir = self.create_publisher(
            String, '/kobot_yolop/obstacle_dir', 10)

        self.pub_motor = None
        self._apply_armed(self.armed, announce=True)
        self.add_on_set_parameters_callback(self._on_parameters)

        self.create_subscription(Image, self.camera_topic, self._on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Bool, '/kobot_yolop/enabled',
                                 self._on_enable, 10)
        self.create_subscription(Bool, '/kobot_yolop/record',
                                 self._on_record, 10)
        self.create_subscription(String, '/traffic_light/state',
                                 self._on_traffic, 10)

        # 대시보드 기록주행 카드가 쓰는 토픽
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
            f'armed={self.armed} roi={self.tracker.config.roi_top_ratio:.2f} '
            f'obstacle={self.enable_obstacle}')

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

        # 장애물 감지 & 회피
        self.declare_parameter('enable_obstacle', True)
        default_yolo_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 'yolov8n.pt')
        self.declare_parameter('yolo_model_path', default_yolo_path)
        self.declare_parameter('yolo_infer_interval', 3)
        self.declare_parameter('yolo_conf_thres', 0.20)
        self.declare_parameter('obstacle_class_ids', [2])
        self.declare_parameter('camera_h_fov_deg', 68.0)
        self.declare_parameter('lidar_angle_offset_rad', -2.1217)
        self.declare_parameter('lidar_valid_min_dist', 0.15)
        self.declare_parameter('lidar_valid_max_dist', 4.0)
        self.declare_parameter('lidar_max_age_sec', 0.5)
        self.declare_parameter('obstacle_avoid_dist', 1.30)
        self.declare_parameter('avoid_hold_frames', 25)
        self.declare_parameter('avoid_speed_ratio', 0.75)
        self.declare_parameter('white_lane_offset_px', 100.0)
        self.declare_parameter('person_stop_dist', 2.0)

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

        # 장애물 설정
        self.enable_obstacle = bool(self._value('enable_obstacle'))
        yolo_path = str(self._value('yolo_model_path'))
        if not yolo_path:
            yolo_path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)), 'yolov8n.pt')
        self.yolo_model_path = yolo_path
        self.yolo_infer_interval = int(self._value('yolo_infer_interval'))
        self.yolo_conf_thres = float(self._value('yolo_conf_thres'))
        self.obstacle_class_ids = [int(x) for x in self._value('obstacle_class_ids')]
        self.camera_h_fov_deg = float(self._value('camera_h_fov_deg'))
        self.lidar_angle_offset_rad = float(self._value('lidar_angle_offset_rad'))
        self.lidar_valid_min_dist = float(self._value('lidar_valid_min_dist'))
        self.lidar_valid_max_dist = float(self._value('lidar_valid_max_dist'))
        self.lidar_max_age_sec = float(self._value('lidar_max_age_sec'))
        self.obstacle_avoid_dist = float(self._value('obstacle_avoid_dist'))
        self.avoid_hold_frames = int(self._value('avoid_hold_frames'))
        self.avoid_speed_ratio = float(self._value('avoid_speed_ratio'))
        self.white_lane_offset_px = float(self._value('white_lane_offset_px'))
        self.person_stop_dist = float(self._value('person_stop_dist'))

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
        """armed 상태를 반영한다. false 면 모터 발행자를 아예 없앤다."""
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
        """armed / max_speed_mps / speed_units / 기타 상수를 실행 중에 바꿀 수 있게 한다."""
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
            elif param.name == 'enable_obstacle':
                self.enable_obstacle = bool(param.value)
                if self.enable_obstacle and self._yolo_model is None:
                    self._load_yolo()
                self.get_logger().warning(f'enable_obstacle = {self.enable_obstacle}')
            elif param.name == 'obstacle_class_ids':
                self.obstacle_class_ids = [int(x) for x in param.value]
                self.get_logger().warning(f'obstacle_class_ids = {self.obstacle_class_ids}')
            elif param.name in self.LIVE_LIMITS:
                low, high = self.LIVE_LIMITS[param.name]
                value = float(param.value)
                if not low <= value <= high:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} 는 {low}~{high} 범위여야 한다')
                if param.name == 'camera_timeout_sec':
                    self.camera_timeout_sec = value
                elif param.name == 'obstacle_avoid_dist':
                    self.obstacle_avoid_dist = value
                elif param.name == 'person_stop_dist':
                    self.person_stop_dist = value
                elif param.name == 'avoid_hold_frames':
                    self.avoid_hold_frames = int(value)
                elif param.name == 'avoid_speed_ratio':
                    self.avoid_speed_ratio = value
                elif param.name == 'white_lane_offset_px':
                    self.white_lane_offset_px = value
                elif param.name == 'yolo_conf_thres':
                    self.yolo_conf_thres = value
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

    def _find_yolo_model_path(self) -> str:
        """yolov8n.pt 모델 경로 탐색."""
        candidates = [
            self.yolo_model_path,
            '/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_vehicle/kobot_vehicle/yolov8n.pt',
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolov8n.pt'),
            '/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_vehicle/yolov8n.pt',
        ]
        for c in candidates:
            if c and os.path.isfile(c):
                return c
        return ''

    def _load_yolo(self) -> None:
        """YOLOv8n 장애물 감지 모델 로드 및 GPU 초기화."""
        model_path = self._find_yolo_model_path()
        if not model_path:
            self.get_logger().error(
                f'YOLOv8n 모델 파일을 찾을 수 없다. 탐색 경로: {self.yolo_model_path}')
            return

        try:
            # ultralytics 모듈 경로 보장
            venv_path = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
            if venv_path not in sys.path:
                sys.path.insert(0, venv_path)

            from ultralytics import YOLO
            self._yolo_model = YOLO(model_path)
            # 워밍업 1회
            dummy = np.zeros((480, 640, 3), dtype=np.uint8)
            self._yolo_model(dummy, imgsz=320, conf=self.yolo_conf_thres, device='cuda:0', verbose=False)
            self.get_logger().info(f'YOLOv8n 적재 및 GPU 준비 완료: {model_path}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLOv8n 모델 적재 실패: {exc}')

    def _infer(self, bgr):
        torch = self._torch
        tensor = torch.from_numpy(
            cv2.resize(bgr, (640, 384))[:, :, ::-1].copy()).cuda()
        tensor = tensor.permute(2, 0, 1).unsqueeze(0).float().div_(255.0)
        if self._engine is not None:
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

    # ------------------------------------------------------------ 장애물 & LiDAR
    def _on_scan(self, message: LaserScan) -> None:
        self._latest_scan = message
        self._latest_scan_at = time.monotonic()

    def _extract_distance_from_bbox(self, x1: int, x2: int, img_width: int) -> float:
        """Bounding Box의 픽셀 범위를 LiDAR 각도로 매핑하여 10th percentile 최단 거리(m) 추출."""
        if self._latest_scan is None:
            return float('inf')
        if (time.monotonic() - self._latest_scan_at) > self.lidar_max_age_sec:
            return float('inf')

        scan = self._latest_scan
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0

        fov_rad = math.radians(self.camera_h_fov_deg)
        angle_start = fov_rad * (0.5 - float(x2) / img_width) - self.lidar_angle_offset_rad
        angle_end = fov_rad * (0.5 - float(x1) / img_width) - self.lidar_angle_offset_rad
        angle_start = math.atan2(math.sin(angle_start), math.cos(angle_start))
        angle_end = math.atan2(math.sin(angle_end), math.cos(angle_end))

        idx_start = int((angle_start - scan.angle_min) / scan.angle_increment)
        idx_end = int((angle_end - scan.angle_min) / scan.angle_increment)

        n = len(ranges)
        if n == 0:
            return float('inf')
        idx_start %= n
        idx_end %= n

        if idx_start > idx_end:
            roi_ranges = np.concatenate((ranges[idx_start:], ranges[:idx_end + 1]))
        else:
            roi_ranges = ranges[idx_start:idx_end + 1]

        valid_ranges = roi_ranges[
            (roi_ranges >= self.lidar_valid_min_dist) & (roi_ranges <= self.lidar_valid_max_dist)
        ]
        if len(valid_ranges) == 0:
            return float('inf')

        return float(np.percentile(valid_ranges, 10))

    def _detect_obstacles(self, bgr, result) -> None:
        """YOLOv8n 추론 및 LiDAR 매칭으로 장애물 거리, 방향, 보행자 감지."""
        if not self.enable_obstacle or self._yolo_model is None:
            return

        height, width = bgr.shape[:2]
        # 실제 천장/머리 위 구조물(상단 15% 약 72px)만 제외 (전방 도로 차량은 정상 검출)
        overhead_y = int(height * 0.15)
        min_obstacle_dist = float('inf')
        closest_obstacle_dir = None
        person_detected = False
        min_person_dist = float('inf')
        detected_boxes = []

        try:
            results = self._yolo_model(
                bgr, imgsz=320, conf=self.yolo_conf_thres, device='cuda:0', verbose=False
            )
            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes
                for box in boxes:
                    cls_id = int(box.cls[0].item())
                    conf = float(box.conf[0].item())

                    # 보행자(0) 또는 obstacle_class_ids 에 포함된 클래스만 처리
                    is_person = (cls_id == 0)
                    is_obstacle = (cls_id in self.obstacle_class_ids)
                    if not is_person and not is_obstacle:
                        continue

                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    if y2 <= overhead_y:
                        continue

                    dist = self._extract_distance_from_bbox(x1, x2, width)
                    cx = (x1 + x2) / 2.0

                    # 노란 중앙선 기준 좌/우 판정
                    if result is not None and result.fit_coefficients is not None:
                        yellow_x = float(np.polyval(result.fit_coefficients, float(y2)))
                    elif result is not None and result.target_far is not None:
                        yellow_x = float(result.target_far)
                    else:
                        yellow_x = width * 0.5

                    obj_dir = 'LEFT' if cx < yellow_x else 'RIGHT'
                    detected_boxes.append((x1, y1, x2, y2, dist, obj_dir, cls_id, conf))

                    if is_person:
                        if dist <= self.person_stop_dist:
                            person_detected = True
                        if dist < min_person_dist:
                            min_person_dist = dist
                    elif is_obstacle:
                        if dist < min_obstacle_dist:
                            min_obstacle_dist = dist
                            closest_obstacle_dir = obj_dir

        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'YOLO 추론 오류: {exc}')

        with self._lock:
            self._last_obstacle_dist = min_obstacle_dist
            self._last_obstacle_dir = closest_obstacle_dir
            self._person_emergency = person_detected
            self._person_dist = min_person_dist
            self._detected_boxes = detected_boxes

        if min_obstacle_dist != float('inf'):
            self.pub_obstacle_dist.publish(Float32(data=float(min_obstacle_dist)))
            dir_msg = String()
            dir_msg.data = closest_obstacle_dir if closest_obstacle_dir else 'NONE'
            self.pub_obstacle_dir.publish(dir_msg)

    def _update_avoidance(self, result) -> None:
        """회피 상태 머신 갱신."""
        if not self.enable_obstacle:
            self._avoid_state = 'STATE_NORMAL'
            return

        dist = self._last_obstacle_dist
        obj_dir = self._last_obstacle_dir

        if dist <= self.obstacle_avoid_dist and obj_dir in ('LEFT', 'RIGHT'):
            if obj_dir == 'LEFT':
                self._avoid_state = 'STATE_AVOID_LEFT'
            else:
                self._avoid_state = 'STATE_AVOID_RIGHT'
            self._avoid_hold_counter = self.avoid_hold_frames
        elif self._avoid_state in ('STATE_AVOID_LEFT', 'STATE_AVOID_RIGHT'):
            if self._avoid_hold_counter > 0:
                self._avoid_hold_counter -= 1
            else:
                self._avoid_state = 'STATE_RETURNING'
        elif self._avoid_state == 'STATE_RETURNING':
            # 노란 중앙선 복귀 확인
            if result is not None and result.basis == 'yellow':
                self._avoid_state = 'STATE_NORMAL'

    def _avoidance_steering(self, bgr, result, now: float) -> float:
        """흰선 마스크 기반 회피 조향 계산."""
        cfg = self.tracker.config
        height, width = bgr.shape[:2]
        centre = width * 0.5

        white_mask = result.white_mask if result is not None else None
        y_sample = int(height * cfg.y_near_ratio)
        if result is not None and result.fit_coefficients is not None:
            center_x = float(np.polyval(result.fit_coefficients, float(y_sample)))
        else:
            center_x = centre

        left_x, right_x = None, None
        if white_mask is not None and y_sample < height:
            y_min = max(0, y_sample - 2)
            y_max = min(height, y_sample + 3)
            sub_mask = white_mask[y_min:y_max]
            xs = np.nonzero(sub_mask.any(axis=0))[0]
            if xs.size > 0:
                left_xs = xs[xs < center_x]
                right_xs = xs[xs > center_x]
                if left_xs.size > 0:
                    left_x = float(left_xs.mean())
                if right_xs.size > 0:
                    right_x = float(right_xs.mean())

        target_x = None
        if self._avoid_state == 'STATE_AVOID_LEFT':
            if right_x is not None:
                target_x = right_x - self.white_lane_offset_px
            elif result is not None and result.target_near is not None:
                target_x = result.target_near + self.white_lane_offset_px
            else:
                target_x = center_x + self.white_lane_offset_px
        elif self._avoid_state == 'STATE_AVOID_RIGHT':
            if left_x is not None:
                target_x = left_x + self.white_lane_offset_px
            elif result is not None and result.target_near is not None:
                target_x = result.target_near - self.white_lane_offset_px
            else:
                target_x = center_x - self.white_lane_offset_px
        elif self._avoid_state == 'STATE_RETURNING':
            if result is not None and result.target_near is not None:
                target_x = result.target_near
            else:
                target_x = center_x

        if target_x is None:
            return result.steer if result is not None else 0.0

        target_x = float(np.clip(target_x, 0.0, float(width)))
        error = float(np.clip((target_x - centre) / centre, -1.0, 1.0))

        dt = 0.05
        if self._avoid_prev_time is not None:
            dt = max(1e-3, min(0.5, now - self._avoid_prev_time))
        derivative = 0.0
        if self._avoid_prev_error is not None:
            derivative = (error - self._avoid_prev_error) / dt
        self._avoid_prev_error, self._avoid_prev_time = error, now

        far = 0.0 if abs(error) < cfg.deadband_norm else error
        raw = far_gain(far, cfg) + cfg.kd * derivative
        raw = cfg.steering_sign * raw

        filtered = (cfg.filter_alpha * raw + (1.0 - cfg.filter_alpha) * self._avoid_steer)
        step = cfg.max_rate * dt
        filtered = max(self._avoid_steer - step, min(self._avoid_steer + step, filtered))
        self._avoid_steer = max(-cfg.max_steer, min(cfg.max_steer, filtered))
        return self._avoid_steer

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

    def _on_traffic(self, message: String) -> None:
        """신호등 상태를 받아 정지와 직진 트리거를 물린다."""
        try:
            state = str(json.loads(message.data).get('state', 'none'))
        except Exception:  # noqa: BLE001
            return
        now = time.monotonic()
        self._traffic_state = state
        self._traffic_at = now

        self._recent.append(state)
        if len(self._recent) > GO_CONFIRM:
            self._recent.pop(0)

        def run_of(want, need):
            tail = self._recent[-need:]
            return len(tail) == need and all(v == want for v in tail)

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
        if now < self._go_locked_until:
            return
        if not run_of(state, GO_CONFIRM):
            return

        self._traffic_stop = False
        self._go_locked_until = now + GO_LOCK_SEC
        self._recent.clear()
        if state == 'green':
            self._straight = True
            self.get_logger().info(
                '초록불 %d장 연속 — 직진 확정, %.0f초간 신호 무시'
                % (GO_CONFIRM, GO_LOCK_SEC))
        else:
            self._straight = False
            self.get_logger().info(
                '좌회전 %d장 연속 — 직진 해제, %.0f초간 신호 무시'
                % (GO_CONFIRM, GO_LOCK_SEC))

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
                             'enabled', 'armed', 'avoid_state',
                             'obstacle_dist', 'obstacle_dir', 'frame'])
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
            avoid_st = self._avoid_state
            obs_dist = self._last_obstacle_dist
            obs_dir = self._last_obstacle_dir
        name = ''
        if index % 3 == 0 and overlay is not None:
            name = f'{index:06d}.jpg'
            cv2.imwrite(str(folder / 'frame' / name), overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
        writer.writerow([f'{time.time():.3f}',
                         result.basis if result else 'none',
                         f'{steer:.2f}', f'{speed:.2f}',
                         f'{result.error_far:.4f}' if result else '0.0',
                         f'{result.error_near:.4f}' if result else '0.0',
                         result.fit_rows if result else 0,
                         result.yellow_blobs if result else 0,
                         result.white_blobs if result else 0,
                         f'{self._latency_ms:.1f}',
                         int(self._enabled), int(self.armed),
                         avoid_st,
                         f'{obs_dist:.2f}' if obs_dist != float('inf') else 'inf',
                         obs_dir or 'NONE',
                         name])
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
                self.tracker.config.straight_only = self._straight
                result = self.tracker.process(bgr, lane, road,
                                              time.monotonic())
                if self.enable_obstacle and (seq % max(1, self.yolo_infer_interval) == 0):
                    self._detect_obstacles(bgr, result)
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

        # 장애물 및 보행자 BBox 시각화
        with self._lock:
            boxes = list(self._detected_boxes)
            avoid_st = self._avoid_state
            person_emg = self._person_emergency
            obs_dist = self._last_obstacle_dist
            obs_dir = self._last_obstacle_dir

        for (bx1, by1, bx2, by2, bdist, bdir, bcls, bconf) in boxes:
            if bcls == 0:  # Person
                bcolor = (0, 0, 255)
                lbl = (f'PERSON {bdist:.2f}m ({bconf:.2f})'
                       if bdist != float('inf') else f'PERSON ({bconf:.2f})')
            else:  # Car
                bcolor = (0, 0, 255) if bdist <= self.obstacle_avoid_dist else (0, 255, 255)
                lbl = (f'CAR {bdir} {bdist:.2f}m ({bconf:.2f})'
                       if bdist != float('inf') else f'CAR {bdir} ({bconf:.2f})')
            cv2.rectangle(overlay, (bx1, by1), (bx2, by2), bcolor, 2)
            cv2.putText(overlay, lbl, (bx1, max(20, by1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, bcolor, 1, cv2.LINE_AA)

        # 상태 텍스트
        if person_emg:
            cv2.putText(overlay, '[EMERGENCY STOP] PERSON DETECTED', (6, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
        elif avoid_st != 'STATE_NORMAL':
            cv2.putText(overlay, f'[{avoid_st}] dist: {obs_dist:.2f}m {obs_dir}', (6, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2, cv2.LINE_AA)

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
                    self._update_avoidance(result)

                    if self._person_emergency:
                        # 보행자 비상 정지 (조향 0, 속도 0)
                        steer = 0.0
                        speed = 0.0
                        basis = 'person_emergency_stop'
                    elif self._traffic_stop:
                        # 빨간불 정지
                        steer = float(np.clip(result.steer, -self.max_steer, self.max_steer))
                        speed = 0.0
                    elif self._avoid_state in ('STATE_AVOID_LEFT', 'STATE_AVOID_RIGHT', 'STATE_RETURNING'):
                        # 장애물 회피 조향 및 속도
                        avoid_steer = self._avoidance_steering(bgr, result, now)
                        steer = float(np.clip(avoid_steer, -self.max_steer, self.max_steer))
                        speed = self.speed_units * self.avoid_speed_ratio
                        basis = self._avoid_state
                    else:
                        # 정상 주행
                        steer = float(np.clip(result.steer, -self.max_steer, self.max_steer))
                        speed = self.speed_units
        else:
            with self._lock:
                self._stale = True

        if self._recording is not None:
            self._write_record(result if payload else None, steer, speed,
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
            avoid_st = self._avoid_state
            obs_dist = self._last_obstacle_dist
            obs_dir = self._last_obstacle_dir
            person_emg = self._person_emergency
        result = payload[0] if payload else None
        if fault:
            mode = 'ERROR'
        elif person_emg:
            mode = 'EMERGENCY_STOP'
        elif avoid_st != 'STATE_NORMAL':
            mode = avoid_st
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
            'avoid_state': avoid_st,
            'obstacle_dist': round(obs_dist, 2) if obs_dist != float('inf') else None,
            'obstacle_dir': obs_dir,
            'person_emergency': bool(person_emg),
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

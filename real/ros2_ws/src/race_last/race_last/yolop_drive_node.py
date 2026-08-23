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
# 아래 세 시간은 "확정한 신호가 시야에서 사라진 시점"부터 센다 (0823 요구).
# 확정만으로 세면 신호등을 아직 지나지도 않았는데 억제가 풀린다.
GO_LOCK_GREEN_SEC = 30.0  # 직진 신호가 사라진 뒤 신호 인식을 쉬는 시간
GO_LOCK_LEFT_SEC = 20.0   # 좌회전 신호가 사라진 뒤 잠금 (0822 실험: 10에서 상향)
GONE_CONFIRM = 5          # 확정한 신호가 이만큼 연속으로 안 보이면 사라진 것
# 콘 구간을 빠져나온 뒤 신호를 더 무시하는 시간. 신호등은 이미 뒤에 있으므로
# 다시 읽을 이유가 없고, 읽으면 오검출로 판단만 흔들린다 (0823 요구).
CONE_EXIT_LOCK_SEC = 20.0
# 사라짐을 기다리는 데에도 상한이 필요하다. 가짜 초록불(127px cyan)이 시야에
# 눌러앉아 억제가 1024초 동안 안 풀린 적이 있다 (0823 실측). 신호가 안
# 사라져도 확정 시점부터 이만큼 지나면 그냥 사라진 것으로 친다.
WAIT_GONE_MAX_SEC = 6.0
# ---- 직진 조향 억제 (0823 저녁 재설계) ----
# 예전: 초록 확정 순간부터 조향×0.15, 신호가 보이는 동안 연장, 사라진 뒤 3초.
#   → 확정 순간 차가 중앙에 안 잡혀 있으면 15% 조향으로 흘러 가끔 이탈했다.
# 지금: 신호가 **사라진 뒤** 먼저 STRAIGHT_HOLD_DELAY 동안 정상 조향(노란선
#   추종)으로 중앙을 잡고, 그 다음 STRAIGHT_HOLD_SEC 동안만 조향을 줄인다.
#   확정~사라짐 사이는 정상 조향이다 (노랑 세로 전파 straight_only 는 그대로).
# 아래 셋은 기본값이고 같은 이름의 파라미터로 주행 중 바꿀 수 있다.
STRAIGHT_HOLD_DELAY_SEC = 1.0  # 사라진 뒤 정상 조향으로 중앙을 잡는 시간
# 0823 저녁: 3.0 -> 0.0 (억제 끔). 직진 구간에서 흰선 중앙을 따라가게 된
# 뒤로는 조향을 따로 줄일 이유가 없다 — 목표가 이미 차로 중앙이다.
# 되살리려면 straight_hold_sec 파라미터에 초를 넣으면 된다.
STRAIGHT_HOLD_SEC = 0.0        # 그 뒤 조향을 줄여 직진만 하는 시간
STRAIGHT_HOLD_GAIN = 0.15      # 그동안 조향에 곱하는 값. 0 이면 완전 직진

# ---- 콘 구간 진입 트리거 (0823) ----
# 첫 라바콘만 차선 위에 놓인다 (실측). 그래서 차선 마스크와 주황이 겹치는
# 픽셀 수가 진입 시점을 가리킨다. 임계는 실측 후 파라미터로 조정한다.
CONE_HSV_LO = (3, 110, 70)     # 주황 하한 (OpenCV HSV, H 0~179)
CONE_HSV_HI = (12, 255, 255)   # 주황 상한. 노란 중앙선(H 30~36)을 뺀다
CONE_DILATE = 15               # 흰 선을 이만큼 부풀려 '선 근처'를 본다.
                               # 콘이 선을 덮으면 그 자리 흰 픽셀이 사라진다
CONE_BAND_TOP = 0.45           # 탐색 띠 상단 — 위쪽은 먼 배경
CONE_BAND_BOT = 0.80           # 하단 — 이 아래는 차체 주황 스티커
CONE_TRIG_CONFIRM = 3          # 이만큼 연속이어야 진입으로 본다

# ---- 앞차 회피. 거리·유지 값은 Lane_follow.py 원본 그대로 ----
AVOID_ENTER_M = 1.30     # 이 거리 안이면 회피 시작
AVOID_CLEAR_M = 1.80     # 이만큼 멀어지면 해제
AVOID_HOLD_SEC = 3.0     # 인식이 꺼진 순간부터 이 시간은 회피 유지,
                         # 지나면 차선 중앙으로 복귀 (0823 요구)
# 같은 방향이 이만큼 연속이어야 차로 인정 (첫 진입만). 단위는 /vehicle/state
# **메시지 수**다 — 추론 횟수가 아니다.
# 카메라 30Hz 로 발행하고 추론은 3프레임당 1회(10Hz)라 같은 판정이 3번씩
# 실려 나간다. 그래서 환산은:
#     3 = 검출 1회로 확정 (0.10초)   ← 0823 저녁 채택
#     5 = 검출 2회 (0.17초)
#     9 = 검출 3회 (0.30초)
# 주의: 이 값은 발행 주기에 묶여 있다. 0823 오전 구독 누수로 발행이 0.5~2Hz
# 였을 때는 5가 2.5초였다. 발행 주기를 바꾸면 이 값도 다시 봐야 한다.
CAR_CONFIRM = 3          # 기본값. 주행 중에는 car_confirm 파라미터로 바꾼다
AVOID_REARM_SEC = 6.0    # 회피가 걸렸던 뒤 이 시간 안에는 한 프레임만 봐도
                         # 바로 회피로 돌아간다. 인식이 깜빡여도 놓지 않는다
VEHICLE_MUTE_SEC = 10.0  # 앞차를 확정하면 이 시간 동안 신호를 무시한다.
                         # 앞차 구간에는 신호등이 없으므로 그 자리의 신호
                         # 판정은 오검출이고, 그것이 회피를 막으면 안 된다
CONE_FRESH_SEC = 0.6     # 콘 관측이 이보다 오래되면 없는 것으로 본다.
                         # 마지막 값을 붙들고 회피를 영구히 막던 문제 (0823)
                         # (0823: 3 -> 5, 순간 오검출로 회피가 서는 것 방지)
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
        self._cone_trig_px = 0        # 차선 위 주황 픽셀 수 (직전 프레임)
        self._cone_trig_run = 0       # 임계를 넘긴 연속 장수
        self._cone_triggered = False  # 콘 구간 진입을 이미 알렸는가
        self._straight_at = 0.0       # 직진(초록)을 확정한 시각 — 주황 감시 지연 기준
        self._lock_kind = None        # 확정했으나 아직 시야에 남아 있는 신호
        self._lock_since = 0.0        # 그 신호를 확정한 시각
        self._gone = 0                # 그 신호를 연속으로 못 본 장수
        # 직진 조향 억제 창 [from, until). 신호가 사라진 시점에 잡힌다.
        self._straight_hold_from = 0.0
        self._straight_hold_until = 0.0

        # 앞차 회피 상태
        self._car_recent = []        # 최근 car.dir. 연속 장수를 센다
        self._avoid_side = None      # 'right' | 'left' | None
        self._avoid_until = 0.0      # 이 시각까지 회피를 유지한다
        self._avoid_rearm_until = 0.0  # 이 시각까지는 한 프레임만 봐도 재무장
        self._car_dist = None
        # 추월을 마쳤다 — 다음 콘 구간 탈출까지 앞차 인식을 끈다 (0823 요구).
        # 이 바퀴 추월 구간은 끝났으므로 나머지 구간의 오검출을 없애고
        # GPU 를 차선에 넘긴다. 콘 탈출(_check_cone_exit)에서 풀린다.
        self._vehicle_passed = False

        self.tracker = LaneTracker(self._lane_config())
        self._load_model()

        self.pub_preview = self.create_publisher(
            Float32MultiArray, '/kobot_yolop/drive_preview', 10)
        self.pub_debug = self.create_publisher(
            Image, '/kobot_yolop/debug_image', 1)
        self.pub_status = self.create_publisher(
            String, '/kobot_yolop/status', 10)
        # 콘 구간 진입을 한 번 알린다. Cone_Perception 이 받아 켜진다.
        self._cone_trigger_pub = self.create_publisher(
            Bool, '/cone/trigger', 10)
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
        self._cone_was_active = False  # 콘 구간 이탈을 잡기 위한 직전 상태
        self._cone_exited = False      # 한 번이라도 콘을 빠져나왔는가
        self._vehicle_force_until = 0.0  # 이 시각까지 앞차 인식을 강제로 켠다
        self.create_subscription(String, '/kobot_yolop/command',
                                 self._on_command, 10)
        self._cone_link = (ConeLink(
            self.get_logger(),
            float(self._value('cone_approach_depth_m')),
            wall_steer=bool(self._value('cone_wall_steer')),
            wall_gain=float(self._value('cone_wall_gain')),
            enter_frames=int(self._value('cone_enter_frames')),
            entry_speed_units=self._cone_entry_units(),
            entry_stable_frames=int(
                self._value('cone_entry_stable_frames')),
            entry_weak_timeout_sec=float(
                self._value('cone_entry_weak_timeout_sec')))
            if self.cone_enabled else None)
        if self._cone_link is not None:
            # Standalone 실차 검증본과 같은 물리 가속도 램프. 트리거/FSM은
            # 건드리지 않고 ConeLink가 활성화된 동안의 상승률만 맞춘다.
            output_mps_per_policy_unit = max(
                1e-6, self.mps_per_unit * self.cone_speed_scale)
            self._cone_link._policy.speed.config.ramp_up_per_s = (
                self.cone_acceleration_limit_mps2
                / output_mps_per_policy_unit)
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
        # 차선 위 주황이 이 픽셀 수를 넘으면 콘 구간 진입. 0 이면 끈다.
        self.declare_parameter('cone_trigger_px', 0)
        self.declare_parameter('cone_speed_scale', 0.7)
        self.declare_parameter('cone_max_speed_mps', 1.5)
        self.declare_parameter('cone_acceleration_limit_mps2', 4.0)
        # 통로가 보이면 진입 전에 이 속도로 미리 감속 (0822 벽 충돌 재발 방지:
        # 2.16 m/s 로 통로 돌진 -> 진입 후 감속으로는 늦다)
        self.declare_parameter('cone_approach_speed_units', 9.0)
        self.declare_parameter('cone_approach_depth_m', 1.2)
        # 콘 조향 근거. True 면 좌·우 벽의 중앙을 겨눈다 (0823 실주행 검증).
        # False 면 예전 policy 조향(짝지은 게이트)으로 돌아간다.
        self.declare_parameter('cone_wall_steer', True)
        self.declare_parameter('cone_wall_gain', 1.5)
        # 초록 확정 뒤 이 시간이 지나야 주황(콘 진입 트리거)을 세기 시작한다.
        # 신호등 바로 앞의 주황 물체로 콘 구간을 잘못 여는 것을 막는다 (0823 요구).
        self.declare_parameter('cone_trigger_delay_sec', 2.0)
        # 라이다가 콘을 본 뒤 콘 모드로 넘어가기까지 필요한 연속 프레임 수.
        # 1 = 즉시 (0823 요구).
        self.declare_parameter('cone_enter_frames', 1)
        # 콘 모드 진입 시 속도 램프의 시작값 [m/s]. 0 이면 예전처럼 0 에서 시작.
        self.declare_parameter('cone_entry_speed_mps', 0.5)
        # 진입 첫 프레임의 단측/임시 경계를 곧바로 조향에 쓰지 않는다.
        # 양쪽 벽의 서로 다른 라이다 프레임 수와 최대 대기시간이다.
        # 시간 초과 뒤에도 정지하지 않고 직전 조향에서 제한 전환한다.
        self.declare_parameter('cone_entry_stable_frames', 2)
        self.declare_parameter('cone_entry_weak_timeout_sec', 0.25)
        # 직진 구간(초록 확정 ~ 콘 트리거) 속도 상한 [m/s]. 0 이면 제한 없음.
        # 0823 실측: 이 구간 끝에서 콘 통로에 2.16 m/s 로 조향 +33 인 채
        # 들어가 우측으로 밀렸다. 통로 진입 속도를 낮춰 그 진입각 오차를 줄인다.
        self.declare_parameter('straight_speed_mps', 1.6)
        # 앞차 확정에 필요한 /vehicle/state 연속 메시지 수 (위 CAR_CONFIRM 주석)
        self.declare_parameter('car_confirm', CAR_CONFIRM)
        # 직진 조향 억제 (상단 상수 주석). 주행 중 ros2 param set 으로 조절.
        self.declare_parameter('straight_hold_delay_sec', STRAIGHT_HOLD_DELAY_SEC)
        self.declare_parameter('straight_hold_sec', STRAIGHT_HOLD_SEC)
        self.declare_parameter('straight_hold_gain', STRAIGHT_HOLD_GAIN)
        # 추월을 마치면 다음 콘 탈출까지 앞차 인식을 끈다. False 면 예전처럼
        # 콘 구간 밖에서는 항상 본다.
        self.declare_parameter('vehicle_off_after_pass', True)

    # 실행 중 바꿀 수 있는 직진 억제 값과 허용 범위
    HOLD_LIMITS = {
        'straight_hold_delay_sec': (0.0, 5.0),
        'straight_hold_sec': (0.0, 10.0),
        'straight_hold_gain': (0.0, 1.0),
    }

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
        self.cone_max_speed_mps = float(
            self._value('cone_max_speed_mps'))
        self.cone_acceleration_limit_mps2 = float(
            self._value('cone_acceleration_limit_mps2'))
        self.cone_approach_units = float(
            self._value('cone_approach_speed_units'))
        self.car_confirm = max(1, int(self._value('car_confirm')))
        self.straight_speed_mps = float(self._value('straight_speed_mps'))
        self.cone_trigger_delay_sec = float(self._value('cone_trigger_delay_sec'))
        self.cone_entry_speed_mps = float(self._value('cone_entry_speed_mps'))
        self.hold_delay_sec = float(self._value('straight_hold_delay_sec'))
        self.hold_sec = float(self._value('straight_hold_sec'))
        self.hold_gain = float(self._value('straight_hold_gain'))
        self.vehicle_off_after_pass = bool(self._value('vehicle_off_after_pass'))
        # 61 단위 = 4.87 m/s. VESC 22500 ERPM 상한과 맞춘다.
        if not 0.0 <= self.speed_units <= 61.0:
            raise ValueError('speed_units 는 0~61 단위로 제한한다')
        if not 0.0 < self.max_steer <= 40.0:
            raise ValueError('max_steer 는 0~40 단위로 제한한다')
        if (not math.isfinite(self.cone_speed_scale)
                or self.cone_speed_scale < 0.0):
            raise ValueError('cone_speed_scale 은 유한한 0 이상 값이어야 한다')
        if (not math.isfinite(self.cone_max_speed_mps)
                or self.cone_max_speed_mps <= 0.0):
            raise ValueError('cone_max_speed_mps 는 유한한 양수여야 한다')
        if (not math.isfinite(self.cone_acceleration_limit_mps2)
                or self.cone_acceleration_limit_mps2 <= 0.0):
            raise ValueError(
                'cone_acceleration_limit_mps2 는 유한한 양수여야 한다')

    def _cone_entry_units(self) -> float:
        """콘 진입 속도 [m/s] → 정책 단위. 노드가 정책 출력에 곱하는
        cone_speed_scale 을 미리 나눠, 실제 모터 명령이 이 m/s 가 되게 한다."""
        if self.cone_entry_speed_mps <= 0.0:
            return 0.0
        return (self.cone_entry_speed_mps / max(self.mps_per_unit, 1e-6)
                / max(self.cone_speed_scale, 1e-6))

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
            elif param.name in self.HOLD_LIMITS:
                low, high = self.HOLD_LIMITS[param.name]
                value = float(param.value)
                if not low <= value <= high:
                    return SetParametersResult(
                        successful=False,
                        reason=f'{param.name} 는 {low}~{high} 범위여야 한다')
                attr = {'straight_hold_delay_sec': 'hold_delay_sec',
                        'straight_hold_sec': 'hold_sec',
                        'straight_hold_gain': 'hold_gain'}[param.name]
                setattr(self, attr, value)
                self.get_logger().warning(
                    f'{param.name} = {value:g} 로 변경')
            elif param.name == 'car_confirm':
                value = int(param.value)
                if not 1 <= value <= 30:
                    return SetParametersResult(
                        successful=False, reason='car_confirm 은 1~30 이어야 한다')
                self.car_confirm = value
                self._car_recent = []
                self.get_logger().warning(
                    '앞차 확정 연속 %d메시지(약 %.2f초)로 변경' % (value, value / 30.0))
            elif param.name == 'cone_trigger_delay_sec':
                value = float(param.value)
                if not 0.0 <= value <= 10.0:
                    return SetParametersResult(
                        successful=False, reason='0~10초 범위여야 한다')
                self.cone_trigger_delay_sec = value
                self.get_logger().warning('주황 감시 지연 %.1f초로 변경' % value)
            elif param.name == 'straight_speed_mps':
                value = float(param.value)
                if not 0.0 <= value <= 4.87:
                    return SetParametersResult(
                        successful=False, reason='0~4.87 m/s 범위여야 한다')
                self.straight_speed_mps = value
                self.get_logger().warning(
                    '직진 구간 속도 상한 %.2f m/s 로 변경' % value)
            elif param.name == 'cone_entry_speed_mps':
                value = float(param.value)
                if not 0.0 <= value <= 2.0:
                    return SetParametersResult(
                        successful=False, reason='0~2 m/s 범위여야 한다')
                self.cone_entry_speed_mps = value
                if self._cone_link is not None:
                    self._cone_link.entry_speed_units = self._cone_entry_units()
                self.get_logger().warning('콘 진입 속도 %.2f m/s 로 변경' % value)
            elif param.name == 'vehicle_off_after_pass':
                self.vehicle_off_after_pass = bool(param.value)
                if not self.vehicle_off_after_pass:
                    self._vehicle_passed = False
                self.get_logger().warning(
                    '추월 후 앞차 인식 끄기 = %s' % self.vehicle_off_after_pass)
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
            # message.data 는 버퍼 프로토콜을 지원한다. bytes() 복사는 불필요.
            buf = np.frombuffer(message.data, dtype=np.uint8)
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
        초록불   GO_CONFIRM 장 연속이면 직진 확정
        좌회전   GO_CONFIRM 장 연속이면 직진 해제. 오염을 그대로 두어 좌측을 따른다
        노란불   아무것도 바꾸지 않는다. 하던 대로 간다
        없음     확정 대기 중이면 사라짐으로 센다. 아니면 하던 대로 간다

        확정한 뒤에는 그 신호가 시야에 남아 있는 동안 신호 잠금을 계속 앞으로
        밀어 둔다. GONE_CONFIRM 장 연속으로 안 보이면 '사라진 것'이고, 그
        시점에 직진 조향 억제 창을 잡는다: 먼저 straight_hold_delay_sec 동안
        정상 조향으로 중앙을 잡고, 그 다음 straight_hold_sec 동안 조향을 줄인다
        (상단 STRAIGHT_HOLD_* 주석, 0823 저녁). 신호가 안 사라지면
        WAIT_GONE_MAX_SEC 에 사라진 것으로 친다.

        초록을 확정할 때마다 콘 진입 트리거를 다시 무장한다 — 2·3바퀴에도
        첫 콘에서 1바퀴와 똑같이 울리게 (0823 저녁 요구). 좌회전 확정은
        _straight 를 내려 트리거가 세지 않게 한다 (지름길은 콘 구간을 건너뛴다).
        """
        try:
            state = str(json.loads(message.data).get('state', 'none'))
        except Exception:  # noqa: BLE001
            return
        now = time.monotonic()
        self._traffic_state = state
        self._traffic_at = now

        # 확정했는데 그 신호가 아직 보인다 — 아직 신호등을 지나지 않았다.
        # 잠금 시각을 계속 밀어 사라질 때까지 유지한다. 여기서도
        # 빨간불은 안 읽는다. 이미 확정한 판단을 흔들 것이 없다.
        if self._lock_kind is not None:
            if now - self._lock_since > WAIT_GONE_MAX_SEC:
                # 안 사라진다. 오검출이 눌러앉은 것으로 보고 여기서 끊는다.
                self.get_logger().warning(
                    '%s 신호가 %.0f초째 안 사라짐 — 오검출로 보고 지금부터 카운트'
                    % (self._lock_kind, WAIT_GONE_MAX_SEC))
                self._signal_gone(now)
                return
            if state == self._lock_kind:
                self._gone = 0
                self._go_locked_until = now + (
                    GO_LOCK_GREEN_SEC if state == 'green'
                    else GO_LOCK_LEFT_SEC)
            else:
                self._gone += 1
                if self._gone >= GONE_CONFIRM:
                    green = self._lock_kind == 'green'
                    self.get_logger().info(
                        '%s 신호가 시야에서 사라짐 — 지금부터 %.0f초 신호 무시%s'
                        % (self._lock_kind,
                           GO_LOCK_GREEN_SEC if green else GO_LOCK_LEFT_SEC,
                           (', %.1f초 뒤 %.1f초 조향 억제'
                            % (self.hold_delay_sec, self.hold_sec))
                           if green else ''))
                    self._signal_gone(now)
            return

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
        self._lock_kind = state   # 사라질 때까지 위에서 계속 밀어 준다
        self._lock_since = now    # 다만 WAIT_GONE_MAX_SEC 를 넘기지는 않는다
        self._gone = 0
        self._recent.clear()
        # 새 확정 — 지난 억제 창은 지운다 (사라짐 시점에 다시 잡힌다)
        self._straight_hold_from = 0.0
        self._straight_hold_until = 0.0
        if state == 'green':
            self._straight = True
            self._straight_at = now      # 여기서부터 cone_trigger_delay_sec 를 센다
            # 콘 진입 트리거 재무장. 1바퀴 첫 확정에서는 이미 무장 상태라
            # 아무 일도 없고, 2·3바퀴 신호등을 지나며 다시 확정되면 그 바퀴
            # 첫 콘에서 다시 울린다. (Cone_Perception 은 트리거 뒤 15초 창이고
            # 좌회전·빨강에서 꺼진다 — 거기는 그대로다.)
            if self._cone_triggered:
                self.get_logger().info('초록 재확정 — 콘 진입 트리거 재무장')
            self._cone_triggered = False
            self._cone_trig_run = 0
            # 새 바퀴 시작 — 지난 바퀴 '추월 완료' 도 푼다. 콘 탈출 이벤트가 안 오는
            # 경우(트리거·게이트 실패, 대기 중 오검출 추월)에 앞차 인식이 경기 내내
            # 꺼진 채 남지 않게 하는 안전망. 콘 접근/진입 중엔 어차피 꺼진다.
            if self._vehicle_passed:
                self.get_logger().info('초록 확정 — 앞차 인식 다시 켬 (지난 추월 완료 해제)')
            self._vehicle_passed = False
            self.get_logger().info(
                '초록불 %d장 연속 — 직진 확정. 신호가 시야에서 사라지면'
                ' %.1f초 정상 조향 뒤 %.1f초 조향 억제, %.0f초 신호 무시'
                % (GO_CONFIRM, self.hold_delay_sec, self.hold_sec,
                   GO_LOCK_GREEN_SEC))
        else:
            self._straight = False
            self.get_logger().info(
                '좌회전 %d장 연속 — 직진 해제(콘 트리거도 안 센다). 신호가'
                ' 사라지면 그때부터 %.0f초간 신호 무시'
                % (GO_CONFIRM, GO_LOCK_LEFT_SEC))

    def _signal_gone(self, now: float) -> None:
        """확정한 신호가 시야에서 사라졌다(또는 사라진 것으로 친다).

        초록이었으면 직진 조향 억제 창을 지금 기준으로 잡는다:
            [now + delay, now + delay + hold)
        delay 동안은 정상 조향으로 차선 중앙을 잡고, 그 뒤 hold 동안 줄인다.
        """
        if self._lock_kind == 'green':
            self._straight_hold_from = now + max(0.0, self.hold_delay_sec)
            self._straight_hold_until = (
                self._straight_hold_from + max(0.0, self.hold_sec))
        self._lock_kind = None
        self._gone = 0

    def _hold_active(self, now: float) -> bool:
        """지금 직진 조향 억제 중인가."""
        return self._straight_hold_from <= now < self._straight_hold_until

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
        # 지금 보고 있는 관측일 때만 의미가 있다. 콘 인식이 꺼지면
        # /cone/reading 이 끊기는데, 마지막 값이 '콘 2개' 라 그 뒤로 회피가
        # 한 번도 안 걸린 적이 있다 (0823 실측).
        if _cone_d is not None:
            _age = time.monotonic() - float(_cone_d.get('stamp', 0.0))
            if _age > CONE_FRESH_SEC:
                _cone_d = None
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
        # 2026-08-23: 거리 조건을 뺀다. 실측에서 dist=1.67m 라 AVOID_ENTER_M(1.3)
        # 를 못 넘겨 _avoid_until 이 갱신되지 않았고, avoid_side 는 'right' 인데
        # offset 이 0/90 을 깜빡이며 차가 중앙으로 돌아갔다.
        # 좌/우가 보이면 회피, 안 보이면 해제. 거리는 로그용으로만 남긴다.
        near = bool(boxes)
        far = not boxes

        self._car_recent.append(side if near else 'NONE')
        while len(self._car_recent) > self.car_confirm:
            self._car_recent.pop(0)
        steady = (len(self._car_recent) == self.car_confirm
                  and len(set(self._car_recent)) == 1)

        # 2026-08-23: 연속 N메시지는 '진입' 조건으로만 쓴다. 한 번 회피 중이면
        # 같은 쪽이 한 번만 보여도 유지 시간을 갱신한다 (아래 holding).
        # 이미 회피 중이면 같은 쪽이 한 번만 보여도 유지 시간을 갱신한다.
        want_now = 'right' if side == 'LEFT' else 'left'
        holding = (self._avoid_side is not None
                   and self._avoid_side == want_now
                   and near and side in ('LEFT', 'RIGHT'))
        if holding:
            self._avoid_until = now + AVOID_HOLD_SEC
        # 회피가 걸렸던 직후라면 5프레임 확정을 건너뛴다. 인식이 깜빡여도
        # (실측 58% 프레임에서만 검출) 회피를 놓지 않기 위한 것 (0823).
        rearm = now < self._avoid_rearm_until

        if (steady or holding or rearm) and near and side in ('LEFT', 'RIGHT'):
            want = want_now
            if self._avoid_side != want:
                self.get_logger().warning(
                    '앞차 %s %s — %s 흰선으로 회피'
                    % (side,
                       ('%.2fm' % float(dist)) if dist is not None else '거리없음',
                       '우측' if want == 'right' else '좌측'))
            self._avoid_side = want
            self._avoid_until = now + AVOID_HOLD_SEC
            self._avoid_rearm_until = now + AVOID_REARM_SEC
            # 앞차가 먼저다. 이 구간엔 신호등이 없으니 신호 판정을 무시하고,
            # 신호등 통과용인 직진 조향억제도 푼다 (0823 요구).
            self._go_locked_until = max(self._go_locked_until,
                                        now + VEHICLE_MUTE_SEC)
            self._lock_kind = None
            if self._straight_hold_until > now:
                self.get_logger().info('앞차 확정 — 직진 조향억제 해제')
            self._straight_hold_from = 0.0
            self._straight_hold_until = 0.0
        elif far and self._avoid_side and now > self._avoid_until:
            self._avoid_side = None
            if self.vehicle_off_after_pass:
                # 이 바퀴 추월은 끝났다. 다음 콘 구간을 빠져나올 때까지 앞차
                # 인식을 끈다 (0823 요구). 대시보드 vehicle_force / reset 로 풀 수 있다.
                self._vehicle_passed = True
                self.get_logger().info(
                    '앞차 벗어남 — 회피 해제. 추월 완료로 보고 다음 콘 탈출까지 앞차 인식 끔')
            else:
                self.get_logger().info('앞차 벗어남 — 회피 해제')

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
        if self._hold_active(now):
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
                # 직진 구간(초록 확정 ~ 콘 트리거)에는 **둘 다** 켠다.
                #  · straight_only      노랑 오염 방지 (세로 전파 + 흰색 되덮기)
                #  · white_center_only  노란선을 목표에서 빼고 흰선 중앙만
                # 오염 방지가 없으면 흰선이 노랑 덩이에 먹혀 white_mask 에서
                # 빠지고, 그러면 흰선이 2줄로 안 잡혀 흰선 중앙 추종 자체가
                # 성립하지 않는다 (0823 프레임309 실측: 흰 덩이 2개 -> 1개).
                # 콘 트리거가 울리면 둘 다 꺼져 노란선 우선으로 돌아간다 —
                # 그 구간은 오염된 노란선을 따라가는 것이 정상이다.
                self.tracker.config.straight_only = self._straight
                self.tracker.config.white_center_only = self._straight
                # 앞차 회피. 0 이면 기존 동작 그대로다.
                self.tracker.config.avoid_offset_px = self._avoid_offset(
                    time.monotonic())
                result = self.tracker.process(bgr, lane, road,
                                              time.monotonic())
                self._latency_ms = (time.perf_counter() - started) * 1000.0
                try:
                    self._check_cone_trigger(bgr, result.white_mask)
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warning(f'콘 트리거 생략: {exc}')
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

    def _check_cone_trigger(self, bgr, white) -> None:
        """흰 차선 위(근처)에 주황이 얹혀 있는가 — 콘 구간 진입 판정.

        첫 콘만 흰 차선 위에 놓이므로 이 겹침은 진입 순간에만 크게 나온다.
        노란 중앙선은 색상으로 빼고, 차체 앞범퍼 스티커는 띠로 뺀다.
        콘이 선을 덮으면 그 자리 흰 픽셀이 사라지므로 흰 마스크를 부풀린다.
        """
        # 이미 한 번 울렸으면(1회성 래치, reset 으로만 풀린다) 셀 이유가 없다.
        # 전체 프레임 HSV 변환 + 15x15 팽창을 그 뒤에도 매 추론마다 하고
        # 있었다 (0823). 울리기 전에는 임계·초록불과 무관하게 계속 센다 —
        # 콘 앞에 차를 두고 cone_trig_px 를 보며 임계를 맞추는 작업이 이
        # 값에 기댄다.
        if white is None or self._cone_triggered:
            self._cone_trig_px = 0
            self._cone_trig_run = 0
            return
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        orange = cv2.inRange(hsv, CONE_HSV_LO, CONE_HSV_HI) > 0
        near = cv2.dilate(white.astype(np.uint8),
                          np.ones((CONE_DILATE, CONE_DILATE), np.uint8)) > 0
        height = orange.shape[0]
        band = np.zeros_like(orange)
        band[int(height * CONE_BAND_TOP):int(height * CONE_BAND_BOT)] = True
        self._cone_trig_px = int(np.count_nonzero(orange & near & band))

        threshold = int(self._value('cone_trigger_px'))
        if threshold <= 0:
            return                     # 0 이면 트리거를 끈 것이다
        if not self._straight:
            self._cone_trig_run = 0    # 초록불 확정 전에는 세지 않는다
            return
        # 초록 확정 직후 잠깐은 세지 않는다. 신호등 앞에서 주황 물체를 보고
        # 콘 구간을 여는 오발동을 막는다 (0823 요구).
        if time.monotonic() - self._straight_at < self.cone_trigger_delay_sec:
            self._cone_trig_run = 0
            return
        if self._cone_trig_px >= threshold:
            self._cone_trig_run += 1
        else:
            self._cone_trig_run = 0
        if self._cone_trig_run >= CONE_TRIG_CONFIRM and not self._cone_triggered:
            self._cone_triggered = True
            self._cone_trigger_pub.publish(Bool(data=True))
            # 직진 구간은 여기서 끝난다 (0823 저녁 요구). 노란선 무시를 풀어
            # 콘을 빠져나온 뒤 곧바로 노란선 추종으로 이어지게 한다 —
            # 탈출 시점에 따로 켜 줄 필요가 없다.
            self._straight = False
            self._straight_hold_from = 0.0
            self._straight_hold_until = 0.0
            # 라이다 조건을 기다리지 않고 바로 콘 모드로 (0823 요구)
            if self._cone_link is not None:
                self._cone_link.request_enter()
            self.get_logger().warning(
                '차선 위 주황 %d px 이 %d장 연속 — 콘 구간 진입, '
                '직진(흰선 중앙) 종료 → 노란선 추종 복귀'
                % (self._cone_trig_px, CONE_TRIG_CONFIRM))

    def _on_command(self, message: String) -> None:
        """대시보드에서 오는 명령. 노드 안에 래치된 상태를 밖에서 푼다.

        하루 동안 같은 사고가 반복됐는데 전부 래치 때문이었고, 밖에서 풀
        방법이 없어 매번 서비스를 재시작해야 했다 (0823).
        """
        try:
            body = json.loads(message.data)
            cmd = str(body.get('cmd', ''))
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning('명령 해석 실패: %r' % message.data[:60])
            return
        now = time.monotonic()
        sec = float(body.get('sec', 0.0) or 0.0)

        if cmd == 'reset':
            self._reset_latches()
            self.get_logger().warning('완전 초기화 — 래치된 상태를 모두 풀었다')
        elif cmd == 'release':
            self._apply_armed(False, announce=True)
            self._enabled = False
            self._publish_motor(*STOP)
            self.get_logger().warning('모터 점유 해제 — armed 내림')
        elif cmd == 'signal_mute':
            self._go_locked_until = now + max(0.0, sec)
            self._lock_kind = None
            self.get_logger().warning('신호 인식 %.0f초 끔' % sec)
        elif cmd == 'signal_wake':
            self._go_locked_until = 0.0
            self._lock_kind = None
            self._gone = 0
            self._recent.clear()
            self.get_logger().warning('신호 잠금 해제 — 지금부터 다시 읽는다')
        elif cmd == 'vehicle_force':
            self._vehicle_force_until = now + max(0.0, sec)
            self.get_logger().warning('앞차 인식 %.0f초 강제 켬' % sec)
        else:
            self.get_logger().warning('모르는 명령: %s' % cmd)

    def _reset_latches(self) -> None:
        """래치된 상태를 전부 초기값으로 되돌린다. 주행 중에도 안전하다 —
        조향·속도 자체는 매 프레임 새로 계산되고, 여기서 지우는 것은 과거
        판정의 잔상뿐이다."""
        self._go_locked_until = 0.0
        self._lock_kind = None
        self._gone = 0
        self._recent.clear()
        self._straight = False
        self._straight_hold_from = 0.0
        self._straight_hold_until = 0.0
        self._traffic_stop = False
        self._last_red_at = 0.0
        self._car_recent = []
        self._avoid_side = None
        self._avoid_until = 0.0
        self._car_dist = None
        self._vehicle_passed = False
        self._cone_reading = None
        self._cone_was_active = False
        self._cone_exited = False
        self._cone_triggered = False
        self._cone_trig_run = 0
        self._vehicle_force_until = 0.0
        self._fault = None
        link = self._cone_link
        if link is not None:
            link._active = False
            link.approach = False
            link._in.reset()
            link._out.reset()
            link._wall_prev = None

    def _check_cone_exit(self, now: float) -> None:
        """콘 구간을 빠져나온 순간을 잡는다.

        빠져나오면 앞차 인식을 켜고(잠금과 무관), 신호는 오히려 더 무시한다.
        신호등은 이미 뒤에 있으므로 다시 읽으면 판단만 흔들린다.
        """
        link = self._cone_link
        if link is None:
            return
        if self._cone_was_active and not link.active:
            self._cone_exited = True
            self._go_locked_until = max(self._go_locked_until,
                                        now + CONE_EXIT_LOCK_SEC)
            self._lock_kind = None      # 사라짐 대기는 여기서 끝낸다
            # 새 추월 구간이 온다 — 지난 바퀴의 '추월 완료'를 푼다
            self._vehicle_passed = False
            self.get_logger().warning(
                '라바콘 구간 이탈 — 앞차 인식 켬, 신호는 %.0f초 더 무시'
                % CONE_EXIT_LOCK_SEC)
        self._cone_was_active = bool(link.active)

    def _vehicle_needed(self, now: float) -> bool:
        """차량 인식을 돌려야 하는가. Vehicle_Perception 이 이 값을 따른다."""
        # 대시보드에서 강제로 켠 동안은 무조건 본다
        if now < self._vehicle_force_until:
            return True
        # 이 바퀴 추월을 마쳤다 — 다음 콘 탈출까지 끈다 (vehicle_off_after_pass)
        if self._vehicle_passed:
            return False
        # 콘 구간(진입 또는 접근)에서는 콘을 차량으로 오인한다
        link = self._cone_link
        if link is not None and (link.active or link.approach):
            return False
        # 신호 잠금은 앞차 인식을 끄지 않는다 (0823).
        #  · 코스상 앞차 구간이 초록불 잠금 30초 안에 들어가, 정작 필요한
        #    구간에서 앞차를 못 봤다 (실측: armed 289행 중 274행이 꺼짐)
        #  · 앞차를 확정하면 신호를 무시하도록 바꾼 뒤로는, 그 잠금이 다시
        #    앞차 인식을 끄는 모순까지 생긴다
        #  · CPU 여유도 있다 — 추론 54ms, 추론 주기 102ms
        # 잠금은 신호 재판정만 막고, 앞차는 콘 구간이 아니면 항상 본다.
        return True

    def _maybe_publish_debug(self, bgr, result) -> None:
        now = time.monotonic()
        if now - self._last_overlay < 1.0 / max(self.debug_hz, 0.1):
            return
        # 아무도 안 보는 오버레이(마스크 칠하기 + 921KB 발행, 7Hz)는 만들지
        # 않는다. 대시보드(web_video_server)가 붙거나 기록 중일 때만 그린다.
        # 본선(dashboard:=false)에서는 구독자가 없다 (2026-08-23).
        if (self.pub_debug.get_subscription_count() == 0
                and self._recording is None):
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
                    # 직진 신호가 사라진 뒤 (delay 동안 중앙을 잡고 나서)
                    # hold 동안 조향을 줄여 사실상 직진만 한다.
                    # 삼거리에서 좌회전 차선으로 끌려가는 것을 막는다.
                    if self._hold_active(now):
                        steer *= self.hold_gain
                    # 빨간불이면 속도만 0 으로 떨군다. 조향은 그대로 둔다.
                    speed = 0.0 if self._traffic_stop else self.speed_units
                    # 직진 구간은 상한을 따로 둔다 (콘 통로에 천천히 닿게)
                    if self._straight and self.straight_speed_mps > 0.0:
                        speed = min(speed, self.straight_speed_mps
                                    / max(self.mps_per_unit, 1e-6))
        else:
            with self._lock:
                self._stale = True
        # 기록은 결과가 있는 틱에만 적는다. (원본은 이 호출이 위 else 안에
        # 있어 — 결과가 **없는** 틱에만 불리고 그때 result 가 미정의라
        # NameError 로 제어 타이머가 죽을 수 있었다. 0823 정리.)
        if self._recording is not None and payload is not None:
            self._write_record(result, steer, speed,
                               getattr(self, '_last_overlay_image', None))

        # ── 라바콘 구간: kmu_race 이식 조향(cone_link)으로 대체 ──
        if self._cone_link is not None and fault is None and enabled:
            with self._lock:
                cone_d = self._cone_reading
                cam_ok = payload is not None and not self._stale
            self._check_cone_exit(now)
            cone_cmd = self._cone_link.tick(
                now, cone_d,
                ego_mps=speed * self.mps_per_unit,
                camera_ok=cam_ok,
                handoff_steer=steer)
            if cone_cmd is not None:
                steer = float(np.clip(cone_cmd[0], -self.max_steer,
                                      self.max_steer))
                cone_speed = min(cone_cmd[1] * self.cone_speed_scale,
                                 self.speed_units,
                                 self.cone_max_speed_mps
                                 / max(self.mps_per_unit, 1e-6))
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
        # avoid_left/right 는 흰선을 정상 추종하는 상태다. 2026-08-23:
        # 목록에 없어 LOST 로 떨어졌고, 대시보드가 READY_MODES
        # (TRACKING/HOLD) 를 못 봐 '8초 안에 유효한 차선을 확인하지
        # 못했습니다' 로 기록주행 시작이 거부됐다.
        elif basis in ('yellow', 'white_center',
                       'avoid_left', 'avoid_right',
                       'cone', 'cone_approach'):
            # 콘 구간은 라이다 벽을 정상 추종하는 상태다. 차선을 놓친 게
            # 아니므로 LOST 로 떨어지면 안 된다 (0823: 대시보드가 기록주행을
            # 끊었다 — avoid 와 같은 원인이었다).
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
            'straight_speed_mps': round(self.straight_speed_mps, 2),
            # straight_hold: 억제가 진행 중이면 남은 초, 아니면 0.
            # straight_hold_in: 억제가 예약돼 있으면 시작까지 남은 초(중앙 잡는 중).
            'straight_hold': round(
                (self._straight_hold_until - now)
                if self._hold_active(now) else 0.0, 2),
            'straight_hold_in': round(max(
                0.0, self._straight_hold_from - now)
                if now < self._straight_hold_until else 0.0, 2),
            'vehicle_passed': bool(self._vehicle_passed),
            'cone_trig_px': int(self._cone_trig_px),
            'cone_exited': bool(self._cone_exited),
            'vehicle_force_s': round(max(
                0.0, self._vehicle_force_until - now), 1),
            'cone_triggered': bool(self._cone_triggered),
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

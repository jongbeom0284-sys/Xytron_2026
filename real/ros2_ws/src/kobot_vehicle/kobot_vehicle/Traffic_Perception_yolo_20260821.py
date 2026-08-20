#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import collections
import os
import sys
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

# YOLOv8 라이브러리 (ultralytics)
try:
    from ultralytics import YOLO
except ImportError:
    venv_site = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
    if os.path.exists(venv_site) and venv_site not in sys.path:
        sys.path.insert(0, venv_site)
    from ultralytics import YOLO


def nothing(x):
    pass


# ==========================================
# 튜닝 변수 기본 설정 (전역상수 - 최종 실차판)
# ==========================================

# ---- 토픽 설정 ----
TOPIC_IMAGE_SUB       = '/image_raw/compressed'
TOPIC_STATE_PUB       = '/traffic_light/state'
TOPIC_DEBUG_PUB       = '/traffic_light/debug_img'
TOPIC_HSV_DEBUG_PUB   = '/traffic_light/hsv_debug'

# ---- YOLO 설정 ----
MODEL_PATH            = os.path.join(os.path.dirname(__file__), 'yolov8n.pt')
CONFIDENCE_THRESHOLD  = 0.01  # 좌회전 화살표 등 초저신뢰도(0.02) 검출 대응
ALLOWED_CLASS_IDS     = [9, 74]  # 9 = traffic light, 74 = clock (원형 렌즈 오인식 허용)
USE_FP16              = True
IMGSZ                 = 320

# ---- 연산 스케줄링 ----
MAX_PROCESSING_HZ     = 10.0  # 초당 최대 YOLO 추론 횟수

# ---- ROI (관심영역 4방향 절단 설정) ----
ROI_TOP_RATIO         = 0.10  # 상단 천장/조명 배제 (상단 10% 컷)
ROI_BOTTOM_RATIO      = 0.40  # 하단 도로 배제 (기존 50% -> 40%로 하단 추가 절단)
ROI_LEFT_RATIO        = 0.15  # 좌측 벽면/배경 배제 (좌측 15% 컷)
ROI_RIGHT_RATIO       = 0.85  # 우측 벽면/배경 배제 (우측 15% 컷, 85% 지점까지 탐색)

# ---- HSV 전처리 설정 ----
GAUSSIAN_KSIZE        = 5     # 가우시안 블러 커널 크기 (홀수, 0이면 비활성)
MORPH_KERNEL_SIZE     = 0     # 모폴로지 연산 커널 크기 (0이면 비활성)

# ---- HSV 색상 초기값 (Red, Green 전용) ----
# 1. Red 영역 (두 구간 결합)
INIT_RED_HUE_LOW1     = 0
INIT_RED_HUE_HIGH1    = 10
INIT_RED_HUE_LOW2     = 160
INIT_RED_HUE_HIGH2    = 180
INIT_RED_SAT_LOW      = 80
INIT_RED_VAL_LOW      = 80

# 2. Green 영역
INIT_GREEN_HUE_LOW    = 40
INIT_GREEN_HUE_HIGH   = 90
INIT_GREEN_SAT_LOW    = 80
INIT_GREEN_VAL_LOW    = 80

# ---- 판별 임계값 초기값 ----
INIT_MIN_PIXEL_RATIO       = 5    # 천분율 (5 = 0.005)
INIT_MIN_PIXEL_ABSOLUTE    = 10   # 절대 최소 픽셀 수
INIT_GREEN_ARROW_THRESHOLD = 15   # 백분율 (15 = 15% 이하 시 좌회전 화살표)

# ---- BBox 필터 및 패딩 ----
MIN_ASPECT_RATIO      = 0.0    # 세로/가로 종횡비 하한 (기본 0.0)
BBOX_PADDING_RATIO    = 0.05   # BBox 상하좌우 여유 패딩 비율 (빛 번짐/외곽 절단 방지)

# ---- Temporal Smoothing & Hysteresis ----
STATE_CONFIRM_COUNT   = 3      # N연속 동일 판별 결과일 때 상태 변경 후보
MIN_STATE_HOLD_SEC    = 0.5    # 상태 전환 후 최소 유지 시간 (초)

# ---- 디버그 모드 ----
DEBUG_MODE            = True   # ROS 토픽 퍼블리시 여부
HSV_DEBUG_MODE        = True   # HSV 마스크 디버그 퍼블리시 여부
USE_CV2_WINDOW        = True   # 화면(X11 디스플레이)에 cv2.imshow 및 트랙바 창 띄우기


class TrafficLightClassifierYOLO(Node):
    def __init__(self):
        super().__init__('traffic_light_classifier_yolo')
        self.bridge = CvBridge()

        # ----------------------------------
        # YOLOv8 모델 로드 및 웜업 추론
        # ----------------------------------
        self.get_logger().info(f'Loading YOLOv8 model from: {MODEL_PATH}')
        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f'Model file not found: {MODEL_PATH}')
            raise FileNotFoundError(f'Model file not found: {MODEL_PATH}')

        self.model = YOLO(MODEL_PATH)
        self.class_names = self.model.names if hasattr(self.model, 'names') else {}
        self.get_logger().info(
            f'Model loaded. Target Classes: {ALLOWED_CLASS_IDS} {[self.class_names.get(c, c) for c in ALLOWED_CLASS_IDS]}. '
            f'Conf: {CONFIDENCE_THRESHOLD}. ROI: Y[{ROI_TOP_RATIO:.2f}~{ROI_BOTTOM_RATIO:.2f}], X[{ROI_LEFT_RATIO:.2f}~{ROI_RIGHT_RATIO:.2f}]. '
            f'Warming up CUDA engine...'
        )
        
        dummy_img = np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8)
        try:
            self.model.predict(
                source=dummy_img,
                imgsz=IMGSZ,
                half=USE_FP16,
                classes=ALLOWED_CLASS_IDS,
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )
            self.get_logger().info('YOLOv8 warm-up inference completed successfully.')
        except Exception as e:
            self.get_logger().warn(f'Warm-up inference warning: {e}')

        # ----------------------------------
        # 상태 변수 초기화
        # ----------------------------------
        self.last_inference_time = 0.0
        self.confirmed_state = 'Unknown'
        self.last_transition_time = 0.0
        self.state_history = collections.deque(maxlen=STATE_CONFIRM_COUNT)
        self.last_raw_state = 'Unknown'
        self.last_best_box = None
        self.last_confidence = 0.0
        self.last_cls_id = 9
        self.frame_count = 0
        self.last_frame_received_time = 0.0

        # ----------------------------------
        # X11 GUI 창 및 HSV 트랙바(조절 바) 초기화
        # ----------------------------------
        self.gui_available = False
        if USE_CV2_WINDOW and os.environ.get("DISPLAY"):
            try:
                cv2.namedWindow("Traffic Light Debug", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Traffic Light Debug", 640, 480)

                if HSV_DEBUG_MODE:
                    cv2.namedWindow("HSV Mask Debug", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("HSV Mask Debug", 420, 260)

                cv2.namedWindow("HSV Tuner", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("HSV Tuner", 420, 440)

                # 1. Red 슬라이더
                cv2.createTrackbar("R_H_LOW1", "HSV Tuner", INIT_RED_HUE_LOW1, 180, nothing)
                cv2.createTrackbar("R_H_HIGH1", "HSV Tuner", INIT_RED_HUE_HIGH1, 180, nothing)
                cv2.createTrackbar("R_H_LOW2", "HSV Tuner", INIT_RED_HUE_LOW2, 180, nothing)
                cv2.createTrackbar("R_H_HIGH2", "HSV Tuner", INIT_RED_HUE_HIGH2, 180, nothing)
                cv2.createTrackbar("R_SAT_MIN", "HSV Tuner", INIT_RED_SAT_LOW, 255, nothing)
                cv2.createTrackbar("R_VAL_MIN", "HSV Tuner", INIT_RED_VAL_LOW, 255, nothing)

                # 2. Green 슬라이더
                cv2.createTrackbar("G_H_LOW", "HSV Tuner", INIT_GREEN_HUE_LOW, 180, nothing)
                cv2.createTrackbar("G_H_HIGH", "HSV Tuner", INIT_GREEN_HUE_HIGH, 180, nothing)
                cv2.createTrackbar("G_SAT_MIN", "HSV Tuner", INIT_GREEN_SAT_LOW, 255, nothing)
                cv2.createTrackbar("G_VAL_MIN", "HSV Tuner", INIT_GREEN_VAL_LOW, 255, nothing)
                cv2.createTrackbar("G_ARROW_%", "HSV Tuner", INIT_GREEN_ARROW_THRESHOLD, 100, nothing)

                # 3. 감도 설정
                cv2.createTrackbar("MIN_PX_ABS", "HSV Tuner", INIT_MIN_PIXEL_ABSOLUTE, 100, nothing)

                self.gui_available = True
                self.get_logger().info('X11 OpenCV GUI & HSV Tuner Windows initialized successfully.')
                self._draw_waiting_screen("Waiting for Camera (/image_raw/compressed)...")
            except Exception as e:
                self.get_logger().warn(f'GUI window init skipped: {e}')
                self.gui_available = False

        self.gui_timer = self.create_timer(0.05, self._gui_heartbeat)

        # ----------------------------------
        # ROS 토픽 설정 (QoS 최신 프레임 우선)
        # ----------------------------------
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sub_img = self.create_subscription(
            CompressedImage,
            TOPIC_IMAGE_SUB,
            self.img_callback,
            sub_qos
        )

        self.pub_state = self.create_publisher(String, TOPIC_STATE_PUB, 10)
        
        if DEBUG_MODE:
            self.pub_debug_img = self.create_publisher(Image, TOPIC_DEBUG_PUB, 10)
        if HSV_DEBUG_MODE:
            self.pub_hsv_debug_img = self.create_publisher(Image, TOPIC_HSV_DEBUG_PUB, 10)

        self.get_logger().info(
            f'Traffic Perception Node is Running. Subscribing {TOPIC_IMAGE_SUB} at max {MAX_PROCESSING_HZ} Hz.'
        )

    def _get_hsv_params(self):
        """ 트랙바가 활성화되어 있으면 트랙바 실시간 값 반환, 아니면 기본 전역상수 반환 """
        if self.gui_available:
            try:
                r_h1 = cv2.getTrackbarPos("R_H_LOW1", "HSV Tuner")
                r_h2 = cv2.getTrackbarPos("R_H_HIGH1", "HSV Tuner")
                r_h3 = cv2.getTrackbarPos("R_H_LOW2", "HSV Tuner")
                r_h4 = cv2.getTrackbarPos("R_H_HIGH2", "HSV Tuner")
                r_s  = cv2.getTrackbarPos("R_SAT_MIN", "HSV Tuner")
                r_v  = cv2.getTrackbarPos("R_VAL_MIN", "HSV Tuner")

                g_h1 = cv2.getTrackbarPos("G_H_LOW", "HSV Tuner")
                g_h2 = cv2.getTrackbarPos("G_H_HIGH", "HSV Tuner")
                g_s  = cv2.getTrackbarPos("G_SAT_MIN", "HSV Tuner")
                g_v  = cv2.getTrackbarPos("G_VAL_MIN", "HSV Tuner")
                g_arrow_pct = cv2.getTrackbarPos("G_ARROW_%", "HSV Tuner") / 100.0

                min_abs = cv2.getTrackbarPos("MIN_PX_ABS", "HSV Tuner")

                return {
                    'r_h1': r_h1, 'r_h2': r_h2, 'r_h3': r_h3, 'r_h4': r_h4, 'r_s': r_s, 'r_v': r_v,
                    'g_h1': g_h1, 'g_h2': g_h2, 'g_s': g_s, 'g_v': g_v, 'g_arrow_thres': g_arrow_pct,
                    'min_abs': min_abs
                }
            except Exception:
                pass

        return {
            'r_h1': INIT_RED_HUE_LOW1, 'r_h2': INIT_RED_HUE_HIGH1,
            'r_h3': INIT_RED_HUE_LOW2, 'r_h4': INIT_RED_HUE_HIGH2,
            'r_s': INIT_RED_SAT_LOW, 'r_v': INIT_RED_VAL_LOW,
            'g_h1': INIT_GREEN_HUE_LOW, 'g_h2': INIT_GREEN_HUE_HIGH,
            'g_s': INIT_GREEN_SAT_LOW, 'g_v': INIT_GREEN_VAL_LOW,
            'g_arrow_thres': INIT_GREEN_ARROW_THRESHOLD / 100.0,
            'min_abs': INIT_MIN_PIXEL_ABSOLUTE
        }

    def _draw_waiting_screen(self, text):
        if not self.gui_available:
            return
        wait_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(wait_img, text, (30, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
        cv2.putText(wait_img, "Please check if 'sensors.launch.py' is running.", (30, 280),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 150), 1)
        cv2.imshow("Traffic Light Debug", wait_img)

        if HSV_DEBUG_MODE:
            hsv_wait = np.zeros((260, 420, 3), dtype=np.uint8)
            cv2.putText(hsv_wait, "Waiting for frames...", (80, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1)
            cv2.imshow("HSV Mask Debug", hsv_wait)
        cv2.waitKey(1)

    def _gui_heartbeat(self):
        if self.gui_available:
            if (time.time() - self.last_frame_received_time) > 1.0:
                self._draw_waiting_screen("Waiting for Camera (/image_raw/compressed)...")
            else:
                cv2.waitKey(1)

    def detect_color(self, roi_img):
        """
        신호등 ROI 영역 내에서 Red, Green 색상만 마스킹하여 상태를 판별합니다.
        (노란색 배제)
        """
        h_roi, w_roi = roi_img.shape[:2]
        box_area = h_roi * w_roi
        if box_area == 0:
            return 'Unknown', None, {}

        # 1. 실시간 HSV 파라미터 읽기
        params = self._get_hsv_params()

        # 2. 가우시안 블러 전처리 (고주파 노이즈 제거)
        if GAUSSIAN_KSIZE > 0:
            k = GAUSSIAN_KSIZE if GAUSSIAN_KSIZE % 2 == 1 else GAUSSIAN_KSIZE + 1
            blurred = cv2.GaussianBlur(roi_img, (k, k), 0)
        else:
            blurred = roi_img

        # 3. BGR -> HSV 변환
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 4. 마스크 생성 (Red, Green)
        # Red (두 구간 결합)
        mask_r1 = cv2.inRange(hsv, np.array([params['r_h1'], params['r_s'], params['r_v']]),
                                  np.array([params['r_h2'], 255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([params['r_h3'], params['r_s'], params['r_v']]),
                                  np.array([params['r_h4'], 255, 255]))
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        # Green
        mask_green = cv2.inRange(hsv, np.array([params['g_h1'], params['g_s'], params['g_v']]),
                                     np.array([params['g_h2'], 255, 255]))

        # 5. 모폴로지 연산 (작은 잡음 픽셀 제거)
        if MORPH_KERNEL_SIZE > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

        # 6. 유효 픽셀 카운트
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)

        red_ratio = red_pixels / float(box_area)
        green_ratio = green_pixels / float(box_area)

        min_req_pixels = max(params['min_abs'], int(box_area * (INIT_MIN_PIXEL_RATIO / 1000.0)))

        # 7. 신호등 상태 판별 로직 (노란색 제외)
        # [Case 1] Red + Green 동시 검출 -> 빨간불 좌회전 (Red_Left)
        if red_pixels >= min_req_pixels and green_pixels >= min_req_pixels:
            raw_state = 'Red_Left'
        # [Case 2] Green이 지배적일 때
        elif green_pixels >= min_req_pixels and green_pixels >= red_pixels:
            if green_ratio <= params['g_arrow_thres']:
                raw_state = 'Green_Left'  # 픽셀 비율이 적으면 화살표 형태
            else:
                raw_state = 'Green'       # 픽셀 비율이 크면 둥근 원형 출발 신호
        # [Case 3] Red 단일
        elif red_pixels >= min_req_pixels and red_pixels > green_pixels:
            raw_state = 'Red'
        else:
            raw_state = 'Off'

        # 8. HSV 디버그 시각화 창 구성 (Red / Green 2단 뷰)
        hsv_debug_frame = None
        if HSV_DEBUG_MODE or self.gui_available:
            target_h = 240
            target_w = int(w_roi * (target_h / float(max(1, h_roi))))
            target_w = max(120, min(240, target_w))

            roi_disp = cv2.resize(roi_img, (target_w, target_h))

            row_h = target_h // 2
            mask_w = 260

            # Red 뷰
            m_r_res = cv2.resize(mask_red, (mask_w, row_h))
            view_r = cv2.cvtColor(m_r_res, cv2.COLOR_GRAY2BGR)
            view_r[m_r_res > 0] = [0, 0, 255]
            cv2.putText(view_r, f"R: {red_pixels}px ({red_ratio*100:.1f}%)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            # Green 뷰
            m_g_res = cv2.resize(mask_green, (mask_w, row_h))
            view_g = cv2.cvtColor(m_g_res, cv2.COLOR_GRAY2BGR)
            view_g[m_g_res > 0] = [0, 255, 0]
            cv2.putText(view_g, f"G: {green_pixels}px ({green_ratio*100:.1f}%)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

            right_stack = np.vstack([view_r, view_g])
            hsv_debug_frame = np.hstack([roi_disp, right_stack])

            header_bar = np.zeros((30, hsv_debug_frame.shape[1], 3), dtype=np.uint8)
            cv2.putText(header_bar, f"HSV State: {raw_state} (min: {min_req_pixels}px)",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            hsv_debug_frame = np.vstack([header_bar, hsv_debug_frame])

        counts = {
            'red': red_pixels, 'red_ratio': red_ratio,
            'green': green_pixels, 'green_ratio': green_ratio,
            'min_req': min_req_pixels
        }

        return raw_state, hsv_debug_frame, counts

    def img_callback(self, msg: CompressedImage):
        current_time = time.time()
        self.frame_count += 1
        self.last_frame_received_time = current_time

        # ----------------------------------------------------
        # [Step 1] 연산 스케줄링 (MAX_PROCESSING_HZ 스로틀링)
        # ----------------------------------------------------
        interval = 1.0 / max(0.1, MAX_PROCESSING_HZ)
        if (current_time - self.last_inference_time) < interval:
            self.pub_state.publish(String(data=self.confirmed_state))
            if self.gui_available:
                cv2.waitKey(1)
            return

        self.last_inference_time = current_time

        # ----------------------------------------------------
        # [Step 2] CompressedImage 디코딩
        # ----------------------------------------------------
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_img is None:
                self.get_logger().warn('Failed to decode compressed image.')
                return
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        h, w = cv_img.shape[:2]
        debug_img = cv_img.copy()

        # ----------------------------------------------------
        # [Step 3] 4방향 ROI Crop (상/하/좌/우 절단)
        # ----------------------------------------------------
        crop_top_y   = int(h * ROI_TOP_RATIO)
        crop_bot_y   = int(h * ROI_BOTTOM_RATIO)
        crop_left_x  = int(w * ROI_LEFT_RATIO)
        crop_right_x = int(w * ROI_RIGHT_RATIO)

        crop_top_y   = max(0, min(h - 1, crop_top_y))
        crop_bot_y   = max(crop_top_y + 10, min(h, crop_bot_y))
        crop_left_x  = max(0, min(w - 1, crop_left_x))
        crop_right_x = max(crop_left_x + 10, min(w, crop_right_x))

        cv_img_roi = cv_img[crop_top_y:crop_bot_y, crop_left_x:crop_right_x]

        # ----------------------------------------------------
        # [Step 4] YOLOv8 객체 탐지 (신호등 9 + 시계 74)
        # ----------------------------------------------------
        best_box = None
        best_conf = 0.0
        best_cls = 9
        max_area = 0
        raw_state = 'Unknown'
        hsv_debug_frame = None

        try:
            results = self.model.predict(
                source=cv_img_roi,
                imgsz=IMGSZ,
                half=USE_FP16,
                classes=ALLOWED_CLASS_IDS,
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )

            if len(results) > 0 and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                for box in boxes:
                    conf = float(box.conf[0].item())
                    cls_id = int(box.cls[0].item())
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    bw = x2 - x1
                    bh = y2 - y1
                    if bw <= 0 or bh <= 0:
                        continue

                    # BBox 종횡비 필터
                    aspect = bh / float(bw)
                    if MIN_ASPECT_RATIO > 0.0 and aspect < MIN_ASPECT_RATIO:
                        continue

                    area = bw * bh
                    if area > max_area:
                        max_area = area
                        best_conf = conf
                        best_cls = cls_id
                        best_box = (x1 + crop_left_x, y1 + crop_top_y, x2 + crop_left_x, y2 + crop_top_y)

        except Exception as e:
            self.get_logger().error(f'YOLO prediction error: {e}')

        # ----------------------------------------------------
        # [Step 5] BBox 영역 색상 판별 (HSV 마스킹)
        # ----------------------------------------------------
        if best_box is not None:
            x1, y1, x2, y2 = best_box

            pad_w = int((x2 - x1) * BBOX_PADDING_RATIO)
            pad_h = int((y2 - y1) * BBOX_PADDING_RATIO)
            x1_p = max(0, x1 - pad_w)
            y1_p = max(0, y1 - pad_h)
            x2_p = min(w, x2 + pad_w)
            y2_p = min(h, y2 + pad_h)

            roi_target = cv_img[y1_p:y2_p, x1_p:x2_p]
            if roi_target.size > 0:
                raw_state, hsv_debug_frame, counts = self.detect_color(roi_target)
        else:
            raw_state = 'Unknown'

        self.last_raw_state = raw_state
        self.last_best_box = best_box
        self.last_confidence = best_conf
        self.last_cls_id = best_cls

        # ----------------------------------------------------
        # [Step 6] Temporal Smoothing & Hysteresis 필터
        # ----------------------------------------------------
        self.state_history.append(raw_state)

        if len(self.state_history) == STATE_CONFIRM_COUNT:
            if all(s == raw_state for s in self.state_history):
                if raw_state != self.confirmed_state:
                    time_since_last_trans = current_time - self.last_transition_time
                    if time_since_last_trans >= MIN_STATE_HOLD_SEC:
                        self.confirmed_state = raw_state
                        self.last_transition_time = current_time
                        cls_name = self.class_names.get(best_cls, str(best_cls))
                        self.get_logger().info(
                            f"🚦 [Traffic Light] State Change -> {self.confirmed_state} (Conf: {best_conf:.2f}, Cls: {best_cls} {cls_name})"
                        )

        # ----------------------------------------------------
        # [Step 7] 상태 토픽 퍼블리시
        # ----------------------------------------------------
        self.pub_state.publish(String(data=self.confirmed_state))

        # ----------------------------------------------------
        # [Step 8] 디버그 오버레이 화면 구성 (4방향 ROI 박스 표시)
        # ----------------------------------------------------
        cv2.rectangle(debug_img, (crop_left_x, crop_top_y), (crop_right_x, crop_bot_y), (255, 140, 0), 2)
        roi_info_text = f"ROI Box: Y[{ROI_TOP_RATIO*100:.0f}%~{ROI_BOTTOM_RATIO*100:.0f}%], X[{ROI_LEFT_RATIO*100:.0f}%~{ROI_RIGHT_RATIO*100:.0f}%]"
        cv2.putText(debug_img, roi_info_text, (crop_left_x + 5, crop_top_y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 140, 0), 1)

        state_colors = {
            'Red': (0, 0, 255),
            'Green': (0, 255, 0),
            'Green_Left': (0, 200, 100),
            'Red_Left': (255, 0, 255),
            'Off': (128, 128, 128),
            'Unknown': (200, 200, 200)
        }
        box_color = state_colors.get(self.confirmed_state, (255, 255, 255))

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cls_name = self.class_names.get(best_cls, str(best_cls))
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), box_color, 2)
            label_text = f"TL [{cls_name}]: {self.confirmed_state} ({best_conf:.2f})"
            (lw, lh), _ = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(debug_img, (x1, max(0, y1 - lh - 8)), (x1 + lw + 6, max(lh + 8, y1)), box_color, -1)
            cv2.putText(debug_img, label_text, (x1 + 3, max(lh + 2, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        else:
            cv2.putText(debug_img, f"TL: {self.confirmed_state} (No Box)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)

        info_text = f"Confirmed: {self.confirmed_state} | Raw: {raw_state} | Hist: {list(self.state_history)}"
        cv2.putText(debug_img, info_text, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # ----------------------------------------------------
        # [Step 9] 디버그 영상 퍼블리시 및 GUI 창 출력
        # ----------------------------------------------------
        if DEBUG_MODE:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
                self.pub_debug_img.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f'Debug image pub error: {e}')

        if HSV_DEBUG_MODE:
            if hsv_debug_frame is None:
                hsv_debug_frame = np.zeros((180, 380, 3), dtype=np.uint8)
                cv2.putText(hsv_debug_frame, "Waiting for Detection...",
                            (40, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

            try:
                hsv_msg = self.bridge.cv2_to_imgmsg(hsv_debug_frame, encoding="bgr8")
                self.pub_hsv_debug_img.publish(hsv_msg)
            except Exception as e:
                self.get_logger().error(f'HSV debug pub error: {e}')

        # X11 GUI 창 출력
        if self.gui_available:
            try:
                cv2.imshow("Traffic Light Debug", debug_img)
                if hsv_debug_frame is not None:
                    cv2.imshow("HSV Mask Debug", hsv_debug_frame)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f"cv2.imshow error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightClassifierYOLO()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gui_available:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

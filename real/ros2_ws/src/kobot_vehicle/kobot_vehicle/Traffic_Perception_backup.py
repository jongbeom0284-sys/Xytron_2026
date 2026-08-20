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
    # clrernet_eval venv 경로 fallback
    venv_site = '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages'
    if os.path.exists(venv_site) and venv_site not in sys.path:
        sys.path.insert(0, venv_site)
    from ultralytics import YOLO

# ==========================================
# 튜닝 변수 설정 (전역상수 - 백업본)
# ==========================================

# ---- 토픽 설정 ----
TOPIC_IMAGE_SUB       = '/image_raw/compressed'
TOPIC_STATE_PUB       = '/traffic_light/state'
TOPIC_DEBUG_PUB       = '/traffic_light/debug_img'
TOPIC_HSV_DEBUG_PUB   = '/traffic_light/hsv_debug'

# ---- YOLO 설정 ----
MODEL_PATH            = os.path.join(os.path.dirname(__file__), 'yolov8n.pt')
CONFIDENCE_THRESHOLD  = 0.05
TRAFFIC_LIGHT_CLASS_ID = 9  # COCO dataset: 9 = traffic light (클래스 확인 후 수정 예정)
USE_FP16              = True
IMGSZ                 = 320

# ---- 연산 스케줄링 ----
MAX_PROCESSING_HZ     = 10.0  # 초당 최대 YOLO 추론 횟수

# ---- ROI 설정 ----
ROI_TOP_RATIO         = 0.0   # 상단 시작 비율 (0.0 = 맨 위)
ROI_BOTTOM_RATIO      = 0.5   # 하단 끝 비율 (0.5 = 상단 50%만 탐색)

# ---- HSV 전처리 설정 ----
GAUSSIAN_KSIZE        = 5     # 가우시안 블러 커널 크기 (홀수, 0이면 비활성)
MORPH_KERNEL_SIZE     = 3     # 모폴로지 연산 커널 크기 (0이면 비활성)

# ---- HSV 색상 임계값 ----
# 1. Red 영역 (두 구간 결합)
RED_HUE_LOW1          = 0
RED_HUE_HIGH1         = 10
RED_HUE_LOW2          = 160
RED_HUE_HIGH2         = 180
RED_SAT_LOW           = 80
RED_VAL_LOW           = 80

# 2. Green 영역
GREEN_HUE_LOW         = 40
GREEN_HUE_HIGH        = 90
GREEN_SAT_LOW         = 80
GREEN_VAL_LOW         = 80

# 3. Yellow 영역
YELLOW_HUE_LOW        = 15
YELLOW_HUE_HIGH       = 35
YELLOW_SAT_LOW        = 80
YELLOW_VAL_LOW        = 80

# ---- 판별 임계값 ----
MIN_PIXEL_RATIO       = 0.005  # BBox 면적 대비 최소 픽셀 비율
MIN_PIXEL_ABSOLUTE    = 10     # 절대 최소 픽셀 수
GREEN_ARROW_THRESHOLD = 0.15   # Green 픽셀 면적비율이 이 이하면 좌회전 화살표(Green_Left), 초과면 원형(Green)

# ---- BBox 필터 ----
MIN_ASPECT_RATIO      = 0.0    # 세로/가로 종횡비 하한 (기본 0.0, 필요시 1.2 등 활성화)
BBOX_PADDING_RATIO    = 0.05   # BBox 상하좌우 여유 패딩 비율 (빛 번짐/외곽 절단 방지)

# ---- Temporal Smoothing & Hysteresis ----
STATE_CONFIRM_COUNT   = 3      # N연속 동일 판별 결과일 때 상태 변경 후보
MIN_STATE_HOLD_SEC    = 0.5    # 상태 전환 후 최소 유지 시간 (초)

# ---- 디버그 모드 ----
DEBUG_MODE            = True   # ROS 토픽 퍼블리시 여부
HSV_DEBUG_MODE        = True   # HSV 마스크 디버그 퍼블리시 여부
USE_CV2_WINDOW        = True   # 화면(X11 디스플레이)에 cv2.imshow 창 띄우기


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
        self.get_logger().info('Model loaded. Warming up CUDA engine...')
        
        dummy_img = np.zeros((IMGSZ, IMGSZ, 3), dtype=np.uint8)
        try:
            self.model.predict(
                source=dummy_img,
                imgsz=IMGSZ,
                half=USE_FP16,
                classes=[TRAFFIC_LIGHT_CLASS_ID],
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
        self.frame_count = 0
        self.last_frame_received_time = 0.0

        # X11 GUI 창 사용 가능 여부 확인
        self.gui_available = False
        if USE_CV2_WINDOW and os.environ.get("DISPLAY"):
            try:
                cv2.namedWindow("Traffic Light Debug", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Traffic Light Debug", 640, 480)
                if HSV_DEBUG_MODE:
                    cv2.namedWindow("HSV Mask Debug", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("HSV Mask Debug", 420, 280)
                self.gui_available = True
                self.get_logger().info('X11 OpenCV GUI Windows initialized successfully.')
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
            hsv_wait = np.zeros((280, 420, 3), dtype=np.uint8)
            cv2.putText(hsv_wait, "Waiting for frames...", (80, 140),
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
        h_roi, w_roi = roi_img.shape[:2]
        box_area = h_roi * w_roi
        if box_area == 0:
            return 'Unknown', None, {}

        if GAUSSIAN_KSIZE > 0:
            k = GAUSSIAN_KSIZE if GAUSSIAN_KSIZE % 2 == 1 else GAUSSIAN_KSIZE + 1
            blurred = cv2.GaussianBlur(roi_img, (k, k), 0)
        else:
            blurred = roi_img

        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_r1 = cv2.inRange(hsv, np.array([RED_HUE_LOW1, RED_SAT_LOW, RED_VAL_LOW]),
                                  np.array([RED_HUE_HIGH1, 255, 255]))
        mask_r2 = cv2.inRange(hsv, np.array([RED_HUE_LOW2, RED_SAT_LOW, RED_VAL_LOW]),
                                  np.array([RED_HUE_HIGH2, 255, 255]))
        mask_red = cv2.bitwise_or(mask_r1, mask_r2)

        mask_green = cv2.inRange(hsv, np.array([GREEN_HUE_LOW, GREEN_SAT_LOW, GREEN_VAL_LOW]),
                                     np.array([GREEN_HUE_HIGH, 255, 255]))

        mask_yellow = cv2.inRange(hsv, np.array([YELLOW_HUE_LOW, YELLOW_SAT_LOW, YELLOW_VAL_LOW]),
                                      np.array([YELLOW_HUE_HIGH, 255, 255]))

        if MORPH_KERNEL_SIZE > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE))
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
            mask_yellow = cv2.morphologyEx(mask_yellow, cv2.MORPH_OPEN, kernel)

        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        yellow_pixels = cv2.countNonZero(mask_yellow)

        red_ratio = red_pixels / float(box_area)
        green_ratio = green_pixels / float(box_area)
        yellow_ratio = yellow_pixels / float(box_area)

        min_req_pixels = max(MIN_PIXEL_ABSOLUTE, int(box_area * MIN_PIXEL_RATIO))

        if red_pixels >= min_req_pixels and green_pixels >= min_req_pixels:
            raw_state = 'Red_Left'
        elif green_pixels >= min_req_pixels and green_pixels >= red_pixels and green_pixels >= yellow_pixels:
            if green_ratio <= GREEN_ARROW_THRESHOLD:
                raw_state = 'Green_Left'
            else:
                raw_state = 'Green'
        elif red_pixels >= min_req_pixels and red_pixels >= green_pixels and red_pixels >= yellow_pixels:
            raw_state = 'Red'
        elif yellow_pixels >= min_req_pixels and yellow_pixels >= red_pixels and yellow_pixels >= green_pixels:
            raw_state = 'Yellow'
        else:
            raw_state = 'Off'

        hsv_debug_frame = None
        if HSV_DEBUG_MODE or self.gui_available:
            target_h = 240
            target_w = int(w_roi * (target_h / float(max(1, h_roi))))
            target_w = max(120, min(240, target_w))

            roi_disp = cv2.resize(roi_img, (target_w, target_h))

            row_h = target_h // 3
            mask_w = 260

            m_r_res = cv2.resize(mask_red, (mask_w, row_h))
            view_r = cv2.cvtColor(m_r_res, cv2.COLOR_GRAY2BGR)
            view_r[m_r_res > 0] = [0, 0, 255]
            cv2.putText(view_r, f"R: {red_pixels}px ({red_ratio*100:.1f}%)", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            m_g_res = cv2.resize(mask_green, (mask_w, row_h))
            view_g = cv2.cvtColor(m_g_res, cv2.COLOR_GRAY2BGR)
            view_g[m_g_res > 0] = [0, 255, 0]
            cv2.putText(view_g, f"G: {green_pixels}px ({green_ratio*100:.1f}%)", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            m_y_res = cv2.resize(mask_yellow, (mask_w, row_h))
            view_y = cv2.cvtColor(m_y_res, cv2.COLOR_GRAY2BGR)
            view_y[m_y_res > 0] = [0, 255, 255]
            cv2.putText(view_y, f"Y: {yellow_pixels}px ({yellow_ratio*100:.1f}%)", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            right_stack = np.vstack([view_r, view_g, view_y])
            hsv_debug_frame = np.hstack([roi_disp, right_stack])

            header_bar = np.zeros((30, hsv_debug_frame.shape[1], 3), dtype=np.uint8)
            cv2.putText(header_bar, f"HSV State: {raw_state} (min: {min_req_pixels}px)",
                        (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            hsv_debug_frame = np.vstack([header_bar, hsv_debug_frame])

        counts = {
            'red': red_pixels, 'red_ratio': red_ratio,
            'green': green_pixels, 'green_ratio': green_ratio,
            'yellow': yellow_pixels, 'yellow_ratio': yellow_ratio,
            'min_req': min_req_pixels
        }

        return raw_state, hsv_debug_frame, counts

    def img_callback(self, msg: CompressedImage):
        current_time = time.time()
        self.frame_count += 1
        self.last_frame_received_time = current_time

        interval = 1.0 / max(0.1, MAX_PROCESSING_HZ)
        if (current_time - self.last_inference_time) < interval:
            self.pub_state.publish(String(data=self.confirmed_state))
            if self.gui_available:
                cv2.waitKey(1)
            return

        self.last_inference_time = current_time

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

        crop_top_y = int(h * ROI_TOP_RATIO)
        crop_bot_y = int(h * ROI_BOTTOM_RATIO)
        cv_img_roi = cv_img[crop_top_y:crop_bot_y, 0:w]

        best_box = None
        best_conf = 0.0
        max_area = 0
        raw_state = 'Unknown'
        hsv_debug_frame = None

        try:
            results = self.model.predict(
                source=cv_img_roi,
                imgsz=IMGSZ,
                half=USE_FP16,
                classes=[TRAFFIC_LIGHT_CLASS_ID],
                conf=CONFIDENCE_THRESHOLD,
                verbose=False
            )

            if len(results) > 0 and len(results[0].boxes) > 0:
                boxes = results[0].boxes
                for box in boxes:
                    conf = float(box.conf[0].item())
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    bw = x2 - x1
                    bh = y2 - y1
                    if bw <= 0 or bh <= 0:
                        continue

                    aspect = bh / float(bw)
                    if MIN_ASPECT_RATIO > 0.0 and aspect < MIN_ASPECT_RATIO:
                        continue

                    area = bw * bh
                    if area > max_area:
                        max_area = area
                        best_conf = conf
                        best_box = (x1, y1 + crop_top_y, x2, y2 + crop_top_y)

        except Exception as e:
            self.get_logger().error(f'YOLO prediction error: {e}')

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

        self.state_history.append(raw_state)

        if len(self.state_history) == STATE_CONFIRM_COUNT:
            if all(s == raw_state for s in self.state_history):
                if raw_state != self.confirmed_state:
                    time_since_last_trans = current_time - self.last_transition_time
                    if time_since_last_trans >= MIN_STATE_HOLD_SEC:
                        self.confirmed_state = raw_state
                        self.last_transition_time = current_time
                        self.get_logger().info(
                            f"🚦 [Traffic Light] State Change -> {self.confirmed_state} (Conf: {best_conf:.2f})"
                        )

        self.pub_state.publish(String(data=self.confirmed_state))

        cv2.line(debug_img, (0, crop_bot_y), (w, crop_bot_y), (255, 120, 0), 2)
        cv2.putText(debug_img, "ROI Detection Limit", (10, crop_bot_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 120, 0), 1)

        state_colors = {
            'Red': (0, 0, 255),
            'Green': (0, 255, 0),
            'Green_Left': (0, 200, 100),
            'Yellow': (0, 255, 255),
            'Red_Left': (255, 0, 255),
            'Off': (128, 128, 128),
            'Unknown': (200, 200, 200)
        }
        box_color = state_colors.get(self.confirmed_state, (255, 255, 255))

        if best_box is not None:
            x1, y1, x2, y2 = best_box
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), box_color, 2)
            label_text = f"TL: {self.confirmed_state} ({best_conf:.2f})"
            cv2.putText(debug_img, label_text, (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
        else:
            cv2.putText(debug_img, f"TL: {self.confirmed_state} (No Box)", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)

        info_text = f"Confirmed: {self.confirmed_state} | Raw: {raw_state} | Hist: {list(self.state_history)}"
        cv2.putText(debug_img, info_text, (10, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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

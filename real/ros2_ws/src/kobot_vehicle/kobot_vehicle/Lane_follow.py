#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 압축 이미지 및 2D LiDAR 기반 통합 자율주행 Node
- 최우선: 노란색 차선(단일/점선 모노레일) 추종
- 차선 소실 시: 흰색 차선(좌/우/양쪽) Fallback 추종
- YOLOv8n + 2D LiDAR 센서 퓨전 장애물 감지 및 좌/우 분류
- 정적 장애물 회피 상태 머신 (좌측 장애물 -> 우측 흰색 차선 우회 / 우측 장애물 -> 좌측 흰색 차선 우회)
- 장애물 통과 후 노란색 중앙선 자동 복귀
- 적응형 피팅(1차/2차) 및 궤적 외삽(Extrapolation), 속도/조향 필터링
"""

import os
import sys

# ==============================================================================
# [중요] 최신 numpy 및 가상환경 패키지를 시스템 기본보다 최우선으로 로드
# ==============================================================================
VENV_SITE_PACKAGES = [
    '/home/xytron/clrernet_eval/venv/lib/python3.10/site-packages',
    '/home/xytron/2026_Xytron/real/.venv/lib/python3.10/site-packages'
]
for p in VENV_SITE_PACKAGES:
    if os.path.exists(p) and p not in sys.path:
        sys.path.insert(0, p)

import time
import math
import cv2
import numpy as np

# YOLOv8 안전 로드
YOLO_AVAILABLE = False
YOLO_IMPORT_ERROR = ""
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception as _yolo_err:
    YOLO_IMPORT_ERROR = str(_yolo_err)
    YOLO_AVAILABLE = False

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from std_msgs.msg import Float32MultiArray, Float32, String
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def nothing(x):
    pass

def find_single_peak(hist, min_height=5 * 255):
    """ 히스토그램에서 가장 높은 단 1개의 피크 위치를 반환합니다. """
    max_idx = np.argmax(hist)
    if hist[max_idx] >= min_height:
        return max_idx
    return None

class Config:
    USE_GUI = True         # GUI 창(cv2.imshow, Trackbar) 띄우기 활성화
    PUBLISH_DEBUG = True   # /debug/lane_image 토픽 퍼블리시 여부
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    
    NWINDOWS = 6
    MARGIN = 40
    MINPIX = 10
    
    ROI_TOP_RATIO = 0.5 
    LOOKAHEAD_RATIO = 0.6  
    
    # PID 제어
    Kp = 0.15
    Ki = 0.0000
    Kd = 0.05
    
    # error 이동평균 필터: 최근 N개 프레임의 error를 평균내어 노이즈 제거
    ERROR_BUFFER_SIZE = 5
    
    EMA_ALPHA_ANGLE = 0.4
    EMA_ALPHA_SPEED = 0.1
    
    # 적응형 피팅: 데이터 부족 시 2차 곡선 대신 1차 직선 사용
    MIN_PIXELS_QUADRATIC = 60   # 2차 피팅에 필요한 최소 픽셀 수
    MIN_WINDOWS_QUADRATIC = 3   # 2차 피팅에 필요한 최소 활성 윈도우 수
    MAX_CURVATURE = 0.005       # 2차 계수(a) 절대값 상한 (초과 시 직선으로 되돌림)
    
    # 궤적 외삽: 점선 끊김 구간에서 최근 추세 기반 위치 예측
    TARGET_X_BUFFER_SIZE = 10   # 최근 target_x 기록 버퍼 크기
    GHOST_MAX_FRAMES = 20       # 점선 끊김 시 최대 예측 프레임 수
    
    # 흰색 차선 Fallback 설정
    WHITE_LANE_HALF_WIDTH = 180 # 한쪽 흰색 차선만 보일 때 중앙 추정 반폭 (픽셀)
    
    # ==========================================================================
    # [YOLOv8 + LiDAR 센서 퓨전 및 장애물 회피 파라미터]
    # ==========================================================================
    ENABLE_YOLO = True
    MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'yolov8n.pt')
    YOLO_INFER_INTERVAL = 3     # N프레임당 1회 추론 (제어 루프 실시간 30FPS 보장)
    YOLO_CONF_THRES = 0.50      # 객체 탐지 신뢰도 임계값
    IGNORE_CLASSES = [72, 62, 28] # 무시할 COCO 클래스 (신호등, 표지판 등)
    CLASS_PERSON = 0            # COCO 'person'
    
    CAMERA_H_FOV_DEG = 68.0     # 카메라 수평 화각 (도)
    LIDAR_VALID_MIN_DIST = 0.15 # LiDAR 유효 최소 거리 (m)
    LIDAR_VALID_MAX_DIST = 4.0  # LiDAR 유효 최대 거리 (m)
    
    # 장애물 회피 상태 머신 거리 임계값
    OBSTACLE_AVOID_DIST = 1.30  # 회피 시작 거리 (m 이내일 때 회피 모드 진입)
    OBSTACLE_CLEAR_DIST = 1.80  # 장애물이 완전히 멀어졌다고 판단하는 거리 (m)
    AVOID_HOLD_FRAMES = 25      # 장애물 소실 후 차선 유지 프레임 수 (약 0.8초)
    AVOID_SPEED = 20.0          # 회피 주행 중 안전 속도
    
    BASE_SPEED = 30.0
    MIN_SPEED = 15.0
    SPEED_DECAY = 0.6

class TrackDriveNode(Node):
    def __init__(self):
        super().__init__("track_drive_node")
        self.bridge = CvBridge()
        
        # 최신 프레임 1개만 유지 (지연 누적에 의한 제어 발산 방지)
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.camera_sub = self.create_subscription(
            CompressedImage, 
            "/image_raw/compressed", 
            self.image_callback, 
            qos_cam
        )
        
        # LiDAR 구독자 (qos_profile_sensor_data 적용)
        self.latest_scan = None
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )
        
        self.motor_pub = self.create_publisher(Float32MultiArray, "/xycar_motor", 10)
        self.obstacle_dist_pub = self.create_publisher(Float32, "/yolo/obstacle_dist", 10)
        self.obstacle_dir_pub = self.create_publisher(String, "/yolo/obstacle_dir", 10)
        
        # ======================================================================
        # [HSV 차선 파라미터 (ROS2 파라미터 선언)]
        # ======================================================================
        # 1. 노란색 차선 HSV 파라미터 (실측 튜닝값)
        self.declare_parameter('h_low', 15)
        self.declare_parameter('h_high', 45)
        self.declare_parameter('s_low', 40)
        self.declare_parameter('s_high', 255)
        self.declare_parameter('v_low', 100)
        self.declare_parameter('v_high', 255)

        # 2. 흰색 차선 HSV 파라미터 (Fallback 및 회피 주행용)
        self.declare_parameter('w_h_low', 0)
        self.declare_parameter('w_h_high', 180)
        self.declare_parameter('w_s_low', 0)
        self.declare_parameter('w_s_high', 80)
        self.declare_parameter('w_v_low', 120)
        self.declare_parameter('w_v_high', 255)

        if Config.PUBLISH_DEBUG:
            self.debug_pub = self.create_publisher(Image, "/debug/lane_image", 10)
            
        # ======================================================================
        # [GUI 창 및 트랙바 초기화 (노란색 6개 + 흰색 6개)]
        # ======================================================================
        self.gui_available = False
        if Config.USE_GUI and os.environ.get("DISPLAY"):
            try:
                cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Trackbars", 500, 500)
                
                # 노란색 차선 HSV 트랙바
                cv2.createTrackbar("Y_H_MIN", "Trackbars", 15, 180, nothing)
                cv2.createTrackbar("Y_H_MAX", "Trackbars", 45, 180, nothing)
                cv2.createTrackbar("Y_S_MIN", "Trackbars", 40, 255, nothing)
                cv2.createTrackbar("Y_S_MAX", "Trackbars", 255, 255, nothing)
                cv2.createTrackbar("Y_V_MIN", "Trackbars", 100, 255, nothing)
                cv2.createTrackbar("Y_V_MAX", "Trackbars", 255, 255, nothing)
                
                # 흰색 차선 HSV 트랙바
                cv2.createTrackbar("W_H_MIN", "Trackbars", 0, 180, nothing)
                cv2.createTrackbar("W_H_MAX", "Trackbars", 180, 180, nothing)
                cv2.createTrackbar("W_S_MIN", "Trackbars", 0, 255, nothing)
                cv2.createTrackbar("W_S_MAX", "Trackbars", 80, 255, nothing)
                cv2.createTrackbar("W_V_MIN", "Trackbars", 120, 255, nothing)
                cv2.createTrackbar("W_V_MAX", "Trackbars", 255, 255, nothing)
                
                self.gui_available = True
            except Exception as e:
                self.get_logger().warn(f"GUI window init skipped: {e}")
                self.gui_available = False

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()
        self.error_buffer = []  # error 이동평균 버퍼
        
        self.prev_ema_angle = 0.0
        self.prev_ema_speed = 0.0
        
        self.prev_target_x = None
        self.blind_count = 0
        self.target_x_buffer = []  # 궤적 외삽용 최근 target_x 기록
        
        # ======================================================================
        # [회피 주행 상태 머신 변수]
        # STATE_NORMAL: 노란색 차선 정상 주행
        # STATE_AVOID_LEFT: 좌측 장애물 감지 -> 우측 흰색 차선 우회
        # STATE_AVOID_RIGHT: 우측 장애물 감지 -> 좌측 흰색 차선 우회
        # STATE_RETURNING: 장애물 통과 후 노란색 차선 복귀 중
        # ======================================================================
        self.avoid_state = "STATE_NORMAL"
        self.avoid_hold_counter = 0
        self.last_obstacle_dir = None
        self.last_obstacle_dist = float('inf')
        self.detected_boxes = [] # 디버그 렌더링용 최근 바운딩 박스 목록
        
        # YOLOv8 모델 로드
        self.yolo_model = None
        self.frame_count = 0
        if Config.ENABLE_YOLO and YOLO_AVAILABLE:
            try:
                self.get_logger().info(f"Loading YOLOv8 model from {Config.MODEL_PATH} ...")
                self.yolo_model = YOLO(Config.MODEL_PATH)
                self.get_logger().info("YOLOv8 Model Loaded Successfully on GPU!")
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
        else:
            self.get_logger().warn(f"YOLOv8 not loaded. ENABLE_YOLO={Config.ENABLE_YOLO}, YOLO_AVAILABLE={YOLO_AVAILABLE}, Error={YOLO_IMPORT_ERROR}")

        self.current_speed = 0.0
        self.current_angle = 0.0
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Integrated Lane Follow & Obstacle Avoidance Node Started!")

    def scan_callback(self, msg: LaserScan):
        """ LiDAR 스캔 데이터 지속적 버퍼링 """
        self.latest_scan = msg

    def extract_distance_from_bbox(self, x1, x2, img_width):
        """ Bounding Box의 픽셀 좌표를 LiDAR 각도로 매핑하여 최단 거리(m) 추출 """
        if self.latest_scan is None:
            return float('inf')

        scan = self.latest_scan
        ranges = np.array(scan.ranges)
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        
        fov_rad = math.radians(Config.CAMERA_H_FOV_DEG)
        angle_start = fov_rad * (0.5 - float(x2) / img_width)
        angle_end = fov_rad * (0.5 - float(x1) / img_width)

        idx_start = int((angle_start - scan.angle_min) / scan.angle_increment)
        idx_end = int((angle_end - scan.angle_min) / scan.angle_increment)
        
        max_idx = len(ranges) - 1
        idx_start = max(0, min(max_idx, idx_start))
        idx_end = max(0, min(max_idx, idx_end))
        
        if idx_start > idx_end:
            idx_start, idx_end = idx_end, idx_start

        roi_ranges = ranges[idx_start:idx_end+1]
        valid_ranges = roi_ranges[(roi_ranges >= Config.LIDAR_VALID_MIN_DIST) & (roi_ranges <= Config.LIDAR_VALID_MAX_DIST)]
        
        if len(valid_ranges) == 0:
            return float('inf')
            
        # 노이즈 방지를 위해 상위 10% 최솟값(10th percentile) 사용
        return float(np.percentile(valid_ranges, 10))

    def timer_callback(self):
        motor_msg = Float32MultiArray()
        motor_msg.data = [float(self.current_angle), float(self.current_speed)]
        self.motor_pub.publish(motor_msg)

    def image_callback(self, msg: CompressedImage):
        try:
            cv_img = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Image decode error: {e}")
            return
            
        current_time = time.time()
        self.frame_count += 1
        cv_img = cv2.resize(cv_img, (Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT))
        
        # ======================================================================
        # [1. YOLOv8 + LiDAR 센서 퓨전 장애물 감지 (N프레임 주기)]
        # ======================================================================
        min_obstacle_dist = float('inf')
        closest_obstacle_dir = None
        
        if self.yolo_model is not None and (self.frame_count % Config.YOLO_INFER_INTERVAL == 0):
            self.detected_boxes = []
            try:
                # 320x320 경량 고속 추론
                results = self.yolo_model(cv_img, imgsz=320, conf=Config.YOLO_CONF_THRES, device='cuda:0', verbose=False)
                if len(results) > 0 and results[0].boxes is not None:
                    boxes = results[0].boxes
                    for box in boxes:
                        cls_id = int(box.cls[0].item())
                        conf = float(box.conf[0].item())
                        
                        if cls_id in Config.IGNORE_CLASSES:
                            continue
                            
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        dist = self.extract_distance_from_bbox(x1, x2, Config.IMAGE_WIDTH)
                        
                        cx = (x1 + x2) / 2.0
                        obj_dir = 'LEFT' if cx < (Config.IMAGE_WIDTH / 2.0) else 'RIGHT'
                        
                        self.detected_boxes.append((x1, y1, x2, y2, dist, obj_dir, cls_id, conf))
                        
                        if dist < min_obstacle_dist:
                            min_obstacle_dist = dist
                            closest_obstacle_dir = obj_dir
            except Exception as e:
                self.get_logger().error(f"YOLO Inference Error: {e}")
                
            self.last_obstacle_dist = min_obstacle_dist
            self.last_obstacle_dir = closest_obstacle_dir
            
            # 토픽 발행
            if min_obstacle_dist != float('inf'):
                self.obstacle_dist_pub.publish(Float32(data=min_obstacle_dist))
                dir_msg = String()
                dir_msg.data = closest_obstacle_dir if closest_obstacle_dir else "NONE"
                self.obstacle_dir_pub.publish(dir_msg)
        else:
            min_obstacle_dist = self.last_obstacle_dist
            closest_obstacle_dir = self.last_obstacle_dir

        # ======================================================================
        # [2. 차선 인식 전처리 (ROI 및 HSV)]
        # ======================================================================
        roi_top = int(Config.IMAGE_HEIGHT * Config.ROI_TOP_RATIO)
        cv_img_roi = cv_img[roi_top:, :]
        cropped_height = cv_img_roi.shape[0]
        window_height = int(cropped_height // Config.NWINDOWS)
        lookahead_y = int(cropped_height * (1.0 - Config.LOOKAHEAD_RATIO))
        
        # 1. 파라미터 기본값 가져오기 (노란색 & 흰색)
        p_h_l = self.get_parameter('h_low').value
        p_h_h = self.get_parameter('h_high').value
        p_s_l = self.get_parameter('s_low').value
        p_s_h = self.get_parameter('s_high').value
        p_v_l = self.get_parameter('v_low').value
        p_v_h = self.get_parameter('v_high').value

        p_w_h_l = self.get_parameter('w_h_low').value
        p_w_h_h = self.get_parameter('w_h_high').value
        p_w_s_l = self.get_parameter('w_s_low').value
        p_w_s_h = self.get_parameter('w_s_high').value
        p_w_v_l = self.get_parameter('w_v_low').value
        p_w_v_h = self.get_parameter('w_v_high').value

        # 2. 트랙바 값 읽기 (GUI 활성화 시)
        if self.gui_available:
            try:
                # 노란색
                h_l = cv2.getTrackbarPos("Y_H_MIN", "Trackbars")
                h_h = cv2.getTrackbarPos("Y_H_MAX", "Trackbars")
                s_l = cv2.getTrackbarPos("Y_S_MIN", "Trackbars")
                s_h = cv2.getTrackbarPos("Y_S_MAX", "Trackbars")
                v_l = cv2.getTrackbarPos("Y_V_MIN", "Trackbars")
                v_h = cv2.getTrackbarPos("Y_V_MAX", "Trackbars")
                
                if h_l < 0 or h_h <= h_l or v_h <= v_l:
                    h_l, h_h, s_l, s_h, v_l, v_h = p_h_l, p_h_h, p_s_l, p_s_h, p_v_l, p_v_h
                    
                # 흰색
                w_h_l = cv2.getTrackbarPos("W_H_MIN", "Trackbars")
                w_h_h = cv2.getTrackbarPos("W_H_MAX", "Trackbars")
                w_s_l = cv2.getTrackbarPos("W_S_MIN", "Trackbars")
                w_s_h = cv2.getTrackbarPos("W_S_MAX", "Trackbars")
                w_v_l = cv2.getTrackbarPos("W_V_MIN", "Trackbars")
                w_v_h = cv2.getTrackbarPos("W_V_MAX", "Trackbars")
                
                if w_h_l < 0 or w_h_h <= w_h_l or w_v_h <= w_v_l:
                    w_h_l, w_h_h, w_s_l, w_s_h, w_v_l, w_v_h = p_w_h_l, p_w_h_h, p_w_s_l, p_w_s_h, p_w_v_l, p_w_v_h
            except Exception:
                h_l, h_h, s_l, s_h, v_l, v_h = p_h_l, p_h_h, p_s_l, p_s_h, p_v_l, p_v_h
                w_h_l, w_h_h, w_s_l, w_s_h, w_v_l, w_v_h = p_w_h_l, p_w_h_h, p_w_s_l, p_w_s_h, p_w_v_l, p_w_v_h
        else:
            h_l, h_h, s_l, s_h, v_l, v_h = p_h_l, p_h_h, p_s_l, p_s_h, p_v_l, p_v_h
            w_h_l, w_h_h, w_s_l, w_s_h, w_v_l, w_v_h = p_w_h_l, p_w_h_h, p_w_s_l, p_w_s_h, p_w_v_l, p_w_v_h
            
        hsv_img = cv2.cvtColor(cv_img_roi, cv2.COLOR_BGR2HSV)
        kernel = np.ones((5, 5), np.uint8)

        yellow_mask = cv2.inRange(hsv_img, np.array([h_l, s_l, v_l]), np.array([h_h, s_h, v_h]))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        out_img = cv_img.copy()

        # ----------------------------------------------------------------------
        # [2-1] 노란색 차선 슬라이딩 윈도우 & 적응형 피팅
        # ----------------------------------------------------------------------
        nonzero = yellow_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        histogram = np.sum(yellow_mask, axis=0)
        p1 = find_single_peak(histogram, min_height=5 * 255) 
        
        x_current = p1
        lane_inds = []
        active_windows = 0
        
        for window in range(Config.NWINDOWS):
            win_y_low = cropped_height - (window + 1) * window_height
            win_y_high = cropped_height - window * window_height
            
            if x_current is not None:
                win_x_low = max(0, x_current - Config.MARGIN)
                win_x_high = min(Config.IMAGE_WIDTH, x_current + Config.MARGIN)
                
                cv2.rectangle(out_img, (win_x_low, win_y_low + roi_top), (win_x_high, win_y_high + roi_top), (0, 255, 0), 2)
                    
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                lane_inds.append(good_inds)
                
                if len(good_inds) > Config.MINPIX:
                    x_current = int(np.mean(nonzerox[good_inds]))
                    active_windows += 1
        
        lane_inds = np.concatenate(lane_inds) if lane_inds else []
        valid_lane = len(lane_inds) > Config.MINPIX * 2
        lanex, laney = nonzerox[lane_inds], nonzeroy[lane_inds] if valid_lane else ([], [])
            
        yellow_target_x = None
        lane_fit = None
        status_txt = "LANE LOST"

        if valid_lane:
            if len(lane_inds) >= Config.MIN_PIXELS_QUADRATIC and active_windows >= Config.MIN_WINDOWS_QUADRATIC:
                lane_fit = np.polyfit(laney, lanex, 2)
                if abs(lane_fit[0]) > Config.MAX_CURVATURE:
                    fit_1d = np.polyfit(laney, lanex, 1)
                    lane_fit = np.array([0.0, fit_1d[0], fit_1d[1]])
                    status_txt = "YELLOW (LINEAR/CURV)"
                else:
                    status_txt = "YELLOW (CURVE)"
            else:
                fit_1d = np.polyfit(laney, lanex, 1)
                lane_fit = np.array([0.0, fit_1d[0], fit_1d[1]])
                status_txt = "YELLOW (LINEAR)"
                
            yellow_target_x = int(lane_fit[0]*lookahead_y**2 + lane_fit[1]*lookahead_y + lane_fit[2])
            yellow_target_x = max(0, min(Config.IMAGE_WIDTH - 1, yellow_target_x))

        # ----------------------------------------------------------------------
        # [2-2] 흰색 차선 슬라이딩 윈도우 (좌/우 개별 검출)
        # ----------------------------------------------------------------------
        white_mask = cv2.inRange(hsv_img, np.array([w_h_l, w_s_l, w_v_l]), np.array([w_h_h, w_s_h, w_v_h]))
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        w_nonzero = white_mask.nonzero()
        w_nonzeroy = np.array(w_nonzero[0])
        w_nonzerox = np.array(w_nonzero[1])

        w_hist = np.sum(white_mask, axis=0)
        midpoint = len(w_hist) // 2
        left_sub = w_hist[:midpoint]
        right_sub = w_hist[midpoint:]

        l_peak = np.argmax(left_sub) if np.max(left_sub) >= 5 * 255 else None
        r_peak = (midpoint + np.argmax(right_sub)) if np.max(right_sub) >= 5 * 255 else None

        l_inds, r_inds = [], []
        l_curr, r_curr = l_peak, r_peak

        for window in range(Config.NWINDOWS):
            win_y_low = cropped_height - (window + 1) * window_height
            win_y_high = cropped_height - window * window_height

            if l_curr is not None:
                l_x_low = max(0, l_curr - Config.MARGIN)
                l_x_high = min(Config.IMAGE_WIDTH, l_curr + Config.MARGIN)
                cv2.rectangle(out_img, (l_x_low, win_y_low + roi_top), (l_x_high, win_y_high + roi_top), (255, 200, 0), 2)
                good_l = ((w_nonzeroy >= win_y_low) & (w_nonzeroy < win_y_high) & (w_nonzerox >= l_x_low) & (w_nonzerox < l_x_high)).nonzero()[0]
                l_inds.append(good_l)
                if len(good_l) > Config.MINPIX:
                    l_curr = int(np.mean(w_nonzerox[good_l]))

            if r_curr is not None:
                r_x_low = max(0, r_curr - Config.MARGIN)
                r_x_high = min(Config.IMAGE_WIDTH, r_curr + Config.MARGIN)
                cv2.rectangle(out_img, (r_x_low, win_y_low + roi_top), (r_x_high, win_y_high + roi_top), (255, 200, 0), 2)
                good_r = ((w_nonzeroy >= win_y_low) & (w_nonzeroy < win_y_high) & (w_nonzerox >= r_x_low) & (w_nonzerox < r_x_high)).nonzero()[0]
                r_inds.append(good_r)
                if len(good_r) > Config.MINPIX:
                    r_curr = int(np.mean(w_nonzerox[good_r]))

        l_inds = np.concatenate(l_inds) if l_inds else []
        r_inds = np.concatenate(r_inds) if r_inds else []

        l_valid = len(l_inds) > Config.MINPIX * 2
        r_valid = len(r_inds) > Config.MINPIX * 2

        white_left_x, white_right_x = None, None
        if l_valid:
            l_fit = np.polyfit(w_nonzeroy[l_inds], w_nonzerox[l_inds], 1)
            white_left_x = int(l_fit[0] * lookahead_y + l_fit[1])
        if r_valid:
            r_fit = np.polyfit(w_nonzeroy[r_inds], w_nonzerox[r_inds], 1)
            white_right_x = int(r_fit[0] * lookahead_y + r_fit[1])

        # ======================================================================
        # [3. 정적 장애물 회피 상태 머신 (Avoidance State Machine)]
        # ======================================================================
        if min_obstacle_dist <= Config.OBSTACLE_AVOID_DIST:
            if closest_obstacle_dir == 'LEFT':
                self.avoid_state = "STATE_AVOID_LEFT"   # 좌측 장애물 -> 우측 흰색 차선 우회
                self.avoid_hold_counter = Config.AVOID_HOLD_FRAMES
            elif closest_obstacle_dir == 'RIGHT':
                self.avoid_state = "STATE_AVOID_RIGHT"  # 우측 장애물 -> 좌측 흰색 차선 우회
                self.avoid_hold_counter = Config.AVOID_HOLD_FRAMES
        elif self.avoid_state in ["STATE_AVOID_LEFT", "STATE_AVOID_RIGHT"]:
            if self.avoid_hold_counter > 0:
                self.avoid_hold_counter -= 1
            else:
                self.avoid_state = "STATE_RETURNING"
        elif self.avoid_state == "STATE_RETURNING":
            if yellow_target_x is not None:
                self.avoid_state = "STATE_NORMAL"

        # 목표 x좌표 결정
        target_x = None
        
        if self.avoid_state == "STATE_AVOID_LEFT":
            if white_right_x is not None:
                target_x = int(white_right_x - Config.WHITE_LANE_HALF_WIDTH)
            elif yellow_target_x is not None:
                target_x = int(yellow_target_x + Config.WHITE_LANE_HALF_WIDTH // 2)
            status_txt = f"AVOID LEFT ({min_obstacle_dist:.1f}m -> RIGHT LANE)"
            
        elif self.avoid_state == "STATE_AVOID_RIGHT":
            if white_left_x is not None:
                target_x = int(white_left_x + Config.WHITE_LANE_HALF_WIDTH)
            elif yellow_target_x is not None:
                target_x = int(yellow_target_x - Config.WHITE_LANE_HALF_WIDTH // 2)
            status_txt = f"AVOID RIGHT ({min_obstacle_dist:.1f}m -> LEFT LANE)"
            
        elif self.avoid_state == "STATE_RETURNING":
            if yellow_target_x is not None:
                target_x = yellow_target_x
            elif white_left_x is not None and white_right_x is not None:
                target_x = int((white_left_x + white_right_x) // 2)
            status_txt = "RETURNING TO CENTER"
            
        else: # STATE_NORMAL
            if yellow_target_x is not None:
                target_x = yellow_target_x
            else:
                if white_left_x is not None and white_right_x is not None:
                    target_x = int((white_left_x + white_right_x) // 2)
                    status_txt = "WHITE FALLBACK (BOTH)"
                elif white_left_x is not None:
                    target_x = int(white_left_x + Config.WHITE_LANE_HALF_WIDTH)
                    status_txt = "WHITE FALLBACK (LEFT)"
                elif white_right_x is not None:
                    target_x = int(white_right_x - Config.WHITE_LANE_HALF_WIDTH)
                    status_txt = "WHITE FALLBACK (RIGHT)"

        # ======================================================================
        # [4. 궤적 버퍼 기록 및 궤적 외삽(점선 끊김 대응)]
        # ======================================================================
        if target_x is not None:
            target_x = max(0, min(Config.IMAGE_WIDTH - 1, target_x))
            self.prev_target_x = target_x
            self.blind_count = 0
            self.target_x_buffer.append(target_x)
            if len(self.target_x_buffer) > Config.TARGET_X_BUFFER_SIZE:
                self.target_x_buffer.pop(0)
        else:
            if len(self.target_x_buffer) >= 3 and self.blind_count < Config.GHOST_MAX_FRAMES:
                recent = self.target_x_buffer[-min(5, len(self.target_x_buffer)):]
                dx_per_frame = (recent[-1] - recent[0]) / max(len(recent) - 1, 1)
                predicted_x = int(self.target_x_buffer[-1] + dx_per_frame * (self.blind_count + 1))
                predicted_x = max(0, min(Config.IMAGE_WIDTH - 1, predicted_x))
                target_x = predicted_x
                self.blind_count += 1
                status_txt = f"PREDICT ({self.blind_count})"
            elif self.prev_target_x is not None and self.blind_count < Config.GHOST_MAX_FRAMES:
                target_x = self.prev_target_x
                self.blind_count += 1
                status_txt = f"GHOST ({self.blind_count})"
            else:
                target_x = None
                status_txt = "LANE LOST"
            
        # ======================================================================
        # [5. PID 제어 및 속도 계산]
        # ======================================================================
        if target_x is not None:
            raw_error = target_x - (Config.IMAGE_WIDTH // 2)
        else:
            raw_error = self.prev_error

        self.error_buffer.append(raw_error)
        if len(self.error_buffer) > Config.ERROR_BUFFER_SIZE:
            self.error_buffer.pop(0)
        error = sum(self.error_buffer) / len(self.error_buffer)

        dt = max(current_time - self.last_time, 0.001)
        
        self.integral_error += error * dt
        self.integral_error = max(-500.0, min(500.0, self.integral_error)) 
        derivative_error = (error - self.prev_error) / dt
        
        raw_angle = (Config.Kp * error) + (Config.Ki * self.integral_error) + (Config.Kd * derivative_error)
        filtered_angle = (Config.EMA_ALPHA_ANGLE * raw_angle) + ((1.0 - Config.EMA_ALPHA_ANGLE) * self.prev_ema_angle)
        
        self.prev_error = error
        self.prev_ema_angle = filtered_angle
        self.last_time = current_time
        
        angle_clamped = float(max(-50.0, min(50.0, filtered_angle)))
        
        base_speed = Config.AVOID_SPEED if "AVOID" in self.avoid_state else Config.BASE_SPEED
        target_speed = float(max(Config.MIN_SPEED, min(base_speed, base_speed - abs(angle_clamped) * Config.SPEED_DECAY)))
        
        if status_txt == "LANE LOST": 
            target_speed = 0.0 
            angle_clamped = 0.0
            
        filtered_speed = (Config.EMA_ALPHA_SPEED * target_speed) + ((1.0 - Config.EMA_ALPHA_SPEED) * self.prev_ema_speed)
        self.prev_ema_speed = filtered_speed
        
        self.current_angle = angle_clamped
        self.current_speed = filtered_speed

        self.get_logger().info(f"State: {status_txt}, StateMachine: {self.avoid_state}, Steer: {angle_clamped:.1f}, Speed: {filtered_speed:.1f}", throttle_duration_sec=0.5)
        
        # ======================================================================
        # [6. 디버그 시각화 렌더링]
        # ======================================================================
        ploty = np.linspace(0, cropped_height - 1, cropped_height)
        if lane_fit is not None:
            pts = np.array([np.transpose(np.vstack([lane_fit[0]*ploty**2 + lane_fit[1]*ploty + lane_fit[2], ploty + roi_top]))], np.int32)
            cv2.polylines(out_img, pts, False, (0, 255, 255), 3)
            
        if target_x is not None:
            cv2.circle(out_img, (target_x, lookahead_y + roi_top), 8, (255, 0, 255), -1)
            cv2.line(out_img, (Config.IMAGE_WIDTH // 2, lookahead_y + roi_top), (target_x, lookahead_y + roi_top), (255, 255, 255), 2)
        
        cv2.line(out_img, (Config.IMAGE_WIDTH // 2, 0), (Config.IMAGE_WIDTH // 2, Config.IMAGE_HEIGHT), (100, 100, 100), 1)
        
        # 탐지된 장애물 바운딩 박스 및 LiDAR 거리 표시
        for (bx1, by1, bx2, by2, bdist, bdir, bcls, bconf) in self.detected_boxes:
            bcolor = (0, 0, 255) if bdist <= Config.OBSTACLE_AVOID_DIST else (0, 255, 255)
            cv2.rectangle(out_img, (bx1, by1), (bx2, by2), bcolor, 2)
            label_txt = f"{bdir} {bdist:.2f}m (ID:{bcls})" if bdist != float('inf') else f"{bdir} (No LiDAR)"
            cv2.putText(out_img, label_txt, (bx1, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, bcolor, 2)
        
        # 상태 표시
        state_color = (0, 0, 255) if "AVOID" in self.avoid_state else ((0, 255, 255) if "YELLOW" in status_txt else (255, 200, 0))
        cv2.putText(out_img, f"Mode: {status_txt}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, state_color, 2)
        cv2.putText(out_img, f"Avoid State: {self.avoid_state}", (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        cv2.putText(out_img, f"Angle: {angle_clamped:.1f} | Speed: {filtered_speed:.1f}", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        
        if Config.PUBLISH_DEBUG:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))
        
        if self.gui_available:
            try:
                # Trackbars 창에 빈 캔버스를 그려 X11 위젯 렌더링 유지
                trackbar_bg = np.zeros((20, 500, 3), dtype=np.uint8)
                cv2.imshow("Trackbars", trackbar_bg)
                
                cv2.imshow("Yellow_Mask (HSV_Cropped)", yellow_mask)
                cv2.imshow("White_Mask (HSV_Cropped)", white_mask)
                cv2.imshow("Lane_Control (Cropped)", out_img)
                cv2.waitKey(1)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = TrackDriveNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        if rclpy.ok():
            try:
                stop_msg = Float32MultiArray()
                stop_msg.data = [0.0, 0.0]
                node.motor_pub.publish(stop_msg) 
            except Exception:
                pass
        
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()
            
        try:
            cv2.destroyAllWindows()
        except:
            pass

if __name__ == "__main__":
    main()

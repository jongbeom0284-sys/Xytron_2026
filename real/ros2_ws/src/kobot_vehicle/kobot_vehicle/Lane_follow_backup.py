#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS2 압축 이미지 기반 노란색 차선(단일) 모노레일 추종 Node
- 노란색 점선(Center line) 1개만을 추종
- 슬라이딩 윈도우 시 점선 빈 공간은 이전 x값 유지
- 속도 EMA 필터 추가로 울컥거림 현상 방지
"""

import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def nothing(x):
    pass

def find_single_peak(hist, min_height=10):
    """ 히스토그램에서 가장 높은 단 1개의 피크를 반환합니다. """
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
    MIN_PIXELS_QUADRATIC = 80   # 2차 피팅에 필요한 최소 픽셀 수
    MIN_WINDOWS_QUADRATIC = 3   # 2차 피팅에 필요한 최소 활성 윈도우 수
    MAX_CURVATURE = 0.005       # 2차 계수(a) 절대값 상한 (초과 시 직선으로 되돌림)
    
    # 궤적 외삽: 점선 끊김 구간에서 최근 추세 기반 위치 예측
    TARGET_X_BUFFER_SIZE = 10   # 최근 target_x 기록 버퍼 크기
    GHOST_MAX_FRAMES = 20       # 점선 끊김 시 최대 예측 프레임 수
    
    BASE_SPEED = 30.0
    MIN_SPEED = 15.0
    SPEED_DECAY = 0.6

class TrackDriveNode(Node):
    def __init__(self):
        super().__init__("track_drive_node")
        self.bridge = CvBridge()
        
        # 콜백 그룹 분리 (카메라 처리 지연이 모터 타이머를 막지 않도록)
        self.cam_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        
        # 최신 프레임 1개만 유지 (지연 누적에 의한 제어 발산 방지)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.camera_sub = self.create_subscription(CompressedImage, "/image_raw/compressed", self.image_callback, qos, callback_group=self.cam_cb_group)
        self.motor_pub = self.create_publisher(Float32MultiArray, "/xycar_motor", 10)
        
        # 노란색 HSV 기본 프리셋 (실측 튜닝값)
        self.declare_parameter('h_low', 25)
        self.declare_parameter('h_high', 40)
        self.declare_parameter('s_low', 61)
        self.declare_parameter('s_high', 120)
        self.declare_parameter('v_low', 190)
        self.declare_parameter('v_high', 255)

        if Config.PUBLISH_DEBUG:
            self.debug_pub = self.create_publisher(Image, "/debug/lane_image", 10)
            
        if Config.USE_GUI:
            cv2.namedWindow("Trackbars", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Trackbars", 400, 300)
            cv2.createTrackbar("H_MIN", "Trackbars", 25, 180, nothing)
            cv2.createTrackbar("H_MAX", "Trackbars", 40, 180, nothing)
            cv2.createTrackbar("S_MIN", "Trackbars", 61, 255, nothing)
            cv2.createTrackbar("S_MAX", "Trackbars", 120, 255, nothing)
            cv2.createTrackbar("V_MIN", "Trackbars", 190, 255, nothing)
            cv2.createTrackbar("V_MAX", "Trackbars", 255, 255, nothing)

        self.prev_error = 0.0
        self.integral_error = 0.0
        self.last_time = time.time()
        self.error_buffer = []  # error 이동평균 버퍼
        
        self.prev_ema_angle = 0.0
        self.prev_ema_speed = 0.0
        
        self.prev_target_x = None
        self.blind_count = 0
        self.target_x_buffer = []  # 궤적 외삽용 최근 target_x 기록
        
        self.current_speed = 0.0
        self.current_angle = 0.0
        self.timer = self.create_timer(0.05, self.timer_callback, callback_group=self.timer_cb_group)
        
        self.get_logger().info("Lane Follow Node Started (Yellow Single Line, Speed Filtered)!")

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
        cv_img = cv2.resize(cv_img, (Config.IMAGE_WIDTH, Config.IMAGE_HEIGHT))
        
        roi_top = int(Config.IMAGE_HEIGHT * Config.ROI_TOP_RATIO)
        cv_img = cv_img[roi_top:, :]
        
        cropped_height = cv_img.shape[0]
        
        if Config.USE_GUI:
            h_l = cv2.getTrackbarPos("H_MIN", "Trackbars")
            h_h = cv2.getTrackbarPos("H_MAX", "Trackbars")
            s_l = cv2.getTrackbarPos("S_MIN", "Trackbars")
            s_h = cv2.getTrackbarPos("S_MAX", "Trackbars")
            v_l = cv2.getTrackbarPos("V_MIN", "Trackbars")
            v_h = cv2.getTrackbarPos("V_MAX", "Trackbars")
        else:
            h_l = self.get_parameter('h_low').value
            h_h = self.get_parameter('h_high').value
            s_l = self.get_parameter('s_low').value
            s_h = self.get_parameter('s_high').value
            v_l = self.get_parameter('v_low').value
            v_h = self.get_parameter('v_high').value
            
        hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv_img, np.array([h_l, s_l, v_l]), np.array([h_h, s_h, v_h]))

        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        nonzero = yellow_mask.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        out_img = None
        if Config.PUBLISH_DEBUG or Config.USE_GUI:
            out_img = cv_img.copy()

        histogram = np.sum(yellow_mask, axis=0)
        p1 = find_single_peak(histogram, min_height=10 * 255) 
        
        x_current = p1
        window_height = int(cropped_height // Config.NWINDOWS)
        lane_inds = []
        active_windows = 0
        
        for window in range(Config.NWINDOWS):
            win_y_low = cropped_height - (window + 1) * window_height
            win_y_high = cropped_height - window * window_height
            
            if x_current is not None:
                win_x_low = max(0, x_current - Config.MARGIN)
                win_x_high = min(Config.IMAGE_WIDTH, x_current + Config.MARGIN)
                
                if Config.PUBLISH_DEBUG or Config.USE_GUI:
                    cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)
                    
                good_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                lane_inds.append(good_inds)
                
                # 점선의 경우 픽셀이 없으면 이전 윈도우의 x_current를 그대로 유지하여 연결!
                if len(good_inds) > Config.MINPIX:
                    x_current = int(np.mean(nonzerox[good_inds]))
                    active_windows += 1
        
        lane_inds = np.concatenate(lane_inds) if lane_inds else []
        valid_lane = len(lane_inds) > Config.MINPIX * 2
        
        lanex, laney = nonzerox[lane_inds], nonzeroy[lane_inds] if valid_lane else ([], [])
            
        target_x = None
        status_txt = "LANE LOST"
        lookahead_y = int(cropped_height * (1.0 - Config.LOOKAHEAD_RATIO))

        # ── 적응형 피팅: 데이터 양에 따라 2차(곡선) 또는 1차(직선) 자동 선택 ──
        lane_fit = None
        if valid_lane:
            if len(lane_inds) >= Config.MIN_PIXELS_QUADRATIC and active_windows >= Config.MIN_WINDOWS_QUADRATIC:
                lane_fit = np.polyfit(laney, lanex, 2)
                # 곡률 안전장치: 물리적으로 불가능한 급커브 감지 시 직선으로 되돌림
                if abs(lane_fit[0]) > Config.MAX_CURVATURE:
                    fit_1d = np.polyfit(laney, lanex, 1)
                    lane_fit = np.array([0.0, fit_1d[0], fit_1d[1]])
                    status_txt = "TRACKING (LINEAR/CURV)"
                else:
                    status_txt = "TRACKING (CURVE)"
            else:
                # 데이터 부족: 직선 피팅 (과적합 방지)
                fit_1d = np.polyfit(laney, lanex, 1)
                lane_fit = np.array([0.0, fit_1d[0], fit_1d[1]])
                status_txt = "TRACKING (LINEAR)"
        
        if valid_lane and lane_fit is not None:
            target_x = int(lane_fit[0]*lookahead_y**2 + lane_fit[1]*lookahead_y + lane_fit[2])
            target_x = max(0, min(Config.IMAGE_WIDTH - 1, target_x))
            self.prev_target_x = target_x
            self.blind_count = 0
            # 궤적 버퍼에 기록 (점선 끊김 시 외삽에 사용)
            self.target_x_buffer.append(target_x)
            if len(self.target_x_buffer) > Config.TARGET_X_BUFFER_SIZE:
                self.target_x_buffer.pop(0)
        else:
            # ── 궤적 외삽: 점선 끊김 시 최근 추세를 연장하여 다음 위치 예측 ──
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
            
        if target_x is not None:
            # 렌즈 정중앙(IMAGE_WIDTH//2)과 타겟과의 거리를 바로 Error로 사용 (모노레일)
            raw_error = target_x - (Config.IMAGE_WIDTH // 2)
        else:
            raw_error = self.prev_error

        # 이동평균 필터: 최근 N프레임의 error를 평균내어 노이즈(점선 깜빡임 등) 제거
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
        
        # 조향각 제한 -50~50 (VESC 에러 방지)
        angle_clamped = float(max(-50.0, min(50.0, filtered_angle)))
        
        # 조향각에 따른 이상적인 목표 속도 계산
        target_speed = float(max(Config.MIN_SPEED, min(Config.BASE_SPEED, Config.BASE_SPEED - abs(angle_clamped) * Config.SPEED_DECAY)))
        
        if status_txt == "LANE LOST": 
            target_speed = 0.0 
            angle_clamped = 0.0
            
        # 속도에도 EMA 필터를 적용하여 급가감속 방지 (울컥거림 해결)
        filtered_speed = (Config.EMA_ALPHA_SPEED * target_speed) + ((1.0 - Config.EMA_ALPHA_SPEED) * self.prev_ema_speed)
        self.prev_ema_speed = filtered_speed
        
        self.current_angle = angle_clamped
        self.current_speed = filtered_speed

        self.get_logger().info(f"State: {status_txt}, TargetX: {target_x}, Steer: {angle_clamped:.1f}, Speed: {filtered_speed:.1f}", throttle_duration_sec=2.0)
        
        if out_img is not None:
            ploty = np.linspace(0, cropped_height - 1, cropped_height)
            if lane_fit is not None:
                pts = np.array([np.transpose(np.vstack([lane_fit[0]*ploty**2 + lane_fit[1]*ploty + lane_fit[2], ploty]))], np.int32)
                cv2.polylines(out_img, pts, False, (0, 255, 255), 3)
                
            if target_x is not None:
                cv2.circle(out_img, (target_x, lookahead_y), 8, (255, 0, 255), -1)
                cv2.line(out_img, (Config.IMAGE_WIDTH // 2, lookahead_y), (target_x, lookahead_y), (255, 255, 255), 2)
            
            cv2.line(out_img, (Config.IMAGE_WIDTH // 2, 0), (Config.IMAGE_WIDTH // 2, cropped_height), (100, 100, 100), 1)
            
            cv2.putText(out_img, f"State: {status_txt}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(out_img, f"Angle: {angle_clamped:.1f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(out_img, f"Speed: {filtered_speed:.1f}", (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            if Config.PUBLISH_DEBUG:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(out_img, encoding="bgr8"))
            
            if Config.USE_GUI:
                cv2.imshow("Yellow_Mask (HSV_Cropped)", yellow_mask)
                cv2.imshow("Lane_Control (Cropped)", out_img)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TrackDriveNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try: 
        executor.spin()
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

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32MultiArray
import math
import numpy as np
import cv2
from cv_bridge import CvBridge

class LidarDriveNode(Node):
    def __init__(self):
        super().__init__('lidar_drive')
        
        # 제어 명령 퍼블리셔 (Float32MultiArray 사용)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'xycar_motor', 1)
        
        # 디버깅 뷰 퍼블리셔 (rqt_image_view 용)
        self.debug_pub = self.create_publisher(Image, '/debug/lidar_view', 1)
        self.bridge = CvBridge()
        
        # 라이다 구독자
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # --- 튜닝 파라미터 ---
        self.OBSTACLE_DIST = 1.0       # 1.0m 이내를 장애물(라바콘)로 인식
        self.FOV_DEG = 80.0            # 전방 +- 80도 시야각 사용 (총 160도)
        self.BASE_SPEED = 12.0         # 기본 주행 속도 (필요시 조정 가능)
        self.MAX_STEER_ANGLE = 50.0    # 자이카 최대 조향각 한계
        
        # 임계각 및 가상 트랙 폭 (코너링/벽 판단용)
        self.CRITICAL_ANGLE = 30.0     # 이 각도보다 작으면(정면) 벽으로 간주, 크면 코너로 간주
        self.TRACK_WIDTH_DEG = 70.0    # 한쪽 라바콘만 보일 때 반대편 라바콘까지의 가상 각도 너비
        
        # PD 제어 게인 (PID 중 P와 D)
        self.Kp = 0.8  # 비례 제어 (목표를 향해 꺾는 힘, 반응성)
        self.Kd = 0.5  # 미분 제어 (오차 변화량을 바탕으로 요동/진동 억제)
        self.prev_error = 0.0
        
        # 디버깅 GUI 활성화 스위치 (연산 속도 확보를 위해 대회 시 False 권장)
        self.ENABLE_DEBUG_GUI = True
        
        self.get_logger().info("--- Lidar PD Gap Follower Node with GUI Started ---")
        
    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        # 무한대나 유효하지 않은 값들은 안전한 먼 거리(10m)로 치환
        ranges[np.isnan(ranges)] = 10.0
        ranges[np.isinf(ranges)] = 10.0
        
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        left_closest_dist = float('inf')
        left_cone_angle = 0.0
        
        right_closest_dist = float('inf')
        right_cone_angle = 0.0
        
        # 1. 시야각 내 데이터 필터링 및 좌/우 가장 가까운 라바콘 탐색
        for i, dist in enumerate(ranges):
            # 라이다 각도 계산 (원본 코드 기준 +math.pi 보정)
            angle_rad = angle_min + (i * angle_inc) + math.pi
            
            while angle_rad > math.pi: angle_rad -= 2 * math.pi
            while angle_rad < -math.pi: angle_rad += 2 * math.pi
            
            angle_deg = math.degrees(angle_rad)
            
            # 설정한 시야각(FOV) 밖의 데이터는 무시
            if abs(angle_deg) > self.FOV_DEG:
                continue
                
            # 장애물(라바콘) 거리 이내인 경우만 가장 가까운 점 추출
            if dist < self.OBSTACLE_DIST:
                if angle_deg >= 0: # 왼쪽 영역 (0 ~ 80도)
                    if dist < left_closest_dist:
                        left_closest_dist = dist
                        left_cone_angle = angle_deg
                else: # 오른쪽 영역 (0 ~ -80도)
                    if dist < right_closest_dist:
                        right_closest_dist = dist
                        right_cone_angle = angle_deg

        has_left_cone = left_closest_dist != float('inf')
        has_right_cone = right_closest_dist != float('inf')
        
        target_angle_deg = 0.0
        
        # 2. 라바콘 감지 상태에 따른 분기 (Target Angle 계산)
        if has_left_cone and has_right_cone:
            # 양쪽에 모두 라바콘이 있음 -> 사잇각의 중심을 목표로 함
            target_angle_deg = (left_cone_angle + right_cone_angle) / 2.0
            
        elif has_left_cone and not has_right_cone:
            # 왼쪽에만 라바콘이 보임
            if left_cone_angle < self.CRITICAL_ANGLE:
                # 정면 근처에 위치 = 벽으로 간주 (오른쪽으로 긴급 회피)
                target_angle_deg = -self.MAX_STEER_ANGLE
            else:
                # 측면에 위치 = 코너링 중 (오른쪽에 가상의 라바콘이 있다고 가정)
                target_angle_deg = left_cone_angle - (self.TRACK_WIDTH_DEG / 2.0)
                
        elif has_right_cone and not has_left_cone:
            # 오른쪽에만 라바콘이 보임
            if abs(right_cone_angle) < self.CRITICAL_ANGLE:
                # 정면 근처에 위치 = 벽으로 간주 (왼쪽으로 긴급 회피)
                target_angle_deg = self.MAX_STEER_ANGLE
            else:
                # 측면에 위치 = 코너링 중 (왼쪽에 가상의 라바콘이 있다고 가정)
                target_angle_deg = right_cone_angle + (self.TRACK_WIDTH_DEG / 2.0)
                
        else:
            # 아무것도 발견되지 않음 -> 직진
            target_angle_deg = 0.0
            
        # 3. PD 제어기 (PD Controller)
        error = target_angle_deg # 현재 차체 각도(0도) 기준이므로 목표 각도 자체가 Error 가 됨
        
        p_term = self.Kp * error
        d_term = self.Kd * (error - self.prev_error)
        
        steer_output = p_term + d_term
        self.prev_error = error
        
        # 최대 조향각 한계 안으로 값 제한 (클리핑)
        steer_output = max(-self.MAX_STEER_ANGLE, min(self.MAX_STEER_ANGLE, steer_output))
        
        # 4. 제어값 퍼블리시
        speed = self.BASE_SPEED
        # 조향각이 클 경우(코너링 시) 속도를 살짝 줄여 안정성 확보
        if abs(steer_output) > 20.0:
            speed = self.BASE_SPEED * 0.7
            
        motor_msg = Float32MultiArray()
        # 주의: [angle, speed] 순서로 전송 (track_drive.py 참고)
        motor_msg.data = [float(steer_output), float(speed)]
        self.motor_pub.publish(motor_msg)
        
        # ----------------------------------------------------
        # 5. 디버깅 GUI 시각화 (rqt_image_view 용)
        # ----------------------------------------------------
        if not self.ENABLE_DEBUG_GUI:
            return
            
        debug_img = np.zeros((500, 500, 3), dtype=np.uint8)
        center_x, center_y = 250, 450
        scale = 100 # 1m = 100픽셀로 매핑
        
        # (1) 기본 그리드 (1m, 2m 반경 원)
        cv2.circle(debug_img, (center_x, center_y), scale, (50, 50, 50), 1)
        cv2.circle(debug_img, (center_x, center_y), scale * 2, (50, 50, 50), 1)
        
        # (2) 좌/우 시야각(FOV) 한계선 그리기 (어두운 회색)
        left_limit_x = int(center_x - (math.sin(math.radians(self.FOV_DEG)) * 500))
        left_limit_y = int(center_y - (math.cos(math.radians(self.FOV_DEG)) * 500))
        cv2.line(debug_img, (center_x, center_y), (left_limit_x, left_limit_y), (40, 40, 40), 1)
        
        right_limit_x = int(center_x - (math.sin(math.radians(-self.FOV_DEG)) * 500))
        right_limit_y = int(center_y - (math.cos(math.radians(-self.FOV_DEG)) * 500))
        cv2.line(debug_img, (center_x, center_y), (right_limit_x, right_limit_y), (40, 40, 40), 1)
        
        # (3) 모든 유효 라이다 점 그리기
        for i, dist in enumerate(ranges):
            angle_rad = angle_min + (i * angle_inc) + math.pi
            while angle_rad > math.pi: angle_rad -= 2 * math.pi
            while angle_rad < -math.pi: angle_rad += 2 * math.pi
            angle_deg = math.degrees(angle_rad)
            
            if dist < 5.0: # 5m 이내 점만 시각화
                px = int(center_x - (math.sin(angle_rad) * dist * scale))
                py = int(center_y - (math.cos(angle_rad) * dist * scale))
                if 0 <= px < 500 and 0 <= py < 500:
                    # FOV 및 장애물 감지 구역 내에 있으면 하얀색, 아니면 어두운 회색
                    if dist < self.OBSTACLE_DIST and abs(angle_deg) <= self.FOV_DEG:
                        cv2.circle(debug_img, (px, py), 2, (255, 255, 255), -1)
                    else:
                        cv2.circle(debug_img, (px, py), 1, (100, 100, 100), -1)
                        
        # (4) 감지된 Left/Right 가장 가까운 라바콘 강조 (Cluster 대표점)
        if has_left_cone:
            lx = int(center_x - (math.sin(math.radians(left_cone_angle)) * left_closest_dist * scale))
            ly = int(center_y - (math.cos(math.radians(left_cone_angle)) * left_closest_dist * scale))
            cv2.circle(debug_img, (lx, ly), 8, (0, 255, 255), -1) # Yellow
            cv2.putText(debug_img, "L", (lx+10, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            
        if has_right_cone:
            rx = int(center_x - (math.sin(math.radians(right_cone_angle)) * right_closest_dist * scale))
            ry = int(center_y - (math.cos(math.radians(right_cone_angle)) * right_closest_dist * scale))
            cv2.circle(debug_img, (rx, ry), 8, (255, 0, 255), -1) # Magenta
            cv2.putText(debug_img, "R", (rx+10, ry), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
            
        # (5) 목표 조향각 (Look-Ahead Point) 그리기 -> 얇은 녹색 선
        target_x = int(center_x - (math.sin(math.radians(target_angle_deg)) * 200))
        target_y = int(center_y - (math.cos(math.radians(target_angle_deg)) * 200))
        cv2.line(debug_img, (center_x, center_y), (target_x, target_y), (0, 255, 0), 2)
        
        # (6) 최종 출력 조향각 (Steer Output - PD Control 적용) -> 굵은 빨간 화살표
        steer_x = int(center_x - (math.sin(math.radians(steer_output)) * 200))
        steer_y = int(center_y - (math.cos(math.radians(steer_output)) * 200))
        cv2.arrowedLine(debug_img, (center_x, center_y), (steer_x, steer_y), (0, 0, 255), 4, tipLength=0.1)
        
        # (7) 텍스트 정보 출력 (각도 및 방향)
        dir_text = "LEFT" if steer_output > 2.0 else "RIGHT" if steer_output < -2.0 else "STRAIGHT"
        cv2.putText(debug_img, f"Look-Ahead: {target_angle_deg:.1f} deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(debug_img, f"Steer Cmd: {steer_output:.1f} deg [{dir_text}]", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(debug_img, f"Speed: {speed:.1f}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # ROS 이미지 메시지로 변환 후 퍼블리시
        try:
            img_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            self.debug_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    node = LidarDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Bool, Float32MultiArray

class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        
        # ==========================================
        # 🔴 [튜닝 파라미터]
        # ==========================================
        self.OBSTACLE_DIST = 0.8        # 감지 거리
        self.MIN_DIST = 0.15            # 본체 필터링용 최소 거리 (15cm)
        self.AVOID_SPEED = 3.0          # [수정] 모터 구동 속도 (기존 0.3 -> 3.0)
        
        self.FOV_DEG = 120.0
        self.FRONT_ANGLE_DEG = 20.0
        
        self.WALL_OFFSET_DEG = 40.0     
        self.STEER_GAIN = 1.5           
        self.MAX_STEER_RAD = 0.70       
        self.STEER_ALPHA = 0.5          
        # ==========================================
        
        self.prev_steer = 0.0

        self.lidar_sub = self.create_subscription(LaserScan, "/scan", self.lidar_callback, qos_profile_sensor_data)
        
        # [수정] Publisher 토픽 및 메시지 타입 변경
        self.drive_pub = self.create_publisher(Float32MultiArray, "/xycar_motor", 1)
        self.safety_pub = self.create_publisher(Bool, "/safety/obstacle_detected", 1)
        self.debug_pub = self.create_publisher(Image, "/debug/lidar_view", 1)
        self.bridge = CvBridge()

        self.get_logger().info(">>> LiDAR Avoidance: Frontal Collision Prevention Mode <<<")

    def lidar_callback(self, msg):
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = 10.0
        
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        debug_img = np.zeros((500, 500, 3), dtype=np.uint8)
        center_x, center_y = 250, 450
        scale = 80 
        
        front_hits = 0
        left_hits = 0
        right_hits = 0
        
        sum_left_dist = 0.0
        sum_right_dist = 0.0
        cnt_left = 0
        cnt_right = 0
        
        obstacle_detected = False
        
        for i, dist in enumerate(ranges):
            angle_rad = angle_min + (i * angle_inc) + math.pi
            while angle_rad > math.pi: angle_rad -= 2 * math.pi
            while angle_rad < -math.pi: angle_rad += 2 * math.pi
            angle_deg = math.degrees(angle_rad)
            
            # 1. 시야각 제한 (-120 ~ 120도)
            if abs(angle_deg) > self.FOV_DEG:
                continue
                
            if dist < 4.0:
                px = int(center_x - (dist * math.sin(angle_rad) * scale))
                py = int(center_y - (dist * math.cos(angle_rad) * scale))
                if 0 <= px < 500 and 0 <= py < 500:
                    cv2.circle(debug_img, (px, py), 1, (50, 50, 50), -1)
            
            if 0 < angle_deg <= 90:
                sum_left_dist += min(dist, 3.0)
                cnt_left += 1
            elif -90 <= angle_deg < 0:
                sum_right_dist += min(dist, 3.0)
                cnt_right += 1

            # 2. 장애물 카운트 
            if self.MIN_DIST < dist < self.OBSTACLE_DIST:
                obstacle_detected = True
                
                if abs(angle_deg) <= self.FRONT_ANGLE_DEG:
                    front_hits += 1
                    color = (0, 0, 255) 
                elif angle_deg > self.FRONT_ANGLE_DEG:
                    left_hits += 1
                    color = (0, 255, 255) 
                else:
                    right_hits += 1
                    color = (255, 0, 255) 
                
                if dist < 4.0:
                    px = int(center_x - (dist * math.sin(angle_rad) * scale))
                    py = int(center_y - (dist * math.cos(angle_rad) * scale))
                    if 0 <= px < 500 and 0 <= py < 500:
                        cv2.circle(debug_img, (px, py), 3, color, -1)

        if not obstacle_detected:
            safety_msg = Bool()
            safety_msg.data = False
            self.safety_pub.publish(safety_msg)
            
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
            except: pass
            
            self.prev_steer *= 0.9
            # 장애물이 없을 때 직진 명령 하달 (속도 3.0)
            self.publish_drive(self.prev_steer, self.AVOID_SPEED)
            return

        safety_msg = Bool()
        safety_msg.data = True
        self.safety_pub.publish(safety_msg)
        
        target_deg = 0.0
        mode_str = ""

        # 막힘 판단
        if front_hits > 5: 
            self.get_logger().warn("FRONT BLOCKED! TURNING TO OPEN SPACE", throttle_duration_sec=0.5)
            mode_str = "FRONT BLOCKED"
            
            avg_left = sum_left_dist / cnt_left if cnt_left > 0 else 0
            avg_right = sum_right_dist / cnt_right if cnt_right > 0 else 0
            
            if avg_left > avg_right:
                target_deg = 60.0
            else:
                target_deg = -60.0
        
        elif left_hits > 0 and right_hits > 0:
            mode_str = "CENTER KEEP"
            target_deg = 0.0
            
        elif left_hits > 0:
            mode_str = "FOLLOW LEFT"
            target_deg = -self.WALL_OFFSET_DEG
            
        elif right_hits > 0:
            mode_str = "FOLLOW RIGHT"
            target_deg = self.WALL_OFFSET_DEG
            
        else:
            mode_str = "CLEAR"
            target_deg = 0.0

        raw_steer = math.radians(target_deg) * self.STEER_GAIN
        raw_steer = max(min(raw_steer, self.MAX_STEER_RAD), -self.MAX_STEER_RAD)
        
        final_steer = (self.STEER_ALPHA * raw_steer) + ((1.0 - self.STEER_ALPHA) * self.prev_steer)
        self.prev_steer = final_steer
        
        self.publish_drive(final_steer, self.AVOID_SPEED)

        target_rad = math.radians(target_deg)
        line_len = 150
        end_px = int(center_x - (math.sin(target_rad) * line_len))
        end_py = int(center_y - (math.cos(target_rad) * line_len))
        
        cv2.line(debug_img, (center_x, center_y), (end_px, end_py), (0, 255, 255), 3)
        cv2.putText(debug_img, f"Mode: {mode_str}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(debug_img, f"Front: {front_hits}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        try:
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_img, "bgr8"))
        except: pass

    # [수정] Float32MultiArray 타입으로 메시지 발행
    def publish_drive(self, steering_angle, speed):
        drive_msg = Float32MultiArray()
        # Float32MultiArray의 data = [angle, speed]
        # 보통 각도(degree)를 사용하므로 라디안에서 도(degree)로 변환합니다.
        angle_deg = math.degrees(steering_angle)
        drive_msg.data = [float(angle_deg), float(speed)]
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
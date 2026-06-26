#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import time
from typing import Any

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import String
from xycar_msgs.msg import XycarMotor

from .cone_follower import ConeCommand, ConeFollower, ConeFollowerConfig
from .lane_controller import LaneController
from .lane_detector import LaneDetector, LaneResult
from .lidar_obstacle import ObstacleDetector, ObstacleResult
from .object_detector import HeadlightDetector, HeadlightResult, ObjectResult, VehicleObjectDetector
from .stopline_detector import StopLineDetector
from .traffic_light_detector import TrafficLightDetector

try:
    from detection_msgs.msg import BoundingBoxes  # type: ignore
except Exception:
    BoundingBoxes = None


class TrackDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("driver")
        self.bridge = CvBridge()
        self.motor_msg = XycarMotor()

        self.image = None
        self.headlight_images: dict[str, np.ndarray] = {}
        self.headlight_image_times: dict[str, float] = {}
        self.last_image_time = 0.0
        self.lidar_ranges = None
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        self.last_scan_time = 0.0
        self.yaw_rate = 0.0
        self.current_yaw_rad: float | None = None
        self.current_yaw_deg: float | None = None

        self.declare_parameter("image_topic", "/usb_cam/image_raw/front")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("motor_topic", "xycar_motor")
        self.declare_parameter("debug_topic", "/track_drive/debug")
        self.declare_parameter("yolo_topic", "/yolov5/detections")
        self.declare_parameter("control_hz", 10.0)
        self.declare_parameter("publish_debug_images", True)
        self.declare_parameter("enable_start_signal_gate", False)
        self.declare_parameter("start_signal_release_latched", True)
        self.declare_parameter("enable_stopline", False)
        self.declare_parameter("enable_lidar_obstacle", True)
        self.declare_parameter("enable_object_detector", True)
        self.declare_parameter("enable_headlight_detector", True)
        self.declare_parameter("headlight_camera_side", "left")
        self.declare_parameter("headlight_left_image_topic", "/usb_cam/image_raw/left")
        self.declare_parameter("headlight_right_image_topic", "/usb_cam/image_raw/right")
        self.declare_parameter("headlight_behind_image_topic", "/usb_cam/image_raw/behind")
        self.declare_parameter("headlight_target_side", "strongest")
        self.declare_parameter("headlight_roi_x_side", "right")
        self.declare_parameter("headlight_roi_y_min_ratio", 0.46)
        self.declare_parameter("headlight_roi_y_max_ratio", 0.72)
        self.declare_parameter("headlight_left_roi_y_min_ratio", 0.46)
        self.declare_parameter("headlight_left_roi_y_max_ratio", 0.72)
        self.declare_parameter("headlight_right_roi_y_min_ratio", 0.44)
        self.declare_parameter("headlight_right_roi_y_max_ratio", 0.70)
        self.declare_parameter("headlight_roi_side_inner_ratio", 0.56)
        self.declare_parameter("headlight_roi_side_outer_margin_ratio", 0.06)
        self.declare_parameter("headlight_left_roi_side_inner_ratio", 0.52)
        self.declare_parameter("headlight_left_roi_side_outer_margin_ratio", 0.16)
        self.declare_parameter("headlight_right_roi_side_inner_ratio", 0.56)
        self.declare_parameter("headlight_right_roi_side_outer_margin_ratio", 0.06)
        self.declare_parameter("headlight_min_pair_gap_px", 14.0)
        self.declare_parameter("headlight_max_pair_gap_px", 180.0)
        self.declare_parameter("headlight_pair_y_tolerance_px", 28.0)
        self.declare_parameter("headlight_min_height_px", 12.0)
        self.declare_parameter("headlight_max_height_px", 45.0)
        self.declare_parameter("headlight_auto_lane_deadband_px", 45.0)
        self.declare_parameter("headlight_image_timeout_s", 0.7)
        self.declare_parameter("front_stop_distance_m", 0.0)
        self.declare_parameter("front_slow_distance_m", 0.9)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("scan_timeout_s", 1.0)
        self.declare_parameter("lane_dropout_hold_s", 0.0)
        self.declare_parameter("lane_dropout_speed_cmd", 15.0)
        self.declare_parameter("lane_good_confidence", 0.35)
        self.declare_parameter("straight_fast_speed_cmd", 30.0)
        self.declare_parameter("caution_speed_cmd", 12.0)
        self.declare_parameter("dynamic_speed_min_cmd", 12.0)
        self.declare_parameter("dynamic_speed_max_cmd", 30.0)
        self.declare_parameter("dynamic_lateral_accel_limit_mps2", 3.0)
        self.declare_parameter("dynamic_curve_score_deadband_px", 20.0)
        self.declare_parameter("dynamic_curve_score_to_curvature", 0.0015)
        self.declare_parameter("dynamic_angle_error_to_curve_score", 4.0)
        self.declare_parameter("dynamic_raw_angle_to_curve_score", 4.0)
        self.declare_parameter("straight_stable_frames", 3)
        self.declare_parameter("straight_curve_score_max_px", 30.0)
        self.declare_parameter("straight_angle_error_max_deg", 18.0)
        self.declare_parameter("straight_raw_angle_max_cmd", 24.0)
        self.declare_parameter("enable_yellow_lane_slowdown", True)
        self.declare_parameter("yellow_lane_slow_min_pixels", 3000)
        self.declare_parameter("yellow_lane_slow_edge_min_pixels", 700)
        self.declare_parameter("yellow_lane_min_component_height_px", 120)
        self.declare_parameter("yellow_lane_slow_speed_cmd", 6.0)
        self.declare_parameter("enable_overtake", True)
        self.declare_parameter("overtake_offset_px", -110)
        self.declare_parameter("overtake_speed_cmd", 15.0)
        self.declare_parameter("overtake_trigger_distance_m", 2.0)
        self.declare_parameter("overtake_clear_distance_m", 2.6)
        self.declare_parameter("overtake_min_hold_s", 1.4)
        self.declare_parameter("overtake_clear_hold_s", 0.6)
        self.declare_parameter("overtake_return_s", 1.0)
        self.declare_parameter("overtake_front_width_deg", 30.0)
        self.declare_parameter("enable_lidar_lane_avoidance", True)
        self.declare_parameter("lidar_lane_avoid_trigger_distance_m", 2.0)
        self.declare_parameter("lidar_lane_avoid_clear_distance_m", 2.6)
        self.declare_parameter("lidar_lane_avoid_side_min_deg", 8.0)
        self.declare_parameter("lidar_lane_avoid_speed_cmd", 10.0)
        self.declare_parameter("lidar_lane_avoid_preferred_side", "left")
        self.declare_parameter("lidar_lane_avoid_release_on_headlight", True)
        self.declare_parameter("lidar_positive_angle_side", "left")
        self.declare_parameter("lidar_front_center_deg", 0.0)
        self.declare_parameter("lidar_front_half_width_deg", 40.0)
        self.declare_parameter("publish_lidar_front_image", True)
        self.declare_parameter("lidar_front_view_max_m", 5.0)
        self.declare_parameter("enable_checkerboard_avoid_reset", True)
        self.declare_parameter("lidar_ignore_self_reflection", True)
        self.declare_parameter("lidar_self_reflect_min_rel_deg", 8.0)
        self.declare_parameter("lidar_self_reflect_max_rel_deg", 24.0)
        self.declare_parameter("lidar_self_reflect_max_distance_m", 0.9)
        self.declare_parameter("enable_lidar_distance_follow", True)
        self.declare_parameter("enable_lidar_distance_hold", False)
        self.declare_parameter("lidar_min_valid_distance_m", 0.2)
        self.declare_parameter("follow_stop_distance_m", 0.8)
        self.declare_parameter("follow_crawl_distance_m", 1.2)
        self.declare_parameter("follow_slow_distance_m", 1.8)
        self.declare_parameter("follow_clear_distance_m", 2.6)
        self.declare_parameter("follow_crawl_speed_cmd", 4.0)
        self.declare_parameter("follow_slow_speed_cmd", 8.0)
        self.declare_parameter("follow_mid_speed_cmd", 12.0)
        self.declare_parameter("follow_target_distance_m", 1.2)
        self.declare_parameter("follow_deadband_m", 0.12)
        self.declare_parameter("follow_kp_speed_cmd_per_m", 8.0)
        self.declare_parameter("follow_reverse_speed_cmd", -3.0)
        self.declare_parameter("follow_forward_max_speed_cmd", 8.0)
        self.declare_parameter("follow_no_target_speed_cmd", 0.0)
        self.declare_parameter("base_speed_cmd", 8.0)
        self.declare_parameter("curve_speed_cmd", 12.0)
        self.declare_parameter("slow_speed_cmd", 8.0)
        self.declare_parameter("lookahead_m", 0.4)
        self.declare_parameter("speed_cmd_to_mps", 0.5)
        self.declare_parameter("compensate_steering_delay", True)
        self.declare_parameter("steering_prediction_horizon_s", 0.16)
        self.declare_parameter("steering_bias_deg", 0.0)
        self.declare_parameter("curve_steering_gain", 2.5)
        self.declare_parameter("curve_steering_score_start_px", 40.0)
        self.declare_parameter("curve_steering_score_full_px", 160.0)
        self.declare_parameter("curve_steering_error_start_deg", 15.0)
        self.declare_parameter("lane_threshold_mode", "auturbo_white_hsv")
        self.declare_parameter("white_s_max", 35)
        self.declare_parameter("white_v_min", 190)
        self.declare_parameter("lane_target_lookahead_y_px", 65)
        self.declare_parameter("lane_visibility_mode", "all")
        self.declare_parameter("enable_timed_start_sequence", False)
        self.declare_parameter("timed_start_straight_s", 3.7)
        self.declare_parameter("timed_start_yaw_s", 1.0)
        self.declare_parameter("timed_start_speed_cmd", 11.111)
        self.declare_parameter("timed_start_yaw_angle_cmd", 0.0)
        self.declare_parameter("enable_cone_mode", False)
        self.declare_parameter("cone_start_active", True)
        self.declare_parameter("cone_finish_to_lane", True)
        self.declare_parameter("cone_publish_debug_image", True)
        self.declare_parameter("cone_cruise_speed", 5.0)
        self.declare_parameter("cone_min_speed", 3.0)
        self.declare_parameter("cone_corner_speed", 3.5)
        self.declare_parameter("cone_finish_speed", 3.0)
        self.declare_parameter("cone_max_steer", 80.0)
        self.declare_parameter("cone_pursuit_gain", 4.2)
        self.declare_parameter("cone_lateral_gain", 18.0)
        self.declare_parameter("cone_clearance_gain", 12.0)
        self.declare_parameter("cone_corner_boost", 1.35)
        self.declare_parameter("cone_smooth_alpha_straight", 0.40)
        self.declare_parameter("cone_smooth_alpha_corner", 0.65)
        self.declare_parameter("cone_lookahead_y_min", 0.45)
        self.declare_parameter("cone_lookahead_y_max", 6.5)
        self.declare_parameter("cone_lookahead_target_y", 3.0)
        self.declare_parameter("cone_left_x_min", -3.5)
        self.declare_parameter("cone_left_x_max", -0.55)
        self.declare_parameter("cone_right_x_min", 0.55)
        self.declare_parameter("cone_right_x_max", 3.2)
        self.declare_parameter("cone_half_corridor", 2.25)
        self.declare_parameter("cone_left_wall_half", 2.15)
        self.declare_parameter("cone_left_wall_center_bias", 0.12)
        self.declare_parameter("cone_front_slow_dist", 1.0)
        self.declare_parameter("cone_front_stop_dist", 0.45)
        self.declare_parameter("cone_clear_frames_required", 25)
        self.declare_parameter("cone_min_tracked_frames", 80)
        self.declare_parameter("cone_yellow_pixel_threshold", 2500)
        self.declare_parameter("cone_yellow_roi_ratio", 0.45)
        self.declare_parameter("cone_hold_max_frames", 25)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.motor_topic = str(self.get_parameter("motor_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.yolo_topic = str(self.get_parameter("yolo_topic").value)
        self.control_hz = float(self.get_parameter("control_hz").value)
        self.publish_debug_images = bool(self.get_parameter("publish_debug_images").value)
        self.enable_start_signal_gate = bool(self.get_parameter("enable_start_signal_gate").value)
        self.start_signal_release_latched = bool(self.get_parameter("start_signal_release_latched").value)
        self.enable_stopline = bool(self.get_parameter("enable_stopline").value)
        self.enable_lidar_obstacle = bool(self.get_parameter("enable_lidar_obstacle").value)
        self.enable_object_detector = bool(self.get_parameter("enable_object_detector").value)
        self.enable_headlight_detector = bool(self.get_parameter("enable_headlight_detector").value)
        self.headlight_camera_side = str(self.get_parameter("headlight_camera_side").value)
        self.headlight_left_image_topic = str(self.get_parameter("headlight_left_image_topic").value)
        self.headlight_right_image_topic = str(self.get_parameter("headlight_right_image_topic").value)
        self.headlight_behind_image_topic = str(self.get_parameter("headlight_behind_image_topic").value)
        self.headlight_target_side = str(self.get_parameter("headlight_target_side").value)
        self.headlight_roi_x_side = str(self.get_parameter("headlight_roi_x_side").value)
        self.headlight_roi_y_min_ratio = float(self.get_parameter("headlight_roi_y_min_ratio").value)
        self.headlight_roi_y_max_ratio = float(self.get_parameter("headlight_roi_y_max_ratio").value)
        self.headlight_left_roi_y_min_ratio = float(self.get_parameter("headlight_left_roi_y_min_ratio").value)
        self.headlight_left_roi_y_max_ratio = float(self.get_parameter("headlight_left_roi_y_max_ratio").value)
        self.headlight_right_roi_y_min_ratio = float(self.get_parameter("headlight_right_roi_y_min_ratio").value)
        self.headlight_right_roi_y_max_ratio = float(self.get_parameter("headlight_right_roi_y_max_ratio").value)
        self.headlight_roi_side_inner_ratio = float(self.get_parameter("headlight_roi_side_inner_ratio").value)
        self.headlight_roi_side_outer_margin_ratio = float(
            self.get_parameter("headlight_roi_side_outer_margin_ratio").value
        )
        self.headlight_left_roi_side_inner_ratio = float(
            self.get_parameter("headlight_left_roi_side_inner_ratio").value
        )
        self.headlight_left_roi_side_outer_margin_ratio = float(
            self.get_parameter("headlight_left_roi_side_outer_margin_ratio").value
        )
        self.headlight_right_roi_side_inner_ratio = float(
            self.get_parameter("headlight_right_roi_side_inner_ratio").value
        )
        self.headlight_right_roi_side_outer_margin_ratio = float(
            self.get_parameter("headlight_right_roi_side_outer_margin_ratio").value
        )
        self.headlight_min_pair_gap_px = float(self.get_parameter("headlight_min_pair_gap_px").value)
        self.headlight_max_pair_gap_px = float(self.get_parameter("headlight_max_pair_gap_px").value)
        self.headlight_pair_y_tolerance_px = float(self.get_parameter("headlight_pair_y_tolerance_px").value)
        self.headlight_min_height_px = float(self.get_parameter("headlight_min_height_px").value)
        self.headlight_max_height_px = float(self.get_parameter("headlight_max_height_px").value)
        self.headlight_auto_lane_deadband_px = float(self.get_parameter("headlight_auto_lane_deadband_px").value)
        self.headlight_image_timeout_s = float(self.get_parameter("headlight_image_timeout_s").value)
        self.front_stop_distance_m = float(self.get_parameter("front_stop_distance_m").value)
        self.front_slow_distance_m = float(self.get_parameter("front_slow_distance_m").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.scan_timeout_s = float(self.get_parameter("scan_timeout_s").value)
        self.lane_dropout_hold_s = float(self.get_parameter("lane_dropout_hold_s").value)
        self.lane_dropout_speed_cmd = float(self.get_parameter("lane_dropout_speed_cmd").value)
        self.lane_good_confidence = float(self.get_parameter("lane_good_confidence").value)
        self.straight_fast_speed_cmd = float(self.get_parameter("straight_fast_speed_cmd").value)
        self.caution_speed_cmd = float(self.get_parameter("caution_speed_cmd").value)
        self.dynamic_speed_min_cmd = float(self.get_parameter("dynamic_speed_min_cmd").value)
        self.dynamic_speed_max_cmd = float(self.get_parameter("dynamic_speed_max_cmd").value)
        self.dynamic_lateral_accel_limit_mps2 = float(
            self.get_parameter("dynamic_lateral_accel_limit_mps2").value
        )
        self.dynamic_curve_score_deadband_px = float(self.get_parameter("dynamic_curve_score_deadband_px").value)
        self.dynamic_curve_score_to_curvature = float(self.get_parameter("dynamic_curve_score_to_curvature").value)
        self.dynamic_angle_error_to_curve_score = float(self.get_parameter("dynamic_angle_error_to_curve_score").value)
        self.dynamic_raw_angle_to_curve_score = float(self.get_parameter("dynamic_raw_angle_to_curve_score").value)
        self.straight_stable_frames = int(self.get_parameter("straight_stable_frames").value)
        self.straight_curve_score_max_px = float(self.get_parameter("straight_curve_score_max_px").value)
        self.straight_angle_error_max_deg = float(self.get_parameter("straight_angle_error_max_deg").value)
        self.straight_raw_angle_max_cmd = float(self.get_parameter("straight_raw_angle_max_cmd").value)
        self.enable_yellow_lane_slowdown = bool(self.get_parameter("enable_yellow_lane_slowdown").value)
        self.yellow_lane_slow_min_pixels = int(self.get_parameter("yellow_lane_slow_min_pixels").value)
        self.yellow_lane_slow_edge_min_pixels = int(self.get_parameter("yellow_lane_slow_edge_min_pixels").value)
        self.yellow_lane_min_component_height_px = int(self.get_parameter("yellow_lane_min_component_height_px").value)
        self.yellow_lane_slow_speed_cmd = float(self.get_parameter("yellow_lane_slow_speed_cmd").value)
        self.enable_overtake = bool(self.get_parameter("enable_overtake").value)
        self.overtake_offset_px = int(self.get_parameter("overtake_offset_px").value)
        self.overtake_speed_cmd = float(self.get_parameter("overtake_speed_cmd").value)
        self.overtake_trigger_distance_m = float(self.get_parameter("overtake_trigger_distance_m").value)
        self.overtake_clear_distance_m = float(self.get_parameter("overtake_clear_distance_m").value)
        self.overtake_min_hold_s = float(self.get_parameter("overtake_min_hold_s").value)
        self.overtake_clear_hold_s = float(self.get_parameter("overtake_clear_hold_s").value)
        self.overtake_return_s = float(self.get_parameter("overtake_return_s").value)
        self.overtake_front_width_deg = float(self.get_parameter("overtake_front_width_deg").value)
        self.enable_lidar_lane_avoidance = bool(self.get_parameter("enable_lidar_lane_avoidance").value)
        self.lidar_lane_avoid_trigger_distance_m = float(
            self.get_parameter("lidar_lane_avoid_trigger_distance_m").value
        )
        self.lidar_lane_avoid_clear_distance_m = float(self.get_parameter("lidar_lane_avoid_clear_distance_m").value)
        self.lidar_lane_avoid_side_min_deg = float(self.get_parameter("lidar_lane_avoid_side_min_deg").value)
        self.lidar_lane_avoid_speed_cmd = float(self.get_parameter("lidar_lane_avoid_speed_cmd").value)
        self.lidar_lane_avoid_preferred_side = str(self.get_parameter("lidar_lane_avoid_preferred_side").value)
        self.lidar_lane_avoid_release_on_headlight = bool(
            self.get_parameter("lidar_lane_avoid_release_on_headlight").value
        )
        self.lidar_positive_angle_side = str(self.get_parameter("lidar_positive_angle_side").value)
        self.lidar_front_center_deg = float(self.get_parameter("lidar_front_center_deg").value)
        self.lidar_front_half_width_deg = float(self.get_parameter("lidar_front_half_width_deg").value)
        self.publish_lidar_front_image = bool(self.get_parameter("publish_lidar_front_image").value)
        self.lidar_front_view_max_m = float(self.get_parameter("lidar_front_view_max_m").value)
        self.enable_checkerboard_avoid_reset = bool(self.get_parameter("enable_checkerboard_avoid_reset").value)
        self.lidar_ignore_self_reflection = bool(self.get_parameter("lidar_ignore_self_reflection").value)
        self.lidar_self_reflect_min_rel_deg = float(self.get_parameter("lidar_self_reflect_min_rel_deg").value)
        self.lidar_self_reflect_max_rel_deg = float(self.get_parameter("lidar_self_reflect_max_rel_deg").value)
        self.lidar_self_reflect_max_distance_m = float(self.get_parameter("lidar_self_reflect_max_distance_m").value)
        self.enable_lidar_distance_follow = bool(self.get_parameter("enable_lidar_distance_follow").value)
        self.enable_lidar_distance_hold = bool(self.get_parameter("enable_lidar_distance_hold").value)
        self.lidar_min_valid_distance_m = float(self.get_parameter("lidar_min_valid_distance_m").value)
        self.follow_stop_distance_m = float(self.get_parameter("follow_stop_distance_m").value)
        self.follow_crawl_distance_m = float(self.get_parameter("follow_crawl_distance_m").value)
        self.follow_slow_distance_m = float(self.get_parameter("follow_slow_distance_m").value)
        self.follow_clear_distance_m = float(self.get_parameter("follow_clear_distance_m").value)
        self.follow_crawl_speed_cmd = float(self.get_parameter("follow_crawl_speed_cmd").value)
        self.follow_slow_speed_cmd = float(self.get_parameter("follow_slow_speed_cmd").value)
        self.follow_mid_speed_cmd = float(self.get_parameter("follow_mid_speed_cmd").value)
        self.follow_target_distance_m = float(self.get_parameter("follow_target_distance_m").value)
        self.follow_deadband_m = float(self.get_parameter("follow_deadband_m").value)
        self.follow_kp_speed_cmd_per_m = float(self.get_parameter("follow_kp_speed_cmd_per_m").value)
        self.follow_reverse_speed_cmd = float(self.get_parameter("follow_reverse_speed_cmd").value)
        self.follow_forward_max_speed_cmd = float(self.get_parameter("follow_forward_max_speed_cmd").value)
        self.follow_no_target_speed_cmd = float(self.get_parameter("follow_no_target_speed_cmd").value)
        self.base_speed_cmd = float(self.get_parameter("base_speed_cmd").value)
        self.curve_speed_cmd = float(self.get_parameter("curve_speed_cmd").value)
        self.slow_speed_cmd = float(self.get_parameter("slow_speed_cmd").value)
        self.lookahead_m = float(self.get_parameter("lookahead_m").value)
        self.speed_cmd_to_mps = float(self.get_parameter("speed_cmd_to_mps").value)
        self.compensate_steering_delay = bool(self.get_parameter("compensate_steering_delay").value)
        self.steering_prediction_horizon_s = float(self.get_parameter("steering_prediction_horizon_s").value)
        self.steering_bias_deg = float(self.get_parameter("steering_bias_deg").value)
        self.curve_steering_gain = float(self.get_parameter("curve_steering_gain").value)
        self.curve_steering_score_start_px = float(self.get_parameter("curve_steering_score_start_px").value)
        self.curve_steering_score_full_px = float(self.get_parameter("curve_steering_score_full_px").value)
        self.curve_steering_error_start_deg = float(self.get_parameter("curve_steering_error_start_deg").value)
        self.lane_threshold_mode = str(self.get_parameter("lane_threshold_mode").value)
        self.white_s_max = int(self.get_parameter("white_s_max").value)
        self.white_v_min = int(self.get_parameter("white_v_min").value)
        self.lane_target_lookahead_y_px = int(self.get_parameter("lane_target_lookahead_y_px").value)
        self.lane_visibility_mode = str(self.get_parameter("lane_visibility_mode").value)
        self.enable_timed_start_sequence = bool(self.get_parameter("enable_timed_start_sequence").value)
        self.timed_start_straight_s = max(0.0, float(self.get_parameter("timed_start_straight_s").value))
        self.timed_start_yaw_s = max(0.0, float(self.get_parameter("timed_start_yaw_s").value))
        self.timed_start_speed_cmd = float(self.get_parameter("timed_start_speed_cmd").value)
        self.timed_start_yaw_angle_cmd = float(self.get_parameter("timed_start_yaw_angle_cmd").value)
        self.enable_cone_mode = bool(self.get_parameter("enable_cone_mode").value)
        self.cone_mode_active = bool(self.get_parameter("cone_start_active").value)
        self.cone_finish_to_lane = bool(self.get_parameter("cone_finish_to_lane").value)
        self.cone_publish_debug_image = bool(self.get_parameter("cone_publish_debug_image").value)
        self.cone_config = ConeFollowerConfig(
            cruise_speed=float(self.get_parameter("cone_cruise_speed").value),
            min_speed=float(self.get_parameter("cone_min_speed").value),
            corner_speed=float(self.get_parameter("cone_corner_speed").value),
            finish_speed=float(self.get_parameter("cone_finish_speed").value),
            max_steer=float(self.get_parameter("cone_max_steer").value),
            pursuit_gain=float(self.get_parameter("cone_pursuit_gain").value),
            lateral_gain=float(self.get_parameter("cone_lateral_gain").value),
            clearance_gain=float(self.get_parameter("cone_clearance_gain").value),
            corner_boost=float(self.get_parameter("cone_corner_boost").value),
            smooth_alpha_straight=float(self.get_parameter("cone_smooth_alpha_straight").value),
            smooth_alpha_corner=float(self.get_parameter("cone_smooth_alpha_corner").value),
            lookahead_y_min=float(self.get_parameter("cone_lookahead_y_min").value),
            lookahead_y_max=float(self.get_parameter("cone_lookahead_y_max").value),
            lookahead_target_y=float(self.get_parameter("cone_lookahead_target_y").value),
            left_x_min=float(self.get_parameter("cone_left_x_min").value),
            left_x_max=float(self.get_parameter("cone_left_x_max").value),
            right_x_min=float(self.get_parameter("cone_right_x_min").value),
            right_x_max=float(self.get_parameter("cone_right_x_max").value),
            half_corridor=float(self.get_parameter("cone_half_corridor").value),
            left_wall_half=float(self.get_parameter("cone_left_wall_half").value),
            left_wall_center_bias=float(self.get_parameter("cone_left_wall_center_bias").value),
            front_slow_dist=float(self.get_parameter("cone_front_slow_dist").value),
            front_stop_dist=float(self.get_parameter("cone_front_stop_dist").value),
            clear_frames_required=int(self.get_parameter("cone_clear_frames_required").value),
            min_tracked_frames=int(self.get_parameter("cone_min_tracked_frames").value),
            yellow_pixel_threshold=int(self.get_parameter("cone_yellow_pixel_threshold").value),
            yellow_roi_ratio=float(self.get_parameter("cone_yellow_roi_ratio").value),
            hold_max_frames=int(self.get_parameter("cone_hold_max_frames").value),
        )

        self.lane_detector = LaneDetector(
            threshold_mode=self.lane_threshold_mode,
            white_s_max=self.white_s_max,
            white_v_min=self.white_v_min,
            target_lookahead_y_px=self.lane_target_lookahead_y_px,
            visibility_mode=self.lane_visibility_mode,
        )
        self.lane_controller = LaneController(
            default_speed=self.base_speed_cmd,
            curve_speed=self.curve_speed_cmd,
            slow_speed=self.slow_speed_cmd,
            lookahead_m=self.lookahead_m,
            compensate_steering_delay=self.compensate_steering_delay,
            steering_prediction_horizon_s=self.steering_prediction_horizon_s,
            steering_bias_deg=self.steering_bias_deg,
            curve_steering_gain=self.curve_steering_gain,
            curve_steering_score_start_px=self.curve_steering_score_start_px,
            curve_steering_score_full_px=self.curve_steering_score_full_px,
            curve_steering_error_start_deg=self.curve_steering_error_start_deg,
        )
        self.cone_follower = ConeFollower(self.cone_config)
        self.obstacle_detector = ObstacleDetector()
        self.object_detector = VehicleObjectDetector()
        self.headlight_left_roi_x_side = self.headlight_roi_side_for_camera("left")
        self.headlight_right_roi_x_side = self.headlight_roi_side_for_camera("right")
        left_roi_y_min, left_roi_y_max = self.headlight_roi_y_for_camera("left")
        right_roi_y_min, right_roi_y_max = self.headlight_roi_y_for_camera("right")
        left_roi_inner, left_roi_outer = self.headlight_roi_geometry_for_camera("left")
        right_roi_inner, right_roi_outer = self.headlight_roi_geometry_for_camera("right")
        self.headlight_detector = HeadlightDetector(
            image_width=640,
            target_side=self.headlight_target_side,
            roi_x_side=self.headlight_left_roi_x_side,
            roi_y_min_ratio=left_roi_y_min,
            roi_y_max_ratio=left_roi_y_max,
            min_pair_y_ratio=left_roi_y_min,
            roi_side_inner_ratio=left_roi_inner,
            roi_side_outer_margin_ratio=left_roi_outer,
            min_pair_gap_px=self.headlight_min_pair_gap_px,
            max_pair_gap_px=self.headlight_max_pair_gap_px,
            pair_y_tolerance_px=self.headlight_pair_y_tolerance_px,
            min_height_px=self.headlight_min_height_px,
            max_height_px=self.headlight_max_height_px,
        )
        self.headlight_detectors = {
            "left": HeadlightDetector(
                image_width=640,
                target_side="strongest",
                roi_x_side=self.headlight_left_roi_x_side,
                roi_y_min_ratio=left_roi_y_min,
                roi_y_max_ratio=left_roi_y_max,
                min_pair_y_ratio=left_roi_y_min,
                roi_side_inner_ratio=left_roi_inner,
                roi_side_outer_margin_ratio=left_roi_outer,
                min_pair_gap_px=self.headlight_min_pair_gap_px,
                max_pair_gap_px=self.headlight_max_pair_gap_px,
                pair_y_tolerance_px=self.headlight_pair_y_tolerance_px,
                min_height_px=self.headlight_min_height_px,
                max_height_px=self.headlight_max_height_px,
            ),
            "right": HeadlightDetector(
                image_width=640,
                target_side="strongest",
                roi_x_side=self.headlight_right_roi_x_side,
                roi_y_min_ratio=right_roi_y_min,
                roi_y_max_ratio=right_roi_y_max,
                min_pair_y_ratio=right_roi_y_min,
                roi_side_inner_ratio=right_roi_inner,
                roi_side_outer_margin_ratio=right_roi_outer,
                min_pair_gap_px=self.headlight_min_pair_gap_px,
                max_pair_gap_px=self.headlight_max_pair_gap_px,
                pair_y_tolerance_px=self.headlight_pair_y_tolerance_px,
                min_height_px=self.headlight_min_height_px,
                max_height_px=self.headlight_max_height_px,
            ),
        }
        self.stopline_detector = StopLineDetector()
        self.start_signal_detector = TrafficLightDetector()
        self.last_object = ObjectResult()
        self.last_headlight = HeadlightResult()
        self.last_headlight_by_camera = {
            "left": HeadlightResult(),
            "right": HeadlightResult(),
        }
        self.last_headlight_image_age_by_camera = {
            "left": float("inf"),
            "right": float("inf"),
        }
        self.last_lane = LaneResult(320, 0.0, 0.0, "no_image", None, None, None)
        self.last_obstacle = ObstacleResult()
        self.last_lane_seen_time = 0.0
        self.has_lane_hold_command = False
        self.last_lane_hold_angle = 0.0
        self.last_lane_hold_speed = self.lane_dropout_speed_cmd
        self.straight_frame_count = 0
        self.last_speed_mode = "none"
        self.last_dynamic_curve_score = 0.0
        self.last_dynamic_curvature = 0.0
        self.last_dynamic_speed_mps = 0.0
        self.yellow_lane_pixels = 0
        self.yellow_lane_edge_pixels = 0
        self.yellow_lane_slowdown_active = False
        self.overtake_state = "idle"
        self.overtake_until = 0.0
        self.overtake_clear_since = 0.0
        self.overtake_return_until = 0.0
        self.overtake_target_offset_px = 0
        self.overtake_front_distance_m = float("inf")
        self.lidar_avoid_target_side = "middle"
        self.lidar_avoid_front_blocked = False
        self.lidar_avoid_blocked_side = "none"
        self.lidar_avoid_left_clear_m = float("inf")
        self.lidar_avoid_right_clear_m = float("inf")
        self.lidar_avoid_verify_camera_side = "none"
        self.lidar_avoid_state = "idle"
        self.lidar_avoid_latched_target_side = "middle"
        self.lidar_avoid_latched_blocked_side = "none"
        self.lidar_avoid_latched_verify_camera_side = "none"
        self.lidar_avoid_release_camera_side = "none"
        self.lidar_avoid_left_headlight_confirmed = False
        self.lidar_avoid_locked_until_checkerboard = False
        self.lidar_avoid_left_headlight_confirmed = False
        self.checkerboard_avoid_reset_detected = False
        self.follow_state = "clear"
        self.follow_speed_limit_cmd = self.dynamic_speed_max_cmd
        self.effective_headlight_camera_side = self.headlight_camera_side
        self.effective_headlight_target_side = self.headlight_target_side
        self.last_headlight_image_age = float("inf")
        self.start_signal_state = TrafficLightDetector.UNKNOWN
        self.start_signal_released = False
        self.start_signal_gate_open = not self.enable_start_signal_gate
        self.start_signal_scores = {
            TrafficLightDetector.START_RED: 0.0,
            TrafficLightDetector.START_YELLOW: 0.0,
            TrafficLightDetector.START_GREEN: 0.0,
        }
        self.stopline_until = 0.0
        self.timed_start_begin_time = 0.0
        self.timed_start_yaw_begin_time = 0.0
        self.timed_start_yaw0_rad: float | None = None
        self.timed_start_yaw0_deg: float | None = None
        self.timed_start_done = False
        self.timed_start_phase = "disabled" if not self.enable_timed_start_sequence else "waiting"
        self.cone_finished = False
        self.last_cone_cmd = ConeCommand()

        self.motor_pub = self.create_publisher(XycarMotor, self.motor_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, "/track_drive/lane_debug_image", 3)
        self.bev_image_pub = self.create_publisher(Image, "/track_drive/lane_bev_image", 3)
        self.mask_image_pub = self.create_publisher(Image, "/track_drive/lane_mask_image", 3)
        self.cone_debug_image_pub = self.create_publisher(Image, "/track_drive/cone_debug_image", 3)
        self.start_signal_debug_image_pub = self.create_publisher(
            Image, "/track_drive/start_signal_debug_image", 3
        )
        self.headlight_left_debug_image_pub = self.create_publisher(
            Image, "/track_drive/headlight_left_debug_image", 3
        )
        self.headlight_right_debug_image_pub = self.create_publisher(
            Image, "/track_drive/headlight_right_debug_image", 3
        )
        self.lidar_front_image_pub = self.create_publisher(Image, "/track_drive/lidar_front_image", 3)

        self.create_subscription(Image, self.image_topic, self.cam_callback, qos_profile_sensor_data)
        self.create_subscription(
            Image,
            self.headlight_left_image_topic,
            lambda msg: self.headlight_cam_callback("left", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.headlight_right_image_topic,
            lambda msg: self.headlight_cam_callback("right", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.headlight_behind_image_topic,
            lambda msg: self.headlight_cam_callback("behind", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(LaserScan, self.scan_topic, self.lidar_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos_profile_sensor_data)
        if self.enable_object_detector and BoundingBoxes is not None:
            self.create_subscription(BoundingBoxes, self.yolo_topic, self.yolo_callback, 10)
            self.get_logger().info(f"YOLO vehicle detector enabled: {self.yolo_topic}")
        elif self.enable_object_detector:
            self.get_logger().warn("detection_msgs/BoundingBoxes not available; YOLO vehicle detector disabled")

        self.create_timer(1.0 / max(1.0, self.control_hz), self.control_step)
        self.get_logger().info(
            "AuTURBO ROS2 driver started: "
            f"image={self.image_topic}, scan={self.scan_topic}, imu={self.imu_topic}, motor={self.motor_topic}"
        )

    def cam_callback(self, data: Image) -> None:
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.last_image_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f"camera conversion failed: {exc}")

    def headlight_cam_callback(self, side: str, data: Image) -> None:
        try:
            self.headlight_images[side] = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.headlight_image_times[side] = time.monotonic()
        except Exception as exc:
            self.get_logger().warn(f"{side} camera conversion failed: {exc}")

    def lidar_callback(self, msg: LaserScan) -> None:
        self.lidar_ranges = list(msg.ranges)
        self.lidar_angle_min = float(msg.angle_min)
        self.lidar_angle_increment = float(msg.angle_increment)
        self.last_scan_time = time.monotonic()
        if self.publish_lidar_front_image:
            self.publish_lidar_front_image_msg(msg)

    def imu_callback(self, msg: Imu) -> None:
        if math.isfinite(msg.angular_velocity.z):
            self.yaw_rate = float(msg.angular_velocity.z)
        orientation = msg.orientation
        norm = math.sqrt(
            orientation.x * orientation.x
            + orientation.y * orientation.y
            + orientation.z * orientation.z
            + orientation.w * orientation.w
        )
        if norm > 1e-6:
            x = orientation.x / norm
            y = orientation.y / norm
            z = orientation.z / norm
            w = orientation.w / norm
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.current_yaw_rad = yaw
            self.current_yaw_deg = math.degrees(yaw)

    def yolo_callback(self, msg: Any) -> None:
        self.last_object = self.object_detector.update_from_msg(msg)

    def publish_lidar_front_image_msg(self, msg: LaserScan) -> None:
        width = 720
        height = 720
        origin = (width // 2, height // 2)
        view_max_m = max(8.0, float(self.lidar_front_view_max_m), float(self.lidar_lane_avoid_clear_distance_m))
        radius = min(width, height) // 2 - 68
        scale = radius / view_max_m

        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        canvas[:] = (8, 10, 12)
        front_angle = math.radians(self.lidar_front_center_deg)

        def point_for(relative_deg: float, meters: float) -> tuple[int, int]:
            rel = math.radians(relative_deg)
            x = origin[0] - math.sin(rel) * meters * scale
            y = origin[1] - math.cos(rel) * meters * scale
            return int(round(x)), int(round(y))

        def draw_spoke(relative_deg: float, color: tuple[int, int, int], thickness: int = 1, label: str = "") -> None:
            end = point_for(relative_deg, view_max_m)
            cv2.line(canvas, origin, end, color, thickness)
            if label:
                lx, ly = point_for(relative_deg, view_max_m + 0.28)
                cv2.putText(canvas, label, (lx - 34, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        front_half = max(0.0, float(self.overtake_front_width_deg))
        dead_half = min(front_half, max(0.0, float(self.lidar_lane_avoid_side_min_deg)))
        for meters in (1.0, 2.0, 3.0, 5.0):
            if meters <= view_max_m:
                cv2.circle(canvas, origin, int(round(meters * scale)), (28, 36, 42), 1)

        draw_spoke(0.0, (0, 255, 255), 4, "0deg")
        if dead_half > 0.0:
            draw_spoke(-dead_half, (255, 0, 255), 2, f"-{dead_half:.0f} dead")
            draw_spoke(dead_half, (255, 0, 255), 2, f"+{dead_half:.0f} dead")
        draw_spoke(-front_half, (0, 180, 255), 2, f"-{front_half:.0f}deg")
        draw_spoke(front_half, (0, 180, 255), 2, f"+{front_half:.0f}deg")
        if dead_half > 0.0:
            cv2.ellipse(
                canvas,
                origin,
                (radius, radius),
                -90,
                -dead_half,
                dead_half,
                (120, 0, 120),
                2,
                cv2.LINE_AA,
            )
        cv2.ellipse(
            canvas,
            origin,
            (radius, radius),
            -90,
            -front_half,
            front_half,
            (0, 90, 120),
            2,
            cv2.LINE_AA,
        )

        closest_front: tuple[float, float, int] | None = None
        drawn = 0
        for idx, distance in enumerate(msg.ranges):
            distance = float(distance)
            if not math.isfinite(distance) or not (msg.range_min < distance < min(msg.range_max, view_max_m)):
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            relative_deg = math.degrees(self.wrap_angle_pi(angle - front_angle))
            px, py = point_for(relative_deg, distance)
            if not (0 <= px < width and 0 <= py < height):
                continue
            color = (90, 90, 90)
            size = 3
            if -front_half <= relative_deg <= front_half:
                color = (0, 240, 255)
                size = 5
                if distance > self.lidar_min_valid_distance_m and (
                    closest_front is None or distance < closest_front[0]
                ):
                    closest_front = (distance, relative_deg, idx)
            cv2.circle(canvas, (px, py), size, color, -1)
            drawn += 1

        if closest_front is not None:
            color = (0, 255, 255)
            distance, relative_deg, _ = closest_front
            px, py = point_for(relative_deg, distance)
            cv2.circle(canvas, (px, py), 12, (255, 255, 255), 1)
            cv2.line(canvas, origin, (px, py), color, 1)

        cv2.circle(canvas, origin, 9, (0, 0, 255), -1)
        cv2.putText(canvas, "CAR", (origin[0] - 16, origin[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 255), 1, cv2.LINE_AA)

        front_text = (
            f"front -{front_half:.0f}..+{front_half:.0f}: "
            + (f"{closest_front[0]:.2f}m @{closest_front[1]:+.0f}deg" if closest_front else "clear")
        )
        cv2.putText(canvas, f"polar lidar center={self.lidar_front_center_deg:.0f}deg dead=+-{dead_half:.0f} min>{self.lidar_min_valid_distance_m:.2f}m view={view_max_m:.1f}m", (16, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(canvas, front_text, (16, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.54, (0, 240, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"points {drawn}", (16, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 190, 160), 1, cv2.LINE_AA)

        self.lidar_front_image_pub.publish(self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8"))

    def drive(self, angle: float, speed: float) -> None:
        self.motor_msg.header.stamp = self.get_clock().now().to_msg()
        self.motor_msg.angle = float(angle)
        self.motor_msg.speed = float(speed)
        self.motor_pub.publish(self.motor_msg)
        self.lane_controller.observe_command(angle)

    def update_start_signal_scores(self) -> None:
        roi, _ = self.start_signal_detector.start_roi.crop(self.image)
        self.start_signal_scores = self.start_signal_detector._color_scores(roi)

    def publish_start_signal_debug_image(self) -> None:
        output = self.image.copy()
        _roi, (left, top, right, bottom) = self.start_signal_detector.start_roi.crop(output)
        color = {
            TrafficLightDetector.START_RED: (0, 0, 255),
            TrafficLightDetector.START_YELLOW: (0, 255, 255),
            TrafficLightDetector.START_GREEN: (0, 255, 0),
        }.get(self.start_signal_state, (160, 160, 160))
        cv2.rectangle(output, (left, top), (right, bottom), color, 2)
        cv2.putText(
            output,
            (
                f"START {self.start_signal_state} "
                f"R:{self.start_signal_scores[TrafficLightDetector.START_RED]:.0f} "
                f"Y:{self.start_signal_scores[TrafficLightDetector.START_YELLOW]:.0f} "
                f"G:{self.start_signal_scores[TrafficLightDetector.START_GREEN]:.0f}"
            ),
            (left + 6, max(22, top + 24)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )
        self.start_signal_debug_image_pub.publish(self.bridge.cv2_to_imgmsg(output, encoding="bgr8"))

    def handle_start_signal_gate(self, image_age: float, scan_age: float) -> bool:
        if not self.enable_start_signal_gate:
            self.start_signal_gate_open = True
            self.start_signal_state = "DISABLED"
            return False
        if self.start_signal_release_latched and self.start_signal_released:
            self.start_signal_gate_open = True
            if self.publish_debug_images:
                self.start_signal_state = self.start_signal_detector.detect_start_light(self.image)
                self.update_start_signal_scores()
                self.publish_start_signal_debug_image()
            return False

        self.start_signal_state = self.start_signal_detector.detect_start_light(self.image)
        self.update_start_signal_scores()
        self.start_signal_gate_open = self.start_signal_state == TrafficLightDetector.START_GREEN
        if self.publish_debug_images:
            self.publish_start_signal_debug_image()

        if self.start_signal_gate_open:
            self.start_signal_released = True
            return False

        self.last_speed_mode = "start_signal_wait"
        self.drive(0.0, 0.0)
        self.publish_debug("start_signal_wait", 0.0, 0.0, image_age, scan_age)
        return True

    def handle_timed_start_sequence(self, now: float, image_age: float, scan_age: float) -> bool:
        if not self.enable_timed_start_sequence or self.timed_start_done:
            if self.timed_start_done:
                self.timed_start_phase = "done"
            return False

        if self.timed_start_begin_time <= 0.0:
            self.timed_start_begin_time = now

        elapsed = now - self.timed_start_begin_time
        if elapsed < self.timed_start_straight_s:
            self.timed_start_phase = "straight"
            self.last_speed_mode = "timed_start_straight"
            self.drive(0.0, self.timed_start_speed_cmd)
            self.publish_debug("timed_start_straight", 0.0, self.timed_start_speed_cmd, image_age, scan_age)
            return True

        yaw_end_s = self.timed_start_straight_s + self.timed_start_yaw_s
        if elapsed < yaw_end_s:
            if self.timed_start_yaw_begin_time <= 0.0:
                self.timed_start_yaw_begin_time = now
                self.timed_start_yaw0_rad = self.current_yaw_rad
                self.timed_start_yaw0_deg = self.current_yaw_deg
            self.timed_start_phase = "yaw"
            self.last_speed_mode = "timed_start_yaw"
            angle = max(-100.0, min(100.0, self.timed_start_yaw_angle_cmd))
            self.drive(angle, self.timed_start_speed_cmd)
            self.publish_debug("timed_start_yaw", angle, self.timed_start_speed_cmd, image_age, scan_age)
            return True

        self.timed_start_done = True
        self.timed_start_phase = "done"
        return False

    def handle_cone_mode(self, image_age: float, scan_age: float) -> bool:
        if not self.enable_cone_mode:
            return False
        if self.cone_finished and self.cone_finish_to_lane:
            return False
        if not self.cone_mode_active:
            return False

        cmd = self.cone_follower.compute(self.lidar_ranges, self.image)
        self.last_cone_cmd = cmd
        self.last_speed_mode = "cone_mode"

        if self.publish_debug_images and self.cone_publish_debug_image and cmd.debug_image is not None:
            self.cone_debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cmd.debug_image, encoding="bgr8"))

        if cmd.finished:
            self.cone_finished = True
            self.cone_mode_active = False
            if self.cone_finish_to_lane:
                self.get_logger().info("cone section finished; handing off to lane driving")
                self.publish_debug("cone_finished", cmd.angle, cmd.speed, image_age, scan_age)
                return False
            self.drive(0.0, 0.0)
            self.publish_debug("cone_finished_stop", 0.0, 0.0, image_age, scan_age)
            return True

        self.drive(cmd.angle, cmd.speed)
        self.publish_debug("cone_follow", cmd.angle, cmd.speed, image_age, scan_age)
        return True

    def control_step(self) -> None:
        now = time.monotonic()
        image_age = now - self.last_image_time if self.last_image_time else float("inf")
        scan_age = now - self.last_scan_time if self.last_scan_time else float("inf")
        if self.image is None or image_age > self.image_timeout_s:
            self.drive(0.0, 0.0)
            self.publish_debug("no_fresh_image", 0.0, 0.0, image_age, scan_age)
            return

        if self.handle_start_signal_gate(image_age, scan_age):
            return

        if self.handle_cone_mode(image_age, scan_age):
            return

        if self.enable_headlight_detector:
            for side in ("left", "right"):
                result, age = self.detect_headlight_camera(side, now)
                self.last_headlight_by_camera[side] = result
                self.last_headlight_image_age_by_camera[side] = age
                if self.publish_debug_images and result.debug_image is not None:
                    roi_x_side = self.headlight_roi_side_for_camera(side)
                    cv2.putText(
                        result.debug_image,
                        f"camera={side} pair=strongest xroi={roi_x_side} age={age:.2f}s",
                        (12, 54),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.58,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    image_msg = self.bridge.cv2_to_imgmsg(result.debug_image, encoding="bgr8")
                    if side == "left":
                        self.headlight_left_debug_image_pub.publish(image_msg)
                    else:
                        self.headlight_right_debug_image_pub.publish(image_msg)

            self.refresh_effective_headlight_selection()
        else:
            self.effective_headlight_camera_side = "disabled"
            self.effective_headlight_target_side = "disabled"
            self.last_headlight_image_age = float("inf")
            self.last_headlight = HeadlightResult()
            for side in ("left", "right"):
                self.last_headlight_by_camera[side] = HeadlightResult()
                self.last_headlight_image_age_by_camera[side] = float("inf")

        obstacle_info = "middle"
        force_slow = False
        target_offset_px = 0
        front_stop_requested = False
        self.overtake_front_distance_m = (
            self.front_lidar_distance(self.overtake_front_width_deg) if scan_age <= self.scan_timeout_s else float("inf")
        )
        self.checkerboard_avoid_reset_detected = (
            self.enable_checkerboard_avoid_reset and self.detect_checkerboard_avoid_reset(self.image)
        )
        if self.checkerboard_avoid_reset_detected:
            self.lidar_avoid_locked_until_checkerboard = False
            self.reset_lidar_lane_avoidance()
        elif self.enable_lidar_lane_avoidance and scan_age <= self.scan_timeout_s:
            self.update_lidar_lane_avoidance()
            if self.enable_headlight_detector:
                self.refresh_effective_headlight_selection()
        else:
            self.reset_lidar_lane_avoidance()
        if self.enable_lidar_lane_avoidance:
            self.overtake_state = "idle"
            self.overtake_target_offset_px = 0
        elif self.enable_overtake:
            self.update_overtake_state(now, self.overtake_front_distance_m)
            target_offset_px = self.current_overtake_offset(now)
        if self.enable_lidar_obstacle and self.lidar_ranges is not None and scan_age <= self.scan_timeout_s:
            self.last_obstacle = self.obstacle_detector.detect(self.lidar_ranges, self.lidar_angle_increment)
            if self.overtake_front_distance_m < self.last_obstacle.front_distance_m:
                self.last_obstacle.front_distance_m = self.overtake_front_distance_m
            if self.last_obstacle.detected and not self.enable_lidar_lane_avoidance:
                obstacle_info = self.last_obstacle.direction
                force_slow = True
            if not self.enable_lidar_lane_avoidance and self.last_obstacle.front_distance_m <= self.front_stop_distance_m:
                front_stop_requested = True
                force_slow = True
            if not self.enable_lidar_lane_avoidance and self.last_obstacle.front_distance_m <= self.front_slow_distance_m:
                force_slow = True

        if self.enable_lidar_lane_avoidance and scan_age <= self.scan_timeout_s:
            if self.overtake_front_distance_m <= self.front_slow_distance_m:
                force_slow = True

        if self.lidar_avoid_target_side in {"left", "right"}:
            obstacle_info = self.lidar_avoid_target_side
            target_offset_px = 0
            force_slow = True

        if self.last_object.detected and self.lidar_avoid_target_side not in {"left", "right"}:
            obstacle_info = self.last_object.direction
            force_slow = True

        self.yellow_lane_pixels, self.yellow_lane_edge_pixels = self.count_yellow_lane_pixels(self.image)
        self.yellow_lane_slowdown_active = (
            self.enable_yellow_lane_slowdown
            and self.yellow_lane_pixels >= self.yellow_lane_slow_min_pixels
            and self.yellow_lane_edge_pixels >= self.yellow_lane_slow_edge_min_pixels
        )
        if self.yellow_lane_slowdown_active and self.lidar_avoid_target_side != "middle":
            self.reset_lidar_lane_avoidance()
            self.lidar_avoid_state = "yellow_lane_reset"
            obstacle_info = "middle"
            target_offset_px = 0
            force_slow = True

        if self.enable_stopline:
            stopline = self.stopline_detector.detect(self.image, make_debug=False)
            if stopline.detected:
                self.stopline_until = now + 3.0
        if now < self.stopline_until:
            self.drive(0.0, 0.0)
            self.publish_debug("stopline_hold", 0.0, 0.0, image_age, scan_age)
            return

        self.last_lane = self.lane_detector.detect(
            self.image,
            obstacle_info=obstacle_info,
            make_debug=self.publish_debug_images,
            target_offset_px=target_offset_px,
            bev_status_text=self.bev_status_text(),
        )

        if self.handle_timed_start_sequence(now, image_age, scan_age):
            self.publish_lane_images(self.last_lane)
            return

        debug_state = "lane_follow"
        lidar_lane_avoid_active = (
            self.lidar_avoid_target_side in {"left", "right"}
            and self.last_lane.confidence >= self.lane_good_confidence
        )
        if front_stop_requested and not lidar_lane_avoid_active:
            self.drive(0.0, 0.0)
            self.publish_lane_images(self.last_lane)
            self.publish_debug("front_stop", 0.0, 0.0, image_age, scan_age)
            return
        if self.last_lane.confidence >= self.lane_good_confidence:
            angle, speed = self.lane_controller.compute(
                self.last_lane.angle_error,
                self.last_lane.confidence,
                force_slow,
                self.last_lane.curve_score,
            )
            speed = self.select_lane_speed(self.last_lane, self.lane_controller.last_raw_angle_cmd)
            if lidar_lane_avoid_active:
                speed = min(speed, self.lidar_lane_avoid_speed_cmd)
                self.last_speed_mode = "lidar_lane_avoid"
                debug_state = "lidar_lane_avoid"
            elif self.overtake_state == "passing":
                speed = max(speed, min(self.overtake_speed_cmd, self.dynamic_speed_max_cmd))
            self.last_lane_seen_time = now
            self.last_lane_hold_angle = angle
            self.last_lane_hold_speed = min(speed, self.lane_dropout_speed_cmd)
            self.has_lane_hold_command = True
        elif self.has_lane_hold_command and (
            self.lane_dropout_hold_s <= 0.0 or now - self.last_lane_seen_time <= self.lane_dropout_hold_s
        ):
            angle = self.last_lane_hold_angle
            speed = self.last_lane_hold_speed
            self.straight_frame_count = 0
            self.last_speed_mode = "lane_hold"
            debug_state = "lane_hold"
        else:
            angle = 0.0
            speed = 0.0
            self.straight_frame_count = 0
            self.last_speed_mode = "lane_lost"
            debug_state = "lane_lost"
        if lidar_lane_avoid_active:
            speed = min(speed, self.lidar_lane_avoid_speed_cmd)
            self.last_speed_mode = "lidar_lane_avoid"
        elif self.enable_lidar_distance_hold and scan_age <= self.scan_timeout_s:
            speed = self.distance_hold_speed_cmd(self.overtake_front_distance_m)
            if self.follow_state in {"hold_reverse", "hold_keep", "hold_no_target"}:
                angle = 0.0
            self.last_speed_mode = self.follow_state
        elif self.enable_lidar_distance_follow and scan_age <= self.scan_timeout_s:
            speed_limit = self.distance_follow_speed_limit(self.overtake_front_distance_m)
            if speed > speed_limit:
                speed = speed_limit
                self.last_speed_mode = self.follow_state
        if self.yellow_lane_slowdown_active and speed > self.yellow_lane_slow_speed_cmd:
            speed = self.yellow_lane_slow_speed_cmd
            self.last_speed_mode = "yellow_lane_slow"
        self.drive(angle, speed)
        self.publish_lane_images(self.last_lane)
        self.publish_debug(debug_state, angle, speed, image_age, scan_age)

    def count_yellow_lane_pixels(self, frame: np.ndarray | None) -> tuple[int, int]:
        if frame is None or frame.size == 0:
            return 0, 0
        height, width = frame.shape[:2]
        y0 = int(max(0, min(height - 1, height * 0.56)))
        roi = frame[y0:height, 0:width]
        if roi.size == 0:
            return 0, 0
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(
            hsv,
            np.array([18, 70, 120], dtype=np.uint8),
            np.array([38, 255, 255], dtype=np.uint8),
        )
        left_edge = yellow_mask[:, : int(width * 0.35)]
        right_edge = yellow_mask[:, int(width * 0.65) :]
        total = int(np.count_nonzero(yellow_mask))
        edge_total = self.count_continuous_yellow_boundary_pixels(left_edge)
        edge_total += self.count_continuous_yellow_boundary_pixels(right_edge)
        return total, edge_total

    def count_continuous_yellow_boundary_pixels(self, mask: np.ndarray) -> int:
        if mask.size == 0:
            return 0
        component_count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, 8)
        pixels = 0
        for idx in range(1, component_count):
            _x, _y, _width, height, area = stats[idx]
            if height >= self.yellow_lane_min_component_height_px:
                pixels += int(area)
        return pixels

    def bev_status_text(self) -> str:
        def side_label(value: str) -> str:
            return {
                "left": "LEFT",
                "right": "RIGHT",
                "middle": "MIDDLE",
                "none": "-",
                "front": "FRONT",
            }.get(str(value), str(value).upper()[:8])

        left = self.last_headlight_by_camera["left"]
        right = self.last_headlight_by_camera["right"]
        mode = "TIMED" if self.enable_timed_start_sequence and not self.timed_start_done else "LANE"
        return (
            f"MODE:{mode} "
            f"SIG:{str(self.start_signal_state)[:5]} "
            f"AVOID:{side_label(self.lidar_avoid_target_side)} "
            f"BLOCK:{side_label(self.lidar_avoid_blocked_side)} "
            f"STATE:{self.lidar_avoid_state.upper()[:10]} "
            f"LANE:{self.lane_visibility_mode.upper()[:9]} "
            f"LOCK:{'Y' if self.lidar_avoid_locked_until_checkerboard else 'N'} "
            f"CHK:{'Y' if self.checkerboard_avoid_reset_detected else 'N'} "
            f"YEL:{'Y' if self.yellow_lane_slowdown_active else 'N'} "
            f"LCHK:{'Y' if self.lidar_avoid_left_headlight_confirmed else 'N'} "
            f"CAM:{side_label(self.lidar_avoid_verify_camera_side)} "
            f"HL:L{left.light_count}/P{left.pair_count} R{right.light_count}/P{right.pair_count}"
        )

    def front_lidar_distance(self, half_width_deg: float | None = None, max_distance_m: float = 20.0) -> float:
        if self.lidar_ranges is None or self.lidar_angle_increment == 0.0:
            return float("inf")
        front_angle = math.radians(self.lidar_front_center_deg)
        center = int(round((front_angle - self.lidar_angle_min) / self.lidar_angle_increment))
        half_width = self.lidar_front_half_width_deg if half_width_deg is None else float(half_width_deg)
        width = max(1, int(round(math.radians(max(0.1, half_width)) / abs(self.lidar_angle_increment))))
        count = len(self.lidar_ranges)
        values = [
            float(self.lidar_ranges[idx])
            for idx in self.front_lidar_indices(center, width, count)
            if self.is_valid_lidar_front_point_from_values(idx, float(self.lidar_ranges[idx]), max_distance_m)
        ]
        return min(values) if values else float("inf")

    def reset_lidar_lane_avoidance(self) -> None:
        self.lidar_avoid_target_side = "middle"
        self.lidar_avoid_front_blocked = False
        self.lidar_avoid_blocked_side = "none"
        self.lidar_avoid_left_clear_m = float("inf")
        self.lidar_avoid_right_clear_m = float("inf")
        self.lidar_avoid_verify_camera_side = "none"
        self.lidar_avoid_state = "idle"
        self.lidar_avoid_latched_target_side = "middle"
        self.lidar_avoid_latched_blocked_side = "none"
        self.lidar_avoid_latched_verify_camera_side = "none"
        self.lidar_avoid_release_camera_side = "none"

    def detect_checkerboard_avoid_reset(self, frame: np.ndarray | None) -> bool:
        if frame is None or frame.size == 0:
            return False
        h, w = frame.shape[:2]
        y0 = int(round(h * 0.42))
        y1 = int(round(h * 0.86))
        x0 = int(round(w * 0.04))
        x1 = int(round(w * 0.96))
        if y1 <= y0 or x1 <= x0:
            return False

        roi = frame[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(
            hsv,
            np.array([0, 0, 175], dtype=np.uint8),
            np.array([180, 80, 255], dtype=np.uint8),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        centers: list[tuple[int, int]] = []
        boxes: list[tuple[int, int, int, int]] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not (25.0 <= area <= 3500.0):
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 5 or bh < 5 or bw > 100 or bh > 70:
                continue
            aspect = bw / max(1.0, float(bh))
            if not (0.35 <= aspect <= 3.5):
                continue
            fill = area / max(1.0, float(bw * bh))
            if fill < 0.22:
                continue
            centers.append((x + bw // 2, y + bh // 2))
            boxes.append((x, y, bw, bh))

        if len(centers) < 8:
            return False
        xs = [x for x, _ in centers]
        ys = [y for _, y in centers]
        if max(xs) - min(xs) < 150 or max(ys) - min(ys) < 24:
            return False

        x_bins = {int(round(x / 24.0)) for x in xs}
        y_bins = {int(round(y / 18.0)) for y in ys}
        return len(x_bins) >= 4 and len(y_bins) >= 2

    def refresh_effective_headlight_selection(self) -> None:
        if self.lidar_avoid_verify_camera_side in {"left", "right"}:
            side = self.lidar_avoid_verify_camera_side
        elif self.headlight_camera_side.strip().lower() in {"left", "right"}:
            side = self.headlight_camera_side.strip().lower()
        else:
            side = "left"
        self.effective_headlight_camera_side = side
        self.effective_headlight_target_side = "strongest"
        self.last_headlight = self.last_headlight_by_camera[side]
        self.last_headlight_image_age = self.last_headlight_image_age_by_camera[side]

    def headlight_detected_on_camera(self, camera_side: str) -> bool:
        side = camera_side.strip().lower()
        if side not in {"left", "right"}:
            return False
        age = self.last_headlight_image_age_by_camera.get(side, float("inf"))
        return bool(self.last_headlight_by_camera[side].detected) and math.isfinite(age) and age <= self.headlight_image_timeout_s

    @staticmethod
    def verify_camera_for_target_side(target_side: str) -> str:
        if target_side == "left":
            return "right"
        if target_side == "right":
            return "left"
        return "none"

    def advance_lidar_avoid_on_headlight(self, target_side: str, blocked_side: str, verify_camera: str) -> bool:
        if not self.lidar_lane_avoid_release_on_headlight or not self.headlight_detected_on_camera(verify_camera):
            return False

        if verify_camera == "right" and not self.lidar_avoid_left_headlight_confirmed:
            return False

        if verify_camera == "left":
            self.lidar_avoid_left_headlight_confirmed = True

        if target_side == "right" and verify_camera == "left":
            self.lidar_avoid_state = "handoff_left_headlight"
            self.lidar_avoid_target_side = "left"
            self.lidar_avoid_blocked_side = "right"
            self.lidar_avoid_verify_camera_side = "right"
            self.lidar_avoid_release_camera_side = "left"
            self.lidar_avoid_latched_target_side = "left"
            self.lidar_avoid_latched_blocked_side = "right"
            self.lidar_avoid_latched_verify_camera_side = "right"
            return True

        self.lidar_avoid_state = "released_headlight"
        self.lidar_avoid_target_side = "middle"
        self.lidar_avoid_blocked_side = blocked_side
        self.lidar_avoid_verify_camera_side = verify_camera
        self.lidar_avoid_release_camera_side = verify_camera
        self.lidar_avoid_latched_target_side = "middle"
        self.lidar_avoid_latched_blocked_side = "none"
        self.lidar_avoid_latched_verify_camera_side = "none"
        return True

    def update_lidar_lane_avoidance(self) -> None:
        front_distance = self.front_lidar_distance(
            self.overtake_front_width_deg,
            max(self.lidar_lane_avoid_clear_distance_m, self.lidar_lane_avoid_trigger_distance_m),
        )
        self.lidar_avoid_front_blocked = front_distance <= self.lidar_lane_avoid_trigger_distance_m
        self.lidar_avoid_left_clear_m = self.lidar_lane_side_distance("left")
        self.lidar_avoid_right_clear_m = self.lidar_lane_side_distance("right")
        left_blocked = self.lidar_avoid_left_clear_m <= self.lidar_lane_avoid_trigger_distance_m
        right_blocked = self.lidar_avoid_right_clear_m <= self.lidar_lane_avoid_trigger_distance_m

        if self.lidar_avoid_latched_target_side in {"left", "right"}:
            verify_camera = self.lidar_avoid_latched_verify_camera_side
            if self.advance_lidar_avoid_on_headlight(
                self.lidar_avoid_latched_target_side,
                self.lidar_avoid_latched_blocked_side,
                verify_camera,
            ):
                return

            self.lidar_avoid_state = "latched"
            self.lidar_avoid_target_side = self.lidar_avoid_latched_target_side
            self.lidar_avoid_blocked_side = self.lidar_avoid_latched_blocked_side
            self.lidar_avoid_verify_camera_side = verify_camera
            self.lidar_avoid_release_camera_side = "none"
            return

        if self.lidar_avoid_locked_until_checkerboard:
            self.lidar_avoid_state = "locked_until_checkerboard"
            self.lidar_avoid_target_side = "middle"
            self.lidar_avoid_blocked_side = "none"
            self.lidar_avoid_verify_camera_side = "none"
            self.lidar_avoid_release_camera_side = "none"
            return

        raw_target_side = "middle"
        raw_blocked_side = "none"

        if left_blocked and not right_blocked:
            raw_blocked_side = "left"
            raw_target_side = "right"
        elif right_blocked and not left_blocked:
            raw_blocked_side = "right"
            raw_target_side = "left"
        if not self.lidar_avoid_front_blocked:
            pass
        elif raw_target_side == "middle":
            left_clear = self.lidar_front_side_distance("left")
            right_clear = self.lidar_front_side_distance("right")
            left_open = left_clear >= self.lidar_lane_avoid_trigger_distance_m or not math.isfinite(left_clear)
            right_open = right_clear >= self.lidar_lane_avoid_trigger_distance_m or not math.isfinite(right_clear)

            preferred = self.lidar_lane_avoid_preferred_side.strip().lower()
            if preferred not in {"left", "right"}:
                preferred = "left"

            if left_open and right_open:
                if math.isfinite(left_clear) and math.isfinite(right_clear) and abs(left_clear - right_clear) > 0.15:
                    raw_target_side = "left" if left_clear > right_clear else "right"
                else:
                    raw_target_side = preferred
            elif left_open:
                raw_target_side = "left"
            elif right_open:
                raw_target_side = "right"
            else:
                if math.isfinite(left_clear) and math.isfinite(right_clear):
                    raw_target_side = "left" if left_clear >= right_clear else "right"
                else:
                    raw_target_side = preferred
            raw_blocked_side = "right" if raw_target_side == "left" else "left"

        if raw_target_side in {"left", "right"}:
            verify_camera = self.verify_camera_for_target_side(raw_target_side)
            self.lidar_avoid_locked_until_checkerboard = True
            if self.advance_lidar_avoid_on_headlight(raw_target_side, raw_blocked_side, verify_camera):
                return

            self.lidar_avoid_state = "latched"
            self.lidar_avoid_target_side = raw_target_side
            self.lidar_avoid_blocked_side = raw_blocked_side
            self.lidar_avoid_verify_camera_side = verify_camera
            self.lidar_avoid_latched_target_side = raw_target_side
            self.lidar_avoid_latched_blocked_side = raw_blocked_side
            self.lidar_avoid_latched_verify_camera_side = verify_camera
            self.lidar_avoid_release_camera_side = "none"
            return

        self.lidar_avoid_state = "idle"
        self.lidar_avoid_target_side = "middle"
        self.lidar_avoid_blocked_side = "none"
        self.lidar_avoid_verify_camera_side = "none"
        self.lidar_avoid_release_camera_side = "none"

    def lidar_lane_side_distance(self, lane_side: str) -> float:
        if self.lidar_ranges is None or self.lidar_angle_increment == 0.0:
            return float("inf")
        side = lane_side.strip().lower()
        if side not in {"left", "right"}:
            return float("inf")

        front_angle = math.radians(self.lidar_front_center_deg)
        side_min = math.radians(max(0.0, self.lidar_lane_avoid_side_min_deg))
        side_max = math.radians(max(0.0, self.lidar_front_half_width_deg))
        positive_side = self.lidar_positive_angle_side.strip().lower()
        if positive_side not in {"left", "right"}:
            positive_side = "left"

        distances: list[float] = []
        for idx, distance in enumerate(self.lidar_ranges):
            distance = float(distance)
            if not self.is_valid_lidar_front_point_from_values(idx, distance, self.lidar_lane_avoid_clear_distance_m):
                continue
            relative = self.wrap_angle_pi(self.lidar_angle_min + idx * self.lidar_angle_increment - front_angle)
            abs_relative = abs(relative)
            if not (side_min <= abs_relative <= side_max):
                continue
            relative_side = positive_side if relative > 0.0 else ("right" if positive_side == "left" else "left")
            if relative_side != side:
                continue
            forward = distance * math.cos(relative)
            if forward > 0.0:
                distances.append(float(forward))
        return min(distances) if distances else float("inf")

    def lidar_front_side_distance(self, lane_side: str) -> float:
        if self.lidar_ranges is None or self.lidar_angle_increment == 0.0:
            return float("inf")
        side = lane_side.strip().lower()
        if side not in {"left", "right"}:
            return float("inf")

        front_angle = math.radians(self.lidar_front_center_deg)
        side_min = math.radians(max(0.0, self.lidar_lane_avoid_side_min_deg))
        side_max = math.radians(max(side_min, float(self.overtake_front_width_deg)))
        positive_side = self.lidar_positive_angle_side.strip().lower()
        if positive_side not in {"left", "right"}:
            positive_side = "left"

        max_distance_m = max(8.0, float(self.lidar_front_view_max_m))
        distances: list[float] = []
        for idx, distance in enumerate(self.lidar_ranges):
            distance = float(distance)
            if not self.is_valid_lidar_front_point_from_values(idx, distance, max_distance_m):
                continue
            relative = self.wrap_angle_pi(self.lidar_angle_min + idx * self.lidar_angle_increment - front_angle)
            abs_relative = abs(relative)
            if not (side_min <= abs_relative <= side_max):
                continue
            relative_side = positive_side if relative > 0.0 else ("right" if positive_side == "left" else "left")
            if relative_side != side:
                continue
            forward = distance * math.cos(relative)
            if forward > 0.0:
                distances.append(float(forward))
        return min(distances) if distances else float("inf")

    def resolve_headlight_target_side(self) -> str:
        mode = self.headlight_target_side.strip().lower()
        if mode in {"left", "right", "middle", "strongest"}:
            return mode
        if mode != "auto_lane":
            return "right"

        image_width = self.image.shape[1] if self.image is not None and self.image.ndim >= 2 else 640
        center_x = image_width * 0.5
        deadband = max(0.0, float(self.headlight_auto_lane_deadband_px))
        target_x = float(self.last_lane.target_x)
        if self.last_lane.confidence < self.lane_good_confidence:
            return "right"
        if target_x < center_x - deadband:
            return "right"
        if target_x > center_x + deadband:
            return "left"
        return "right"

    def resolve_headlight_camera_side(self) -> str:
        mode = self.headlight_camera_side.strip().lower()
        if mode in {"left", "right", "behind", "front"}:
            return mode
        if mode != "auto_lane":
            return "right"

        image_width = self.image.shape[1] if self.image is not None and self.image.ndim >= 2 else 640
        center_x = image_width * 0.5
        deadband = max(0.0, float(self.headlight_auto_lane_deadband_px))
        target_x = float(self.last_lane.target_x)
        if self.last_lane.confidence < self.lane_good_confidence:
            return "right"
        if target_x < center_x - deadband:
            return "right"
        if target_x > center_x + deadband:
            return "left"
        return "right"

    def headlight_roi_side_for_camera(self, camera_side: str) -> str:
        base = self.headlight_roi_x_side.strip().lower()
        if base not in {"left", "right"}:
            return base
        side = camera_side.strip().lower()
        if side == "right":
            return "left" if base == "right" else "right"
        return base

    def headlight_roi_y_for_camera(self, camera_side: str) -> tuple[float, float]:
        side = camera_side.strip().lower()
        if side == "left":
            return self.headlight_left_roi_y_min_ratio, self.headlight_left_roi_y_max_ratio
        if side == "right":
            return self.headlight_right_roi_y_min_ratio, self.headlight_right_roi_y_max_ratio
        return self.headlight_roi_y_min_ratio, self.headlight_roi_y_max_ratio

    def headlight_roi_geometry_for_camera(self, camera_side: str) -> tuple[float, float]:
        side = camera_side.strip().lower()
        if side == "left":
            return self.headlight_left_roi_side_inner_ratio, self.headlight_left_roi_side_outer_margin_ratio
        if side == "right":
            return self.headlight_right_roi_side_inner_ratio, self.headlight_right_roi_side_outer_margin_ratio
        return self.headlight_roi_side_inner_ratio, self.headlight_roi_side_outer_margin_ratio

    def current_headlight_image(self, camera_side: str, now: float, front_image_age: float) -> tuple[np.ndarray | None, float]:
        side = camera_side.strip().lower()
        if side == "front":
            return self.image, front_image_age
        image = self.headlight_images.get(side)
        image_time = self.headlight_image_times.get(side, 0.0)
        if image is None or image_time <= 0.0:
            return None, float("inf")
        return image, now - image_time

    def detect_headlight_camera(self, camera_side: str, now: float) -> tuple[HeadlightResult, float]:
        image, age = self.current_headlight_image(camera_side, now, float("inf"))
        if image is None or age > self.headlight_image_timeout_s:
            return HeadlightResult(), age
        detector = self.headlight_detectors.get(camera_side)
        roi_y_min, roi_y_max = self.headlight_roi_y_for_camera(camera_side)
        roi_inner, roi_outer = self.headlight_roi_geometry_for_camera(camera_side)
        if detector is None:
            detector = HeadlightDetector(
                image_width=image.shape[1],
                target_side="strongest",
                roi_x_side=self.headlight_roi_side_for_camera(camera_side),
                roi_y_min_ratio=roi_y_min,
                roi_y_max_ratio=roi_y_max,
                min_pair_y_ratio=roi_y_min,
                roi_side_inner_ratio=roi_inner,
                roi_side_outer_margin_ratio=roi_outer,
                min_pair_gap_px=self.headlight_min_pair_gap_px,
                max_pair_gap_px=self.headlight_max_pair_gap_px,
                pair_y_tolerance_px=self.headlight_pair_y_tolerance_px,
                min_height_px=self.headlight_min_height_px,
                max_height_px=self.headlight_max_height_px,
            )
            self.headlight_detectors[camera_side] = detector
        detector.roi_x_side = self.headlight_roi_side_for_camera(camera_side)
        detector.roi_y_min_ratio = roi_y_min
        detector.roi_y_max_ratio = roi_y_max
        detector.min_pair_y_ratio = roi_y_min
        detector.roi_side_inner_ratio = roi_inner
        detector.roi_side_outer_margin_ratio = roi_outer
        detector.min_pair_gap_px = self.headlight_min_pair_gap_px
        detector.max_pair_gap_px = self.headlight_max_pair_gap_px
        detector.pair_y_tolerance_px = self.headlight_pair_y_tolerance_px
        detector.min_height_px = self.headlight_min_height_px
        detector.max_height_px = self.headlight_max_height_px
        detector.target_side = "strongest"
        return detector.detect(image, make_debug=self.publish_debug_images), age

    @staticmethod
    def front_lidar_indices(center: int, width: int, count: int) -> list[int]:
        if count <= 0:
            return []
        return [int((center + offset) % count) for offset in range(-width, width + 1)]

    def front_lidar_points(self, msg: LaserScan, max_distance_m: float) -> list[tuple[int, float, float, float, float]]:
        if msg.angle_increment == 0.0 or len(msg.ranges) == 0:
            return []
        front_angle = math.radians(self.lidar_front_center_deg)
        center = int(round((front_angle - msg.angle_min) / msg.angle_increment))
        width = max(1, int(round(math.radians(self.lidar_front_half_width_deg) / abs(msg.angle_increment))))
        points: list[tuple[int, float, float, float, float]] = []
        for idx in self.front_lidar_indices(center, width, len(msg.ranges)):
            distance = float(msg.ranges[idx])
            if not (
                self.is_valid_lidar_front_point_from_values(idx, distance, max_distance_m)
                and max(msg.range_min, self.lidar_min_valid_distance_m) < distance < min(msg.range_max, max_distance_m)
            ):
                continue
            angle = msg.angle_min + idx * msg.angle_increment
            relative = self.wrap_angle_pi(angle - front_angle)
            forward = distance * math.cos(relative)
            lateral = distance * math.sin(relative)
            if forward > 0.0:
                points.append((idx, distance, relative, forward, lateral))
        return points

    @staticmethod
    def wrap_angle_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def is_valid_lidar_front_point(self, msg: LaserScan, idx: int, distance: float, max_distance_m: float) -> bool:
        if not math.isfinite(distance):
            return False
        if not (max(msg.range_min, self.lidar_min_valid_distance_m) < distance < min(msg.range_max, max_distance_m)):
            return False
        return self.is_valid_lidar_front_point_from_values(idx, distance, max_distance_m)

    def is_valid_lidar_front_point_from_values(self, idx: int, distance: float, max_distance_m: float) -> bool:
        if not math.isfinite(distance) or not (self.lidar_min_valid_distance_m < distance < max_distance_m):
            return False
        if self.lidar_angle_increment == 0.0:
            return True
        front_angle = math.radians(self.lidar_front_center_deg)
        relative_deg = math.degrees(
            self.wrap_angle_pi(self.lidar_angle_min + idx * self.lidar_angle_increment - front_angle)
        )
        if (
            self.lidar_ignore_self_reflection
            and self.lidar_self_reflect_min_rel_deg <= relative_deg <= self.lidar_self_reflect_max_rel_deg
            and distance <= self.lidar_self_reflect_max_distance_m
        ):
            return False
        return True

    def distance_hold_speed_cmd(self, front_distance_m: float) -> float:
        if not math.isfinite(front_distance_m):
            self.follow_state = "hold_no_target"
            self.follow_speed_limit_cmd = self.follow_no_target_speed_cmd
            return self.follow_speed_limit_cmd

        error = float(front_distance_m) - self.follow_target_distance_m
        if abs(error) <= self.follow_deadband_m:
            self.follow_state = "hold_keep"
            self.follow_speed_limit_cmd = 0.0
            return 0.0

        command = error * self.follow_kp_speed_cmd_per_m
        if command > 0.0:
            self.follow_state = "hold_forward"
            command = min(command, self.follow_forward_max_speed_cmd)
        else:
            self.follow_state = "hold_reverse"
            command = max(command, self.follow_reverse_speed_cmd)
        self.follow_speed_limit_cmd = float(command)
        return float(command)

    def distance_follow_speed_limit(self, front_distance_m: float) -> float:
        if not math.isfinite(front_distance_m):
            self.follow_state = "clear"
            self.follow_speed_limit_cmd = self.dynamic_speed_max_cmd
            return self.follow_speed_limit_cmd
        if front_distance_m <= self.follow_stop_distance_m:
            self.follow_state = "follow_stop"
            self.follow_speed_limit_cmd = 0.0
        elif front_distance_m <= self.follow_crawl_distance_m:
            self.follow_state = "follow_crawl"
            self.follow_speed_limit_cmd = self.follow_crawl_speed_cmd
        elif front_distance_m <= self.follow_slow_distance_m:
            self.follow_state = "follow_slow"
            self.follow_speed_limit_cmd = self.follow_slow_speed_cmd
        elif front_distance_m <= self.follow_clear_distance_m:
            self.follow_state = "follow_match"
            self.follow_speed_limit_cmd = self.follow_mid_speed_cmd
        else:
            self.follow_state = "clear"
            self.follow_speed_limit_cmd = self.dynamic_speed_max_cmd
        return self.follow_speed_limit_cmd

    def update_overtake_state(self, now: float, front_distance_m: float) -> None:
        vehicle_ahead = 0.1 < front_distance_m <= self.overtake_trigger_distance_m
        clear_ahead = front_distance_m >= self.overtake_clear_distance_m or not math.isfinite(front_distance_m)

        if self.overtake_state == "idle":
            if vehicle_ahead:
                self.overtake_state = "passing"
                self.overtake_until = now + self.overtake_min_hold_s
                self.overtake_clear_since = 0.0
        elif self.overtake_state == "passing":
            if vehicle_ahead:
                self.overtake_clear_since = 0.0
                self.overtake_until = max(self.overtake_until, now + self.overtake_min_hold_s)
            elif clear_ahead:
                if self.overtake_clear_since <= 0.0:
                    self.overtake_clear_since = now
                if now >= self.overtake_until and now - self.overtake_clear_since >= self.overtake_clear_hold_s:
                    self.overtake_state = "returning"
                    self.overtake_return_until = now + max(0.1, self.overtake_return_s)
            else:
                self.overtake_clear_since = 0.0
        elif self.overtake_state == "returning":
            if vehicle_ahead:
                self.overtake_state = "passing"
                self.overtake_until = now + self.overtake_min_hold_s
                self.overtake_clear_since = 0.0
            elif now >= self.overtake_return_until:
                self.overtake_state = "idle"
                self.overtake_clear_since = 0.0

    def current_overtake_offset(self, now: float) -> int:
        if self.overtake_state == "passing":
            self.overtake_target_offset_px = self.overtake_offset_px
        elif self.overtake_state == "returning":
            duration = max(0.1, self.overtake_return_s)
            remaining = max(0.0, self.overtake_return_until - now)
            self.overtake_target_offset_px = int(round(self.overtake_offset_px * min(1.0, remaining / duration)))
        else:
            self.overtake_target_offset_px = 0
        return self.overtake_target_offset_px

    def select_lane_speed(self, lane: LaneResult, raw_angle_cmd: float) -> float:
        straight_candidate = (
            lane.confidence >= 0.75
            and lane.curve_score <= self.straight_curve_score_max_px
            and abs(lane.angle_error) <= self.straight_angle_error_max_deg
            and abs(raw_angle_cmd) <= self.straight_raw_angle_max_cmd
        )
        if straight_candidate:
            self.straight_frame_count += 1
        else:
            self.straight_frame_count = 0

        if self.straight_frame_count >= self.straight_stable_frames:
            self.last_speed_mode = "straight_fast"
            speed = min(self.straight_fast_speed_cmd, self.dynamic_speed_max_cmd)
            self.last_dynamic_curve_score = 0.0
            self.last_dynamic_curvature = 0.0
            self.last_dynamic_speed_mps = speed * self.speed_cmd_to_mps
            return speed

        return self.dynamic_curve_speed(lane, raw_angle_cmd)

    def dynamic_curve_speed(self, lane: LaneResult, raw_angle_cmd: float) -> float:
        effective_score = max(
            float(lane.curve_score),
            abs(float(lane.angle_error)) * self.dynamic_angle_error_to_curve_score,
            abs(float(raw_angle_cmd)) * self.dynamic_raw_angle_to_curve_score,
        )
        excess_score = max(0.0, effective_score - self.dynamic_curve_score_deadband_px)
        curvature = excess_score * self.dynamic_curve_score_to_curvature
        if curvature <= 1e-6:
            speed_cmd = min(self.caution_speed_cmd, self.dynamic_speed_max_cmd)
            speed_mps = speed_cmd * self.speed_cmd_to_mps
        else:
            speed_mps = math.sqrt(max(0.0, self.dynamic_lateral_accel_limit_mps2) / curvature)
            speed_cmd = speed_mps / max(1e-6, self.speed_cmd_to_mps)
            speed_cmd = max(self.dynamic_speed_min_cmd, min(self.dynamic_speed_max_cmd, speed_cmd))
            speed_mps = speed_cmd * self.speed_cmd_to_mps

        self.last_dynamic_curve_score = effective_score
        self.last_dynamic_curvature = curvature
        self.last_dynamic_speed_mps = speed_mps
        if speed_cmd <= self.dynamic_speed_min_cmd + 1.0:
            self.last_speed_mode = "sharp_curve"
        else:
            self.last_speed_mode = "dynamic_curve"
        return float(speed_cmd)

    def publish_lane_images(self, lane: LaneResult) -> None:
        if not self.publish_debug_images:
            return
        if lane.debug_image is not None:
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(lane.debug_image, encoding="bgr8"))
        if lane.bev_image is not None:
            self.bev_image_pub.publish(self.bridge.cv2_to_imgmsg(lane.bev_image, encoding="bgr8"))
        if lane.mask_image is not None:
            self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(lane.mask_image, encoding="bgr8"))

    def publish_debug(self, state: str, angle: float, speed: float, image_age: float, scan_age: float) -> None:
        front = self.last_obstacle.front_distance_m
        cone_dbg = self.last_cone_cmd.debug
        cone_left = cone_dbg.get("left_cone")
        cone_right = cone_dbg.get("right_cone")
        try:
            cone_front_dist = float(cone_dbg.get("front_dist", float("inf")))
        except (TypeError, ValueError):
            cone_front_dist = float("inf")
        payload = {
            "controller_state": state,
            "lane_state": self.last_lane.state,
            "lane_confidence": round(float(self.last_lane.confidence), 3),
            "lane_target_x": int(self.last_lane.target_x),
            "lane_angle_error": round(float(self.last_lane.angle_error), 3),
            "lane_curve_score": round(float(self.last_lane.curve_score), 3),
            "lane_left_count": int(self.last_lane.left_count),
            "lane_mid_count": int(self.last_lane.mid_count),
            "lane_right_count": int(self.last_lane.right_count),
            "lane_has_left": bool(self.last_lane.left_count > 0),
            "lane_has_right": bool(self.last_lane.right_count > 0),
            "speed_mode": self.last_speed_mode,
            "cone_mode_enabled": bool(self.enable_cone_mode),
            "cone_mode_active": bool(self.cone_mode_active),
            "cone_finished": bool(self.cone_finished),
            "cone_cmd_finished": bool(self.last_cone_cmd.finished),
            "cone_mode": str(cone_dbg.get("mode", "none")),
            "cone_follow_mode": str(cone_dbg.get("follow_mode", "none")),
            "cone_steer_mode": str(cone_dbg.get("steer_mode", "none")),
            "cone_lost_frames": int(cone_dbg.get("lost_frames", 0)),
            "cone_tracked_frames": int(cone_dbg.get("tracked_frames", 0)),
            "cone_path_count": int(cone_dbg.get("path_count", 0)),
            "cone_center_offset": round(float(cone_dbg.get("center_offset", 0.0)), 3),
            "cone_front_dist": round(cone_front_dist, 3) if math.isfinite(cone_front_dist) else None,
            "cone_path_cx": round(float(cone_dbg.get("path_cx", 0.0)), 3),
            "cone_path_cy": round(float(cone_dbg.get("path_cy", 0.0)), 3),
            "cone_left_x": round(float(getattr(cone_left, "cx", 0.0)), 3) if cone_left else None,
            "cone_left_y": round(float(getattr(cone_left, "cy", 0.0)), 3) if cone_left else None,
            "cone_right_x": round(float(getattr(cone_right, "cx", 0.0)), 3) if cone_right else None,
            "cone_right_y": round(float(getattr(cone_right, "cy", 0.0)), 3) if cone_right else None,
            "cone_right_blocked": bool(cone_dbg.get("right_blocked", False)),
            "start_signal_gate_enabled": bool(self.enable_start_signal_gate),
            "start_signal": self.start_signal_state,
            "start_signal_gate_open": bool(self.start_signal_gate_open),
            "start_signal_released": bool(self.start_signal_released),
            "start_signal_red_score": round(
                float(self.start_signal_scores.get(TrafficLightDetector.START_RED, 0.0)), 3
            ),
            "start_signal_yellow_score": round(
                float(self.start_signal_scores.get(TrafficLightDetector.START_YELLOW, 0.0)), 3
            ),
            "start_signal_green_score": round(
                float(self.start_signal_scores.get(TrafficLightDetector.START_GREEN, 0.0)), 3
            ),
            "timed_start_enabled": bool(self.enable_timed_start_sequence),
            "timed_start_phase": self.timed_start_phase,
            "timed_start_done": bool(self.timed_start_done),
            "timed_start_straight_s": round(float(self.timed_start_straight_s), 3),
            "timed_start_yaw_s": round(float(self.timed_start_yaw_s), 3),
            "timed_start_elapsed_s": round(float(time.monotonic() - self.timed_start_begin_time), 3)
            if self.timed_start_begin_time
            else 0.0,
            "timed_start_yaw_angle_cmd": round(float(self.timed_start_yaw_angle_cmd), 3),
            "timed_start_yaw0_deg": round(float(self.timed_start_yaw0_deg), 3)
            if self.timed_start_yaw0_deg is not None
            else None,
            "yellow_lane_pixels": int(self.yellow_lane_pixels),
            "yellow_lane_edge_pixels": int(self.yellow_lane_edge_pixels),
            "yellow_lane_slowdown": bool(self.yellow_lane_slowdown_active),
            "yellow_lane_slow_min_pixels": int(self.yellow_lane_slow_min_pixels),
            "yellow_lane_slow_edge_min_pixels": int(self.yellow_lane_slow_edge_min_pixels),
            "yellow_lane_min_component_height_px": int(self.yellow_lane_min_component_height_px),
            "yellow_lane_slow_speed_cmd": round(float(self.yellow_lane_slow_speed_cmd), 3),
            "dynamic_curve_score": round(float(self.last_dynamic_curve_score), 3),
            "dynamic_curvature": round(float(self.last_dynamic_curvature), 6),
            "dynamic_speed_mps": round(float(self.last_dynamic_speed_mps), 3),
            "obstacle_direction": self.last_obstacle.direction,
            "front_distance_m": round(float(front), 3) if math.isfinite(front) else None,
            "overtake_state": self.overtake_state,
            "overtake_offset_px": int(self.overtake_target_offset_px),
            "overtake_front_distance_m": round(float(self.overtake_front_distance_m), 3)
            if math.isfinite(self.overtake_front_distance_m)
            else None,
            "follow_state": self.follow_state,
            "follow_speed_limit_cmd": round(float(self.follow_speed_limit_cmd), 3),
            "follow_target_distance_m": round(float(self.follow_target_distance_m), 3),
            "lidar_front_center_deg": round(float(self.lidar_front_center_deg), 3),
            "lidar_front_half_width_deg": round(float(self.lidar_front_half_width_deg), 3),
            "lidar_avoid_state": self.lidar_avoid_state,
            "lidar_avoid_front_blocked": bool(self.lidar_avoid_front_blocked),
            "lidar_avoid_blocked_side": self.lidar_avoid_blocked_side,
            "lidar_avoid_target_side": self.lidar_avoid_target_side,
            "lidar_avoid_verify_camera_side": self.lidar_avoid_verify_camera_side,
            "lidar_avoid_latched_target_side": self.lidar_avoid_latched_target_side,
            "lidar_avoid_release_camera_side": self.lidar_avoid_release_camera_side,
            "lidar_avoid_locked_until_checkerboard": bool(self.lidar_avoid_locked_until_checkerboard),
            "lidar_avoid_left_headlight_confirmed": bool(self.lidar_avoid_left_headlight_confirmed),
            "checkerboard_avoid_reset_detected": bool(self.checkerboard_avoid_reset_detected),
            "lidar_avoid_left_clear_m": round(float(self.lidar_avoid_left_clear_m), 3)
            if math.isfinite(self.lidar_avoid_left_clear_m)
            else None,
            "lidar_avoid_right_clear_m": round(float(self.lidar_avoid_right_clear_m), 3)
            if math.isfinite(self.lidar_avoid_right_clear_m)
            else None,
            "object_detected": bool(self.last_object.detected),
            "object_direction": self.last_object.direction,
            "headlight_detected": bool(self.last_headlight.detected),
            "headlight_camera_side": self.headlight_camera_side,
            "headlight_effective_camera_side": self.effective_headlight_camera_side,
            "headlight_image_age_s": round(float(self.last_headlight_image_age), 3)
            if math.isfinite(self.last_headlight_image_age)
            else None,
            "headlight_target_side": self.headlight_target_side,
            "headlight_roi_x_side": self.headlight_roi_x_side,
            "headlight_left_roi_x_side": self.headlight_left_roi_x_side,
            "headlight_right_roi_x_side": self.headlight_right_roi_x_side,
            "headlight_left_roi_y_min": round(float(self.headlight_left_roi_y_min_ratio), 3),
            "headlight_left_roi_y_max": round(float(self.headlight_left_roi_y_max_ratio), 3),
            "headlight_right_roi_y_min": round(float(self.headlight_right_roi_y_min_ratio), 3),
            "headlight_right_roi_y_max": round(float(self.headlight_right_roi_y_max_ratio), 3),
            "headlight_left_roi_inner": round(float(self.headlight_left_roi_side_inner_ratio), 3),
            "headlight_left_roi_outer": round(float(self.headlight_left_roi_side_outer_margin_ratio), 3),
            "headlight_right_roi_inner": round(float(self.headlight_right_roi_side_inner_ratio), 3),
            "headlight_right_roi_outer": round(float(self.headlight_right_roi_side_outer_margin_ratio), 3),
            "headlight_min_pair_gap_px": round(float(self.headlight_min_pair_gap_px), 3),
            "headlight_max_pair_gap_px": round(float(self.headlight_max_pair_gap_px), 3),
            "headlight_pair_y_tolerance_px": round(float(self.headlight_pair_y_tolerance_px), 3),
            "headlight_min_height_px": round(float(self.headlight_min_height_px), 3),
            "headlight_max_height_px": round(float(self.headlight_max_height_px), 3),
            "headlight_effective_target_side": self.effective_headlight_target_side,
            "headlight_direction": self.last_headlight.direction,
            "headlight_light_count": int(self.last_headlight.light_count),
            "headlight_pair_count": int(self.last_headlight.pair_count),
            "headlight_x_mid": round(float(self.last_headlight.x_mid), 3),
            "headlight_y_mid": round(float(self.last_headlight.y_mid), 3),
            "headlight_pair_width_px": round(float(self.last_headlight.pair_width_px), 3),
            "headlight_left_detected": bool(self.last_headlight_by_camera["left"].detected),
            "headlight_left_age_s": round(float(self.last_headlight_image_age_by_camera["left"]), 3)
            if math.isfinite(self.last_headlight_image_age_by_camera["left"])
            else None,
            "headlight_left_light_count": int(self.last_headlight_by_camera["left"].light_count),
            "headlight_left_pair_count": int(self.last_headlight_by_camera["left"].pair_count),
            "headlight_left_x_mid": round(float(self.last_headlight_by_camera["left"].x_mid), 3),
            "headlight_left_pair_width_px": round(
                float(self.last_headlight_by_camera["left"].pair_width_px), 3
            ),
            "headlight_right_detected": bool(self.last_headlight_by_camera["right"].detected),
            "headlight_right_age_s": round(float(self.last_headlight_image_age_by_camera["right"]), 3)
            if math.isfinite(self.last_headlight_image_age_by_camera["right"])
            else None,
            "headlight_right_light_count": int(self.last_headlight_by_camera["right"].light_count),
            "headlight_right_pair_count": int(self.last_headlight_by_camera["right"].pair_count),
            "headlight_right_x_mid": round(float(self.last_headlight_by_camera["right"].x_mid), 3),
            "headlight_right_pair_width_px": round(
                float(self.last_headlight_by_camera["right"].pair_width_px), 3
            ),
            "xycar_angle_cmd": round(float(angle), 3),
            "raw_angle_cmd": round(float(self.lane_controller.last_raw_angle_cmd), 3),
            "steering_gain": round(float(self.lane_controller.last_steering_gain), 3),
            "estimated_steering_deg": round(float(self.lane_controller.current_steering_deg), 3),
            "desired_steering_deg": round(float(self.lane_controller.last_desired_steering_deg), 3),
            "target_steering_deg": round(float(self.lane_controller.last_target_steering_needed_deg), 3),
            "steering_horizon_s": round(float(self.lane_controller.steering_prediction_horizon_s), 3),
            "xycar_speed_cmd": round(float(speed), 3),
            "speed_kmh_est": round(float(speed) * self.speed_cmd_to_mps * 3.6, 3),
            "image_age_s": round(float(image_age), 3) if math.isfinite(image_age) else None,
            "scan_age_s": round(float(scan_age), 3) if math.isfinite(scan_age) else None,
            "yaw_rate": round(float(self.yaw_rate), 4),
            "imu_yaw_deg": round(float(self.current_yaw_deg), 3) if self.current_yaw_deg is not None else None,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.debug_pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self.drive(0.0, 0.0)
        except Exception:
            pass
        cv2.destroyAllWindows()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TrackDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

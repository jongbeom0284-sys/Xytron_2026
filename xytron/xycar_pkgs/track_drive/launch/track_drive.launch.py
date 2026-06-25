from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="track_drive",
                executable="track_drive",
                name="driver",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/usb_cam/image_raw/front",
                        "scan_topic": "/scan",
                        "imu_topic": "/imu",
                        "motor_topic": "xycar_motor",
                        "enable_start_signal_gate": False,
                        "start_signal_release_latched": True,
                        "enable_stopline": False,
                        "enable_lidar_obstacle": False,
                        "enable_object_detector": True,
                        "front_stop_distance_m": 0.0,
                        "front_slow_distance_m": 0.0,
                        "lane_dropout_hold_s": 0.0,
                        "lane_dropout_speed_cmd": 8.0,
                        "lane_good_confidence": 0.35,
                        "straight_fast_speed_cmd": 30.0,
                        "caution_speed_cmd": 12.0,
                        "dynamic_speed_min_cmd": 12.0,
                        "dynamic_speed_max_cmd": 30.0,
                        "dynamic_lateral_accel_limit_mps2": 1.6,
                        "dynamic_curve_score_deadband_px": 20.0,
                        "dynamic_curve_score_to_curvature": 0.0015,
                        "dynamic_angle_error_to_curve_score": 4.0,
                        "dynamic_raw_angle_to_curve_score": 4.0,
                        "straight_stable_frames": 3,
                        "straight_curve_score_max_px": 30.0,
                        "straight_angle_error_max_deg": 18.0,
                        "straight_raw_angle_max_cmd": 24.0,
                        "base_speed_cmd": 8.0,
                        "curve_speed_cmd": 12.0,
                        "slow_speed_cmd": 8.0,
                        "speed_cmd_to_mps": 0.5,
                        "compensate_steering_delay": True,
                        "steering_prediction_horizon_s": 0.16,
                        "steering_bias_deg": 0.0,
                        "lane_threshold_mode": "auturbo_white_hsv",
                        "white_s_max": 35,
                        "white_v_min": 190,
                        "lane_target_lookahead_y_px": 65,
                    }
                ],
            )
        ]
    )

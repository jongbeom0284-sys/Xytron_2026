"""라바콘 통로 주행 독립 ROS 2 노드.

2D LaserScan(/scan)을 수신하여 라바콘 통로 중심을 추종하는 조향 및 속도 프리뷰를 발행한다.
안전 규칙에 따라 하드웨어 모터 제어 토픽을 직접 발행하지 않으며,
브리지 노드(twinmixing_center_drive_bridge)와 연동하여 실차 주행을 수행한다.
"""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from .cone_corridor import (
    ConeCorridorConfig,
    ConeCorridorResult,
    EMPTY_CORRIDOR_RESULT,
    analyze_cone_corridor,
)
from .cone_path_guard import ConePathContinuityGuard, ConePathGuardConfig
from .drive_command import limit_forward_speed
from .geometry import clamp
from .obstacle_lidar import LidarSectorConfig


class ConeCorridorDriveNode(Node):
    """라바콘 단독 주행 판단 노드."""

    def __init__(self) -> None:
        super().__init__("cone_corridor_drive")
        self._declare_parameters()

        self._config = self._build_cone_config()
        self._lidar_config = self._build_lidar_config()
        self._path_guard = self._build_path_guard()
        self._filtered_lookahead_m = self._config.target_lookahead_m
        self._lookahead_rate_mps = float(
            self.get_parameter("cone_lookahead_rate_mps").value
        )

        self._last_scan_time = 0.0
        self._last_control_time = 0.0
        self._last_log_time = 0.0
        self._last_result = EMPTY_CORRIDOR_RESULT
        self._filtered_steering_rad = 0.0

        # 토픽 설정
        lidar_topic = str(self.get_parameter("lidar_topic").value)
        preview_topic = str(self.get_parameter("drive_preview_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        enable_topic = str(self.get_parameter("enable_topic").value)

        self._preview_publisher = self.create_publisher(
            Float32MultiArray, preview_topic, 10
        )
        self._status_publisher = self.create_publisher(String, status_topic, 10)

        enable_qos = QoSProfile(depth=1)
        enable_qos.reliability = ReliabilityPolicy.RELIABLE
        enable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._enable_publisher = self.create_publisher(Bool, enable_topic, enable_qos)
        if bool(self.get_parameter("auto_enable").value):
            self._enable_publisher.publish(Bool(data=True))

        self.create_subscription(
            LaserScan,
            lidar_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"ConeCorridorDriveNode initialized. Subscribed to {lidar_topic}, "
            f"publishing preview to {preview_topic}"
        )

    def _declare_parameters(self) -> None:
        # 사람의 enable 신호가 있어야 실차가 움직이는 것이 기본 계약이다.
        # 자동 출발은 launch에서 명시적으로 opt-in 할 때만 허용한다.
        self.declare_parameter("auto_enable", False)
        self.declare_parameter(
            "enable_topic", "/kobot_lane_drive/record_drive/enabled"
        )
        self.declare_parameter("lidar_topic", "/scan")
        self.declare_parameter(
            "drive_preview_topic", "/kobot_lane_drive/cone_corridor/drive_preview"
        )
        self.declare_parameter(
            "status_topic", "/kobot_lane_drive/cone_corridor/status"
        )

        # 라바콘 기하 및 탐색 설정
        self.declare_parameter("cone_corridor_width_m", 0.80)
        self.declare_parameter("cone_single_wall_fallback_margin_m", 0.40)
        self.declare_parameter("cone_corridor_width_min_m", 0.55)
        self.declare_parameter("cone_corridor_width_max_m", 0.95)
        self.declare_parameter("cone_forward_min_m", 0.25)
        self.declare_parameter("cone_forward_max_m", 2.00)
        self.declare_parameter("cone_lateral_max_m", 0.75)
        self.declare_parameter("cone_cluster_distance_tolerance_m", 0.12)
        self.declare_parameter("cone_min_cluster_points", 3)
        self.declare_parameter("cone_max_cluster_points", 25)
        self.declare_parameter("cone_min_cluster_span_m", 0.03)
        self.declare_parameter("cone_max_cluster_span_m", 0.25)
        self.declare_parameter("cone_max_link_distance_m", 0.50)
        self.declare_parameter("cone_far_max_link_distance_m", 0.50)
        self.declare_parameter("cone_link_distance_far_start_m", 0.30)
        self.declare_parameter("cone_link_distance_far_full_m", 0.75)
        self.declare_parameter("cone_pair_max_longitudinal_offset_m", 0.38)
        self.declare_parameter("cone_pair_root_max_center_offset_m", 0.25)
        self.declare_parameter("cone_pair_root_min_side_x_m", 0.05)
        self.declare_parameter("cone_pair_root_width_tolerance_m", 0.15)
        self.declare_parameter("cone_boundary_root_lateral_tolerance_m", 0.25)
        self.declare_parameter("cone_boundary_max_heading_change_rad", 1.05)
        self.declare_parameter("cone_min_boundary_cones", 2)
        self.declare_parameter("cone_min_paired_boundary_pairs", 2)
        self.declare_parameter("cone_root_max_y_m", 0.75)
        self.declare_parameter("cone_boundary_extrapolation_m", 0.25)
        self.declare_parameter("cone_slice_step_m", 0.15)
        self.declare_parameter("cone_target_lookahead_m", 0.48)
        self.declare_parameter("cone_corner_lookahead_m", 0.50)
        self.declare_parameter("cone_target_local_window_m", 0.15)
        self.declare_parameter("cone_lookahead_bend_start_m", 0.08)
        self.declare_parameter("cone_lookahead_bend_full_m", 0.28)
        self.declare_parameter("cone_lookahead_near_min_m", 0.30)
        self.declare_parameter("cone_lookahead_near_max_m", 0.55)
        self.declare_parameter("cone_lookahead_far_min_m", 0.75)
        self.declare_parameter("cone_lookahead_far_max_m", 1.00)
        self.declare_parameter("cone_lookahead_curvature_start_1pm", 0.15)
        self.declare_parameter("cone_lookahead_curvature_full_1pm", 0.90)
        self.declare_parameter("cone_lookahead_rate_mps", 0.40)
        self.declare_parameter("cone_polynomial_fit_min_y_m", 0.25)
        self.declare_parameter("cone_polynomial_fit_max_y_m", 1.50)
        self.declare_parameter("cone_polynomial_fit_min_points", 5)
        self.declare_parameter("cone_polynomial_fit_outlier_tolerance_m", 0.15)
        self.declare_parameter("cone_polynomial_fit_maximum_rmse_m", 0.08)
        self.declare_parameter("cone_lidar_to_rear_axle_m", 0.0)
        self.declare_parameter("cone_curve_outward_bias_max_m", 0.0)
        self.declare_parameter("cone_curve_outward_bias_left_max_m", -1.0)
        self.declare_parameter("cone_curve_outward_bias_right_max_m", -1.0)
        self.declare_parameter("cone_curve_outward_bias_start_ratio", 0.25)
        self.declare_parameter("cone_curve_outward_bias_full_ratio", 0.80)
        self.declare_parameter("cone_vehicle_width_m", 0.35)
        self.declare_parameter("cone_minimum_boundary_clearance_m", 0.05)
        self.declare_parameter("cone_drive_speed_mps", 0.25)
        self.declare_parameter("cone_corner_speed_mps", 0.20)
        self.declare_parameter("cone_fallback_speed_mps", 0.25)
        # 직선/코너/단측/hold를 전부 덮는 최종 planner 명령 상한. launch의
        # max_speed_mps가 bridge의 같은 이름과 동시에 이 값을 덮어쓴다.
        self.declare_parameter("max_speed_mps", 1.50)
        self.declare_parameter("cone_detection_min_slices", 3)
        self.declare_parameter("cone_detection_min_total_points", 6)
        self.declare_parameter("cone_steering_filter_alpha", 0.60)
        self.declare_parameter("cone_max_steering_rate_rad_s", 2.5)
        self.declare_parameter("cone_steering_gain", 2.2)
        self.declare_parameter("cone_heading_gain", 0.35)
        self.declare_parameter("cone_path_max_target_jump_m", 0.18)
        self.declare_parameter("cone_path_max_steering_jump_rad", 0.20)
        self.declare_parameter("cone_path_hold_sec", 0.35)
        self.declare_parameter("cone_path_hold_speed_mps", 0.50)
        self.declare_parameter("cone_path_sharp_hold_steering_rad", 0.349)
        self.declare_parameter("cone_path_sharp_hold_sec", 0.12)
        self.declare_parameter("cone_path_sharp_hold_speed_mps", 0.30)
        self.declare_parameter("cone_path_stop_on_lost", True)
        self.declare_parameter("cone_path_transition_target_rate_mps", 0.80)
        self.declare_parameter("cone_path_transition_steering_rate_rad_s", 2.00)
        self.declare_parameter("cone_path_pending_target_step_m", 0.05)
        self.declare_parameter("cone_path_pending_steering_step_rad", 0.10)
        self.declare_parameter("cone_path_candidate_confirmation_frames", 2)
        self.declare_parameter("cone_path_candidate_match_target_m", 0.08)
        self.declare_parameter("cone_path_candidate_match_steering_rad", 0.10)
        self.declare_parameter("cone_path_profile_min_overlap_slices", 3)
        self.declare_parameter("cone_path_profile_median_tolerance_m", 0.12)
        self.declare_parameter("cone_path_profile_max_tolerance_m", 0.20)

        # 라이다 각도 오프셋
        self.declare_parameter("lidar_angle_offset_rad", -2.1217)

    def _build_cone_config(self) -> ConeCorridorConfig:
        return ConeCorridorConfig(
            corridor_width_m=float(self.get_parameter("cone_corridor_width_m").value),
            single_wall_fallback_margin_m=float(
                self.get_parameter("cone_single_wall_fallback_margin_m").value
            ),
            corridor_width_min_m=float(
                self.get_parameter("cone_corridor_width_min_m").value
            ),
            corridor_width_max_m=float(
                self.get_parameter("cone_corridor_width_max_m").value
            ),
            forward_min_m=float(self.get_parameter("cone_forward_min_m").value),
            forward_max_m=float(self.get_parameter("cone_forward_max_m").value),
            lateral_max_m=float(self.get_parameter("cone_lateral_max_m").value),
            cluster_distance_tolerance_m=float(
                self.get_parameter("cone_cluster_distance_tolerance_m").value
            ),
            cone_min_cluster_points=int(
                self.get_parameter("cone_min_cluster_points").value
            ),
            cone_max_cluster_points=int(
                self.get_parameter("cone_max_cluster_points").value
            ),
            cone_min_cluster_span_m=float(
                self.get_parameter("cone_min_cluster_span_m").value
            ),
            cone_max_cluster_span_m=float(
                self.get_parameter("cone_max_cluster_span_m").value
            ),
            max_cone_link_distance_m=float(
                self.get_parameter("cone_max_link_distance_m").value
            ),
            far_max_cone_link_distance_m=float(
                self.get_parameter("cone_far_max_link_distance_m").value
            ),
            link_distance_far_start_m=float(
                self.get_parameter("cone_link_distance_far_start_m").value
            ),
            link_distance_far_full_m=float(
                self.get_parameter("cone_link_distance_far_full_m").value
            ),
            pair_maximum_longitudinal_offset_m=float(
                self.get_parameter("cone_pair_max_longitudinal_offset_m").value
            ),
            pair_root_maximum_center_offset_m=float(
                self.get_parameter("cone_pair_root_max_center_offset_m").value
            ),
            pair_root_minimum_side_x_m=float(
                self.get_parameter("cone_pair_root_min_side_x_m").value
            ),
            pair_root_width_tolerance_m=float(
                self.get_parameter("cone_pair_root_width_tolerance_m").value
            ),
            boundary_root_lateral_tolerance_m=float(
                self.get_parameter("cone_boundary_root_lateral_tolerance_m").value
            ),
            boundary_maximum_heading_change_rad=float(
                self.get_parameter("cone_boundary_max_heading_change_rad").value
            ),
            minimum_boundary_cones=int(
                self.get_parameter("cone_min_boundary_cones").value
            ),
            minimum_paired_boundary_pairs=int(
                self.get_parameter("cone_min_paired_boundary_pairs").value
            ),
            root_maximum_y_m=float(
                self.get_parameter("cone_root_max_y_m").value
            ),
            boundary_extrapolation_m=float(
                self.get_parameter("cone_boundary_extrapolation_m").value
            ),
            slice_step_m=float(self.get_parameter("cone_slice_step_m").value),
            target_lookahead_m=float(
                self.get_parameter("cone_target_lookahead_m").value
            ),
            corner_target_lookahead_m=float(
                self.get_parameter("cone_corner_lookahead_m").value
            ),
            target_local_window_m=float(
                self.get_parameter("cone_target_local_window_m").value
            ),
            lookahead_bend_start_m=float(
                self.get_parameter("cone_lookahead_bend_start_m").value
            ),
            lookahead_bend_full_m=float(
                self.get_parameter("cone_lookahead_bend_full_m").value
            ),
            lookahead_near_min_m=float(
                self.get_parameter("cone_lookahead_near_min_m").value
            ),
            lookahead_near_max_m=float(
                self.get_parameter("cone_lookahead_near_max_m").value
            ),
            lookahead_far_min_m=float(
                self.get_parameter("cone_lookahead_far_min_m").value
            ),
            lookahead_far_max_m=float(
                self.get_parameter("cone_lookahead_far_max_m").value
            ),
            lookahead_curvature_start_1pm=float(
                self.get_parameter("cone_lookahead_curvature_start_1pm").value
            ),
            lookahead_curvature_full_1pm=float(
                self.get_parameter("cone_lookahead_curvature_full_1pm").value
            ),
            polynomial_fit_min_y_m=float(
                self.get_parameter("cone_polynomial_fit_min_y_m").value
            ),
            polynomial_fit_max_y_m=float(
                self.get_parameter("cone_polynomial_fit_max_y_m").value
            ),
            polynomial_fit_min_points=int(
                self.get_parameter("cone_polynomial_fit_min_points").value
            ),
            polynomial_fit_outlier_tolerance_m=float(
                self.get_parameter("cone_polynomial_fit_outlier_tolerance_m").value
            ),
            polynomial_fit_maximum_rmse_m=float(
                self.get_parameter("cone_polynomial_fit_maximum_rmse_m").value
            ),
            lidar_to_rear_axle_m=float(
                self.get_parameter("cone_lidar_to_rear_axle_m").value
            ),
            curve_outward_bias_max_m=float(
                self.get_parameter("cone_curve_outward_bias_max_m").value
            ),
            curve_outward_bias_left_max_m=float(
                self.get_parameter("cone_curve_outward_bias_left_max_m").value
            ),
            curve_outward_bias_right_max_m=float(
                self.get_parameter("cone_curve_outward_bias_right_max_m").value
            ),
            curve_outward_bias_start_ratio=float(
                self.get_parameter("cone_curve_outward_bias_start_ratio").value
            ),
            curve_outward_bias_full_ratio=float(
                self.get_parameter("cone_curve_outward_bias_full_ratio").value
            ),
            vehicle_width_m=float(
                self.get_parameter("cone_vehicle_width_m").value
            ),
            minimum_boundary_clearance_m=float(
                self.get_parameter("cone_minimum_boundary_clearance_m").value
            ),
            steering_gain=float(self.get_parameter("cone_steering_gain").value),
            heading_gain=float(self.get_parameter("cone_heading_gain").value),
            drive_speed_mps=float(self.get_parameter("cone_drive_speed_mps").value),
            corner_speed_mps=float(self.get_parameter("cone_corner_speed_mps").value),
            fallback_speed_mps=float(
                self.get_parameter("cone_fallback_speed_mps").value
            ),
            detection_min_slices=int(
                self.get_parameter("cone_detection_min_slices").value
            ),
            detection_min_total_points=int(
                self.get_parameter("cone_detection_min_total_points").value
            ),
            steering_filter_alpha=float(
                self.get_parameter("cone_steering_filter_alpha").value
            ),
            max_steering_rate_rad_s=float(
                self.get_parameter("cone_max_steering_rate_rad_s").value
            ),
        )

    def _build_lidar_config(self) -> LidarSectorConfig:
        return LidarSectorConfig(
            angle_offset_rad=float(self.get_parameter("lidar_angle_offset_rad").value)
        )

    def _build_path_guard(self) -> ConePathContinuityGuard:
        return ConePathContinuityGuard(
            ConePathGuardConfig(
                maximum_target_jump_m=float(
                    self.get_parameter("cone_path_max_target_jump_m").value
                ),
                maximum_steering_jump_rad=float(
                    self.get_parameter("cone_path_max_steering_jump_rad").value
                ),
                hold_seconds=float(self.get_parameter("cone_path_hold_sec").value),
                hold_speed_mps=float(
                    self.get_parameter("cone_path_hold_speed_mps").value
                ),
                sharp_hold_steering_rad=float(
                    self.get_parameter(
                        "cone_path_sharp_hold_steering_rad"
                    ).value
                ),
                sharp_hold_seconds=float(
                    self.get_parameter("cone_path_sharp_hold_sec").value
                ),
                sharp_hold_speed_mps=float(
                    self.get_parameter("cone_path_sharp_hold_speed_mps").value
                ),
                stop_on_lost=bool(
                    self.get_parameter("cone_path_stop_on_lost").value
                ),
                transition_target_rate_mps=float(
                    self.get_parameter("cone_path_transition_target_rate_mps").value
                ),
                transition_steering_rate_rad_s=float(
                    self.get_parameter(
                        "cone_path_transition_steering_rate_rad_s"
                    ).value
                ),
                pending_target_step_m=float(
                    self.get_parameter("cone_path_pending_target_step_m").value
                ),
                pending_steering_step_rad=float(
                    self.get_parameter(
                        "cone_path_pending_steering_step_rad"
                    ).value
                ),
                candidate_confirmation_frames=int(
                    self.get_parameter(
                        "cone_path_candidate_confirmation_frames"
                    ).value
                ),
                candidate_match_target_m=float(
                    self.get_parameter("cone_path_candidate_match_target_m").value
                ),
                candidate_match_steering_rad=float(
                    self.get_parameter(
                        "cone_path_candidate_match_steering_rad"
                    ).value
                ),
                profile_minimum_overlap_slices=int(
                    self.get_parameter(
                        "cone_path_profile_min_overlap_slices"
                    ).value
                ),
                profile_median_tolerance_m=float(
                    self.get_parameter(
                        "cone_path_profile_median_tolerance_m"
                    ).value
                ),
                profile_maximum_tolerance_m=float(
                    self.get_parameter(
                        "cone_path_profile_max_tolerance_m"
                    ).value
                ),
            )
        )

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        dt = 0.05 if self._last_control_time == 0.0 else max(0.01, min(0.20, now - self._last_control_time))
        self._last_scan_time = now
        self._last_control_time = now

        raw_result: ConeCorridorResult = analyze_cone_corridor(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            config=self._config,
            lidar_config=self._lidar_config,
            target_lookahead_override_m=self._filtered_lookahead_m,
        )
        suggested_lookahead = self._filtered_lookahead_m
        next_filtered_lookahead = self._filtered_lookahead_m
        if raw_result.slices:
            suggested_lookahead = float(raw_result.suggested_lookahead_m)
            max_lookahead_delta = max(0.0, self._lookahead_rate_mps) * dt
            next_filtered_lookahead = float(
                clamp(
                    suggested_lookahead,
                    self._filtered_lookahead_m - max_lookahead_delta,
                    self._filtered_lookahead_m + max_lookahead_delta,
                )
            )
        decision = self._path_guard.update(
            raw_result,
            now,
            last_commanded_steering_rad=self._filtered_steering_rad,
        )
        # 거절된 한 프레임 후보가 다음 프레임의 planner lookahead까지 바꾸면
        # trusted target을 유지해도 내부 경로가 흔들린다. 정상/확정 경로만 반영한다.
        if decision.path_state == "TRACKING":
            self._filtered_lookahead_m = next_filtered_lookahead
        result = decision.result
        self._last_result = result

        raw_steering = float(result.steering_rad) if result.found else 0.0
        drive_active = result.found
        target_steer = raw_steering
        planned_speed = float(result.speed_mps) if drive_active else 0.0
        if not math.isfinite(planned_speed):
            planned_speed = 0.0
        requested_maximum_speed = float(self.get_parameter("max_speed_mps").value)
        maximum_speed = (
            max(0.0, requested_maximum_speed)
            if math.isfinite(requested_maximum_speed)
            else 0.0
        )
        cmd_speed = limit_forward_speed(planned_speed, maximum_speed)
        target_x_m = float(result.target_x_m) if drive_active else 0.0
        target_y_m = (
            float(result.target_y_m)
            if drive_active
            else self._config.target_lookahead_m
        )

        # 조향 저역통과 필터 및 변화율 제한 적용
        alpha = self._config.steering_filter_alpha
        smoothed_steer = self._filtered_steering_rad + alpha * (target_steer - self._filtered_steering_rad)

        # Slew rate limit
        max_delta = self._config.max_steering_rate_rad_s * dt
        final_steer = float(
            clamp(
                smoothed_steer,
                self._filtered_steering_rad - max_delta,
                self._filtered_steering_rad + max_delta,
            )
        )
        self._filtered_steering_rad = final_steer

        # 프리뷰 명령 발행 [steering_rad, speed_mps, target_x, target_y]
        preview_msg = Float32MultiArray()
        preview_msg.data = [
            final_steer,
            cmd_speed,
            target_x_m,
            target_y_m,
        ]
        self._preview_publisher.publish(preview_msg)

        # 상태 JSON 발행 (모터 브리지 규격 100% 준수: mode in READY_MODES, motor_output is False)
        output_mode = (
            "HOLD" if decision.path_state.startswith("HOLD") else "TRACKING"
        )
        if not drive_active:
            output_mode = "HOLD"

        # 경계 좌표는 dataclass 내부의 tuple/float 표현을 그대로 노출하지 않고,
        # 대시보드와 로그 수집기가 안정적으로 소비할 수 있는 작은 JSON 배열로 만든다.
        # 비정상 수치는 그리지 않는 편이 안전하므로 직렬화 단계에서 제외한다.
        def rounded_xy(points: tuple[tuple[float, float], ...]) -> list[list[float]]:
            return [
                [round(float(x_m), 3), round(float(y_m), 3)]
                for x_m, y_m in points
                if math.isfinite(float(x_m)) and math.isfinite(float(y_m))
            ]

        status_payload = {
            "stamp": now,
            "mode": output_mode,
            "motor_output": False,
            "found": drive_active,
            "target_x_m": round(target_x_m, 3),
            "target_y_m": round(target_y_m, 3),
            "control_target_y_m": round(float(result.control_target_y_m), 3),
            "centerline_target_x_m": round(
                float(result.centerline_target_x_m), 3
            ),
            "curve_outward_bias_m": round(float(result.curve_outward_bias_m), 3),
            "lidar_to_rear_axle_m": round(
                max(0.0, float(self._config.lidar_to_rear_axle_m)), 3
            ),
            "steering_rad": round(final_steer, 4),
            "steering_deg": round(math.degrees(final_steer), 1),
            # 기존 소비자 호환: raw_steering_deg는 최종 필터 전의 채택 명령이다.
            "raw_steering_deg": round(math.degrees(raw_steering), 1),
            # speed_mps는 preview/bridge에 실제 전달한 값이다. 튜닝 프로파일이
            # 원래 제안한 속도는 planned_speed_mps로 따로 남겨 로그 혼동을 막는다.
            "speed_mps": round(cmd_speed, 2),
            "planned_speed_mps": round(planned_speed, 2),
            "max_speed_mps": round(maximum_speed, 2),
            "speed_limited": cmd_speed + 1.0e-9 < max(0.0, planned_speed),
            "confidence": round(result.confidence, 2),
            "left_points": raw_result.left_points_count,
            "right_points": raw_result.right_points_count,
            "left_cones": raw_result.left_cones_count,
            "right_cones": raw_result.right_cones_count,
            "left_candidate_cones": raw_result.left_candidates_count,
            "right_candidate_cones": raw_result.right_candidates_count,
            "width_est_m": round(result.corridor_width_est_m, 2)
            if result.corridor_width_est_m is not None
            else None,
            "primary_side": result.primary_side,
            # 최종 채택(또는 guard가 유지 중인 trusted) 경계. 기존 필드는 그대로
            # 두고 additive 진단 필드만 제공해 구버전 bridge/dashboard와 호환한다.
            "boundary_mode": result.boundary_mode,
            "left_boundary_xy": rounded_xy(result.left_boundary_xy),
            "right_boundary_xy": rounded_xy(result.right_boundary_xy),
            "path_state": decision.path_state,
            "rejection_reason": decision.rejection_reason,
            "raw_found": raw_result.found,
            "raw_target_x_m": round(raw_result.target_x_m, 3)
            if raw_result.found
            else None,
            "raw_target_y_m": round(raw_result.target_y_m, 3)
            if raw_result.found
            else None,
            "raw_control_target_y_m": round(raw_result.control_target_y_m, 3)
            if raw_result.found
            else None,
            "raw_candidate_steering_deg": round(
                math.degrees(raw_result.steering_rad), 1
            )
            if raw_result.found
            else None,
            "raw_primary_side": raw_result.primary_side,
            # 이번 LaserScan에서 planner가 실제 만든 후보와 경계. guard가 후보를
            # 거절한 프레임에도 accepted 경계와 나란히 비교할 수 있다.
            "raw_candidate_centers_xy": rounded_xy(
                raw_result.candidate_centers_xy
            ),
            "raw_left_boundary_xy": rounded_xy(raw_result.left_boundary_xy),
            "raw_right_boundary_xy": rounded_xy(raw_result.right_boundary_xy),
            "raw_boundary_mode": raw_result.boundary_mode,
            "raw_boundary_fallback_reason": raw_result.boundary_fallback_reason,
            "raw_paired_slice_count": int(raw_result.paired_slice_count),
            "raw_boundary_heading_rad": round(
                float(raw_result.boundary_heading_rad), 4
            ),
            "raw_width_est_m": round(raw_result.corridor_width_est_m, 3)
            if raw_result.corridor_width_est_m is not None
            else None,
            "raw_path_model": raw_result.path_model,
            "raw_path_curvature_1pm": round(
                float(raw_result.path_curvature_1pm), 3
            ),
            "raw_polynomial_rmse_m": round(raw_result.polynomial_rmse_m, 3)
            if raw_result.polynomial_rmse_m is not None
            else None,
            "target_jump_m": round(decision.target_jump_m, 3)
            if decision.target_jump_m is not None
            else None,
            "steering_jump_rad": round(decision.steering_jump_rad, 3)
            if decision.steering_jump_rad is not None
            else None,
            "hold_age_sec": round(decision.hold_age_sec, 3),
            "transition_phase": decision.transition_phase,
            "hold_profile": decision.hold_profile,
            "hold_limit_sec": round(decision.hold_limit_sec, 3),
            "hold_speed_limit_mps": round(decision.hold_speed_limit_mps, 2),
            "profile_error_m": round(decision.profile_error_m, 3)
            if decision.profile_error_m is not None
            else None,
            "lookahead_m": round(target_y_m, 3),
            "suggested_lookahead_m": round(suggested_lookahead, 3),
            "turn_ratio": round(raw_result.turn_ratio, 3),
            "path_model": result.path_model,
            "path_curvature_1pm": round(result.path_curvature_1pm, 3),
            "polynomial_rmse_m": round(result.polynomial_rmse_m, 3)
            if result.polynomial_rmse_m is not None
            else None,
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload)
        self._status_publisher.publish(status_msg)

        # 주기적 로깅 (1초에 2회)
        if now - self._last_log_time >= 0.50:
            self._last_log_time = now
            if result.found:
                self.get_logger().info(
                    f"[CONE {decision.path_state}] Target: "
                    f"(x={result.target_x_m:+.2f}m, lidar_y={result.target_y_m:.2f}m, "
                    f"axle_y={result.control_target_y_m:.2f}m) | "
                    f"OutBias: {result.curve_outward_bias_m:.2f}m | "
                    f"Steer: {math.degrees(final_steer):+5.1f}° (raw {math.degrees(raw_steering):+5.1f}°) | "
                    f"Speed: {cmd_speed:.2f}m/s "
                    f"(plan {planned_speed:.2f}, cap {maximum_speed:.2f}) | "
                    f"Side: {result.primary_side:<10} "
                    f"(L:{raw_result.left_points_count:2d}, R:{raw_result.right_points_count:2d}) | "
                    f"Width: {status_payload['width_est_m']}m | "
                    f"Path: {result.path_model} k={result.path_curvature_1pm:+.2f}/m | "
                    f"Reason: {decision.rejection_reason}"
                )
            else:
                self.get_logger().warn(
                    f"[CONE {decision.path_state}] 통로 미검출 "
                    f"(reason={decision.rejection_reason}, "
                    f"L:{raw_result.left_points_count}, R:{raw_result.right_points_count})"
                )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ConeCorridorDriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

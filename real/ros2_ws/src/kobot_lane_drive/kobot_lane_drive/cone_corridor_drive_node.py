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
from .geometry import clamp
from .obstacle_lidar import LidarSectorConfig


class ConeCorridorDriveNode(Node):
    """라바콘 단독 주행 판단 노드."""

    def __init__(self) -> None:
        super().__init__("cone_corridor_drive")
        self._declare_parameters()

        self._config = self._build_cone_config()
        self._lidar_config = self._build_lidar_config()

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
        self.declare_parameter("auto_enable", True)
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
        self.declare_parameter("cone_slice_step_m", 0.15)
        self.declare_parameter("cone_target_lookahead_m", 0.48)
        self.declare_parameter("cone_drive_speed_mps", 0.25)
        self.declare_parameter("cone_corner_speed_mps", 0.20)
        self.declare_parameter("cone_detection_min_slices", 3)
        self.declare_parameter("cone_detection_min_total_points", 6)
        self.declare_parameter("cone_steering_filter_alpha", 0.60)
        self.declare_parameter("cone_max_steering_rate_rad_s", 2.5)
        self.declare_parameter("cone_steering_gain", 2.2)
        self.declare_parameter("cone_heading_gain", 0.35)

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
            slice_step_m=float(self.get_parameter("cone_slice_step_m").value),
            target_lookahead_m=float(
                self.get_parameter("cone_target_lookahead_m").value
            ),
            steering_gain=float(self.get_parameter("cone_steering_gain").value),
            heading_gain=float(self.get_parameter("cone_heading_gain").value),
            drive_speed_mps=float(self.get_parameter("cone_drive_speed_mps").value),
            corner_speed_mps=float(self.get_parameter("cone_corner_speed_mps").value),
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

    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        dt = 0.05 if self._last_control_time == 0.0 else max(0.01, min(0.20, now - self._last_control_time))
        self._last_scan_time = now
        self._last_control_time = now

        result: ConeCorridorResult = analyze_cone_corridor(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            config=self._config,
            lidar_config=self._lidar_config,
        )
        self._last_result = result

        # 조향 저역통과 필터 및 변화율 제한 적용
        raw_steering = float(result.steering_rad) if result.found else 0.0
        alpha = self._config.steering_filter_alpha
        smoothed_steer = self._filtered_steering_rad + alpha * (raw_steering - self._filtered_steering_rad)

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
            float(result.speed_mps) if result.found else 0.0,
            float(result.target_x_m),
            float(result.target_y_m),
        ]
        self._preview_publisher.publish(preview_msg)

        # 상태 JSON 발행 (모터 브리지 호환 mode/motor_output 포함)
        status_payload = {
            "stamp": now,
            "mode": "TRACKING" if result.found else "HOLD",
            "motor_output": False,
            "found": result.found,
            "target_x_m": round(result.target_x_m, 3),
            "target_y_m": round(result.target_y_m, 3),
            "steering_rad": round(final_steer, 4),
            "steering_deg": round(math.degrees(final_steer), 1),
            "raw_steering_deg": round(math.degrees(raw_steering), 1),
            "speed_mps": round(result.speed_mps, 2),
            "confidence": round(result.confidence, 2),
            "left_points": result.left_points_count,
            "right_points": result.right_points_count,
            "width_est_m": round(result.corridor_width_est_m, 2)
            if result.corridor_width_est_m is not None
            else None,
            "primary_side": result.primary_side,
        }
        status_msg = String()
        status_msg.data = json.dumps(status_payload)
        self._status_publisher.publish(status_msg)

        # 주기적 로깅 (1초에 2회)
        if now - self._last_log_time >= 0.50:
            self._last_log_time = now
            if result.found:
                self.get_logger().info(
                    f"[CONE] Target: (x={result.target_x_m:+.2f}m, y={result.target_y_m:.2f}m) | "
                    f"Steer: {math.degrees(final_steer):+5.1f}° (raw {math.degrees(raw_steering):+5.1f}°) | "
                    f"Speed: {result.speed_mps:.2f}m/s | "
                    f"Side: {result.primary_side:<10} (L:{result.left_points_count:2d}, R:{result.right_points_count:2d}) | "
                    f"Width: {status_payload['width_est_m']}m"
                )
            else:
                self.get_logger().warn(
                    f"[CONE] 통로 미검출 (L:{result.left_points_count}, R:{result.right_points_count})"
                )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ConeCorridorDriveNode()
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

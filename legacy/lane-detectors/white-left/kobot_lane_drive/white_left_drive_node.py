from __future__ import annotations

import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .white_left_boundary import (
    WhiteLeftConfig,
    detect_white_left_boundary,
    detect_white_right_boundary,
    render_dual_roi_overlay,
    select_lane_control,
)


def _move_towards(current: float, target: float, maximum_step: float) -> float:
    if target > current:
        return min(target, current + maximum_step)
    return max(target, current - maximum_step)


class WhiteLeftDriveNode(Node):
    """Frame-90 white-left detector and motor-free fixed-pixel controller."""

    def __init__(self) -> None:
        super().__init__("white_left_drive")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._config = WhiteLeftConfig(
            work_width=int(self.get_parameter("work_width").value),
            work_height=int(self.get_parameter("work_height").value),
            near_y_ratio=float(self.get_parameter("near_y_ratio").value),
            far_y_ratio=float(self.get_parameter("far_y_ratio").value),
            control_y_ratio=float(self.get_parameter("control_y_ratio").value),
            left_search_ratio=float(self.get_parameter("left_search_ratio").value),
            minimum_path_score=float(self.get_parameter("minimum_path_score").value),
            continuation_minimum_score=float(
                self.get_parameter("continuation_minimum_score").value
            ),
            continuation_maximum_gap_rows=int(
                self.get_parameter("continuation_maximum_gap_rows").value
            ),
            sharp_turn_window_rows=int(
                self.get_parameter("sharp_turn_window_rows").value
            ),
            maximum_turn_angle_deg=float(
                self.get_parameter("maximum_turn_angle_deg").value
            ),
            edge_gradient_threshold=float(
                self.get_parameter("edge_gradient_threshold").value
            ),
            stripe_minimum_width_ratio=float(
                self.get_parameter("stripe_minimum_width_ratio").value
            ),
            stripe_maximum_width_ratio=float(
                self.get_parameter("stripe_maximum_width_ratio").value
            ),
            stripe_minimum_contrast=float(
                self.get_parameter("stripe_minimum_contrast").value
            ),
            stripe_maximum_saturation=float(
                self.get_parameter("stripe_maximum_saturation").value
            ),
            ransac_iterations=int(
                self.get_parameter("ransac_iterations").value
            ),
            ransac_residual_ratio=float(
                self.get_parameter("ransac_residual_ratio").value
            ),
            ransac_minimum_inliers=int(
                self.get_parameter("ransac_minimum_inliers").value
            ),
            minimum_inward_progress_ratio=float(
                self.get_parameter("minimum_inward_progress_ratio").value
            ),
            maximum_bottom_slope_error_ratio=float(
                self.get_parameter("maximum_bottom_slope_error_ratio").value
            ),
            bottom_extrapolation_ratio=float(
                self.get_parameter("bottom_extrapolation_ratio").value
            ),
            desired_left_distance_ratio=float(
                self.get_parameter("desired_left_distance_ratio").value
            ),
            single_lane_distance_ratio=float(
                self.get_parameter("single_lane_distance_ratio").value
            ),
        )
        self._last_started_at = 0.0
        self._last_debug_at = 0.0
        self._last_control_at = 0.0
        self._steering_rad = 0.0
        self._last_valid_preview: tuple[float, float, float] | None = None
        self._minimum_period = 1.0 / max(
            1.0, float(self.get_parameter("max_processing_hz").value)
        )
        debug_hz = float(self.get_parameter("debug_publish_hz").value)
        self._debug_period = 0.0 if debug_hz <= 0.0 else 1.0 / debug_hz

        self._debug_publisher = self.create_publisher(
            Image,
            str(self.get_parameter("debug_image_topic").value),
            1,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("status_topic").value),
            10,
        )
        self._preview_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("drive_preview_topic").value),
            10,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().warning(
            "White-left frame-90 controller is preview-only; no motor publisher"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter(
            "debug_image_topic", "/kobot_lane_drive/white_left/debug_image"
        )
        self.declare_parameter(
            "status_topic", "/kobot_lane_drive/white_left/status"
        )
        self.declare_parameter(
            "drive_preview_topic", "/kobot_lane_drive/white_left/drive_preview"
        )
        self.declare_parameter("max_processing_hz", 20.0)
        self.declare_parameter("debug_publish_hz", 5.0)
        self.declare_parameter("work_width", 320)
        self.declare_parameter("work_height", 240)
        self.declare_parameter("near_y_ratio", 0.80)
        self.declare_parameter("far_y_ratio", 0.625)
        self.declare_parameter("control_y_ratio", 0.70)
        self.declare_parameter("left_search_ratio", 0.52)
        self.declare_parameter("minimum_path_score", 0.30)
        self.declare_parameter("continuation_minimum_score", 0.25)
        self.declare_parameter("continuation_maximum_gap_rows", 5)
        self.declare_parameter("sharp_turn_window_rows", 5)
        self.declare_parameter("maximum_turn_angle_deg", 60.0)
        self.declare_parameter("edge_gradient_threshold", 25.0)
        self.declare_parameter("stripe_minimum_width_ratio", 4.0 / 640.0)
        self.declare_parameter("stripe_maximum_width_ratio", 28.0 / 640.0)
        self.declare_parameter("stripe_minimum_contrast", 7.0)
        self.declare_parameter("stripe_maximum_saturation", 115.0)
        self.declare_parameter("ransac_iterations", 1200)
        self.declare_parameter("ransac_residual_ratio", 7.0 / 640.0)
        self.declare_parameter("ransac_minimum_inliers", 12)
        self.declare_parameter("minimum_inward_progress_ratio", 12.0 / 640.0)
        self.declare_parameter("maximum_bottom_slope_error_ratio", 80.0 / 640.0)
        self.declare_parameter("bottom_extrapolation_ratio", 80.0 / 640.0)
        self.declare_parameter("desired_left_distance_ratio", 244.0 / 640.0)
        self.declare_parameter("single_lane_distance_ratio", 180.0 / 640.0)
        self.declare_parameter("speed_mps", 0.50)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_kp", 0.42)
        self.declare_parameter("steering_output_gain", 4.0)
        self.declare_parameter("steering_filter_alpha", 0.35)
        # User-selected test limit: 70 units * 0.0068 rad/unit = 0.476 rad.
        self.declare_parameter("max_steering_rad", 0.476)
        self.declare_parameter("max_steering_rate_rad_s", 1.20)

    def _tracking_steering(self, offset_px: float, width: int, now: float) -> float:
        dt = 0.05 if self._last_control_at <= 0.0 else max(
            0.001, min(0.20, now - self._last_control_at)
        )
        self._last_control_at = now
        normalized_error = offset_px / max(1.0, width * 0.5)
        target = (
            float(self.get_parameter("steering_sign").value)
            * float(self.get_parameter("steering_kp").value)
            * float(self.get_parameter("steering_output_gain").value)
            * normalized_error
        )
        maximum = float(self.get_parameter("max_steering_rad").value)
        target = max(-maximum, min(maximum, target))
        alpha = float(self.get_parameter("steering_filter_alpha").value)
        filtered = self._steering_rad + alpha * (target - self._steering_rad)
        self._steering_rad = _move_towards(
            self._steering_rad,
            filtered,
            float(self.get_parameter("max_steering_rate_rad_s").value) * dt,
        )
        return self._steering_rad

    def _publish(self, payload: dict[str, object], preview: tuple[float, float, float]) -> None:
        self._preview_publisher.publish(Float32MultiArray(data=list(preview)))
        self._status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_started_at < self._minimum_period:
            return
        self._last_started_at = now
        publish_debug = (
            self._debug_period > 0.0
            and now - self._last_debug_at >= self._debug_period
        )
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            result = detect_white_left_boundary(
                frame,
                config=self._config,
                render_debug=False,
            )
            right_result = detect_white_right_boundary(frame, config=self._config)
            payload = result.status()
            payload.update(right_result.status())
            used_lane, center_offset, selected_confidence = select_lane_control(
                result,
                right_result,
                frame.shape[1] * self._config.single_lane_distance_ratio,
            )
            payload["used_lane"] = used_lane
            if center_offset is not None:
                steering = self._tracking_steering(
                    center_offset,
                    frame.shape[1],
                    now,
                )
                preview = (
                    steering,
                    float(self.get_parameter("speed_mps").value),
                    selected_confidence,
                )
                self._last_valid_preview = preview
                payload.update({
                    "mode": "TRACKING",
                    "steering_rad": round(steering, 5),
                    "speed_mps": round(preview[1], 3),
                    "reason": f"edge_pair_ransac_{used_lane.lower()}",
                })
            elif self._last_valid_preview is not None:
                preview = self._last_valid_preview
                payload.update({
                    "mode": "HOLD",
                    "steering_rad": round(preview[0], 5),
                    "speed_mps": round(preview[1], 3),
                    "reason": "both_lanes_lost_hold_last",
                })
            else:
                preview = (0.0, 0.0, 0.0)
                payload.update({
                    "mode": "WAITING",
                    "steering_rad": 0.0,
                    "speed_mps": 0.0,
                    "reason": "no_valid_lane_yet",
                })
            self._publish(payload, preview)
            if publish_debug:
                overlay = render_dual_roi_overlay(
                    frame,
                    self._config,
                    result,
                    right_result,
                )
                debug = self._bridge.cv2_to_imgmsg(
                    overlay,
                    encoding="bgr8",
                )
                debug.header = message.header
                self._debug_publisher.publish(debug)
                self._last_debug_at = now
        except Exception as error:  # noqa: BLE001 - fail closed on node errors
            payload = {
                "state": "ERROR",
                "mode": "ERROR",
                "confidence": 0.0,
                "steering_rad": 0.0,
                "speed_mps": 0.0,
                "motor_output": False,
                "error": str(error),
            }
            self._publish(payload, (0.0, 0.0, 0.0))
            self.get_logger().error(f"White-left perception failed: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WhiteLeftDriveNode()
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

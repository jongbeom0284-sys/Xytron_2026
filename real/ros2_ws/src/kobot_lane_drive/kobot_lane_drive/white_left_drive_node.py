from __future__ import annotations

import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .white_left_boundary import WhiteLeftConfig, detect_white_left_boundary


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
            desired_left_distance_ratio=float(
                self.get_parameter("desired_left_distance_ratio").value
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
        self.declare_parameter("far_y_ratio", 0.52)
        self.declare_parameter("control_y_ratio", 0.70)
        self.declare_parameter("left_search_ratio", 0.52)
        self.declare_parameter("minimum_path_score", 0.30)
        self.declare_parameter("desired_left_distance_ratio", 244.0 / 640.0)
        self.declare_parameter("speed_mps", 0.20)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_kp", 0.42)
        self.declare_parameter("steering_filter_alpha", 0.35)
        self.declare_parameter("max_steering_rad", 0.34)
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
                render_debug=publish_debug,
            )
            payload = result.status()
            if result.state == "READY" and result.center_offset_px is not None:
                steering = self._tracking_steering(
                    result.center_offset_px,
                    frame.shape[1],
                    now,
                )
                preview = (
                    steering,
                    float(self.get_parameter("speed_mps").value),
                    result.confidence,
                )
                self._last_valid_preview = preview
                payload.update({
                    "mode": "TRACKING",
                    "steering_rad": round(steering, 5),
                    "speed_mps": round(preview[1], 3),
                    "reason": "white_left_boundary",
                })
            elif self._last_valid_preview is not None:
                preview = self._last_valid_preview
                payload.update({
                    "mode": "HOLD",
                    "steering_rad": round(preview[0], 5),
                    "speed_mps": round(preview[1], 3),
                    "reason": "fresh_lane_miss_hold_last",
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
            if publish_debug and result.debug_image is not None:
                debug = self._bridge.cv2_to_imgmsg(
                    result.debug_image,
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

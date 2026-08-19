from __future__ import annotations

import json
import time

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .fast_lane import FastLaneConfig, detect_fast_lane


class FastLaneNode(Node):
    """Calibration-free pixel lane preview. This node has no actuator publisher."""

    def __init__(self) -> None:
        super().__init__("fast_lane_perception")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._last_started_at = 0.0
        self._last_debug_at = 0.0
        self._minimum_period = 1.0 / float(self.get_parameter("max_processing_hz").value)
        debug_hz = float(self.get_parameter("debug_publish_hz").value)
        self._debug_period = 0.0 if debug_hz <= 0.0 else 1.0 / debug_hz
        self._config = FastLaneConfig(
            roi_top_ratio=float(self.get_parameter("roi_top_ratio").value),
            roi_bottom_ratio=float(self.get_parameter("roi_bottom_ratio").value),
            lookahead_ratio=float(self.get_parameter("lookahead_ratio").value),
            expected_bottom_width_ratio=float(self.get_parameter("expected_bottom_width_ratio").value),
            expected_top_width_ratio=float(self.get_parameter("expected_top_width_ratio").value),
            clahe_clip_limit=float(self.get_parameter("clahe_clip_limit").value),
            white_s_max=int(self.get_parameter("white_s_max").value),
            white_v_min=int(self.get_parameter("white_v_min").value),
            white_v_percentile=float(self.get_parameter("white_v_percentile").value),
            white_min_contrast=int(self.get_parameter("white_min_contrast").value),
            yellow_h_low=int(self.get_parameter("yellow_h_low").value),
            yellow_h_high=int(self.get_parameter("yellow_h_high").value),
            yellow_s_min=int(self.get_parameter("yellow_s_min").value),
            yellow_v_min=int(self.get_parameter("yellow_v_min").value),
            max_white_support_ratio=float(self.get_parameter("max_white_support_ratio").value),
            max_yellow_support_ratio=float(self.get_parameter("max_yellow_support_ratio").value),
        )

        self._debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), 1
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._pixel_path_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("pixel_path_topic").value),
            10,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            sensor_qos,
        )
        self.get_logger().warning(
            "Fast lane perception is pixel preview-only: calibration and motor output are disabled"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("debug_image_topic", "/kobot_lane_drive/fast_lane/debug_image")
        self.declare_parameter("status_topic", "/kobot_lane_drive/fast_lane/status")
        self.declare_parameter("pixel_path_topic", "/kobot_lane_drive/fast_lane/pixel_path")
        # Keep this above the camera rate. An equal 30 Hz throttle can turn
        # normal USB timing jitter into an unintended every-other-frame drop.
        self.declare_parameter("max_processing_hz", 60.0)
        self.declare_parameter("debug_publish_hz", 2.0)
        self.declare_parameter("roi_top_ratio", 0.43)
        self.declare_parameter("roi_bottom_ratio", 0.94)
        self.declare_parameter("lookahead_ratio", 0.70)
        self.declare_parameter("expected_bottom_width_ratio", 0.82)
        self.declare_parameter("expected_top_width_ratio", 0.30)
        self.declare_parameter("clahe_clip_limit", 1.6)
        self.declare_parameter("white_s_max", 125)
        self.declare_parameter("white_v_min", 140)
        self.declare_parameter("white_v_percentile", 62.0)
        self.declare_parameter("white_min_contrast", 12)
        self.declare_parameter("yellow_h_low", 15)
        self.declare_parameter("yellow_h_high", 40)
        self.declare_parameter("yellow_s_min", 70)
        self.declare_parameter("yellow_v_min", 80)
        self.declare_parameter("max_white_support_ratio", 0.65)
        self.declare_parameter("max_yellow_support_ratio", 0.45)

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
            result = detect_fast_lane(
                frame,
                config=self._config,
                render_debug=publish_debug,
            )
            status = result.status()
            status["source_stamp_sec"] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            status_message = String()
            status_message.data = json.dumps(status, ensure_ascii=False)
            self._status_publisher.publish(status_message)

            if result.center_offset_px is not None:
                pixel_path = Float32MultiArray()
                pixel_path.data = [
                    float(result.lookahead_y_px),
                    float(frame.shape[1] * 0.5 + result.center_offset_px),
                    float(result.center_offset_px),
                    float(result.confidence),
                ]
                self._pixel_path_publisher.publish(pixel_path)

            if publish_debug and result.debug_image is not None:
                debug_message = self._bridge.cv2_to_imgmsg(
                    result.debug_image, encoding="bgr8"
                )
                debug_message.header = message.header
                self._debug_publisher.publish(debug_message)
                self._last_debug_at = now
        except Exception as exc:  # noqa: BLE001 - status must expose every frame failure
            status_message = String()
            status_message.data = json.dumps(
                {
                    "state": "ERROR",
                    "confidence": 0.0,
                    "motor_output": False,
                    "error": str(exc),
                },
                ensure_ascii=False,
            )
            self._status_publisher.publish(status_message)
            self.get_logger().error(f"Fast lane perception failed: {exc}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLaneNode()
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

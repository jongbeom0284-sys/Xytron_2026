from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from .drive_command import make_legacy_command


class WhiteLeftDriveBridgeNode(Node):
    """Explicitly armed bridge for the frame-90 white-left controller."""

    READY_MODES = {"TRACKING", "HOLD"}

    def __init__(self) -> None:
        super().__init__("white_left_drive_bridge")
        self._declare_parameters()
        if not bool(self.get_parameter("armed").value):
            raise RuntimeError("White-left bridge refused to start: armed must be true")

        self._preview: tuple[float, float] | None = None
        self._preview_at = 0.0
        self._status_ready = False
        self._status_at = 0.0
        self._run_enabled = False
        self._publisher = self.create_publisher(
            Float32MultiArray,
            "/xycar_motor",
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("drive_preview_topic").value),
            self._on_preview,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("status_topic").value),
            self._on_status,
            10,
        )
        enable_qos = QoSProfile(depth=1)
        enable_qos.reliability = ReliabilityPolicy.RELIABLE
        enable_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            Bool,
            str(self.get_parameter("enable_topic").value),
            self._on_enabled,
            enable_qos,
        )
        self.create_timer(0.05, self._publish)
        self.get_logger().error(
            "WHITE-LEFT DRIVE BRIDGE ARMED: waiting for dashboard enable"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("armed", False)
        self.declare_parameter(
            "drive_preview_topic", "/kobot_lane_drive/white_left/drive_preview"
        )
        self.declare_parameter(
            "status_topic", "/kobot_lane_drive/white_left/status"
        )
        self.declare_parameter(
            "enable_topic", "/kobot_lane_drive/record_drive/enabled"
        )
        self.declare_parameter("command_timeout_sec", 0.30)
        self.declare_parameter("max_speed_mps", 0.20)
        self.declare_parameter("max_steering_rad", 0.34)
        self.declare_parameter("angle_scale", 0.0068)
        self.declare_parameter("speed_scale", 0.08)

    def _on_preview(self, message: Float32MultiArray) -> None:
        if len(message.data) < 2:
            self._preview = None
            return
        steering = float(message.data[0])
        speed = float(message.data[1])
        if not math.isfinite(steering) or not math.isfinite(speed):
            self._preview = None
            return
        maximum_steering = float(self.get_parameter("max_steering_rad").value)
        maximum_speed = float(self.get_parameter("max_speed_mps").value)
        self._preview = (
            max(-maximum_steering, min(maximum_steering, steering)),
            max(0.0, min(maximum_speed, speed)),
        )
        self._preview_at = time.monotonic()

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._status_ready = (
                payload.get("mode") in self.READY_MODES
                and payload.get("motor_output") is False
            )
        except (TypeError, ValueError):
            self._status_ready = False
        self._status_at = time.monotonic()

    def _on_enabled(self, message: Bool) -> None:
        self._run_enabled = bool(message.data)

    def _publish(self) -> None:
        now = time.monotonic()
        timeout = float(self.get_parameter("command_timeout_sec").value)
        fresh = (
            self._run_enabled
            and self._preview is not None
            and now - self._preview_at <= timeout
            and now - self._status_at <= timeout
        )
        steering, speed = self._preview if self._preview is not None else (None, 0.0)
        command = make_legacy_command(
            steering,
            speed,
            float(self.get_parameter("angle_scale").value),
            float(self.get_parameter("speed_scale").value),
            self._status_ready and self._run_enabled,
            fresh,
        )
        self._publisher.publish(Float32MultiArray(data=command))

    def destroy_node(self):
        if hasattr(self, "_publisher"):
            self._publisher.publish(Float32MultiArray(data=[0.0, 0.0]))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WhiteLeftDriveBridgeNode()
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

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, String

from .drive_command import make_legacy_command


class DriveBridgeNode(Node):
    """Approval-gated adapter from Stanley preview radians to Kobot motor input."""

    def __init__(self) -> None:
        super().__init__("lane_drive_bridge")
        self.declare_parameter("armed", False)
        self.declare_parameter("speed_mps", 0.30)
        self.declare_parameter("angle_scale", 0.0068)
        self.declare_parameter("speed_scale", 0.08)
        self.declare_parameter("command_timeout_sec", 0.30)
        if not bool(self.get_parameter("armed").value):
            raise RuntimeError("Drive bridge refused to start: armed must be true")

        self._steering: float | None = None
        self._ready = False
        self._last_status_at = 0.0
        self._publisher = self.create_publisher(Float32MultiArray, "/xycar_motor", 10)
        self.create_subscription(Float32, "/kobot_lane_drive/steering_preview", self._on_steering, 10)
        self.create_subscription(String, "/kobot_lane_drive/status", self._on_status, 10)
        self.create_timer(0.05, self._publish)
        self.get_logger().error("LANE DRIVE BRIDGE ARMED: publishing /xycar_motor")

    def _on_steering(self, message: Float32) -> None:
        self._steering = float(message.data)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._ready = payload.get("state") == "READY" and payload.get("motor_output") is False
        except (TypeError, ValueError):
            self._ready = False
        self._last_status_at = time.monotonic()

    def _publish(self) -> None:
        timeout = float(self.get_parameter("command_timeout_sec").value)
        fresh = time.monotonic() - self._last_status_at <= timeout
        angle_scale = float(self.get_parameter("angle_scale").value)
        speed_scale = float(self.get_parameter("speed_scale").value)
        command = make_legacy_command(
            self._steering,
            float(self.get_parameter("speed_mps").value),
            angle_scale,
            speed_scale,
            self._ready,
            fresh,
        )
        self._publisher.publish(Float32MultiArray(data=command))

    def destroy_node(self):
        if hasattr(self, "_publisher"):
            self._publisher.publish(Float32MultiArray(data=[0.0, 0.0]))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

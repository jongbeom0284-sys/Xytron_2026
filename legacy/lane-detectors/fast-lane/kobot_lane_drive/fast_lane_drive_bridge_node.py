from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .drive_command import make_legacy_command


class FastLaneDriveBridgeNode(Node):
    """Explicitly armed, watchdog-gated fast-lane motor bridge."""

    READY_MODES = {"TRACKING", "GRACE", "DECEL"}

    def __init__(self) -> None:
        super().__init__("fast_lane_drive_bridge")
        self._declare_parameters()
        if not bool(self.get_parameter("armed").value):
            raise RuntimeError("Fast lane drive bridge refused to start: armed must be true")

        self._preview: tuple[float, float] | None = None
        self._preview_at = 0.0
        self._control_ready = False
        self._control_status_at = 0.0
        self._publisher = self.create_publisher(Float32MultiArray, "/xycar_motor", 10)
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("drive_preview_topic").value),
            self._on_preview,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("control_status_topic").value),
            self._on_control_status,
            10,
        )
        self.create_timer(0.05, self._publish)
        self.get_logger().error(
            "FAST LANE DRIVE BRIDGE ARMED: publishing /xycar_motor"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("armed", False)
        self.declare_parameter("drive_preview_topic", "/kobot_lane_drive/fast_lane/drive_preview")
        self.declare_parameter("control_status_topic", "/kobot_lane_drive/fast_lane/control_status")
        self.declare_parameter("command_timeout_sec", 0.20)
        self.declare_parameter("max_speed_mps", 0.70)
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
        max_steering = float(self.get_parameter("max_steering_rad").value)
        max_speed = float(self.get_parameter("max_speed_mps").value)
        self._preview = (
            max(-max_steering, min(max_steering, steering)),
            max(0.0, min(max_speed, speed)),
        )
        self._preview_at = time.monotonic()

    def _on_control_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self._control_ready = (
                payload.get("mode") in self.READY_MODES
                and payload.get("motor_output") is False
            )
        except (TypeError, ValueError):
            self._control_ready = False
        self._control_status_at = time.monotonic()

    def _publish(self) -> None:
        now_sec = time.monotonic()
        timeout = float(self.get_parameter("command_timeout_sec").value)
        fresh = (
            self._preview is not None
            and now_sec - self._preview_at <= timeout
            and now_sec - self._control_status_at <= timeout
        )
        steering, speed = self._preview if self._preview is not None else (None, 0.0)
        command = make_legacy_command(
            steering,
            speed,
            float(self.get_parameter("angle_scale").value),
            float(self.get_parameter("speed_scale").value),
            self._control_ready,
            fresh,
        )
        self._publisher.publish(Float32MultiArray(data=command))

    def destroy_node(self):
        if hasattr(self, "_publisher"):
            self._publisher.publish(Float32MultiArray(data=[0.0, 0.0]))
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLaneDriveBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

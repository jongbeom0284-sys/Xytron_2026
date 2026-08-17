from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .fast_lane_control import (
    FastLaneControlConfig,
    FastLaneController,
    LaneObservation,
)


class FastLaneControllerNode(Node):
    """Convert fast-lane pixels into a motor-free steering and speed preview."""

    def __init__(self) -> None:
        super().__init__("fast_lane_controller")
        self._declare_parameters()
        self._message_timeout = float(self.get_parameter("message_timeout_sec").value)
        self._controller = FastLaneController(
            FastLaneControlConfig(
                image_width_px=float(self.get_parameter("image_width_px").value),
                max_speed_mps=float(self.get_parameter("max_speed_mps").value),
                single_lane_max_speed_mps=float(
                    self.get_parameter("single_lane_max_speed_mps").value
                ),
                minimum_tracking_speed_mps=float(
                    self.get_parameter("minimum_tracking_speed_mps").value
                ),
                lost_hold_speed_mps=float(
                    self.get_parameter("lost_hold_speed_mps").value
                ),
                max_steering_rad=float(self.get_parameter("max_steering_rad").value),
                max_steering_rate_rad_s=float(
                    self.get_parameter("max_steering_rate_rad_s").value
                ),
                steering_sign=float(self.get_parameter("steering_sign").value),
                steering_kp=float(self.get_parameter("steering_kp").value),
                steering_kd=float(self.get_parameter("steering_kd").value),
                steering_filter_alpha=float(
                    self.get_parameter("steering_filter_alpha").value
                ),
                single_lane_hold_sec=float(
                    self.get_parameter("single_lane_hold_sec").value
                ),
                offset_filter_alpha_both=float(
                    self.get_parameter("offset_filter_alpha_both").value
                ),
                offset_filter_alpha_single=float(
                    self.get_parameter("offset_filter_alpha_single").value
                ),
                max_offset_step_px=float(
                    self.get_parameter("max_offset_step_px").value
                ),
                minimum_confidence=float(
                    self.get_parameter("minimum_confidence").value
                ),
                curve_slowdown_ratio=float(
                    self.get_parameter("curve_slowdown_ratio").value
                ),
                invalid_grace_sec=float(
                    self.get_parameter("invalid_grace_sec").value
                ),
                invalid_stop_sec=float(
                    self.get_parameter("invalid_stop_sec").value
                ),
            )
        )
        self._status: dict[str, object] | None = None
        self._status_at = 0.0
        self._path: list[float] | None = None
        self._path_at = 0.0

        self._preview_publisher = self.create_publisher(
            Float32MultiArray,
            str(self.get_parameter("drive_preview_topic").value),
            10,
        )
        self._status_publisher = self.create_publisher(
            String,
            str(self.get_parameter("control_status_topic").value),
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("lane_status_topic").value),
            self._on_status,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            str(self.get_parameter("pixel_path_topic").value),
            self._on_path,
            10,
        )
        control_hz = float(self.get_parameter("control_hz").value)
        self.create_timer(1.0 / max(1.0, control_hz), self._on_timer)
        self.get_logger().warning(
            "Fast lane controller is PREVIEW-ONLY; it has no motor publisher"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("lane_status_topic", "/kobot_lane_drive/fast_lane/status")
        self.declare_parameter("pixel_path_topic", "/kobot_lane_drive/fast_lane/pixel_path")
        self.declare_parameter("drive_preview_topic", "/kobot_lane_drive/fast_lane/drive_preview")
        self.declare_parameter("control_status_topic", "/kobot_lane_drive/fast_lane/control_status")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("message_timeout_sec", 0.20)
        self.declare_parameter("image_width_px", 640.0)
        self.declare_parameter("max_speed_mps", 0.70)
        self.declare_parameter("single_lane_max_speed_mps", 0.48)
        self.declare_parameter("minimum_tracking_speed_mps", 0.28)
        self.declare_parameter("lost_hold_speed_mps", 0.30)
        self.declare_parameter("max_steering_rad", 0.34)
        self.declare_parameter("max_steering_rate_rad_s", 1.20)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_kp", 0.42)
        self.declare_parameter("steering_kd", 0.003)
        self.declare_parameter("steering_filter_alpha", 0.35)
        self.declare_parameter("single_lane_hold_sec", 0.30)
        self.declare_parameter("offset_filter_alpha_both", 0.50)
        self.declare_parameter("offset_filter_alpha_single", 0.15)
        self.declare_parameter("max_offset_step_px", 35.0)
        self.declare_parameter("minimum_confidence", 0.20)
        self.declare_parameter("curve_slowdown_ratio", 0.55)
        self.declare_parameter("invalid_grace_sec", 0.25)
        self.declare_parameter("invalid_stop_sec", 0.50)

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if isinstance(payload, dict):
                self._status = payload
                self._status_at = time.monotonic()
        except (TypeError, ValueError):
            self._status = None

    def _on_path(self, message: Float32MultiArray) -> None:
        if len(message.data) < 4 or not all(math.isfinite(float(x)) for x in message.data[:4]):
            return
        self._path = [float(x) for x in message.data[:4]]
        self._path_at = time.monotonic()

    def _observation(self, now_sec: float) -> LaneObservation | None:
        if self._status is None or now_sec - self._status_at > self._message_timeout:
            return None
        state = str(self._status.get("state", "LOST"))
        mask_rejected = bool(self._status.get("mask_rejected", False))
        status_confidence = float(self._status.get("confidence", 0.0) or 0.0)
        if self._path is None or now_sec - self._path_at > self._message_timeout:
            return LaneObservation(state, status_confidence, None, mask_rejected)
        return LaneObservation(
            state=state,
            confidence=min(status_confidence, float(self._path[3])),
            offset_px=float(self._path[2]),
            mask_rejected=mask_rejected,
        )

    def _on_timer(self) -> None:
        now_sec = time.monotonic()
        command = self._controller.step(self._observation(now_sec), now_sec)
        self._preview_publisher.publish(
            Float32MultiArray(
                data=[
                    float(command.steering_rad),
                    float(command.speed_mps),
                    float(command.confidence),
                ]
            )
        )
        payload = {
            "mode": command.mode,
            "steering_rad": round(command.steering_rad, 5),
            "speed_mps": round(command.speed_mps, 3),
            "source_state": command.source_state,
            "confidence": round(command.confidence, 3),
            "reason": command.reason,
            "motor_output": False,
        }
        self._status_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FastLaneControllerNode()
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

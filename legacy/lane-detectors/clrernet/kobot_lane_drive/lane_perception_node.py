from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from .clrernet_engine import CLRerNetEngine
from .image_reference_controller import (
    ImageReferenceController,
    ImageSteeringPreview,
    PixelLaneReference,
)
from .lane_selector import LeftLaneSelection, select_left_lane
from .overlay import draw_perception_overlay


class LanePerceptionNode(Node):
    """Fixed-camera CLRerNet and target-pixel steering preview node."""

    def __init__(self) -> None:
        super().__init__("lane_perception")
        self._declare_parameters()
        package_share = Path(get_package_share_directory("kobot_lane_drive"))
        vendor_root = package_share / "third_party" / "CLRerNet"
        model_config = (
            vendor_root
            / "configs"
            / "clrernet"
            / "culane"
            / "clrernet_culane_dla34_ema.py"
        )
        self._engine = CLRerNetEngine(
            vendor_root=vendor_root,
            config_path=model_config,
            checkpoint_path=str(self.get_parameter("checkpoint_path").value),
            device=str(self.get_parameter("device").value),
            preprocess_size=(
                int(self.get_parameter("preprocess_width").value),
                int(self.get_parameter("preprocess_height").value),
            ),
        )

        maximum_hz = float(self.get_parameter("max_inference_hz").value)
        if maximum_hz <= 0.0:
            raise ValueError("max_inference_hz must be positive")
        self._minimum_period = 1.0 / maximum_hz
        self._last_started_at = 0.0
        self._bridge = CvBridge()
        self._controller = self._make_controller()

        self._debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), 2
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._steering_publisher = self.create_publisher(
            Float32, str(self.get_parameter("steering_preview_topic").value), 10
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("image_topic").value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().warning(
            "Lane perception is preview-only: no actuator publisher"
        )
        if self._controller is None:
            self.get_logger().error(
                "REFERENCE_NOT_READY: capture a correctly positioned fixed-camera image"
            )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("debug_image_topic", "/kobot_lane_drive/debug_image")
        self.declare_parameter("status_topic", "/kobot_lane_drive/status")
        self.declare_parameter(
            "steering_preview_topic", "/kobot_lane_drive/steering_preview"
        )
        self.declare_parameter(
            "checkpoint_path",
            "/home/xytron/2026_Xytron/real/models/clrernet_culane_dla34_ema.pth",
        )
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("max_inference_hz", 10.0)
        self.declare_parameter("minimum_lane_bottom_ratio", 0.55)
        self.declare_parameter("reference_ready", False)
        self.declare_parameter("near_y_ratio", 0.85)
        self.declare_parameter("far_y_ratio", 0.60)
        self.declare_parameter("reference_near_x_ratio", -1.0)
        self.declare_parameter("reference_far_x_ratio", -1.0)
        self.declare_parameter("near_pid_kp", 1.0)
        self.declare_parameter("near_pid_ki", 0.0)
        self.declare_parameter("near_pid_kd", 0.0)
        self.declare_parameter("far_pid_kp", 0.65)
        self.declare_parameter("far_pid_ki", 0.0)
        self.declare_parameter("far_pid_kd", 0.0)
        self.declare_parameter("maximum_steering_rad", 0.34)
        self.declare_parameter("preprocess_width", 1640)
        self.declare_parameter("preprocess_height", 590)

    def _make_controller(self) -> ImageReferenceController | None:
        if not bool(self.get_parameter("reference_ready").value):
            return None
        reference = PixelLaneReference(
            near_y_ratio=float(self.get_parameter("near_y_ratio").value),
            far_y_ratio=float(self.get_parameter("far_y_ratio").value),
            near_x_ratio=float(
                self.get_parameter("reference_near_x_ratio").value
            ),
            far_x_ratio=float(self.get_parameter("reference_far_x_ratio").value),
        )
        return ImageReferenceController(
            reference=reference,
            near_tunings=(
                float(self.get_parameter("near_pid_kp").value),
                float(self.get_parameter("near_pid_ki").value),
                float(self.get_parameter("near_pid_kd").value),
            ),
            far_tunings=(
                float(self.get_parameter("far_pid_kp").value),
                float(self.get_parameter("far_pid_ki").value),
                float(self.get_parameter("far_pid_kd").value),
            ),
            maximum_steering_rad=float(
                self.get_parameter("maximum_steering_rad").value
            ),
        )

    def _publish_status(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_started_at < self._minimum_period:
            return
        self._last_started_at = now
        source = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        lane: LeftLaneSelection | None = None
        preview: ImageSteeringPreview | None = None
        error: str | None = None
        inference_ms = 0.0
        try:
            inference = self._engine.infer(source)
            inference_ms = inference.elapsed_ms
            lane = select_left_lane(
                inference.lanes,
                image_width=source.shape[1],
                image_height=source.shape[0],
                minimum_bottom_ratio=float(
                    self.get_parameter("minimum_lane_bottom_ratio").value
                ),
            )
            if lane.points is None:
                raise RuntimeError("LEFT_LANE_LOST")
            if self._controller is None:
                raise RuntimeError("REFERENCE_NOT_READY")
            preview = self._controller.calculate(
                lane.points,
                image_width=source.shape[1],
                image_height=source.shape[0],
            )
            self._steering_publisher.publish(
                Float32(data=preview.steering_angle_rad)
            )
        except Exception as exc:  # noqa: BLE001 - every failure blocks control
            error = str(exc)
            if self._controller is not None:
                self._controller.reset()

        measurement = None if preview is None else preview.measurement
        valid = error is None and preview is not None
        reference = None if self._controller is None else self._controller.reference
        self._publish_status(
            {
                "state": "READY" if valid else "BLOCKED",
                "control_mode": "fixed_camera_pixel_reference",
                "left_lane": None if lane is None else lane.state,
                "source_lane_index": None if lane is None else lane.source_index,
                "source_lane_count": 0 if lane is None else lane.source_lane_count,
                "reference_ready": reference is not None,
                "reference_near_x_ratio": (
                    None if reference is None else round(reference.near_x_ratio, 6)
                ),
                "reference_far_x_ratio": (
                    None if reference is None else round(reference.far_x_ratio, 6)
                ),
                "measured_near_x_ratio": (
                    None if measurement is None else round(measurement.near_x_ratio, 6)
                ),
                "measured_far_x_ratio": (
                    None if measurement is None else round(measurement.far_x_ratio, 6)
                ),
                "near_error_ratio": (
                    None if measurement is None else round(measurement.near_error_ratio, 6)
                ),
                "far_error_ratio": (
                    None if measurement is None else round(measurement.far_error_ratio, 6)
                ),
                "steering_preview_rad": (
                    None if preview is None else round(preview.steering_angle_rad, 5)
                ),
                "inference_ms": round(inference_ms, 2),
                "motor_output": False,
                "accuracy": "unverified",
                "error": error,
            }
        )
        overlay = draw_perception_overlay(
            source, lane, preview, inference_ms, error
        )
        debug = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        debug.header = message.header
        self._debug_publisher.publish(debug)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LanePerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

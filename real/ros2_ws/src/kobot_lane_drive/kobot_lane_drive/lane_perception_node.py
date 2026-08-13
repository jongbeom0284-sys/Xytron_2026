from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from ipm_library.ipm import IPM
from ipm_library.utils import create_horizontal_plane
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String

from .clrernet_engine import CLRerNetEngine
from .lane_selector import LeftLaneSelection, select_left_lane
from .metric_path import MetricPath, offset_left_lane, validate_camera_matrix
from .overlay import draw_perception_overlay
from .stanley_adapter import calculate_stanley_preview


class LanePerceptionNode(Node):
    """CLRerNet + ros-sports/IPM + Shapely + PythonRobotics preview node."""

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
        self._pythonrobotics_root = package_share / "third_party" / "PythonRobotics"
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
        self._camera_info: CameraInfo | None = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._ipm = IPM(
            self._tf_buffer,
            distortion=bool(self.get_parameter("use_distortion").value),
        )

        self._debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), 2
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
        )
        self._path_publisher = self.create_publisher(
            PathMessage, str(self.get_parameter("target_path_topic").value), 10
        )
        self._steering_publisher = self.create_publisher(
            Float32, str(self.get_parameter("steering_preview_topic").value), 10
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._on_camera_info,
            qos_profile_sensor_data,
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

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")
        self.declare_parameter("debug_image_topic", "/kobot_lane_drive/debug_image")
        self.declare_parameter("status_topic", "/kobot_lane_drive/status")
        self.declare_parameter("target_path_topic", "/kobot_lane_drive/target_path")
        self.declare_parameter("steering_preview_topic", "/kobot_lane_drive/steering_preview")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("use_distortion", True)
        self.declare_parameter("checkpoint_path", "/home/xytron/2026_Xytron/real/models/clrernet_culane_dla34_ema.pth")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("max_inference_hz", 10.0)
        self.declare_parameter("target_distance_m", 0.50)
        self.declare_parameter("minimum_lane_bottom_ratio", 0.55)
        self.declare_parameter("minimum_forward_m", 0.15)
        self.declare_parameter("maximum_forward_m", 4.0)
        self.declare_parameter("preview_speed_mps", 0.50)
        self.declare_parameter("wheelbase_m", 0.24)
        self.declare_parameter("stanley_gain", 0.5)
        self.declare_parameter("maximum_steering_rad", 0.34)
        self.declare_parameter("preprocess_width", 1640)
        self.declare_parameter("preprocess_height", 590)

    def _on_camera_info(self, message: CameraInfo) -> None:
        try:
            validate_camera_matrix(np.asarray(message.k))
        except ValueError as exc:
            if self._camera_info is not None:
                self.get_logger().error(f"Camera calibration became invalid: {exc}")
            self._camera_info = None
            return
        self._camera_info = message
        self._ipm.set_camera_info(message)

    def _publish_status(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)

    def _project_lane(self, lane: LeftLaneSelection, stamp: Time) -> MetricPath:
        if self._camera_info is None:
            raise RuntimeError("CALIBRATION_INVALID: /camera_info K matrix is missing or zero")
        base_frame = str(self.get_parameter("base_frame").value)
        _, ground_points = self._ipm.map_points(
            create_horizontal_plane(),
            lane.points.astype(np.float32),
            stamp,
            plane_frame_id=base_frame,
            output_frame_id=base_frame,
        )
        return offset_left_lane(
            ground_points,
            target_distance_m=float(self.get_parameter("target_distance_m").value),
            minimum_forward_m=float(self.get_parameter("minimum_forward_m").value),
            maximum_forward_m=float(self.get_parameter("maximum_forward_m").value),
        )

    def _publish_path(self, metric_path: MetricPath, stamp: Time) -> None:
        path = PathMessage()
        path.header.stamp = stamp
        path.header.frame_id = str(self.get_parameter("base_frame").value)
        for x_value, y_value in metric_path.target_path_xy:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x_value)
            pose.pose.position.y = float(y_value)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_publisher.publish(path)

    def _on_image(self, message: Image) -> None:
        now = time.monotonic()
        if now - self._last_started_at < self._minimum_period:
            return
        self._last_started_at = now
        source = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        lane: LeftLaneSelection | None = None
        metric_path: MetricPath | None = None
        steering_angle: float | None = None
        error: str | None = None
        inference_ms = 0.0
        try:
            inference = self._engine.infer(source)
            inference_ms = inference.elapsed_ms
            lane = select_left_lane(
                inference.lanes,
                image_width=source.shape[1],
                image_height=source.shape[0],
                minimum_bottom_ratio=float(self.get_parameter("minimum_lane_bottom_ratio").value),
            )
            if lane.points is None:
                raise RuntimeError("LEFT_LANE_LOST")
            metric_path = self._project_lane(lane, message.header.stamp)
            preview = calculate_stanley_preview(
                metric_path.target_path_xy,
                speed_mps=float(self.get_parameter("preview_speed_mps").value),
                vendor_root=self._pythonrobotics_root,
                wheelbase_m=float(self.get_parameter("wheelbase_m").value),
                gain=float(self.get_parameter("stanley_gain").value),
                maximum_steering_rad=float(self.get_parameter("maximum_steering_rad").value),
            )
            steering_angle = preview.steering_angle_rad
            steering_message = Float32(data=steering_angle)
            self._steering_publisher.publish(steering_message)
            self._publish_path(metric_path, message.header.stamp)
        except Exception as exc:  # noqa: BLE001 - every pipeline failure blocks control
            error = str(exc)

        valid = error is None and metric_path is not None and steering_angle is not None
        self._publish_status(
            {
                "state": "READY" if valid else "BLOCKED",
                "left_lane": None if lane is None else lane.state,
                "source_lane_index": None if lane is None else lane.source_index,
                "source_lane_count": 0 if lane is None else lane.source_lane_count,
                "metric_calibrated": self._camera_info is not None,
                "target_distance_m": float(self.get_parameter("target_distance_m").value),
                "measured_distance_m": None if metric_path is None else round(metric_path.measured_distance_m, 4),
                "lateral_error_m": None if metric_path is None else round(metric_path.lateral_error_m, 4),
                "steering_preview_rad": None if steering_angle is None else round(steering_angle, 5),
                "inference_ms": round(inference_ms, 2),
                "motor_output": False,
                "error": error,
            }
        )
        overlay = draw_perception_overlay(
            source, lane, metric_path, steering_angle, inference_ms, error
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

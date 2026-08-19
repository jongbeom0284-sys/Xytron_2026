from __future__ import annotations

import json
import sys
import time
import types
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from .twinmixing_center import (
    TwinMixingCenterConfig,
    extract_center_path,
    letterbox,
    render_overlay,
    restore_mask,
)
from .geometry import move_towards


def _install_optional_thop_stub() -> None:
    if "thop" in sys.modules:
        return
    module = types.ModuleType("thop")

    def unavailable(*_args: object, **_kwargs: object) -> None:
        raise RuntimeError("thop profiling is not available in vehicle inference")

    module.profile = unavailable
    sys.modules["thop"] = module


class TwinMixingCenterNode(Node):
    """CUDA TwinMixing preview that follows the gap between two drive areas."""

    def __init__(self) -> None:
        super().__init__("twinmixing_center_drive")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._torch, self._model, self._device_name = self._load_model()
        self._config = TwinMixingCenterConfig(
            roi_top_ratio=float(self.get_parameter("roi_top_ratio").value),
            roi_bottom_ratio=float(self.get_parameter("roi_bottom_ratio").value),
            row_step=int(self.get_parameter("row_step").value),
            search_half_width_ratio=float(
                self.get_parameter("search_half_width_ratio").value
            ),
            minimum_gap_width_ratio=float(
                self.get_parameter("minimum_gap_width_ratio").value
            ),
            maximum_gap_width_ratio=float(
                self.get_parameter("maximum_gap_width_ratio").value
            ),
            minimum_side_support_ratio=float(
                self.get_parameter("minimum_side_support_ratio").value
            ),
            minimum_points=int(self.get_parameter("minimum_points").value),
            lookahead_y_ratio=float(self.get_parameter("lookahead_y_ratio").value),
            minimum_region_width_ratio=float(
                self.get_parameter("minimum_region_width_ratio").value
            ),
            region_search_half_width_ratio=float(
                self.get_parameter("region_search_half_width_ratio").value
            ),
        )
        self._minimum_period = 1.0 / max(
            1.0, float(self.get_parameter("max_processing_hz").value)
        )
        debug_hz = float(self.get_parameter("debug_publish_hz").value)
        self._debug_period = 0.0 if debug_hz <= 0.0 else 1.0 / debug_hz
        self._last_started_at = 0.0
        self._last_debug_at = 0.0
        self._last_control_at = 0.0
        self._last_valid_at = 0.0
        self._steering_rad = 0.0
        self._last_valid_preview: tuple[float, float, float] | None = None
        self._fps = 0.0

        self._debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_image_topic").value), 1
        )
        self._status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10
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
            "TwinMixing CUDA center-line controller is preview-only; no motor publisher"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter(
            "debug_image_topic", "/kobot_lane_drive/twinmixing_center/debug_image"
        )
        self.declare_parameter(
            "status_topic", "/kobot_lane_drive/twinmixing_center/status"
        )
        self.declare_parameter(
            "drive_preview_topic", "/kobot_lane_drive/twinmixing_center/drive_preview"
        )
        self.declare_parameter("max_processing_hz", 15.0)
        self.declare_parameter("debug_publish_hz", 7.0)
        self.declare_parameter("variant", "large")
        self.declare_parameter("use_fp16", True)
        self.declare_parameter("roi_top_ratio", 0.46)
        self.declare_parameter("roi_bottom_ratio", 0.97)
        self.declare_parameter("row_step", 4)
        self.declare_parameter("search_half_width_ratio", 0.24)
        self.declare_parameter("minimum_gap_width_ratio", 0.004)
        self.declare_parameter("maximum_gap_width_ratio", 0.18)
        self.declare_parameter("minimum_side_support_ratio", 0.02)
        self.declare_parameter("minimum_points", 18)
        self.declare_parameter("lookahead_y_ratio", 0.67)
        self.declare_parameter("minimum_region_width_ratio", 0.12)
        self.declare_parameter("region_search_half_width_ratio", 0.40)
        self.declare_parameter("minimum_confidence", 0.35)
        self.declare_parameter("speed_mps", 0.50)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_kp", 0.42)
        self.declare_parameter("steering_output_gain", 4.0)
        self.declare_parameter("steering_filter_alpha", 0.35)
        self.declare_parameter("max_steering_rad", 0.476)
        self.declare_parameter("max_steering_rate_rad_s", 1.20)

    def _load_model(self):
        import torch

        if not torch.cuda.is_available():
            raise RuntimeError("CUDA PyTorch is required but torch.cuda.is_available() is false")
        share = Path(get_package_share_directory("kobot_lane_drive"))
        vendor_root = share / "third_party" / "TwinMixing"
        variant = str(self.get_parameter("variant").value)
        checkpoint_path = vendor_root / "pretrained" / f"{variant}.pth"
        if not checkpoint_path.is_file():
            raise RuntimeError(f"TwinMixing checkpoint not found: {checkpoint_path}")
        _install_optional_thop_stub()
        sys.path.insert(0, str(vendor_root))
        from models.twinmixing import TwinMixing

        model = TwinMixing(SimpleNamespace(type=variant, sell=False, seda=False))
        checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        model.load_state_dict(checkpoint, strict=True)
        model = model.cuda().eval()
        if bool(self.get_parameter("use_fp16").value):
            model = model.half()
        torch.backends.cudnn.benchmark = True
        device_name = torch.cuda.get_device_name(0)
        self.get_logger().info(
            f"Loaded TwinMixing {variant} on {device_name}; fp16={next(model.parameters()).dtype}"
        )
        return torch, model, device_name

    def _preprocess(self, frame: np.ndarray):
        widescreen = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_LINEAR)
        padded, padding = letterbox(widescreen)
        rgb_chw = padded[:, :, ::-1].transpose(2, 0, 1)
        tensor = self._torch.from_numpy(np.ascontiguousarray(rgb_chw)).cuda()
        tensor = tensor.half() if bool(self.get_parameter("use_fp16").value) else tensor.float()
        return tensor.div_(255.0).unsqueeze(0), padding

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
        self._steering_rad = move_towards(
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
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            tensor, padding = self._preprocess(frame)
            self._torch.cuda.synchronize()
            inference_started = time.perf_counter()
            with self._torch.inference_mode():
                driving_logits, lane_logits = self._model(tensor)
            self._torch.cuda.synchronize()
            inference_ms = (time.perf_counter() - inference_started) * 1000.0
            instant_fps = 1000.0 / max(0.001, inference_ms)
            self._fps = instant_fps if self._fps <= 0.0 else 0.85 * self._fps + 0.15 * instant_fps
            driving_raw = driving_logits.argmax(1)[0].byte().cpu().numpy()
            lane_raw = lane_logits.argmax(1)[0].byte().cpu().numpy()
            output_size = (frame.shape[1], frame.shape[0])
            driving = restore_mask(driving_raw, padding, output_size)
            lane = restore_mask(lane_raw, padding, output_size)
            result = extract_center_path(driving, lane, self._config)
            minimum_confidence = float(self.get_parameter("minimum_confidence").value)
            if result.found and result.confidence >= minimum_confidence:
                steering = self._tracking_steering(
                    float(result.center_offset_px), frame.shape[1], now
                )
                preview = (
                    steering,
                    float(self.get_parameter("speed_mps").value),
                    result.confidence,
                )
                self._last_valid_preview = preview
                self._last_valid_at = now
                mode = "TRACKING"
                reason = (
                    "twinmixing_region_center"
                    if result.path_source == "REGION_CENTER"
                    else "twinmixing_center_gap"
                )
            elif self._last_valid_preview is not None:
                preview = self._last_valid_preview
                steering = preview[0]
                mode = "HOLD_LAST"
                reason = "hold_last_without_timeout"
            else:
                preview = (0.0, 0.0, 0.0)
                steering = 0.0
                mode = "WAITING"
                reason = "center_gap_not_found"
            payload = {
                "state": result.state,
                "mode": mode,
                "path_source": result.path_source,
                "confidence": round(result.confidence, 4),
                "center_offset_px": (
                    round(float(result.center_offset_px), 2)
                    if result.center_offset_px is not None
                    else None
                ),
                "lookahead_x_px": result.lookahead[0] if result.lookahead else None,
                "lookahead_y_px": result.lookahead[1] if result.lookahead else None,
                "heading_deg": (
                    round(float(result.heading_deg), 2)
                    if result.heading_deg is not None
                    else None
                ),
                "center_gap_width_px": (
                    round(float(result.mean_gap_width_px), 2)
                    if result.mean_gap_width_px is not None
                    else None
                ),
                "lane_support_ratio": round(result.lane_support_ratio, 3),
                "path_points": int(len(result.fitted_points)),
                "steering_rad": round(float(steering), 5),
                "speed_mps": round(float(preview[1]), 3),
                "inference_ms": round(inference_ms, 2),
                "inference_fps": round(self._fps, 2),
                "device": self._device_name,
                "runtime": "pytorch_cuda_fp16" if bool(self.get_parameter("use_fp16").value) else "pytorch_cuda_fp32",
                "motor_output": False,
                "reason": reason,
            }
            self._publish(payload, preview)
            if self._debug_period > 0.0 and now - self._last_debug_at >= self._debug_period:
                overlay = render_overlay(
                    frame,
                    driving,
                    lane,
                    result,
                    float(steering),
                    inference_ms,
                    self._fps,
                    self._device_name,
                    mode,
                )
                debug = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
                debug.header = message.header
                self._debug_publisher.publish(debug)
                self._last_debug_at = now
        except Exception as error:  # noqa: BLE001 - fail closed on inference errors
            payload = {
                "state": "ERROR",
                "mode": "ERROR",
                "confidence": 0.0,
                "steering_rad": 0.0,
                "speed_mps": 0.0,
                "motor_output": False,
                "runtime": "pytorch_cuda",
                "error": str(error),
            }
            self._publish(payload, (0.0, 0.0, 0.0))
            self.get_logger().error(f"TwinMixing CUDA inference failed: {error}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinMixingCenterNode()
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

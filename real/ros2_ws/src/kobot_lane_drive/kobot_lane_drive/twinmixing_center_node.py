"""TwinMixing 인식 + 노란 중앙선 우선 주행 판단 노드.

주행 목표선은 다음 순서로 고른다.

  1. YELLOW_LINE    노란 중앙선 (색으로 확인된 것)
  2. CENTER_GAP     두 주행 영역 사이의 틈
  3. REGION_CENTER  주행 영역이 하나면 그 중앙
  4. HOLD/WAITING   마지막 유효값 유지, 이후 정지

목표선이 정해지면 그 좌우 차선 폭을 재고, 한쪽이 장애물 차량에 막혔으면 반대편
차선 중앙으로 비켜섰다가 추월 후 중앙으로 돌아온다. 점유 판정은 카메라와 라이다가
모두 동의할 때만 확정된다.

이 노드에는 모터 publisher 가 없다. 실제 출력은 승인 잠금 bridge 한 곳뿐이다.
"""

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
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32MultiArray, String

from .geometry import move_towards
from .lane_policy import (
    EMPTY_OCCUPANCY,
    LanePolicy,
    LanePolicyConfig,
    measure_lanes,
)
from .obstacle_lidar import EMPTY_SECTORS, LidarSectorConfig, summarize_scan
from .twinmixing_center import (
    TwinMixingCenterConfig,
    extract_center_path,
    letterbox,
    render_overlay,
    restore_mask,
)
from .yellow_center import (
    YellowCenterConfig,
    YellowCenterTracker,
    extract_yellow_center,
)


def _install_optional_thop_stub() -> None:
    if "thop" in sys.modules:
        return
    module = types.ModuleType("thop")

    def unavailable(*_args: object, **_kwargs: object) -> None:
        raise RuntimeError("thop profiling is not available in vehicle inference")

    module.profile = unavailable
    sys.modules["thop"] = module


class TwinMixingCenterNode(Node):
    """노란 중앙선 우선 주행 판단. 모터 publisher 없음."""

    def __init__(self) -> None:
        super().__init__("twinmixing_center_drive")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._torch, self._model, self._device_name = self._load_model()

        self._config = self._build_center_config()
        self._yellow_config = self._build_yellow_config()
        self._policy_config = self._build_policy_config()
        self._lidar_config = self._build_lidar_config()

        self._yellow_tracker = YellowCenterTracker(self._yellow_config)
        self._policy = LanePolicy(self._policy_config)
        self._sectors = EMPTY_SECTORS

        self._minimum_period = 1.0 / max(
            1.0, float(self.get_parameter("max_processing_hz").value)
        )
        debug_hz = float(self.get_parameter("debug_publish_hz").value)
        self._debug_period = 0.0 if debug_hz <= 0.0 else 1.0 / debug_hz
        self._last_started_at = 0.0
        self._last_debug_at = 0.0
        self._last_control_at = 0.0
        self._last_policy_at = 0.0
        self._steering_rad = 0.0
        self._last_speed_mps = 0.0
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
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("lidar_topic").value),
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().warning(
            "TwinMixing yellow-center controller is preview-only; no motor publisher"
        )

    # ------------------------------------------------------------------ 설정

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("lidar_topic", "/scan")
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

        # 중심 경로 추출 (2·3단계)
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

        # 노란 중앙선 (1단계). 조명이 바뀌면 재빌드 없이 여기만 고친다.
        self.declare_parameter("yellow_enabled", True)
        self.declare_parameter("yellow_hue_low", 15)
        self.declare_parameter("yellow_hue_high", 45)
        self.declare_parameter("yellow_saturation_low", 55)
        self.declare_parameter("yellow_saturation_high", 255)
        self.declare_parameter("yellow_value_low", 110)
        self.declare_parameter("yellow_value_high", 255)
        self.declare_parameter("yellow_lane_dilate_px", 5)
        self.declare_parameter("yellow_open_kernel_px", 3)
        self.declare_parameter("yellow_search_half_width_ratio", 0.34)
        self.declare_parameter("yellow_maximum_run_width_ratio", 0.22)
        self.declare_parameter("yellow_minimum_run_pixels", 3)
        self.declare_parameter("yellow_minimum_rows", 8)
        self.declare_parameter("yellow_ghost_maximum_frames", 20)
        self.declare_parameter("yellow_history_size", 10)
        self.declare_parameter("yellow_minimum_confidence", 0.30)

        # 차선 점유와 회피
        self.declare_parameter("maneuvers_enabled", False)
        self.declare_parameter("lane_sample_rows", 14)
        self.declare_parameter("lane_roi_top_ratio", 0.52)
        self.declare_parameter("lane_roi_bottom_ratio", 0.95)
        self.declare_parameter("lane_minimum_width_ratio", 0.05)
        self.declare_parameter("lane_narrow_width_ratio", 0.55)
        self.declare_parameter("lane_narrow_rows_required", 4)
        self.declare_parameter("lane_width_drop_ratio", 0.60)
        self.declare_parameter("lane_baseline_alpha", 0.05)
        self.declare_parameter("lane_baseline_warmup_frames", 20)
        self.declare_parameter("policy_enter_hold_sec", 0.30)
        self.declare_parameter("policy_clear_hold_sec", 0.40)
        self.declare_parameter("policy_pass_distance_m", 0.70)
        self.declare_parameter("policy_target_rate_px_s", 260.0)
        self.declare_parameter("policy_shift_speed_scale", 0.75)
        self.declare_parameter("policy_return_tolerance_px", 12.0)
        self.declare_parameter("policy_straight_only", True)
        self.declare_parameter("policy_maximum_heading_deg", 12.0)

        # 라이다 구역
        self.declare_parameter("lidar_angle_offset_rad", float(np.pi))
        self.declare_parameter("lidar_forward_minimum_m", 0.35)
        self.declare_parameter("lidar_forward_maximum_m", 2.50)
        self.declare_parameter("lidar_side_lateral_minimum_m", 0.12)
        self.declare_parameter("lidar_side_lateral_maximum_m", 0.75)
        self.declare_parameter("lidar_front_lateral_maximum_m", 0.20)
        self.declare_parameter("lidar_minimum_range_m", 0.15)
        self.declare_parameter("lidar_maximum_range_m", 6.0)
        self.declare_parameter("lidar_minimum_hits", 5)
        self.declare_parameter("lidar_confirm_distance_m", 1.60)
        self.declare_parameter("lidar_maximum_age_sec", 0.50)

        # 조향과 속도
        self.declare_parameter("speed_mps", 0.50)
        self.declare_parameter("steering_sign", -1.0)
        self.declare_parameter("steering_kp", 0.42)
        self.declare_parameter("steering_output_gain", 4.0)
        self.declare_parameter("steering_filter_alpha", 0.35)
        self.declare_parameter("max_steering_rad", 0.476)
        self.declare_parameter("max_steering_rate_rad_s", 1.20)

    def _value(self, name: str):
        return self.get_parameter(name).value

    def _build_center_config(self) -> TwinMixingCenterConfig:
        return TwinMixingCenterConfig(
            roi_top_ratio=float(self._value("roi_top_ratio")),
            roi_bottom_ratio=float(self._value("roi_bottom_ratio")),
            row_step=int(self._value("row_step")),
            search_half_width_ratio=float(self._value("search_half_width_ratio")),
            minimum_gap_width_ratio=float(self._value("minimum_gap_width_ratio")),
            maximum_gap_width_ratio=float(self._value("maximum_gap_width_ratio")),
            minimum_side_support_ratio=float(self._value("minimum_side_support_ratio")),
            minimum_points=int(self._value("minimum_points")),
            lookahead_y_ratio=float(self._value("lookahead_y_ratio")),
            minimum_region_width_ratio=float(self._value("minimum_region_width_ratio")),
            region_search_half_width_ratio=float(
                self._value("region_search_half_width_ratio")
            ),
        )

    def _build_yellow_config(self) -> YellowCenterConfig:
        return YellowCenterConfig(
            hue_low=int(self._value("yellow_hue_low")),
            hue_high=int(self._value("yellow_hue_high")),
            saturation_low=int(self._value("yellow_saturation_low")),
            saturation_high=int(self._value("yellow_saturation_high")),
            value_low=int(self._value("yellow_value_low")),
            value_high=int(self._value("yellow_value_high")),
            roi_top_ratio=float(self._value("roi_top_ratio")),
            roi_bottom_ratio=float(self._value("roi_bottom_ratio")),
            row_step=int(self._value("row_step")),
            lane_dilate_px=int(self._value("yellow_lane_dilate_px")),
            open_kernel_px=int(self._value("yellow_open_kernel_px")),
            search_half_width_ratio=float(self._value("yellow_search_half_width_ratio")),
            maximum_run_width_ratio=float(self._value("yellow_maximum_run_width_ratio")),
            minimum_run_pixels=int(self._value("yellow_minimum_run_pixels")),
            minimum_rows=int(self._value("yellow_minimum_rows")),
            ghost_maximum_frames=int(self._value("yellow_ghost_maximum_frames")),
            history_size=int(self._value("yellow_history_size")),
        )

    def _build_policy_config(self) -> LanePolicyConfig:
        return LanePolicyConfig(
            sample_rows=int(self._value("lane_sample_rows")),
            roi_top_ratio=float(self._value("lane_roi_top_ratio")),
            roi_bottom_ratio=float(self._value("lane_roi_bottom_ratio")),
            minimum_lane_width_ratio=float(self._value("lane_minimum_width_ratio")),
            narrow_width_ratio=float(self._value("lane_narrow_width_ratio")),
            narrow_rows_required=int(self._value("lane_narrow_rows_required")),
            width_drop_ratio=float(self._value("lane_width_drop_ratio")),
            baseline_alpha=float(self._value("lane_baseline_alpha")),
            baseline_warmup_frames=int(self._value("lane_baseline_warmup_frames")),
            lidar_confirm_distance_m=float(self._value("lidar_confirm_distance_m")),
            lidar_maximum_age_sec=float(self._value("lidar_maximum_age_sec")),
            enter_hold_sec=float(self._value("policy_enter_hold_sec")),
            clear_hold_sec=float(self._value("policy_clear_hold_sec")),
            pass_distance_m=float(self._value("policy_pass_distance_m")),
            target_rate_px_s=float(self._value("policy_target_rate_px_s")),
            shift_speed_scale=float(self._value("policy_shift_speed_scale")),
            return_tolerance_px=float(self._value("policy_return_tolerance_px")),
            straight_only=bool(self._value("policy_straight_only")),
            maximum_heading_deg=float(self._value("policy_maximum_heading_deg")),
        )

    def _build_lidar_config(self) -> LidarSectorConfig:
        return LidarSectorConfig(
            angle_offset_rad=float(self._value("lidar_angle_offset_rad")),
            forward_minimum_m=float(self._value("lidar_forward_minimum_m")),
            forward_maximum_m=float(self._value("lidar_forward_maximum_m")),
            side_lateral_minimum_m=float(self._value("lidar_side_lateral_minimum_m")),
            side_lateral_maximum_m=float(self._value("lidar_side_lateral_maximum_m")),
            front_lateral_maximum_m=float(self._value("lidar_front_lateral_maximum_m")),
            minimum_range_m=float(self._value("lidar_minimum_range_m")),
            maximum_range_m=float(self._value("lidar_maximum_range_m")),
            minimum_hits=int(self._value("lidar_minimum_hits")),
        )

    # ------------------------------------------------------------------ 모델

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

    # ------------------------------------------------------------------ 제어

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

    # ------------------------------------------------------------------ 콜백

    def _on_scan(self, message: LaserScan) -> None:
        self._sectors = summarize_scan(
            message.ranges,
            float(message.angle_min),
            float(message.angle_increment),
            time.monotonic(),
            self._lidar_config,
        )

    def _select_target_line(self, frame, driving, lane):
        """우선순위 1 → 2 → 3 순으로 목표선을 고른다."""
        yellow_evidence = None
        if bool(self.get_parameter("yellow_enabled").value):
            yellow_result, yellow_evidence = extract_yellow_center(
                frame, driving, lane, self._yellow_config, self._config
            )
            tracked = self._yellow_tracker.update(yellow_result, frame.shape[1])
            minimum = float(self.get_parameter("yellow_minimum_confidence").value)
            if tracked.found and tracked.confidence >= minimum:
                return tracked, yellow_evidence
        else:
            self._yellow_tracker.reset()
        return extract_center_path(driving, lane, self._config), yellow_evidence

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

            result, yellow_evidence = self._select_target_line(frame, driving, lane)

            width = frame.shape[1]
            base_speed = float(self.get_parameter("speed_mps").value)
            minimum_confidence = float(self.get_parameter("minimum_confidence").value)
            occupancy = EMPTY_OCCUPANCY
            decision = None

            if result.found and result.confidence >= minimum_confidence:
                dt = 0.05 if self._last_policy_at <= 0.0 else max(
                    0.001, min(0.50, now - self._last_policy_at)
                )
                self._last_policy_at = now
                occupancy = measure_lanes(driving, result.fitted_points, self._policy_config)
                decision = self._policy.update(
                    center_x_px=float(result.lookahead[0]),
                    occupancy=occupancy,
                    sectors=self._sectors,
                    speed_mps=self._last_speed_mps,
                    dt=dt,
                    now=now,
                    frame_width=width,
                    maneuvers_enabled=bool(self.get_parameter("maneuvers_enabled").value),
                    heading_deg=result.heading_deg,
                )
                offset = decision.target_x_px - width * 0.5
                steering = self._tracking_steering(offset, width, now)
                speed = base_speed * decision.speed_scale
                preview = (steering, speed, result.confidence)
                self._last_valid_preview = preview
                self._last_speed_mps = speed
                mode = "TRACKING"
                reason = decision.reason
            elif self._last_valid_preview is not None:
                preview = self._last_valid_preview
                steering = preview[0]
                mode = "HOLD_LAST"
                reason = "hold_last_without_timeout"
            else:
                preview = (0.0, 0.0, 0.0)
                steering = 0.0
                self._last_speed_mps = 0.0
                mode = "WAITING"
                reason = "target_line_not_found"

            payload = self._status_payload(
                result, occupancy, decision, mode, reason, steering, preview, inference_ms
            )
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
                    yellow_mask=yellow_evidence,
                    decision=decision,
                    occupancy=occupancy,
                    sectors=self._sectors,
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
            self.get_logger().error(f"TwinMixing inference failed: {error}")

    def _status_payload(
        self, result, occupancy, decision, mode, reason, steering, preview, inference_ms
    ) -> dict[str, object]:
        def rounded(value, digits=2):
            return None if value is None else round(float(value), digits)

        return {
            "state": result.state,
            "mode": mode,
            "path_source": result.path_source,
            "confidence": round(result.confidence, 4),
            "center_offset_px": rounded(result.center_offset_px),
            "lookahead_x_px": result.lookahead[0] if result.lookahead else None,
            "lookahead_y_px": result.lookahead[1] if result.lookahead else None,
            "heading_deg": rounded(result.heading_deg),
            "center_gap_width_px": rounded(result.mean_gap_width_px),
            "lane_support_ratio": round(result.lane_support_ratio, 3),
            "path_points": int(len(result.fitted_points)),
            "ghost_frames": self._yellow_tracker.blind_frames,
            "drive_line": decision.state if decision else self._policy.state,
            "blocked_side": decision.blocked_side if decision else None,
            "target_x_px": rounded(decision.target_x_px) if decision else None,
            "speed_scale": rounded(decision.speed_scale) if decision else None,
            "left_lane_width_px": rounded(occupancy.left_width_px),
            "right_lane_width_px": rounded(occupancy.right_width_px),
            "narrow_side": occupancy.narrow_side,
            "lidar_front_m": rounded(self._sectors.front_m),
            "lidar_left_m": rounded(self._sectors.left_m),
            "lidar_right_m": rounded(self._sectors.right_m),
            "maneuvers_enabled": bool(self.get_parameter("maneuvers_enabled").value),
            "steering_rad": round(float(steering), 5),
            "speed_mps": round(float(preview[1]), 3),
            "inference_ms": round(inference_ms, 2),
            "inference_fps": round(self._fps, 2),
            "device": self._device_name,
            "runtime": "pytorch_cuda_fp16"
            if bool(self.get_parameter("use_fp16").value)
            else "pytorch_cuda_fp32",
            "motor_output": False,
            "reason": reason,
        }


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

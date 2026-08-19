from __future__ import annotations

import math
import time


class Timer:
    def __init__(self) -> None:
        self.last = time.monotonic()

    def update(self) -> None:
        self.last = time.monotonic()

    def __call__(self) -> float:
        return time.monotonic() - self.last


class LaneController:
    """Small AuTURBO-inspired target-angle controller."""

    def __init__(
        self,
        default_speed: float = 8.0,
        curve_speed: float = 8.0,
        slow_speed: float = 8.0,
        lookahead_m: float = 0.4,
        compensate_steering_delay: bool = True,
        steering_prediction_horizon_s: float = 0.16,
        unity_fixed_dt_s: float = 0.02,
        unity_steering_speed: float = 2.0,
        steering_cmd_to_deg: float = 3.6,
        max_steering_cmd: float = 100.0,
        steering_bias_deg: float = 0.0,
        curve_steering_gain: float = 2.5,
        curve_steering_score_start_px: float = 40.0,
        curve_steering_score_full_px: float = 160.0,
        curve_steering_error_start_deg: float = 15.0,
    ) -> None:
        self.wheel_base_m = 0.24
        self.lookahead_m = float(lookahead_m)
        self.default_speed = float(default_speed)
        self.curve_speed = float(curve_speed)
        self.slow_speed = float(slow_speed)
        self.compensate_steering_delay = bool(compensate_steering_delay)
        self.steering_prediction_horizon_s = float(steering_prediction_horizon_s)
        self.unity_fixed_dt_s = float(unity_fixed_dt_s)
        self.unity_steering_speed = float(unity_steering_speed)
        self.steering_cmd_to_deg = float(steering_cmd_to_deg)
        self.max_steering_cmd = float(max_steering_cmd)
        self.steering_bias_deg = float(steering_bias_deg)
        self.curve_steering_gain = max(1.0, float(curve_steering_gain))
        self.curve_steering_score_start_px = float(curve_steering_score_start_px)
        self.curve_steering_score_full_px = max(
            self.curve_steering_score_start_px + 1.0,
            float(curve_steering_score_full_px),
        )
        self.curve_steering_error_start_deg = float(curve_steering_error_start_deg)
        self.current_steering_deg = 0.0
        self.last_target_steering_deg = 0.0
        self.last_update_time = time.monotonic()
        self.last_raw_angle_cmd = 0.0
        self.last_steering_gain = 1.0
        self.last_desired_steering_deg = 0.0
        self.last_compensated_angle_cmd = 0.0
        self.last_target_steering_needed_deg = 0.0

    def compute(
        self,
        angle_error_deg: float,
        confidence: float,
        force_slow: bool = False,
        curve_score_px: float = 0.0,
    ) -> tuple[float, float]:
        self._advance_steering_estimate()
        diff_angle = math.radians(self.steering_bias_deg - float(angle_error_deg))
        raw_angle_cmd = -math.degrees(
            math.atan2(2.0 * self.wheel_base_m * math.sin(diff_angle), self.lookahead_m)
        )
        gain = self._curve_steering_gain(float(angle_error_deg), float(curve_score_px))
        raw_angle_cmd *= gain
        raw_angle_cmd = max(-self.max_steering_cmd, min(self.max_steering_cmd, raw_angle_cmd))
        self.last_steering_gain = float(gain)
        self.last_raw_angle_cmd = float(raw_angle_cmd)
        angle_cmd = self._compensate_angle(raw_angle_cmd)

        if confidence <= 0.0:
            speed = 0.0
        elif confidence < 0.35 or force_slow:
            speed = self.slow_speed
        elif abs(raw_angle_cmd) > 22.0:
            speed = self.curve_speed
        else:
            speed = self.default_speed
        return float(angle_cmd), float(speed)

    def _curve_steering_gain(self, angle_error_deg: float, curve_score_px: float) -> float:
        score_weight = (
            (curve_score_px - self.curve_steering_score_start_px)
            / (self.curve_steering_score_full_px - self.curve_steering_score_start_px)
        )
        error_denominator = max(1.0, 50.0 - self.curve_steering_error_start_deg)
        error_weight = (abs(angle_error_deg) - self.curve_steering_error_start_deg) / error_denominator
        weight = max(0.0, min(1.0, max(score_weight, error_weight)))
        return 1.0 + (self.curve_steering_gain - 1.0) * weight

    def _advance_steering_estimate(self) -> None:
        now = time.monotonic()
        elapsed = max(0.0, now - self.last_update_time)
        self.last_update_time = now
        remain = self._steering_remain(elapsed)
        self.current_steering_deg = self.last_target_steering_deg + (
            self.current_steering_deg - self.last_target_steering_deg
        ) * remain

    def _compensate_angle(self, raw_angle_cmd: float) -> float:
        desired_steering_deg = float(raw_angle_cmd) * self.steering_cmd_to_deg
        self.last_desired_steering_deg = desired_steering_deg
        if not self.compensate_steering_delay:
            angle_cmd = float(raw_angle_cmd)
            target_steering_deg = angle_cmd * self.steering_cmd_to_deg
        else:
            remain = self._steering_remain(max(0.0, self.steering_prediction_horizon_s))
            response = max(1e-6, 1.0 - remain)
            target_steering_deg = (desired_steering_deg - remain * self.current_steering_deg) / response
            angle_cmd = target_steering_deg / self.steering_cmd_to_deg
        angle_cmd = max(-self.max_steering_cmd, min(self.max_steering_cmd, angle_cmd))
        target_steering_deg = angle_cmd * self.steering_cmd_to_deg
        self.last_compensated_angle_cmd = float(angle_cmd)
        self.last_target_steering_needed_deg = float(target_steering_deg)
        self.last_target_steering_deg = float(target_steering_deg)
        return float(angle_cmd)

    def observe_command(self, angle_cmd: float) -> None:
        self._advance_steering_estimate()
        angle_cmd = max(-self.max_steering_cmd, min(self.max_steering_cmd, float(angle_cmd)))
        self.last_target_steering_deg = angle_cmd * self.steering_cmd_to_deg

    def _steering_remain(self, seconds: float) -> float:
        fixed_dt = max(1e-6, self.unity_fixed_dt_s)
        alpha = max(0.0, min(1.0, self.unity_steering_speed * fixed_dt))
        steps = max(0.0, seconds / fixed_dt)
        return float((1.0 - alpha) ** steps)

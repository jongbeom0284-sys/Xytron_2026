from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FastLaneControlConfig:
    image_width_px: float = 640.0
    max_speed_mps: float = 0.70
    single_lane_max_speed_mps: float = 0.48
    minimum_tracking_speed_mps: float = 0.28
    lost_hold_speed_mps: float = 0.30
    max_steering_rad: float = 0.34
    max_steering_rate_rad_s: float = 1.20
    steering_sign: float = -1.0
    steering_kp: float = 0.42
    steering_kd: float = 0.003
    steering_filter_alpha: float = 0.35
    single_lane_hold_sec: float = 0.30
    offset_filter_alpha_both: float = 0.50
    offset_filter_alpha_single: float = 0.15
    max_offset_step_px: float = 35.0
    minimum_confidence: float = 0.20
    curve_slowdown_ratio: float = 0.55
    invalid_grace_sec: float = 0.25
    invalid_stop_sec: float = 0.50


@dataclass(frozen=True)
class LaneObservation:
    state: str
    confidence: float
    offset_px: float | None
    mask_rejected: bool = False


@dataclass(frozen=True)
class FastLaneCommand:
    mode: str
    steering_rad: float
    speed_mps: float
    source_state: str
    confidence: float
    reason: str


def _move_towards(current: float, target: float, maximum_step: float) -> float:
    if target > current:
        return min(target, current + maximum_step)
    return max(target, current - maximum_step)


class FastLaneController:
    """Temporal pixel-path controller with no ROS or motor dependency."""

    VALID_STATES = {"BOTH", "LEFT_ONLY", "RIGHT_ONLY"}

    def __init__(self, config: FastLaneControlConfig | None = None) -> None:
        self.config = config or FastLaneControlConfig()
        self._last_step_at: float | None = None
        self._last_valid_at: float | None = None
        self._previous_error: float | None = None
        self._filtered_offset_px: float | None = None
        self._last_both_at: float | None = None
        self._last_both_offset_px: float | None = None
        self._steering_rad = 0.0
        self._last_tracking_speed_mps = 0.0

    def step(
        self,
        observation: LaneObservation | None,
        now_sec: float,
    ) -> FastLaneCommand:
        config = self.config
        if self._last_step_at is None:
            dt = 0.05
        else:
            dt = max(0.001, min(0.20, now_sec - self._last_step_at))
        self._last_step_at = now_sec

        valid = (
            observation is not None
            and observation.state in self.VALID_STATES
            and observation.offset_px is not None
            and math.isfinite(observation.offset_px)
            and math.isfinite(observation.confidence)
            and observation.confidence >= config.minimum_confidence
            and not observation.mask_rejected
        )
        if valid and observation is not None:
            return self._tracking_command(observation, now_sec, dt)
        return self._invalid_command(observation, now_sec, dt)

    def _tracking_command(
        self,
        observation: LaneObservation,
        now_sec: float,
        dt: float,
    ) -> FastLaneCommand:
        config = self.config
        raw_offset_px = float(observation.offset_px)
        if observation.state == "BOTH":
            self._last_both_at = now_sec
            self._last_both_offset_px = raw_offset_px
        elif (
            self._last_both_at is not None
            and self._last_both_offset_px is not None
            and now_sec - self._last_both_at <= config.single_lane_hold_sec
        ):
            # A momentary BOTH -> single transition should not jump to the
            # reconstructed single-lane center in one frame.
            raw_offset_px = self._last_both_offset_px

        if self._filtered_offset_px is None:
            self._filtered_offset_px = raw_offset_px
        else:
            offset_delta = max(
                -config.max_offset_step_px,
                min(
                    config.max_offset_step_px,
                    raw_offset_px - self._filtered_offset_px,
                ),
            )
            offset_alpha = (
                config.offset_filter_alpha_both
                if observation.state == "BOTH"
                else config.offset_filter_alpha_single
            )
            self._filtered_offset_px += offset_alpha * offset_delta

        half_width = max(1.0, config.image_width_px * 0.5)
        error = self._filtered_offset_px / half_width
        derivative = (
            0.0
            if self._previous_error is None
            else (error - self._previous_error) / dt
        )
        self._previous_error = error

        raw_steering = config.steering_sign * (
            config.steering_kp * error + config.steering_kd * derivative
        )
        raw_steering = float(
            max(-config.max_steering_rad, min(config.max_steering_rad, raw_steering))
        )
        filtered_target = self._steering_rad + config.steering_filter_alpha * (
            raw_steering - self._steering_rad
        )
        self._steering_rad = _move_towards(
            self._steering_rad,
            filtered_target,
            config.max_steering_rate_rad_s * dt,
        )

        state_speed = (
            config.max_speed_mps
            if observation.state == "BOTH"
            else min(config.max_speed_mps, config.single_lane_max_speed_mps)
        )
        confidence_unit = max(
            0.0,
            min(
                1.0,
                (observation.confidence - config.minimum_confidence)
                / max(0.001, 1.0 - config.minimum_confidence),
            ),
        )
        confidence_factor = 0.70 + 0.30 * confidence_unit
        steering_fraction = min(
            1.0, abs(self._steering_rad) / max(0.001, config.max_steering_rad)
        )
        curve_factor = 1.0 - config.curve_slowdown_ratio * steering_fraction
        speed = state_speed * confidence_factor * curve_factor
        speed = min(
            state_speed,
            max(config.minimum_tracking_speed_mps, speed),
        )

        self._last_valid_at = now_sec
        self._last_tracking_speed_mps = speed
        return FastLaneCommand(
            mode="TRACKING",
            steering_rad=self._steering_rad,
            speed_mps=speed,
            source_state=observation.state,
            confidence=float(observation.confidence),
            reason="valid_lane",
        )

    def _invalid_command(
        self,
        observation: LaneObservation | None,
        now_sec: float,
        dt: float,
    ) -> FastLaneCommand:
        config = self.config
        source_state = "STALE" if observation is None else observation.state
        confidence = 0.0 if observation is None else float(observation.confidence)
        reason = (
            "mask_rejected"
            if observation is not None and observation.mask_rejected
            else "lane_invalid"
        )
        elapsed = (
            math.inf
            if self._last_valid_at is None
            else max(0.0, now_sec - self._last_valid_at)
        )

        if elapsed <= config.invalid_grace_sec:
            return FastLaneCommand(
                mode="GRACE",
                steering_rad=self._steering_rad,
                speed_mps=min(
                    self._last_tracking_speed_mps,
                    config.lost_hold_speed_mps,
                ),
                source_state=source_state,
                confidence=confidence,
                reason=reason,
            )

        if elapsed <= config.invalid_stop_sec:
            span = max(0.001, config.invalid_stop_sec - config.invalid_grace_sec)
            fraction = max(0.0, (config.invalid_stop_sec - elapsed) / span)
            return FastLaneCommand(
                mode="DECEL",
                steering_rad=self._steering_rad,
                speed_mps=min(
                    self._last_tracking_speed_mps,
                    config.lost_hold_speed_mps,
                )
                * fraction,
                source_state=source_state,
                confidence=confidence,
                reason=reason,
            )

        self._steering_rad = _move_towards(
            self._steering_rad,
            0.0,
            config.max_steering_rate_rad_s * dt,
        )
        self._previous_error = None
        self._filtered_offset_px = None
        self._last_both_at = None
        self._last_both_offset_px = None
        return FastLaneCommand(
            mode="STOPPED",
            steering_rad=self._steering_rad,
            speed_mps=0.0,
            source_state=source_state,
            confidence=confidence,
            reason=reason,
        )

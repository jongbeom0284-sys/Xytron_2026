from __future__ import annotations


def make_legacy_command(
    steering_rad: float | None,
    speed_mps: float,
    angle_scale: float,
    speed_scale: float,
    ready: bool,
    fresh: bool,
) -> list[float]:
    if not ready or not fresh or steering_rad is None:
        return [0.0, 0.0]
    if angle_scale <= 0.0 or speed_scale <= 0.0:
        raise ValueError("motor adapter scales must be positive")
    legacy_angle = max(-70.0, min(70.0, -steering_rad / angle_scale))
    legacy_speed = max(0.0, min(50.0, speed_mps / speed_scale))
    return [legacy_angle, legacy_speed]

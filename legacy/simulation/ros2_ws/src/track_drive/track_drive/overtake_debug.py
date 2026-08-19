#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OvertakeDebugNode(Node):
    def __init__(self):
        super().__init__("overtake_debug")
        self.declare_parameter("verbose", False)
        self.declare_parameter("repeat_period_s", 0.5)
        self.verbose = bool(self.get_parameter("verbose").value)
        self.repeat_period_s = max(0.1, float(self.get_parameter("repeat_period_s").value))
        self.create_subscription(String, "/track_drive/debug", self._cb, 10)
        self._last_key = None
        self._last_print_time = 0.0
        mode = "verbose" if self.verbose else "lidar avoid only"
        self.get_logger().info(f"overtake_debug: watching /track_drive/debug ({mode})")

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("debug json parse failed")
            return

        if not self.verbose:
            now = time.monotonic()
            key = (
                data.get("lidar_avoid_target_side"),
                data.get("lidar_avoid_blocked_side"),
            )
            line = self._avoid_action_line(data)
            if line and (key != self._last_key or now - self._last_print_time >= self.repeat_period_s):
                self._last_key = key
                self._last_print_time = now
                self.get_logger().info(line)
            return

        key = (
            data.get("controller_state"),
            data.get("speed_mode"),
            data.get("object_detected"),
            data.get("object_direction"),
            data.get("overtake_state"),
            data.get("lidar_avoid_state"),
            data.get("lidar_avoid_target_side"),
            data.get("lidar_avoid_blocked_side"),
            data.get("lidar_avoid_verify_camera_side"),
            data.get("lidar_avoid_release_camera_side"),
            data.get("lidar_avoid_locked_until_checkerboard"),
            data.get("lidar_avoid_left_headlight_confirmed"),
            data.get("headlight_detected"),
            data.get("headlight_direction"),
            data.get("headlight_left_detected"),
            data.get("headlight_right_detected"),
            data.get("xycar_angle_cmd"),
            data.get("xycar_speed_cmd"),
        )
        if key == self._last_key:
            return
        self._last_key = key

        lines = [
            self._control_line(data),
            self._vehicle_line(data),
            self._avoid_line(data),
            self._headlight_line(data),
            self._motor_line(data),
        ]
        self.get_logger().info("\n" + "\n".join(lines))

    @staticmethod
    def _fmt(value, suffix=""):
        if value is None:
            return "none"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return str(value)
        if not math.isfinite(number):
            return "none"
        return f"{number:.2f}{suffix}"

    def _control_line(self, data):
        return (
            f"CONTROL state={data.get('controller_state')} speed_mode={data.get('speed_mode')} "
            f"lane_conf={self._fmt(data.get('lane_confidence'))} "
            f"front={self._fmt(data.get('overtake_front_distance_m'), 'm')}"
        )

    def _vehicle_line(self, data):
        detected = bool(data.get("object_detected"))
        direction = data.get("object_direction", "none")
        overtake_state = data.get("overtake_state", "idle")
        offset = data.get("overtake_offset_px", 0)
        if detected:
            return f"VEHICLE camera_detected=yes direction={direction} overtake_state={overtake_state} offset_px={offset}"
        return f"VEHICLE camera_detected=no direction={direction} overtake_state={overtake_state} offset_px={offset}"

    def _avoid_line(self, data):
        state = data.get("lidar_avoid_state", "idle")
        target = data.get("lidar_avoid_target_side", "middle")
        blocked = data.get("lidar_avoid_blocked_side", "none")
        verify = data.get("lidar_avoid_verify_camera_side", "none")
        release = data.get("lidar_avoid_release_camera_side", "none")
        locked = bool(data.get("lidar_avoid_locked_until_checkerboard"))
        if target in {"left", "right"}:
            action = f"{blocked.upper()} BLOCKED -> GO {target.upper()}"
        else:
            action = "NO LIDAR AVOID"
        return (
            f"LIDAR_AVOID {action} state={state} verify_cam={verify} "
            f"release_cam={release} locked_until_checkerboard={locked}"
        )

    def _avoid_action_line(self, data):
        target = data.get("lidar_avoid_target_side", "middle")
        blocked = data.get("lidar_avoid_blocked_side", "none")
        if target not in {"left", "right"}:
            return ""
        return f"LIDAR_AVOID {blocked.upper()} BLOCKED -> GO {target.upper()}"

    def _headlight_line(self, data):
        detected = bool(data.get("headlight_detected"))
        left = bool(data.get("headlight_left_detected"))
        right = bool(data.get("headlight_right_detected"))
        direction = data.get("headlight_direction", "none")
        camera = data.get("headlight_effective_camera_side", data.get("headlight_camera_side", "none"))
        left_age = self._fmt(data.get("headlight_left_age_s"), "s")
        right_age = self._fmt(data.get("headlight_right_age_s"), "s")
        left_count = data.get("headlight_left_light_count", 0)
        right_count = data.get("headlight_right_light_count", 0)
        return (
            f"HEADLIGHT detected={detected} direction={direction} camera={camera} "
            f"left={left} age={left_age} lights={left_count} "
            f"right={right} age={right_age} lights={right_count}"
        )

    def _motor_line(self, data):
        return (
            f"MOTOR angle={self._fmt(data.get('xycar_angle_cmd'))} "
            f"speed={self._fmt(data.get('xycar_speed_cmd'))} "
            f"image_age={self._fmt(data.get('image_age_s'), 's')} "
            f"scan_age={self._fmt(data.get('scan_age_s'), 's')}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = OvertakeDebugNode()
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

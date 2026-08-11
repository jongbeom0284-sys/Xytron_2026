#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LidarAvoidDebugNode(Node):
    def __init__(self):
        super().__init__("lidar_avoid_debug")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("trigger_distance_m", 3.5)
        self.declare_parameter("clear_distance_m", 4.2)
        self.declare_parameter("side_min_deg", 8.0)
        self.declare_parameter("front_half_width_deg", 40.0)
        self.declare_parameter("front_width_deg", 30.0)
        self.declare_parameter("front_center_deg", 0.0)
        self.declare_parameter("positive_angle_side", "left")
        self.declare_parameter("preferred_side", "left")
        self.declare_parameter("min_valid_distance_m", 0.05)
        self.declare_parameter("ignore_self_reflection", True)
        self.declare_parameter("self_reflect_min_rel_deg", 8.0)
        self.declare_parameter("self_reflect_max_rel_deg", 24.0)
        self.declare_parameter("self_reflect_max_distance_m", 0.6)
        self.declare_parameter("report_period_s", 0.2)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.trigger_distance_m = float(self.get_parameter("trigger_distance_m").value)
        self.clear_distance_m = float(self.get_parameter("clear_distance_m").value)
        self.side_min_deg = float(self.get_parameter("side_min_deg").value)
        self.front_half_width_deg = float(self.get_parameter("front_half_width_deg").value)
        self.front_width_deg = float(self.get_parameter("front_width_deg").value)
        self.front_center_deg = float(self.get_parameter("front_center_deg").value)
        self.positive_angle_side = str(self.get_parameter("positive_angle_side").value).strip().lower()
        if self.positive_angle_side not in {"left", "right"}:
            self.positive_angle_side = "left"
        self.preferred_side = str(self.get_parameter("preferred_side").value).strip().lower()
        if self.preferred_side not in {"left", "right"}:
            self.preferred_side = "left"
        self.min_valid_distance_m = float(self.get_parameter("min_valid_distance_m").value)
        self.ignore_self_reflection = bool(self.get_parameter("ignore_self_reflection").value)
        self.self_reflect_min_rel_deg = float(self.get_parameter("self_reflect_min_rel_deg").value)
        self.self_reflect_max_rel_deg = float(self.get_parameter("self_reflect_max_rel_deg").value)
        self.self_reflect_max_distance_m = float(self.get_parameter("self_reflect_max_distance_m").value)
        self.report_period_s = max(0.05, float(self.get_parameter("report_period_s").value))

        self.last_scan: LaserScan | None = None
        self.last_scan_time = 0.0
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_timer(self.report_period_s, self.report)
        self.get_logger().info(
            "lidar_avoid_debug: passive /scan watcher only; no motor commands will be published"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_time = time.monotonic()

    def report(self) -> None:
        if self.last_scan is None:
            self.get_logger().info("waiting for /scan")
            return

        msg = self.last_scan
        max_side_distance = max(self.clear_distance_m, self.trigger_distance_m)
        front = self.front_distance(msg, self.front_width_deg, max_side_distance)
        left = self.side_distance(msg, "left", self.clear_distance_m)
        right = self.side_distance(msg, "right", self.clear_distance_m)

        front_blocked = front <= self.trigger_distance_m
        left_blocked = left <= self.trigger_distance_m
        right_blocked = right <= self.trigger_distance_m

        decision = "CLEAR"
        if left_blocked and not right_blocked:
            decision = "LEFT BLOCKED -> GO RIGHT"
        elif right_blocked and not left_blocked:
            decision = "RIGHT BLOCKED -> GO LEFT"
        elif front_blocked:
            target = self.choose_front_target(msg)
            decision = f"FRONT BLOCKED -> GO {target.upper()}"

        age = time.monotonic() - self.last_scan_time
        self.get_logger().info(
            "LIDAR_CHECK "
            f"decision={decision} "
            f"front={self.format_distance(front)} blocked={front_blocked} "
            f"left={self.format_distance(left)} blocked={left_blocked} "
            f"right={self.format_distance(right)} blocked={right_blocked} "
            f"dead=+-{self.side_min_deg:.0f}deg "
            f"age={age:.2f}s"
        )

    def choose_front_target(self, msg: LaserScan) -> str:
        left_clear = self.front_side_distance(msg, "left")
        right_clear = self.front_side_distance(msg, "right")
        left_open = left_clear >= self.trigger_distance_m or not math.isfinite(left_clear)
        right_open = right_clear >= self.trigger_distance_m or not math.isfinite(right_clear)
        if left_open and right_open:
            if math.isfinite(left_clear) and math.isfinite(right_clear) and abs(left_clear - right_clear) > 0.15:
                return "left" if left_clear > right_clear else "right"
            return self.preferred_side
        if left_open:
            return "left"
        if right_open:
            return "right"
        if math.isfinite(left_clear) and math.isfinite(right_clear):
            return "left" if left_clear >= right_clear else "right"
        return self.preferred_side

    def front_distance(self, msg: LaserScan, half_width_deg: float, max_distance_m: float) -> float:
        front_angle = math.radians(self.front_center_deg)
        half_width = math.radians(max(0.0, half_width_deg))
        distances: list[float] = []
        for idx, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if not self.valid_point(msg, idx, distance, max_distance_m):
                continue
            relative = self.wrap_angle_pi(msg.angle_min + idx * msg.angle_increment - front_angle)
            if abs(relative) > half_width:
                continue
            forward = distance * math.cos(relative)
            if forward > 0.0:
                distances.append(float(forward))
        return min(distances) if distances else float("inf")

    def side_distance(self, msg: LaserScan, side: str, max_distance_m: float) -> float:
        return self.distance_in_side_band(
            msg,
            side,
            self.side_min_deg,
            self.front_half_width_deg,
            max_distance_m,
        )

    def front_side_distance(self, msg: LaserScan, side: str) -> float:
        return self.distance_in_side_band(
            msg,
            side,
            self.side_min_deg,
            max(self.side_min_deg, self.front_width_deg),
            max(8.0, self.clear_distance_m),
        )

    def distance_in_side_band(
        self,
        msg: LaserScan,
        side: str,
        side_min_deg: float,
        side_max_deg: float,
        max_distance_m: float,
    ) -> float:
        front_angle = math.radians(self.front_center_deg)
        side_min = math.radians(max(0.0, side_min_deg))
        side_max = math.radians(max(side_min_deg, side_max_deg))
        distances: list[float] = []
        for idx, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if not self.valid_point(msg, idx, distance, max_distance_m):
                continue
            relative = self.wrap_angle_pi(msg.angle_min + idx * msg.angle_increment - front_angle)
            abs_relative = abs(relative)
            if not (side_min <= abs_relative <= side_max):
                continue
            relative_side = self.positive_angle_side if relative > 0.0 else (
                "right" if self.positive_angle_side == "left" else "left"
            )
            if relative_side != side:
                continue
            forward = distance * math.cos(relative)
            if forward > 0.0:
                distances.append(float(forward))
        return min(distances) if distances else float("inf")

    def valid_point(self, msg: LaserScan, idx: int, distance: float, max_distance_m: float) -> bool:
        if not math.isfinite(distance):
            return False
        if not (max(msg.range_min, self.min_valid_distance_m) < distance < min(msg.range_max, max_distance_m)):
            return False
        relative_deg = math.degrees(
            self.wrap_angle_pi(
                msg.angle_min + idx * msg.angle_increment - math.radians(self.front_center_deg)
            )
        )
        if (
            self.ignore_self_reflection
            and self.self_reflect_min_rel_deg <= relative_deg <= self.self_reflect_max_rel_deg
            and distance <= self.self_reflect_max_distance_m
        ):
            return False
        return True

    @staticmethod
    def wrap_angle_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    @staticmethod
    def format_distance(value: float) -> str:
        return f"{value:.2f}m" if math.isfinite(value) else "inf"


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidDebugNode()
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

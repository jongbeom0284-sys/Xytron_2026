#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish lidar candidate points for signal-control police detection.

The signal module consumes /lidar_cluster only.  This node is the preprocessing
boundary that converts /scan into a compact PointCloud2 point list.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class LidarClusterNode(Node):
    def __init__(self):
        super().__init__("lidar_cluster_node")
        self.declare_parameter("front_angle_offset", math.pi / 2.0)
        self.declare_parameter("y_sign", -1.0)
        self.declare_parameter("x_min", 0.05)
        self.declare_parameter("x_max", 8.00)
        self.declare_parameter("y_abs_max", 4.00)
        self.declare_parameter("range_min", 0.10)
        self.declare_parameter("range_max", 8.0)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.pub = self.create_publisher(PointCloud2, "/lidar_cluster", qos)
        self.create_subscription(LaserScan, "/scan", self._on_scan, qos_profile_sensor_data)
        self.get_logger().info("lidar_cluster_node ready: /scan -> /lidar_cluster")

    def _on_scan(self, msg: LaserScan) -> None:
        points: List[Tuple[float, float, float]] = []
        offset = float(self.get_parameter("front_angle_offset").value)
        y_sign = float(self.get_parameter("y_sign").value)
        x_min = float(self.get_parameter("x_min").value)
        x_max = float(self.get_parameter("x_max").value)
        y_abs_max = float(self.get_parameter("y_abs_max").value)
        r_min = float(self.get_parameter("range_min").value)
        r_max = float(self.get_parameter("range_max").value)

        for idx, raw_range in enumerate(msg.ranges):
            r = float(raw_range)
            if not math.isfinite(r) or r <= r_min or r >= r_max:
                continue
            theta = normalize_angle(float(msg.angle_min) + idx * float(msg.angle_increment) - offset)
            x = r * math.cos(theta)
            y = r * math.sin(theta) * y_sign
            if x_min <= x <= x_max and abs(y) <= y_abs_max:
                points.append((x, y, 0.0))

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id or "lidar_frame"
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(header, fields, points)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = LidarClusterNode()
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

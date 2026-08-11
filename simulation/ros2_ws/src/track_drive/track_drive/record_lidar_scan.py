#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan


def wrap_angle_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def angle_in_sector(angle_deg: float, center_deg: float, half_width_deg: float) -> bool:
    return abs(wrap_angle_deg(angle_deg - center_deg)) <= half_width_deg


class LidarRecorder(Node):
    def __init__(
        self,
        topic: str,
        duration_s: float,
        out_dir: Path,
        center_deg: float,
        half_width_deg: float,
        min_range_m: float,
        max_range_m: float,
        cluster_gap_deg: float,
        cluster_distance_gap_m: float,
        min_cluster_points: int,
    ) -> None:
        super().__init__("lidar_scan_recorder")
        self.topic = topic
        self.duration_s = max(0.1, float(duration_s))
        self.out_dir = out_dir
        self.center_deg = float(center_deg)
        self.half_width_deg = max(0.0, float(half_width_deg))
        self.min_range_m = max(0.0, float(min_range_m))
        self.max_range_m = max(self.min_range_m + 0.01, float(max_range_m))
        self.cluster_gap_deg = max(0.1, float(cluster_gap_deg))
        self.cluster_distance_gap_m = max(0.1, float(cluster_distance_gap_m))
        self.min_cluster_points = max(1, int(min_cluster_points))
        self.start_time = time.monotonic()
        self.scan_count = 0
        self.point_rows: list[dict[str, float | int | str]] = []
        self.cluster_rows: list[dict[str, float | int]] = []
        self.create_subscription(LaserScan, self.topic, self.scan_callback, qos_profile_sensor_data)

    def done(self) -> bool:
        return time.monotonic() - self.start_time >= self.duration_s

    def scan_callback(self, msg: LaserScan) -> None:
        elapsed = time.monotonic() - self.start_time
        scan_id = self.scan_count
        self.scan_count += 1
        sector_points: list[dict[str, float | int]] = []

        if msg.angle_increment == 0.0:
            return

        for idx, raw_distance in enumerate(msg.ranges):
            distance = float(raw_distance)
            if not math.isfinite(distance):
                continue
            valid_min = max(float(msg.range_min), self.min_range_m)
            valid_max = min(float(msg.range_max), self.max_range_m)
            if not (valid_min < distance < valid_max):
                continue

            angle_rad = float(msg.angle_min) + idx * float(msg.angle_increment)
            angle_deg = math.degrees(angle_rad)
            rel_deg = wrap_angle_deg(angle_deg - self.center_deg)
            in_sector = abs(rel_deg) <= self.half_width_deg
            forward_m = distance * math.cos(math.radians(rel_deg))
            lateral_m = distance * math.sin(math.radians(rel_deg))

            row = {
                "scan_id": scan_id,
                "time_s": round(elapsed, 4),
                "index": idx,
                "angle_deg": round(angle_deg, 4),
                "relative_deg": round(rel_deg, 4),
                "distance_m": round(distance, 4),
                "forward_m": round(forward_m, 4),
                "lateral_m": round(lateral_m, 4),
                "in_sector": int(in_sector),
            }
            self.point_rows.append(row)
            if in_sector and forward_m > 0.0:
                sector_points.append(
                    {
                        "idx": idx,
                        "relative_deg": rel_deg,
                        "distance_m": distance,
                        "forward_m": forward_m,
                        "lateral_m": lateral_m,
                    }
                )

        for cluster_id, cluster in enumerate(self.cluster_sector_points(sector_points)):
            if len(cluster) < self.min_cluster_points:
                continue
            distances = [float(p["distance_m"]) for p in cluster]
            forwards = [float(p["forward_m"]) for p in cluster]
            laterals = [float(p["lateral_m"]) for p in cluster]
            rel_angles = [float(p["relative_deg"]) for p in cluster]
            self.cluster_rows.append(
                {
                    "scan_id": scan_id,
                    "time_s": round(elapsed, 4),
                    "cluster_id": cluster_id,
                    "points": len(cluster),
                    "min_relative_deg": round(min(rel_angles), 4),
                    "max_relative_deg": round(max(rel_angles), 4),
                    "min_distance_m": round(min(distances), 4),
                    "mean_distance_m": round(sum(distances) / len(distances), 4),
                    "min_forward_m": round(min(forwards), 4),
                    "mean_forward_m": round(sum(forwards) / len(forwards), 4),
                    "mean_lateral_m": round(sum(laterals) / len(laterals), 4),
                    "min_lateral_m": round(min(laterals), 4),
                    "max_lateral_m": round(max(laterals), 4),
                    "width_lateral_m": round(max(laterals) - min(laterals), 4),
                }
            )

    def cluster_sector_points(self, points: list[dict[str, float | int]]) -> list[list[dict[str, float | int]]]:
        if not points:
            return []
        ordered = sorted(points, key=lambda p: float(p["relative_deg"]))
        clusters: list[list[dict[str, float | int]]] = [[ordered[0]]]
        for point in ordered[1:]:
            prev = clusters[-1][-1]
            angle_gap = abs(float(point["relative_deg"]) - float(prev["relative_deg"]))
            distance_gap = abs(float(point["distance_m"]) - float(prev["distance_m"]))
            if angle_gap <= self.cluster_gap_deg and distance_gap <= self.cluster_distance_gap_m:
                clusters[-1].append(point)
            else:
                clusters.append([point])

        if len(clusters) > 1:
            first = clusters[0]
            last = clusters[-1]
            wrap_gap = (
                float(first[0]["relative_deg"])
                + 2.0 * self.half_width_deg
                - float(last[-1]["relative_deg"])
            )
            distance_gap = abs(float(first[0]["distance_m"]) - float(last[-1]["distance_m"]))
            if wrap_gap <= self.cluster_gap_deg and distance_gap <= self.cluster_distance_gap_m:
                clusters[0] = last + first
                clusters.pop()
        return clusters

    def write_files(self) -> None:
        self.out_dir.mkdir(parents=True, exist_ok=True)
        point_path = self.out_dir / "points.csv"
        cluster_path = self.out_dir / "clusters.csv"
        summary_path = self.out_dir / "summary.json"

        point_fields = [
            "scan_id",
            "time_s",
            "index",
            "angle_deg",
            "relative_deg",
            "distance_m",
            "forward_m",
            "lateral_m",
            "in_sector",
        ]
        cluster_fields = [
            "scan_id",
            "time_s",
            "cluster_id",
            "points",
            "min_relative_deg",
            "max_relative_deg",
            "min_distance_m",
            "mean_distance_m",
            "min_forward_m",
            "mean_forward_m",
            "mean_lateral_m",
            "min_lateral_m",
            "max_lateral_m",
            "width_lateral_m",
        ]

        with point_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=point_fields)
            writer.writeheader()
            writer.writerows(self.point_rows)

        with cluster_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=cluster_fields)
            writer.writeheader()
            writer.writerows(self.cluster_rows)

        summary = {
            "topic": self.topic,
            "duration_s": self.duration_s,
            "scan_count": self.scan_count,
            "point_rows": len(self.point_rows),
            "cluster_rows": len(self.cluster_rows),
            "center_deg": self.center_deg,
            "half_width_deg": self.half_width_deg,
            "sector": f"{self.center_deg:.1f}+/-{self.half_width_deg:.1f}deg",
            "min_range_m": self.min_range_m,
            "max_range_m": self.max_range_m,
            "cluster_gap_deg": self.cluster_gap_deg,
            "cluster_distance_gap_m": self.cluster_distance_gap_m,
            "min_cluster_points": self.min_cluster_points,
            "files": {
                "points_csv": str(point_path),
                "clusters_csv": str(cluster_path),
                "summary_json": str(summary_path),
            },
        }
        summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        self.get_logger().info(f"wrote {point_path}")
        self.get_logger().info(f"wrote {cluster_path}")
        self.get_logger().info(f"wrote {summary_path}")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record LaserScan points for overtaking analysis.")
    parser.add_argument("--topic", default="/scan")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--out-dir", default="/tmp/track_drive_lidar_capture")
    parser.add_argument("--center-deg", type=float, default=0.0)
    parser.add_argument("--half-width-deg", type=float, default=40.0)
    parser.add_argument("--min-range", type=float, default=0.25)
    parser.add_argument("--max-range", type=float, default=12.0)
    parser.add_argument("--cluster-gap-deg", type=float, default=3.0)
    parser.add_argument("--cluster-distance-gap", type=float, default=2.5)
    parser.add_argument("--min-cluster-points", type=int, default=3)
    return parser.parse_args(argv)


def main() -> None:
    clean_argv = remove_ros_args(args=sys.argv)[1:]
    args = parse_args(clean_argv)
    rclpy.init(args=None)
    node = LidarRecorder(
        topic=args.topic,
        duration_s=args.duration,
        out_dir=Path(args.out_dir),
        center_deg=args.center_deg,
        half_width_deg=args.half_width_deg,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        cluster_gap_deg=args.cluster_gap_deg,
        cluster_distance_gap_m=args.cluster_distance_gap,
        min_cluster_points=args.min_cluster_points,
    )
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.write_files()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

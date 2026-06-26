from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .lidar_utils import detect_cones_in_corridor, scan_to_points


class ConeDebugNode(Node):
    def __init__(self) -> None:
        super().__init__("cone_debug")
        self.ranges = None
        self.create_subscription(LaserScan, "/scan", self._cb, qos_profile_sensor_data)
        self._left_wall_latched = False
        self._lost_frames = 0
        self.create_timer(0.5, self._report)
        self.get_logger().info("cone_debug ready; compare with lidar_scan while tuning")

    def _cb(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges

    def _sample_ring(self, ranges, n: int = 36) -> list[str]:
        step = max(1, len(ranges) // n)
        out = []
        for idx in range(0, len(ranges), step):
            distance = ranges[idx]
            if math.isfinite(distance):
                out.append(f"{distance:.2f}@{idx}")
        return out

    def _sector_min(self, ranges, i0: int, i1: int):
        vals = [ranges[idx] for idx in range(i0, i1) if idx < len(ranges) and math.isfinite(ranges[idx])]
        return min(vals) if vals else None

    def _report(self) -> None:
        if self.ranges is None:
            return

        ranges = self.ranges
        count = len(ranges)
        front_idx = count // 2

        left, right, dbg = detect_cones_in_corridor(ranges, force_left_wall=self._left_wall_latched)
        lc = dbg.get("left_cone")
        rc = dbg.get("right_cone")
        if dbg.get("follow_mode") == "left_wall":
            self._left_wall_latched = True
        elif dbg.get("left_turn", 0) > 0.14 and lc:
            self._left_wall_latched = True
        if dbg.get("mode") == "none":
            self._lost_frames += 1
        else:
            self._lost_frames = 0
        if self._lost_frames > 25:
            self._left_wall_latched = False

        lc_s = f"({lc.cx:.2f},{lc.cy:.2f})" if lc else "none"
        rc_s = f"({rc.cx:.2f},{rc.cy:.2f})" if rc else "none"

        path = dbg.get("path_targets") or []
        if path:
            wsum = sum(point.weight for point in path)
            px = sum(point.cx * point.weight for point in path) / wsum
            py = sum(point.cy * point.weight for point in path) / wsum
            path_s = f"({px:.2f},{py:.2f}) n={len(path)}"
        else:
            path_s = "none"

        front_value = ranges[front_idx] if front_idx < count and math.isfinite(ranges[front_idx]) else float("nan")
        self.get_logger().info(
            f"=== CONE DEBUG ===\n"
            f"  scan_len={count}  front[idx={front_idx}]={front_value:.2f}m\n"
            f"  mode={dbg.get('mode')}  follow={dbg.get('follow_mode')}  "
            f"left_turn={dbg.get('left_turn', 0):.2f}  "
            f"right_ok={dbg.get('right_reliable')}  "
            f"right_blocked={dbg.get('right_blocked')}  "
            f"latched={self._left_wall_latched}\n"
            f"  L={lc_s}  R={rc_s}  clusters L/R={dbg.get('left_clusters')}/{dbg.get('right_clusters')}\n"
            f"  path={path_s}\n"
            f"  sector_min  left[30:88]={self._fmt(self._sector_min(ranges, 30, 88))}  "
            f"right[92:210]={self._fmt(self._sector_min(ranges, 92, 210))}  "
            f"front[78:102]={self._fmt(self._sector_min(ranges, 78, 102))}\n"
            f"  sample(36): {self._sample_ring(ranges)}"
        )

        if lc:
            near_pts = [
                point
                for point in scan_to_points(ranges)
                if abs(point.x - lc.cx) < 0.6 and abs(point.y - lc.cy) < 0.8
            ]
            if near_pts:
                self.get_logger().info(
                    f"  left_wall_near: {len(near_pts)} pts  "
                    f"x=[{min(point.x for point in near_pts):.2f},{max(point.x for point in near_pts):.2f}]"
                )

    @staticmethod
    def _fmt(value) -> str:
        return f"{value:.2f}m" if value is not None else "none"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ConeDebugNode()
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

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from .lidar_utils import ConeCluster, PathTarget, center_path_distance, detect_cones_in_corridor


@dataclass
class ConeFollowerConfig:
    cruise_speed: float = 5.0
    min_speed: float = 3.0
    corner_speed: float = 3.5
    finish_speed: float = 3.0
    max_steer: float = 80.0
    pursuit_gain: float = 4.2
    lateral_gain: float = 18.0
    clearance_gain: float = 12.0
    corner_boost: float = 1.35
    smooth_alpha_straight: float = 0.40
    smooth_alpha_corner: float = 0.65
    lookahead_y_min: float = 0.45
    lookahead_y_max: float = 6.5
    lookahead_target_y: float = 3.0
    left_x_min: float = -3.5
    left_x_max: float = -0.55
    right_x_min: float = 0.55
    right_x_max: float = 3.2
    half_corridor: float = 2.25
    left_wall_half: float = 2.15
    left_wall_center_bias: float = 0.12
    front_slow_dist: float = 1.0
    front_stop_dist: float = 0.45
    clear_frames_required: int = 25
    min_tracked_frames: int = 80
    yellow_pixel_threshold: int = 2500
    yellow_roi_ratio: float = 0.45
    hold_max_frames: int = 25


@dataclass
class ConeDriveCommand:
    angle: float = 0.0
    speed: float = 0.0
    finished: bool = False
    debug: dict = field(default_factory=dict)
    debug_image: np.ndarray | None = None


ConeCommand = ConeDriveCommand


class ConeFollower:
    """LiDAR cone corridor follower from the team cone section.

    All tuning values live in ConeFollowerConfig or cone_* ROS parameters. The
    existing lane, obstacle, and distance-follow parameters are not read here.
    """

    def __init__(self, config: ConeFollowerConfig | None = None) -> None:
        cfg = config or ConeFollowerConfig()
        self.cruise_speed = float(cfg.cruise_speed)
        self.min_speed = float(cfg.min_speed)
        self.corner_speed = float(cfg.corner_speed)
        self.finish_speed = float(cfg.finish_speed)

        self.max_steer = float(cfg.max_steer)
        self.pursuit_gain = float(cfg.pursuit_gain)
        self.lateral_gain = float(cfg.lateral_gain)
        self.clearance_gain = float(cfg.clearance_gain)
        self.corner_boost = float(cfg.corner_boost)
        self.smooth_alpha_straight = float(cfg.smooth_alpha_straight)
        self.smooth_alpha_corner = float(cfg.smooth_alpha_corner)

        self.lookahead_y_min = float(cfg.lookahead_y_min)
        self.lookahead_y_max = float(cfg.lookahead_y_max)
        self.lookahead_target_y = float(cfg.lookahead_target_y)
        self.left_x = (float(cfg.left_x_min), float(cfg.left_x_max))
        self.right_x = (float(cfg.right_x_min), float(cfg.right_x_max))
        self.half_corridor = float(cfg.half_corridor)
        self.left_wall_half = float(cfg.left_wall_half)
        self.left_wall_center_bias = float(cfg.left_wall_center_bias)

        self.front_slow_dist = float(cfg.front_slow_dist)
        self.front_stop_dist = float(cfg.front_stop_dist)

        self.clear_frames_required = int(cfg.clear_frames_required)
        self.min_tracked_frames = int(cfg.min_tracked_frames)
        self.yellow_pixel_threshold = int(cfg.yellow_pixel_threshold)
        self.yellow_roi_ratio = float(cfg.yellow_roi_ratio)

        self.hold_max_frames = int(cfg.hold_max_frames)

        self._left_wall_latched = False
        self._filtered_angle = 0.0
        self._last_good_angle = 0.0
        self._lost_frames = 0
        self._clear_frames = 0
        self._tracked_frames = 0

    def reset(self) -> None:
        self._filtered_angle = 0.0
        self._last_good_angle = 0.0
        self._lost_frames = 0
        self._clear_frames = 0
        self._tracked_frames = 0
        self._left_wall_latched = False

    def _detect_yellow_lane(self, image: Optional[np.ndarray]) -> bool:
        if image is None:
            return False
        h, w = image.shape[:2]
        y0 = int(h * (1.0 - self.yellow_roi_ratio))
        roi = image[y0:h, int(w * 0.2) : int(w * 0.8)]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (18, 80, 100), (38, 255, 255))
        return int(np.count_nonzero(mask)) >= self.yellow_pixel_threshold

    def _detect_cones(self, ranges):
        return detect_cones_in_corridor(
            ranges,
            y_min=self.lookahead_y_min,
            y_max=self.lookahead_y_max,
            left_x=self.left_x,
            right_x=self.right_x,
            target_y=self.lookahead_target_y,
            half_corridor=self.half_corridor,
            force_left_wall=self._left_wall_latched,
        )

    def _blend_path_target(self, path_targets: list[PathTarget]) -> Optional[tuple]:
        if not path_targets:
            return None
        wsum = sum(t.weight for t in path_targets)
        cx = sum(t.cx * t.weight for t in path_targets) / wsum
        cy = sum(t.cy * t.weight for t in path_targets) / wsum
        far = max(path_targets, key=lambda t: t.depth)
        return cx, cy, far.cx, far.cy

    def _pure_pursuit_angle(self, cx: float, cy: float, boost: float = 1.0) -> float:
        cy = max(cy, 0.8)
        heading = math.degrees(math.atan2(cx, cy))
        lateral = self.lateral_gain * cx / cy
        angle = (heading * self.pursuit_gain + lateral) * boost
        return float(np.clip(angle, -self.max_steer, self.max_steer))

    def compute(
        self,
        ranges,
        image: Optional[np.ndarray] = None,
    ) -> ConeDriveCommand:
        if ranges is None:
            return ConeDriveCommand(angle=0.0, speed=0.0, finished=False)

        left, right, det_dbg = self._detect_cones(ranges)
        mode = det_dbg.get("mode", "none")
        follow_mode = det_dbg.get("follow_mode", "center")
        path_targets: list[PathTarget] = det_dbg.get("path_targets", [])

        if mode in ("pair", "single", "left_wall"):
            self._tracked_frames += 1
            self._lost_frames = 0
        else:
            self._lost_frames += 1

        if follow_mode == "left_wall":
            self._left_wall_latched = True
        elif det_dbg.get("left_turn", 0) > 0.14 and left:
            self._left_wall_latched = True
        if mode == "none" and self._lost_frames > 30:
            self._left_wall_latched = False

        finished = self._check_finished_with_cones(left, right, image, mode)
        angle, steer_dbg = self._compute_steer(left, right, det_dbg, path_targets, mode, follow_mode)
        speed = self._compute_speed_with_cones(ranges, left, right, finished, mode, follow_mode)
        front_dist = self._front_clearance(ranges, left, right, path_targets)

        debug = {
            **det_dbg,
            **steer_dbg,
            "left_cone": left,
            "right_cone": right,
            "mode": mode,
            "finished": finished,
            "clear_frames": self._clear_frames,
            "tracked_frames": self._tracked_frames,
            "lost_frames": self._lost_frames,
            "front_dist": front_dist,
            "path_count": len(path_targets),
            "follow_mode": follow_mode,
        }
        return ConeDriveCommand(
            angle=angle,
            speed=speed,
            finished=finished,
            debug=debug,
            debug_image=self._debug_image(image, left, right, debug),
        )

    def _compute_steer(self, left, right, det_dbg, path_targets, mode, follow_mode):
        steer_mode = mode
        boost = 1.0

        if follow_mode == "left_wall":
            boost = self.corner_boost
            steer_mode = "left_wall"
        elif mode == "single":
            boost = self.corner_boost
        elif mode == "pair" and path_targets:
            blended = self._blend_path_target(path_targets)
            if blended and abs(blended[0]) > 0.35:
                boost = 1.15

        blended = self._blend_path_target(path_targets)

        if blended:
            cx, cy, far_cx, far_cy = blended
            if follow_mode == "left_wall":
                cx += self.left_wall_center_bias
                far_cx += self.left_wall_center_bias * 0.6
                near_angle = self._pure_pursuit_angle(cx, cy, boost)
                far_angle = self._pure_pursuit_angle(far_cx, far_cy, boost * 1.05)
                raw_angle = 0.30 * near_angle + 0.70 * far_angle
            else:
                near_angle = self._pure_pursuit_angle(cx, cy, boost)
                far_angle = self._pure_pursuit_angle(far_cx, far_cy, boost * 1.1)
                raw_angle = 0.35 * near_angle + 0.65 * far_angle
            center_offset = cx
        elif left and right:
            cx = (left.cx + right.cx) / 2.0
            cy = (left.cy + right.cy) / 2.0
            raw_angle = self._pure_pursuit_angle(cx, cy, boost)
            center_offset = cx
            left_clear = abs(left.cx)
            right_clear = abs(right.cx)
            clearance_offset = (left_clear - right_clear) / max(left_clear + right_clear, 0.1)
            raw_angle -= self.clearance_gain * clearance_offset
        elif left:
            boost = self.corner_boost
            cx = left.cx + self.left_wall_half
            cy = max(left.cy, 1.0)
            raw_angle = self._pure_pursuit_angle(cx, cy, boost)
            center_offset = cx
            steer_mode = "single_left"
        elif right and not det_dbg.get("right_blocked"):
            boost = self.corner_boost
            cx = right.cx - self.half_corridor
            cy = max(right.cy, 1.0)
            raw_angle = self._pure_pursuit_angle(cx, cy, boost)
            center_offset = cx
            steer_mode = "single_right"
        elif self._lost_frames <= self.hold_max_frames:
            steer_mode = "hold"
            raw_angle = self._last_good_angle * 1.05
            raw_angle = float(np.clip(raw_angle, -self.max_steer, self.max_steer))
            alpha = self.smooth_alpha_corner
            self._filtered_angle += alpha * (raw_angle - self._filtered_angle)
            return self._filtered_angle, {
                "center_offset": 0.0,
                "raw_angle": raw_angle,
                "filtered_angle": self._filtered_angle,
                "steer_mode": steer_mode,
                "path_cx": 0.0,
                "path_cy": 0.0,
            }
        else:
            raw_angle = 0.0
            center_offset = 0.0

        raw_angle = float(np.clip(raw_angle, -self.max_steer, self.max_steer))

        if mode in ("pair", "single", "left_wall"):
            self._last_good_angle = raw_angle

        is_corner = follow_mode == "left_wall" or mode != "pair" or abs(raw_angle) > 12.0
        alpha = self.smooth_alpha_corner if is_corner else self.smooth_alpha_straight
        self._filtered_angle += alpha * (raw_angle - self._filtered_angle)

        path_cx = blended[0] if blended else center_offset
        path_cy = blended[1] if blended else (left.cy if left else 99.0)

        return self._filtered_angle, {
            "center_offset": center_offset if blended is None else blended[0],
            "raw_angle": raw_angle,
            "filtered_angle": self._filtered_angle,
            "steer_mode": steer_mode,
            "path_cx": path_cx,
            "path_cy": path_cy,
            "boost": boost,
        }

    def _compute_speed_with_cones(self, ranges, left, right, finished, mode, follow_mode):
        if finished:
            return self.finish_speed

        front_dist = self._front_clearance(ranges, left, right, [])
        if follow_mode == "left_wall" or mode != "pair" or self._lost_frames > 0:
            base = self.corner_speed
        else:
            base = self.cruise_speed

        if front_dist < self.front_stop_dist:
            return 0.0
        if front_dist < self.front_slow_dist:
            ratio = (front_dist - self.front_stop_dist) / (self.front_slow_dist - self.front_stop_dist)
            return self.min_speed + ratio * (base - self.min_speed)
        return base

    def _check_finished_with_cones(self, left, right, image, mode):
        no_cones = mode == "none" and self._lost_frames > self.hold_max_frames
        lane_visible = self._detect_yellow_lane(image)
        tracked_enough = self._tracked_frames >= self.min_tracked_frames

        if no_cones and lane_visible and tracked_enough:
            self._clear_frames += 1
        else:
            self._clear_frames = 0

        return self._clear_frames >= self.clear_frames_required

    def _front_clearance(
        self,
        ranges,
        left: Optional[ConeCluster],
        right: Optional[ConeCluster],
        path_targets: list[PathTarget],
    ) -> float:
        center_front = center_path_distance(ranges, y_max=self.lookahead_y_max)

        pair_front = 99.0
        if path_targets:
            pair_front = min(t.cy for t in path_targets)
        elif left and right:
            avg_y = (left.cy + right.cy) / 2.0
            if avg_y > 1.0:
                pair_front = avg_y

        front = min(center_front, pair_front)
        if front < 1.0 and center_front > 1.2:
            front = center_front
        return front

    def _debug_image(
        self,
        image: Optional[np.ndarray],
        left: Optional[ConeCluster],
        right: Optional[ConeCluster],
        debug: dict,
    ) -> np.ndarray | None:
        if image is None:
            return None
        vis = image.copy()
        lines = [
            f"CONE mode={debug.get('mode')} follow={debug.get('follow_mode')}",
            f"angle={debug.get('filtered_angle', 0.0):+.1f} raw={debug.get('raw_angle', 0.0):+.1f}",
            f"path=({debug.get('path_cx', 0.0):+.2f},{debug.get('path_cy', 0.0):.2f}) front={debug.get('front_dist', 99.0):.2f}",
            f"tracked={debug.get('tracked_frames')} lost={debug.get('lost_frames')} clear={debug.get('clear_frames')}",
        ]
        for idx, text in enumerate(lines):
            cv2.putText(vis, text, (12, 28 + idx * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 255, 255), 2, cv2.LINE_AA)
        if left:
            cv2.putText(vis, f"L ({left.cx:+.2f},{left.cy:.2f})", (12, vis.shape[0] - 48), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 120, 0), 2, cv2.LINE_AA)
        if right:
            cv2.putText(vis, f"R ({right.cx:+.2f},{right.cy:.2f})", (12, vis.shape[0] - 22), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 120), 2, cv2.LINE_AA)
        return vis

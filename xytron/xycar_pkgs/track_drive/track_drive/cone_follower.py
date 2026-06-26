from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class ConeCandidate:
    x: int
    y: int
    w: int
    h: int
    area: float

    @property
    def cx(self) -> float:
        return self.x + self.w * 0.5

    @property
    def cy(self) -> float:
        return self.y + self.h * 0.5


@dataclass
class ConeCommand:
    angle: float = 0.0
    speed: float = 0.0
    finished: bool = False
    debug: dict[str, object] = field(default_factory=dict)
    debug_image: np.ndarray | None = None


@dataclass(frozen=True)
class ConeFollowerConfig:
    speed_cmd: float = 7.0
    slow_speed_cmd: float = 4.0
    lost_speed_cmd: float = 0.0
    max_angle_cmd: float = 70.0
    steer_gain: float = 72.0
    single_cone_path_offset_px: float = 140.0
    target_smoothing: float = 0.35
    camera_roi_y_min_ratio: float = 0.38
    camera_roi_y_max_ratio: float = 0.96
    camera_h_min: int = 5
    camera_h_max: int = 25
    camera_s_min: int = 80
    camera_v_min: int = 80
    min_area_px: float = 35.0
    max_area_px: float = 9000.0
    min_height_px: int = 8
    max_height_ratio: float = 0.55
    min_aspect: float = 0.35
    max_aspect: float = 2.40
    lidar_front_center_deg: float = 0.0
    lidar_front_half_width_deg: float = 24.0
    lidar_side_min_deg: float = 8.0
    lidar_min_valid_distance_m: float = 0.12
    lidar_max_distance_m: float = 6.0
    front_stop_distance_m: float = 0.35
    front_slow_distance_m: float = 0.85
    right_block_distance_m: float = 0.75
    finish_min_seen_frames: int = 8
    finish_lost_frames: int = 16
    finish_front_clear_distance_m: float = 1.10


class ConeFollower:
    """Self-contained orange cone corridor follower.

    This module intentionally owns all cone-driving constants. It does not read
    lane-following, obstacle, or speed parameters from the main driver.
    """

    def __init__(self, config: ConeFollowerConfig | None = None) -> None:
        self.config = config or ConeFollowerConfig()
        self.last_target_x: float | None = None
        self.lost_frames = 0
        self.seen_frames = 0
        self.seen_cone_once = False

    def compute(
        self,
        lidar_ranges: list[float] | tuple[float, ...] | None,
        frame: np.ndarray | None,
        angle_min: float = 0.0,
        angle_increment: float = 0.0,
    ) -> ConeCommand:
        candidates, mask = self._detect_cones(frame)
        left_cone, right_cone = self._select_track_cones(candidates, frame)
        path_cx, path_cy, follow_mode = self._path_target(left_cone, right_cone, frame)
        path_count = len(candidates)

        if path_cx is None:
            self.lost_frames += 1
            path_cx = self.last_target_x
            follow_mode = "hold" if path_cx is not None else "lost"
        else:
            self.seen_cone_once = True
            self.seen_frames += 1
            self.lost_frames = 0
            path_cx = self._smooth_target(path_cx)
            self.last_target_x = path_cx

        image_width = self._image_width(frame)
        center_x = image_width * 0.5
        if path_cx is None:
            center_offset = 0.0
        else:
            center_offset = (float(path_cx) - center_x) / max(1.0, center_x)

        front_dist = self._front_distance(lidar_ranges, angle_min, angle_increment)
        right_blocked = self._right_blocked(lidar_ranges, angle_min, angle_increment)
        angle = float(np.clip(center_offset * self.config.steer_gain, -self.config.max_angle_cmd, self.config.max_angle_cmd))
        speed, mode, steer_mode = self._speed_and_mode(front_dist, follow_mode, angle)
        finished = self._finished(front_dist)

        debug = {
            "left_cone": left_cone,
            "right_cone": right_cone,
            "center_offset": float(center_offset),
            "front_dist": float(front_dist) if math.isfinite(front_dist) else float("inf"),
            "mode": mode,
            "path_cx": float(path_cx) if path_cx is not None else 0.0,
            "path_cy": float(path_cy) if path_cy is not None else 0.0,
            "path_count": int(path_count),
            "follow_mode": follow_mode,
            "right_blocked": bool(right_blocked),
            "steer_mode": steer_mode,
            "lost_frames": int(self.lost_frames),
            "seen_frames": int(self.seen_frames),
        }
        return ConeCommand(
            angle=angle,
            speed=speed,
            finished=finished,
            debug=debug,
            debug_image=self._debug_image(frame, mask, candidates, left_cone, right_cone, path_cx, path_cy, debug),
        )

    def _detect_cones(self, frame: np.ndarray | None) -> tuple[list[ConeCandidate], np.ndarray | None]:
        if frame is None or frame.size == 0:
            return [], None

        cfg = self.config
        height, width = frame.shape[:2]
        y0 = int(np.clip(round(height * cfg.camera_roi_y_min_ratio), 0, max(0, height - 1)))
        y1 = int(np.clip(round(height * cfg.camera_roi_y_max_ratio), y0 + 1, height))
        roi = frame[y0:y1, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([cfg.camera_h_min, cfg.camera_s_min, cfg.camera_v_min], dtype=np.uint8),
            np.array([cfg.camera_h_max, 255, 255], dtype=np.uint8),
        )
        mask = cv2.medianBlur(mask, 3)
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        candidates: list[ConeCandidate] = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not (cfg.min_area_px <= area <= cfg.max_area_px):
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if h < cfg.min_height_px or h > max(cfg.min_height_px, int(height * cfg.max_height_ratio)):
                continue
            aspect = w / max(1.0, float(h))
            if not (cfg.min_aspect <= aspect <= cfg.max_aspect):
                continue
            fill = area / max(1.0, float(w * h))
            if fill < 0.18:
                continue
            candidates.append(ConeCandidate(int(x), int(y + y0), int(w), int(h), area))

        candidates.sort(key=lambda c: (c.cy, c.area), reverse=True)
        return candidates, mask

    def _select_track_cones(
        self,
        candidates: list[ConeCandidate],
        frame: np.ndarray | None,
    ) -> tuple[ConeCandidate | None, ConeCandidate | None]:
        if not candidates:
            return None, None

        center_x = self._image_width(frame) * 0.5
        left = [candidate for candidate in candidates if candidate.cx < center_x]
        right = [candidate for candidate in candidates if candidate.cx >= center_x]

        def score(candidate: ConeCandidate) -> float:
            return candidate.area + candidate.cy * 0.8

        left_cone = max(left, key=score) if left else None
        right_cone = max(right, key=score) if right else None
        return left_cone, right_cone

    def _path_target(
        self,
        left_cone: ConeCandidate | None,
        right_cone: ConeCandidate | None,
        frame: np.ndarray | None,
    ) -> tuple[float | None, float | None, str]:
        width = self._image_width(frame)
        cfg = self.config
        if left_cone is not None and right_cone is not None:
            return (left_cone.cx + right_cone.cx) * 0.5, (left_cone.cy + right_cone.cy) * 0.5, "pair"
        if left_cone is not None:
            return min(width - 1.0, left_cone.cx + cfg.single_cone_path_offset_px), left_cone.cy, "left_only"
        if right_cone is not None:
            return max(0.0, right_cone.cx - cfg.single_cone_path_offset_px), right_cone.cy, "right_only"
        return None, None, "lost"

    def _smooth_target(self, target_x: float) -> float:
        alpha = float(np.clip(self.config.target_smoothing, 0.0, 1.0))
        if self.last_target_x is None:
            return float(target_x)
        return float(self.last_target_x * alpha + target_x * (1.0 - alpha))

    def _speed_and_mode(self, front_dist: float, follow_mode: str, angle: float) -> tuple[float, str, str]:
        cfg = self.config
        steer_mode = "turn" if abs(angle) > cfg.max_angle_cmd * 0.45 else "track"
        if math.isfinite(front_dist) and front_dist <= cfg.front_stop_distance_m:
            return 0.0, "front_blocked", steer_mode
        if follow_mode == "lost":
            return cfg.lost_speed_cmd, "lost", "lost"
        if follow_mode == "hold":
            return min(cfg.slow_speed_cmd, cfg.speed_cmd), "hold", steer_mode
        if math.isfinite(front_dist) and front_dist <= cfg.front_slow_distance_m:
            return min(cfg.slow_speed_cmd, cfg.speed_cmd), "front_slow", steer_mode
        if steer_mode == "turn":
            return min(cfg.slow_speed_cmd, cfg.speed_cmd), "turn_slow", steer_mode
        return cfg.speed_cmd, "track", steer_mode

    def _finished(self, front_dist: float) -> bool:
        cfg = self.config
        clear_front = (not math.isfinite(front_dist)) or front_dist >= cfg.finish_front_clear_distance_m
        return (
            self.seen_cone_once
            and self.seen_frames >= max(0, cfg.finish_min_seen_frames)
            and self.lost_frames >= max(1, cfg.finish_lost_frames)
            and clear_front
        )

    def _front_distance(
        self,
        ranges: list[float] | tuple[float, ...] | None,
        angle_min: float,
        angle_increment: float,
    ) -> float:
        points = self._lidar_points(ranges, angle_min, angle_increment)
        front = [distance for distance, relative in points if abs(math.degrees(relative)) <= self.config.lidar_front_half_width_deg]
        return min(front) if front else float("inf")

    def _right_blocked(
        self,
        ranges: list[float] | tuple[float, ...] | None,
        angle_min: float,
        angle_increment: float,
    ) -> bool:
        cfg = self.config
        for distance, relative in self._lidar_points(ranges, angle_min, angle_increment):
            relative_deg = math.degrees(relative)
            if -cfg.lidar_front_half_width_deg <= relative_deg <= -cfg.lidar_side_min_deg:
                if distance <= cfg.right_block_distance_m:
                    return True
        return False

    def _lidar_points(
        self,
        ranges: list[float] | tuple[float, ...] | None,
        angle_min: float,
        angle_increment: float,
    ) -> list[tuple[float, float]]:
        cfg = self.config
        if ranges is None or len(ranges) == 0 or angle_increment == 0.0:
            return []
        front_angle = math.radians(cfg.lidar_front_center_deg)
        points: list[tuple[float, float]] = []
        for idx, raw_distance in enumerate(ranges):
            distance = float(raw_distance)
            if not math.isfinite(distance):
                continue
            if not (cfg.lidar_min_valid_distance_m < distance < cfg.lidar_max_distance_m):
                continue
            angle = angle_min + idx * angle_increment
            relative = self._wrap_angle_pi(angle - front_angle)
            if math.cos(relative) <= 0.0:
                continue
            points.append((distance, relative))
        return points

    def _debug_image(
        self,
        frame: np.ndarray | None,
        mask: np.ndarray | None,
        candidates: list[ConeCandidate],
        left_cone: ConeCandidate | None,
        right_cone: ConeCandidate | None,
        path_cx: float | None,
        path_cy: float | None,
        debug: dict[str, object],
    ) -> np.ndarray | None:
        if frame is None or frame.size == 0:
            return None
        vis = frame.copy()
        height, width = vis.shape[:2]
        y0 = int(np.clip(round(height * self.config.camera_roi_y_min_ratio), 0, max(0, height - 1)))
        y1 = int(np.clip(round(height * self.config.camera_roi_y_max_ratio), y0 + 1, height))
        cv2.rectangle(vis, (0, y0), (width - 1, y1), (0, 180, 255), 2)
        for candidate in candidates:
            color = (0, 140, 255)
            if candidate is left_cone:
                color = (255, 80, 0)
            elif candidate is right_cone:
                color = (0, 255, 80)
            cv2.rectangle(vis, (candidate.x, candidate.y), (candidate.x + candidate.w, candidate.y + candidate.h), color, 2)
            cv2.circle(vis, (int(candidate.cx), int(candidate.cy)), 3, color, -1)
        if path_cx is not None and path_cy is not None:
            cv2.circle(vis, (int(path_cx), int(path_cy)), 7, (255, 0, 255), -1)
            cv2.line(vis, (width // 2, height - 1), (int(path_cx), int(path_cy)), (255, 0, 255), 2)
        lines = [
            f"CONE mode={debug.get('mode')} follow={debug.get('follow_mode')}",
            f"angle={debug.get('center_offset', 0.0):+.2f} lost={debug.get('lost_frames')}",
            f"front={debug.get('front_dist')} n={debug.get('path_count')}",
        ]
        for idx, text in enumerate(lines):
            cv2.putText(vis, text, (12, 28 + idx * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)
        if mask is not None:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_bgr = cv2.resize(mask_bgr, (max(1, width // 4), max(1, mask_bgr.shape[0] // 4)))
            mh, mw = mask_bgr.shape[:2]
            vis[0:mh, width - mw:width] = mask_bgr
        return vis

    @staticmethod
    def _image_width(frame: np.ndarray | None) -> int:
        if frame is not None and frame.ndim >= 2 and frame.shape[1] > 0:
            return int(frame.shape[1])
        return 640

    @staticmethod
    def _wrap_angle_pi(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

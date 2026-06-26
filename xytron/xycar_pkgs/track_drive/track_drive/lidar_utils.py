from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class LidarPoint:
    idx: int
    x: float
    y: float
    dist: float


@dataclass
class ConeCluster:
    side: str
    cx: float
    cy: float
    count: int
    min_y: float


@dataclass
class PathTarget:
    cx: float
    cy: float
    weight: float
    depth: float


def scan_to_points(ranges: Sequence[float]) -> list[LidarPoint]:
    points: list[LidarPoint] = []
    for idx, dist in enumerate(ranges):
        if not math.isfinite(dist):
            continue
        angle = math.radians(idx - 90)
        x = -dist * math.cos(angle)
        y = -dist * math.sin(angle)
        points.append(LidarPoint(idx=idx, x=x, y=y, dist=dist))
    return points


def right_sector_blocked(
    ranges: Sequence[float],
    i0: int = 92,
    i1: int = 210,
    block_dist: float = 0.85,
) -> bool:
    vals = [ranges[i] for i in range(i0, min(i1, len(ranges))) if math.isfinite(ranges[i])]
    if vals and min(vals) < block_dist:
        return True

    echo = [p for p in scan_to_points(ranges) if p.x > 0.35 and p.y < 2.8 and p.dist < 0.95]
    return len(echo) >= 2


def is_right_reliable(
    left_cones: list[ConeCluster],
    right_cones: list[ConeCluster],
    ranges: Optional[Sequence[float]] = None,
) -> bool:
    if ranges is not None and right_sector_blocked(ranges):
        return False

    right_cones = _filter_side_cones(right_cones, "right")
    if len(right_cones) < 2 or len(left_cones) < 1:
        return False

    if any(c.cy < 1.2 for c in right_cones):
        return False

    if len(right_cones) < len(left_cones) * 0.55:
        return False

    right_xs = [c.cx for c in right_cones]
    if float(np.mean(right_xs)) > 2.4:
        return False
    if max(right_xs) - min(right_xs) > 1.8:
        return False

    for left in left_cones:
        matched = [r for r in right_cones if abs(r.cy - left.cy) < 1.5]
        if not matched:
            continue
        right_near = min(matched, key=lambda r: abs(r.cy - left.cy))
        width = right_near.cx - left.cx
        if width < 2.5 or width > 6.5:
            return False

    return True


def should_follow_left_wall(
    left_cones: list[ConeCluster],
    right_cones: list[ConeCluster],
    ranges: Optional[Sequence[float]] = None,
    curvature_threshold: float = 0.12,
    force: bool = False,
) -> bool:
    if force:
        lt = _left_turn_strength(left_cones) if len(left_cones) >= 2 else 0.0
        return len(left_cones) >= 1 and lt > 0.08

    lt = _left_turn_strength(left_cones) if len(left_cones) >= 2 else 0.0
    if len(left_cones) >= 2 and lt > curvature_threshold:
        return True
    if len(left_cones) >= 1 and ranges is not None and right_sector_blocked(ranges) and lt > 0.08:
        return True
    if len(left_cones) >= 2 and not is_right_reliable(left_cones, right_cones, ranges):
        return lt > 0.05
    return False


def build_left_wall_path(
    left_cones: list[ConeCluster],
    depths: tuple[float, ...] = (1.6, 3.0, 4.6, 6.0),
    weights: tuple[float, ...] = (0.10, 0.20, 0.35, 0.35),
    half_corridor: float = 2.1,
) -> list[PathTarget]:
    targets: list[PathTarget] = []
    for depth, weight in zip(depths, weights):
        anchor = _pick_left_at_depth(left_cones, depth)
        if anchor is None:
            continue
        targets.append(PathTarget(cx=anchor.cx + half_corridor, cy=anchor.cy, weight=weight, depth=depth))

    if left_cones:
        far = max(left_cones, key=lambda c: c.cy)
        targets.append(PathTarget(cx=far.cx + half_corridor, cy=far.cy, weight=0.35, depth=far.cy))

    turn = _left_turn_strength(left_cones)
    targets.extend(_extrapolate_left_wall(left_cones, half_corridor, turn))
    return targets


def build_lookahead_path(
    left_cones: list[ConeCluster],
    right_cones: list[ConeCluster],
    depths: tuple[float, ...] = (1.6, 3.0, 4.6),
    weights: tuple[float, ...] = (0.15, 0.30, 0.55),
    half_corridor: float = 2.25,
) -> list[PathTarget]:
    targets: list[PathTarget] = []
    for depth, weight in zip(depths, weights):
        left, right = _pick_best_pair(left_cones, right_cones, depth, y_tol=2.4, width_min=2.0, width_max=7.0)

        if left and right:
            cx, cy = _center_from_pair(left, right)
        elif left:
            cx, cy = _center_from_single(left, half_corridor)
        elif right:
            cx, cy = _center_from_single(right, half_corridor)
        else:
            continue

        targets.append(PathTarget(cx=cx, cy=cy, weight=weight, depth=depth))
    return targets


def center_path_distance(
    ranges: Sequence[float],
    x_limit: float = 0.75,
    y_min: float = 0.5,
    y_max: float = 6.0,
    dist_min: float = 0.35,
    dist_max: float = 6.5,
    default: float = 99.0,
) -> float:
    ys = []
    for p in scan_to_points(ranges):
        if abs(p.x) > x_limit:
            continue
        if p.y < y_min or p.y > y_max:
            continue
        if p.dist < dist_min or p.dist > dist_max:
            continue
        ys.append(p.y)
    return min(ys) if ys else default


def detect_cones_in_corridor(
    ranges: Sequence[float],
    y_min: float = 0.45,
    y_max: float = 6.5,
    left_x: tuple[float, float] = (-3.5, -0.55),
    right_x: tuple[float, float] = (0.55, 3.2),
    dist_min: float = 0.35,
    dist_max: float = 6.5,
    target_y: float = 3.0,
    half_corridor: float = 2.25,
    force_left_wall: bool = False,
) -> tuple[Optional[ConeCluster], Optional[ConeCluster], dict]:
    points = scan_to_points(ranges)
    right_blocked = right_sector_blocked(ranges)

    left_cones = _collect_side_cones(points, "left", left_x, y_min, y_max, dist_min, dist_max, strict=True)
    right_cones = _collect_side_cones(points, "right", right_x, y_min, y_max, dist_min, dist_max, strict=True)

    if not left_cones and not right_cones:
        left_cones = _collect_side_cones(points, "left", left_x, y_min, y_max, dist_min, dist_max, strict=False)
        right_cones = _collect_side_cones(points, "right", right_x, y_min, y_max, dist_min, dist_max, strict=False)

    if not left_cones:
        left_cones = _recover_left_from_raw(points, left_x)

    left_cones = _filter_side_cones(left_cones, "left")
    right_cones = _filter_side_cones(right_cones, "right")

    if right_blocked:
        right_cones = []

    follow_mode = "center"
    use_left_wall = should_follow_left_wall(left_cones, right_cones, ranges, force=force_left_wall)

    if use_left_wall and left_cones:
        follow_mode = "left_wall"
        path_targets = build_left_wall_path(left_cones, half_corridor=half_corridor + 0.20)
    else:
        path_targets = build_lookahead_path(left_cones, right_cones, half_corridor=half_corridor)

    left, right = _pick_best_pair(left_cones, right_cones, target_y)
    detect_pass = "strict"

    if left is None or right is None:
        left, right = _pick_best_pair(left_cones, right_cones, target_y, y_tol=2.4, width_min=2.0, width_max=7.0)
        detect_pass = "relaxed_pair"

    if follow_mode == "left_wall":
        mode = "left_wall"
        left = _side_far_blend(left_cones, 3) if left_cones else None
        right = None
        detect_pass = "left_wall"
    elif left and right:
        mode = "pair"
    elif left_cones or (right_cones and not right_blocked):
        mode = "single"
        left = _side_far_blend(left_cones, 2) if left_cones else None
        right = _side_far_blend(right_cones, 2) if right_cones else None
        if right_blocked:
            right = None
        detect_pass = f"{detect_pass}_far"
    else:
        mode = "none"

    raw_right_cones = _filter_side_cones(
        _collect_side_cones(points, "right", right_x, y_min, y_max, dist_min, dist_max, strict=True)
        or _collect_side_cones(points, "right", right_x, y_min, y_max, dist_min, dist_max, strict=False),
        "right",
    )
    debug = {
        "left_candidates": len(left_cones),
        "right_candidates": len(right_cones),
        "left_clusters": len(left_cones),
        "right_clusters": len(right_cones),
        "left_cone": left,
        "right_cone": right,
        "mode": mode,
        "target_y": target_y,
        "detect_pass": detect_pass,
        "path_targets": path_targets,
        "follow_mode": follow_mode,
        "left_turn": _left_turn_strength(left_cones),
        "right_reliable": is_right_reliable(left_cones, raw_right_cones, ranges),
        "right_blocked": right_blocked,
        "left_cones_all": left_cones,
        "right_cones_all": right_cones,
    }
    return left, right, debug


def _cluster_points(points: list[LidarPoint], eps: float = 0.42) -> list[list[LidarPoint]]:
    if not points:
        return []
    clusters: list[list[LidarPoint]] = []
    used = [False] * len(points)

    for i, seed in enumerate(points):
        if used[i]:
            continue
        cluster = [seed]
        used[i] = True
        queue = [seed]
        while queue:
            current = queue.pop()
            for j, other in enumerate(points):
                if used[j]:
                    continue
                dx = current.x - other.x
                dy = current.y - other.y
                if dx * dx + dy * dy <= eps * eps:
                    used[j] = True
                    cluster.append(other)
                    queue.append(other)
        clusters.append(cluster)
    return clusters


def _cluster_to_cone(side: str, cluster: list[LidarPoint], strict: bool = True) -> Optional[ConeCluster]:
    if not cluster:
        return None

    xs = [p.x for p in cluster]
    ys = [p.y for p in cluster]
    count = len(cluster)
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)

    if strict:
        if count > 18:
            return None
        if y_span > 1.4 and count > 6:
            return None
        if x_span > 1.2 and count > 6:
            return None
    else:
        if count > 28:
            return None
        if y_span > 2.2 and count > 10:
            return None

    return ConeCluster(side=side, cx=float(np.mean(xs)), cy=float(np.mean(ys)), count=count, min_y=float(min(ys)))


def _filter_side_cones(cones: list[ConeCluster], side: str) -> list[ConeCluster]:
    kept: list[ConeCluster] = []
    for cone in cones:
        if cone.cy < 0.85:
            continue
        if side == "right" and cone.cx < 1.0 and cone.cy < 1.5:
            continue
        if side == "left" and cone.cx > -0.8:
            continue
        kept.append(cone)
    return kept


def _collect_side_cones(
    points: list[LidarPoint],
    side: str,
    x_range: tuple[float, float],
    y_min: float,
    y_max: float,
    dist_min: float,
    dist_max: float,
    strict: bool = True,
) -> list[ConeCluster]:
    candidates = [
        p
        for p in points
        if y_min <= p.y <= y_max and dist_min <= p.dist <= dist_max and x_range[0] <= p.x <= x_range[1]
    ]
    cones = [
        cone
        for cone in (_cluster_to_cone(side, cluster, strict) for cluster in _cluster_points(candidates))
        if cone is not None
    ]
    cones.sort(key=lambda cone: cone.cy)
    return cones


def _pair_score(
    left: ConeCluster,
    right: ConeCluster,
    target_y: float,
    y_tol: float,
    width_min: float,
    width_max: float,
) -> float:
    y_diff = abs(left.cy - right.cy)
    if y_diff > y_tol:
        return 999.0
    width = right.cx - left.cx
    if width < width_min or width > width_max:
        return 999.0
    avg_y = (left.cy + right.cy) / 2.0
    return abs(avg_y - target_y) + 0.35 * y_diff


def _pick_best_pair(
    left_cones: list[ConeCluster],
    right_cones: list[ConeCluster],
    target_y: float,
    y_tol: float = 1.3,
    width_min: float = 2.8,
    width_max: float = 6.0,
) -> tuple[Optional[ConeCluster], Optional[ConeCluster]]:
    best_score = 999.0
    best_pair: tuple[Optional[ConeCluster], Optional[ConeCluster]] = (None, None)
    for left in left_cones:
        for right in right_cones:
            score = _pair_score(left, right, target_y, y_tol, width_min, width_max)
            if score < best_score:
                best_score = score
                best_pair = (left, right)
    return best_pair


def _side_far_blend(cones: list[ConeCluster], count: int = 2) -> Optional[ConeCluster]:
    if not cones:
        return None
    picked = sorted(cones, key=lambda cone: cone.cy, reverse=True)[:count]
    side = picked[0].side
    weights = [1.0, 0.55][: len(picked)]
    wsum = sum(weights)
    cx = sum(cone.cx * weight for cone, weight in zip(picked, weights)) / wsum
    cy = sum(cone.cy * weight for cone, weight in zip(picked, weights)) / wsum
    return ConeCluster(side=side, cx=cx, cy=cy, count=sum(cone.count for cone in picked), min_y=min(cone.min_y for cone in picked))


def _center_from_pair(left: ConeCluster, right: ConeCluster) -> tuple[float, float]:
    return (left.cx + right.cx) / 2.0, (left.cy + right.cy) / 2.0


def _center_from_single(cone: ConeCluster, half_corridor: float) -> tuple[float, float]:
    if cone.side == "left":
        return cone.cx + half_corridor, cone.cy
    return cone.cx - half_corridor, cone.cy


def _left_turn_strength(left_cones: list[ConeCluster]) -> float:
    if len(left_cones) < 2:
        return 0.0
    near = left_cones[0]
    far = left_cones[-1]
    return near.cx - far.cx


def _pick_left_at_depth(left_cones: list[ConeCluster], depth: float) -> Optional[ConeCluster]:
    if not left_cones:
        return None
    return min(left_cones, key=lambda cone: abs(cone.cy - depth))


def _recover_left_from_raw(
    points: list[LidarPoint],
    left_x: tuple[float, float],
    y_min: float = 1.0,
    y_max: float = 7.5,
    dist_min: float = 1.0,
    dist_max: float = 6.5,
) -> list[ConeCluster]:
    candidates = [
        point
        for point in points
        if left_x[0] <= point.x <= left_x[1] and y_min < point.y < y_max and dist_min < point.dist < dist_max
    ]
    if not candidates:
        return []

    cones: list[ConeCluster] = []
    for cluster in _cluster_points(candidates, eps=0.55):
        cone = _cluster_to_cone("left", cluster, strict=False)
        if cone is not None:
            cones.append(cone)
    cones.sort(key=lambda cone: cone.cy)
    return _filter_side_cones(cones, "left")


def _extrapolate_left_wall(left_cones: list[ConeCluster], half_corridor: float, left_turn: float) -> list[PathTarget]:
    if len(left_cones) < 2 or left_turn < 0.15:
        return []

    near = left_cones[0]
    far = left_cones[-1]
    dy = far.cy - near.cy
    if dy < 0.35:
        return []

    slope = (far.cx - near.cx) / dy
    extras: list[PathTarget] = []
    for ey, weight in ((4.5, 0.50), (6.0, 0.55), (7.5, 0.45)):
        if ey <= far.cy + 0.3:
            continue
        ex = far.cx + slope * (ey - far.cy)
        if left_turn > 0.25:
            ex = max(ex, far.cx + 0.08 * (ey - far.cy))
        extras.append(PathTarget(cx=ex + half_corridor, cy=ey, weight=weight, depth=ey))
    return extras

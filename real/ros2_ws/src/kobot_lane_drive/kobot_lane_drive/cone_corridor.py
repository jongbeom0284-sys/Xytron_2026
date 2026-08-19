"""라바콘 통로(Corridor) 주행 알고리즘 모듈.

LaserScan 점군(차량 좌표계 x: 좌+, y: 전방+)을 분석하여,
좌/우 라바콘을 군집화(Clustering)하고 최대 연결 거리 제한(Max Link Distance)을 통해
연속된 가상 경계선(Chained Polyline Boundary)을 형성함으로써,
라바콘 사이 틈새 이탈 및 원거리 대각선 오연결을 원천 차단하고 통로 중심을 정밀하게 추종한다.
ROS 런타임에 의존하지 않는 순수 계산 함수들로 구성된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .geometry import clamp
from .obstacle_lidar import LidarSectorConfig, scan_to_vehicle_points


@dataclass(frozen=True)
class ConeCorridorConfig:
    # 통로 기본 기하
    corridor_width_m: float = 0.80
    single_wall_fallback_margin_m: float = 0.40  # 한쪽 벽만 보일 때 가상 중심 오프셋 (통로 반폭)

    # 통로 폭 유효 범위
    corridor_width_min_m: float = 0.50
    corridor_width_max_m: float = 1.30

    # 전방 탐색 범위 (차량 앞 범퍼 기준)
    forward_min_m: float = 0.20
    forward_max_m: float = 1.60
    lateral_max_m: float = 0.90  # 횡방향 최대 탐색 거리 (좌우 90cm)

    # 라바콘 유클리드 클러스터링 필터 (긴 벽 및 단일 잡음 배제)
    cluster_distance_tolerance_m: float = 0.12  # 점 간 이격 거리
    cone_min_cluster_points: int = 3            # 라바콘 최소 반사점 (잡음 배제)
    cone_max_cluster_points: int = 25           # 라바콘 최대 반사점
    cone_min_cluster_span_m: float = 0.03       # 라바콘 최소 외경
    cone_max_cluster_span_m: float = 0.25       # 라바콘 최대 외경 (25cm 초과 긴 벽 배제)

    # 라바콘 간 최대 연결 거리 (대각선 오연결 방지)
    max_cone_link_distance_m: float = 0.50

    # 슬라이스 분할 설정
    slice_step_m: float = 0.10
    min_points_per_side: int = 1

    # 목표 추종 (Pure Pursuit / Lookahead & Gain)
    target_lookahead_m: float = 0.65    # 90도 코너 및 곡선 구간 적정 주시 (0.65m)
    wheelbase_m: float = 0.33
    max_steering_rad: float = 0.45      # 약 25.8도
    steering_gain: float = 1.7          # 조향 추종 게인 (안쪽 파고들기 완화)
    heading_gain: float = 0.15           # 코너 벽면 곡률 피드포워드 게인 (인코스 완화)

    # 조향 저역통과 필터 및 변화율 제한
    steering_filter_alpha: float = 0.60  # 빠른 코너 조향 반응성
    max_steering_rate_rad_s: float = 2.5 # 급선회 허용 변화율

    # 주행 속도
    drive_speed_mps: float = 0.30
    corner_speed_mps: float = 0.25

    # 상태 전이 및 감지 판단 기준
    detection_min_slices: int = 2
    detection_min_total_points: int = 3


@dataclass(frozen=True)
class CorridorSlice:
    """전방 특정 거리(y) 구간에서의 좌/우 경계 및 중심점."""

    y_m: float
    left_x_m: float | None
    right_x_m: float | None
    center_x_m: float | None
    is_interpolated: bool


@dataclass(frozen=True)
class ConeCorridorResult:
    """라바콘 통로 분석 및 주행 제어 결과."""

    found: bool
    target_x_m: float
    target_y_m: float
    steering_rad: float
    speed_mps: float
    confidence: float
    slices: list[CorridorSlice]
    left_points_count: int
    right_points_count: int
    corridor_width_est_m: float | None
    primary_side: str  # "BOTH", "LEFT_ONLY", "RIGHT_ONLY", "NONE"


EMPTY_CORRIDOR_RESULT = ConeCorridorResult(
    found=False,
    target_x_m=0.0,
    target_y_m=0.70,
    steering_rad=0.0,
    speed_mps=0.0,
    confidence=0.0,
    slices=[],
    left_points_count=0,
    right_points_count=0,
    corridor_width_est_m=None,
    primary_side="NONE",
)


def filter_cone_points(
    points_x: np.ndarray,
    points_y: np.ndarray,
    config: ConeCorridorConfig,
) -> tuple[np.ndarray, np.ndarray, list[tuple[float, float]]]:
    """유클리드 클러스터링으로 긴 연속 벽(Span > 0.25m)이나 1~2개 튀는 노이즈를 버리고
    30cm 원뿔 라바콘 덩어리(Span 0.03~0.25m, 점 3~25개)에 속한 점들 및 콘 중심점들만 반환한다."""
    if points_x.size == 0 or points_y.size == 0:
        empty = np.empty(0, dtype=np.float64)
        return empty, empty, []

    pts = np.column_stack([points_x, points_y])
    num_pts = len(pts)
    visited = np.zeros(num_pts, dtype=bool)

    valid_indices: list[int] = []
    cone_centers: list[tuple[float, float]] = []
    tol = config.cluster_distance_tolerance_m

    for i in range(num_pts):
        if visited[i]:
            continue
        c_indices = [i]
        visited[i] = True
        queue = [i]
        while queue:
            curr = queue.pop(0)
            dists = np.hypot(pts[:, 0] - pts[curr, 0], pts[:, 1] - pts[curr, 1])
            neighbors = np.where((dists <= tol) & (~visited))[0]
            for n in neighbors:
                visited[n] = True
                queue.append(n)
                c_indices.append(n)

        c_pts = pts[c_indices]
        span_x = float(c_pts[:, 0].max() - c_pts[:, 0].min())
        span_y = float(c_pts[:, 1].max() - c_pts[:, 1].min())
        span = float(math.hypot(span_x, span_y))
        count = len(c_indices)

        # 라바콘 판정: 점수 및 물체 외형 지름 검증
        if (
            config.cone_min_cluster_points <= count <= config.cone_max_cluster_points
            and config.cone_min_cluster_span_m <= span <= config.cone_max_cluster_span_m
        ):
            valid_indices.extend(c_indices)
            cone_centers.append((float(c_pts[:, 0].mean()), float(c_pts[:, 1].mean())))

    if not valid_indices:
        empty = np.empty(0, dtype=np.float64)
        return empty, empty, []

    valid_arr = np.array(valid_indices, dtype=np.int64)
    return points_x[valid_arr], points_y[valid_arr], cone_centers


def chain_cone_boundary(
    root_candidates: list[tuple[float, float]],
    all_cones: list[tuple[float, float]],
    max_link_distance_m: float,
) -> list[tuple[float, float]]:
    """차량에 가장 가까운 기준 콘부터 시작하여, 최대 연결 거리 이내의 인접한 콘들을 순서대로 체이닝한다.
    
    멀리 떨어진 콘(예: 좌회전 코너 탈출부)으로 대각선 연결이 생성되는 것을 방지한다.
    """
    if not root_candidates:
        return []

    # 차량 원점 (0, 0)에서 가장 가까운 콘을 체인의 시작점으로 선정
    root = min(root_candidates, key=lambda c: math.hypot(c[0], c[1]))
    chain = [root]
    curr = root
    remaining = [c for c in all_cones if c != root]

    while remaining:
        valid_next: list[tuple[float, tuple[float, float]]] = []
        for c in remaining:
            dist = math.hypot(c[0] - curr[0], c[1] - curr[1])
            # 거리가 max_link_distance_m 이내이고, 전방 방향으로 진행
            if dist <= max_link_distance_m and c[1] >= curr[1] - 0.10:
                valid_next.append((dist, c))
        if not valid_next:
            break
        valid_next.sort(key=lambda item: item[0])
        next_cone = valid_next[0][1]
        chain.append(next_cone)
        remaining.remove(next_cone)
        curr = next_cone

    return chain


def interpolate_polyline_x_at_y(
    cones: list[tuple[float, float]],
    target_y: float,
    max_link_distance_m: float = 0.50,
) -> float | None:
    """인접한 라바콘 체인 선분으로부터 특정 거리 y에서의 경계 x좌표를 연속 보간한다.
    
    연결 거리가 max_link_distance_m을 초과하는 경우 보간하지 않는다.
    """
    if not cones:
        return None
    if len(cones) == 1:
        if abs(cones[0][1] - target_y) <= 0.40:
            return cones[0][0]
        return None

    # 선분 구간 탐색 및 선형 보간
    for i in range(len(cones) - 1):
        c1, c2 = cones[i], cones[i + 1]
        dist = math.hypot(c2[0] - c1[0], c2[1] - c1[1])
        if dist > max_link_distance_m:
            continue
        y_low, y_high = min(c1[1], c2[1]), max(c1[1], c2[1])
        if y_low <= target_y <= y_high:
            if abs(c2[1] - c1[1]) < 1e-4:
                return (c1[0] + c2[0]) * 0.5
            ratio = (target_y - c1[1]) / (c2[1] - c1[1])
            return c1[0] + ratio * (c2[0] - c1[0])

    # 체인의 끝단에서 target_y와의 거리가 0.40m 이내일 때만 말단 콘 x 확장
    closest = min(cones, key=lambda c: abs(c[1] - target_y))
    if abs(closest[1] - target_y) <= 0.40:
        return closest[0]
    return None


def build_polyline_corridor_slices(
    left_cones: list[tuple[float, float]],
    right_cones: list[tuple[float, float]],
    config: ConeCorridorConfig,
) -> list[CorridorSlice]:
    """좌/우 라바콘 체인 경계선으로부터 구간별 중심선 슬라이스를 생성한다."""
    slices: list[CorridorSlice] = []
    y_min = config.forward_min_m
    y_max = config.forward_max_m
    step = config.slice_step_m
    max_link = config.max_cone_link_distance_m

    num_steps = max(1, int(math.ceil((y_max - y_min) / step)))

    for i in range(num_steps):
        s_y_mid = y_min + (i + 0.5) * step
        if s_y_mid > y_max:
            break

        left_x = interpolate_polyline_x_at_y(left_cones, s_y_mid, max_link)
        right_x = interpolate_polyline_x_at_y(right_cones, s_y_mid, max_link)

        center_x: float | None = None
        is_interpolated = False

        if left_x is not None and right_x is not None:
            width = left_x - right_x
            if config.corridor_width_min_m <= width <= config.corridor_width_max_m:
                center_x = (left_x + right_x) * 0.5
            else:
                nominal_dist = config.single_wall_fallback_margin_m
                if abs(left_x - nominal_dist) <= abs(abs(right_x) - nominal_dist):
                    center_x = left_x - config.single_wall_fallback_margin_m
                else:
                    center_x = right_x + config.single_wall_fallback_margin_m
                is_interpolated = True
        elif left_x is not None:
            center_x = left_x - config.single_wall_fallback_margin_m
            is_interpolated = True
        elif right_x is not None:
            center_x = right_x + config.single_wall_fallback_margin_m
            is_interpolated = True

        slices.append(
            CorridorSlice(
                y_m=s_y_mid,
                left_x_m=left_x,
                right_x_m=right_x,
                center_x_m=center_x,
                is_interpolated=is_interpolated,
            )
        )

    return slices


extract_corridor_slices = build_polyline_corridor_slices


def compute_target_point(
    slices: list[CorridorSlice],
    target_lookahead_m: float,
    fallback_margin_m: float,
) -> tuple[float, float, float]:
    """슬라이스들의 중심점들을 가중 평균하여 부드러운 목표 (x, y) 및 신뢰도를 산출한다."""
    valid_slices = [s for s in slices if s.center_x_m is not None]
    if not valid_slices:
        return 0.0, target_lookahead_m, 0.0

    INTERP_PENALTY = 0.50  # 단측 벽 추종 시에도 목표점이 정상 유도되도록 적절한 가중치 부여

    weights = []
    xs = []
    ys = []

    for s in valid_slices:
        dist = abs(s.y_m - target_lookahead_m)
        weight = 1.0 / (dist * dist + 0.04)  # 거리 역제곱 가중치
        if s.is_interpolated:
            weight *= INTERP_PENALTY
        weights.append(weight)
        xs.append(float(s.center_x_m))
        ys.append(float(s.y_m))

    total_weight = sum(weights)
    target_x = float(sum(w * x for w, x in zip(weights, xs)) / total_weight)
    target_y = float(sum(w * y for w, y in zip(weights, ys)) / total_weight)

    both_count = sum(1 for s in valid_slices if not s.is_interpolated)
    confidence = min(1.0, (both_count * 2.0 + len(valid_slices)) / (len(slices) * 1.5 + 1e-6))

    return target_x, target_y, confidence


def compute_steering_pure_pursuit(
    target_x: float,
    target_y: float,
    wheelbase_m: float,
    max_steering_rad: float,
    steering_gain: float = 2.2,
    heading_feedforward_rad: float = 0.0,
) -> float:
    """차량 좌표계 목표점 (target_x, target_y)에 대한 Pure Pursuit 조향각 및 게인/피드포워드를 계산한다.
    
    좌측 조향은 양수(+), 우측 조향은 음수(-).
    """
    ld_sq = target_x * target_x + target_y * target_y
    if ld_sq < 1e-4:
        return 0.0

    kappa = 2.0 * target_x / ld_sq
    raw_steering = math.atan(kappa * wheelbase_m)
    total_steering = raw_steering * steering_gain + heading_feedforward_rad
    return float(clamp(total_steering, -max_steering_rad, max_steering_rad))


def analyze_cone_corridor(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    config: ConeCorridorConfig | None = None,
    lidar_config: LidarSectorConfig | None = None,
) -> ConeCorridorResult:
    """LaserScan 거리 배열을 입력받아 라바콘 통로 주행 결과를 반환한다."""
    config = config or ConeCorridorConfig()
    lidar_config = lidar_config or LidarSectorConfig()

    points_x, points_y = scan_to_vehicle_points(
        ranges, angle_min, angle_increment, lidar_config
    )

    if points_x.size == 0 or points_y.size == 0:
        return EMPTY_CORRIDOR_RESULT

    # ROI 필터링
    roi_mask = (
        (points_y >= config.forward_min_m)
        & (points_y <= config.forward_max_m)
        & (np.abs(points_x) <= config.lateral_max_m)
    )
    roi_x = points_x[roi_mask]
    roi_y = points_y[roi_mask]

    # 유클리드 클러스터링으로 개별 라바콘 객체 분리
    cone_x, cone_y, cone_centers = filter_cone_points(roi_x, roi_y, config)

    left_points = cone_x[cone_x > 0.0]
    right_points = cone_x[cone_x < 0.0]
    left_count = int(left_points.size)
    right_count = int(right_points.size)
    total_count = left_count + right_count

    if total_count < config.detection_min_total_points or not cone_centers:
        return EMPTY_CORRIDOR_RESULT

    # 좌/우 시작 후보 콘 (차량 전방 근거리)
    left_roots = [c for c in cone_centers if c[0] > 0.05 and c[1] <= 0.90] or [c for c in cone_centers if c[0] > 0.0]
    right_roots = [c for c in cone_centers if c[0] < -0.05 and c[1] <= 0.90] or [c for c in cone_centers if c[0] < 0.0]

    # 체이닝으로 연속된 실제 벽면만 추적 (멀리 떨어진 코너 탈출부 콘과의 대각선 연결 방지)
    left_cones = chain_cone_boundary(left_roots, cone_centers, config.max_cone_link_distance_m)
    right_cones = chain_cone_boundary(right_roots, cone_centers, config.max_cone_link_distance_m)

    # 라바콘을 연결한 가상 폴리라인 벽으로 슬라이스 생성
    slices = build_polyline_corridor_slices(left_cones, right_cones, config)
    valid_slices = [s for s in slices if s.center_x_m is not None]

    if len(valid_slices) < config.detection_min_slices:
        return EMPTY_CORRIDOR_RESULT

    target_x, target_y, confidence = compute_target_point(
        slices, config.target_lookahead_m, config.single_wall_fallback_margin_m
    )

    # 코너 벽면 곡률 피드포워드 (선회 벽면의 기울기 보정)
    heading_feedforward = 0.0
    dominant_chain = right_cones if len(right_cones) >= len(left_cones) else left_cones
    if len(dominant_chain) >= 2:
        dx = dominant_chain[-1][0] - dominant_chain[0][0]
        dy = dominant_chain[-1][1] - dominant_chain[0][1]
        if dy > 0.10:
            heading_feedforward = math.atan2(dx, dy) * config.heading_gain

    steering = compute_steering_pure_pursuit(
        target_x,
        target_y,
        config.wheelbase_m,
        config.max_steering_rad,
        config.steering_gain,
        heading_feedforward,
    )

    # 통로 폭 추정 (양쪽 벽이 정상 폭으로 잡힌 슬라이스들의 평균 폭)
    widths = [
        (s.left_x_m - s.right_x_m)
        for s in slices
        if s.left_x_m is not None
        and s.right_x_m is not None
        and (config.corridor_width_min_m <= (s.left_x_m - s.right_x_m) <= config.corridor_width_max_m)
    ]
    corridor_width_est = float(np.mean(widths)) if widths else None

    # 주된 관측 벽
    if left_cones and right_cones:
        primary_side = "BOTH"
    elif left_cones:
        primary_side = "LEFT_ONLY"
    elif right_cones:
        primary_side = "RIGHT_ONLY"
    else:
        primary_side = "NONE"

    # 조향각이 큰 급커브에서는 코너 감속 속도 적용
    speed = (
        config.corner_speed_mps
        if abs(steering) > (config.max_steering_rad * 0.4)
        else config.drive_speed_mps
    )

    return ConeCorridorResult(
        found=True,
        target_x_m=target_x,
        target_y_m=target_y,
        steering_rad=steering,
        speed_mps=speed,
        confidence=confidence,
        slices=slices,
        left_points_count=left_count,
        right_points_count=right_count,
        corridor_width_est_m=corridor_width_est,
        primary_side=primary_side,
    )

"""라바콘 통로(Corridor) 주행 알고리즘 모듈.

LaserScan 점군(차량 좌표계 x: 좌+, y: 전방+)을 분석하여,
좌/우 라바콘을 군집화(Clustering)하고 최대 연결 거리 제한(Max Link Distance)을 통해
연속된 가상 경계선(Chained Polyline Boundary)을 형성함으로써,
라바콘 사이 틈새 이탈 및 원거리 대각선 오연결을 원천 차단하고 통로 중심을 정밀하게 추종한다.
ROS 런타임에 의존하지 않는 순수 계산 함수들로 구성된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
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
    far_max_cone_link_distance_m: float = 0.50
    link_distance_far_start_m: float = 0.30
    link_distance_far_full_m: float = 0.75
    pair_maximum_longitudinal_offset_m: float = 0.38
    pair_root_maximum_center_offset_m: float = 0.25
    pair_root_minimum_side_x_m: float = 0.05
    pair_root_width_tolerance_m: float = 0.15
    boundary_root_lateral_tolerance_m: float = 0.25
    boundary_maximum_heading_change_rad: float = 1.05
    minimum_boundary_cones: int = 2
    minimum_paired_boundary_pairs: int = 2
    root_maximum_y_m: float = 0.75
    boundary_extrapolation_m: float = 0.25

    # 슬라이스 분할 설정
    slice_step_m: float = 0.10
    min_points_per_side: int = 1

    # 목표 추종 (Pure Pursuit / Lookahead & Gain)
    target_lookahead_m: float = 0.65    # 직선 구간 최대 룩어헤드
    corner_target_lookahead_m: float = 0.50
    target_local_window_m: float = 0.15
    lookahead_bend_start_m: float = 0.08
    lookahead_bend_full_m: float = 0.28
    lookahead_near_min_m: float = 0.30
    lookahead_near_max_m: float = 0.55
    lookahead_far_min_m: float = 0.75
    lookahead_far_max_m: float = 1.00
    lookahead_curvature_start_1pm: float = 0.15
    lookahead_curvature_full_1pm: float = 0.90
    polynomial_fit_min_y_m: float = 0.25
    polynomial_fit_max_y_m: float = 1.50
    polynomial_fit_min_points: int = 5
    polynomial_fit_outlier_tolerance_m: float = 0.15
    polynomial_fit_maximum_rmse_m: float = 0.08
    # Pure Pursuit 제어 기준은 뒷바퀴 축 중심이다. 라이다가 차량 앞에 있으므로
    # 라이다 목표 y에 이 거리를 더해 뒷축 좌표의 목표점으로 변환한다.
    lidar_to_rear_axle_m: float = 0.0
    # 회전 안쪽 차체가 콘을 자르지 않도록 목표 중심을 회전 바깥쪽으로 옮긴다.
    curve_outward_bias_max_m: float = 0.0
    # 좌/우 비대칭 조향 및 차체 거동을 보정한다. 음수이면 위 공통값을 사용한다.
    curve_outward_bias_left_max_m: float = -1.0
    curve_outward_bias_right_max_m: float = -1.0
    curve_outward_bias_start_ratio: float = 0.25
    curve_outward_bias_full_ratio: float = 0.80
    vehicle_width_m: float = 0.35
    minimum_boundary_clearance_m: float = 0.05
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
    fallback_speed_mps: float = 0.25

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
class BoundaryPair:
    """같은 통로 단면에 속하는 좌/우 라바콘 한 쌍."""

    left: tuple[float, float]
    right: tuple[float, float]
    center_x_m: float
    center_y_m: float
    width_m: float
    longitudinal_offset_m: float


@dataclass(frozen=True)
class PolynomialCenterline:
    """차량 좌표계에서 y를 독립변수로 사용한 2차 중심 경로."""

    coefficient_a: float
    coefficient_b: float
    coefficient_c: float
    minimum_y_m: float
    maximum_y_m: float
    points_used: int
    rmse_m: float

    def x_at(self, y_m: float) -> float:
        y = float(y_m)
        return float(
            self.coefficient_a * y * y
            + self.coefficient_b * y
            + self.coefficient_c
        )

    def heading_at(self, y_m: float) -> float:
        slope = 2.0 * self.coefficient_a * float(y_m) + self.coefficient_b
        return float(math.atan(slope))

    def curvature_at(self, y_m: float) -> float:
        slope = 2.0 * self.coefficient_a * float(y_m) + self.coefficient_b
        denominator = (1.0 + slope * slope) ** 1.5
        return float(2.0 * self.coefficient_a / max(1e-6, denominator))


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
    # 기본값을 둬 기존 외부 테스트/도구의 생성자 호환성을 유지한다.
    control_target_y_m: float = 0.70
    centerline_target_x_m: float = 0.0
    curve_outward_bias_m: float = 0.0
    left_cones_count: int = 0
    right_cones_count: int = 0
    left_candidates_count: int = 0
    right_candidates_count: int = 0
    suggested_lookahead_m: float = 0.70
    turn_ratio: float = 0.0
    path_model: str = "LOCAL"
    path_curvature_1pm: float = 0.0
    polynomial_rmse_m: float | None = None
    candidate_centers_xy: tuple[tuple[float, float], ...] = ()
    left_boundary_xy: tuple[tuple[float, float], ...] = ()
    right_boundary_xy: tuple[tuple[float, float], ...] = ()
    boundary_mode: str = "NONE"
    boundary_fallback_reason: str = "NONE"
    paired_slice_count: int = 0
    boundary_heading_rad: float = 0.0


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
    control_target_y_m=0.70,
    centerline_target_x_m=0.0,
    curve_outward_bias_m=0.0,
    left_cones_count=0,
    right_cones_count=0,
    left_candidates_count=0,
    right_candidates_count=0,
    suggested_lookahead_m=0.70,
    turn_ratio=0.0,
    path_model="LOCAL",
    path_curvature_1pm=0.0,
    polynomial_rmse_m=None,
    candidate_centers_xy=(),
    left_boundary_xy=(),
    right_boundary_xy=(),
    boundary_mode="NONE",
    boundary_fallback_reason="NONE",
    paired_slice_count=0,
    boundary_heading_rad=0.0,
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


def adaptive_cone_link_distance(
    forward_y_m: float,
    near_distance_m: float,
    far_distance_m: float | None = None,
    far_start_m: float = 0.30,
    far_full_m: float = 0.75,
) -> float:
    """원거리 라이다 희소성을 보상하는 경계 연결 거리 한계를 반환한다.

    차량 바로 앞은 벽 모서리 오연결을 막기 위해 기존 한계를 그대로 쓰고,
    반사점이 희소해지는 전방에서는 실제 콘 한 칸을 놓치지 않을 만큼만 늘린다.
    """

    near = max(0.05, float(near_distance_m))
    far = max(near, float(far_distance_m if far_distance_m is not None else near))
    span = max(1e-3, float(far_full_m) - float(far_start_m))
    ratio = clamp((float(forward_y_m) - float(far_start_m)) / span, 0.0, 1.0)
    return float(near + ratio * (far - near))


def chain_cone_boundary(
    root_candidates: list[tuple[float, float]],
    all_cones: list[tuple[float, float]],
    max_link_distance_m: float,
    far_max_link_distance_m: float | None = None,
    link_distance_far_start_m: float = 0.30,
    link_distance_far_full_m: float = 0.75,
    maximum_heading_change_rad: float = 1.05,
    preferred_root_x_m: float | None = None,
) -> list[tuple[float, float]]:
    """단독 모서리보다 전방으로 가장 길게 이어지는 경계 체인을 고른다.

    기존에는 차량에 가장 가까운 후보를 무조건 시작점으로 써서, 벽 모서리
    하나가 실제 라바콘 열보다 가까우면 경로를 독점했다. 모든 가능한 시작점에서
    체인을 만든 뒤 콘 수, 전방 범위, 전방 도달 거리 순으로 선택한다.
    """

    available_roots = [root for root in root_candidates if root in all_cones]
    if not available_roots:
        return []

    def grow_from(root: tuple[float, float]) -> list[tuple[float, float]]:
        chain = [root]
        current = root
        remaining = [cone for cone in all_cones if cone != root]

        while remaining:
            valid_next: list[tuple[float, float, tuple[float, float]]] = []
            previous_heading: float | None = None
            if len(chain) >= 2:
                previous_heading = math.atan2(
                    chain[-1][0] - chain[-2][0],
                    chain[-1][1] - chain[-2][1],
                )
            for cone in remaining:
                distance = math.hypot(cone[0] - current[0], cone[1] - current[1])
                allowed_distance = adaptive_cone_link_distance(
                    (cone[1] + current[1]) * 0.5,
                    max_link_distance_m,
                    far_max_link_distance_m,
                    link_distance_far_start_m,
                    link_distance_far_full_m,
                )
                heading_continuous = True
                if previous_heading is not None:
                    candidate_heading = math.atan2(
                        cone[0] - current[0],
                        cone[1] - current[1],
                    )
                    heading_delta = abs(
                        math.atan2(
                            math.sin(candidate_heading - previous_heading),
                            math.cos(candidate_heading - previous_heading),
                        )
                    )
                    heading_continuous = heading_delta <= max(
                        0.10,
                        float(maximum_heading_change_rad),
                    )
                if (
                    distance <= allowed_distance
                    and cone[1] >= current[1] - 0.10
                    and heading_continuous
                ):
                    # 거리가 같으면 더 먼 전방으로 이어지는 콘을 선택한다.
                    valid_next.append((distance, -cone[1], cone))
            if not valid_next:
                break

            # 차량 옆 벽 모서리가 실제 첫 콘과 거의 같은 y에 있으면 거리만으로는
            # 모서리가 더 가까워 체인의 첫 점을 훔칠 수 있다. 정상적으로 전진하는
            # 후보가 하나라도 있을 때는 제자리/후진 후보를 버린다. 90도 코너처럼
            # 모든 후보의 y가 비슷한 경우에는 기존 후보를 유지해 횡방향 연결을
            # 막지 않는다.
            forward_next = [
                item
                for item in valid_next
                if item[2][1] >= current[1] + 0.05
            ]
            if forward_next:
                valid_next = forward_next
            valid_next.sort(key=lambda item: (item[0], item[1]))
            next_cone = valid_next[0][2]
            chain.append(next_cone)
            remaining.remove(next_cone)
            current = next_cone
        return chain

    def rank(
        chain: list[tuple[float, float]],
    ) -> tuple[int, float, float, float, float, int, float]:
        ys = [cone[1] for cone in chain]
        forward_span = max(ys) - min(ys)
        forward_reach = max(ys)
        root_distance = math.hypot(chain[0][0], chain[0][1])

        # 콘 개수를 최우선으로 두면 벽 모서리 하나가 실제 체인 앞에 붙는 순간
        # 무조건 이긴다. 연속 선분의 방향 변화량을 먼저 평가해 매끄럽게 전방으로
        # 이어지는 실제 경계를 선택하고, 콘 개수는 동률 보조 기준으로만 쓴다.
        headings = [
            math.atan2(
                chain[index + 1][0] - chain[index][0],
                chain[index + 1][1] - chain[index][1],
            )
            for index in range(len(chain) - 1)
        ]
        roughness = 0.0
        for previous, current in zip(headings, headings[1:]):
            delta = math.atan2(
                math.sin(current - previous),
                math.cos(current - previous),
            )
            roughness += abs(delta)
        roughness += sum(
            max(0.0, chain[index][1] - chain[index + 1][1]) * 5.0
            for index in range(len(chain) - 1)
        )
        # 단측 폴백은 현재 차체 기준 예상 경계(+/- 통로 반폭)에서 시작해야 한다.
        # 더 멀리 이어진다는 이유만으로 안쪽 가구 다리 열을 고르는 것을 막되,
        # 직접 호출자가 기준점을 주지 않은 경우에는 기존 순위를 그대로 유지한다.
        root_anchor_score = (
            -abs(chain[0][0] - float(preferred_root_x_m))
            if preferred_root_x_m is not None
            else 0.0
        )
        return (
            int(len(chain) >= 2),
            root_anchor_score,
            forward_reach,
            -roughness,
            forward_span,
            len(chain),
            -root_distance,
        )

    return max((grow_from(root) for root in available_roots), key=rank)


def chain_paired_boundaries(
    cone_centers: list[tuple[float, float]],
    config: ConeCorridorConfig,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """통로 폭에 맞는 좌/우 콘 쌍을 전방으로 동시에 연결한다.

    한쪽 경계를 먼저 만들고 사용한 콘을 제거하는 순차 체이닝은 급커브에서
    첫 체인이 반대쪽 콘을 훔칠 수 있다. 여기서는 폭이 맞는 단면(pair)을 먼저
    만들고, 좌/우 링크가 *둘 다* 연속인 단면만 다음 구간으로 허용한다.
    따라서 반환된 두 경계는 항상 같은 수의 서로 겹치지 않는 콘으로 구성된다.
    """

    if len(cone_centers) < 2:
        return [], []

    # 실제 콘이 약간 엇갈려 배치된 경우는 허용하되, 한 칸 뒤의 대각선 콘을
    # 통로 맞은편으로 짝짓지 않도록 종방향 차이를 콘 간격보다 작게 제한한다.
    maximum_pair_dy = max(0.18, config.pair_maximum_longitudinal_offset_m)
    pairs: list[BoundaryPair] = []
    for first_index, first in enumerate(cone_centers):
        for second in cone_centers[first_index + 1 :]:
            if abs(first[1] - second[1]) > maximum_pair_dy:
                continue
            # 사진의 실제 배치는 좌우 콘이 종방향으로 엇갈린다. 통로 폭은
            # 두 콘의 유클리드 거리가 아니라 차량 횡방향 간격으로 판단한다.
            width = abs(first[0] - second[0])
            if not (config.corridor_width_min_m <= width <= config.corridor_width_max_m):
                continue
            left, right = (first, second) if first[0] > second[0] else (second, first)
            # 같은 벽의 앞뒤 콘이 폭 범위에 우연히 들어와도 좌우 단면이 되지
            # 않도록 최소 횡방향 분리를 요구한다.
            if left[0] - right[0] < config.corridor_width_min_m * 0.75:
                continue
            pairs.append(
                BoundaryPair(
                    left=left,
                    right=right,
                    center_x_m=(left[0] + right[0]) * 0.5,
                    center_y_m=(left[1] + right[1]) * 0.5,
                    width_m=width,
                    longitudinal_offset_m=abs(left[1] - right[1]),
                )
            )

    # 첫 단면은 반드시 현재 차량 통로를 가로질러야 한다. 이 조건이 없으면
    # 사진 속 의자 다리/책상 다리처럼 차량 한쪽에 나란히 놓인 두 물체 열이
    # 더 길다는 이유만으로 좌/우 경계로 선택될 수 있다. 첫 단면 이후에는
    # 급커브에서 양쪽 경계가 모두 x=0 한쪽으로 이동할 수 있으므로 제한하지 않는다.
    roots = [
        pair
        for pair in pairs
        if pair.center_y_m <= config.root_maximum_y_m
        and abs(pair.center_x_m) <= config.pair_root_maximum_center_offset_m
        and pair.left[0] >= config.pair_root_minimum_side_x_m
        and pair.right[0] <= -config.pair_root_minimum_side_x_m
        and abs(pair.width_m - config.corridor_width_m)
        <= config.pair_root_width_tolerance_m
    ]
    if not roots:
        return [], []

    maximum_width_change = max(0.15, config.corridor_width_m * 0.30)

    def can_follow(previous: BoundaryPair, candidate: BoundaryPair) -> bool:
        center_dy = candidate.center_y_m - previous.center_y_m
        center_allowed_link = adaptive_cone_link_distance(
            (candidate.center_y_m + previous.center_y_m) * 0.5,
            config.max_cone_link_distance_m,
            config.far_max_cone_link_distance_m,
            config.link_distance_far_start_m,
            config.link_distance_far_full_m,
        )
        left_allowed_link = adaptive_cone_link_distance(
            (candidate.left[1] + previous.left[1]) * 0.5,
            config.max_cone_link_distance_m,
            config.far_max_cone_link_distance_m,
            config.link_distance_far_start_m,
            config.link_distance_far_full_m,
        )
        right_allowed_link = adaptive_cone_link_distance(
            (candidate.right[1] + previous.right[1]) * 0.5,
            config.max_cone_link_distance_m,
            config.far_max_cone_link_distance_m,
            config.link_distance_far_start_m,
            config.link_distance_far_full_m,
        )
        # 각 경계의 실제 선분 y에서 보간과 같은 adaptive limit을 계산한다.
        # 엇갈린 pair의 평균 y를 쓰면 체이닝은 승인하고 보간은 자르는 불일치가 생긴다.
        maximum_center_step = center_allowed_link * 1.20
        if center_dy < 0.06 or center_dy > maximum_center_step:
            return False
        # 엇갈린 배치는 허용하지만 어느 한쪽 경계가 뒤로 크게 되감기는 조합은
        # 다른 단면의 콘을 교차 짝지은 것이므로 제외한다.
        if candidate.left[1] < previous.left[1] - 0.08:
            return False
        if candidate.right[1] < previous.right[1] - 0.08:
            return False
        if candidate.left in (previous.left, previous.right):
            return False
        if candidate.right in (previous.left, previous.right):
            return False
        left_step = math.hypot(
            candidate.left[0] - previous.left[0],
            candidate.left[1] - previous.left[1],
        )
        right_step = math.hypot(
            candidate.right[0] - previous.right[0],
            candidate.right[1] - previous.right[1],
        )
        center_step = math.hypot(
            candidate.center_x_m - previous.center_x_m,
            center_dy,
        )
        return (
            left_step <= left_allowed_link
            and right_step <= right_allowed_link
            and center_step <= maximum_center_step
            and abs(candidate.width_m - previous.width_m) <= maximum_width_change
        )

    def rank(chain: list[BoundaryPair]) -> tuple[int, float, float, float, float]:
        headings = [
            math.atan2(
                current.center_x_m - previous.center_x_m,
                current.center_y_m - previous.center_y_m,
            )
            for previous, current in zip(chain, chain[1:])
        ]
        roughness = sum(
            abs(math.atan2(math.sin(b - a), math.cos(b - a)))
            for a, b in zip(headings, headings[1:])
        )
        width_error = sum(
            abs(pair.width_m - config.corridor_width_m) for pair in chain
        ) / max(1, len(chain))
        root = chain[0]
        root_error = (
            abs(root.center_x_m)
            + abs(root.width_m - config.corridor_width_m)
            + root.longitudinal_offset_m
        )
        return (
            len(chain),
            max(pair.center_y_m for pair in chain),
            -root_error,
            -roughness,
            -width_error,
        )

    def valid_continuations(chain: list[BoundaryPair]) -> list[BoundaryPair]:
        previous = chain[-1]
        used_cones = {
            cone
            for existing_pair in chain
            for cone in (existing_pair.left, existing_pair.right)
        }
        valid = [
            pair
            for pair in pairs
            if pair not in chain
            and pair.left not in used_cones
            and pair.right not in used_cones
            and can_follow(previous, pair)
        ]
        if len(chain) < 2:
            return valid

        before = chain[-2]
        previous_heading = math.atan2(
            previous.center_x_m - before.center_x_m,
            previous.center_y_m - before.center_y_m,
        )

        def heading_change(pair: BoundaryPair) -> float:
            heading = math.atan2(
                pair.center_x_m - previous.center_x_m,
                pair.center_y_m - previous.center_y_m,
            )
            return abs(
                math.atan2(
                    math.sin(heading - previous_heading),
                    math.cos(heading - previous_heading),
                )
            )

        return [
            pair
            for pair in valid
            if heading_change(pair) <= config.boundary_maximum_heading_change_rad
        ]

    def grow_from(root: BoundaryPair) -> list[BoundaryPair]:
        """국소 최단 pair가 막다른 모서리여도 실제 긴 경로를 보존한다.

        한 단계 greedy 선택은 엇갈린 벽 모서리가 조금 가깝다는 이유로 실제 pair의
        콘 하나를 소비하고 곧바로 막힌다. 작은 beam으로 가능한 연속 단면을 함께
        추적한 뒤 전체 체인 품질로 고른다. 콘 수가 적은 실차 ROI라 계산량도 제한적이다.
        """

        beam: list[list[BoundaryPair]] = [[root]]
        finished: list[list[BoundaryPair]] = []
        beam_width = 4
        maximum_depth = max(1, len(cone_centers) // 2)

        for _ in range(maximum_depth):
            expanded: list[list[BoundaryPair]] = []
            for chain in beam:
                continuations = valid_continuations(chain)
                if not continuations:
                    finished.append(chain)
                    continue
                expanded.extend([*chain, pair] for pair in continuations)
            if not expanded:
                break
            expanded.sort(key=rank, reverse=True)
            beam = expanded[:beam_width]

        finished.extend(beam)
        return max(finished, key=rank) if finished else [root]

    best = max((grow_from(root) for root in roots), key=rank)
    if len(best) < config.minimum_paired_boundary_pairs:
        return [], []
    return [pair.left for pair in best], [pair.right for pair in best]


def _select_single_boundary(
    left_cones: list[tuple[float, float]],
    right_cones: list[tuple[float, float]],
    config: ConeCorridorConfig,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    """폭이 성립하지 않는 두 체인 중 차량의 예상 경계 위치에 가까운 쪽만 남긴다."""

    if not left_cones or not right_cones:
        return left_cones, right_cones

    margin = config.single_wall_fallback_margin_m
    # 먼 코너에서는 실제 경계도 크게 휘므로 체인 전체 x의 중앙값은 nominal
    # 위치에서 멀어진다. 현재 통로를 고정하는 것은 차량에 가장 가까운 root다.
    # 따라서 양쪽 폭이 성립하지 않을 때는 root anchor 오차로 한쪽을 고른다.
    left_root = min(left_cones, key=lambda cone: cone[1])
    right_root = min(right_cones, key=lambda cone: cone[1])
    left_error = abs(left_root[0] - margin)
    right_error = abs(right_root[0] + margin)
    if left_error <= right_error:
        return left_cones, []
    return [], right_cones


def interpolate_polyline_x_at_y(
    cones: list[tuple[float, float]],
    target_y: float,
    max_link_distance_m: float = 0.50,
    max_extrapolation_m: float = 0.25,
    far_max_link_distance_m: float | None = None,
    link_distance_far_start_m: float = 0.30,
    link_distance_far_full_m: float = 0.75,
) -> float | None:
    """인접한 라바콘 체인 선분으로부터 특정 거리 y에서의 경계 x좌표를 연속 보간한다.
    
    연결 거리가 max_link_distance_m을 초과하는 경우 보간하지 않는다.
    """
    if not cones:
        return None
    if len(cones) == 1:
        if abs(cones[0][1] - target_y) <= max_extrapolation_m:
            return cones[0][0]
        return None

    # 선분 구간 탐색 및 선형 보간
    for i in range(len(cones) - 1):
        c1, c2 = cones[i], cones[i + 1]
        dist = math.hypot(c2[0] - c1[0], c2[1] - c1[1])
        allowed_distance = adaptive_cone_link_distance(
            (c1[1] + c2[1]) * 0.5,
            max_link_distance_m,
            far_max_link_distance_m,
            link_distance_far_start_m,
            link_distance_far_full_m,
        )
        if dist > allowed_distance:
            continue
        y_low, y_high = min(c1[1], c2[1]), max(c1[1], c2[1])
        if y_low <= target_y <= y_high:
            if abs(c2[1] - c1[1]) < 1e-4:
                return (c1[0] + c2[0]) * 0.5
            ratio = (target_y - c1[1]) / (c2[1] - c1[1])
            return c1[0] + ratio * (c2[0] - c1[0])

    # 체인의 끝단을 너무 멀리 평행 연장하면 급코너가 직선처럼 보인다.
    # 짧은 센서 공백만 메우고, 그 이상은 해당 슬라이스를 경로에서 제외한다.
    closest = min(cones, key=lambda c: abs(c[1] - target_y))
    if abs(closest[1] - target_y) <= max_extrapolation_m:
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

        left_x = interpolate_polyline_x_at_y(
            left_cones,
            s_y_mid,
            max_link,
            config.boundary_extrapolation_m,
            config.far_max_cone_link_distance_m,
            config.link_distance_far_start_m,
            config.link_distance_far_full_m,
        )
        right_x = interpolate_polyline_x_at_y(
            right_cones,
            s_y_mid,
            max_link,
            config.boundary_extrapolation_m,
            config.far_max_cone_link_distance_m,
            config.link_distance_far_start_m,
            config.link_distance_far_full_m,
        )

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


def fit_polynomial_centerline(
    slices: list[CorridorSlice],
    minimum_y_m: float = 0.25,
    maximum_y_m: float = 1.50,
    minimum_points: int = 5,
    outlier_tolerance_m: float = 0.15,
    maximum_rmse_m: float = 0.08,
) -> PolynomialCenterline | None:
    """양측 경계의 중간점들에 강건한 2차 중심선을 피팅한다.

    단측 가상 중심은 실제 통로 폭을 보장하지 않으므로 피팅에서 제외한다.
    1차 피팅 잔차가 큰 점을 제거한 뒤 다시 피팅하며, 충분한 y 범위와 낮은
    RMSE를 만족하지 못하면 호출자가 기존 국소 목표점 방식으로 fallback한다.
    """

    selected = [
        corridor_slice
        for corridor_slice in slices
        if corridor_slice.center_x_m is not None
        and not corridor_slice.is_interpolated
        and float(minimum_y_m) <= corridor_slice.y_m <= float(maximum_y_m)
    ]
    required = max(3, int(minimum_points))
    if len(selected) < required:
        return None

    y_values = np.array([item.y_m for item in selected], dtype=np.float64)
    x_values = np.array(
        [float(item.center_x_m) for item in selected],
        dtype=np.float64,
    )
    if float(np.ptp(y_values)) < 0.35:
        return None

    # 먼 중심점도 코너 예측에 필요하지만 근거리 경로가 조향 기준이므로 완만한
    # 거리 가중치를 사용한다. sqrt를 쓰는 것은 np.polyfit의 w가 잔차에 직접
    # 곱해지기 때문이다.
    weights = np.sqrt(1.0 / (0.35 + y_values))
    coefficients = np.polyfit(y_values, x_values, 2, w=weights)
    predicted = np.polyval(coefficients, y_values)
    tolerance = max(0.02, float(outlier_tolerance_m))
    inliers = np.abs(x_values - predicted) <= tolerance
    if int(np.count_nonzero(inliers)) < required:
        return None

    inlier_y = y_values[inliers]
    inlier_x = x_values[inliers]
    if float(np.ptp(inlier_y)) < 0.35:
        return None
    inlier_weights = np.sqrt(1.0 / (0.35 + inlier_y))
    coefficients = np.polyfit(inlier_y, inlier_x, 2, w=inlier_weights)
    residuals = inlier_x - np.polyval(coefficients, inlier_y)
    rmse = float(math.sqrt(float(np.mean(residuals * residuals))))
    if not math.isfinite(rmse) or rmse > max(0.01, float(maximum_rmse_m)):
        return None

    coefficient_a, coefficient_b, coefficient_c = [
        float(value) for value in coefficients
    ]
    if not all(
        math.isfinite(value)
        for value in (coefficient_a, coefficient_b, coefficient_c)
    ):
        return None
    return PolynomialCenterline(
        coefficient_a=coefficient_a,
        coefficient_b=coefficient_b,
        coefficient_c=coefficient_c,
        minimum_y_m=float(np.min(inlier_y)),
        maximum_y_m=float(np.max(inlier_y)),
        points_used=int(len(inlier_y)),
        rmse_m=rmse,
    )


def estimate_local_centerline_heading(
    slices: list[CorridorSlice],
    target_y_m: float,
    window_m: float = 0.30,
) -> float:
    """목표점 주변 중심선의 국소 진행 방향을 추정한다.

    전체 체인의 첫 점과 끝 점을 잇는 방향은 S자의 서로 반대인 굴곡을
    상쇄할 수 있다. 룩어헤드 주변 슬라이스만 1차 피팅해 현재 진입 중인
    커브의 부호를 조향 피드포워드에 유지한다.
    """

    valid = [item for item in slices if item.center_x_m is not None]
    if len(valid) < 2:
        return 0.0
    window = max(0.10, float(window_m))
    local = [item for item in valid if abs(item.y_m - target_y_m) <= window]
    if len(local) < 2:
        local = sorted(valid, key=lambda item: abs(item.y_m - target_y_m))[:3]
    if len(local) < 2:
        return 0.0

    y_values = np.array([item.y_m for item in local], dtype=np.float64)
    x_values = np.array(
        [float(item.center_x_m) for item in local],
        dtype=np.float64,
    )
    if float(np.ptp(y_values)) < 0.05:
        return 0.0
    slope, _ = np.polyfit(y_values, x_values, 1)
    slope_value = float(slope)
    return float(math.atan(slope_value)) if math.isfinite(slope_value) else 0.0


def select_centerline_target(
    local_target_x_m: float,
    lookahead_m: float,
    turn_ratio: float,
    polynomial: PolynomialCenterline | None,
    maximum_disagreement_m: float = 0.10,
    local_heading_rad: float | None = None,
    maximum_heading_disagreement_rad: float = 0.25,
) -> tuple[float, str]:
    """S자 절삭을 피하면서 완만한 구간에서만 POLY2 목표를 채택한다."""

    if polynomial is None:
        return float(local_target_x_m), "LOCAL"
    if not (
        polynomial.minimum_y_m - 0.15
        <= float(lookahead_m)
        <= polynomial.maximum_y_m + 0.15
    ):
        return float(local_target_x_m), "LOCAL"
    polynomial_target_x = polynomial.x_at(lookahead_m)
    # 코너라는 이유만으로 POLY2를 전부 막으면 S자에서도 늘 짧은 LOCAL 점만
    # 쫓게 되어 좌우 사냥이 커진다. 코너일수록 허용 오차를 더 좁히고, 현재
    # 국소 목표와 방향까지 일치하는 모델만 채택한다. 다음 역커브를 평균내어
    # 부호가 뒤집힌 전역 2차식은 계속 LOCAL로 거부한다.
    turn = clamp(float(turn_ratio), 0.0, 1.0)
    disagreement_limit = max(
        0.02,
        float(maximum_disagreement_m) * (1.0 - 0.50 * turn),
    )
    direction_conflict = (
        abs(float(local_target_x_m)) > 0.03
        and abs(polynomial_target_x) > 0.03
        and float(local_target_x_m) * polynomial_target_x < 0.0
    )
    heading_conflict = False
    if local_heading_rad is not None:
        polynomial_heading = polynomial.heading_at(lookahead_m)
        heading_delta = abs(
            math.atan2(
                math.sin(polynomial_heading - float(local_heading_rad)),
                math.cos(polynomial_heading - float(local_heading_rad)),
            )
        )
        heading_conflict = heading_delta > max(
            0.05,
            float(maximum_heading_disagreement_rad),
        )
    if (
        direction_conflict
        or heading_conflict
        or abs(polynomial_target_x - float(local_target_x_m))
        > disagreement_limit
    ):
        return float(local_target_x_m), "LOCAL"
    return float(polynomial_target_x), "POLY2"


def compute_adaptive_lookahead(
    slices: list[CorridorSlice],
    straight_lookahead_m: float,
    corner_lookahead_m: float,
    bend_start_m: float,
    bend_full_m: float,
    boundary_heading_rad: float = 0.0,
    near_min_m: float = 0.30,
    near_max_m: float = 0.55,
    far_min_m: float = 0.75,
    far_max_m: float = 1.00,
    centerline_curvature_1pm: float = 0.0,
    curvature_start_1pm: float = 0.15,
    curvature_full_1pm: float = 0.90,
) -> tuple[float, float]:
    """직선은 멀리, 급커브는 가까이 보는 동적 룩어헤드를 계산한다.

    전역 평균은 S자에서 서로 반대인 중심점을 상쇄해 직진 목표를 만들 수 있다.
    근거리/원거리 중심의 중앙값 차이와 경계 방향을 함께 사용하면 실제 코너는
    일찍 감지하면서 단일 원거리 이상점의 영향은 작게 유지할 수 있다.
    """

    valid = [s for s in slices if s.center_x_m is not None]
    straight = max(0.05, float(straight_lookahead_m))
    corner = min(straight, max(0.05, float(corner_lookahead_m)))

    def median_x(y_min: float, y_max: float) -> float | None:
        values = [
            float(s.center_x_m)
            for s in valid
            if y_min <= s.y_m <= y_max
        ]
        return float(np.median(values)) if values else None

    near_x = median_x(float(near_min_m), float(near_max_m))
    far_x = median_x(float(far_min_m), float(far_max_m))
    bend = abs(far_x - near_x) if near_x is not None and far_x is not None else 0.0
    bend_span = max(1e-3, float(bend_full_m) - float(bend_start_m))
    bend_ratio = clamp((bend - float(bend_start_m)) / bend_span, 0.0, 1.0)

    # 단측 경계에서도 코너를 놓치지 않도록 기존 경계 기울기를 보조 지표로 쓴다.
    heading_ratio = clamp((abs(boundary_heading_rad) - 0.10) / 0.35, 0.0, 1.0)
    curvature_span = max(
        1e-3,
        float(curvature_full_1pm) - float(curvature_start_1pm),
    )
    curvature_ratio = clamp(
        (abs(float(centerline_curvature_1pm)) - float(curvature_start_1pm))
        / curvature_span,
        0.0,
        1.0,
    )
    turn_ratio = float(max(bend_ratio, heading_ratio, curvature_ratio))
    lookahead = straight - turn_ratio * (straight - corner)
    return float(lookahead), turn_ratio


def compute_target_point(
    slices: list[CorridorSlice],
    target_lookahead_m: float,
    fallback_margin_m: float,
    local_window_m: float = 0.15,
) -> tuple[float, float, float]:
    """룩어헤드 주변의 국소 중심선만 사용해 고속용 목표점을 계산한다."""
    del fallback_margin_m  # API 호환 유지. 단측 오프셋은 슬라이스 생성에서 적용한다.
    valid_slices = [s for s in slices if s.center_x_m is not None]
    if not valid_slices:
        return 0.0, target_lookahead_m, 0.0

    window = max(0.05, float(local_window_m))
    local = [
        s for s in valid_slices
        if abs(s.y_m - target_lookahead_m) <= window
    ]

    # 국소 창이 비면 가장 가까운 두 슬라이스까지만 사용한다. 25cm보다 먼
    # 끝단을 억지로 끌어오지 않아 존재하지 않는 직선 경로 생성을 막는다.
    if not local:
        nearest = sorted(valid_slices, key=lambda s: abs(s.y_m - target_lookahead_m))
        if not nearest or abs(nearest[0].y_m - target_lookahead_m) > 0.25:
            return 0.0, target_lookahead_m, 0.0
        local = [s for s in nearest[:2] if abs(s.y_m - target_lookahead_m) <= 0.25]

    # 양측 벽 중심이 두 개 이상 있으면 단측 폴백 슬라이스는 목표 계산에서 제외한다.
    both_local = [s for s in local if not s.is_interpolated]
    selected = both_local if len(both_local) >= 2 else local

    xs = np.array([float(s.center_x_m) for s in selected], dtype=np.float64)
    median_x = float(np.median(xs))
    trimmed = [
        s for s in selected
        if abs(float(s.center_x_m) - median_x) <= 0.15
    ]
    if trimmed:
        selected = trimmed

    weights: list[float] = []
    target_x_values: list[float] = []
    target_y_values: list[float] = []
    for s in selected:
        dist = abs(s.y_m - target_lookahead_m)
        weight = 1.0 / (dist * dist + 0.01)
        if s.is_interpolated:
            weight *= 0.50
        weights.append(weight)
        target_x_values.append(float(s.center_x_m))
        target_y_values.append(float(s.y_m))

    total_weight = sum(weights)
    if total_weight <= 0.0:
        return 0.0, target_lookahead_m, 0.0

    target_x = float(
        sum(w * x for w, x in zip(weights, target_x_values)) / total_weight
    )
    # LOCAL 목표는 실제로 관측된 중심선의 x와 y를 한 쌍으로 사용한다.
    # 가까운 슬라이스에서 x를 얻고 요청 lookahead를 y로 넣으면 같은 횡오차를
    # 실제보다 먼 목표로 해석해 급커브 조향이 약해진다. POLY2 모델만 아래
    # analyze 단계에서 정확한 lookahead y로 다시 설정한다.
    target_y = float(
        sum(w * y for w, y in zip(weights, target_y_values)) / total_weight
    )
    both_count = sum(1 for s in valid_slices if not s.is_interpolated)
    confidence = min(
        1.0,
        (both_count * 2.0 + len(valid_slices)) / (len(slices) * 1.5 + 1e-6),
    )
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
    target_lookahead_override_m: float | None = None,
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
    left_candidate_count = sum(1 for x, _ in cone_centers if x > 0.05)
    right_candidate_count = sum(1 for x, _ in cone_centers if x < -0.05)

    if total_count < config.detection_min_total_points or not cone_centers:
        return replace(
            EMPTY_CORRIDOR_RESULT,
            left_points_count=left_count,
            right_points_count=right_count,
            left_candidates_count=left_candidate_count,
            right_candidates_count=right_candidate_count,
            candidate_centers_xy=tuple(cone_centers),
        )

    # 1순위: 통로 폭에 맞는 좌/우 단면들을 동시에 연결한다. 이 단계가 성공하면
    # 한 경계가 반대편 콘을 훔칠 수 없고, 모든 중심점은 실제 양쪽 경계의 중점이다.
    left_cones, right_cones = chain_paired_boundaries(cone_centers, config)
    paired_chain_used = bool(left_cones and right_cones)
    boundary_fallback_reason = "NONE"

    # 2순위: 양쪽 단면 체인이 만들어지지 않을 때만 단측 경계를 찾는다.
    # 경계의 좌/우 정체성은 차량 가까이의 root가 결정한다. 급커브에서 실제
    # 외벽은 차량 좌표 x=0을 넘어갈 수 있으므로 이후 체인은 같은 반평면으로
    # 자르지 않고 남은 전체 콘에서 연결한다. 먼저 선택한 물리 경계의 콘은
    # 제거한 뒤 반대 경계를 만들어 두 경계가 같은 콘을 공유하지 않게 한다.
    boundary_root_tolerance = max(
        0.05,
        config.boundary_root_lateral_tolerance_m,
    )
    left_roots = [
        c
        for c in cone_centers
        if c[0] > 0.05 and c[1] <= config.root_maximum_y_m
        and abs(c[0] - config.single_wall_fallback_margin_m)
        <= boundary_root_tolerance
    ]
    right_roots = [
        c
        for c in cone_centers
        if c[0] < -0.05 and c[1] <= config.root_maximum_y_m
        and abs(c[0] + config.single_wall_fallback_margin_m)
        <= boundary_root_tolerance
    ]
    if not left_cones or not right_cones:
        boundary_fallback_reason = "PAIR_CHAIN_UNAVAILABLE"

        def build_single_chain(
            roots: list[tuple[float, float]],
            available_cones: list[tuple[float, float]],
            expected_root_x_m: float,
        ) -> list[tuple[float, float]]:
            available_roots = [root for root in roots if root in available_cones]
            if not available_roots:
                return []
            return chain_cone_boundary(
                available_roots,
                available_cones,
                config.max_cone_link_distance_m,
                config.far_max_cone_link_distance_m,
                config.link_distance_far_start_m,
                config.link_distance_far_full_m,
                config.boundary_maximum_heading_change_rad,
                expected_root_x_m,
            )

        def nearest_root_distance(roots: list[tuple[float, float]]) -> float:
            return min(
                (math.hypot(root[0], root[1]) for root in roots),
                default=float("inf"),
            )

        available = list(cone_centers)
        left_first = nearest_root_distance(left_roots) <= nearest_root_distance(
            right_roots
        )
        if left_first:
            left_cones = build_single_chain(
                left_roots,
                available,
                config.single_wall_fallback_margin_m,
            )
            used = set(left_cones)
            available = [cone for cone in available if cone not in used]
            right_cones = build_single_chain(
                right_roots,
                available,
                -config.single_wall_fallback_margin_m,
            )
        else:
            right_cones = build_single_chain(
                right_roots,
                available,
                -config.single_wall_fallback_margin_m,
            )
            used = set(right_cones)
            available = [cone for cone in available if cone not in used]
            left_cones = build_single_chain(
                left_roots,
                available,
                config.single_wall_fallback_margin_m,
            )

    # 단측 고립 모서리 하나는 계속 차단한다. 다만 좌우 콘 한 쌍이 같은 y에서
    # 정상 통로 폭을 이루면 원거리 반사 누락 프레임을 위한 희소 양측 단면으로
    # 인정한다. 이 경우 속도는 아래에서 코너 속도로 제한한다.
    sparse_paired_boundary = bool(
        left_cones
        and right_cones
        and len(left_cones) == len(right_cones)
        and len(left_cones) >= config.minimum_paired_boundary_pairs
        and len(left_cones) < config.minimum_boundary_cones
    )
    if not sparse_paired_boundary:
        if len(left_cones) < config.minimum_boundary_cones:
            left_cones = []
        if len(right_cones) < config.minimum_boundary_cones:
            right_cones = []

    # 라바콘을 연결한 가상 폴리라인 벽으로 슬라이스 생성
    slices = build_polyline_corridor_slices(left_cones, right_cones, config)
    paired_slice_count = sum(
        1
        for corridor_slice in slices
        if corridor_slice.center_x_m is not None
        and not corridor_slice.is_interpolated
    )
    # 두 체인이 잡혀도 유효한 통로 폭이 연속해서 성립하지 않으면 양측 경계로
    # 취급하지 않는다. 슬라이스마다 좌/우를 번갈아 고르는 대신 신뢰도 높은
    # 한쪽 경계 하나만 고정해 중심 경로가 좌우로 점프하는 것을 막는다.
    if left_cones and right_cones and paired_slice_count < config.minimum_boundary_cones:
        boundary_fallback_reason = "WIDTH_SLICES_REJECTED"
        left_cones, right_cones = _select_single_boundary(
            left_cones,
            right_cones,
            config,
        )
        slices = build_polyline_corridor_slices(left_cones, right_cones, config)
    valid_slices = [s for s in slices if s.center_x_m is not None]

    if len(valid_slices) < config.detection_min_slices:
        return replace(
            EMPTY_CORRIDOR_RESULT,
            left_points_count=left_count,
            right_points_count=right_count,
            left_cones_count=len(left_cones),
            right_cones_count=len(right_cones),
            left_candidates_count=left_candidate_count,
            right_candidates_count=right_candidate_count,
            candidate_centers_xy=tuple(cone_centers),
            left_boundary_xy=tuple(left_cones),
            right_boundary_xy=tuple(right_cones),
            boundary_mode="NONE",
            boundary_fallback_reason=boundary_fallback_reason,
            paired_slice_count=paired_slice_count,
        )

    # 코너 벽면 방향을 먼저 계산해 룩어헤드와 선제 감속에도 사용한다.
    boundary_heading = estimate_local_centerline_heading(
        slices,
        min(config.target_lookahead_m, 0.75),
        0.30,
    )

    if left_cones and right_cones:
        if sparse_paired_boundary:
            boundary_mode = "SPARSE_PAIR"
        elif paired_chain_used:
            boundary_mode = "PAIRED"
        else:
            boundary_mode = "DUAL_FALLBACK"
    elif left_cones:
        boundary_mode = "LEFT_FALLBACK"
    elif right_cones:
        boundary_mode = "RIGHT_FALLBACK"
    else:
        boundary_mode = "NONE"

    # 두 물리 경계에 각각 최소 3개 콘이 있을 때만 2차 모델을 만든다.
    # 한두 쌍의 끝단 extrapolation이 여러 슬라이스로 부풀려져 완벽한
    # 모델처럼 보이는 현상을 차단한다.
    polynomial = None
    if len(left_cones) >= 3 and len(right_cones) >= 3:
        polynomial = fit_polynomial_centerline(
            slices,
            config.polynomial_fit_min_y_m,
            config.polynomial_fit_max_y_m,
            config.polynomial_fit_min_points,
            config.polynomial_fit_outlier_tolerance_m,
            config.polynomial_fit_maximum_rmse_m,
        )
    polynomial_curvature = 0.0
    if polynomial is not None:
        curvature_y = clamp(
            config.target_lookahead_m,
            polynomial.minimum_y_m,
            polynomial.maximum_y_m,
        )
        polynomial_curvature = polynomial.curvature_at(curvature_y)

    adaptive_lookahead, lookahead_turn_ratio = compute_adaptive_lookahead(
        slices,
        config.target_lookahead_m,
        config.corner_target_lookahead_m,
        config.lookahead_bend_start_m,
        config.lookahead_bend_full_m,
        boundary_heading,
        config.lookahead_near_min_m,
        config.lookahead_near_max_m,
        config.lookahead_far_min_m,
        config.lookahead_far_max_m,
        polynomial_curvature,
        config.lookahead_curvature_start_1pm,
        config.lookahead_curvature_full_1pm,
    )
    effective_lookahead = adaptive_lookahead
    if target_lookahead_override_m is not None:
        lower = min(config.corner_target_lookahead_m, config.target_lookahead_m)
        upper = max(config.corner_target_lookahead_m, config.target_lookahead_m)
        effective_lookahead = float(
            clamp(target_lookahead_override_m, lower, upper)
        )
    local_target_x, target_y, confidence = compute_target_point(
        slices,
        effective_lookahead,
        config.single_wall_fallback_margin_m,
        config.target_local_window_m,
    )
    local_heading_reference = estimate_local_centerline_heading(
        slices,
        effective_lookahead,
        max(0.20, config.target_local_window_m * 2.0),
    )
    # left-right-left S 전체를 2차식 하나로 평균내면 현재 커브의 부호가
    # 뒤집힐 수 있다. 완만하고 국소 목표와 일치할 때만 POLY2를 쓴다.
    target_x, path_model = select_centerline_target(
        local_target_x,
        effective_lookahead,
        lookahead_turn_ratio,
        polynomial,
        local_heading_rad=local_heading_reference,
    )
    if path_model == "POLY2" and polynomial is not None:
        target_y = effective_lookahead
        fit_support = min(
            1.0,
            polynomial.points_used
            / max(1.0, config.polynomial_fit_min_points * 2.0),
        )
        fit_quality = clamp(
            1.0
            - polynomial.rmse_m
            / max(0.01, config.polynomial_fit_maximum_rmse_m),
            0.0,
            1.0,
        )
        confidence = max(confidence, float(fit_support * fit_quality))
    if confidence <= 0.0:
        return replace(
            EMPTY_CORRIDOR_RESULT,
            target_y_m=effective_lookahead,
            left_points_count=left_count,
            right_points_count=right_count,
            left_cones_count=len(left_cones),
            right_cones_count=len(right_cones),
            left_candidates_count=left_candidate_count,
            right_candidates_count=right_candidate_count,
            suggested_lookahead_m=adaptive_lookahead,
            turn_ratio=lookahead_turn_ratio,
            candidate_centers_xy=tuple(cone_centers),
            left_boundary_xy=tuple(left_cones),
            right_boundary_xy=tuple(right_cones),
            boundary_mode=boundary_mode,
            boundary_fallback_reason=boundary_fallback_reason,
            paired_slice_count=paired_slice_count,
            boundary_heading_rad=boundary_heading,
        )

    heading_reference = local_heading_reference
    if polynomial is not None and path_model == "POLY2":
        heading_reference = polynomial.heading_at(effective_lookahead)
    heading_feedforward = heading_reference * config.heading_gain

    # 통로 폭 추정 (양쪽 벽이 정상 폭으로 잡힌 슬라이스들의 평균 폭)
    widths = [
        (s.left_x_m - s.right_x_m)
        for s in slices
        if s.left_x_m is not None
        and s.right_x_m is not None
        and (
            config.corridor_width_min_m
            <= (s.left_x_m - s.right_x_m)
            <= config.corridor_width_max_m
        )
    ]
    corridor_width_est = float(np.mean(widths)) if widths else None

    # 경계 검출과 lookahead 선택은 기존 라이다 좌표계로 유지한다. 조향 계산만
    # 실제 Ackermann 제어 기준인 뒷바퀴 축 중심으로 평행 이동한다. 라이다가
    # 차량 앞에 있을 때 같은 물리 목표점은 뒷축에서 그만큼 더 멀리 있다.
    lidar_to_rear_axle = max(0.0, float(config.lidar_to_rear_axle_m))
    control_target_y = max(0.05, float(target_y) + lidar_to_rear_axle)
    centerline_target_x = float(target_x)
    centerline_steering = compute_steering_pure_pursuit(
        target_x,
        control_target_y,
        config.wheelbase_m,
        config.max_steering_rad,
        config.steering_gain,
        heading_feedforward,
    )

    # 좌회전(+)에서는 목표를 오른쪽(-), 우회전(-)에서는 왼쪽(+)으로 옮겨
    # 차체 안쪽이 경계를 자르는 것을 줄인다. 방향별 요청값은 최대값이며,
    # 실제 통로 폭에서 차체 반폭과 최소 여유를 뺀 범위를 넘지 않는다.
    steering_strength = clamp(
        abs(centerline_steering) / max(1.0e-6, config.max_steering_rad),
        0.0,
        1.0,
    )
    curve_strength = float(max(lookahead_turn_ratio, steering_strength))
    bias_span = max(
        1.0e-3,
        float(config.curve_outward_bias_full_ratio)
        - float(config.curve_outward_bias_start_ratio),
    )
    bias_ratio = clamp(
        (curve_strength - float(config.curve_outward_bias_start_ratio)) / bias_span,
        0.0,
        1.0,
    )
    clearance_width = (
        float(corridor_width_est)
        if corridor_width_est is not None
        else max(0.0, float(config.corridor_width_m))
    )
    geometric_bias_limit = max(
        0.0,
        clearance_width * 0.5
        - max(0.0, float(config.vehicle_width_m)) * 0.5
        - max(0.0, float(config.minimum_boundary_clearance_m)),
    )
    requested_bias = float(config.curve_outward_bias_max_m)
    if centerline_steering > 1.0e-4:
        directional_bias = float(config.curve_outward_bias_left_max_m)
        if directional_bias >= 0.0:
            requested_bias = directional_bias
    elif centerline_steering < -1.0e-4:
        directional_bias = float(config.curve_outward_bias_right_max_m)
        if directional_bias >= 0.0:
            requested_bias = directional_bias
    maximum_bias = min(max(0.0, requested_bias), geometric_bias_limit)
    curve_outward_bias = maximum_bias * bias_ratio
    if abs(centerline_steering) > 1.0e-4 and curve_outward_bias > 0.0:
        target_x = float(
            centerline_target_x
            - math.copysign(curve_outward_bias, centerline_steering)
        )
    steering = compute_steering_pure_pursuit(
        target_x,
        control_target_y,
        config.wheelbase_m,
        config.max_steering_rad,
        config.steering_gain,
        heading_feedforward,
    )

    # 주된 관측 벽
    if left_cones and right_cones:
        primary_side = "BOTH"
    elif left_cones:
        primary_side = "LEFT_ONLY"
    elif right_cones:
        primary_side = "RIGHT_ONLY"
    else:
        primary_side = "NONE"

    # 현재 조향뿐 아니라 전방 중심선 굴곡도 함께 사용한다. 따라서 조향이 아직
    # 작아도 앞쪽 코너가 확인되면 0.80 -> 0.70m/s 범위에서 선제 감속한다.
    deadzone = config.max_steering_rad * 0.35  # 약 9도 이내 선회는 감속 없이 최고속도 유지
    steer_ratio = clamp(
        (abs(steering) - deadzone)
        / max(0.01, config.max_steering_rad * 0.80 - deadzone),
        0.0,
        1.0,
    )
    turn_ratio = float(max(steer_ratio, lookahead_turn_ratio))
    speed = config.drive_speed_mps - turn_ratio * (
        config.drive_speed_mps - config.corner_speed_mps
    )
    speed = float(max(config.corner_speed_mps, min(config.drive_speed_mps, speed)))
    # 단측 경계는 순간 가림을 통과하기 위한 fallback이지 레이싱 경로가 아니다.
    # 양쪽 경계가 다시 잡힐 때까지 코너 속도 이상으로 가속하지 않는다.
    if primary_side != "BOTH" or sparse_paired_boundary:
        speed = min(speed, config.fallback_speed_mps)

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
        control_target_y_m=control_target_y,
        centerline_target_x_m=centerline_target_x,
        curve_outward_bias_m=curve_outward_bias,
        left_cones_count=len(left_cones),
        right_cones_count=len(right_cones),
        left_candidates_count=left_candidate_count,
        right_candidates_count=right_candidate_count,
        suggested_lookahead_m=adaptive_lookahead,
        turn_ratio=lookahead_turn_ratio,
        path_model=path_model,
        path_curvature_1pm=polynomial_curvature,
        polynomial_rmse_m=polynomial.rmse_m if polynomial is not None else None,
        candidate_centers_xy=tuple(cone_centers),
        left_boundary_xy=tuple(left_cones),
        right_boundary_xy=tuple(right_cones),
        boundary_mode=boundary_mode,
        boundary_fallback_reason=boundary_fallback_reason,
        paired_slice_count=paired_slice_count,
        boundary_heading_rad=boundary_heading,
    )

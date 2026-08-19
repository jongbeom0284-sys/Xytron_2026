"""중앙선을 기준으로 좌우 차선을 재고, 어느 선을 따라 달릴지 정한다.

원칙은 중앙선 위 — 2차선 전체 폭을 쓰며 달린다. 코너 반경을 최대로 확보하기 위해서다.
한쪽 차선이 장애물 차량에 막히면 반대편 차선 중앙으로 비켜서고, 추월이 끝나면
다시 중앙으로 돌아온다.

점유 판정은 네 근거가 모두 있어야 확정된다.
  1. 카메라: 그쪽 차선의 주행 가능 폭이 반대편보다 뚜렷하게 좁다
  2. 카메라: 그 차선 자신의 평상시 폭 대비 뚜렷하게 줄었다 (벽을 낀 구간 배제)
  3. 라이다: 같은 쪽 옆 차선 코리도에 임계 거리 이내의 물체가 있다
  4. 기하: 지금 직선 구간이다 (대회 장애물은 직선 차선에만 놓인다)
라이다만으로는 확정하지 않는다. 카메라만으로도 확정하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import clamp, move_towards
from .obstacle_lidar import LidarSectors, side_is_occupied

CENTER = "CENTER"
SHIFT_LEFT = "SHIFT_LEFT"
SHIFT_RIGHT = "SHIFT_RIGHT"
RETURNING = "RETURNING"

_OPPOSITE = {"left": "right", "right": "left"}
_SHIFT_STATE = {"left": SHIFT_LEFT, "right": SHIFT_RIGHT}


@dataclass(frozen=True)
class LanePolicyConfig:
    sample_rows: int = 14
    roi_top_ratio: float = 0.52
    roi_bottom_ratio: float = 0.95

    # 차선으로 인정할 최소 폭 (프레임 폭 대비)
    minimum_lane_width_ratio: float = 0.05
    # 반대편 대비 이 비율 이하로 좁아진 행을 "좁은 행"으로 센다
    narrow_width_ratio: float = 0.55
    narrow_rows_required: int = 4

    # 좁다는 것만으로는 장애물이 아니다. 벽을 낀 구간은 원래 한쪽이 좁다.
    # 그 차선 자신의 최근 평균 대비 뚜렷하게 줄어들어야 장애물로 본다.
    width_drop_ratio: float = 0.60
    baseline_alpha: float = 0.05
    baseline_warmup_frames: int = 20

    lidar_confirm_distance_m: float = 1.60
    lidar_maximum_age_sec: float = 0.50

    # 회피는 직선 구간에서만 허용한다. 코너에서는 목표선이 크게 기울어 좌우 폭이
    # 원근으로 왜곡되고, 어차피 대회 장애물은 직선 차선에만 놓인다.
    straight_only: bool = True
    maximum_heading_deg: float = 12.0

    enter_hold_sec: float = 0.30
    clear_hold_sec: float = 0.40
    pass_distance_m: float = 0.70

    target_rate_px_s: float = 260.0
    shift_speed_scale: float = 0.75
    return_tolerance_px: float = 12.0


@dataclass(frozen=True)
class LaneOccupancy:
    left_width_px: float
    right_width_px: float
    left_center_px: float | None
    right_center_px: float | None
    narrow_side: str | None
    measured_rows: int


@dataclass(frozen=True)
class LaneDecision:
    target_x_px: float
    state: str
    blocked_side: str | None
    speed_scale: float
    reason: str


EMPTY_OCCUPANCY = LaneOccupancy(0.0, 0.0, None, None, None, 0)


def _run_bounds(row: np.ndarray, origin: int, direction: int) -> tuple[int, int] | None:
    """origin 에서 direction 방향으로 이어지는 True 구간의 [시작, 끝]."""
    width = row.size
    index = origin
    while 0 <= index < width and not row[index]:
        index += direction
        if abs(index - origin) > width:
            return None
    if not (0 <= index < width) or not row[index]:
        return None
    start = end = index
    while 0 <= start - 1 and row[start - 1]:
        start -= 1
    while end + 1 < width and row[end + 1]:
        end += 1
    return start, end


def measure_lanes(
    driving_mask: np.ndarray,
    fitted_points: np.ndarray,
    config: LanePolicyConfig | None = None,
) -> LaneOccupancy:
    """중앙선 좌우의 주행 가능 폭을 잰다.

    fitted_points 는 (x, y) 정수쌍 배열이다. 중앙선이 없으면 잴 수 없으므로
    빈 결과를 준다.
    """
    config = config or LanePolicyConfig()
    if fitted_points is None or len(fitted_points) == 0:
        return EMPTY_OCCUPANCY

    driving = driving_mask.astype(bool)
    height, width = driving.shape[:2]
    y_top = int(round(height * config.roi_top_ratio))
    y_bottom = min(height - 1, int(round(height * config.roi_bottom_ratio)))

    inside = fitted_points[
        (fitted_points[:, 1] >= y_top) & (fitted_points[:, 1] <= y_bottom)
    ]
    if len(inside) == 0:
        return EMPTY_OCCUPANCY
    step = max(1, len(inside) // config.sample_rows)
    selected = inside[::step]

    minimum_width = max(4.0, width * config.minimum_lane_width_ratio)
    left_widths: list[float] = []
    right_widths: list[float] = []
    left_centers: list[float] = []
    right_centers: list[float] = []
    narrow_left = 0
    narrow_right = 0

    for x, y in selected:
        row = driving[int(y)]
        left = _run_bounds(row, max(0, int(x) - 1), -1)
        right = _run_bounds(row, min(width - 1, int(x) + 1), 1)
        left_width = float(left[1] - left[0] + 1) if left else 0.0
        right_width = float(right[1] - right[0] + 1) if right else 0.0
        left_widths.append(left_width)
        right_widths.append(right_width)
        if left and left_width >= minimum_width:
            left_centers.append((left[0] + left[1]) * 0.5)
        if right and right_width >= minimum_width:
            right_centers.append((right[0] + right[1]) * 0.5)
        if right_width > 0.0 and left_width < right_width * config.narrow_width_ratio:
            narrow_left += 1
        if left_width > 0.0 and right_width < left_width * config.narrow_width_ratio:
            narrow_right += 1

    left_mean = float(np.mean(left_widths)) if left_widths else 0.0
    right_mean = float(np.mean(right_widths)) if right_widths else 0.0

    # 행 단위 카운트만으로는 부족하다. 원근 때문에 ROI 위쪽 몇 행은 늘 좁게 나오고,
    # 그것만으로 차선을 넘으면 안 된다. 평균 폭까지 뚜렷하게 좁아야 점유 후보로 본다.
    narrow_side = None
    left_narrow = (
        narrow_left >= config.narrow_rows_required
        and narrow_left > narrow_right
        and left_mean < right_mean * config.narrow_width_ratio
    )
    right_narrow = (
        narrow_right >= config.narrow_rows_required
        and narrow_right > narrow_left
        and right_mean < left_mean * config.narrow_width_ratio
    )
    if left_narrow:
        narrow_side = "left"
    elif right_narrow:
        narrow_side = "right"

    return LaneOccupancy(
        left_width_px=left_mean,
        right_width_px=right_mean,
        left_center_px=float(np.mean(left_centers)) if left_centers else None,
        right_center_px=float(np.mean(right_centers)) if right_centers else None,
        narrow_side=narrow_side,
        measured_rows=len(selected),
    )


class LanePolicy:
    """주행 라인 상태기계.

    CENTER ──점유 확정(좌)──> SHIFT_RIGHT ─┐
           ──점유 확정(우)──> SHIFT_LEFT  ─┴─통과 판정─> RETURNING ─> CENTER

    상태 전이에는 최소 유지 시간을 둬 한 프레임짜리 오검출로 차선을 넘나들지 않게 한다.
    목표 x 는 레이트 리밋으로만 움직인다. 급전환은 조향 포화를 부른다.
    """

    def __init__(self, config: LanePolicyConfig | None = None) -> None:
        self._config = config or LanePolicyConfig()
        self._state = CENTER
        self._blocked_side: str | None = None
        self._candidate_side: str | None = None
        self._candidate_since = 0.0
        self._clear_since = 0.0
        self._passed_m = 0.0
        self._target_x: float | None = None
        self._baseline: dict[str, float] = {"left": 0.0, "right": 0.0}
        self._baseline_frames = 0

    @property
    def state(self) -> str:
        return self._state

    @property
    def baseline_widths(self) -> dict[str, float]:
        return dict(self._baseline)

    @property
    def blocked_side(self) -> str | None:
        return self._blocked_side

    def reset(self) -> None:
        self._state = CENTER
        self._blocked_side = None
        self._candidate_side = None
        self._candidate_since = 0.0
        self._clear_since = 0.0
        self._passed_m = 0.0
        self._target_x = None

    def update(
        self,
        center_x_px: float,
        occupancy: LaneOccupancy,
        sectors: LidarSectors,
        speed_mps: float,
        dt: float,
        now: float,
        frame_width: int,
        maneuvers_enabled: bool = True,
        heading_deg: float | None = None,
    ) -> LaneDecision:
        config = self._config
        if self._target_x is None:
            self._target_x = center_x_px

        if not maneuvers_enabled:
            if self._state != CENTER:
                self.reset()
                self._target_x = center_x_px
            return self._decide(center_x_px, CENTER, None, dt, frame_width, "maneuvers_disabled")

        self._update_baseline(occupancy)
        confirmed = self._confirmed_side(occupancy, sectors, now, heading_deg)
        self._advance_state(confirmed, speed_mps, dt, now)

        # 상태를 먼저 옮긴 뒤 목표를 정한다. 전환된 프레임부터 새 차선을 향하게 하기
        # 위해서다. 실제 이동은 레이트 리밋이 걸리므로 급전환이 되지는 않는다.
        if self._state in (SHIFT_LEFT, SHIFT_RIGHT):
            free_side = "left" if self._state == SHIFT_LEFT else "right"
            desired = self._free_lane_center(occupancy, free_side, center_x_px)
            reason = f"driving_{free_side}_lane"
        elif self._state == RETURNING:
            desired = center_x_px
            reason = "returning_to_center"
        else:
            desired = center_x_px
            reason = "center_line"

        decision = self._decide(
            desired, self._state, self._blocked_side, dt, frame_width, reason
        )
        if (
            self._state == RETURNING
            and abs(decision.target_x_px - center_x_px) <= config.return_tolerance_px
        ):
            self._state = CENTER
            self._blocked_side = None
            self._candidate_side = None
            self._passed_m = 0.0
            return LaneDecision(
                target_x_px=decision.target_x_px,
                state=CENTER,
                blocked_side=None,
                speed_scale=1.0,
                reason="returned_to_center",
            )
        return decision

    def _advance_state(
        self, confirmed: str | None, speed_mps: float, dt: float, now: float
    ) -> None:
        config = self._config
        if self._state == CENTER:
            if confirmed is None:
                self._candidate_side = None
                return
            if self._candidate_side != confirmed:
                self._candidate_side = confirmed
                self._candidate_since = now
            elif now - self._candidate_since >= config.enter_hold_sec:
                self._state = _SHIFT_STATE[_OPPOSITE[confirmed]]
                self._blocked_side = confirmed
                self._candidate_side = None
                self._passed_m = 0.0
                self._clear_since = 0.0
            return

        if self._state in (SHIFT_LEFT, SHIFT_RIGHT):
            self._passed_m += max(0.0, speed_mps) * max(0.0, dt)
            if confirmed == self._blocked_side:
                self._clear_since = 0.0
            elif self._clear_since == 0.0:
                self._clear_since = now
            elif (
                now - self._clear_since >= config.clear_hold_sec
                and self._passed_m >= config.pass_distance_m
            ):
                self._state = RETURNING

    def _update_baseline(self, occupancy: LaneOccupancy) -> None:
        """차선별 평상시 폭을 천천히 학습한다.

        회피 중이거나 이미 좁아진 프레임에서는 갱신하지 않는다. 그러지 않으면
        장애물 옆을 지나는 동안 좁은 폭이 새 기준이 되어 복귀 판정이 무너진다.
        """
        if occupancy.measured_rows == 0 or self._state != CENTER:
            return
        if occupancy.narrow_side is not None:
            return
        alpha = self._config.baseline_alpha
        for side, width in (
            ("left", occupancy.left_width_px),
            ("right", occupancy.right_width_px),
        ):
            if width <= 0.0:
                return
            current = self._baseline[side]
            self._baseline[side] = width if current <= 0.0 else current + alpha * (width - current)
        self._baseline_frames += 1

    def _confirmed_side(
        self,
        occupancy: LaneOccupancy,
        sectors: LidarSectors,
        now: float,
        heading_deg: float | None,
    ) -> str | None:
        side = occupancy.narrow_side
        if side is None:
            return None
        if (
            self._config.straight_only
            and heading_deg is not None
            and abs(heading_deg) > self._config.maximum_heading_deg
        ):
            return None
        # 평상시 폭을 아직 배우지 못했으면 판단을 보류한다 (닫힘 방향으로 실패).
        if self._baseline_frames < self._config.baseline_warmup_frames:
            return None
        baseline = self._baseline[side]
        current = occupancy.left_width_px if side == "left" else occupancy.right_width_px
        if baseline <= 0.0 or current > baseline * self._config.width_drop_ratio:
            return None
        if not side_is_occupied(
            sectors,
            side,
            self._config.lidar_confirm_distance_m,
            now,
            self._config.lidar_maximum_age_sec,
        ):
            return None
        return side

    def _free_lane_center(
        self, occupancy: LaneOccupancy, side: str, fallback: float
    ) -> float:
        center = occupancy.left_center_px if side == "left" else occupancy.right_center_px
        return fallback if center is None else center

    def _decide(
        self,
        desired_x: float,
        state: str,
        blocked_side: str | None,
        dt: float,
        frame_width: int,
        reason: str,
    ) -> LaneDecision:
        step = self._config.target_rate_px_s * max(0.0, dt)
        self._target_x = clamp(
            move_towards(float(self._target_x), float(desired_x), step),
            0.0,
            float(max(1, frame_width - 1)),
        )
        speed_scale = (
            self._config.shift_speed_scale
            if state in (SHIFT_LEFT, SHIFT_RIGHT, RETURNING)
            else 1.0
        )
        return LaneDecision(
            target_x_px=self._target_x,
            state=state,
            blocked_side=blocked_side,
            speed_scale=speed_scale,
            reason=reason,
        )

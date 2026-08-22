"""차로 좌표 — "내 차로를 막고 있는가" 와 "차로 기준 어느 쪽인가".

왜 별도 모듈인가 (0821):
  같은 기하를 **세 곳**이 필요로 한다.
    · race_fsm  — 회피/추월 **트리거** ("차로를 막고 있는 물체가 있는가")
    · policy    — 종방향 governor ("얼마나 앞에 있는가" → 속도 상한)
    · policy    — 추월 **방향과 이동량** ("차로 기준 어느 쪽에 얼마나")
  0820 저녁에 FSM 안의 private 메서드로 들어간 뒤 policy 가 같은 판단을
  필요로 하게 됐는데, 복사하면 두 판정이 조용히 어긋난다 (한쪽만 튜닝됨).

★ 차로 좌표가 왜 중요한가 — 0821 실측으로 확인한 두 가지.

  (1) **규정이 차로 기준이다.** p.33 "방해차량이 1차선에서 주행하고 있다면
      오른쪽(2차선)으로 추월해야 함". 1차선/2차선은 노란 중앙선이 가르는
      것이지 우리 차체가 가르는 게 아니다. 우리가 차로 중앙에서 0.2m 만
      어긋나 있으면 차선 중앙(±0.206m)에 선 차가 차량 좌표로는 ±0.006m 로
      보여 "정면"이 된다 — 규정과 무관한 근거로 방향이 정해진다.

  (2) **차량 좌표를 쓰면 횡이동이 절반만 된다 (충돌).** 이동량 식은
      d = clearance − side·y 인데, y 를 차량 기준으로 매 틱 다시 읽으면
      내가 d 만큼 옮기는 동안 타겟도 그만큼 멀어 보여서 목표가 같이 물러난다.
      고정점이 o = (clearance − y_lane)/2 다. 0821 폐루프 재현:
          의도 clearance 0.350 m → 수렴한 중심간 거리 0.303 m
          차폭 0.30 m 두 대 → **차체 간격 0.003 m** (긁는다)
      차로 좌표에서는 내 움직임이 y_lane 을 바꾸지 않으므로 목표가 고정되고
      의도한 값으로 수렴한다.

부호 규약: y 는 좌측이 +. 차로 좌표 y_lane 도 같다 (0 = 차로 중심선).
"""

from __future__ import annotations

from typing import Optional

from .types import Observation, TrackedObject

# BEV 축척 (0818 저녁 실측 — 반차선 335px ↔ 0.41m). 세로는 bev_aspect 0.9 가정.
PX_PER_M_LAT = 810.0
PX_PER_M_FWD = 730.0


def lane_center_y(
    obs: Observation,
    x_m: float,
    *,
    px_per_m_lat: float = PX_PER_M_LAT,
    px_per_m_fwd: float = PX_PER_M_FWD,
) -> float:
    """전방 x[m] 지점에서 **차로 중심선**의 횡위치 [m] (차량 기준, 좌 +).

    offset_px > 0 = 목표선이 화면 중앙보다 오른쪽 = 차로가 내 오른쪽에 있다.
    heading_slope 는 BEV 픽셀 기울기(dx_px/df_px)라 가로·세로 축척이 달라
    미터 기울기로 환산해야 한다: (dx/810)/(df/730) = slope·730/810.
    """
    lane = obs.lane
    y0 = -lane.offset_px / px_per_m_lat
    slope_m = lane.heading_slope * (px_per_m_fwd / px_per_m_lat)
    return y0 - slope_m * x_m


def lane_lateral_of(
    obs: Observation,
    ob: TrackedObject,
    *,
    px_per_m_lat: float = PX_PER_M_LAT,
    px_per_m_fwd: float = PX_PER_M_FWD,
) -> float:
    """물체의 **차로 기준** 횡위치 [m]. + = 차로 중심선의 왼쪽(=1차선 쪽)."""
    return ob.y - lane_center_y(
        obs, ob.x, px_per_m_lat=px_per_m_lat, px_per_m_fwd=px_per_m_fwd
    )


def lane_front_target(
    obs: Observation,
    *,
    half_m: float,
    min_dist_m: float,
    max_dist_m: float,
    use_lane: bool = True,
    px_per_m_lat: float = PX_PER_M_LAT,
    px_per_m_fwd: float = PX_PER_M_FWD,
) -> Optional[TrackedObject]:
    """전방에서 '내 차로를 막고 있는' 가장 가까운 물체.

    차선이 유효하면 **차로를 따라 휘는** 코리도로 판정한다. 직사각형 코리도는
    코너에서 바깥 벽을 구조적으로 삼킨다 (0820 실측: 랩당 회피 45~65회).
    차선이 없으면 옛 직사각형으로 폴백하되 **거리창은 같이 쓴다** — 상한이
    없으면 2~2.5m 앞 코너 벽까지 타겟이 된다 (0820 밤 감사).
    """
    if not (use_lane and obs.lane.valid):
        ob = obs.lidar.nearest_front()
        if ob is None or ob.x < min_dist_m or ob.x > max_dist_m:
            return None
        return ob

    best: Optional[TrackedObject] = None
    for ob in obs.lidar.objects:
        if ob.x < min_dist_m or ob.x > max_dist_m:
            continue
        y_lane = lane_lateral_of(
            obs, ob, px_per_m_lat=px_per_m_lat, px_per_m_fwd=px_per_m_fwd
        )
        if abs(y_lane) > half_m:
            continue
        if best is None or ob.x < best.x:
            best = ob
    return best


__all__ = [
    "PX_PER_M_LAT",
    "PX_PER_M_FWD",
    "lane_center_y",
    "lane_lateral_of",
    "lane_front_target",
]

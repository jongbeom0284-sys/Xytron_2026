"""주행 스택 전역에서 쓰는 데이터 계약.

이 모듈은 rclpy / cv2 / numpy 에 의존하지 않는다.
퍼셉션(ROS 노드)이 채워 넣고 core 로직이 읽기만 하는 값들의 정의라서,
실차 없이 pytest 로 core 를 통째로 돌릴 수 있는 근거가 여기에 있다.

좌표계 규약 (전 모듈 공통):
  x : 차량 전방 (+)  [m]
  y : 차량 좌측 (+)  [m]
  조향: 음수 = 좌회전, 양수 = 우회전  [deg]
        (자이카 실차 규약. io/motor.py 의 Float32MultiArray[angle, speed] 와 일치)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple


# ──────────────────────────────────────────────────────────────────────
# 신호등
# ──────────────────────────────────────────────────────────────────────
class SignalState(str, Enum):
    """4구 신호등 판정 결과.

    규정(p.36) 기준 실제로 나오는 조합만 정의한다.
      출발      : RED  → GREEN
      2랩 시작  : GREEN → GREEN_LEFT (직좌 동시신호)  또는  GREEN 유지
      3랩 시작  : GREEN (직진만)
    """

    UNKNOWN = "UNKNOWN"      # 신호등을 못 찾음 (ROI 밖 / 가림 / 블러)
    RED = "RED"              # 빨강만 점등 → 정지 유지
    GREEN = "GREEN"          # 녹색만 점등 → 직진
    GREEN_LEFT = "GREEN_LEFT"  # 녹색 + 좌회전 화살표 → 좌회전(지름길) 가능

    @property
    def is_go(self) -> bool:
        """출발/통과해도 되는 신호인가."""
        return self in (SignalState.GREEN, SignalState.GREEN_LEFT)

    @property
    def allows_left(self) -> bool:
        """좌회전(지름길)이 허용되는 신호인가."""
        return self is SignalState.GREEN_LEFT


@dataclass(frozen=True)
class SignalReading:
    """신호등 검출기의 한 프레임 출력.

    lamp_brightness 는 좌→우 [빨강, 노랑, 좌회전, 녹색] 4개 램프의 평균 밝기.
    판정 실패 시 state=UNKNOWN 이고 reject_reason 에 사유가 담긴다
    (실차 튜닝 때 '어느 단계에서 막혔는지' 좁히기 위한 진단 필드).
    """

    state: SignalState = SignalState.UNKNOWN
    lamp_brightness: Tuple[float, ...] = ()
    circle_count: int = 0
    reject_reason: str = "no_frame"
    # 랩 마커: '검증된 채도 초록 램프 블롭'의 가로폭(px). 원 검출과 독립이며
    # 접근 거리 대용 지표로 쓴다(멀수록 작다, 실측 7→14→35px). 블롭이 없는
    # 장면(빨간불 대기 포함)은 0 — 판정 체인과 실측 근거는
    # perception/signal_detector.py 모듈 주석 참조.
    gantry_span_px: int = 0
    # 출발 전용 초록 블롭 폭(px) — 출발선 원거리(8~10m)에서는 램프 프린지가
    # 사라져 gantry_span 경로(링 검증)가 0 이 되므로, 심의 '균형 초록 끼'
    # (G−R, G−B 동시 양수)로만 잡는 완화 경로. **랩 카운팅에는 절대 쓰지
    # 않는다** (오검 1건 = 유령 랩 = 조기 FINISH). WAIT_GREEN 의 상승엣지·
    # RED 거부·정지 차량 조건이 오검을 흡수한다. 0817 실차 출발선 실측.
    start_blob_px: int = 0
    # ── 위치 기반 워처 결과 (perception/signal_watcher.py) ──
    # 색을 전혀 쓰지 않는 경로. watcher_go 는 '켜져 있던 불빛이 꺼지고 그
    # 오른쪽에 새 불빛이 켜졌다' = 빨강→초록 전환을 관측했다는 뜻이다.
    watcher_go: bool = False
    watcher_green_left: bool = False
    lamp_spacing_px: float = 0.0

    @property
    def valid(self) -> bool:
        return self.state is not SignalState.UNKNOWN


# ──────────────────────────────────────────────────────────────────────
# 차선
# ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class LaneReading:
    """BEV 차선 검출 결과.

    offset_px      : 차량 중심 대비 주행 목표선의 횡오차 (오른쪽이 +)
    lookahead_px   : 전방 주시점의 횡오차. 곡률 선행 반응용.
    heading_slope  : 차 바로 앞에서 목표선이 기울어진 정도 [px/px, 무차원].
                     +면 전방에서 목표선이 오른쪽으로 간다 = 차가 왼쪽을 향함.
                     '내가 차선 방향과 얼마나 어긋났나' — Stanley 의 heading 항.
    curvature_px   : 목표선의 곡률 [1/px]. +면 전방이 오른쪽으로 휜다.
                     코너 피드포워드(δ_ff = atan(축거·κ))의 입력.
    left_found / right_found : 각 차선을 실제로 관측했는지 (추정 보간이 아니라)
    lane_side      : 지금 어느 차선에 있는가. -1 왼쪽 / 0 가운데 / +1 오른쪽
                     규정상 1·2차선 구분은 없지만(p.27) 추월 방향 3순위 판단에 쓴다.

    ★ heading_slope·curvature_px 는 0817 첫 자율주행의 코너 실패에서 나왔다.
      횡오차만 보는 P 제어는 '차가 바깥으로 밀려나야' 조향이 생기므로 정상
      선회 중에는 구조적으로 언더스티어한다 (실측: 1랩 129샘플 최대 조향 19,
      물리 최대 42의 절반). 곡률 피드포워드가 있어야 코너에서 필요한 조향이
      오차 없이 먼저 나온다.
    """

    offset_px: float = 0.0
    lookahead_px: float = 0.0
    heading_slope: float = 0.0
    curvature_px: float = 0.0
    left_found: bool = False
    right_found: bool = False
    # 노란 중앙선 폴백으로 목표선을 세웠는가 (0818: 곡선에서 흰 선이 시야
    # 밖으로 나가는 프레임의 생존 수단 — lane_detector._fit_center)
    center_found: bool = False
    lane_width_px: float = 0.0
    lane_side: int = 0

    @property
    def valid(self) -> bool:
        return self.left_found or self.right_found or self.center_found

    @property
    def both_found(self) -> bool:
        return self.left_found and self.right_found


# ──────────────────────────────────────────────────────────────────────
# 라이다 / 장애물
# ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class TrackedObject:
    """라이다 클러스터 하나. 좌표는 차량 기준 (x 전방+, y 좌측+) [m]."""

    x: float
    y: float
    width: float = 0.0
    track_id: int = -1
    # 트래커가 추정한 상대속도 [m/s]. 정지 장애물과 방해차량 구분에 쓴다.
    vx: float = 0.0
    vy: float = 0.0
    age: int = 0

    @property
    def distance(self) -> float:
        return (self.x * self.x + self.y * self.y) ** 0.5

    # 주의: '움직이는가' 판정은 여기에 두지 않는다. 정지 물체도 주행 중에는
    # vx ≈ -(자차 속도) 로 관측되므로 vx 만으로는 판별이 불가능하고, 자차
    # 속도를 아는 쪽이 판정해야 한다 → core/race_fsm.py 의 _target_is_moving.
    # (과거 이 자리에 있던 is_moving 프로퍼티는 사실상 항상 True 를 반환하는
    #  오답 API 라 제거했다 — 리뷰 R4.)


@dataclass(frozen=True)
class LidarReading:
    """한 스캔에서 뽑아낸 주행 판단용 요약값.

    front_dist  : 정면 섹터 최소거리 [m] (없으면 inf)
    left_clear / right_clear : 그쪽으로 차선변경할 공간이 있는가
    objects     : 트래킹된 클러스터 목록 (가까운 순)
    """

    front_dist: float = float("inf")
    left_dist: float = float("inf")
    right_dist: float = float("inf")
    left_clear: bool = True
    right_clear: bool = True
    objects: Tuple[TrackedObject, ...] = ()
    stamp: float = 0.0

    @property
    def valid(self) -> bool:
        return self.stamp > 0.0

    def nearest_front(self, max_lateral: float = 0.45) -> Optional[TrackedObject]:
        """전방(내 진행 경로 위)에서 가장 가까운 물체.

        max_lateral 은 '내 경로를 막고 있다'고 볼 횡방향 반폭[m].
        차폭(약 0.30m)에 여유를 더한 값이 기본이다.
        """
        best: Optional[TrackedObject] = None
        for o in self.objects:
            # x 하한 0.15m: 라이다 바로 옆(x≈0)의 물체는 '전방'이 아니다 —
            # 0817 실사고: 차체 자기 반사(x 0.04, y −0.10)가 전방 타겟이 됐다.
            if o.x <= 0.15 or abs(o.y) > max_lateral:
                continue
            if best is None or o.x < best.x:
                best = o
        return best


# ──────────────────────────────────────────────────────────────────────
# 라바콘
# ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class ConeReading:
    """라바콘 구간 인지 결과.

    규정(p.23~25): 라바콘이 좌우 두 줄로 놓이고 그 사이를 통과한다.
    그래서 '주황 덩어리 하나를 따라간다'가 아니라 '좌우 게이트의 중앙을 지난다'
    로 풀어야 한다. gate_center_px 가 그 중앙이다.
    """

    detected: bool = False
    orange_px: int = 0
    left_cone_px: int = 0
    right_cone_px: int = 0
    gate_center_px: float = 0.0     # ROI 중심 대비 게이트 중앙의 횡오차
    gate_valid: bool = False        # 좌우 양쪽 콘을 모두 봤는가
    nearest_cone_m: float = float("inf")
    # 게이트 쌍의 깊이 [BEV px, ROI 하단=0]. ★ 조준점을 '관측된 콘 너머'로
    # 두면 코너에서 통로를 가로질러 버린다 — 경쟁팀 ddm 이 실주행 bag 으로
    # 진단한 실패 모드다. policy 가 이 값으로 전방주시거리를 잘라 쓴다.
    gate_depth_px: float = 0.0
    gate_width_px: float = 0.0      # 관측된 게이트 폭 (적응 코리도 학습용)
    # ── 라이다 게이트 (0818 신설, perception/lidar_cones.py) ──
    # 카메라 게이트는 조명·반사에 휘둘려 성립률 22~58% 였지만 라이다는 같은
    # 구간에서 80% 였고, 무엇보다 **미터로 직접** 준다 — px_per_m 추정(미실측
    # 420) 오차가 통째로 사라진다. policy 는 이 쪽을 1순위로 쓴다.
    lidar_gate_valid: bool = False
    lidar_gate_depth_m: float = 0.0
    lidar_gate_width_m: float = 0.0
    lidar_gate_lateral_m: float = 0.0   # 게이트 중앙 횡위치 (좌측 +)
    lidar_cones: int = 0


# ──────────────────────────────────────────────────────────────────────
# 제어 출력
# ──────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class DriveCommand:
    """모터로 나가는 최종 명령.

    steer : deg, 음수 좌 / 양수 우
    speed : 자이카 속도 단위 (실차 검증 범위 0~50, 실사용 8~25)
    """

    steer: float = 0.0
    speed: float = 0.0
    source: str = ""    # 어느 모듈이 만든 명령인지 (로그/디버그용)

    def clamped(self, max_steer: float, max_speed: float) -> "DriveCommand":
        # NaN 은 min/max 를 통과해 최대값으로 둔갑하므로(비교가 False) 먼저 거른다.
        s = self.steer if math.isfinite(self.steer) else 0.0
        v = self.speed if math.isfinite(self.speed) else 0.0
        s = max(-max_steer, min(max_steer, s))
        v = max(0.0, min(max_speed, v))
        return DriveCommand(s, v, self.source)


# ──────────────────────────────────────────────────────────────────────
# 통합 관측 (FSM 입력)
# ──────────────────────────────────────────────────────────────────────
@dataclass
class Observation:
    """한 제어 주기에 FSM 이 보는 세상 전부.

    FSM 은 이 객체 하나만 읽는다. ROS 메시지도, cv2 도 여기까지 올라오지 않는다.
    덕분에 test/ 에서 이 객체를 손으로 만들어 FSM 전이를 전부 검증할 수 있다.
    """

    now: float = 0.0
    signal: SignalReading = field(default_factory=SignalReading)
    lane: LaneReading = field(default_factory=LaneReading)
    lidar: LidarReading = field(default_factory=LidarReading)
    cone: ConeReading = field(default_factory=ConeReading)
    # IMU 누적 요(yaw) [deg]. 랩 카운팅 교차검증과 지름길 좌회전 완료 판정에 쓴다.
    yaw_deg: float = 0.0
    # 자차 속도 추정 [m/s]. 정지/주행 장애물 구분에 필요.
    ego_speed_mps: float = 0.0
    # 퍼셉션 입력이 살아있는가 (토픽 타임아웃 감시 결과)
    camera_ok: bool = False
    lidar_ok: bool = False


__all__ = [
    "SignalState",
    "SignalReading",
    "LaneReading",
    "TrackedObject",
    "LidarReading",
    "ConeReading",
    "DriveCommand",
    "Observation",
]

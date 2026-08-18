"""조향 계산 — 퓨어 퍼수트와 픽셀-오차 PID.

자이카 축거(wheelbase)는 0.30m 이다 (자이카 ROS2 미니PC Y모델).
퓨어 퍼수트를 쓰는 이유: 픽셀 오차 PID 는 저속에선 잘 듣지만 속도가 올라가면
같은 게인이 과조향을 만든다. 4분 제한(규정 p.37) 안에 3바퀴를 넣으려면 고속
구간이 필수라 속도에 따라 전방주시거리가 늘어나는 기하 기반 제어가 유리하다.

조향 부호: 음수 = 좌회전, 양수 = 우회전 (자이카 실차 규약).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

WHEELBASE_M = 0.30
# motor_adapter(팀원 kobot_vehicle) 실측 체인: δ[rad] = cmd × 0.0068,
# 이후 vesc.yaml servo 0.15~0.85 가 δ 를 ±0.288rad(16.5°)로 자른다.
CMD_TO_RAD = 0.0068


def pure_pursuit_steer(
    target_x: float,
    target_y: float,
    *,
    wheelbase: float = WHEELBASE_M,
    max_steer: float = 50.0,
) -> float:
    """차량 기준 목표점(x 전방+, y 좌측+)을 향하는 조향각 [deg].

    δ = atan2(2·L·sin(α), Ld),  α = 목표점 방위각, Ld = 목표점까지 거리.
    y>0(좌측)이면 좌회전이어야 하므로 부호를 뒤집어 반환한다.
    """
    ld_sq = target_x * target_x + target_y * target_y
    if ld_sq < 1e-6:
        return 0.0
    ld = math.sqrt(ld_sq)
    alpha = math.atan2(target_y, target_x)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    steer = -math.degrees(delta)
    return max(-max_steer, min(max_steer, steer))


@dataclass
class PixelPID:
    """BEV 픽셀 오차용 PID. 차선주행 기본 제어.

    적분항은 기본으로 꺼 둔다(Ki=0). 켤 거면 반드시 anti-windup 을 함께 켤 것 —
    픽셀 오차는 차선을 놓친 프레임에 크게 튀어서 적분이 순식간에 폭주한다.
    """

    # ★ kp 0.16 → 0.34 (0817 첫 자율주행 실측): 1랩 로그 129샘플 중 |조향|≥25
    #   가 2건, 최대 19 — 물리 최대(cmd≈42, 바퀴 16.5°, 회전반경 1.08m)의
    #   절반도 못 써 코너(반경 ~1m)에서 언더스티어 → 차선 이탈. 코너에서는
    #   포화까지 가야 하므로 실측 오프셋(~120px)에서 cmd 40 이 나오는 값.
    #   직선 소오프셋(10~30px)엔 3~10° 라 진동 위험 낮음 (스무더 5창+420°/s).
    kp: float = 0.34
    ki: float = 0.0
    kd: float = 0.27
    deadzone_px: float = 10.0
    max_steer: float = 50.0
    integral_limit: float = 400.0

    _prev: float = 0.0
    _integral: float = 0.0

    def reset(self) -> None:
        self._prev = 0.0
        self._integral = 0.0

    def update(self, error_px: float) -> float:
        if abs(error_px) < self.deadzone_px:
            error_px = 0.0

        self._integral += error_px
        # anti-windup: 적분항을 물리적으로 의미 있는 범위에 가둔다.
        self._integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral)
        )
        derivative = error_px - self._prev
        self._prev = error_px

        steer = self.kp * error_px + self.ki * self._integral + self.kd * derivative
        return max(-self.max_steer, min(self.max_steer, steer))


@dataclass
class LaneController:
    """차선주행 조향 = 곡률 피드포워드 + 방향 정렬 + 횡오차 되먹임.

    왜 3항인가 (0817 첫 자율주행 실패 분석):
      횡오차만 보는 P 제어는 정상 선회 중 구조적으로 언더스티어한다. 코너에서
      필요한 조향각은 오차가 아니라 **곡률**이 정하는데(δ = atan(L·κ)), P 항은
      차가 바깥으로 밀려나 오차가 쌓여야 비로소 그만큼을 만들어 낸다. 그
      '밀려나는 양'이 곧 차선 이탈이었다 (실측 최대 조향 19 / 물리 최대 42).

      · 피드포워드(κ): 코너에 필요한 조향을 오차 없이 미리 만든다.
      · 방향 정렬(slope): 차가 차선과 어긋난 각을 편다. Stanley 의 heading 항.
      · 횡오차(P/D): 남은 중심 이탈을 천천히 지운다.

    단위: 입력은 전부 BEV 픽셀 기준(무차원 slope, 1/px 곡률)이고 출력은
    자이카 조향 명령값(cmd, 42 부근에서 서보 포화)이다.
    """

    # ── 곡률 피드포워드 게인은 '고정 상수'가 아니라 매 프레임 유도된다 ──
    # 필요 조향: cmd = δ_rad/0.0068,  δ_rad ≈ 축거[m] × κ_real[1/m]
    # BEV 픽셀 곡률 → 실제 곡률: κ_real = κ_px × s_fwd² / s_lat
    #   s_lat [px/m] 은 **매 프레임 측정된 차선폭**으로 구한다 (자가 보정!):
    #       s_lat = lane_width_px / road_width_m
    #   s_fwd 는 직접 못 재므로 종횡비 bev_aspect = s_fwd/s_lat 로 둔다.
    #   ⇒ k = (축거/0.0068) × s_lat × bev_aspect² × ff_scale
    # 이렇게 하면 px_per_m 실측이 없어도 축척 변화에 자동 추종한다 —
    # 0817 시뮬 스윕에서 축척 420~620 px/m 전 구간 통과의 근거.
    road_width_m: float = 0.80       # 규정 도면 주행로 폭 (실측 시 갱신)
    bev_aspect: float = 0.90         # s_fwd/s_lat — 호모그래피 종횡비
    # 0817 시뮬 실측: 폴리핏 곡률이 기하 예측보다 ~1.6배 크게 나온다(BEV
    # 종횡비 미실측 + 엣지 기반 이진화의 영향). 과대 피드포워드는 코너 진입
    # 과조향이 되므로 보수적으로 잡고 부족분은 방향·횡오차 항에 맡긴다.
    ff_scale: float = 0.55           # 안정 여유
    # 기울기 → 조향 게인. slope 0.2(≈11° 어긋남) 에서 cmd 10 정도가 나온다.
    # 0818 실차 80→50: 실제 카메라의 폴리핏 기울기는 시뮬보다 잡음이 커서
    # 80 이면 횡오차 항을 이기고 반대로 꺾는 구간이 생겼다(t=62~70s 실측).
    k_heading: float = 50.0
    # ── 검출 품질 게이트 (0818 첫 실차 실측으로 추가) ──
    # 곡률·기울기 항은 '깨끗한 2차 폴리핏'을 전제한다. 실차에선 한쪽 차선만
    # 보이거나 콘 하이라이트가 섞이면 폭 추정이 250~350px 로 무너지고, 그때의
    # 곡률·기울기는 잡음이라 믿으면 반대로 꺾는다 (꼬불길·라바콘 구간 실패의
    # 원인). 양쪽 차선이 보이고 폭이 정상 대역일 때만 두 항을 켠다 —
    # 아니면 어제 잘 되던 순수 횡오차 제어로 자동 강등한다.
    trust_width_min_px: float = 360.0
    trust_width_max_px: float = 540.0
    # 신뢰도는 급변하지 않게 EMA (0.3 ≈ 3~4프레임)
    trust_alpha: float = 0.3
    # 곡률·기울기는 짧은 BEV 창의 2차 폴리핏이라 잡음이 크다 — EMA 로 눌러
    # 쓴다. 0.35 는 30fps 에서 시정수 ≈0.1s (코너 진입 지연 무시 가능).
    ema_alpha: float = 0.35
    # 피드포워드 항의 절대 상한 [cmd]. 곡률 오검(차선 놓침 직후 폴리핏 튐)이
    # 한 프레임에 풀락을 때리는 것을 막는다.
    ff_limit: float = 45.0
    max_steer: float = 50.0

    # ── 아펙스 커팅 (0817 시뮬레이터로 확정한 필수 기능) ──────────────
    # 이 차의 최대 바퀴각은 16.5°(vesc.yaml servo 0.15~0.85) 이고 축거가
    # 0.30m 이므로 **최소 회전반경이 1.01m** 다. 즉 반경 1.0m 이하 코너에서는
    # 차선 중심선을 따라가는 것이 물리적으로 불가능하다 (필요 16.7° > 16.5°).
    # 유일한 해법은 주행로 폭을 써서 실효 반경을 키우는 것 — 코너에서 목표를
    # 안쪽으로 밀어 레이싱 라인을 그린다. 시뮬 실측: R=1.0m 코너 최대 이탈
    # 0.56m(이탈) → 0.11m(주행로 안), R=0.8m 1.15m → 0.24m.
    #
    # ★ 발동 조건은 '곡률 추정치'가 아니라 **실제 조향 포화**로 판단한다.
    #   BEV 축척(px/m)이 미실측이라 곡률의 절대값은 최대 1.6배까지 틀릴 수
    #   있는데(0817 시뮬 실측), 그걸 기준으로 삼으면 멀쩡한 코너에서 아펙스가
    #   켜져 오히려 이탈한다. 반면 '조향을 끝까지 썼는데도 부족하다'는 사실은
    #   축척과 무관하게 참이다 — 그때만, 그리고 그것이 지속될 때만 판다.
    apex_gain: float = 0.55          # 반차선폭의 몇 배까지 안쪽으로 붙일까
    #   (0.55 × 0.4m ≈ 0.22m — 주행로 여유 0.4m 의 절반만 쓴다)
    apex_engage: float = 0.80        # 포화율(느린 EMA)이 이 이상일 때 발동
    cmd_saturation: float = 42.0     # 서보 포화 cmd (16.5° / 0.0068)
    # 차선폭을 못 읽었을 때 쓸 기본 반차선폭 [px]
    default_half_lane_px: float = 225.0
    # 아펙스 목표가 한 번에 튀지 않도록 하는 변화율 제한 [px/프레임].
    # S자 변곡점에서 목표가 좌↔우로 계단 이동하면 그대로 진동이 된다.
    bias_rate_px: float = 12.0
    # ★ 아펙스 판단은 **느린** EMA 로 한다 (시정수 ≈0.4s). 지속적인 한쪽
    # 코너에서만 켜지고, 좌우가 번갈아 오는 S자·슬라럼에서는 평균이 0 으로
    # 상쇄돼 저절로 꺼진다 — 슬라럼은 안쪽에 붙는 게 아니라 가운데로 지나야
    # 하므로 이게 물리적으로도 옳다 (0817 시뮬: 연속 S자 이탈 0.90m → 통과).
    apex_ema_alpha: float = 0.08

    pid: PixelPID = field(default_factory=PixelPID)

    _curv_ema: float = 0.0
    _sat_slow: float = 0.0
    _slope_ema: float = 0.0
    _bias_px: float = 0.0
    _trust: float = 0.0
    _primed: bool = False

    def reset(self) -> None:
        self.pid.reset()
        self._curv_ema = 0.0
        self._sat_slow = 0.0
        self._slope_ema = 0.0
        self._bias_px = 0.0
        self._trust = 0.0
        self._primed = False

    def k_curvature(self, lane_width_px: float) -> float:
        """측정된 차선폭으로 곡률→조향 게인을 유도한다 (클래스 주석 참조)."""
        half = (
            lane_width_px / 2.0 if lane_width_px > 1.0 else self.default_half_lane_px
        )
        s_lat = (2.0 * half) / max(0.1, self.road_width_m)
        return (
            (WHEELBASE_M / CMD_TO_RAD)
            * s_lat
            * (self.bev_aspect ** 2)
            * self.ff_scale
        )

    def update(
        self,
        offset_px: float,
        heading_slope: float,
        curvature_px: float,
        lane_width_px: float = 0.0,
        both_found: bool = True,
    ) -> float:
        if not self._primed:
            self._curv_ema = curvature_px
            self._slope_ema = heading_slope
            self._primed = True
        else:
            a = self.ema_alpha
            self._curv_ema = a * curvature_px + (1.0 - a) * self._curv_ema
            self._slope_ema = a * heading_slope + (1.0 - a) * self._slope_ema


        # 검출 품질 → 곡률·기울기 항 신뢰도 (0 이면 순수 횡오차 제어)
        good = (
            both_found
            and self.trust_width_min_px <= lane_width_px <= self.trust_width_max_px
        )
        a_t = self.trust_alpha
        self._trust = a_t * (1.0 if good else 0.0) + (1.0 - a_t) * self._trust

        k = self.k_curvature(lane_width_px)
        ff = max(-self.ff_limit, min(self.ff_limit, k * self._curv_ema)) * self._trust
        head = self.k_heading * self._slope_ema * self._trust

        # 아펙스 커팅: 중심선으로는 못 도는 코너에서 목표를 안쪽으로 옮긴다.
        # tight ∈ [-1,1] — +면 우코너(안쪽=오른쪽).
        # 지속 포화율 — '조향을 끝까지 썼는데도 부족한 상태'가 얼마나 이어졌나.
        tight = self._sat_slow
        over = abs(tight) - self.apex_engage
        half_lane = (
            lane_width_px / 2.0 if lane_width_px > 1.0 else self.default_half_lane_px
        )
        if over > 0.0:
            span = max(1e-6, 1.0 - self.apex_engage)
            target_bias = (
                self.apex_gain
                * min(1.0, over / span)
                * (1.0 if tight > 0 else -1.0)
                * half_lane
            )
        else:
            target_bias = 0.0
        # 변화율 제한 — S자 변곡점에서 목표가 계단 이동하면 진동이 된다.
        step = max(-self.bias_rate_px, min(self.bias_rate_px, target_bias - self._bias_px))
        self._bias_px += step

        # 목표선이 차 오른쪽에 보이면(=차가 왼쪽에 있으면) offset 이 +다.
        # 우코너에서 차를 오른쪽(안쪽)으로 붙이려면 목표 offset 이 -bias 이므로
        # 오차에 +bias 를 더한다.
        fb = self.pid.update(offset_px + self._bias_px)

        steer = ff + head + fb
        # 물리 포화(cmd 42)를 넘겨 봐야 바퀴는 안 꺾이고 하류 로직만 오해한다.
        lim = min(self.max_steer, self.cmd_saturation)
        steer = max(-lim, min(lim, steer))
        # 다음 프레임의 아펙스 판단용 — 부호 있는 포화율의 느린 EMA
        b = self.apex_ema_alpha
        self._sat_slow = b * (steer / lim) + (1.0 - b) * self._sat_slow
        return steer


@dataclass
class SteerSmoother:
    """이동평균 + 슬루 제한. 서보 떨림과 급조향을 억제한다.

    ★ window 5→3 (0817 시뮬 실측): 이동평균은 지연을 (window-1)/2 프레임
    만큼 순수하게 더한다 — 30fps 에서 5창은 67ms 로, 체인 지연(센싱 50 +
    구동 40 + 서보 τ60ms)에 얹히면 빠른 S자에서 조향이 한 박자 늦어 이탈
    했다(최악 0.93m). 3창(33ms)으로 줄이면 같은 조건 0.27m.
    곡률·기울기는 이미 LaneController 에서 EMA 로 걸러지므로 출력단 평활은
    서보 떨림 억제 정도면 충분하다.
    """

    window: int = 3
    max_rate_deg_per_s: float = 420.0

    def __post_init__(self) -> None:
        self._hist: list[float] = []
        self._last: float = 0.0
        self._last_t: float | None = None

    def reset(self) -> None:
        self._hist.clear()
        self._last = 0.0
        self._last_t = None

    def update(self, steer: float, now: float) -> float:
        self._hist.append(steer)
        if len(self._hist) > max(1, self.window):
            self._hist.pop(0)
        smoothed = sum(self._hist) / len(self._hist)

        if self._last_t is None:
            self._last_t = now
            self._last = smoothed
            return smoothed

        dt = max(1e-3, now - self._last_t)
        max_delta = self.max_rate_deg_per_s * dt
        delta = smoothed - self._last
        if delta > max_delta:
            smoothed = self._last + max_delta
        elif delta < -max_delta:
            smoothed = self._last - max_delta

        self._last = smoothed
        self._last_t = now
        return smoothed


__all__ = [
    "WHEELBASE_M",
    "pure_pursuit_steer",
    "PixelPID",
    "LaneController",
    "SteerSmoother",
]

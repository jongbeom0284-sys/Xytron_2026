"""속도 정책 — 4분 예산(규정 p.37) 안에 3바퀴를 넣기 위한 계층.

세 가지를 겹쳐서 최종 목표속도를 만든다.
  1) 모드별 기본속도      : 라바콘은 느리게, 직선은 빠르게
  2) 곡률·조향 기반 감속  : 코너에서 알아서 줄인다
  3) 시간 압박 보정       : 뒤처지고 있으면 전 구간을 올린다

마지막에 램프(가감속 제한)를 걸어 급가속으로 인한 휠스핀과 급감속으로 인한
피치 변화(카메라 시야 흔들림)를 막는다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Optional


@dataclass
class SpeedConfig:
    # 모드별 기본 목표속도 [자이카 단위]
    base: Dict[str, float] = field(
        default_factory=lambda: {
            "WAIT_GREEN": 0.0,
            # ★ 20 → 24 (0820 밤). 본선 설정 실주행(race_0820_2119)에서
            #   라바콘 구간 간격으로 잰 한 바퀴가 약 92초였다 — 3바퀴면
            #   276초로 규정 240초를 넘는다. 즉 속도는 선택이 아니라 완주
            #   조건이다.
            #   올려도 되는 근거는 지연 포함 시뮬(tools/lanesim, 실측 축척
            #   810/730)이다. 최대 이탈이 **속도에 거의 무관**하다:
            #       시나리오          1.0m/s   1.4m/s   1.8m/s
            #       직선              0.02m    0.02m    0.02m
            #       급코너 R=1.0m     0.20m    0.15m    0.25m
            #       최악코너 R=0.8m   0.43m    0.42m    0.41m  ← 전부 이탈
            #                          (최소회전반경 1.01m > R — 속도 무관 한계)
            #       연속 S자          0.24m    0.34m    0.39m  ← 여기만 민감
            #   반차선이 0.41m 이므로 연속 S자가 유일한 제약이고, 1.4m/s 는
            #   0.34m(83%)로 여유가 있다. 곡선·굽이침 감속 비율은 **그대로**
            #   둔다 — 그게 코너 속도를 잡아 주는 장치라, 직선에서만 빨라진다.
            #   실측 조향 트레이스로 재생: 중앙 14.5 → 17.2 (1.16 → 1.38 m/s).
            #   max_speed 26 상한 덕에 시간압박 부스트가 걸려도 2.08m/s 라
            #   회피 판정창(1.05m)을 0.505초에 통과 > target_enter_seconds 0.40.
            #   ★ 24 → 28 (0820 밤 2차). 완주만이 아니라 **기록**도 필요하다는
            #     요구에 맞춰 다시 올렸다. 근거는 폐루프 시뮬을 LANE_DRIVE 가
            #     실제로 도는 구간만으로 다시 돌린 것이다 — 앞선 판단은
            #     '연속 S자'가 제약이라고 봤는데, 그 구간은 실차에서 CONE_DRIVE
            #     (0.64m/s 고정)로 돌므로 LANE_DRIVE 속도와 무관했다.
            #     LANE_DRIVE 구간(직선·완만R1.5·급R1.0·S자R1.2)만 보면 순항
            #     2.4m/s 에서도 최대이탈이 0.10~0.18m 로 반차선 0.41m 대비
            #     여유가 크다. 남은 '최악 코너 R=0.8m'는 차 최소회전반경
            #     1.01m 보다 작아 **속도와 무관하게** 실패한다(1.6m/s 이탈,
            #     2.6m/s 통과 — 위상 노이즈이지 추세가 아니다).
            #     실측 조향 트레이스 재생: 평균 1.39 → 1.62 m/s, 한 바퀴
            #     68 → 61초, 3바퀴 205 → 183초.
            "LANE_DRIVE": 28.0,
            # ★ 11 → 18 (0820 밤). 실주행에서 이 구간은 **min_speed 8.0 바닥에
            #   68% 붙어** 0.64m/s 로 기어갔다 — base 11 에 곡선감속이 걸리면
            #   11×0.55=6.1 이라 바닥에 눌리기 때문이다. 즉 11 이라는 값은
            #   거의 쓰이지도 않았다.
            #   실측 조향(|steer| 중앙 15.7도 → 곡선비 0.792)으로 환산하면
            #     base 11 → 8.7 (0.70m/s, 통과 15.1초)
            #     base 18 → 14.3 (1.14m/s, 통과 9.2초)   ← 랩당 7.2초 절약
            #   통로 실측 81.8cm, 차폭 30cm → 편측 여유 25.9cm.
            #   전방주시거리는 관측된 게이트 깊이(실측 50~64cm)로 잘리므로
            #   1.14m/s 에서 예견시간 0.44~0.56초 — 지연 합계(센싱0.05+구동
            #   0.04+서보0.06=0.15초)의 3배 이상은 남는다.
            #   ★ 트랙에서 바로 조절하라: race.launch.py 의 cone_speed 인자.
            "CONE_DRIVE": 18.0,
            "AVOID_STATIC": 13.0,
            "OVERTAKE": 15.0,
            "SHORTCUT": 12.0,
            "FINISH": 0.0,
            "STOP": 0.0,
        }
    )
    min_speed: float = 8.0
    # ★ 26 → 28 (0820 밤 2차). base 28 이 시간압박 부스트 없이도 상한에
    #   닿으므로 같이 올린다. **28 이 상한인 이유**: 회피 판정창(0.40~1.45m
    #   = 1.05m)을 최고속으로 통과하는 시간이 target_enter_seconds(0.40초)를
    #   넘어야 회피가 발화한다. 28×0.08 = 2.24m/s → 0.469초 (여유 +17%).
    #   30 이면 0.437초(+9%), 32 면 0.410초(+3%) 로 마진이 사라진다.
    #   더 올리려면 target_enter_seconds 를 먼저 내려야 하는데, 그 값은
    #   오검을 거르는 장치라 실차 검증 없이 건드리지 않는다.
    max_speed: float = 28.0

    # 조향이 클수록 감속. |steer| 가 steer_full_slow 일 때 curve_min_ratio 배가 된다.
    steer_full_slow_deg: float = 34.0
    curve_min_ratio: float = 0.55

    # ── 라바콘 구간 전용 곡선감속 (0820 밤 신설) ──────────────────────
    # 위 34/0.55 는 **차선주행용**이다. 차선에서 조향이 크다는 건 곡률을
    # 모르는 코너에 들어갔다는 뜻이라 줄이는 게 맞다 (폐루프 시뮬로 확인:
    # 조금만 풀면 슬라럼이 0.36→0.78m 로 무너진다).
    # 라바콘 통로는 사정이 다르다:
    #   · 기하가 알려져 있다 — 실측 폭 81.8cm 의 직선형 통로
    #   · 조향이 큰 것은 코너가 아니라 **콘 사이를 누비는 것**이라, 조향
    #     한 번당 실제 횡이탈은 작다
    #   · 라이다 게이트가 61% 프레임에서 미터로 직접 목표를 준다
    # 그런데 같은 감속을 쓰는 바람에, 통로를 누비는 조향이 그대로 감속을
    # 불러 0.64m/s 로 기어갔다.
    # 실측(race_0820_2343, 게이트 수정 후 |조향| 중앙 16.1 / p90 24.4):
    #     34/0.55 (차선용) → 1.16m/s, 10.5m 통과 9.1초
    #     45/0.75         → 1.32m/s,            7.9초
    #     60/0.85         → 1.39m/s,            7.6초
    # 상한은 **예견시간**이 정한다 = 전방주시(실측 61cm) ÷ 속도.
    # 지연 합계(센싱0.05+구동0.04+서보0.06=0.15초)의 2.7배인 0.40초를
    # 안전선으로 잡으면 1.5m/s 가 한계다. 60/0.85 는 0.44초로 그 안이다.
    # ★ 위 실측(1.16/1.32/1.39 m/s)은 전부 **조향 단위 버그 상태**에서 잰 것이다.
    #   그때 발행 |조향| 중앙이 16.1 이었던 건 제어가 부드러워서가 아니라
    #   퓨어 퍼수트의 도(deg) 값이 그대로 나가 실효 조향이 39% 였기 때문이다
    #   (control.pure_pursuit_cmd 주석). 단위를 바로잡으면 같은 주행의
    #   |조향| 중앙이 39.9 로 뛴다.
    #   full_slow=60 은 그대로 두어도 되지만(최대치가 42 라 ratio 0.90~1.0),
    #   **하한 0.85 는 굽은 통로에서 감속이 사실상 없는 것과 같다**.
    #   라이다 폐루프 재현(콘 통로 82cm, 게이트 끊김 실측 83.5% 재현,
    #   지연 0.05+0.04+τ0.06):
    #       현행(deg, 하한 0.85)   3/7 통과  1.18 m/s
    #       단위만 수정            4/7 통과  1.02 m/s
    #       단위 + 하한 0.70       6/7 통과  0.97 m/s   ← 채택
    #   R1.5/R1.3 이 0.85 에서는 콘을 긁고 0.70 에서는 통과한다.
    #   대가는 18% 감속(라바콘 10.5m 기준 8.9→10.8초). 3랩 예산 240초에
    #   현재 183초라 여유 57초 안에 든다.
    #   ※ 조향이 포화하면 반경은 더 못 줄인다. 남은 유일한 레버가 속도다 —
    #     지연 0.15초 동안 가는 거리가 1.18m/s 면 17.7cm, 0.97m/s 면 14.6cm 로
    #     줄고, 편측 여유가 25.9cm 뿐이라 그 3cm 가 통과/접촉을 가른다.
    cone_steer_full_slow_deg: float = 60.0
    cone_curve_min_ratio: float = 0.70

    # ── 굽은 길(조향이 계속 바뀌는 구간) 추가 감속 ──────────────────
    # 0817 지연 포함 시뮬 실측: 연속 S자를 1.6m/s 로 들어가면 위상 지연
    # (센싱 0.05 + 구동 0.04 + 서보 0.06s) 탓에 조향이 항상 한 박자 늦어
    # 0.69m 이탈한다. |steer| 크기만 보는 감속은 여기서 안 듣는다 —
    # S자에서는 부호가 계속 바뀌어 순간 |steer| 가 작은 구간이 섞이기 때문.
    # 그래서 **조향 변화율**(굽이침의 직접 지표)로 따로 감속한다.
    # 이 값(단위/초)에서 wind_min_ratio 배까지 줄어든다.
    # 0817 스윕 실측: 260/0.55 는 최악 축척(620px/m)·최고속(1.6m/s) S자에서
    # 0.93m 이탈. 200/0.45 로 조이면 같은 조건 0.33m 로 통과한다 (실주행
    # 속도는 0.92m/s — 4분 예산에 지장 없는 수준).
    steer_rate_full_slow: float = 200.0
    wind_min_ratio: float = 0.45
    steer_rate_alpha: float = 0.25

    # 시간 압박이 1.0 일 때 기본속도에 곱하는 값.
    # ★ 1.30 → 1.0 (꺼짐, 0820 밤 2차). base 28 = max_speed 28 로 올리면서
    #   이 부스트는 **직선에서 상한에 잘려 아무 효과가 없고, 코너에서만
    #   살아남는다** — 곱해지는 자리가 곡선·굽이침 감속 **뒤**라
    #   (target = base × ratio × wind × boost) 남는 효과가 '코너 감속을
    #   되돌리는 것'뿐이다. 그건 정확히 원하지 않는 방향이다.
    #   덤으로 0820 감사가 짚은 위험도 함께 사라진다: 랩 카운터가 얼면
    #   laps_left 가 과대추정돼 후반 내내 압박 1.0 이 걸리던 경로.
    #   base 28 로 3바퀴 183초(240초 제한, 여유 57초)라 따라잡기 가속이
    #   필요한 상황 자체가 드물다. 되살리려면 max_speed 를 base 보다
    #   높여 직선에 여유를 준 뒤, 회피 판정창 마진을 다시 계산할 것.
    pressure_boost: float = 1.0

    # 램프 [단위/초]
    ramp_up_per_s: float = 22.0
    ramp_down_per_s: float = 40.0

    # 퍼셉션 열화 시(safety DEGRADED) 상한
    degraded_speed: float = 9.0


class SpeedPolicy:
    def __init__(self, config: Optional[SpeedConfig] = None) -> None:
        self.config = config or SpeedConfig()
        self._current = 0.0
        self._last_t: Optional[float] = None
        self._prev_steer: Optional[float] = None
        self._rate_ema = 0.0

    def reset(self) -> None:
        self._current = 0.0
        self._last_t = None
        self._prev_steer = None
        self._rate_ema = 0.0

    def note_steer(self, steer_deg: float, now: float) -> float:
        """조향 변화율(단위/초)의 EMA 를 갱신하고 돌려준다."""
        cfg = self.config
        if self._prev_steer is None or self._last_t is None:
            self._prev_steer = steer_deg
            return self._rate_ema
        dt = max(1e-3, now - self._last_t)
        rate = abs(steer_deg - self._prev_steer) / dt
        self._prev_steer = steer_deg
        a = cfg.steer_rate_alpha
        self._rate_ema = a * rate + (1.0 - a) * self._rate_ema
        return self._rate_ema

    def target_for(
        self,
        mode: str,
        *,
        steer_deg: float,
        time_pressure: float,
        override: Optional[float] = None,
        degraded: bool = False,
    ) -> float:
        """램프 적용 전의 목표속도."""
        cfg = self.config
        base = cfg.base.get(mode, cfg.base["LANE_DRIVE"])
        if override is not None:
            base = override
        if base <= 0.0:
            return 0.0

        # 곡선 감속 — 라바콘 구간은 전용 상수를 쓴다 (위 주석의 실측 근거)
        if mode == "CONE_DRIVE":
            full_slow = cfg.cone_steer_full_slow_deg
            min_ratio = cfg.cone_curve_min_ratio
        else:
            full_slow = cfg.steer_full_slow_deg
            min_ratio = cfg.curve_min_ratio
        ratio = 1.0 - (1.0 - min_ratio) * min(
            1.0, abs(steer_deg) / max(1e-3, full_slow)
        )
        # 굽이침 감속 — 조향 변화율이 크면(=S자) 추가로 줄인다
        wind = 1.0 - (1.0 - cfg.wind_min_ratio) * min(
            1.0, self._rate_ema / max(1e-3, cfg.steer_rate_full_slow)
        )
        target = base * ratio * wind

        # 시간 압박 보정. 미션 구간에서는 올리지 않는다 — 거기서 서두르면
        # 라바콘 충돌(3초)·추돌(10초)로 오히려 손해다.
        if mode in ("LANE_DRIVE", "SHORTCUT"):
            target *= 1.0 + (cfg.pressure_boost - 1.0) * max(
                0.0, min(1.0, time_pressure)
            )

        if degraded:
            target = min(target, cfg.degraded_speed)

        # min_speed 바닥은 일반 주행에만 적용한다. override 는 기동 컨트롤러의
        # 명시적 의도(예: 추돌 방지 서행 3.3)라 바닥으로 끌어올리면 0.55m 앞
        # 정지 차량에 2배 속도로 접근하는 사고가 된다.
        floor = 0.0 if override is not None else cfg.min_speed
        return max(floor, min(cfg.max_speed, target))

    def step(self, target: float, now: float) -> float:
        """램프를 적용해 실제 발행할 속도를 만든다."""
        cfg = self.config
        if self._last_t is None:
            self._last_t = now
            # 출발 순간부터 램프를 태운다 (정지 → 목표 급발진 방지)
            self._current = 0.0 if target > 0.0 else target
            return self._current

        dt = max(1e-3, now - self._last_t)
        self._last_t = now

        if target > self._current:
            self._current = min(self._current + cfg.ramp_up_per_s * dt, target)
        else:
            self._current = max(self._current - cfg.ramp_down_per_s * dt, target)
        return self._current

    def prime(self, value: float, now: float) -> None:
        """램프를 0 이 아니라 value 에서 시작시킨다.

        reset() 직후 첫 step() 은 '정지 → 목표 급발진 방지' 때문에 현재값을
        0 으로 강제한다. 모드 전환처럼 **이미 달리고 있는** 상황에서는 그것이
        곤두박질이 된다 (0823 실측: 라바콘 진입마다 2.16 -> 0.29 m/s).
        그 경우 이 함수로 시작값을 채운다. 첫 호출 분기를 건너뛰도록
        _last_t 도 함께 채운다.
        """
        self._current = max(0.0, float(value))
        self._last_t = now

    @property
    def current(self) -> float:
        return self._current


__all__ = ["SpeedConfig", "SpeedPolicy"]

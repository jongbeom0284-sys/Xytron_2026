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
            "LANE_DRIVE": 20.0,
            "CONE_DRIVE": 11.0,
            "AVOID_STATIC": 13.0,
            "OVERTAKE": 15.0,
            "SHORTCUT": 12.0,
            "FINISH": 0.0,
            "STOP": 0.0,
        }
    )
    min_speed: float = 8.0
    max_speed: float = 26.0

    # 조향이 클수록 감속. |steer| 가 steer_full_slow 일 때 curve_min_ratio 배가 된다.
    steer_full_slow_deg: float = 34.0
    curve_min_ratio: float = 0.55

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

    # 시간 압박이 1.0 일 때 기본속도에 곱하는 값
    pressure_boost: float = 1.30

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

        # 곡선 감속
        ratio = 1.0 - (1.0 - cfg.curve_min_ratio) * min(
            1.0, abs(steer_deg) / max(1e-3, cfg.steer_full_slow_deg)
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

    @property
    def current(self) -> float:
        return self._current


__all__ = ["SpeedConfig", "SpeedPolicy"]

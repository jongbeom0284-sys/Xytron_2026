"""안전 감시 — 이 대회에서 '안전'의 정의는 일반 로보틱스와 다르다.

규정 p.21: "주행시간 측정이 시작된 이후에는 차량이 계속 주행해야 함.
            주행을 멈춘 후 1분 이내에 주행을 계속하지 못하면 실격."
규정 p.22: 차량에 손대면 5초 벌초. 한 바퀴에 15번(60초) 넘으면 실격.
규정 p.37: 3바퀴 주행시간 4분 초과 시 실격.

따라서 센서가 끊겼을 때 정석대로 비상정지하면 **실격에 가까워진다**.
코스를 이탈해 터치를 받는 쪽이 5초 벌초로 훨씬 싸다. 그래서 이 모듈의 기본
대응은 STOP 이 아니라 CREEP(마지막 유효 조향을 유지하며 저속 전진)이다.

STOP 은 두 경우에만 낸다:
  1) 3바퀴를 마쳐 결승선을 넘었다 (정상 종료)
  2) 퍼셉션이 아주 오래 죽어 크립조차 위험하다 → 이 시점엔 이미 심판 개입이
     들어올 상황이므로 정지가 오히려 안전하다
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .types import Observation


class SafetyLevel(str, Enum):
    OK = "OK"
    DEGRADED = "DEGRADED"   # 입력 일부 상실 → 크립으로 버틴다
    STOP = "STOP"           # 정지해야 함


@dataclass(frozen=True)
class SafetyDecision:
    level: SafetyLevel = SafetyLevel.OK
    reason: str = ""
    inputs_ready: bool = False

    @property
    def must_stop(self) -> bool:
        return self.level is SafetyLevel.STOP

    @property
    def degraded(self) -> bool:
        return self.level is SafetyLevel.DEGRADED


@dataclass
class SafetyConfig:
    # 출발 전 이 입력들이 준비돼야 WAIT_GREEN 으로 넘어간다.
    require_lidar_at_start: bool = True
    # 카메라가 이 시간(s) 이상 죽어 있으면 크립으로 내려간다.
    camera_degraded_after_s: float = 0.4
    # 크립으로도 이 시간(s) 이상 버티지 못하면 정지한다.
    # 1분(실격 기준)보다 충분히 짧게 잡아 심판 개입 여지를 남긴다.
    camera_dead_after_s: float = 12.0
    # 라이다가 죽어도 차선주행은 된다 — 회피만 못 한다. 그래서 정지 사유가 아니다.
    lidar_degraded_after_s: float = 1.0


class SafetyMonitor:
    """입력 신선도를 보고 안전 등급을 매긴다. 상태 전이 결정은 FSM 이 한다."""

    def __init__(self, config: Optional[SafetyConfig] = None) -> None:
        self.config = config or SafetyConfig()
        self._camera_ok_at: Optional[float] = None
        self._lidar_ok_at: Optional[float] = None

    def evaluate(self, obs: Observation, *, racing: bool) -> SafetyDecision:
        cfg = self.config
        now = obs.now

        if obs.camera_ok:
            self._camera_ok_at = now
        if obs.lidar_ok:
            self._lidar_ok_at = now

        cam_age = float("inf") if self._camera_ok_at is None else now - self._camera_ok_at
        lidar_age = float("inf") if self._lidar_ok_at is None else now - self._lidar_ok_at

        inputs_ready = obs.camera_ok and (obs.lidar_ok or not cfg.require_lidar_at_start)

        # 출발 전에는 준비될 때까지 얌전히 기다린다. 아직 시계가 안 돌아간다.
        if not racing:
            if inputs_ready:
                return SafetyDecision(SafetyLevel.OK, "", True)
            missing = []
            if not obs.camera_ok:
                missing.append("camera")
            if cfg.require_lidar_at_start and not obs.lidar_ok:
                missing.append("lidar")
            return SafetyDecision(
                SafetyLevel.OK, f"waiting for {'+'.join(missing)}", False
            )

        # 주행 중 — 여기서부터는 멈추는 것이 비싸다.
        if cam_age >= cfg.camera_dead_after_s:
            return SafetyDecision(
                SafetyLevel.STOP,
                f"camera dead {cam_age:.1f}s (creep exhausted)",
                inputs_ready,
            )
        if cam_age >= cfg.camera_degraded_after_s:
            return SafetyDecision(
                SafetyLevel.DEGRADED, f"camera stale {cam_age:.1f}s", inputs_ready
            )
        if lidar_age >= cfg.lidar_degraded_after_s:
            # 회피는 불가하지만 차선주행은 유효하다. 정지 사유가 아니다.
            return SafetyDecision(
                SafetyLevel.DEGRADED, f"lidar stale {lidar_age:.1f}s", inputs_ready
            )
        return SafetyDecision(SafetyLevel.OK, "", inputs_ready)


__all__ = ["SafetyLevel", "SafetyDecision", "SafetyConfig", "SafetyMonitor"]

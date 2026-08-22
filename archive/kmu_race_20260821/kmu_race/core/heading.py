"""IMU 헤딩(누적 요) 추정 — 자이로 적분 기반.

왜 쿼터니언을 안 쓰나 (2026-08-17 실차 bag `run_0817_1211` 포렌식):
  2바퀴(≈±720°) 주행 bag 에서 IMU orientation 쿼터니언의 오일러 3축 언랩
  적산이 roll −17° / pitch −8° / yaw +0° — **어느 축도 회전을 담지 않았다.**
  자세 융합(AHRS)이 헤딩을 전혀 적분하지 못하는 상태다 (실내 자기장 교란
  또는 펌웨어 캘리브레이션 문제로 추정).

  반면 원시 각속도(angular_velocity)는 살아 있다. 단, 바이어스가 크다:
  정지 구간 실측 gx +3.75 / gy +2.30 / gz +2.67 °/s (시작·끝 동일 = 고정
  바이어스, 드라이버가 바이어스 캘리브레이션을 안 함). 정지 바이어스를
  빼고 적분하면 gz = −811° ≈ −720°(2바퀴) − 91°(주차 회전) 로 정확히
  맞아떨어진다. → **차량 헤딩 = 자이로 z 적분 (바이어스 보정 필수).**

사용 계약:
  - 기동 직후 차량이 정지해 있는 동안(calib_s) 바이어스를 추정한다.
    본선 절차상 INIT/WAIT_GREEN 에서 차는 완전 정지이므로 성립한다.
    리플레이도 녹화 규약(차 정지 후 녹화 시작)으로 성립한다.
  - 캘리브레이션이 끝나기 전에는 yaw 0 을 유지한다 (정지 가정이므로 참).
  - 부호는 뒤집지 않고 그대로 적분한다 — 소비자(랩 교차검증)는 |Δyaw| 만
    쓰므로 장착 방향이 바뀌어도 크기는 보존된다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class GyroHeadingConfig:
    # 적분할 자이로 축. 0817 실차 실측: 차량 헤딩은 'z' (IMU 장착 자세 기준).
    # IMU 를 다시 장착하면 3축 적산으로 재확인할 것 (모듈 주석의 방법).
    axis: str = "z"
    # 바이어스 추정 창 길이 [s]. ★ '정지 상태 창'만 인정한다 (0817 실사고:
    # 노드 기동 순간 차를 들고 옮기던 중이라 바이어스가 엉터리로 잡혀 요가
    # 폭주 → 출발 37s 만에 가짜 랩 보충 → 136s FINISH 자진 정지).
    # 창 안 표준편차가 quiet_std_dps 미만이고 평균 크기가 max_bias_dps 미만
    # 이어야 캘리브 확정 — 아니면 조용해질 때까지 계속 기다린다(yaw 0 유지).
    calib_s: float = 3.0
    quiet_std_dps: float = 1.5      # 실측 정지 표준편차 ≈0.3~0.6°/s
    max_bias_dps: float = 8.0       # 실측 바이어스 2.5~2.7°/s (센서 고정 편향)


class GyroHeading:
    """각속도 [rad/s] 를 받아 누적 헤딩 [deg] 을 적분한다."""

    def __init__(self, config: GyroHeadingConfig | None = None) -> None:
        self.config = config or GyroHeadingConfig()
        self.yaw_deg = 0.0
        self.calibrated = False
        self.bias_rad_s = 0.0
        self._prev_t: float | None = None
        self._win: list = []            # (t, g) 캘리브 창 (슬라이딩)
        self.calib_rejects = 0          # 진단: 움직임으로 창을 버린 횟수

    def _pick(self, gx: float, gy: float, gz: float) -> float:
        a = self.config.axis
        return gx if a == "x" else gy if a == "y" else gz

    def update(self, gx: float, gy: float, gz: float, t: float) -> float:
        """IMU 한 건 반영. 반환값 = 현재 누적 헤딩 [deg]."""
        g = self._pick(gx, gy, gz)
        if not (math.isfinite(g) and math.isfinite(t)):
            return self.yaw_deg

        if not self.calibrated:
            # 슬라이딩 창 — '조용한' calib_s 창이 나올 때까지 기다린다.
            self._win.append((t, g))
            while self._win and (t - self._win[0][0]) > self.config.calib_s:
                self._win.pop(0)
            if self._win and (t - self._win[0][0]) >= self.config.calib_s * 0.95:
                vals = [v for _, v in self._win]
                n = len(vals)
                mean = sum(vals) / n
                var = sum((v - mean) ** 2 for v in vals) / n
                std_dps = math.degrees(math.sqrt(var))
                if (
                    std_dps < self.config.quiet_std_dps
                    and abs(math.degrees(mean)) < self.config.max_bias_dps
                ):
                    self.bias_rad_s = mean
                    self.calibrated = True
                    self._win = []
                else:
                    self.calib_rejects += 1
            self._prev_t = t
            return self.yaw_deg  # 캘리브 전 — 0 유지 (정지 대기 구간)

        if self._prev_t is not None:
            dt = t - self._prev_t
            # 시간 역행/정체(dt<=0)와 드랍아웃(비정상 dt)은 적분하지 않는다.
            if 0.0 < dt < 0.5:
                self.yaw_deg += math.degrees((g - self.bias_rad_s) * dt)
        self._prev_t = t
        return self.yaw_deg


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → ZYX 오일러 yaw [deg]. (진단·교차검증용으로만 남긴다)"""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(siny, cosy))


__all__ = ["GyroHeadingConfig", "GyroHeading", "yaw_from_quat"]

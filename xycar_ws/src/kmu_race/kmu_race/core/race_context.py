"""경주 진행 상태 — 지금 몇 바퀴째이고, 지름길을 썼고, 시간이 얼마 남았는가.

랩 카운팅 근거 (규정 p.6/p.18/p.38 트랙 배치에서 유도):
  출발 지점은 Gate(통과감지장치)와 4구 신호등 **사이**이고, 차량은 신호등 쪽을 본다.
  따라서 한 바퀴의 이동 순서는

      [출발] → 신호등(분기점) → ...트랙 한 바퀴... → Gate → [출발지점] → 신호등 → ...

  즉 **신호등 겐트리를 한 바퀴에 정확히 한 번** 만난다. Gate 는 우리 차가 읽을 수
  없는 심판측 장비지만 신호등은 카메라로 보이므로, 신호등 목격을 랩 마커로 쓴다.
  Gate 가 신호등보다 **먼저** 오므로, 4번째 목격 시점에는 이미 결승선을 넘은 뒤다.

      목격 1 = 출발 대기 (1바퀴 시작)
      목격 2 = 1바퀴 완료, 2바퀴 시작 → 경로 선택
      목격 3 = 2바퀴 완료, 3바퀴 시작 → 경로 선택
      목격 4 = 3바퀴 완료 (결승선 통과 후) → 종료

  IMU 누적 요를 교차검증으로 쓴다. 지름길을 타든 안 타든 닫힌 루프라 한 바퀴는
  약 360°다. 카메라가 신호등을 놓쳐도 요 적산으로 랩 누락을 잡아낼 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional


TOTAL_LAPS = 3
# 규정 p.37: 3바퀴 총 주행시간 4분 초과 시 실격 (벌초 미포함)
TIME_LIMIT_S = 240.0


@dataclass
class LapTrackerConfig:
    """랩 마커(신호등 목격) 판정 파라미터."""

    # 겐트리가 이 크기(px) 이상으로 보여야 '가까이서 목격'으로 친다.
    # 2026-08-13 지표 변경: 이 값은 '검증된 초록 램프 블롭의 가로폭'이다
    # (perception/signal_detector.py 참조).
    # 2026-08-17 재산출(kobot 새 카메라, 본선 트랙 2바퀴 bag run_0817_1211):
    #   새 카메라는 과노출로 램프 심이 작게 잡힌다 — 통과 접근 시 6~8px,
    #   출발 대기 시에도 6~8px (run4 구 카메라의 32px 과는 딴판).
    #   크기로는 근/원거리 분리가 안 되므로 오검 억제는 검출기의 링·구조
    #   게이트와 LapTracker 의 min_gap_s 가 담당한다 (동 bag 오검 0 실측).
    min_gantry_span_px: int = 5
    # 목격 확정에 필요한 연속 가시 시간 [s].
    # ★ 프레임 수가 아니라 '시간'이다 — 실차는 50Hz 제어 틱(카메라 프레임을
    #   ~1.7회 재소비)이고 리플레이는 30fps 카메라 프레임이라, 프레임 수 기준
    #   디바운스는 두 환경에서 다른 기준으로 목격을 세어 스윕으로 잡은
    #   임계값이 실차로 이전되지 않는다 (리뷰 T1 — 조기 FINISH 실격 경로).
    # 0817 실측 하향(0.10→0.06): 출발 대기 거리에서 검출 스트릭이 3프레임
    # (0.066s@30fps)까지 짧아진다 — 0.06 이면 3연속 프레임에 확정되면서
    # 1~2프레임 플리커는 여전히 걸러진다 (50Hz 실차 틱으로는 3틱).
    confirm_s: float = 0.06
    # 목격이 끝났다고 보기까지 필요한 연속 미검출 시간 [s]
    release_s: float = 0.40
    # 직전 목격으로부터 최소 이 시간(s)이 지나야 새 목격으로 센다.
    # 한 바퀴 최단(지름길)이 약 35~45s 이므로 15s 면 충분히 보수적이다.
    min_gap_s: float = 15.0
    # 요 적산 교차검증(경고용): 한 바퀴치 이상 돌았는데 목격이 없으면 의심.
    yaw_per_lap_deg: float = 300.0
    # ★ 요 보충 랩 (0817 3바퀴 bag `run_0817_1431` 로 설계 확정):
    #   이 대회장에서 통과 시 램프는 3~8px 로 날마다 요동한다 — 임계 5 는
    #   정밀도(주행 중 오검 0)는 완벽하지만 재현율이 나쁘다 (3랩 중 1랩만
    #   목격). 반면 자이로 z 적산은 3랩 −1112°(오차 3%)로 견고하다.
    #   그래서 목격이 놓쳐도 직전 랩 기준 누적 |Δ요| 가 이 값을 넘으면
    #   랩을 보충한다. 355 검증: 요 랩 경계 실측 −137/−497/−857 에 대해
    #   보충 발화 시각이 실제 통과와 1s 이내로 일치. 드리프트 여유는
    #   실측 1.1°/min × 4분 ≈ 4.4° < 5°. (360 이상은 금지 — 영원히 안 참)
    yaw_fill_deg: float = 355.0


@dataclass
class LapTracker:
    """신호등 겐트리 목격의 상승엣지를 세어 랩을 카운트한다.

    상태를 밖에서 못 건드리게 update() 하나만 노출한다.
    반환값 True = 이번 프레임에 '새 목격'이 확정됨.
    """

    config: LapTrackerConfig = field(default_factory=LapTrackerConfig)

    sightings: int = 0
    last_sighting_at: float = -1e9
    yaw_filled: int = 0                 # 요 보충으로 오른 랩 수 (진단용)
    _visible_since: Optional[float] = None
    _last_close_at: float = -1e9
    _in_sighting: bool = False
    _yaw_at_last_sighting: float = 0.0

    def update(self, *, gantry_span_px: int, now: float, yaw_deg: float) -> bool:
        """한 프레임 갱신. 새 목격(또는 요 보충 랩)이 확정되면 True.

        2단 하이브리드 (0817 재설계 — yaw_fill_deg 주석 참조):
          1) 목격 채널: 검증된 램프 블롭, 임계 보수적(정밀 우선)
          2) 요 채널: 목격이 놓친 랩을 누적 회전으로 보충(재현 담당)
        요 보충이 목격보다 먼저 발화해도 min_gap_s 가 곧이어 오는 실제
        목격의 중복 카운트를 막는다 (355° 시점 → 실제 통과까지 <15s).
        """
        close_enough = gantry_span_px >= self.config.min_gantry_span_px

        if close_enough:
            if self._visible_since is None:
                self._visible_since = now
            self._last_close_at = now
        else:
            self._visible_since = None

        # 목격 종료 판정: 충분히 오래 안 보이면 다음 목격을 받을 준비를 한다.
        if self._in_sighting and (now - self._last_close_at) >= self.config.release_s:
            self._in_sighting = False

        if not self._in_sighting and (
            self._visible_since is not None
            and (now - self._visible_since) >= self.config.confirm_s
        ):
            if (now - self.last_sighting_at) < self.config.min_gap_s:
                # 너무 빨리 다시 잡혔다 = 같은 겐트리를 두 번 센 것. 무시하되
                # 목격 상태로는 잠가서 스트릭이 계속 재발화하지 않게 한다.
                self._in_sighting = True
                return False
            self._in_sighting = True
            self.sightings += 1
            self.last_sighting_at = now
            self._yaw_at_last_sighting = yaw_deg
            return True

        # ── 요 보충 랩 — 첫 목격(출발 기준선) 이후에만 ──
        if (
            self.sightings >= 1
            and (now - self.last_sighting_at) >= self.config.min_gap_s
            and abs(yaw_deg - self._yaw_at_last_sighting) >= self.config.yaw_fill_deg
        ):
            self.sightings += 1
            self.yaw_filled += 1
            self.last_sighting_at = now
            self._yaw_at_last_sighting = yaw_deg
            self._in_sighting = False
            return True

        return False

    def ensure_start_sighting(self, *, now: float, yaw_deg: float) -> None:
        """출발 확정 시 호출 — '목격 1'(출발 기준선)이 없으면 강제로 세운다.

        출발 전용 원거리 경로(start_blob_px)로 출발했을 때 랩 마커 span 은
        0 이라 sightings 가 0 인 채 주행이 시작된다 (0817 실사고: lap 0/3
        으로 출발 → 4바퀴를 돌아야 FINISH). 출발 = 신호등 앞 = 목격 1 이
        규정상 자명하므로 여기서 기준선을 확정한다 (요 보충 랩의 기준도 됨).
        """
        if self.sightings == 0:
            self.sightings = 1
            self.last_sighting_at = now
            self._yaw_at_last_sighting = yaw_deg
            self._in_sighting = True

    def hold_start_baseline(self, *, yaw_deg: float) -> None:
        """출발 대기(WAIT_GREEN) 중 매 틱 호출 — 출발 전 상태를 고정한다.

        ① 목격 수를 1로 캡: 대기 중 재점멸/이송 중 오검(책상 장면 실측)이
           랩을 미리 채워 조기 FINISH 로 이어지는 것을 차단 (0817 실측:
           출발 전 유령 목격 3건).
        ② 요 기준선을 현재 헤딩으로 동기: 이송 중 회전(실측 +235°)이
           보충 랩 계산에 새어 들어가지 않게 한다.
        """
        if self.sightings > 1:
            self.sightings = 1
            self.yaw_filled = 0
        if self.sightings >= 1:
            self._yaw_at_last_sighting = yaw_deg

    def yaw_suggests_missed_lap(self, yaw_deg: float) -> bool:
        """카메라가 신호등을 놓쳤는지 요 적산으로 의심해 본다.

        직전 목격 이후 한 바퀴치 이상 돌았는데 목격이 없었다면 놓친 것이다.
        (자동 보정은 하지 않는다 — 오검출로 랩을 잘못 올리면 3바퀴를 못 채워
         실격이므로, 판단은 상위에 맡기고 여기서는 의심만 보고한다.)
        """
        if self.sightings == 0:
            return False
        turned = abs(yaw_deg - self._yaw_at_last_sighting)
        return turned >= self.config.yaw_per_lap_deg

    @property
    def current_lap(self) -> int:
        """지금 몇 바퀴째를 달리고 있는가 (1..3). 출발 전이면 0."""
        return min(self.sightings, TOTAL_LAPS)

    @property
    def finished(self) -> bool:
        """3바퀴를 마치고 결승선을 넘었는가."""
        return self.sightings > TOTAL_LAPS


@dataclass
class RaceContext:
    """경주 한 판의 진행 상태. FSM 과 정책 모듈이 공유한다."""

    lap: LapTracker = field(default_factory=LapTracker)

    race_started_at: Optional[float] = None
    state_entered_at: float = 0.0
    stop_reason: str = ""

    # 규정 p.6: 지름길은 3바퀴 중 1번만. 이 플래그가 그 '1번'을 잠근다.
    shortcut_used: bool = False
    shortcut_committed_at: Optional[float] = None

    # 좌회전 신호를 실제로 본 랩 (진단/로그용)
    left_signal_seen_on_lap: Optional[int] = None

    # 미션 수행 이력 (로그 및 회차 간 비교용)
    events: List[str] = field(default_factory=list)

    # ── 시간 예산 ──────────────────────────────────────────────────
    def elapsed(self, now: float) -> float:
        if self.race_started_at is None:
            return 0.0
        return now - self.race_started_at

    def remaining(self, now: float) -> float:
        return TIME_LIMIT_S - self.elapsed(now)

    def time_pressure(self, now: float) -> float:
        """0.0(여유) ~ 1.0(임박). 속도 정책이 읽어서 목표속도를 올린다.

        남은 바퀴 수 대비 남은 시간으로 계산한다. 지름길을 이미 썼으면
        남은 바퀴가 짧아지므로 압박이 줄어드는 것이 맞다.
        """
        if self.race_started_at is None:
            return 0.0
        # 시운전(total_laps 99 = 랩 무제한)에서는 '남은 바퀴' 개념이 없다 —
        # 계산하면 per_lap 이 2.4s 로 나와 출발부터 압박 1.0 → LANE_DRIVE 속도
        # ×1.3 이 걸린다 ("40%" 로 돌린다고 믿었는데 실제 52%, 0818 저녁 감사).
        if TOTAL_LAPS > 3:
            return 0.0
        laps_left = max(0, TOTAL_LAPS - self.lap.current_lap + 1)
        if laps_left == 0:
            return 0.0
        remaining = self.remaining(now)
        if remaining <= 0.0:
            return 1.0
        # 한 바퀴에 쓸 수 있는 시간. 넉넉하면 0, 빠듯하면 1로 수렴.
        per_lap = remaining / laps_left
        comfortable = 70.0   # 이 정도면 미션 포함 여유롭게 한 바퀴
        tight = 35.0         # 이 밑이면 전력
        if per_lap >= comfortable:
            return 0.0
        if per_lap <= tight:
            return 1.0
        return (comfortable - per_lap) / (comfortable - tight)

    # ── 지름길 판단 ────────────────────────────────────────────────
    def may_take_shortcut(self, signal_allows_left: bool) -> bool:
        """지금 지름길로 들어가도 되는가.

        규정 p.6/p.35: 4구 신호등에서 좌회전 신호를 인식한 뒤 지름길 주행 가능.
                   좌회전 신호는 2바퀴 또는 3바퀴(마지막)째에 랜덤으로 나온다.
                   3바퀴 중 1번만 통과할 수 있다.
        기회는 랜덤하게 딱 한 번 오므로, 보이면 무조건 잡는다.
        """
        if self.shortcut_used:
            return False
        if not signal_allows_left:
            return False
        return self.lap.current_lap in (2, 3)

    def commit_shortcut(self, now: float) -> None:
        self.shortcut_used = True
        self.shortcut_committed_at = now
        self.left_signal_seen_on_lap = self.lap.current_lap
        self.log(now, f"shortcut committed on lap {self.lap.current_lap}")

    def log(self, now: float, msg: str) -> None:
        self.events.append(f"[{self.elapsed(now):6.1f}s] {msg}")


__all__ = [
    "TOTAL_LAPS",
    "TIME_LIMIT_S",
    "LapTrackerConfig",
    "LapTracker",
    "RaceContext",
]

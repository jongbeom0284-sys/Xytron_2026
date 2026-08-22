"""전방 타겟 통과 — 고정장애물 회피(p.30)와 방해차량 추월(p.31~34)의 공통 컨트롤러.

설계 출처: mastic-choi/UMK `controller/obstacle_avoidance.py` 의 통찰을 따랐다.
  "두 미션은 결국 같은 기동이다 — 타겟이 없는 쪽으로 지나간다.
   다른 점은 재평가 빈도뿐이다."
본 구현은 거기에 2026 규정의 두 조항을 명시적으로 넣었다:
  · 복귀 시한 (p.32 "뒷차와의 간격이 90센치 이상이면 차선이탈로 판정")
  · 추돌 벌초 회피 (p.34 가해·피해 모두 10초)

규정 인용 (p.32~33):
  · 방해차량은 한 대가 느린 속도로 1차선과 2차선을 오가며 주행한다.
  · "방해차량이 주행하지 않는 차선 쪽으로 추월해야 함."
    반대쪽으로 시도하면 차선이탈로 간주한다.
  · 추월 중에는 실선 이탈이 허용된다. 다만 추월 후 최대한 빨리 차선 안으로
    복귀해야 하고, 뒷차와 90cm 이상 벌어진 시점에도 밖이면 차선이탈 판정.

부호 규약: y(횡) 는 좌측이 +. side 는 -1 = 왼쪽으로 통과, +1 = 오른쪽으로 통과.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .types import TrackedObject


class PassPhase(str, Enum):
    IDLE = "IDLE"
    SHIFT = "SHIFT"          # 옆 차선으로 이동 중
    ALONGSIDE = "ALONGSIDE"  # 타겟 옆을 지나는 중
    RETURN = "RETURN"        # 원래 차선으로 복귀 중


@dataclass
class PassConfig:
    # ── 횡이동량 (0818 저녁 재설계) ──
    # 옛 코드는 고정 300px 을 옆으로 갔다. 실측 축척 ≈810 px/m 에서 300px 은
    # 0.37m — 도로 반폭 0.41m 이라 차(폭 0.30m) 오른 바퀴가 흰 실선을 넘는다.
    # 필요한 이동량은 타겟의 횡위치에서 나온다:
    #     d = clearance − side·target_y   (target_y 좌측+, side +1=우측 통과)
    # 예) 타겟이 좌차선 중앙(+0.2m)이고 우측 통과 → 0.35−0.2 = 0.15m ≈ 120px
    #     타겟이 거의 정면(+0.05m) → 0.30m ≈ 245px (여전히 도로 안)
    clearance_m: float = 0.35          # 차 반폭 0.15 + 모형 반폭 ~0.15 + 여유
    px_per_m: float = 810.0            # BEV 가로 축척 (0818 실측)
    min_shift_px: float = 100.0        # 이보다 작은 이동은 무의미
    # 최대 횡이동 [px] — 상한(안전 캡). 0.37m: 이 이상은 흰 실선 밖이다.
    pass_offset_px: float = 300.0
    # 기동 중 타겟을 계속 추적할 횡방향 코리도 반폭 [m]. 기본 nearest_front
    # 코리도(0.45m)는 '내 경로를 막는가' 용이라, 내가 옆 차선으로 0.7m 비키는
    # 순간 타겟이 시야에서 사라져 '통과 완료'로 오판된다(리뷰 C2 — 장애물
    # 앞에서 원래 차선으로 복귀하는 사고). 횡이동 폭 + 여유만큼 넓게 잡는다.
    track_corridor_m: float = 1.2
    # 타겟 횡중심이 이 값[m] 이내면 '정면'으로 보고 다른 근거로 방향을 정한다.
    center_deadzone_m: float = 0.14
    # 진입/이탈 히스테리시스 (Apollo lane_borrow 패턴)
    clear_frames_to_return: int = 5
    # ★ 프레임 수만으로는 부족하다 (0820 감사). 제어는 50Hz 인데 라이다는
    #   실측 8.8Hz(스캔 주기 0.114s) 다 — 5틱은 0.10s 라 **한 스캔도 안 되는**
    #   시간이고, 연관이 한 번만 끊겨도(lidar_tracker 는 missed==0 만 내보낸다)
    #   같은 '타겟 없음' 관측이 5틱 연속 반복돼 즉시 복귀 조건이 선다.
    #   장애물 바로 앞에서 차선으로 돌아오는 경로다. 최소 3스캔을 요구한다.
    clear_seconds_to_return: float = 0.35
    switch_frames: int = 3
    # 횡이동 수렴 계수
    lateral_alpha_out: float = 0.22
    lateral_alpha_back: float = 0.30
    lateral_done_px: float = 24.0
    # 추돌 방지 종방향 간격 [m]. 이보다 가까운데 아직 못 비켰으면 속도를 죽인다.
    min_gap_m: float = 0.55
    # 규정 p.32: 뒷차와 이 거리[m] 이상 벌어지기 전에 차선 안으로 들어와야 한다.
    return_deadline_m: float = 0.90
    # 속도 [자이카 단위]
    shift_speed: float = 12.0
    alongside_speed: float = 16.0   # 추월은 상대보다 빨라야 지나간다
    return_speed: float = 14.0
    # 조향 목표에서 전방주시 성분의 비중
    preview: float = 0.4
    # 기동 전체 타임아웃 [s]. 교착 시 차선주행으로 되돌려 '멈춤'을 피한다.
    timeout_s: float = 6.0


@dataclass
class PassResult:
    lateral_offset_px: float = 0.0
    speed: float = 0.0
    done: bool = False
    # ★ 타임아웃 탈출은 '통과 완료'가 아니다 (0820 감사). 옛 코드는 둘 다
    #   done=True 로 내서 FSM 이 'pass complete' 로 읽었고, 다음 틱에 타겟이
    #   그대로 있으면 곧바로 재진입해 리밋사이클이 됐다.
    aborted: bool = False
    status: str = "idle"     # idle / passing / blocked
    phase: PassPhase = PassPhase.IDLE
    side: int = 0


class TargetPassing:
    """정지 차량·주행 차량 공통 통과 컨트롤러.

    moving=True  → 방해차량. 매 프레임 방향을 재평가하고 끼어들기에 대응한다.
    moving=False → 고장난 차량. 한번 정한 방향을 유지한다.
    """

    def __init__(
        self, *, moving: bool = False, config: Optional[PassConfig] = None
    ) -> None:
        self.moving = moving
        self.config = config or PassConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = PassPhase.IDLE
        self.side = 0
        # ★ offset_px 는 '차선 목표에 더하는 부호 있는 횡이동량'[px] 이다 —
        #   측정된 lane_offset_px 에 이 값을 더한 것이 PID 오차가 된다. 옛
        #   코드는 이 램프 자체를 오차로 넣어 측정과 무관한 개루프 풀락(−50,
        #   1.7s)이 됐다 (0818 저녁 수치 재현). 차가 실제로 d 만큼 옮기면
        #   lane_offset_px → −side·d 가 되어 오차가 0 으로 닫힌다 (폐루프).
        self.offset_px = 0.0
        self.target_offset_px = 0.0
        self._clear_cnt = 0
        self._clear_since: Optional[float] = None
        self._conflict_cnt = 0
        self._started_at: Optional[float] = None
        self._passed_target_x: Optional[float] = None

    def desired_shift_px(self, side: int, target_y: float) -> float:
        """타겟 횡위치에서 필요한 부호 있는 횡이동량 [px] (설정 주석의 식)."""
        cfg = self.config
        d_m = cfg.clearance_m - side * target_y
        d_px = d_m * cfg.px_per_m
        d_px = max(cfg.min_shift_px, min(cfg.pass_offset_px, d_px))
        return side * d_px

    # ── 방향 선택 ─────────────────────────────────────────────────
    def choose_side(
        self,
        target_y: float,
        *,
        left_clear: bool,
        right_clear: bool,
        lane_side: int,
    ) -> int:
        """타겟이 '없는' 쪽을 고른다. 반환 -1(좌) / +1(우) / 0(양쪽 막힘).

        1순위 — 타겟 횡위치. 규정이 요구하는 "방해차량이 주행하지 않는 차선"이
                바로 이것이다. 타겟이 왼쪽에 있으면 오른쪽으로 간다.
        2순위 — 정면이라 1순위로 못 가릴 때: 비어 있는 쪽.
        3순위 — 둘 다 비었으면 노란 점선 건너편. 규정 p.27 이 1·2차선 구분을
                없앴으므로 어느 쪽이든 합법이고, 바깥 흰 실선만 안 넘으면 된다.
        """
        cfg = self.config
        if target_y > cfg.center_deadzone_m:
            want = +1          # 타겟이 좌측 → 우측으로 통과
        elif target_y < -cfg.center_deadzone_m:
            want = -1          # 타겟이 우측 → 좌측으로 통과
        else:
            want = 0

        if want != 0:
            if self._side_clear(want, left_clear, right_clear):
                return want
            # 규정이 요구하는 쪽이 막혔다. 반대쪽은 '차선이탈 간주' 리스크가 있지만
            # 충돌(추돌 10초 + 원위치)보다는 낫다. 막히지 않았을 때만 쓴다.
            other = -want
            if self._side_clear(other, left_clear, right_clear):
                return other
            return 0

        if left_clear and not right_clear:
            return -1
        if right_clear and not left_clear:
            return +1
        if left_clear and right_clear:
            return -1 if lane_side >= 0 else +1
        return 0

    @staticmethod
    def _side_clear(side: int, left_clear: bool, right_clear: bool) -> bool:
        return left_clear if side < 0 else right_clear

    def _target_cuts_in(self, target_y: float) -> bool:
        """주행 타겟이 내가 가려는 쪽으로 넘어왔는가.

        내가 옆으로 비키는 움직임 자체는 target_y 를 반대 방향으로 밀기 때문에
        (좌로 비키면 타겟이 상대적으로 우측으로 보인다) 자기 기동이 오검출을
        만들지 않는다.
        """
        dz = self.config.center_deadzone_m
        if self.side < 0:
            return target_y > dz
        return target_y < -dz

    # ── 메인 갱신 ─────────────────────────────────────────────────
    def update(
        self,
        *,
        target: Optional[TrackedObject],
        lane_offset_px: float,
        lane_lookahead_px: float,
        lane_side: int,
        left_clear: bool,
        right_clear: bool,
        now: float,
    ) -> PassResult:
        cfg = self.config
        status = "passing"
        done = False

        if self._started_at is None and self.phase is not PassPhase.IDLE:
            self._started_at = now
        if (
            self._started_at is not None
            and (now - self._started_at) > cfg.timeout_s
        ):
            # 교착. 규정 p.21 상 멈추는 게 가장 비싸므로 차선주행으로 돌려보낸다.
            self.reset()
            return PassResult(
                0.0, cfg.return_speed, True, True, "idle", PassPhase.IDLE, 0
            )

        target_present = target is not None
        target_y = target.y if target else 0.0
        target_x = target.x if target else float("inf")

        if self.phase is PassPhase.IDLE:
            status = "idle"
            if target_present:
                side = self.choose_side(
                    target_y,
                    left_clear=left_clear,
                    right_clear=right_clear,
                    lane_side=lane_side,
                )
                if side == 0:
                    # 양쪽 다 막혔다. 흰 실선 밖으로 도망가지 않고 감속해 버틴다.
                    # 단 타임아웃 시계는 여기서도 무장한다 — blocked 로 영원히
                    # 머물면 FSM 에 시간 탈출구가 없어 1분 정지 실격으로 간다
                    # (리뷰 C3).
                    if self._started_at is None:
                        self._started_at = now
                    status = "blocked"
                else:
                    self.side = side
                    self.offset_px = 0.0
                    self.target_offset_px = self.desired_shift_px(side, target_y)
                    self.phase = PassPhase.SHIFT
                    self._started_at = now

        elif self.phase is PassPhase.SHIFT:
            if self.moving and target_present and self._target_cuts_in(target_y):
                self._conflict_cnt += 1
                if self._conflict_cnt >= cfg.switch_frames:
                    other = -self.side
                    if self._side_clear(other, left_clear, right_clear):
                        self.side = other
                        self.target_offset_px = self.desired_shift_px(other, target_y)
                        self._conflict_cnt = 0
                    else:
                        self.phase = PassPhase.RETURN
                        self._conflict_cnt = 0
                        status = "blocked"
            else:
                self._conflict_cnt = 0
                if target_present:
                    # 타겟이 옆으로 움직이면(방해차량) 필요한 이동량도 따라간다
                    self.target_offset_px = self.desired_shift_px(self.side, target_y)

            self.offset_px += cfg.lateral_alpha_out * (
                self.target_offset_px - self.offset_px
            )
            if abs(self.target_offset_px - self.offset_px) < cfg.lateral_done_px:
                self.phase = PassPhase.ALONGSIDE

        elif self.phase is PassPhase.ALONGSIDE:
            if target_present and target_x > -0.1:
                self._clear_cnt = 0
                self._clear_since = None
            else:
                self._clear_cnt += 1
                if self._clear_since is None:
                    self._clear_since = now
            if (
                self._clear_cnt >= cfg.clear_frames_to_return
                and self._clear_since is not None
                and (now - self._clear_since) >= cfg.clear_seconds_to_return
            ):
                self._clear_cnt = 0
                self._clear_since = None
                self._passed_target_x = target_x if target_present else -0.2
                self.phase = PassPhase.RETURN

        elif self.phase is PassPhase.RETURN:
            # 규정 p.32: 뒷차와 90cm 벌어지기 전에 차선 안으로 들어와야 한다.
            # 남은 여유가 줄수록 복귀를 공격적으로 만든다.
            urgency = 1.0
            if target_present and target_x < 0.0:
                gap = -target_x
                slack = max(0.0, cfg.return_deadline_m - gap)
                urgency = 1.0 + 2.0 * (1.0 - slack / cfg.return_deadline_m)
            alpha = min(0.9, cfg.lateral_alpha_back * urgency)
            self.offset_px *= 1.0 - alpha
            if abs(self.offset_px) < 8.0:
                self.reset()
                done = True
                status = "idle"

        # PID 오차 = 측정된 차선 목표 + 부호 있는 횡이동량 (근/원거리 혼합).
        # 차가 옮겨가면 lane_offset_px 가 −offset_px 로 다가가 오차가 닫힌다.
        steer_target_px = (
            (1.0 - cfg.preview) * (lane_offset_px + self.offset_px)
            + cfg.preview * (lane_lookahead_px + self.offset_px)
        )

        speed = self._speed_for_phase()
        if (
            target_present
            and target_x < cfg.min_gap_m
            and self.phase in (PassPhase.IDLE, PassPhase.SHIFT)
        ):
            # 추돌하면 벌초 10초 + 원위치. 아직 못 비켰으면 붙지 않는다.
            speed = min(speed, cfg.shift_speed * 0.3)
        if status == "blocked":
            speed = min(speed, cfg.shift_speed * 0.25)

        return PassResult(
            lateral_offset_px=steer_target_px,
            speed=speed,
            done=done,
            status=status,
            phase=self.phase,
            side=self.side,
        )

    def _speed_for_phase(self) -> float:
        cfg = self.config
        if self.phase is PassPhase.ALONGSIDE:
            return cfg.alongside_speed
        if self.phase is PassPhase.RETURN:
            return cfg.return_speed
        return cfg.shift_speed


__all__ = ["PassPhase", "PassConfig", "PassResult", "TargetPassing"]

"""모드 → 주행명령. core 계층의 최상단이며 여전히 rclpy 를 모른다.

FSM 이 '무엇을 하는 중인가'를 정하고, 여기서 '그래서 바퀴를 어떻게 돌릴까'를 만든다.
둘을 나눠 놓은 덕에 test/ 에서 관측을 손으로 지어내 명령까지 끝단 검증할 수 있다.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .control import LaneController, PixelPID, SteerSmoother, pure_pursuit_steer
from .race_context import RaceContext
from .race_fsm import Mode
from .safety import SafetyDecision
from .speed_policy import SpeedConfig, SpeedPolicy
from .target_passing import PassConfig, TargetPassing
from .types import DriveCommand, Observation


@dataclass
class PolicyConfig:
    max_steer: float = 50.0
    pid: PixelPID = field(default_factory=PixelPID)
    # 차선주행 조향기 — 곡률 피드포워드 + 방향정렬 + 횡오차 (control.py 주석)
    lane: LaneController = field(default_factory=LaneController)
    speed: SpeedConfig = field(default_factory=SpeedConfig)
    pass_static: PassConfig = field(
        default_factory=lambda: PassConfig(
            shift_speed=11.0, alongside_speed=14.0, return_speed=13.0
        )
    )
    pass_moving: PassConfig = field(default_factory=PassConfig)
    smoother: SteerSmoother = field(default_factory=SteerSmoother)

    # ── 라바콘 ──
    # 게이트 중앙을 향하는 퓨어 퍼수트 전방주시거리 [m]. ★ 이 값은 '상한'
    # 이고 실제로는 **관측된 게이트 깊이**로 잘라 쓴다 — 실제로 본 콘보다 먼
    # 곳을 조준하면 코너에서 통로를 가로지른다 (경쟁팀 ddm 이 실주행 bag 으로
    # 진단한 실패 모드, chooseTargetX 클램프).
    cone_lookahead_m: float = 0.75
    cone_lookahead_min_m: float = 0.35
    # BEV 픽셀 → 미터 환산 (가로, ROI 안 균일).
    # ★ 0818 저녁 재실측: 노란 중앙선↔흰 선 = 335px ↔ 반차선 0.41m(줄자)
    #   ⇒ ≈ 810 px/m (근·원거리 행 동일). 낮의 545 는 '흰선 간격 450px' 가정
    #   에서 나온 값인데 그 450 은 가짜 쌍(이음새 등)이 섞인 폭 히스토리였다.
    #   세로 축척은 미실측 — control.bev_aspect(0.9) 가정으로 ≈730 px/m.
    # (라이다 게이트가 1순위라 이 값의 역할은 카메라 폴백 경로의 전방주시
    #  클램프·가상 게이트 환산뿐이다. 원거리 투영 콘은 ROI 밖 외삽이라 축척이
    #  더 흐릿하다 — 그래서 카메라 경로는 폴백이다.)
    px_per_m: float = 810.0
    px_per_m_fwd: float = 730.0
    # 한쪽 콘만 보일 때 밀어내는 세기 [deg / 정규화오차] (가상 게이트 실패 시)
    cone_repulse_gain: float = 26.0
    # 한쪽 줄만 보일 때 쓸 '학습된 통로 반폭' [m]. 관측될 때마다 EMA 로
    # 갱신해 가상 게이트를 만든다 — 단순 반발(풀락)은 반대편 콘 열로 처박는
    # 동작이라 쓰지 않는다 (tj2024 실패 사례).
    cone_half_corridor_m: float = 0.30
    cone_half_corridor_alpha: float = 0.15
    cone_half_corridor_min_m: float = 0.15
    cone_half_corridor_max_m: float = 0.70

    # ── 지름길 ──
    # 진입 좌회전은 개루프로 돈다. 분기점에서는 차선 도색이 끊겨 차선검출을
    # 믿을 수 없기 때문이다. 요 변화량으로 종료를 판정한다(FSM 담당).
    shortcut_steer_deg: float = -34.0

    # ── 결승 관성 주행 (FINISH 즉시 정지 금지 — update() FINISH 분기 참조) ──
    # 16(≈1.28m/s) × 6s ≈ 7.7m: 요 보충 발화점(마지막 커브 출구)에서
    # Gate·신호등을 지나 확실히 넘는 거리. 시간이 다 되면 정지.
    finish_coast_s: float = 6.0
    finish_coast_speed: float = 16.0

    # ── 출발 직진 홀드 ──
    # 출발 위치에서는 겐트리 지주·신호등 기둥이 차선으로 오검되기 쉽다.
    # 2026 참가팀 셋(jeonghun43·pumkyQ·UMK)이 독립적으로 같은 가드에 수렴한
    # 공통 실패 모드라, 출발 후 이 시간 동안은 조향을 중립에 고정한다.
    # 출발 시 차는 정지 상태에서 정렬돼 있으므로 짧은 맹목 직진은 안전하다.
    start_straight_hold_s: float = 1.2

    # ── 열화 모드 ──
    # 카메라가 잠깐 죽었을 때 마지막 조향을 이만큼 감쇠시키며 유지한다.
    # 규정 p.21 상 멈추는 것이 가장 비싸므로 직진 크립으로 버틴다.
    degraded_steer_decay: float = 0.90


class DrivePolicy:
    def __init__(self, config: Optional[PolicyConfig] = None) -> None:
        self.config = config or PolicyConfig()
        self.pid = self.config.pid
        self.lane_ctl = self.config.lane
        # 두 컨트롤러가 같은 PID 인스턴스를 공유하면 적분·미분 상태가 섞인다.
        self.lane_ctl.pid = self.pid
        self.speed = SpeedPolicy(self.config.speed)
        self.smoother = self.config.smoother
        self.pass_static = TargetPassing(
            moving=False, config=self.config.pass_static
        )
        self.pass_moving = TargetPassing(
            moving=True, config=self.config.pass_moving
        )
        self._last_steer = 0.0
        self._last_pass_done = False
        self._prev_mode: Optional[Mode] = None
        # 라바콘 통로 반폭 학습값 [m] — 단측 구간의 가상 게이트에 쓴다
        self._cone_half_m = self.config.cone_half_corridor_m

    @property
    def maneuver_done(self) -> bool:
        """직전 update() 에서 통과 기동이 끝났는가. FSM 이 읽어 간다."""
        return self._last_pass_done

    def reset(self) -> None:
        self.pid.reset()
        self.lane_ctl.reset()
        self.speed.reset()
        self.smoother.reset()
        self.pass_static.reset()
        self.pass_moving.reset()
        self._last_steer = 0.0
        self._last_pass_done = False

    # ──────────────────────────────────────────────────────────────
    def update(
        self,
        mode: Mode,
        obs: Observation,
        ctx: RaceContext,
        safety: SafetyDecision,
    ) -> DriveCommand:
        cfg = self.config
        self._last_pass_done = False

        # 기동 상태 위생: 통과 컨트롤러는 모드에 '진입하는 순간' 백지여야 한다.
        # 승격(AVOID→OVERTAKE)·STOP 개입으로 중간 상태(_started_at 등)가 남으면
        # 다음 진입 첫 틱에 6초 타임아웃이 걸려 가짜 '통과 완료'가 나온다.
        if mode is not self._prev_mode:
            if mode is Mode.AVOID_STATIC:
                self.pass_static.reset()
            elif mode is Mode.OVERTAKE:
                self.pass_moving.reset()
            if mode is Mode.LANE_DRIVE:
                # 기동에서 돌아오면 곡률·기울기 EMA 가 옛 장면 값이다.
                self.lane_ctl.reset()
            self._prev_mode = mode

        if mode is Mode.FINISH:
            # ★ 결승 관성 주행 (0817 하이브리드 랩의 구조적 귀결):
            # 요 보충 랩은 마지막 커브가 355°를 채우는 지점에서 발화하는데,
            # 직선에서는 요가 안 쌓이므로 그 지점은 결승선(Gate) '앞'일 수
            # 있다 (3바퀴 bag 실측: 보충 발화가 겐트리 실통과보다 11~15s
            # 앞섬). 즉시 정지하면 선을 안 넘은 채 서 버린다 — FINISH 후
            # finish_coast_s 동안 차선을 따라 저속 전진해 확실히 넘고 선다.
            self.pass_static.reset()
            self.pass_moving.reset()
            if (
                ctx.state_entered_at > 0.0
                and (obs.now - ctx.state_entered_at) < cfg.finish_coast_s
            ):
                steer = 0.0
                if obs.camera_ok and obs.lane.valid:
                    steer = self.pid.update(obs.lane.offset_px)
                steer = self.smoother.update(steer, obs.now)
                self._last_steer = steer
                return DriveCommand(
                    steer, cfg.finish_coast_speed, "FINISH_COAST"
                ).clamped(cfg.max_steer, cfg.speed.max_speed)
            self.speed.reset()
            return DriveCommand(0.0, 0.0, mode.value)

        if mode is Mode.STOP:
            self.speed.reset()
            self.pass_static.reset()
            self.pass_moving.reset()
            return DriveCommand(0.0, 0.0, mode.value)

        if mode in (Mode.INIT, Mode.WAIT_GREEN):
            # INIT: 입력 대기 중 — 여기서 기본 분기(차선주행)로 떨어지면
            # 런치 직후 1~2초간 실제 주행 명령이 나간다 (2026-08-13 실차
            # 스모크 테스트에서 v=17.7 발행 실측). 출발 전에는 무조건 정지.
            # WAIT_GREEN: 규정 p.19 — 신호등 앞 정차 대기. 조향 중립.
            return DriveCommand(0.0, 0.0, mode.value)

        # 카메라가 상했으면 마지막 조향을 감쇠시키며 저속 직진으로 버틴다.
        if safety.degraded and not obs.camera_ok:
            steer = self._last_steer * cfg.degraded_steer_decay
            self._last_steer = steer
            target = self.speed.target_for(
                mode.value, steer_deg=steer, time_pressure=0.0, degraded=True
            )
            return DriveCommand(
                steer, self.speed.step(target, obs.now), "DEGRADED"
            ).clamped(cfg.max_steer, cfg.speed.max_speed)

        steer, speed_override = self._steer_for(mode, obs, ctx)

        steer = self.smoother.update(steer, obs.now)
        self._last_steer = steer
        # 굽이침 지표 갱신 (speed_policy 의 조향 변화율 감속)
        self.speed.note_steer(steer, obs.now)

        target = self.speed.target_for(
            mode.value,
            steer_deg=steer,
            time_pressure=ctx.time_pressure(obs.now),
            override=speed_override,
            degraded=safety.degraded,
        )
        speed = self.speed.step(target, obs.now)

        return DriveCommand(steer, speed, mode.value).clamped(
            cfg.max_steer, cfg.speed.max_speed
        )

    # ──────────────────────────────────────────────────────────────
    def _steer_for(
        self, mode: Mode, obs: Observation, ctx: RaceContext
    ) -> tuple[float, Optional[float]]:
        """모드별 조향 [deg] 과 속도 오버라이드를 만든다."""
        cfg = self.config

        if mode is Mode.SHORTCUT:
            # 개루프 좌회전. 분기점은 차선 도색이 끊겨 있어 검출을 믿지 않는다.
            return cfg.shortcut_steer_deg, None

        if mode is Mode.CONE_DRIVE:
            return self._cone_steer(obs), None

        if mode in (Mode.AVOID_STATIC, Mode.OVERTAKE):
            passer = (
                self.pass_moving if mode is Mode.OVERTAKE else self.pass_static
            )
            # 기동 중에는 넓은 코리도로 타겟을 계속 추적한다 — 기본 0.45m 는
            # 자기 횡이동(0.7m)만으로 타겟이 사라져 조기 복귀를 만든다(리뷰 C2).
            result = passer.update(
                target=obs.lidar.nearest_front(
                    max_lateral=passer.config.track_corridor_m
                ),
                lane_offset_px=obs.lane.offset_px,
                lane_lookahead_px=obs.lane.lookahead_px,
                lane_side=obs.lane.lane_side,
                left_clear=obs.lidar.left_clear,
                right_clear=obs.lidar.right_clear,
                now=obs.now,
            )
            self._last_pass_done = result.done
            return self.pid.update(result.lateral_offset_px), result.speed

        # LANE_DRIVE (및 그 외 기본)
        if (
            ctx.race_started_at is not None
            and (obs.now - ctx.race_started_at) < cfg.start_straight_hold_s
        ):
            # 출발 직진 홀드 — 겐트리 지주 오검으로 인한 초반 급조향 차단
            return 0.0, None
        return self._lane_steer(obs), None

    def _lane_steer(self, obs: Observation) -> float:
        """차선 중앙 추종. 전방주시점을 섞어 곡률에 선행 반응한다."""
        lane = obs.lane
        if not lane.valid:
            # 양쪽 다 못 봤다. 직전 조향을 유지하며 빠져나가길 기다린다.
            return self._last_steer * 0.95
        # 0817: 원거리 가중 0.4→0.2 (원거리 점 자체도 검출기에서 중간 거리로
        # 내렸다). 코너 선행 조향 과다 = 진입 전 꺾음 → 차선 이탈의 주범.
        # 0817 재설계: 원거리 점 혼합(0.8·near + 0.2·far) → 곡률 피드포워드 +
        # 방향 정렬 + 횡오차. far 점 혼합은 코너가 화면에 '보이기만 해도'
        # 조향을 만들어 진입 전에 꺾었다 (사용자 관찰과 실측 일치).
        return self.lane_ctl.update(
            lane.offset_px,
            lane.heading_slope,
            lane.curvature_px,
            lane.lane_width_px,
            both_found=lane.both_found,
        )

    def _cone_steer(self, obs: Observation) -> float:
        """라바콘 구간 조향.

        규정 p.23~25 의 도면은 콘이 **좌우 두 줄**로 놓이고 그 사이를 지나가는
        형태다. 그래서 '주황 덩어리 하나를 따라간다'가 아니라 '좌우 게이트의
        중앙을 지난다'로 푼다.

        0817 재설계 두 가지 (경쟁팀 ddm 의 실주행 bag 진단 반영):
          ① 전방주시거리를 **관측된 게이트 깊이로 자른다**. 실제로 본 콘보다
             먼 곳을 조준하면 코너에서 통로를 가로지른다.
          ② 한쪽 줄만 보이면 **학습된 통로 반폭으로 가상 게이트**를 만든다.
             반발 조향(풀락)은 반대편 콘 열로 처박는 동작이다 (tj2024 사례).
        콘을 건드리면 개당 3초(p.24)다.
        """
        cfg = self.config
        cone = obs.cone
        px_per_m = max(1.0, cfg.px_per_m)

        # 관측될 때마다 통로 반폭을 학습한다 (단측 구간의 가상 게이트용)
        if cone.gate_valid and cone.gate_width_px > 1.0:
            measured = (cone.gate_width_px / px_per_m) / 2.0
            a = cfg.cone_half_corridor_alpha
            self._cone_half_m = (1.0 - a) * self._cone_half_m + a * measured
            self._cone_half_m = max(
                cfg.cone_half_corridor_min_m,
                min(cfg.cone_half_corridor_max_m, self._cone_half_m),
            )

        # ── 1순위: 라이다 게이트 (0818 신설, perception/lidar_cones.py) ──
        # 같은 구간 실측: 카메라 게이트 58% vs 라이다 80%. 게다가 라이다는
        # 조명·반사와 무관하고 **미터로 직접** 재므로 px_per_m 추정 오차가
        # 없다. 카메라 게이트는 라이다가 못 만든 프레임의 폴백이 된다.
        if cone.lidar_gate_valid:
            half_lidar = cone.lidar_gate_width_m / 2.0
            if half_lidar > 0.05:
                a = cfg.cone_half_corridor_alpha
                self._cone_half_m = (1.0 - a) * self._cone_half_m + a * half_lidar
                self._cone_half_m = max(
                    cfg.cone_half_corridor_min_m,
                    min(cfg.cone_half_corridor_max_m, self._cone_half_m),
                )
            la = min(cfg.cone_lookahead_m, cone.lidar_gate_depth_m)
            la = max(cfg.cone_lookahead_min_m, la)
            # lidar lateral 은 좌측+ — pure_pursuit 의 y(좌측+)에 그대로 넣는다
            return pure_pursuit_steer(
                la, cone.lidar_gate_lateral_m, max_steer=cfg.max_steer
            )

        # 전방주시거리 = 상한과 '관측된 게이트 깊이' 중 작은 쪽 (하한 보장)
        # 깊이는 BEV 세로축이라 세로 축척으로 환산한다.
        px_per_m_fwd = max(1.0, cfg.px_per_m_fwd)
        lookahead = cfg.cone_lookahead_m
        if cone.gate_valid and cone.gate_depth_px > 1.0:
            lookahead = min(lookahead, cone.gate_depth_px / px_per_m_fwd)
        lookahead = max(cfg.cone_lookahead_min_m, lookahead)

        if cone.gate_valid:
            # 게이트 중앙(px)을 차량 기준 목표점(m)으로 바꿔 퓨어 퍼수트로 간다.
            lateral_m = cone.gate_center_px / px_per_m
            return pure_pursuit_steer(
                lookahead, -lateral_m, max_steer=cfg.max_steer
            )

        if cone.left_cone_px > 0 or cone.right_cone_px > 0:
            # 단측: 보이는 줄에서 반폭만큼 떨어진 '가상 게이트 중앙'을 향한다.
            total = float(cone.left_cone_px + cone.right_cone_px)
            bias = (cone.left_cone_px - cone.right_cone_px) / max(1.0, total)
            # bias>0 = 왼쪽에 콘이 많다 → 통로는 오른쪽 → 목표도 오른쪽
            lateral_m = bias * self._cone_half_m
            return pure_pursuit_steer(
                lookahead, -lateral_m, max_steer=cfg.max_steer
            )

        # 콘을 놓쳤다. 차선주행으로 버틴다.
        return self._lane_steer(obs)


__all__ = ["PolicyConfig", "DrivePolicy"]

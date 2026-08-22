"""라바콘 구간 연결부 — kmu_race 이식본의 콘 경로를 yolop_drive 에 잇는다.

데이터 흐름:
  Cone_Perception(/cone/reading, 라이다) → ConeLink(진입/탈출 + 조향) →
  yolop_drive 의 모터 발행 경로 (armed/enabled 게이트는 그쪽이 쥔다).

조향·속도 수식과 모든 임계값은 race_last/kmu_race/ 사본(2026-08-21 새벽
실측 튜닝, 원본과 sha 동일)에서 그대로 가져온다 — 여기서 숫자를 새로
만들지 않는다. 진입/탈출 판정은 race_fsm.py 의 LANE_DRIVE↔CONE_DRIVE
블록을 문자 그대로 옮긴 것이다.

카메라 주황(orange_px)은 아직 미연결이라 항상 0 으로 들어간다. 친구 코드가
'라이다만으로도 진입한다'(0821 밤) 경로를 이미 갖고 있어 라이다 단독으로
진입·유지·탈출이 전부 동작한다. 카메라 게이트 부재의 영향도 같다 —
policy 는 라이다 게이트를 1순위로 쓴다 (types.ConeReading 주석).
"""
import math

from .kmu_race.core.policy import DrivePolicy
from .kmu_race.core.race_context import RaceContext
from .kmu_race.core.race_fsm import FSMConfig, Mode, _Debouncer
from .kmu_race.core.safety import SafetyDecision, SafetyLevel
from .kmu_race.core.types import ConeReading, Observation

READY = SafetyDecision(SafetyLevel.OK, "", True)


class ConeLink:
    def __init__(self, logger, approach_depth_m=1.2):
        self.approach_depth_m = float(approach_depth_m)
        self._log = logger
        self._policy = DrivePolicy()
        self._ctx = RaceContext()
        self._fsm = FSMConfig()
        self._in = _Debouncer(self._fsm.cone_enter_frames)
        self._out = _Debouncer(self._fsm.cone_exit_frames)
        self._active = False
        # 통로가 앞에 보이는 상태 (진입 전) — 노드가 접근 감속에 쓴다
        self.approach = False
        self._entered_at = 0.0
        self._gate_last_valid = 0.0
        self._started = False

    @property
    def active(self) -> bool:
        return self._active

    @staticmethod
    def reading_from(d, now):
        """Cone_Perception 의 JSON 한 건 → ConeReading (라이다 필드만)."""
        if not d:
            return ConeReading(), 9.9
        age = max(0.0, now - float(d.get('stamp', 0.0)))
        pts = tuple(
            (float(x), float(y)) for x, y in d.get('pts', ()) or ())
        nl = float(d.get('near_left', 9.9))
        nr = float(d.get('near_right', 9.9))
        return ConeReading(
            detected=int(d.get('cones', 0)) > 0,
            lidar_gate_valid=bool(d.get('valid', False)),
            lidar_gate_depth_m=float(d.get('depth', 0.0)),
            lidar_gate_width_m=float(d.get('width', 0.0)),
            lidar_gate_lateral_m=float(d.get('lateral', 0.0)),
            lidar_cones=int(d.get('cones', 0)),
            lidar_near_left_m=(math.inf if nl >= 9.8 else nl),
            lidar_near_right_m=(math.inf if nr >= 9.8 else nr),
            lidar_cone_pts=pts,
            lidar_gate_age_s=age,
        ), age

    def tick(self, now, d, ego_mps, camera_ok):
        """제어 틱마다 부른다. 콘 구간이면 (steer, speed_units), 아니면 None."""
        if not self._started:
            self._started = True
            # time_pressure 가 monotonic 원점 기준으로 폭주하지 않게 지금을
            # 출발 시각으로 잡는다. 콘 정책에서 이 값은 감속 보수성에만 쓰인다.
            self._ctx.race_started_at = now
        cone, age = self.reading_from(d, now)
        # 게이트가 '가까울 때만' 접근 감속 — 멀리 보이는 콘만으로 출발
        # 가속을 끊던 오발동 수정 (0822 motorlog: 명령이 2.16↔0.72 왕복)
        self.approach = bool(
            age < 0.6 and cone.lidar_cones >= 2
            and cone.lidar_gate_valid
            and cone.lidar_gate_depth_m <= self.approach_depth_m)
        obs = Observation(now=now, cone=cone, ego_speed_mps=max(0.0, ego_mps),
                          camera_ok=camera_ok, lidar_ok=age < 0.6)
        cfg = self._fsm

        if not self._active:
            # ── LANE→CONE 진입 (race_fsm.py 블록 그대로) ──
            cone_hit = cone.orange_px >= cfg.cone_min_orange_px
            gate_seen = cone.gate_valid or cone.lidar_gate_valid
            if cfg.cone_enter_requires_gate:
                cone_hit = cone_hit and gate_seen
            lidar_solid = (
                cone.lidar_cones >= cfg.cone_enter_min_lidar
                and cone.lidar_gate_valid
            )
            cone_hit = cone_hit or lidar_solid
            if cone.lidar_cones < cfg.cone_enter_require_lidar:
                cone_hit = False
            if not self._in.update(cone_hit, now):
                return None
            self._out.reset()
            self._active = True
            self._entered_at = now
            self._gate_last_valid = now
            # 새 구간 — 지난 구간 상태를 물려받지 않는다 (policy.update 의
            # CONE 진입 초기화 분기를 타도록 _prev_mode 를 비운다)
            self._policy.reset()
            self._policy._prev_mode = None
            self._log.info(
                '라바콘 구간 진입 — 콘 %d개, 게이트 %s' %
                (cone.lidar_cones, '유효' if cone.lidar_gate_valid else '무효'))

        # ── CONE 유지/탈출 (race_fsm.py 블록 그대로) ──
        if cone.gate_valid or cone.lidar_gate_valid:
            self._gate_last_valid = now
        gate_lost = (
            (now - self._gate_last_valid) >= cfg.cone_gate_lost_exit_s)
        lidar_busy = cone.lidar_cones >= cfg.cone_exit_min_lidar
        gone = (cone.orange_px < cfg.cone_min_orange_px) and not lidar_busy
        if now - self._entered_at > cfg.cone_max_dwell_s:
            gate_lost = True
        if gate_lost or self._out.update(gone, now):
            self._in.reset()
            self._active = False
            self._log.info(
                '라바콘 구간 이탈 (%s) — 차선 주행 복귀'
                % ('gate lost' if gate_lost else 'cones gone'))
            return None

        cmd = self._policy.update(Mode.CONE_DRIVE, obs, self._ctx, READY)
        return float(cmd.steer), float(cmd.speed)

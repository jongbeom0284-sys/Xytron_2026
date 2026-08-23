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

from . import cone_wall
from .kmu_race.core.policy import DrivePolicy
from .kmu_race.core.race_context import RaceContext
from .kmu_race.core.race_fsm import FSMConfig, Mode, _Debouncer
from .kmu_race.core.safety import SafetyDecision, SafetyLevel
from .kmu_race.core.types import ConeReading, Observation

READY = SafetyDecision(SafetyLevel.OK, "", True)

# 라이다 관측이 이보다 오래되면 콘 구간으로 보지 않는다. Cone_Perception 이
# 꺼지면 /cone/reading 이 끊기는데, 마지막 값을 붙들고 진입하던 문제를 막는다.
STALE_SEC = 0.6


class ConeLink:
    def __init__(self, logger, approach_depth_m=1.2,
                 wall_steer=True, wall_gain=1.5,
                 enter_frames=1, entry_speed_units=0.0,
                 entry_stable_frames=2,
                 entry_weak_timeout_sec=0.25):
        self.approach_depth_m = float(approach_depth_m)
        # 조향 근거를 벽 추종으로 바꾼다 (0823). 끄면 예전 policy 조향.
        self.wall_steer = bool(wall_steer)
        self.wall_gain = float(wall_gain)
        self._wall_prev = None      # 조향 변화율 제한용 직전 값
        self.wall_basis = ''        # 상태 표시용
        self._log = logger
        self._policy = DrivePolicy()
        self._ctx = RaceContext()
        self._fsm = FSMConfig()
        # 진입 디바운스. 기본 1 = 라이다가 콘을 본 첫 프레임에 바로 넘어간다
        # (0823 저녁 요구: "A 가 콘을 인식하면 바로 콘 모드"). 게이트 유효·
        # 콘 개수 조건은 그대로라, 콘을 못 본 상태로 들어가지는 않는다.
        self._in = _Debouncer(max(1, int(enter_frames)))
        # 콘 모드로 넘어가는 순간 속도 램프를 0 이 아니라 이 값에서 시작한다.
        # 0 이면 예전처럼 0 에서 램프 (실측: 2.16 -> 0.29 m/s 까지 곤두박질).
        self.entry_speed_units = float(entry_speed_units)
        # 콘 모드 진입 직후에는 한쪽 벽만 보이는 임시 목표가 자주 나온다.
        # 직전 차선 조향을 유지한 채 양쪽 벽의 서로 다른 라이다 프레임을
        # 확인하고, 확인이 늦어져도 정지하지 않고 기존 변화율 제한으로 넘긴다.
        self.entry_stable_frames = max(1, int(entry_stable_frames))
        self.entry_weak_timeout_sec = max(
            0.0, float(entry_weak_timeout_sec))
        self._entry_settling = False
        self._entry_stable_count = 0
        self._entry_last_stamp = None
        # 카메라 주황 트리거(A)가 울리면 True. 그 뒤 라이다 관측이 오는 첫 틱에
        # **콘 개수·게이트 유효를 따지지 않고** 콘 모드로 들어간다 (0823 요구).
        # 그 조건들을 기다리다 0.7초를 흘려보낸 실측이 있었다
        # (192730 세션 f262~f268: 콘 1~3개·게이트 무효로 대기, 5개에서야 진입).
        self._force_enter = False
        # 이 구간을 이미 빠져나왔는가. True 인 동안에는 approach(진입 전 감속)를
        # 켜지 않는다 — 빠져나온 통로의 콘 쌍이 라이다에 계속 보여서 approach 가
        # 되살아나고, 그때마다 속도가 27 -> 9 units 로 떨어졌다 (0823 실측:
        # 214410 세션 f319~324, f331~340). 다음 주황 트리거나 재진입에서 풀린다.
        self._exited = False
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

    def request_enter(self) -> None:
        """다음 관측에서 조건 없이 콘 모드로 들어간다 (카메라 주황 트리거용)."""
        self._force_enter = True
        self._exited = False        # 새 구간이다 — 접근 감속을 다시 허용한다

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

    def _start_entry_handoff(self, now, handoff_steer):
        """직전 차선 조향에서 라바콘 조향으로 연속적으로 인계한다."""
        steer = 0.0 if handoff_steer is None else float(handoff_steer)
        if not math.isfinite(steer):
            steer = 0.0
        self._wall_prev = max(-cone_wall.MAX_STEER,
                              min(cone_wall.MAX_STEER, steer))
        self._entry_settling = self.wall_steer
        self._entry_stable_count = 0
        self._entry_last_stamp = None
        self._entry_started_at = float(now)

    def _reset_entry_handoff(self):
        self._entry_settling = False
        self._entry_stable_count = 0
        self._entry_last_stamp = None

    def tick(self, now, d, ego_mps, camera_ok, handoff_steer=None):
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
        # 이미 빠져나온 구간이면 접근 감속을 켜지 않는다 (위 _exited 주석의 근거).
        if self._exited and not self._active:
            self.approach = False
        obs = Observation(now=now, cone=cone, ego_speed_mps=max(0.0, ego_mps),
                          camera_ok=camera_ok, lidar_ok=age < STALE_SEC)
        cfg = self._fsm

        # 관측이 오래됐다 — 콘 인식이 꺼졌거나 라이다가 끊긴 것이다.
        # 옛 관측으로 구간을 열면 출발선에서도 진입한다 (0823 실측).
        if age >= STALE_SEC:
            if self._active:
                self._active = False
                self._wall_prev = None
                self._exited = True
                self._reset_entry_handoff()
                self._log.info('콘 관측 끊김 — 차선 주행 복귀')
            self._in.reset()
            self._out.reset()
            self.approach = False
            return None

        if not self._active:
            # ── 카메라 주황 트리거로 들어온 강제 진입 (0823 요구) ──
            # 라이다 조건을 보지 않는다. 관측이 신선하기만 하면(위 STALE_SEC
            # 검사 통과) 바로 넘어간다. 콘을 못 보는 상태로 들어가면 policy 는
            # 차선을 보지 않고 직진으로 풀리며, 게이트가
            # cone_gate_lost_exit_s 동안 안 잡히면 스스로 빠져나온다.
            if self._force_enter:
                self._force_enter = False
                self._exited = False
                self._out.reset()
                self._in.reset()
                self._active = True
                self._entered_at = now
                self._gate_last_valid = now
                self._policy.reset()
                self._policy._prev_mode = None
                if self.entry_speed_units > 0.0:
                    self._policy.speed.prime(self.entry_speed_units, now)
                self._start_entry_handoff(now, handoff_steer)
                self._log.info(
                    '주황 트리거 — 조건 없이 라바콘 모드 시작 (콘 %d개, 게이트 %s)'
                    % (cone.lidar_cones,
                       '유효' if cone.lidar_gate_valid else '무효'))
                cmd = self._policy.update(Mode.CONE_DRIVE, obs, self._ctx, READY)
                return self._steer_speed(now, d, cone, age, cmd)

            # ── 한 번 빠져나온 뒤에는 라이다만 보고 다시 들어가지 않는다 ──
            # 재진입 경로는 위 _force_enter 하나뿐이고, 그 트리거는 초록불
            # 확정으로만 재무장된다. 즉 재진입 조건 = 초록불.
            # 콘 구간 밖에서도 게이트는 성립한다: 230555 f293/f297 은 곡선
            # 바깥 벽의 좌·우 반사가 '폭 0.82m 통로'로 잡혀 f299 에 콘 모드가
            # 켜졌다 (신호등 램프 0개, 신호잠금은 줄기만 했다). 225657 은
            # 정지차량 옆 라바콘으로 같은 일이 나 -40 풀락으로 이탈했다.
            if self._exited:
                self._in.reset()
                return None

            # ── LANE→CONE 진입 (race_fsm.py 블록 그대로) ──
            # 진입에는 **언제나** 주황이 있어야 한다 (0823 밤 요구).
            # 예전에는 여기 `cone_hit = cone_hit or lidar_solid` 가 있어서
            # 주황 0px 이어도 라이다가 콘 4개 + 유효 게이트만 보면 들어갔다.
            # 그것이 위 두 사고의 경로다. 지웠다.
            # 주의: orange_px 는 아직 미연결이라 항상 0 이다(파일 상단 주석).
            #       따라서 지금 실제로 살아 있는 진입 경로는 _force_enter
            #       (카메라 주황 트리거) 하나뿐이다. 이 블록은 orange_px 가
            #       연결되면 그때 살아난다.
            cone_hit = cone.orange_px >= cfg.cone_min_orange_px
            gate_seen = cone.gate_valid or cone.lidar_gate_valid
            if cfg.cone_enter_requires_gate:
                cone_hit = cone_hit and gate_seen
            if cone.lidar_cones < cfg.cone_enter_require_lidar:
                cone_hit = False
            if not self._in.update(cone_hit, now):
                return None
            self._out.reset()
            self._active = True
            self._exited = False
            self._entered_at = now
            self._gate_last_valid = now
            # 새 구간 — 지난 구간 상태를 물려받지 않는다 (policy.update 의
            # CONE 진입 초기화 분기를 타도록 _prev_mode 를 비운다)
            self._policy.reset()
            self._policy._prev_mode = None
            # reset 은 속도 램프를 0 으로 되돌린다. 그대로 두면 콘 진입마다
            # 거의 정지 수준(실측 0.29~0.40 m/s)까지 떨어졌다 다시 올라온다.
            # 진입 속도를 미리 채워 그 곤두박질을 없앤다 (0823 저녁 요구).
            if self.entry_speed_units > 0.0:
                self._policy.speed.prime(self.entry_speed_units, now)
            self._start_entry_handoff(now, handoff_steer)
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
            self._wall_prev = None
            self._exited = True
            self._reset_entry_handoff()
            self._log.info(
                '라바콘 구간 이탈 (%s) — 차선 주행 복귀'
                % ('gate lost' if gate_lost else 'cones gone'))
            return None

        cmd = self._policy.update(Mode.CONE_DRIVE, obs, self._ctx, READY)
        return self._steer_speed(now, d, cone, age, cmd)

    def _steer_speed(self, now, d, cone, age, cmd):
        """조향만 벽 추종으로 교체한다. 속도는 policy 가 낸 값을 그대로 쓴다 —
        콘 구간 감속 로직이 거기 있고 실주행에서 문제가 없었다."""
        aim = d.get('wall_aim') if d else None
        self.wall_basis = (d.get('wall_basis') or '') if d else ''
        released_now = False

        if self._entry_settling:
            left_count = int(d.get('wall_left', 0) or 0) if d else 0
            right_count = int(d.get('wall_right', 0) or 0) if d else 0
            strong = bool(
                aim is not None and age < STALE_SEC
                and self.wall_basis in ('both', 'CENTER_OFFSET_both')
                and left_count >= 2 and right_count >= 2)

            # 제어는 20Hz, 라이다는 약 10Hz라 같은 관측을 두 번 받을 수 있다.
            # stamp가 달라진 실제 라이다 프레임만 안정 프레임으로 센다.
            stamp = d.get('stamp') if d else None
            if stamp != self._entry_last_stamp:
                self._entry_last_stamp = stamp
                self._entry_stable_count = (
                    self._entry_stable_count + 1 if strong else 0)

            stable = self._entry_stable_count >= self.entry_stable_frames
            timed_out = (
                now - self._entry_started_at >= self.entry_weak_timeout_sec)
            if not stable and not timed_out:
                # 불완전한 첫 목표로 꺾지 않는다. 속도 정책도 진입 속도에서
                # 얼려 두어 확인이 끝난 순간 내부 속도가 갑자기 뛰지 않게 한다.
                speed = float(cmd.speed)
                if self.entry_speed_units > 0.0:
                    self._policy.speed.prime(self.entry_speed_units, now)
                    speed = min(speed, self.entry_speed_units)
                return float(self._wall_prev or 0.0), speed

            self._entry_settling = False
            released_now = True
            if self.entry_speed_units > 0.0:
                self._policy.speed.prime(self.entry_speed_units, now)
            if stable:
                self._log.info(
                    '라바콘 양쪽 경계 %d프레임 확인 — 조향 인계 시작'
                    % self._entry_stable_count)
            else:
                self._log.info(
                    '라바콘 진입 %.2f초 — 불완전 경계를 '
                    '변화율 제한으로 인계'
                    % self.entry_weak_timeout_sec)

        if self.wall_steer and aim is not None and age < 0.6:
            steer = cone_wall.steer_for(float(aim), self.wall_gain,
                                        self._wall_prev)
            self._wall_prev = steer
            speed = float(cmd.speed)
            if released_now and self.entry_speed_units > 0.0:
                speed = min(speed, self.entry_speed_units)
            return float(steer), speed
        self._wall_prev = float(cmd.steer)
        speed = float(cmd.speed)
        if released_now and self.entry_speed_units > 0.0:
            speed = min(speed, self.entry_speed_units)
        return float(cmd.steer), speed

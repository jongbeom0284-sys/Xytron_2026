"""경주 상태 기계 — 규정 p.5~p.38 의 미션 순서를 그대로 옮긴 것.

트랙 형상 (규정 p.6/p.7/p.37 도면에서 유도):
  둥근 사각 루프 + 가운데 세로 연결로 = θ 자 형태. 주행은 **반시계**.
  (윗변은 서쪽으로, 좌변은 남쪽으로, 아랫변은 동쪽으로, 우변(S자)은 북쪽으로)

      ┌──────────── ← ────────────┐
      │            신호등 ┬ Gate   │
      ↓            지름길 │        ↑   ← 우변은 S자 구간
      │  라바콘/고정장애물/ │        │
      │      방해차량      │        │
      └──────────── → ────┴────────┘

  지름길 = 윗변에서 **좌회전**해 가운데 세로길로 진입 → 아랫변에서 다시 좌회전.
  이 경로는 좌측 구역(라바콘·고정장애물·방해차량)을 통째로 건너뛴다.
  FSM 이 담당하는 건 **진입 좌회전 한 번**뿐이다. 세로길 주행과 아랫변 합류
  코너는 평범한 차선주행이라 LANE_DRIVE 가 그대로 처리한다.

설계 원칙:
  · 미션 진입은 '스크립트'가 아니라 '반응'이다. 랩마다 같은 미션을 다시 만나고,
    지름길을 타면 아예 안 만나기 때문에 순서를 하드코딩하면 깨진다.
  · 기동 상태(CONE_DRIVE/AVOID/OVERTAKE/SHORTCUT)는 **커밋**된다. 일단 들어가면
    자기 종료 조건으로만 나온다. 안 그러면 프레임마다 상태가 진동한다.
  · 안전은 항상 최우선이되, 이 대회에서 STOP 은 거의 최후수단이다 (safety.py 참조).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .race_context import RaceContext
from .safety import SafetyDecision
from .types import Observation, SignalState


class Mode(str, Enum):
    INIT = "INIT"
    WAIT_GREEN = "WAIT_GREEN"        # 신호등 앞 정지, 파란불 대기
    LANE_DRIVE = "LANE_DRIVE"        # 기본 차선 주행
    CONE_DRIVE = "CONE_DRIVE"        # 라바콘 구간 (p.23~25)
    AVOID_STATIC = "AVOID_STATIC"    # 고정장애물 = 고장난 차량 (p.30)
    OVERTAKE = "OVERTAKE"            # 방해차량 추월 (p.31~34)
    SHORTCUT = "SHORTCUT"            # 지름길 진입 좌회전 (p.35)
    FINISH = "FINISH"                # 3바퀴 완료
    STOP = "STOP"                    # 안전 정지 — 원인이 걷히면 복귀한다 (p.21)


MANEUVER_MODES = frozenset(
    {Mode.CONE_DRIVE, Mode.AVOID_STATIC, Mode.OVERTAKE, Mode.SHORTCUT}
)


@dataclass(frozen=True)
class Transition:
    source: Mode
    target: Mode
    reason: str

    @property
    def changed(self) -> bool:
        return self.source is not self.target


@dataclass
class FSMConfig:
    # ── 출발 ──
    # 규정 p.20: 빨간불에 출발하면 벌초 10초. 오검출 한 프레임에 10초를 날릴 수
    # 없으므로 출발 판정만 유독 보수적으로 잡는다.
    # 0817 하향(4/0.12 → 3/0.06): 출발 거리(4~5m)에서 검출 스트릭이 3카메라
    # 프레임(0.066s@30fps)까지 짧다는 것이 실차 bag 실측 — 4프레임 요구는
    # 리플레이에서 출발을 138s(1랩 접근)까지 지연시켰다. 0.06s AND 3연속은
    # LapTracker confirm_s 와 같은 디바운스 표준이고, 실차 50Hz 틱으로는
    # 4틱(0.06s) — 오검 방어는 상승엣지·RED 거부·링 게이트가 겹겹이 맡는다.
    green_confirm_frames: int = 3
    green_confirm_seconds: float = 0.06
    # 출발 판정 보조: 출발선 거리(3~4m)에서는 램프가 작아(반지름 3~7px)
    # 4구 원 분류가 거의 안 된다 — run4 bag 실측 11,056프레임 중 GREEN 분류
    # 2프레임뿐, FSM 리플레이에서 출발 실패. 반면 채도 초록 블롭은 green-on
    # 순간부터 안정적으로 잡히므로(36.7s~), 분류가 UNKNOWN 이어도 블롭이
    # 이만큼 보이면 파란불로 인정한다. RED 로 분류된 프레임은 예외(빨간불 우선).
    # 0817 재산출: kobot 새 카메라(과노출)에서 출발 대기 램프 실측 6~8px → 5.
    start_go_min_span_px: int = 5
    # 상승엣지 요구: 블롭이 '빨간 구간 동안 없다가 새로 나타난' 것이어야 한다.
    # 정적 오검 블롭(실측 최악 14px — 조명·유도등)은 대기 내내 켜져 있어
    # 어두운 구간이 없으므로 여기서 걸러진다. RED 거부만으로는 못 막는다 —
    # 출발 거리에서는 4구 분류 자체가 거의 안 되기 때문(RED 도 안 나온다).
    # 0817 실사고로 1.0→3.0: 원거리 출발 경로는 판별 마진이 작아(7px, 끼
    # +9.5 vs 임계 6) 깜빡임이 있고, 1s 공백이면 '가짜 어두운 구간'이 만들
    # 어져 이미 켜진 초록불에서 재출발이 났다. 실제 빨간불 구간은 수 초~
    # 수십 초라 3s 는 실전에서 항상 충족된다.
    start_dark_min_s: float = 3.0

    # ── 라바콘 ──
    cone_enter_frames: int = 3
    cone_exit_frames: int = 14          # 콘이 사라졌다고 확신할 때까지
    cone_min_orange_px: int = 900
    # 게이트가 이 시간 이상 안 생기면 구간이 끝난 것으로 보고 이탈한다.
    # 벽면 보관 콘 무더기는 orange_px 를 계속 채우지만 게이트를 못 만들어
    # (run4 실측: 660프레임 중 게이트 5프레임) 이탈 디바운스를 굶긴다 —
    # CONE_DRIVE 에 잔류하면 전방 타겟 회피가 꺼진 채 장애물로 직진한다.
    # 0818 실차: 슬라럼 안에서 게이트 성립률이 최대 37%(BEV 원근 왜곡으로
    # 콘이 얼룩이 됨)라 1.2s 는 구간 한복판에서 튕겨 나가게 만들었다 —
    # 나가면 콘에 오염된 차선 추종으로 떨어져 실패. 4.0s 로 늘리고, 그 사이
    # 단측 가상 게이트가 조향을 맡는다. 무더기(벽면 보관) 이탈은 여전히 된다.
    cone_gate_lost_exit_s: float = 4.0
    # 규정 p.25: 라바콘 구간 1분 내 통과 실패 = 실격. 이 시간을 넘기면
    # 로그로 경고한다 (자동 행동 변경은 없음 — 어차피 계속 전진 중이다).
    cone_section_warn_s: float = 30.0
    # 진입에는 '좌우 두 줄 게이트 성립'을 요구한다. 대회장은 못 치우고 벽에
    # 보관 콘 무더기가 그대로 있는데(run4 실측: orange>900 이 660프레임,
    # 그중 게이트 성립은 5프레임뿐), 무더기는 한쪽 덩어리라 게이트를 못
    # 만든다. 규정 p.23~25 의 두 줄 배치가 곧 미션 구간의 시그니처다.
    # 구간 '이탈' 판정은 기존대로 orange_px 만 본다 (구간 중 한 줄이 잠깐
    # 끊겨도 머물러야 하므로).
    cone_enter_requires_gate: bool = True

    # ── 전방 타겟 (고정장애물 / 방해차량) ──
    target_trigger_dist_m: float = 1.45
    target_enter_frames: int = 2
    # 정지/주행 판별: 상대 접근속도가 자차속도의 이 비율보다 '느리면' 주행 중이다.
    # 정지 물체는 vx ≈ -v_ego 로 관측되고, 같은 방향으로 달리는 차는 그보다 작다.
    moving_ratio: float = 0.6
    moving_min_ego_mps: float = 0.35    # 이보다 느리면 판별 자체가 무의미

    # ── 지름길 ──
    # 좌회전 신호를 본 뒤 이 시간(s) 안에 분기점에 닿으면 유효한 것으로 본다.
    # 겐트리 목격 프레임에서만 신호를 읽으면 한 프레임 놓쳐 기회를 날릴 수 있어
    # 짧은 래치를 둔다.
    left_signal_latch_s: float = 4.0
    # 좌회전 래치의 근접 게이트: 겐트리 블롭(gantry_span_px)이 이만큼은 보여야
    # GREEN_LEFT 를 래치한다. 채도 게이트가 가짜 GREEN_LEFT 를 거의 없앴지만
    # (run4 bag 0건), 지름길 오발동은 곧 차선이탈이라 방어층을 하나 더 둔다 —
    # 겐트리 실물이 안 보이는 곳에서 읽힌 좌회전 신호는 믿지 않는다.
    # 0817 재산출: kobot 새 카메라에서 램프 폭 자체가 6~8px 라 10 은 영원히
    # 안 넘는다 (지름길 기회 상실). 실측 하한으로 내린다.
    left_latch_min_span_px: int = 5
    # 진입 좌회전 완료 판정 요 변화량 [deg]
    shortcut_turn_deg: float = 72.0
    # 요가 안 돌아도 이 시간이 지나면 완료로 보고 차선주행에 넘긴다 (교착 방지)
    shortcut_timeout_s: float = 3.2

    # ── 안전 정지 복귀 ──
    # 규정 p.21: 멈춘 뒤 1분 내 재개 못하면 실격. 그래서 STOP 은 터미널이
    # 아니다 — 정지 원인(카메라 사망)이 걷히면 차선주행으로 복귀한다.
    # 깜빡이는 회복에 낚이지 않도록 연속 프레임 + 최소 지속시간을 요구한다.
    stop_recover_frames: int = 5
    stop_recover_seconds: float = 0.5


class _Debouncer:
    """연속 N프레임 + 최소 지속시간을 만족해야 True."""

    def __init__(self, frames: int, seconds: float = 0.0) -> None:
        self.frames = max(1, frames)
        self.seconds = max(0.0, seconds)
        self.reset()

    def reset(self) -> None:
        self._count = 0
        self._since: Optional[float] = None

    def update(self, hit: bool, now: float) -> bool:
        if not hit:
            self.reset()
            return False
        if self._count == 0:
            self._since = now
        self._count += 1
        return self._count >= self.frames and (now - self._since) >= self.seconds


class RaceFSM:
    """관측 → 상태. 제어 명령은 만들지 않는다 (그건 policy 계층 몫)."""

    # FINISH 만 터미널이다. STOP 은 복귀 가능 — p.21 의 1분 룰 때문에
    # '한 번 서면 끝'인 상태를 만들면 그 자체가 실격 경로가 된다.
    TERMINAL = frozenset({Mode.FINISH})

    def __init__(self, config: Optional[FSMConfig] = None) -> None:
        self.config = config or FSMConfig()
        self.state = Mode.INIT

        c = self.config
        self._green = _Debouncer(c.green_confirm_frames, c.green_confirm_seconds)
        self._cone_in = _Debouncer(c.cone_enter_frames)
        self._cone_out = _Debouncer(c.cone_exit_frames)
        self._target_in = _Debouncer(c.target_enter_frames)
        self._stop_recover = _Debouncer(c.stop_recover_frames, c.stop_recover_seconds)

        # 좌회전 신호를 마지막으로 확실히 본 시각
        self._left_seen_at: float = -1e9
        # SHORTCUT 진입 시점의 요와 시각
        self._shortcut_yaw0: float = 0.0
        self._shortcut_t0: float = 0.0
        # 기동 상태가 자기 종료를 보고하는 통로 (policy 가 써 준다)
        self.maneuver_done: bool = False
        # ── 시운전 스위치 (race_node 파라미터로 설정, 본선은 둘 다 기본값) ──
        # manual_go: 외부(/kmu_race/go)에서 출발 명령이 오면 다음 틱에 출발.
        #   0817 첫 자율주행: 원거리(8~10m) 출발 판정 마진 부족으로 오출발 —
        #   차선 추종 검증을 위해 출발만 수동 트리거로 대체하는 통로.
        self.manual_go: bool = False
        # manual_start_mode=True 면 자동 초록 판정으로는 절대 출발하지 않는다
        # (0817: 수동 출발 시운전 중 오검 자동 출발 재발 방지).
        self.manual_start_mode: bool = False
        # maneuvers_enabled=False: 전방 타겟(회피/추월) 트리거 잠금. 붐비는
        #   연습장(사람·의자가 트랙 가장자리 0.4m 안)에서 회피 루프에 갇히는
        #   것을 막고 차선 추종만 본다. 라바콘·지름길·랩 카운팅은 그대로.
        self.maneuvers_enabled: bool = True
        # shortcut_enabled=False: 지름길 트리거 잠금 (0818 시운전 — 바닥 흰
        #   물체 오검으로 가짜 GREEN_LEFT→3초 좌풀락 실사고. 실차 검증 전까지)
        self.shortcut_enabled: bool = True
        # 출발 상승엣지: 블롭 없는 구간 관측 (start_dark_min_s)
        self._dark_since: Optional[float] = None
        self._dark_phase_seen: bool = False
        # 지름길 판단 윈도우 (목격 확정 원샷이 아니라 잠시 열린다)
        self._decision_until: float = -1e9
        # 라바콘 구간 상태
        self._cone_gate_last_valid: float = 0.0
        self._cone_entered_at: float = 0.0
        self._cone_warned: bool = False

    # ──────────────────────────────────────────────────────────────
    def step(
        self,
        obs: Observation,
        ctx: RaceContext,
        safety: SafetyDecision,
    ) -> Transition:
        if self.state in self.TERMINAL:
            return self._stay("terminal")

        if safety.must_stop:
            ctx.stop_reason = safety.reason
            # 복귀 디바운스를 처음부터 다시 세게 한다 — 깜빡 회복 후 재사망이
            # 이전 카운트를 이어받으면 너무 이르게 복귀한다.
            self._stop_recover.reset()
            return self._to(Mode.STOP, safety.reason, ctx, obs.now)

        # ── 정지 복귀 (규정 p.21 — 1분 내 재개 못하면 실격) ────────────
        # 여기 도달했다면 must_stop 은 이미 걷힌 상태다. 카메라가 안정되면
        # 차선주행으로 복귀한다. 정지 전의 기동 상태는 내부 이력이 낡았으므로
        # 되살리지 않는다 — 조건이 여전하면 LANE_DRIVE 가 반응형으로 재진입한다.
        if self.state is Mode.STOP:
            # 복귀 신호는 safety 판정과 합의한다 — 원시 camera_ok 스트릭을
            # 요구하면 카메라가 ~2Hz 로 절뚝일 때(스트릭 최대 0.35s < 0.5s)
            # 복귀가 영영 안 되고 must_stop 재발령도 없어 STOP 에 고착된다
            # (정지 1분 = 실격). safety 가 STOP 을 거둔 상태가 안정되면
            # 복귀하고, 카메라가 여전히 부실하면 DEGRADED 크립이 받는다.
            if self._stop_recover.update(not safety.must_stop, obs.now):
                self._stop_recover.reset()
                ctx.log(obs.now, f"resumed after stop ({ctx.stop_reason})")
                ctx.stop_reason = ""
                return self._to(Mode.LANE_DRIVE, "inputs recovered", ctx, obs.now)
            return self._stay("stopped - waiting recovery")

        # ── 좌회전 신호 래치 ──
        # 분기점에 닿기 전 몇 프레임 안에 본 좌회전도 유효하게 인정한다.
        # 단, 겐트리 블롭이 실제로 보이는 프레임에서만 — 겐트리가 없는 곳에서
        # 읽힌 GREEN_LEFT 는 조명 오검일 가능성이 높고, 그걸 래치하면
        # 지름길 오발동(차선이탈)으로 직결된다.
        # camera_ok 게이트: 카메라가 죽으면 race_node 의 관측이 마지막 프레임에
        # 동결되는데, 그 프레임이 GREEN_LEFT 면 래치가 매 틱 갱신되어 4초
        # 신선창이 정전 시간만큼 늘어난다. 신선한 프레임에서만 래치한다.
        if (
            obs.camera_ok
            and (
                obs.signal.state is SignalState.GREEN_LEFT
                or obs.signal.watcher_green_left
            )
            and obs.signal.gantry_span_px >= self.config.left_latch_min_span_px
        ):
            self._left_seen_at = obs.now
            # (0817 폐기 기록) GREEN_LEFT 목격이 결정 창을 재개방하는 변경을
            # 시도했다가 되돌렸다 — 주행 중 가짜 GREEN_LEFT 페어가 실측돼
            # (진짜 겐트리 접근 순간 포함) 창 밖 오검을 커밋 위험으로 승격
            # 시키는 꼴이었다. 창은 목격(요 보충 포함) 앵커를 유지한다.
            # 요 보충 조기 발화로 창이 이르게 닫히는 문제는 좌회전 신호
            # 판별을 재검증한 뒤 창 길이 조정으로 풀 것 (전수조사백로그).

        if self.state is Mode.INIT:
            if safety.inputs_ready:
                self._green.reset()
                return self._to(Mode.WAIT_GREEN, "inputs ready", ctx, obs.now)
            return self._stay(safety.reason or "waiting inputs")

        # ── 출발 대기 ──────────────────────────────────────────────
        if self.state is Mode.WAIT_GREEN:
            # 이 시점에 차는 신호등 바로 앞에 서 있다. 랩 트래커의 '목격 1'이
            # 여기서 잡히고, 그 결과 current_lap 이 1이 된다.
            ctx.lap.update(
                gantry_span_px=obs.signal.gantry_span_px,
                now=obs.now,
                yaw_deg=obs.yaw_deg,
            )
            # 출발 전 목격은 의미상 1회뿐 — 캡 + 요 기준선 동기 (0817 실측:
            # 이송·대기 중 유령 목격 3건과 이송 회전 +235° 오염 차단).
            ctx.lap.hold_start_baseline(yaw_deg=obs.yaw_deg)
            if not safety.inputs_ready:
                self._green.reset()
                return self._stay(safety.reason or "waiting inputs")
            # 규정 p.19: 빨간불이 꺼지는 동시에 파란불이 켜진다. 파란불만 보고 나간다.
            # 분류(is_go)가 실패해도 채도 초록 블롭이 보이면 파란불이다 —
            # 단 ① RED 로 분류된 프레임은 절대 아니고 ② 블롭은 '빨간 구간 동안
            # 없다가 새로 나타난' 것이어야 한다(상승엣지). 대기 내내 켜져 있는
            # 정적 오검 블롭은 ②에서 걸러진다.
            # 출발 판정 블롭 = 랩 마커 span 또는 출발 전용 원거리 경로
            # (start_blob_px — 출발선 8~10m 실측, types.py 주석). 랩 카운팅은
            # 여전히 gantry_span 만 쓴다.
            blob = (
                max(obs.signal.gantry_span_px, obs.signal.start_blob_px)
                >= self.config.start_go_min_span_px
            )
            if not blob:
                if self._dark_since is None:
                    self._dark_since = obs.now
                if obs.now - self._dark_since >= self.config.start_dark_min_s:
                    self._dark_phase_seen = True
            else:
                self._dark_since = None
            # 판정 경로 3중화 (우선순위 순):
            #  ① 위치 워처 — 색 무관, '불빛이 오른쪽으로 옮겨감' (가장 강함)
            #  ② 4구 분류 — 근거리에서만 성립
            #  ③ 블롭 상승엣지 — 원거리 보조 (RED 로 분류되면 거부)
            go_hit = (
                obs.signal.watcher_go
                or obs.signal.state.is_go
                or (
                    obs.signal.state is not SignalState.RED
                    and blob
                    and self._dark_phase_seen
                )
            )
            if self.manual_go:
                self.manual_go = False
                ctx.race_started_at = obs.now
                ctx.lap.ensure_start_sighting(now=obs.now, yaw_deg=obs.yaw_deg)
                ctx.log(obs.now, "MANUAL GO → start")
                return self._to(Mode.LANE_DRIVE, "manual go", ctx, obs.now)
            if self.manual_start_mode:
                return self._stay("waiting manual go")
            if self._green.update(go_hit, obs.now):
                ctx.race_started_at = obs.now
                # 출발 = 신호등 앞 목격 1 — 원거리 출발 경로로 span 이 0 이어도
                # 랩 기준선을 세운다 (0817 실사고 lap0 출발 방지).
                ctx.lap.ensure_start_sighting(now=obs.now, yaw_deg=obs.yaw_deg)
                ctx.log(obs.now, "GREEN → start")
                return self._to(Mode.LANE_DRIVE, "green confirmed", ctx, obs.now)
            return self._stay("waiting green")

        # ── 여기서부터 주행 중 ─────────────────────────────────────
        new_sighting = ctx.lap.update(
            gantry_span_px=obs.signal.gantry_span_px,
            now=obs.now,
            yaw_deg=obs.yaw_deg,
        )

        if ctx.lap.finished:
            ctx.log(obs.now, f"finished {ctx.lap.sightings - 1} laps")
            return self._to(Mode.FINISH, "3 laps complete", ctx, obs.now)

        if new_sighting:
            ctx.log(obs.now, f"lap {ctx.lap.current_lap} begins")
            # 분기점 도착. 판단은 원샷이 아니라 잠시 열어 둔다 — 좌회전 분류가
            # 목격 확정보다 몇 틱 늦게 도착해도(분류 가능 거리와 확정 거리가
            # 근접) 기회를 놓치지 않기 위해서다. 기회는 대회 전체에 1회다.
            self._decision_until = obs.now + self.config.left_signal_latch_s

        if (
            self.shortcut_enabled
            and obs.now <= self._decision_until
            and self.state not in MANEUVER_MODES
        ):
            left_fresh = (obs.now - self._left_seen_at) <= self.config.left_signal_latch_s
            if ctx.may_take_shortcut(left_fresh):
                self._decision_until = -1e9
                ctx.commit_shortcut(obs.now)
                self._shortcut_yaw0 = obs.yaw_deg
                self._shortcut_t0 = obs.now
                return self._to(Mode.SHORTCUT, "left signal → shortcut", ctx, obs.now)

        # ── 커밋된 기동은 자기 종료 조건으로만 빠져나온다 ──────────
        if self.state is Mode.SHORTCUT:
            turned = abs(obs.yaw_deg - self._shortcut_yaw0)
            elapsed = obs.now - self._shortcut_t0
            if turned >= self.config.shortcut_turn_deg:
                ctx.log(obs.now, f"shortcut entry turn done ({turned:.0f}deg)")
                return self._to(Mode.LANE_DRIVE, "shortcut turn complete", ctx, obs.now)
            if elapsed >= self.config.shortcut_timeout_s:
                ctx.log(obs.now, f"shortcut turn timeout ({turned:.0f}deg)")
                return self._to(Mode.LANE_DRIVE, "shortcut timeout", ctx, obs.now)
            return self._stay(f"turning {turned:.0f}deg")

        if self.state is Mode.CONE_DRIVE:
            if obs.cone.gate_valid:
                self._cone_gate_last_valid = obs.now
            # 규정 p.25: 구간 1분 내 통과 실패 = 실격. 지체가 길어지면 알린다.
            if (
                not self._cone_warned
                and obs.now - self._cone_entered_at > self.config.cone_section_warn_s
            ):
                self._cone_warned = True
                ctx.log(obs.now, "cone section >30s — p.25 1분 제한 주의")
            # 이탈 경로 2개: ① 주황이 사라짐(기존) ② 게이트가 오래 안 생김 —
            # 벽면 보관 무더기는 주황을 계속 채우지만 게이트를 못 만들므로
            # ②가 없으면 여기 갇혀 전방 타겟 회피가 영영 꺼진다.
            gate_lost = (
                obs.now - self._cone_gate_last_valid
                >= self.config.cone_gate_lost_exit_s
            )
            gone = obs.cone.orange_px < self.config.cone_min_orange_px
            if gate_lost or self._cone_out.update(gone, obs.now):
                self._cone_in.reset()
                reason = "gate lost" if gate_lost else "cones gone"
                ctx.log(obs.now, f"cone section cleared ({reason})")
                return self._to(Mode.LANE_DRIVE, reason, ctx, obs.now)
            return self._stay("in cone section")

        if self.state in (Mode.AVOID_STATIC, Mode.OVERTAKE):
            if self.maneuver_done:
                self.maneuver_done = False
                self._target_in.reset()
                ctx.log(obs.now, f"{self.state.value} complete")
                return self._to(Mode.LANE_DRIVE, "pass complete", ctx, obs.now)
            # 정지 물체인 줄 알았는데 움직이면 추월 규정이 적용된다. 승격시킨다.
            if self.state is Mode.AVOID_STATIC and self._target_is_moving(obs):
                ctx.log(obs.now, "static target moving → OVERTAKE")
                return self._to(Mode.OVERTAKE, "target moving", ctx, obs.now)
            return self._stay("passing target")

        # ── LANE_DRIVE 에서의 반응형 진입 ──────────────────────────
        if self.state is Mode.LANE_DRIVE:
            cone_hit = obs.cone.orange_px >= self.config.cone_min_orange_px
            if self.config.cone_enter_requires_gate:
                # 한쪽 덩어리(벽면 보관 무더기)로는 진입하지 않는다.
                cone_hit = cone_hit and obs.cone.gate_valid
            if self._cone_in.update(cone_hit, obs.now):
                self._cone_out.reset()
                self._cone_entered_at = obs.now
                self._cone_gate_last_valid = obs.now
                self._cone_warned = False
                ctx.log(obs.now, "cone section entered")
                return self._to(Mode.CONE_DRIVE, "cones detected", ctx, obs.now)

            target = obs.lidar.nearest_front()
            blocked = (
                self.maneuvers_enabled
                and target is not None
                and target.x <= self.config.target_trigger_dist_m
            )
            if self._target_in.update(blocked, obs.now):
                moving = self._target_is_moving(obs)
                mode = Mode.OVERTAKE if moving else Mode.AVOID_STATIC
                ctx.log(
                    obs.now,
                    f"target at {target.x:.2f}m → {mode.value}",
                )
                return self._to(mode, "front target", ctx, obs.now)

            return self._stay("lane driving")

        return self._stay("no transition")

    # ──────────────────────────────────────────────────────────────
    def _target_is_moving(self, obs: Observation) -> bool:
        """전방 타겟이 방해차량(주행)인가 고정장애물(정지)인가.

        규정상 둘은 같은 크기의 차량 모형이라 형상으로는 못 가른다.
        정지 물체는 상대 접근속도가 자차속도와 같고(vx ≈ -v_ego), 같은 방향으로
        느리게 달리는 차는 그보다 작다.

        자차가 거의 서 있으면 판별이 무의미하므로 그때는 '주행 중'으로 본다 —
        주행 타겟용 컨트롤러가 매 프레임 재평가를 하므로 정지 타겟에도 안전하고,
        반대로 오판했을 때의 손해(추월 방향 위반 = 차선이탈 판정)가 더 크기 때문.
        """
        target = obs.lidar.nearest_front()
        if target is None:
            return False
        if obs.ego_speed_mps < self.config.moving_min_ego_mps:
            return True
        closing = abs(target.vx)
        return closing < self.config.moving_ratio * obs.ego_speed_mps

    def _stay(self, reason: str) -> Transition:
        return Transition(self.state, self.state, reason)

    def _to(
        self, target: Mode, reason: str, ctx: RaceContext, now: float
    ) -> Transition:
        source = self.state
        if target is source:
            return self._stay(reason)
        self.state = target
        ctx.state_entered_at = now
        return Transition(source, target, reason)


__all__ = ["Mode", "MANEUVER_MODES", "Transition", "FSMConfig", "RaceFSM"]

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
from .types import Observation, SignalState, TrackedObject


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
    # ── 출발 대기 워치독 (0820 감사) ────────────────────────────────
    # WAIT_GREEN 의 탈출구는 manual_go 와 go_hit 둘뿐이고 시간 폴백이 없다.
    # 세 판정 경로가 모두 침묵하면 무한 대기 = DNF 다. 두 겹으로 막는다.
    #  (1) 진단 에스컬레이션 — 이 주기마다 '어디서 막혔는지'를 WARN 으로
    #      찍는다. 심판·운영자가 즉시 안다. 항상 켜져 있다.
    wait_green_warn_period_s: float = 5.0
    #  (2) 최후 폴백 — 이 시간이 지나면 RED 가 아닌 한 출발한다 [s].
    #      ★ 기본값 0.0 = 꺼짐. 이건 규정 트레이드오프라 사람이 정할 일이다:
    #        켜면 판독 실패 시 DNF 를 면하지만, 아직 빨간불인데 나가면
    #        규정 p.20 의 벌초를 받는다. 실차에서 '초록인데 못 읽는' 것이
    #        확인되면 실제 대기시간보다 넉넉히 큰 값으로 켤 것.
    wait_green_timeout_s: float = 0.0
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
    # ★ 라바콘 구간을 **나가려면** 라이다 콘도 이보다 적어야 한다 (0821 밤).
    #   카메라만 보고 나가면, 라이다가 콘을 8~14개 보는 상황에서 차선
    #   검출로 조향이 넘어간다 (실측 31~33%). 각진 벽 근처에서 그게
    #   코스 이탈이 된다.
    cone_exit_min_lidar: int = 3
    # 라이다 콘이 이만큼 보이고 게이트가 서면 **카메라 없이도** 진입한다.
    # 4개면 좌우 두 줄이 두 쌍은 보인다는 뜻이라 벽 무더기와 구분된다.
    cone_enter_min_lidar: int = 4
    # 진입의 **최소 조건**: 라이다에 콘이 이만큼은 보여야 한다.
    # (카메라가 먼저 보는 경우를 죽이지 않도록 낮게 둔다)
    cone_enter_require_lidar: int = 2
    # 게이트가 이 시간 이상 안 생기면 구간이 끝난 것으로 보고 이탈한다.
    # 벽면 보관 콘 무더기는 orange_px 를 계속 채우지만 게이트를 못 만들어
    # (run4 실측: 660프레임 중 게이트 5프레임) 이탈 디바운스를 굶긴다 —
    # CONE_DRIVE 에 잔류하면 전방 타겟 회피가 꺼진 채 장애물로 직진한다.
    # 0818 실차: 슬라럼 안에서 게이트 성립률이 최대 37%(BEV 원근 왜곡으로
    # 콘이 얼룩이 됨)라 1.2s 는 구간 한복판에서 튕겨 나가게 만들었다 —
    # 나가면 콘에 오염된 차선 추종으로 떨어져 실패. 4.0s 로 늘리고, 그 사이
    # 단측 가상 게이트가 조향을 맡는다. 무더기(벽면 보관) 이탈은 여전히 된다.
    cone_gate_lost_exit_s: float = 4.0
    # 라이다에 콘이 남아 있어도 이 시간을 넘기면 나간다 (갇힘 방지).
    # 규정 p.25 는 라바콘 구간 1분 초과를 실격으로 본다.
    cone_max_dwell_s: float = 30.0
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
    # ★ 진입 최소 지속시간 [s] (0820 저녁 신설). 옛 게이트는 '연속 2틱'뿐인데
    #   제어는 50Hz, 라이다는 8.8Hz다 — 2틱(0.04s)은 **같은 스캔을 두 번 본
    #   것**이라 사실상 디바운스가 없었다. 진짜 장애물은 1.45m→0.5m 를
    #   0.5m/s 로 약 2초에 걸쳐 계속 막는다. 실측(run_0820_1821, 차선기준
    #   코리도 0.40×0.30, 주행 186초):
    #       지속 임계   남는 이벤트   기동 총시간
    #        0.07s        54회        33.0초(18%)
    #        0.40s        23회        27.2초(15%)
    #        0.80s        16회        22.9초(12%)
    #        1.50s         7회        13.4초( 7%)
    #   0.80 은 '스쳐가는 것'을 자르면서 접근 중인 장애물은 0.5m 남기고도
    #   충분히 일찍(1.45−0.5×0.8 = 1.05m) 잡는 자리다.
    # ★ 0.80 → 0.40 (0820 밤 감사). 0.80 은 **본선 속도에서 구조적으로 발화
    #   불가**였다: _front_target 이 타겟을 인정하는 창은
    #   x ∈ [target_min_dist_m 0.40, target_trigger_dist_m 1.45] = 1.05m 인데,
    #   LANE_DRIVE base 20 × speed_to_mps 0.08 = 1.60m/s 면 그 창을 0.656초에
    #   통과한다. 0.80초 연속을 요구하면 정지 장애물에 **어떤 프레임에서도**
    #   AVOID_STATIC 이 서지 않는다 (실주행 race_0820_2119 에서 OVERTAKE 가
    #   0.46m 에서 딱 1회 선 것도 그 지점이 저속이었기 때문이다).
    #   0.40 이면 2.6m/s 까지 창이 성립한다. run_0820_1821 재생에서 이미
    #   계측된 동작점이다 (이벤트 23회 / 기동 27.2초 = 15%).
    target_enter_seconds: float = 0.40
    # 접근 확인 — 진입 창 동안 타겟이 실제로 가까워졌는가 [m].
    # 정면을 가로질러 스쳐가는 물체는 x 가 줄지 않는다. 0 이면 검사 안 함.
    target_approach_min_m: float = 0.12
    # 타임아웃 탈출 후 재진입 금지 시간 [s] — PassResult.aborted 주석 참조.
    maneuver_cooldown_s: float = 2.0
    # ★ 차선 기준 코리도 (0820 저녁 신설) ─────────────────────────────
    # 문제: nearest_front 의 코리도는 **차 정면으로 곧게 뻗은 직사각형**이다.
    #   이 트랙의 차로 반폭은 0.41m(노란선↔흰선 335px @810px/m)인데 코리도
    #   반폭은 0.45m — 차로보다 넓다. 그래서 코너에서 바깥 벽이 정면 1.45m
    #   안에 구조적으로 들어온다. run_0820_1821 실측(주행 1796스캔):
    #       코리도 ±0.45m → 트리거 후보 38.3%, 2스캔연속 전이 35회
    #       코리도 ±0.20m → 트리거 후보 24.7%, 전이 26회  ← 좁혀도 안 없어진다
    #   좁히는 걸로는 못 고친다. 코너에서는 **바깥 벽이 진짜로 정면에 있기**
    #   때문이다. 고쳐야 할 것은 폭이 아니라 코리도의 '모양'이다.
    # 해법: 카메라가 아는 차로를 따라 코리도를 휘게 한다. 장애물은 '차로를
    #   막고 있는 것'이고 벽은 '차로의 경계'다 — 이 둘은 차로 좌표에서만
    #   갈린다. 차선이 안 잡히는 프레임은 옛 직사각형 코리도로 되돌아간다
    #   (검출을 좁히지 않는다 — 판정 기준을 바꾸는 것이다).
    lane_corridor_enabled: bool = True
    lane_corridor_half_m: float = 0.30   # 차로 중심 기준 반폭 [m]
    # 전방으로 인정하는 종방향 하한 [m]. nearest_front 의 0.15 는 '자기 반사
    # 배제'가 목적이었는데, 이 값이 낮으면 (0.15, −0.30) 처럼 **차 옆 63°**에
    # 있는 반환까지 전방 장애물로 센다 — 이미 옆에 있는 것은 피할 수도 없다.
    # 라이다는 차체 중앙 부근이고 앞 범퍼까지 ≈0.25m 이므로 그 밖을 요구한다.
    target_min_dist_m: float = 0.40
    # BEV 축척 (0818 실측 — tools/lanesim.py 주석과 같은 출처)
    bev_px_per_m_lat: float = 810.0
    bev_px_per_m_fwd: float = 730.0
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
    # ★ 0818 실측 캘리브 (run4 랩3 — 차가 실제로 지름길을 달린 bag):
    #   진입 좌회전은 약 57°/1.5s 였고 그 뒤는 긴 대각 직선(~7s)이다.
    #   90° 급전환이 아니다 — 72° 를 기다리면 과회전해 반대편 차선을 문다.
    shortcut_turn_deg: float = 55.0
    # 요가 안 돌아도 이 시간이 지나면 완료로 보고 차선주행에 넘긴다 (교착 방지)
    # ★ 3.2 → 1.6 (0820 밤 감사). imu_enabled=false 면 obs.yaw_deg 가 영구히
    #   0.0 이라 `turned >= shortcut_turn_deg(55)` 종료 조건은 **구조적으로
    #   절대 발화하지 않고**, 이 타임아웃이 유일한 exit 이 된다. 그 동안
    #   policy 가 차선 피드백 없이 shortcut_steer_deg 를 고정 출력하므로
    #   이 값이 곧 개루프 회전각이다. 실차 캘리브는 '진입 좌회전 57°/1.5초'
    #   (run4) 이므로 3.2초는 78~120° = 주석이 '반대편 차선을 문다'고 못박은
    #   72° 를 모든 경우에 넘는다. 언더슛은 LANE_DRIVE 가 차선 피드백으로
    #   회수할 수 있고 오버슛은 회수 불가라 짧은 쪽으로 틀리는 것이 옳다.
    shortcut_timeout_s: float = 1.6
    # ── 규정 추론 (race_context.may_take_shortcut 의 infer_lap3) ──
    # 규정 p.35 상 좌회전은 랩2·랩3 중 정확히 한 번 나온다. 랩2 에서 겐트리를
    # 이만큼 또렷이 보고도 직좌가 아니었으면 좌회전은 랩3 확정이다.
    # ⚠️ 기본 봉인. 켜면 랩3 에서 검출이 침묵해도 지름길로 들어간다 —
    #    기회 상실(놓침)을 무신호 진입 리스크와 맞바꾸는 선택이라 운영자
    #    판단 영역이다. 진짜 직좌 bag 으로 페어 검출이 검증되기 전에는 켜지 말 것.
    lap3_infer_enabled: bool = False
    lap2_clear_frames_min: int = 8

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
        self._target_in = _Debouncer(c.target_enter_frames, c.target_enter_seconds)
        # 진입 창 동안의 최원거리 — 접근 확인용 (target_approach_min_m)
        self._target_far_x: float = 0.0
        self._stop_recover = _Debouncer(c.stop_recover_frames, c.stop_recover_seconds)

        # WAIT_GREEN 워치독이 마지막으로 경고한 대기시간 [s]
        self._wait_warned_at: float = 0.0
        # 좌회전 신호를 마지막으로 확실히 본 시각
        self._left_seen_at: float = -1e9
        # SHORTCUT 진입 시점의 요와 시각
        self._shortcut_yaw0: float = 0.0
        self._shortcut_t0: float = 0.0
        # 기동 상태가 자기 종료를 보고하는 통로 (policy 가 써 준다)
        self.maneuver_done: bool = False
        # 타임아웃 탈출 신호 (policy 가 써 준다). '통과 완료'와 구분한다.
        self.maneuver_aborted: bool = False
        # 타임아웃 탈출 직후 재진입 금지 만료 시각
        self._maneuver_cooldown_until: float = -1e9
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
            # ★ watcher_green_left 제거 (0820 밤 감사). signal_detector 의
            #   동형 페어 경로는 pair_green_left_enabled=False 로 이미 봉인돼
            #   있는데(0817 전수조사: 진짜 직좌 0발화·주행 중 오검 다수),
            #   signal_watcher 의 페어는 '색을 쓰지 않는' 밝기·모양 판정이라
            #   더 약한데도 FSM 래치로 우회해 살아 있었다. 지름길 오래치 1회 =
            #   직진 랩 한복판 좌회전이다. 진짜 경로는 4구 구조검증이 담당한다.
            and obs.signal.state is SignalState.GREEN_LEFT
            # 0820 저녁: 하우징 폭도 받아 준다. GREEN_LEFT 는 이제 4구 구조
            # 검증을 통과해야만 나오므로 하우징 폭이 곧 근접 증거이고, 블롭
            # 경로(watcher)만 살아 있는 프레임도 여전히 통과한다 — 둘 중
            # 큰 쪽을 쓰는 것은 게이트를 좁히지 않고 넓히는 방향이다.
            and max(obs.signal.housing_span_px, obs.signal.gantry_span_px)
            >= self.config.left_latch_min_span_px
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
                self._wait_warned_at = 0.0
                return self._to(Mode.WAIT_GREEN, "inputs ready", ctx, obs.now)
            return self._stay(safety.reason or "waiting inputs")

        # ── 출발 대기 ──────────────────────────────────────────────
        if self.state is Mode.WAIT_GREEN:
            # 이 시점에 차는 신호등 바로 앞에 서 있다. 랩 트래커의 '목격 1'이
            # 여기서 잡히고, 그 결과 current_lap 이 1이 된다.
            ctx.lap.update(
                gantry_span_px=obs.signal.lap_marker_px,
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
            # 워치독 (1) — 판정이 왜 안 되는지 주기적으로 알린다
            waited = obs.now - ctx.state_entered_at
            if waited - self._wait_warned_at >= self.config.wait_green_warn_period_s:
                self._wait_warned_at = waited
                sig = obs.signal
                ctx.log(
                    obs.now,
                    f"WAIT_GREEN {waited:.0f}s — state={sig.state.value} "
                    f"reject={sig.reject_reason or '-'} "
                    f"housing={sig.housing_span_px} span={sig.gantry_span_px} "
                    f"start_blob={sig.start_blob_px} watcher={sig.watcher_go} "
                    f"dark_seen={self._dark_phase_seen}",
                )
            # 워치독 (2) — 최후 폴백 (기본 꺼짐, 설정 주석 참조)
            if (
                self.config.wait_green_timeout_s > 0.0
                and waited >= self.config.wait_green_timeout_s
                and obs.signal.state is not SignalState.RED
            ):
                ctx.race_started_at = obs.now
                ctx.lap.ensure_start_sighting(now=obs.now, yaw_deg=obs.yaw_deg)
                ctx.log(
                    obs.now,
                    f"⚠ WAIT_GREEN 워치독 {waited:.0f}s 초과 → 출발 "
                    f"(신호 판독 실패, 규정 벌초 감수)",
                )
                return self._to(Mode.LANE_DRIVE, "wait watchdog", ctx, obs.now)
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
            gantry_span_px=obs.signal.lap_marker_px,
            now=obs.now,
            yaw_deg=obs.yaw_deg,
        )
        # 제3 채널 — 라바콘 구간 보충 랩 (LapTrackerConfig.cone_fill_enabled
        # 주석에 실측 근거). 겐트리도 IMU 도 못 쓰는 상태에서 완주 판정이
        # 0채널이 되는 것을 막는다.
        if ctx.lap.note_cone_section(
            in_cone=(self.state is Mode.CONE_DRIVE), now=obs.now
        ):
            new_sighting = True
            ctx.log(obs.now, f"라바콘 구간 통과 → 보충 랩 {ctx.lap.sightings}")

        if ctx.lap.finished:
            ctx.log(obs.now, f"finished {ctx.lap.sightings - 1} laps")
            return self._to(Mode.FINISH, "3 laps complete", ctx, obs.now)

        if new_sighting:
            # 새 목격 = 이전 랩의 결정 창은 끝났다. 랩2 였다면 여기서 결론.
            ctx.conclude_lap2_observation(
                min_frames=self.config.lap2_clear_frames_min, now=obs.now
            )
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
            # 랩2 음성 관측 집계 — '겐트리를 또렷이 봤는데 직좌가 아니었다'
            # 는 프레임을 센다. 규정 p.35 상 그게 성립하면 좌회전은 랩3 이다.
            ctx.note_lap2_gantry_frame(
                gantry_clear=(
                    obs.camera_ok
                    and obs.signal.gantry_span_px
                    >= self.config.left_latch_min_span_px
                ),
                saw_left=left_fresh,
            )
            if ctx.may_take_shortcut(
                left_fresh, infer_lap3=self.config.lap3_infer_enabled
            ):
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
            # 게이트 생존 판정은 카메라·라이다 어느 쪽이든 인정한다 (0818 저녁).
            # policy 는 라이다 게이트를 1순위로 조향하는데 여기서 카메라만 보면
            # 카메라 게이트 4s 공백(실측 성립률 37~58%)에 라이다가 멀쩡히
            # 조향 중인 슬라럼 한복판에서 LANE_DRIVE 로 튕겨 나간다.
            if obs.cone.gate_valid or obs.cone.lidar_gate_valid:
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
            # ★ 나가는 판정에 **라이다도 본다** (0821 밤).
            #   옛 코드는 카메라 주황 픽셀만 봤다. 그래서 라이다가 콘을
            #   8~14개 보고 있는데도 CONE_DRIVE 를 나가 **카메라 차선**으로
            #   조향했다 — 실측(cone_0821_0344/0323): 차선 주행 프레임의
            #   31~33% 가 그런 상태였고, t=13.1s 에는 콘 11개가 보이는데
            #   차선을 따라 +42(풀락)로 꺾었다. 각진 벽 근처에서 차선
            #   검출이 헛것을 잡으면 그대로 코스를 이탈한다.
            #   라이다에 콘이 남아 있으면 아직 라바콘 구간이다.
            lidar_busy = obs.cone.lidar_cones >= self.config.cone_exit_min_lidar
            gone = (obs.cone.orange_px < self.config.cone_min_orange_px
                    and not lidar_busy)
            # ※ gate_lost 는 라이다로 막지 않는다. 벽면 보관 무더기는 콘이
            #   많아도 통로가 아니다 — 게이트가 cone_gate_lost_exit_s 동안
            #   한 번도 안 서면 그건 통로가 아니라는 뜻이다.
            #   (게이트가 깜빡이는 문제는 가상 게이트가 이미 해결했다:
            #    구간 내 게이트 성립 52% → 93.5%)
            # ★ 다만 **영영 갇히면 안 된다**. 벽면 보관 무더기처럼 라이다에
            #   콘이 계속 잡히는데 통로가 아닌 상황이 있을 수 있다.
            #   구간 제한(규정 p.25 1분)의 절반을 넘기면 라이다와 무관하게
            #   나간다 — 갇힘이 실격이라 이탈보다 나쁘다.
            if obs.now - self._cone_entered_at > self.config.cone_max_dwell_s:
                gate_lost = True
            if gate_lost or self._cone_out.update(gone, obs.now):
                self._cone_in.reset()
                reason = "gate lost" if gate_lost else "cones gone"
                ctx.log(obs.now, f"cone section cleared ({reason})")
                return self._to(Mode.LANE_DRIVE, reason, ctx, obs.now)
            return self._stay("in cone section")

        if self.state in (Mode.AVOID_STATIC, Mode.OVERTAKE):
            if self.maneuver_done:
                aborted = self.maneuver_aborted
                self.maneuver_done = False
                self.maneuver_aborted = False
                self._target_in.reset()
                self._target_far_x = 0.0
                if aborted:
                    # ★ 타임아웃 탈출 — 타겟은 아직 거기 있다. 그대로 돌려
                    #   보내면 2틱 만에 재진입해 리밋사이클이 된다 (0820 감사).
                    #   쿨다운 동안은 차선주행으로 지나가되, 다시 걸리지
                    #   않게 트리거를 잠근다.
                    self._maneuver_cooldown_until = (
                        obs.now + self.config.maneuver_cooldown_s
                    )
                    ctx.log(
                        obs.now,
                        f"{self.state.value} 타임아웃 탈출 → 차선주행 "
                        f"({self.config.maneuver_cooldown_s:.0f}s 재진입 금지)",
                    )
                else:
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
            gate_seen = obs.cone.gate_valid or obs.cone.lidar_gate_valid
            if self.config.cone_enter_requires_gate:
                # 한쪽 덩어리(벽면 보관 무더기)로는 진입하지 않는다.
                # 게이트는 카메라 또는 라이다 어느 쪽이든 인정 (0818 저녁 —
                # 라이다 쌍은 미터 직접 관측이라 카메라보다 먼저·자주 선다).
                cone_hit = cone_hit and gate_seen
            # ★ 라이다만으로도 진입한다 (0821 밤). 나가는 판정에 라이다를
            #   넣은 것(cone_exit_min_lidar)과 대칭이다.
            #   옛 코드는 **카메라 주황 픽셀**이 병목이라 두 가지가 생겼다:
            #     · 라이다에 콘이 0개인데 진입 (cone_0821_0407 t=12.7/14.2/
            #       35.9/38.0) — 따라갈 통로가 없는 채로 콘 모드가 된다.
            #     · 진입이 늦다. 실측 cone_0821_0356 t=32.3 은 최근접 콘
            #       47cm 에서 들어갔는데 콘은 0.69초 전 95cm 에서 이미 보였다.
            #   진입이 늦으면 정렬할 거리가 없다 — 실측 진입각이 중앙 13.5°,
            #   38%가 20°를 넘는데, R1.5 통로에서 20°를 지우려면 1.11m 가
            #   필요하고 그동안 19.3cm 가 밀린다 (여유는 18.5cm 뿐).
            #   실측 효과: 진입 16~24회 → 7~9회(떨림 감소), 콘 0개 진입 소멸.
            lidar_solid = (
                obs.cone.lidar_cones >= self.config.cone_enter_min_lidar
                and obs.cone.lidar_gate_valid
            )
            cone_hit = cone_hit or lidar_solid
            # ★ 그리고 **라이다에 콘이 하나도 없으면 진입하지 않는다**
            #   (0821 밤 2차). 위를 OR 로 두니 카메라 단독 진입이 그대로
            #   남아, 실측(cone_0821_0443)에서 진입 10회 중 **4회가 라이다
            #   콘 0개** 였다. 통로가 안 보이는데 콘 모드로 들어가면 따라갈
            #   게 없어 조향이 폴백(직전 목표 유지)으로 흐른다.
            #   카메라가 먼저 보는 경우를 살리려 문턱은 낮게 둔다(2개).
            if obs.cone.lidar_cones < self.config.cone_enter_require_lidar:
                cone_hit = False
            if self._cone_in.update(cone_hit, obs.now):
                self._cone_out.reset()
                self._cone_entered_at = obs.now
                self._cone_gate_last_valid = obs.now
                self._cone_warned = False
                ctx.log(obs.now, "cone section entered")
                return self._to(Mode.CONE_DRIVE, "cones detected", ctx, obs.now)

            # ── 전방 타겟(회피/추월) 트리거 ──
            # ★ 콘 게이트가 보이는 동안은 억제한다 (0818 저녁). 통로 폭 0.91m
            #   의 콘은 nearest_front 코리도(±0.45m)·1.45m 안에 수시로 들어와
            #   2틱이면 AVOID_STATIC 이 서는데, 콘 진입(3틱+게이트)보다 빨라
            #   라바콘 구간 입구에서 회피 기동으로 콘 열에 처박힌다. 좌우 두 줄
            #   게이트는 '장애물'이 아니라 '지나갈 통로'라는 확정 신호다.
            target = self._front_target(obs)
            blocked = (
                self.maneuvers_enabled
                and not gate_seen
                and target is not None
                and obs.now >= self._maneuver_cooldown_until
            )
            # 접근 확인 — 진입 창이 열려 있는 동안 본 가장 먼 x 를 기억한다
            if blocked:
                self._target_far_x = max(self._target_far_x, target.x)
            else:
                self._target_far_x = 0.0
            approached = (
                self.config.target_approach_min_m <= 0.0
                or (target is not None
                    and self._target_far_x - target.x
                    >= self.config.target_approach_min_m)
            )
            if self._target_in.update(blocked, obs.now) and approached:
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
    def _front_target(self, obs: Observation) -> Optional[TrackedObject]:
        """전방에서 '내 차로를 막고 있는' 가장 가까운 물체.

        차선이 유효하면 차로를 따라 휘는 코리도로 판정한다
        (FSMConfig.lane_corridor_half_m 주석에 실측 근거).
        차선이 없으면 옛 직사각형 코리도로 되돌아간다.
        """
        cfg = self.config
        lane = obs.lane
        if not (cfg.lane_corridor_enabled and lane.valid):
            # 차선이 없으면 옛 직사각형으로 폴백하되 **거리창은 같이 쓴다**.
            # nearest_front 의 조건은 x>0.15, |y|≤0.45 뿐이라 상한이 없어,
            # 코너 바깥 벽이 2~2.5m 앞에 있어도 타겟이 된다. 진입 지속을
            # 0.40s 로 낮춘 지금은 '차선 잠깐 잃음 + 코너 벽'이 곧바로
            # 회피 오발동으로 승격된다 (0820 밤 감사).
            ob = obs.lidar.nearest_front()
            if (
                ob is None
                or ob.x < cfg.target_min_dist_m
                or ob.x > cfg.target_trigger_dist_m
            ):
                return None
            return ob

        # 차로 중심의 횡위치 [m] (차량 기준 y+ = 좌).
        #   offset_px > 0 = 목표선이 화면 중앙보다 오른쪽 = 차로가 내 오른쪽
        y0 = -lane.offset_px / cfg.bev_px_per_m_lat
        # heading_slope 는 BEV 픽셀 기울기(dx_px/df_px)다. 가로·세로 축척이
        # 달라 미터 기울기로 환산해야 한다: (dx/810)/(df/730) = slope·730/810
        slope_m = lane.heading_slope * (cfg.bev_px_per_m_fwd / cfg.bev_px_per_m_lat)

        best: Optional[TrackedObject] = None
        for ob in obs.lidar.objects:
            # x 하한 0.15m 는 nearest_front 와 같은 근거(자기 반사) — types 주석
            if ob.x < cfg.target_min_dist_m or ob.x > cfg.target_trigger_dist_m:
                continue
            y_center = y0 - slope_m * ob.x     # + 기울기 = 전방에서 우측으로
            if abs(ob.y - y_center) > cfg.lane_corridor_half_m:
                continue
            if best is None or ob.x < best.x:
                best = ob
        return best

    def _target_is_moving(self, obs: Observation) -> bool:
        """전방 타겟이 방해차량(주행)인가 고정장애물(정지)인가.

        규정상 둘은 같은 크기의 차량 모형이라 형상으로는 못 가른다.
        정지 물체는 상대 접근속도가 자차속도와 같고(vx ≈ -v_ego), 같은 방향으로
        느리게 달리는 차는 그보다 작다.

        자차가 거의 서 있으면 판별이 무의미하므로 그때는 '주행 중'으로 본다 —
        주행 타겟용 컨트롤러가 매 프레임 재평가를 하므로 정지 타겟에도 안전하고,
        반대로 오판했을 때의 손해(추월 방향 위반 = 차선이탈 판정)가 더 크기 때문.
        """
        target = self._front_target(obs)
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

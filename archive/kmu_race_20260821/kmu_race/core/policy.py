"""모드 → 주행명령. core 계층의 최상단이며 여전히 rclpy 를 모른다.

FSM 이 '무엇을 하는 중인가'를 정하고, 여기서 '그래서 바퀴를 어떻게 돌릴까'를 만든다.
둘을 나눠 놓은 덕에 test/ 에서 관측을 손으로 지어내 명령까지 끝단 검증할 수 있다.
"""

from __future__ import annotations

import math

from dataclasses import dataclass, field, replace
from typing import Optional

from .control import (
    CMD_SATURATION,
    CMD_TO_RAD,
    WHEELBASE_M,
    clamp_steer_for_cones,
    LaneController,
    PixelPID,
    SteerSmoother,
    pure_pursuit_cmd,
)
from .race_context import RaceContext
from .race_fsm import Mode
from .safety import SafetyDecision
from .speed_policy import SpeedConfig, SpeedPolicy
from .target_passing import PassConfig, PassPhase, TargetPassing
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
    # ★ 0.35 → 0.50 (0820 밤). 게이트가 코앞에 잡힌 프레임에서도 35cm 를
    #   조준하면 횡오차 12cm 에 조향 30도가 나온다(lidar_cones.prefer_depth_m
    #   주석의 환산표). 게이트 우선순위를 앞쪽으로 옮겼어도 통로 끝에서는
    #   여전히 가까운 쌍이 잡히므로 하한 자체를 올려 둔다.
    cone_lookahead_min_m: float = 0.50
    # ── 게이트가 끊겼을 때 (0820 밤 신설) ──────────────────────────────
    # 라바콘 구간에서는 **콘 사이로 가야지 차선을 보면 안 된다.**
    # 그런데 옛 코드의 마지막 폴백은 `return self._lane_steer(obs)` 였다.
    # 실측(cone_0820_2358 라바콘 525프레임): 라이다 게이트 50.3% /
    # **차선주행 폴백 48.6%**(총 10.4초, 최장 5.4초 연속) / 카메라 1.1%.
    # 즉 구간의 절반을 '콘 위치를 전혀 모르는' 차선 목표로 달렸고, 그 동안
    # 차가 통로를 가로질러 콘을 쳤다 (횡편차가 수렴하지 않고 ±37cm 로 진동,
    # 26%가 여유 25.9cm 초과 — 옛 게이트 설정에서도 28.6%로 같았다).
    # 이제 게이트가 끊기면 **직전 콘 목표를 유지**하고, 그마저 오래되면
    # 통로를 따라 **직진**한다. 차선은 절대 보지 않는다.
    # (구간 이탈은 FSM 의 cone_gate_lost_exit_s=4.0 이 따로 판단한다.)
    # ── 근접 콘 회피 (0821 신설) ──────────────────────────────────────
    # 게이트는 **앞쪽** 쌍을 조준한다(prefer_depth_m 0.55). 그 사이 차 옆을
    # 지나는 콘은 아무도 보지 않았다 — 그게 접촉이다.
    # cone_0821_0025 t10.0~11.3 실측(1.3초):
    #     t10.10  게이트 깊이34cm 폭75cm '횡편차 +36cm' 인데 **우측 콘 1cm**
    #     t10.24  게이트 깊이58cm 폭89cm '횡편차 +11cm' 인데 **우측 콘 1cm**
    #     t10.93  게이트 없음, 좌측 콘 14cm (반대쪽으로 튕김)
    # 같은 구간의 정상 프레임은 좌 33cm / 우 44cm 였으므로, 25cm 를 안전선으로
    # 두면 정상 주행은 건드리지 않고 접촉만 잡는다.
    # 게이트 추종은 그대로 두고 **그 위에 반발을 얹는다** — 한쪽이 안전선보다
    # 가까우면 부족한 만큼 목표를 반대쪽으로 민다.
    cone_safe_clear_m: float = 0.25
    cone_repel_gain: float = 1.0     # 부족분 × 이 값만큼 목표를 민다
    cone_repel_max_m: float = 0.25   # 밀어내기 상한
    # ── 전방주시 (커브 적응은 0821 저녁에 **철회**) ────────────────
    # ★ 한때 '커브에서는 전방주시를 짧게' 를 넣었다가 뺐다. 두 가지 이유다.
    #
    # ① 근거가 사라졌다. 그 기능은 "커브에서 안 꺾인다"를 고치려던 것인데
    #    진짜 원인은 조향 단위였다 (control.pure_pursuit_cmd 주석 — 실효
    #    조향이 39% 였다). 단위를 바로잡은 지금은 풀락이면 R 1.02m 를 돈다.
    #
    # ② 그대로 두면 **곡률을 두 번 센다**. 게이트 횡편차는 이미 통로 곡률을
    #    담고 있다: 반경 R_c 통로에서 깊이 d 인 게이트의 횡편차는 d²/2R_c 다.
    #    전방주시를 게이트 깊이로 두면 퓨어 퍼수트가 내는 경로 반경이
    #        R_path = Ld²/(2e) = d²/(2·d²/2R_c) = R_c
    #    로 **정확히 통로 반경**이 된다. 더 줄이면 그만큼 과조향이다.
    #    실측(manual_0821_0238 — 사람이 통로 한가운데(횡위치 -0.0cm)로
    #    통과한 궤적, 차가 통로 안인 137프레임):
    #        커브 적응(전방주시 중앙 44.7cm)  |조향| 중앙 42.0  포화 54.7%
    #        게이트 깊이 그대로(중앙 63.9cm)  |조향| 중앙 35.3  포화 47.4%
    #    같은 궤적에서 사람 조향은 중앙 0 이었다.
    #
    # 그래서 전방주시는 **관측된 게이트 깊이**를 그대로 쓴다 (상·하한만 둔다).
    # 관측 밖을 조준하면 코너에서 통로를 가로지른다 (0817 ddm bag 진단).
    # ★ 변화율 제한만은 남긴다 — 게이트가 끊겼다 붙으면 깊이가 급변하는데
    #   그게 곧 조향 진동이었다 (0821 실측: 프레임간 최대 30.4cm → 8.0cm).
    cone_lookahead_rate_mps: float = 0.6
    # ── 마지막 관문: 콘 관통 방지 클램프 (0821 밤) ────────────────
    # DrivePolicy._cone_guard 주석에 근거. 요약: 추정을 다 맞히는 대신
    # **명령이 콘을 관통하지 못하게** 막아 주행 간 편차를 없앤다.
    # ★ 콘 목표의 **변화율 제한** [m/s] (0821 밤 2차).
    #   통로 중심은 30ms 만에 30cm 움직이지 않는다. 그런데 게이트가 좌우
    #   쌍 → 단측 가상 게이트로 바뀌는 순간 목표가 통째로 뛰었다:
    #       t=0.00  횡 +1.0cm  → 조향 -42
    #       t=0.32  횡 -30.0cm → 조향 +42     ← 한 프레임에 31cm
    #   그 한 번의 점프로 차가 통로를 벗어나 돌아오지 못했다(이탈 275cm).
    #   값은 진입조건 스윕(통로 4종 × 횡위치 7 × 진입각 5 = 140케이스)으로
    #   정했다: 끔 77% / 3.0 77% / **2.0 79%** / 1.0 75% / 0.6 71%.
    #   너무 조이면 큰 진입각에서 필요한 빠른 목표 이동까지 막는다.
    #   2.0m/s 면 프레임당 6.6cm — 정상 기동은 통과, 순간이동(31cm)은 차단.
    cone_lat_rate_mps: float = 2.0
    # ── 접근 중 통로 정렬 (0821 밤 신설) ─────────────────────────
    # 근거는 DrivePolicy._lane_with_cone_approach 주석.
    # 게이트 깊이가 far → near 로 가까워질수록 콘 조향 비중을 0 → max_w 로
    # 올린다. 갈아타지 않고 섞는다 — 접근 중 통로 추정이 틀려도 차선이 받는다.
    # ★ 조향 혼합은 **기본 끔**. 실측(cone_0821_0407/0356)에서 차선
    #   프레임의 42~50%나 발동하면서 차선 조향을 중앙 15.8~19.6 만큼
    #   바꿨는데, 정작 통로 쪽으로 미는 비율이 57~60% — 동전던지기와
    #   큰 차이가 없었다. 가상 게이트 때문에 '통로가 앞에 있다'가
    #   아무 데서나 성립하는 게 원인이다. 조건을 좁히기 전에는 위험하다.
    cone_approach_enabled: bool = False
    cone_approach_far_m: float = 1.50    # 이보다 멀면 안 섞는다
    cone_approach_near_m: float = 0.70   # 이만큼 가까우면 최대 비중
    cone_approach_max_w: float = 0.70    # 최대 비중 (1.0 = 완전히 통로만)
    cone_approach_min_cones: int = 4     # 통로로 인정할 최소 콘 수
    # ── 진입 전 감속 (0821 밤 신설) ───────────────────────────────
    # ★ 실차는 라바콘 구간 **직전까지 2.16 m/s** 로 달린다. 그런데
    #   CONE_DRIVE 진입은 최근접 콘 ~50cm 에서 일어나고(실측), 모터 가속
    #   제한 2.5 m/s² 로 2.16 → 1.44 m/s 감속에 0.29초·**0.52m** 가 걸린다.
    #   즉 차는 첫 콘에 거의 2.16 m/s 로 도달한다.
    #   빠를수록 지연(0.15초) 중 이동이 늘어 진입각 오차가 그대로 횡이탈이
    #   된다: 1.0m/s 면 15cm, 2.16m/s 면 **32cm** 를 눈감고 간다.
    #   라이다는 통로를 0.7~1.9초 전(76~95cm)에 이미 보므로, 그때부터
    #   목표속도를 라바콘 속도로 미리 낮춘다. **감속은 틀려도 안전하다** —
    #   조향 혼합과 달리 오발동의 대가가 시간뿐이다.
    cone_approach_slow_enabled: bool = True
    cone_approach_slow_far_m: float = 1.60   # 이보다 멀면 안 줄인다
    cone_approach_slow_near_m: float = 0.60  # 이만큼 가까우면 라바콘 속도
    cone_approach_slow_min_m: float = 0.45   # 이보다 가까우면 이미 진입
    cone_approach_outside_m: float = 0.60    # 옆 콘이 이보다 가까우면 통로 안
    # ── 라바콘 PID (0821 밤 신설) ────────────────────────────────
    # 퓨어 퍼수트는 기하 피드포워드다 — 목표점을 향해 '얼마나 꺾어야
    # 도달하는가'를 계산할 뿐, 오차가 **얼마나 빨리 줄고 있는지**는 보지
    # 않는다. 그래서 반응이 한 박자 늦다.
    # 그 위에 추종오차 e(= 목표 횡위치, 차가 0 이므로 곧 오차)에 대한
    # P·D 를 더해 반응을 앞당긴다. 단위는 cmd (control.CMD_TO_RAD 기준).
    #
    # 적분항은 두지 않는다: 통로는 몇 초면 지나가고, 게이트가 끊긴 프레임에
    # 적분이 폭주한다 (PixelPID 주석의 경고와 같은 이유).
    #
    # 게인은 폐루프 진입조건 스윕(통로 4종 × 횡위치 7 × 진입각 5)으로 정했다:
    #     kp    kd    통과      최소여유
    #      0     0    80%       7.0cm
    #     20     0    80%       7.3cm   ← 채택
    #     30     0    77%       6.8cm
    #     20     3    78%       6.8cm
    #     20     6    76%       6.8cm
    #     30    10    72%       5.8cm
    # ★ **미분항은 넣을수록 나빠진다.** 이 루프는 게인이 모자란 게 아니라
    #   포화와 지연에 묶여 있다 — 이미 25~30% 프레임이 풀락인데 게인을
    #   올리면 더 빨리 포화할 뿐이고, D 는 라이다 잡음(게이트 횡편차)을
    #   증폭한다. 그래서 kd 는 0 으로 둔다. 코드는 남겨 두어 나중에 신호가
    #   깨끗해지면(예: 통로 곡률 직접 추정) 다시 시험할 수 있게 한다.
    # ── 라이다 외삽 (0821 밤 신설) ────────────────────────────────
    # 라이다는 ~9Hz, 제어는 50Hz 다. 스캔 사이에 관측이 최대 110ms 낡고,
    # 그동안 차는 1.0m/s 면 11cm, 2.16m/s 면 24cm 를 간다. 그 낡음이 그대로
    # 조향 지연이 된다 — 게인을 올려 봐야(PID 스윕 참조) 이건 안 줄어든다.
    # 자차 속도와 조향은 **이미 알고 있으므로** 관측을 현재 좌표로 옮긴다:
    #     ds = v·age,  dθ = (v/L)·tan(δ)·age,  p' = R(−dθ)·(p − (ds,0))
    # δ 는 발행 조향이 아니라 **지연을 통과한 실제 바퀴각**을 쓴다
    # (구동 0.04 + 서보 τ 0.06 = 0.10초 뒤처진다).
    # ★ 실측 결과 **기본 끔**. 폐루프 진입조건 스윕(140케이스):
    #       외삽 끔      80%  여유 7.3cm
    #       외삽  30%    78%  여유 6.8cm
    #       외삽  50%    80%  여유 6.9cm
    #       외삽  70%    77%  여유 6.4cm
    #       외삽 100%    77%  여유 6.4cm
    #       외삽 100%(회전 무시)  74%  여유 4.5cm
    #   회전항을 빼면 확실히 나빠지므로 구현 자체는 맞다 — **효과가 없는
    #   것**이다. 이 루프는 정보 지연이 아니라 **포화와 최소 회전반경**에
    #   묶여 있다 (같은 결론: PID 게인·미분항·아펙스 커팅 모두 무효였다).
    #   코드는 남긴다 — 통로가 넓어지거나 서보가 빨라져 포화가 풀리면
    #   그때는 지연이 병목이 되어 이 외삽이 값을 낼 수 있다.
    cone_extrapolate_enabled: bool = False
    cone_extrapolate_max_s: float = 0.20   # 이보다 낡으면 믿지 않는다
    cone_extrap_delta_tau_s: float = 0.10  # 구동지연 + 서보 시정수
    cone_pid_kp: float = 20.0 # cmd per m
    cone_pid_kd: float = 0.0      # cmd per (m/s)
    cone_pid_d_alpha: float = 0.3  # 미분 평활 (라이다 잡음 억제)
    cone_pid_max: float = 20.0    # PID 기여분 상한 [cmd]
    cone_guard_enabled: bool = True
    cone_guard_margin_m: float = 0.03   # 이만큼 여유가 나야 통과시킨다
    cone_guard_react_s: float = 0.35    # 구동 0.04 + 서보 τ 0.06 + 여유
    # ★ 조향만으로는 못 막는다. 실측(cone_0821_0307/0323)에서 관문을 켜도
    #   관통 프레임이 9.2→8.0%, 13.1→12.6% 밖에 안 줄었다 — 남은 것은
    #   **어떤 조향으로도 안전하지 않은** 상태, 즉 이미 늦은 프레임이다.
    #   수평선이 v × react_s 이므로 속도를 줄이면 수평선이 짧아져 안전한
    #   조향이 다시 생긴다. 물리적으로도 느릴수록 빠져나올 시간이 는다.
    #   그래서 여유가 모자라면 그만큼 속도를 깎는다.
    cone_guard_slow_at_m: float = -0.10   # 이만큼 파고들면 최저 속도
    cone_guard_min_ratio: float = 0.30    # 그때 곱하는 값
    cone_hold_s: float = 1.0        # 직전 목표를 그대로 유지하는 시간
    cone_straighten_s: float = 2.5  # 이 시간에 걸쳐 목표를 0(직진)으로 완화
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
    # ── 단측 반발 bias 의 시간 평활 ─────────────────────────────────
    # 게이트가 안 설 때 쓰는 bias = (L-R)/(L+R) 은 실측상 **잡음이다**.
    # 0820 bag 콘 구간 실측(원본 프레임): 검출되는 콘이 1~3개뿐이라 인접
    # 프레임에서 +1.00 ↔ -1.00 로 통째로 뒤집힌다 (t075 +1.00 / t080 -1.00,
    # lap03 +1.00 / lap04 -1.00). 통로는 30Hz 로 좌우를 바꾸지 않는다.
    # 구조적 원인: BEV 가로 시야가 640/810 = 0.790m 인데 콘 통로가 0.800m 라
    # 좌·우 콘이 동시에 근접 BEV 에 못 들어온다 — 차가 0.5cm 만 치우쳐도 그쪽만
    # 잡힌다. 즉 이 신호는 '통로 방향'이 아니라 '치우친 방향'에 가깝다.
    # 출력단 SteerSmoother(창 3프레임=33ms)로는 이 뒤집힘을 못 막는다.
    # 그래서 신호 단계에서 EMA 를 건다 (alpha 0.15 → 시정수 ≈0.2s @30fps).
    cone_bias_alpha: float = 0.15
    # 회피 '방향을 정하는 틱'의 타겟 코리도 반폭 [m] — FSM 의
    # lane_corridor_half_m 과 같은 값으로 맞춘다 (위 A6 주석 참조).
    fsm_corridor_half_m: float = 0.30
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


def _clamp_cmd(v: float) -> float:
    """서보 포화로 자른다 (PID 를 더한 뒤 다시 한 번)."""
    return max(-CMD_SATURATION, min(CMD_SATURATION, v))


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
        self._last_pass_aborted = False
        self._prev_mode: Optional[Mode] = None
        # 라바콘 통로 반폭 학습값 [m] — 단측 구간의 가상 게이트에 쓴다
        self._cone_half_m = self.config.cone_half_corridor_m
        # 단측 반발 bias 의 EMA 상태 (설정 주석 참조)
        self._cone_bias: float = 0.0
        # 마지막으로 본 콘 목표 (게이트 끊김 대비) — cone_hold_s 주석 참조
        self._cone_last_lat: float = 0.0
        self._cone_last_la: float = 0.0
        self._cone_last_t: float = -1e9
        self._cone_lat: float = 0.0
        self._cone_lat_t: float = -1e9
        self._cone_err_rate: float = 0.0
        self._cone_err_prev: float = 0.0
        self._cone_err_t: float = -1e9
        self._cone_delta: float = 0.0
        self._cone_delta_t: float = -1e9
        self._cone_guard_gap: float = 9.9
        self._cone_guard_hits: int = 0
        # 실제로 쓰는 전방주시 (변화율 제한을 거친 값)
        self._cone_la: float = 0.0
        self._cone_la_t: float = -1e9

    @property
    def maneuver_aborted(self) -> bool:
        """타임아웃 탈출인가 (통과 완료가 아니다 — PassResult.aborted 주석)."""
        return self._last_pass_aborted

    @property
    def maneuver_done(self) -> bool:
        """직전 update() 에서 통과 기동이 끝났는가. FSM 이 읽어 간다."""
        return self._last_pass_done

    def reset(self) -> None:
        self.pid.reset()
        self._cone_bias = 0.0
        # 라바콘 구간 상태 — 새로 시작할 때 지난 구간을 물려받지 않는다
        self._cone_la = 0.0
        self._cone_la_t = -1e9
        self._cone_lat = 0.0
        self._cone_lat_t = -1e9
        self._cone_err_rate = 0.0
        self._cone_err_prev = 0.0
        self._cone_err_t = -1e9
        self._cone_delta = 0.0
        self._cone_delta_t = -1e9
        self._cone_last_lat = 0.0
        self._cone_last_la = 0.0
        self._cone_last_t = -1e9
        self.lane_ctl.reset()
        self.speed.reset()
        self.smoother.reset()
        self.pass_static.reset()
        self.pass_moving.reset()
        self._last_steer = 0.0
        self._last_pass_done = False
        self._last_pass_aborted = False

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
        self._last_pass_aborted = False

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
            if mode is Mode.CONE_DRIVE:
                # 새 라바콘 구간 — 지난 구간의 목표를 물려받지 않는다
                self._cone_la = 0.0
                self._cone_la_t = -1e9
                self._cone_lat = 0.0
                self._cone_lat_t = -1e9
                self._cone_err_rate = 0.0
                self._cone_err_prev = 0.0
                self._cone_err_t = -1e9
                self._cone_delta = 0.0
                self._cone_delta_t = -1e9
                self._cone_last_t = -1e9
                self._cone_last_la = 0.0
                self._cone_last_lat = 0.0
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
        if mode is Mode.CONE_DRIVE:
            # 관문이 '조향으로는 못 피한다' 고 보면 그만큼 늦춘다
            # (cone_guard_slow_at_m 주석 — 수평선이 v 에 비례한다)
            target *= self.cone_guard_speed_ratio()
        elif mode is Mode.LANE_DRIVE:
            # 통로가 앞에 보이면 **미리** 라바콘 속도로 내린다
            # (cone_approach_slow_far_m 주석의 실측 근거)
            target = min(target, self._cone_approach_speed(obs, target))
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
            return self._cone_guard(self._cone_steer(obs), obs), None

        if mode in (Mode.AVOID_STATIC, Mode.OVERTAKE):
            passer = (
                self.pass_moving if mode is Mode.OVERTAKE else self.pass_static
            )
            # 기동 중에는 넓은 코리도로 타겟을 계속 추적한다 — 기본 0.45m 는
            # 자기 횡이동(0.7m)만으로 타겟이 사라져 조기 복귀를 만든다(리뷰 C2).
            #
            # ★ 단, **방향을 정하는 첫 틱**(phase=IDLE)만은 FSM 과 같은 좁은
            #   기준으로 고른다 (0820 밤 감사). 트랙 차로 반폭이 0.41m 인데
            #   ±1.2m 는 노변까지 통째로 포함해서, 장애물(x=1.0, y=-0.21)과
            #   노변물체(x=0.6, y=+0.55)가 함께 있으면 **더 가까운 노변물체**를
            #   골라 side 부호가 뒤집힌다 = 장애물 쪽으로 비킨다(재현 확인).
            #   기동에 들어간 뒤의 추적 코리도는 그대로 1.2m 다.
            committed = passer.phase is not PassPhase.IDLE
            result = passer.update(
                target=obs.lidar.nearest_front(
                    max_lateral=(
                        passer.config.track_corridor_m
                        if committed
                        else cfg.fsm_corridor_half_m
                    )
                ),
                lane_offset_px=obs.lane.offset_px,
                lane_lookahead_px=obs.lane.lookahead_px,
                lane_side=obs.lane.lane_side,
                left_clear=obs.lidar.left_clear,
                right_clear=obs.lidar.right_clear,
                now=obs.now,
            )
            self._last_pass_done = result.done
            self._last_pass_aborted = result.aborted
            return self.pid.update(result.lateral_offset_px), result.speed

        # LANE_DRIVE (및 그 외 기본)
        if (
            ctx.race_started_at is not None
            and (obs.now - ctx.race_started_at) < cfg.start_straight_hold_s
        ):
            # 출발 직진 홀드 — 겐트리 지주 오검으로 인한 초반 급조향 차단
            return 0.0, None
        return self._lane_with_cone_approach(obs), None

    def _cone_approach_speed(self, obs: Observation, lane_target: float) -> float:
        """통로가 가까워질수록 목표속도를 라바콘 속도로 낮춘다.

        근거는 cone_approach_slow_far_m 주석. 요약: 실차는 구간 직전까지
        2.16 m/s 인데 진입 판정이 콘 ~50cm 에서 나고 감속에 0.52m 가 걸려
        첫 콘에 거의 2.16 m/s 로 닿는다. 빠를수록 지연(0.15초) 중 눈감고
        가는 거리가 늘어 진입각 오차가 그대로 횡이탈이 된다.
        """
        cfg = self.config
        if not cfg.cone_approach_slow_enabled:
            return lane_target
        cone = obs.cone
        if not cone.lidar_gate_valid or cone.lidar_cones < cfg.cone_approach_min_cones:
            return lane_target
        # ★ **아직 통로 밖일 때만** 줄인다. 옆에 콘이 붙어 있으면 이미
        #   통로 안이고, 그때는 CONE_DRIVE 의 속도 정책이 맡는다.
        #   이 조건이 없으면 통로 안에서도 계속 발동해 시간을 크게 잃는다
        #   (실측: 차선 프레임의 43~51%, 71~87초 중 18~19초).
        side = min(cone.lidar_near_left_m, cone.lidar_near_right_m)
        if side < cfg.cone_approach_outside_m:
            return lane_target
        d = cone.lidar_gate_depth_m
        if d <= 0.0 or d >= cfg.cone_approach_slow_far_m:
            return lane_target
        if d < cfg.cone_approach_slow_min_m:
            return lane_target          # 코앞 = 이미 진입 — CONE_DRIVE 가 맡는다
        far, near = cfg.cone_approach_slow_far_m, cfg.cone_approach_slow_near_m
        w = max(0.0, min(1.0, (far - d) / max(1e-3, far - near)))
        cone_target = cfg.speed.base.get("CONE_DRIVE", lane_target)
        return lane_target + (cone_target - lane_target) * w

    def _cone_approach_weight(self, obs: Observation) -> float:
        """통로가 앞에 보일 때 콘 조향을 섞을 비중 0~1 (설정 주석의 근거)."""
        cfg = self.config
        if not cfg.cone_approach_enabled:
            return 0.0
        cone = obs.cone
        if not cone.lidar_gate_valid:
            return 0.0
        if cone.lidar_cones < cfg.cone_approach_min_cones:
            return 0.0
        d = cone.lidar_gate_depth_m
        if d <= 0.0:
            return 0.0
        far, near = cfg.cone_approach_far_m, cfg.cone_approach_near_m
        if d >= far:
            return 0.0
        w = (far - d) / max(1e-3, far - near)
        return max(0.0, min(1.0, w)) * cfg.cone_approach_max_w

    def _lane_with_cone_approach(self, obs: Observation) -> float:
        """차선 조향에 **접근 중인 통로**를 섞는다.

        ★ 0821 밤 신설. 라바콘 접촉의 편차는 통로 안이 아니라 **진입각**에서
          온다. 실측(cone_0821_* 4개 bag, 콘 두 열 직선적합): 진입각이
          중앙 13.5°, **38%가 20° 초과**다. 그런데 R1.5 통로에서 20°를
          지우려면 차가 통로보다 조여 돌아야 하고(여유 1/1.02−1/1.5=
          0.313 rad/m) 1.11m 가 걸린다 — 그동안 19.3cm 밀리는데 쓸 수 있는
          여유는 18.5cm 뿐이다. **통로 안에서는 이미 늦다.**
          라이다는 통로를 진입 0.7~1.9초 전(76~95cm)에 이미 본다. 그 구간을
          차선만 보고 달리는 대신, 통로가 가까워질수록 통로 조향을 섞어
          **미리 정렬한다**.

        섞기만 하고 갈아타지 않는 이유: 접근 중 통로 추정이 틀리면 차선을
        벗어난다. 비중은 게이트 깊이로만 올리고 상한을 둔다.
        """
        lane = self._lane_steer(obs)
        w = self._cone_approach_weight(obs)
        if w <= 0.0:
            return lane
        cone = self._cone_guard(self._cone_steer(obs), obs)
        return (1.0 - w) * lane + w * cone

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
            now=obs.now,
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
        cone = self._extrapolate_cone(obs)
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
            la = self._cone_lookahead(cone.lidar_gate_depth_m, obs.now)
            # lidar lateral 은 좌측+ — pure_pursuit 의 y(좌측+)에 그대로 넣는다
            lat = self._cone_lat_track(
                self._cone_repel(cone, cone.lidar_gate_lateral_m), obs.now)
            self._remember_cone(la, lat, obs.now)
            return _clamp_cmd(pure_pursuit_cmd(la, lat, max_cmd=CMD_SATURATION)
                              + self._cone_pid(lat, obs.now))

        # 전방주시거리 = 상한과 '관측된 게이트 깊이' 중 작은 쪽 (하한 보장)
        # 깊이는 BEV 세로축이라 세로 축척으로 환산한다.
        px_per_m_fwd = max(1.0, cfg.px_per_m_fwd)
        depth_m = 0.0
        if cone.gate_valid and cone.gate_depth_px > 1.0:
            depth_m = cone.gate_depth_px / px_per_m_fwd
        lookahead = self._cone_lookahead(depth_m, obs.now)

        if cone.gate_valid:
            # 게이트 중앙(px)을 차량 기준 목표점(m)으로 바꿔 퓨어 퍼수트로 간다.
            lateral_m = cone.gate_center_px / px_per_m
            lat = self._cone_lat_track(self._cone_repel(cone, -lateral_m), obs.now)
            self._remember_cone(lookahead, lat, obs.now)
            return _clamp_cmd(pure_pursuit_cmd(lookahead, lat, max_cmd=CMD_SATURATION)
                              + self._cone_pid(lat, obs.now))

        if cone.left_cone_px > 0 or cone.right_cone_px > 0:
            # 단측: 보이는 줄에서 반폭만큼 떨어진 '가상 게이트 중앙'을 향한다.
            total = float(cone.left_cone_px + cone.right_cone_px)
            raw = (cone.left_cone_px - cone.right_cone_px) / max(1.0, total)
            # raw 는 프레임마다 ±1.0 로 뒤집히는 잡음이다 (설정 주석의 실측 참조).
            # 시간 평활을 거쳐야 좌우 최대락을 번갈아 치지 않는다.
            a = cfg.cone_bias_alpha
            self._cone_bias = a * raw + (1.0 - a) * self._cone_bias
            # bias>0 = 왼쪽에 콘이 많다 → 통로는 오른쪽 → 목표도 오른쪽
            lateral_m = self._cone_bias * self._cone_half_m
            lat = self._cone_repel(cone, -lateral_m)
            self._remember_cone(lookahead, lat, obs.now)
            return pure_pursuit_cmd(lookahead, lat, max_cmd=CMD_SATURATION)

        # ★ 게이트도 콘 픽셀도 없다. **차선은 보지 않는다** (cone_hold_s 주석).
        #   직전 콘 목표를 유지하고, 오래됐으면 통로를 따라 직진으로 푼다.
        age = obs.now - self._cone_last_t
        if age < 0.0 or self._cone_last_la <= 0.0:
            # 본 콘이 없어도 옆에 콘이 붙어 있으면 밀어내기는 해야 한다
            lat = self._cone_repel(cone, 0.0)
            if abs(lat) < 1e-3:
                return 0.0
            return pure_pursuit_cmd(cfg.cone_lookahead_min_m, lat,
                                    max_cmd=CMD_SATURATION)
        if age <= cfg.cone_hold_s:
            decay = 1.0                     # 유지
        else:
            span = max(1e-3, cfg.cone_straighten_s)
            decay = max(0.0, 1.0 - (age - cfg.cone_hold_s) / span)
        return pure_pursuit_cmd(
            self._cone_last_la,
            self._cone_repel(cone, self._cone_last_lat * decay),
            max_cmd=CMD_SATURATION,
        )

    def _cone_lookahead(self, gate_depth_m: float,
                        now: Optional[float] = None) -> float:
        """전방주시 = **관측된 게이트 깊이** (상·하한만 둔다).

        커브에서 더 줄이지 않는 이유는 설정 주석 참조 — 게이트 횡편차가
        이미 통로 곡률을 담고 있어, 깊이를 그대로 쓰면 퓨어 퍼수트가 통로
        반경을 정확히 재현한다.

        ★ 목표값을 바로 따르지 않고 cone_lookahead_rate_mps 로 제한해
          따라간다 (설정 주석의 실측 근거).
        """
        cfg = self.config
        target = (max(cfg.cone_lookahead_min_m, min(cfg.cone_lookahead_m, gate_depth_m))
                  if gate_depth_m > 0.0 else cfg.cone_lookahead_min_m)
        if now is None:
            return target
        if self._cone_la <= 0.0 or now < self._cone_la_t:
            self._cone_la, self._cone_la_t = target, now
            return target
        dt = max(0.0, min(0.5, now - self._cone_la_t))
        self._cone_la_t = now
        step = cfg.cone_lookahead_rate_mps * dt
        delta = target - self._cone_la
        self._cone_la += max(-step, min(step, delta))
        return self._cone_la

    def cone_guard_speed_ratio(self) -> float:
        """관문이 본 여유로 속도를 깎는 배율 (cone_guard_slow_at_m 주석)."""
        cfg = self.config
        gap = self._cone_guard_gap
        if not cfg.cone_guard_enabled or gap >= cfg.cone_guard_margin_m:
            return 1.0
        lo = cfg.cone_guard_slow_at_m
        hi = cfg.cone_guard_margin_m
        t = (gap - lo) / max(1e-3, hi - lo)
        t = max(0.0, min(1.0, t))
        return cfg.cone_guard_min_ratio + (1.0 - cfg.cone_guard_min_ratio) * t

    def _cone_guard(self, steer: float, obs: Observation) -> float:
        """마지막 관문 — 보이는 콘을 관통하는 조향은 내보내지 않는다.

        ★ 0821 밤 신설. 오늘 라바콘 결함 다섯 개를 고쳤는데도 주행마다
          결과가 갈렸다(어떤 주행은 통과, 어떤 주행은 접촉). 추정을 전부
          맞히는 방식으로는 일관성이 안 나온다 — 게이트가 한 프레임만
          틀려도 그 순간 명령이 콘으로 향한다.
          그래서 조향을 내보내기 직전에 **콘 좌표로 직접** 검사하고,
          관통하면 가장 가까운 안전한 값으로 바꾼다. 게이트·통로 추정이
          무엇을 말하든 이 검사는 독립적이다.

        수평선은 v × 0.35초 — 그 안에서만 등속곡률 가정이 유효하다.
        """
        cfg = self.config
        if not cfg.cone_guard_enabled:
            return steer
        pts = getattr(self._extrapolate_cone(obs), "lidar_cone_pts", ())
        if not pts:
            self._cone_guard_gap = 9.9
            return steer
        safe, gap = clamp_steer_for_cones(
            steer, pts,
            speed_mps=max(0.2, float(obs.ego_speed_mps)),
            margin_m=cfg.cone_guard_margin_m,
            react_s=cfg.cone_guard_react_s,
        )
        self._cone_guard_gap = gap
        if abs(safe - steer) > 0.01:
            self._cone_guard_hits += 1
        return safe

    def _wheel_angle(self, now: float) -> float:
        """지연을 통과한 **실제 바퀴각** [rad]. 발행 조향은 0.10초 앞선다."""
        cfg = self.config
        target = -self._last_steer * CMD_TO_RAD
        dt = 0.0 if self._cone_delta_t < -1e8 else max(0.0, min(0.3, now - self._cone_delta_t))
        self._cone_delta_t = now
        if dt <= 0.0:
            self._cone_delta = target
        else:
            lag = max(1e-3, cfg.cone_extrap_delta_tau_s)
            self._cone_delta += (target - self._cone_delta) * (1.0 - math.exp(-dt / lag))
        return self._cone_delta

    def _extrapolate_cone(self, obs: Observation):
        """낡은 라이다 관측을 자차 운동만큼 현재 좌표로 옮긴다.

        근거는 cone_extrapolate_enabled 주석. 라이다 9Hz / 제어 50Hz 라
        스캔 사이 관측이 최대 110ms 낡고, 그 낡음이 곧 조향 지연이다.
        """
        cfg = self.config
        cone = obs.cone
        age = float(getattr(cone, "lidar_gate_age_s", 0.0))
        if not cfg.cone_extrapolate_enabled or age <= 1e-3:
            return cone
        if age > cfg.cone_extrapolate_max_s:
            return cone                     # 너무 낡았다 — 외삽도 못 믿는다
        v = max(0.0, float(obs.ego_speed_mps))
        if v <= 0.01:
            return cone
        delta = self._wheel_angle(obs.now)
        ds = v * age
        dth = (v / WHEELBASE_M) * math.tan(delta) * age
        cs, sn = math.cos(-dth), math.sin(-dth)

        def move(x, y):
            qx, qy = x - ds, y
            return qx * cs - qy * sn, qx * sn + qy * cs

        gx, gy = move(cone.lidar_gate_depth_m, cone.lidar_gate_lateral_m)
        pts = tuple(move(px, py) for (px, py) in (cone.lidar_cone_pts or ()))
        nl = min((abs(py) for (px, py) in pts
                  if py > 0.05 and 0.15 <= px <= 0.45), default=float("inf"))
        nr = min((abs(py) for (px, py) in pts
                  if py < -0.05 and 0.15 <= px <= 0.45), default=float("inf"))
        return replace(
            cone,
            lidar_gate_depth_m=max(0.0, gx),
            lidar_gate_lateral_m=gy,
            lidar_cone_pts=pts,
            lidar_near_left_m=nl,
            lidar_near_right_m=nr,
            lidar_gate_age_s=0.0,
        )

    def _cone_pid(self, err_m: float, now: float) -> float:
        """추종오차에 대한 P·D 기여분 [cmd] (설정 주석의 근거).

        err_m 은 목표 횡위치다 — 차는 자기 좌표 원점이므로 그대로 오차다.
        부호: 목표가 왼쪽(+)이면 좌회전(cmd 음수)이어야 하므로 뒤집는다.
        """
        cfg = self.config
        if cfg.cone_pid_kp == 0.0 and cfg.cone_pid_kd == 0.0:
            return 0.0
        # ★ 전용 시각을 쓴다. _cone_lat_track 이 _cone_lat_t 를 먼저 갱신하므로
        #   그걸 쓰면 dt 가 항상 0 이 되어 **미분항이 죽는다** (초기 구현 버그).
        dt = now - self._cone_err_t if self._cone_err_t > -1e8 else 0.0
        self._cone_err_t = now
        if 1e-4 < dt <= 0.5:
            a = cfg.cone_pid_d_alpha
            raw = (err_m - self._cone_err_prev) / dt
            self._cone_err_rate = a * raw + (1.0 - a) * self._cone_err_rate
        self._cone_err_prev = err_m
        out = -(cfg.cone_pid_kp * err_m + cfg.cone_pid_kd * self._cone_err_rate)
        return max(-cfg.cone_pid_max, min(cfg.cone_pid_max, out))

    def _cone_lat_track(self, target_m: float, now: float) -> float:
        """콘 목표 횡위치를 변화율 제한으로 따라간다 (설정 주석의 근거)."""
        cfg = self.config
        if self._cone_lat_t < -1e8 or now < self._cone_lat_t:
            self._cone_lat, self._cone_lat_t = target_m, now
            return target_m
        dt = max(0.0, min(0.5, now - self._cone_lat_t))
        self._cone_lat_t = now
        step = cfg.cone_lat_rate_mps * dt
        self._cone_lat += max(-step, min(step, target_m - self._cone_lat))
        return self._cone_lat

    def _cone_repel(self, cone, lateral_m: float) -> float:
        """차 옆 콘이 안전선보다 가까우면 목표를 반대쪽으로 민다.

        cone_safe_clear_m 주석에 실측 근거. 좌측(+y)이 가까우면 목표를 우(−)로,
        우측(−y)이 가까우면 목표를 좌(+)로 민다.
        """
        cfg = self.config
        safe = cfg.cone_safe_clear_m
        push = 0.0
        nl = cone.lidar_near_left_m
        nr = cone.lidar_near_right_m
        if nl < safe:
            push -= (safe - nl) * cfg.cone_repel_gain      # 왼쪽이 가깝다 → 우로
        if nr < safe:
            push += (safe - nr) * cfg.cone_repel_gain      # 오른쪽이 가깝다 → 좌로
        push = max(-cfg.cone_repel_max_m, min(cfg.cone_repel_max_m, push))
        return lateral_m + push

    def _remember_cone(self, la: float, lateral_m: float, now: float) -> None:
        """마지막으로 본 콘 목표를 기억한다 (게이트 끊김 대비)."""
        self._cone_last_la = float(la)
        self._cone_last_lat = float(lateral_m)
        self._cone_last_t = float(now)


__all__ = ["PolicyConfig", "DrivePolicy"]

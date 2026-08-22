"""조향 계산 — 퓨어 퍼수트와 픽셀-오차 PID.

자이카 축거(wheelbase)는 0.30m 이다 (자이카 ROS2 미니PC Y모델).
퓨어 퍼수트를 쓰는 이유: 픽셀 오차 PID 는 저속에선 잘 듣지만 속도가 올라가면
같은 게인이 과조향을 만든다. 4분 제한(규정 p.37) 안에 3바퀴를 넣으려면 고속
구간이 필수라 속도에 따라 전방주시거리가 늘어나는 기하 기반 제어가 유리하다.

조향 부호: 음수 = 좌회전, 양수 = 우회전 (자이카 실차 규약).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence, Tuple

WHEELBASE_M = 0.30
# motor_adapter(팀원 kobot_vehicle) 실측 체인: δ[rad] = cmd × 0.0068,
# 이후 vesc.yaml servo 0.15~0.85 가 δ 를 ±0.288rad(16.5°)로 자른다.
CMD_TO_RAD = 0.0068
# 서보 하드스톱 [rad] — vesc.yaml servo 0.15~0.85 → ±0.288rad(16.5°)
SERVO_MAX_RAD = 0.288
# 서보 하드스톱: vesc.yaml servo 0.15~0.85 → δ ±0.288rad(16.5°) → cmd ±42.3.
# 이 위로 올려 봐야 바퀴는 더 안 꺾이고, 감속 로직만 과하게 자극한다.
CMD_SATURATION = 42.0
CAR_HALF_M = 0.15        # 차 반폭 (실측 차폭 30cm)
CONE_RADIUS_M = 0.075    # 콘 밑면 반지름 (실측 지름 15cm)
# ★ 차체는 점도 원반도 아니다 (0821 밤). 기준점은 **뒷차축**이고 차체는
#   그 앞뒤로 뻗어 있다. 돌 때 **앞 바깥 코너가 가장 크게 쓸고 나간다**:
#       R=1.0m 에서  안쪽 뒷모서리 0.85m,  바깥 앞코너 √(1.15²+0.40²)=1.22m
#       → 실효 폭 37cm (차폭 30cm 보다 7cm 넓다)
#   원반(반지름 0.15) 모델은 이 앞오버행 스윙아웃을 통째로 놓쳐서,
#   "여유 있다"고 판단한 조향이 실제로는 콘을 친다.
CAR_FRONT_M = 0.40       # 뒷차축에서 앞범퍼까지 (축거 0.30 + 앞오버행 0.10)
CAR_REAR_M = 0.12        # 뒷차축에서 뒷범퍼까지


def path_clearance(
    cmd: float,
    cones: Sequence[Tuple[float, float]],
    *,
    horizon_m: float,
    wheelbase: float = WHEELBASE_M,
) -> float:
    """이 조향으로 갈 때 콘까지 최소 여유 [m]. 음수면 부딪힌다.

    차체를 반폭 CAR_HALF_M 의 원반으로 근사해 호를 따라 쓸어본다.
    수평선(horizon_m)은 **명령이 실제로 유지되는 시간**만큼만 잡는다 —
    그 이상은 어차피 다시 조향하므로 등속곡률 가정이 깨진다.
    """
    if not cones:
        return 9.9
    delta = max(-SERVO_MAX_RAD, min(SERVO_MAX_RAD, cmd * CMD_TO_RAD))
    # 뒷차축 궤적 + 그 지점에서의 차체 방향
    poses = []
    s = 0.05
    if abs(delta) < 1e-4:
        while s <= horizon_m:
            poses.append((s, 0.0, 0.0)); s += 0.05
    else:
        radius = wheelbase / math.tan(abs(delta))
        sgn = -1.0 if cmd > 0.0 else 1.0        # +cmd = 우회전
        while s <= horizon_m:
            th = s / radius
            poses.append((radius * math.sin(th),
                          sgn * radius * (1.0 - math.cos(th)),
                          sgn * th))
            s += 0.05
    if not poses:
        return 9.9
    best = 9.9
    for (cx, cy) in cones:
        for (px, py, pth) in poses:
            # 콘을 차체 좌표로 옮겨 **사각형**까지의 거리를 잰다
            dx, dy = cx - px, cy - py
            c, sn = math.cos(pth), math.sin(pth)
            bx = dx * c + dy * sn
            by = -dx * sn + dy * c
            ox = max(-CAR_REAR_M - bx, bx - CAR_FRONT_M, 0.0)
            oy = max(abs(by) - CAR_HALF_M, 0.0)
            gap = math.hypot(ox, oy) - CONE_RADIUS_M
            if gap < best:
                best = gap
                if best < -0.5:
                    return best
    return best


def clamp_steer_for_cones(
    cmd: float,
    cones: Sequence[Tuple[float, float]],
    *,
    speed_mps: float,
    margin_m: float = 0.03,
    react_s: float = 0.35,
    min_horizon_m: float = 0.25,
    max_cmd: float = CMD_SATURATION,
    step: float = 2.0,
) -> Tuple[float, float]:
    """콘을 관통하는 조향이면, **가장 가까운 안전한 조향**으로 바꾼다.

    ★ 0821 밤 신설. 오늘 라바콘 결함을 다섯 개 고쳤는데도 주행마다
      결과가 갈렸다 — 어떤 주행은 통과하고 어떤 주행은 박았다.
      추정(게이트·곡률·전방주시)을 전부 맞히는 방식으로는 일관성이 안 나온다.
      그래서 **마지막 단계가 충돌 명령 자체를 못 내게** 막는다.
      게이트가 틀려도, 통로 추정이 틀려도, 보이는 콘을 관통하는 명령은
      나가지 않는다.

    수평선은 v × react_s (구동지연 0.04 + 서보 τ 0.06 + 여유)로 잡는다.
    그 안에서는 등속곡률 가정이 유효하다.

    반환: (안전한 조향, 그때의 여유 [m])
    """
    if not cones:
        return cmd, 9.9
    horizon = max(min_horizon_m, abs(speed_mps) * react_s)
    base = path_clearance(cmd, cones, horizon_m=horizon)
    if base >= margin_m:
        return cmd, base
    # 원 명령에서 가까운 순서로 훑어 처음으로 안전해지는 값을 쓴
    best_cmd, best_gap = cmd, base
    k = 1
    while k * step <= 2.0 * max_cmd:
        for cand in (cmd - k * step, cmd + k * step):
            if cand < -max_cmd or cand > max_cmd:
                continue
            gap = path_clearance(cand, cones, horizon_m=horizon)
            if gap >= margin_m:
                return cand, gap
            if gap > best_gap:
                best_cmd, best_gap = cand, gap
        k += 1
    return best_cmd, best_gap


def pure_pursuit_steer(
    target_x: float,
    target_y: float,
    *,
    wheelbase: float = WHEELBASE_M,
    max_steer: float = 50.0,
) -> float:
    """차량 기준 목표점(x 전방+, y 좌측+)을 향하는 조향각 [deg].

    δ = atan2(2·L·sin(α), Ld),  α = 목표점 방위각, Ld = 목표점까지 거리.
    y>0(좌측)이면 좌회전이어야 하므로 부호를 뒤집어 반환한다.
    """
    ld_sq = target_x * target_x + target_y * target_y
    if ld_sq < 1e-6:
        return 0.0
    ld = math.sqrt(ld_sq)
    alpha = math.atan2(target_y, target_x)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
    steer = -math.degrees(delta)
    return max(-max_steer, min(max_steer, steer))


def pure_pursuit_cmd(
    target_x: float,
    target_y: float,
    *,
    wheelbase: float = WHEELBASE_M,
    max_cmd: float = CMD_SATURATION,
) -> float:
    """퓨어 퍼수트 조향을 **모터 cmd 단위**로 낸다.

    ★ 0821: 이 함수가 없어서 라바콘 구간만 조향의 39%밖에 안 걸리고 있었다.

    /xycar_motor 로 나가는 값은 **각도가 아니라 무차원 cmd** 다 —
    motor_adapter 가 `steering_angle = -(cmd × 0.0068)` 로 rad 을 만든다
    (real/ros2_ws/.../motor_adapter.py). 차선 경로는 처음부터 cmd 단위였고
    (LaneController.k_curvature 가 WHEELBASE_M/CMD_TO_RAD 를 곱한다),
    분기 조향 -34 도 cmd 다. 그런데 라바콘 경로만 pure_pursuit_steer 의
    **도(deg)** 값을 환산 없이 그대로 발행하고 있었다.

    결과: δ_실제 = 발행값 × 0.3896° 라 **의도한 조향의 1/2.567** 만 걸린다.
        발행 -22.1 → 바퀴 8.61° → 선회반경 1.98m
        발행 -10.9 → 바퀴 4.25° → 선회반경 4.04m
    통로가 R 1.0~1.5m 로 굽는데 차는 R 2~4m 짜리 원을 그렸다 — 사용자가
    본 "커브인데 그대로 직진해서 콘에 부딪힌다"가 이것이다.
    ±50 클램프도 **도 단위**라 δ=50° 가 필요해야 걸린다: 실주행 로그의
    '조향 포화 0회'는 안정의 증거가 아니라 이 버그의 지문이었다.

    참고 (cmd 단위로 바로잡은 뒤의 필요값):
        R 2.0m → 21.9    R 1.5m → 29.0    R 1.2m → 36.0
        R 1.0m → 42.9 (서보 한계 42.3 바로 위)   R 0.8m → 52.8 (불가)
    """
    ld_sq = target_x * target_x + target_y * target_y
    if ld_sq < 1e-6:
        return 0.0
    ld = math.sqrt(ld_sq)
    alpha = math.atan2(target_y, target_x)
    delta = math.atan2(2.0 * wheelbase * math.sin(alpha), ld)   # [rad]
    cmd = -delta / CMD_TO_RAD
    return max(-max_cmd, min(max_cmd, cmd))


@dataclass
class PixelPID:
    """BEV 픽셀 오차용 PID. 차선주행 기본 제어.

    적분항은 기본으로 꺼 둔다(Ki=0). 켤 거면 반드시 anti-windup 을 함께 켤 것 —
    픽셀 오차는 차선을 놓친 프레임에 크게 튀어서 적분이 순식간에 폭주한다.
    """

    # ★ kp 0.16 → 0.34 (0817 첫 자율주행 실측): 1랩 로그 129샘플 중 |조향|≥25
    #   가 2건, 최대 19 — 물리 최대(cmd≈42, 바퀴 16.5°, 회전반경 1.08m)의
    #   절반도 못 써 코너(반경 ~1m)에서 언더스티어 → 차선 이탈. 코너에서는
    #   포화까지 가야 하므로 실측 오프셋(~120px)에서 cmd 40 이 나오는 값.
    #   직선 소오프셋(10~30px)엔 3~10° 라 진동 위험 낮음 (스무더 5창+420°/s).
    # ★ 0820 실주행 + lanesim 재튜닝.
    #   실주행(run_0820_0019) 실측: |offset|>150px 인 프레임이 51.9%. 옛 kp 0.34
    #   에서 offset 150px 는 cmd 51 로 max_steer 50 을 넘겨 **주행의 절반 이상
    #   조향이 포화**했다 — 좌우 최대락을 번갈아 치는 bang-bang 이 되고, 실측
    #   1.22Hz 좌우 왕복·차선 이탈의 직접 원인이다.
    #   lanesim(지연 모델 포함, 긴 직선 30m + 초기 횡오차 0.18m + 코너 R=1.0m)
    #   스윕 결과 — 코너 최대오차가 kp 에 단조 의존한다:
    #     kp 0.34 → 0.344m (반차선 0.41m 의 84%, 사실상 이탈)
    #     kp 0.24 → 0.207m / kp 0.20 → 0.130m / kp 0.16 → 0.110m
    #   직선 정상상태 오차는 어느 조합에서도 1cm 내로 수렴하므로, 코너 여유가
    #   가장 크게 벌리는 쪽을 택한다.
    #
    #   ★ 0820 밤 2차 조정: 0.20 → 0.16.
    #   (1) BEV 축척 스윕(±13%, 60 시나리오): 이탈이 0.34→9, 0.20→7, **0.16→5**.
    #       특히 축척 920 에서 0.20 은 **연속 S자(슬라럼) 이탈**이 새로 생겼는데
    #       (0.34 보다 나쁨) 0.16 은 슬라럼 이탈이 0 이다. 슬라럼은 라바콘 구간과
    #       같은 형태라 이 항목이 중요하다.
    #   (2) 최악 코너 R=0.8m @0.6m/s: 0.20 은 이탈(0.415m), **0.16 은 완주(0.380m)**.
    #   실패 원인이 굼뜸이 아니라 오버슈트·진동이라 낮출수록 좋아진다.
    #   더 낮춰도(0.14/0.12) 이득이 미미해 응답성만 버리므로 0.16 에서 멈춘다.
    #   ※ R=0.8m @1.0m/s 는 어떤 게인으로도 이탈한다(0.42~0.44m vs 반차선 0.4125m).
    #     그 속도로 그 반경은 물리적으로 불가능하다 — 게인 문제가 아니다.
    kp: float = 0.16
    ki: float = 0.0
    kd: float = 0.10
    deadzone_px: float = 10.0
    max_steer: float = 50.0
    integral_limit: float = 400.0

    _prev: float = 0.0
    _integral: float = 0.0

    def reset(self) -> None:
        self._prev = 0.0
        self._integral = 0.0

    def update(self, error_px: float) -> float:
        if abs(error_px) < self.deadzone_px:
            error_px = 0.0

        self._integral += error_px
        # anti-windup: 적분항을 물리적으로 의미 있는 범위에 가둔다.
        self._integral = max(
            -self.integral_limit, min(self.integral_limit, self._integral)
        )
        derivative = error_px - self._prev
        self._prev = error_px

        steer = self.kp * error_px + self.ki * self._integral + self.kd * derivative
        return max(-self.max_steer, min(self.max_steer, steer))


@dataclass
class LaneController:
    """차선주행 조향 = 곡률 피드포워드 + 방향 정렬 + 횡오차 되먹임.

    왜 3항인가 (0817 첫 자율주행 실패 분석):
      횡오차만 보는 P 제어는 정상 선회 중 구조적으로 언더스티어한다. 코너에서
      필요한 조향각은 오차가 아니라 **곡률**이 정하는데(δ = atan(L·κ)), P 항은
      차가 바깥으로 밀려나 오차가 쌓여야 비로소 그만큼을 만들어 낸다. 그
      '밀려나는 양'이 곧 차선 이탈이었다 (실측 최대 조향 19 / 물리 최대 42).

      · 피드포워드(κ): 코너에 필요한 조향을 오차 없이 미리 만든다.
      · 방향 정렬(slope): 차가 차선과 어긋난 각을 편다. Stanley 의 heading 항.
      · 횡오차(P/D): 남은 중심 이탈을 천천히 지운다.

    단위: 입력은 전부 BEV 픽셀 기준(무차원 slope, 1/px 곡률)이고 출력은
    자이카 조향 명령값(cmd, 42 부근에서 서보 포화)이다.
    """

    # ── 곡률 피드포워드 게인은 '고정 상수'가 아니라 매 프레임 유도된다 ──
    # 필요 조향: cmd = δ_rad/0.0068,  δ_rad ≈ 축거[m] × κ_real[1/m]
    # BEV 픽셀 곡률 → 실제 곡률: κ_real = κ_px × s_fwd² / s_lat
    #   s_lat [px/m] 은 **매 프레임 측정된 차선폭**으로 구한다 (자가 보정!):
    #       s_lat = lane_width_px / road_width_m
    #   s_fwd 는 직접 못 재므로 종횡비 bev_aspect = s_fwd/s_lat 로 둔다.
    #   ⇒ k = (축거/0.0068) × s_lat × bev_aspect² × ff_scale
    # 이렇게 하면 px_per_m 실측이 없어도 축척 변화에 자동 추종한다 —
    # 0817 시뮬 스윕에서 축척 420~620 px/m 전 구간 통과의 근거.
    # 0818 오후 사용자 실측: 전폭(흰선↔흰선) 0.80~0.85m, 반차선(흰↔노랑)
    # 0.40~0.42m. 중앙값 채택 — 곡률 FF 게인의 축척이 이 값에 정비례한다.
    road_width_m: float = 0.825
    bev_aspect: float = 0.90         # s_fwd/s_lat — 호모그래피 종횡비
    # 0817 시뮬 실측: 폴리핏 곡률이 기하 예측보다 ~1.6배 크게 나온다(BEV
    # 종횡비 미실측 + 엣지 기반 이진화의 영향). 과대 피드포워드는 코너 진입
    # 과조향이 되므로 보수적으로 잡고 부족분은 방향·횡오차 항에 맡긴다.
    ff_scale: float = 0.55           # 안정 여유
    # 기울기 → 조향 게인. slope 0.2(≈11° 어긋남) 에서 cmd 16 정도가 나온다.
    # ★ 0818 낮 회귀 교훈: 아침에 80→50 으로 줄이고 신뢰 게이트를 기울기에도
    #   걸었더니 시뮬 스윕이 S자 발산(36~57m)으로 무너졌고 실차도 4·5차에서
    #   3차에 성공했던 코너까지 전부 실패했다. 방향 정렬은 이 제어기의 척추다
    #   — 80 으로 복원한다. t=62~70s 의 '반대로 꺾기'는 게인이 아니라 검출
    #   붕괴(노란 중앙선 미사용 + 유리창 반사)가 원인이었다 (프레임 목시).
    k_heading: float = 80.0
    # ── (철회) 검출 품질 게이트 ──
    # 0818 아침에 '폭이 이상하면 FF·기울기를 끈다' 게이트를 넣었다가 같은 날
    # 철회했다. 실차 4·5차에서 3차에 성공한 코너까지 전부 무너졌고, 시뮬
    # 스윕도 S자 발산(36~59m)으로 재현됐다 — 게이트가 꺼지는 순간 제어기가
    # 순수 P 로 강등돼 언더스티어가 재발하는 구조였기 때문이다. 곡률 오염의
    # 진짜 원인(콘 주황이 차선 이진화에 새는 것)은 lane_detector 의 주황
    # 제외 마스크로 근본 차단했다. 같은 게이트를 다시 넣고 싶어지면 이 주석을
    # 먼저 읽을 것: 잡음은 입력(지각)에서 잡아야지 출력(제어)에서 자르면
    # 제어기의 척추가 부러진다.
    # 곡률·기울기는 짧은 BEV 창의 2차 폴리핏이라 잡음이 크다 — EMA 로 눌러
    # 쓴다. 0.35 는 30fps 에서 시정수 ≈0.1s (코너 진입 지연 무시 가능).
    ema_alpha: float = 0.35
    # 피드포워드 항의 절대 상한 [cmd]. 곡률 오검(차선 놓침 직후 폴리핏 튐)이
    # 한 프레임에 풀락을 때리는 것을 막는다.
    ff_limit: float = 45.0
    max_steer: float = 50.0

    # ── 아펙스 커팅 (0817 시뮬레이터로 확정한 필수 기능) ──────────────
    # 이 차의 최대 바퀴각은 16.5°(vesc.yaml servo 0.15~0.85) 이고 축거가
    # 0.30m 이므로 **최소 회전반경이 1.01m** 다. 즉 반경 1.0m 이하 코너에서는
    # 차선 중심선을 따라가는 것이 물리적으로 불가능하다 (필요 16.7° > 16.5°).
    # 유일한 해법은 주행로 폭을 써서 실효 반경을 키우는 것 — 코너에서 목표를
    # 안쪽으로 밀어 레이싱 라인을 그린다. 시뮬 실측: R=1.0m 코너 최대 이탈
    # 0.56m(이탈) → 0.11m(주행로 안), R=0.8m 1.15m → 0.24m.
    #
    # ★ 발동 조건은 '곡률 추정치'가 아니라 **실제 조향 포화**로 판단한다.
    #   BEV 축척(px/m)이 미실측이라 곡률의 절대값은 최대 1.6배까지 틀릴 수
    #   있는데(0817 시뮬 실측), 그걸 기준으로 삼으면 멀쩡한 코너에서 아펙스가
    #   켜져 오히려 이탈한다. 반면 '조향을 끝까지 썼는데도 부족하다'는 사실은
    #   축척과 무관하게 참이다 — 그때만, 그리고 그것이 지속될 때만 판다.
    apex_gain: float = 0.55          # 반차선폭의 몇 배까지 안쪽으로 붙일까
    #   (0.55 × 0.4m ≈ 0.22m — 주행로 여유 0.4m 의 절반만 쓴다)
    apex_engage: float = 0.80        # 포화율(느린 EMA)이 이 이상일 때 발동
    cmd_saturation: float = 42.0     # 서보 포화 cmd (16.5° / 0.0068)
    # 차선폭을 못 읽었을 때 쓸 기본 반차선폭 [px]
    default_half_lane_px: float = 225.0
    # 아펙스 목표가 한 번에 튀지 않도록 하는 변화율 제한 [px/프레임].
    # S자 변곡점에서 목표가 좌↔우로 계단 이동하면 그대로 진동이 된다.
    bias_rate_px: float = 12.0
    # ★ 아펙스 판단은 **느린** EMA 로 한다 (시정수 ≈0.4s). 지속적인 한쪽
    # 코너에서만 켜지고, 좌우가 번갈아 오는 S자·슬라럼에서는 평균이 0 으로
    # 상쇄돼 저절로 꺼진다 — 슬라럼은 안쪽에 붙는 게 아니라 가운데로 지나야
    # 하므로 이게 물리적으로도 옳다 (0817 시뮬: 연속 S자 이탈 0.90m → 통과).
    apex_ema_alpha: float = 0.08
    # ★ 아펙스 폭주 차단 (0820 저녁 실측). 발동 조건이 '출력 포화율' 하나뿐이라
    #   양의 되먹임이 성립한다 — 가짜 목표로 0.7초만 포화하면 _sat_slow 가
    #   0.80 을 넘고, 그때 더해지는 bias 184px 는 그 자체로 0.16×184 = 29cmd
    #   (포화한계 42 의 70%)라 검출이 정상으로 돌아와도 포화가 안 풀린다.
    #   run_0820_1821 실측 재생: 발동 968/6508 = 14.9%, t144.03~149.22 에
    #   **5.20초 연속 풀락**. 두 겹으로 막는다.
    #
    #   (1) 곡률 부호 일치 — 진짜 코너면 목표선 곡률이 같은 쪽을 가리킨다.
    #       곡률 '크기'는 요구하지 않는다(짧은 피팅에서 과소추정되므로).
    apex_require_curv_sign: bool = True
    #   (2) 연속 발동 시간 상한 [s] — 넘으면 bias 를 0 으로 되돌리고, 포화가
    #       한 번 풀릴 때까지 재발동을 막는다. 트랙 최악 코너(R≈0.8m)를 도는
    #       데 필요한 시간이 약 1.6초라 2.5초면 진짜 코너는 안 자른다.
    apex_max_engage_s: float = 2.5

    pid: PixelPID = field(default_factory=PixelPID)

    _curv_ema: float = 0.0
    _sat_slow: float = 0.0
    _apex_since: float = -1.0        # 아펙스 연속 발동 시작 시각 (미발동 -1)
    _apex_locked: bool = False       # 상한 초과로 잠긴 상태
    _slope_ema: float = 0.0
    _bias_px: float = 0.0
    _primed: bool = False

    def reset(self) -> None:
        self.pid.reset()
        self._curv_ema = 0.0
        self._sat_slow = 0.0
        self._apex_since = -1.0
        self._apex_locked = False
        self._slope_ema = 0.0
        self._bias_px = 0.0
        self._primed = False

    def k_curvature(self, lane_width_px: float) -> float:
        """측정된 차선폭으로 곡률→조향 게인을 유도한다 (클래스 주석 참조)."""
        half = (
            lane_width_px / 2.0 if lane_width_px > 1.0 else self.default_half_lane_px
        )
        s_lat = (2.0 * half) / max(0.1, self.road_width_m)
        return (
            (WHEELBASE_M / CMD_TO_RAD)
            * s_lat
            * (self.bev_aspect ** 2)
            * self.ff_scale
        )

    def update(
        self,
        offset_px: float,
        heading_slope: float,
        curvature_px: float,
        lane_width_px: float = 0.0,
        both_found: bool = True,   # (미사용) 게이트 철회 후 호환용 — 주석 참조
        now: float = 0.0,          # 아펙스 연속 발동 시간 상한용 [s]
    ) -> float:
        if not self._primed:
            self._curv_ema = curvature_px
            self._slope_ema = heading_slope
            self._primed = True
        else:
            a = self.ema_alpha
            self._curv_ema = a * curvature_px + (1.0 - a) * self._curv_ema
            self._slope_ema = a * heading_slope + (1.0 - a) * self._slope_ema


        k = self.k_curvature(lane_width_px)
        ff = max(-self.ff_limit, min(self.ff_limit, k * self._curv_ema))
        head = self.k_heading * self._slope_ema

        # 아펙스 커팅: 중심선으로는 못 도는 코너에서 목표를 안쪽으로 옮긴다.
        # tight ∈ [-1,1] — +면 우코너(안쪽=오른쪽).
        # 지속 포화율 — '조향을 끝까지 썼는데도 부족한 상태'가 얼마나 이어졌나.
        tight = self._sat_slow
        over = abs(tight) - self.apex_engage
        half_lane = (
            lane_width_px / 2.0 if lane_width_px > 1.0 else self.default_half_lane_px
        )
        # ── 아펙스 폭주 차단 (apex_max_engage_s 주석 참조) ──
        if over > 0.0 and self.apex_require_curv_sign:
            # 곡률이 사실상 0 인 프레임은 방향 정보가 없다고 보고 통과시킨다
            # (짧은 피팅에서 곡률은 자주 0 에 가깝다 — 여기서 막으면 진짜
            #  코너의 아펙스를 놓친다). 부호가 **반대**일 때만 거부한다.
            if self._curv_ema * tight < 0.0:
                over = -1.0
        if over > 0.0:
            if self._apex_since < 0.0:
                self._apex_since = now
            elif (
                not self._apex_locked
                and now - self._apex_since > self.apex_max_engage_s
            ):
                self._apex_locked = True
        else:
            self._apex_since = -1.0
        if self._apex_locked:
            # 포화가 한 번이라도 풀려야 잠금을 해제한다 — 안 그러면 bias 만
            # 0 으로 갔다가 곧바로 같은 되먹임으로 재발동한다.
            if abs(self._sat_slow) < self.apex_engage:
                self._apex_locked = False
            else:
                over = -1.0
        if over > 0.0:
            span = max(1e-6, 1.0 - self.apex_engage)
            target_bias = (
                self.apex_gain
                * min(1.0, over / span)
                * (1.0 if tight > 0 else -1.0)
                * half_lane
            )
        else:
            target_bias = 0.0
        # 변화율 제한 — S자 변곡점에서 목표가 계단 이동하면 진동이 된다.
        step = max(-self.bias_rate_px, min(self.bias_rate_px, target_bias - self._bias_px))
        self._bias_px += step

        # 목표선이 차 오른쪽에 보이면(=차가 왼쪽에 있으면) offset 이 +다.
        # 우코너에서 차를 오른쪽(안쪽)으로 붙이려면 목표 offset 이 -bias 이므로
        # 오차에 +bias 를 더한다.
        fb = self.pid.update(offset_px + self._bias_px)

        steer = ff + head + fb
        # 물리 포화(cmd 42)를 넘겨 봐야 바퀴는 안 꺾이고 하류 로직만 오해한다.
        lim = min(self.max_steer, self.cmd_saturation)
        steer = max(-lim, min(lim, steer))
        # 다음 프레임의 아펙스 판단용 — 부호 있는 포화율의 느린 EMA
        b = self.apex_ema_alpha
        self._sat_slow = b * (steer / lim) + (1.0 - b) * self._sat_slow
        return steer


@dataclass
class SteerSmoother:
    """이동평균 + 슬루 제한. 서보 떨림과 급조향을 억제한다.

    ★ window 5→3 (0817 시뮬 실측): 이동평균은 지연을 (window-1)/2 프레임
    만큼 순수하게 더한다 — 30fps 에서 5창은 67ms 로, 체인 지연(센싱 50 +
    구동 40 + 서보 τ60ms)에 얹히면 빠른 S자에서 조향이 한 박자 늦어 이탈
    했다(최악 0.93m). 3창(33ms)으로 줄이면 같은 조건 0.27m.
    곡률·기울기는 이미 LaneController 에서 EMA 로 걸러지므로 출력단 평활은
    서보 떨림 억제 정도면 충분하다.
    """

    window: int = 3
    max_rate_deg_per_s: float = 420.0

    def __post_init__(self) -> None:
        self._hist: list[float] = []
        self._last: float = 0.0
        self._last_t: float | None = None

    def reset(self) -> None:
        self._hist.clear()
        self._last = 0.0
        self._last_t = None

    def update(self, steer: float, now: float) -> float:
        self._hist.append(steer)
        if len(self._hist) > max(1, self.window):
            self._hist.pop(0)
        smoothed = sum(self._hist) / len(self._hist)

        if self._last_t is None:
            self._last_t = now
            self._last = smoothed
            return smoothed

        dt = max(1e-3, now - self._last_t)
        max_delta = self.max_rate_deg_per_s * dt
        delta = smoothed - self._last
        if delta > max_delta:
            smoothed = self._last + max_delta
        elif delta < -max_delta:
            smoothed = self._last - max_delta

        self._last = smoothed
        self._last_t = now
        return smoothed


__all__ = [
    "WHEELBASE_M",
    "pure_pursuit_steer",
    "pure_pursuit_cmd",
    "CMD_SATURATION",
    "clamp_steer_for_cones",
    "path_clearance",
    "CAR_FRONT_M",
    "CAR_REAR_M",
    "CAR_HALF_M",
    "PixelPID",
    "LaneController",
    "SteerSmoother",
]

"""라이다 라바콘 게이트 — 조명·반사와 무관한 제2의 눈 (0818 오후 신설).

왜 만들었나 (0818 실측):
  카메라 콘 게이트는 조명·과노출·바닥 반사에 휘둘려 실주행 성립률이
  22~58% 였다. 반면 라바콘은 라이다에 **작고 고립된 점 클러스터**로 또렷이
  찍힌다 — 같은 bag 의 같은 구간에서 스캔당 콘 후보 8.5개, 좌우 쌍(게이트)
  성립률 **80%**, 통로 폭 중앙값 **0.91m**. 게다가 라이다는

    · 조명·노출·반사와 완전히 무관하고 (실내 형광등·유리창 반사 무영향)
    · 거리를 **미터로 직접** 준다 — 카메라 경로의 px_per_m 추정(420, 미실측)
      이 통째로 필요 없어진다. 조준점 오차의 큰 원인 하나가 사라진다.

  그래서 콘 구간의 조향 근거는 라이다를 1순위, 카메라를 보조로 쓴다.

한계:
  · 라이다는 한 평면만 본다. 콘이 낮게 잘려 보이거나(설치 높이) 사람 다리·
    의자 다리도 비슷한 크기로 잡힌다 → 크기 상한과 '좌우 쌍' 조건으로 거른다.
  · 통로가 넓게 열린 구간(콘 끝)에서는 쌍이 안 만들어진다 → 그때는 카메라
    또는 단측 가상 게이트로 자연히 넘어간다 (policy 담당).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


@dataclass
class LidarConeConfig:
    # 유효 거리 [m] — 0.30 미만은 자기 반사(차체), 2.5 초과는 조향에 무의미
    min_range_m: float = 0.30
    max_range_m: float = 2.50
    # 전방 부채꼴 (x 전방+, y 좌측+)
    min_x_m: float = 0.15
    max_abs_y_m: float = 1.20
    # 인접점이 이보다 멀면 다른 물체 (클러스터 분리 기준)
    cluster_gap_m: float = 0.14
    # 클러스터가 콘이려면: 점 2개 이상, 폭 0.30m 이하
    # (실측 콘 밑면 ≈0.20m. 사람 다리 0.15~0.2 도 통과하지만 좌우 쌍
    #  조건에서 걸러진다 — 사람은 통로 양쪽에 대칭으로 서 있지 않다.)
    min_points: int = 2
    max_width_m: float = 0.30
    # ── 의자·책상 다리 배제 (2026-08-23) ──────────────────────
    # 실측(00:58 주행): 실내 의자·책상 다리가 콘으로 잡혀 폭 0.78~0.89m
    # 게이트를 만들었고 차가 콘 구간으로 오인해 들어갔다. 좌표를 보면
    # (0.26,+0.42) 와 (0.29,+0.40) 처럼 3cm 떨어진 점이 각각 콘이 됐다.
    # 라바콘 밑면은 약 0.20m 라 그렇게 붙을 수 없다.
    min_width_m: float = 0.06      # 이보다 얇으면 다리로 본다
    merge_dist_m: float = 0.30     # 이 안에 붙은 후보끼리는 하나로 병합
    # ── 벽/모서리 배제 (0821 밤) ─────────────────────────────────
    # 콘은 **고립**돼 있다: 양옆 빔이 배경으로 뚝 떨어진다. 벽 조각은
    # 양옆으로 벽이 이어져 거리가 매끄럽다. 이 계단이 이만큼은 나야 콘으로
    # 본다. 0 이면 검사를 끈다.
    isolation_step_m: float = 0.15
    isolation_gap_beams: float = 3.0   # 이만큼 빔이 비면 그 자체로 열린 공간
    # 게이트: 좌우(y 부호 반대) + 깊이 차 0.45m 이내 + 폭 0.35~1.60m
    # 실측 통로 폭 0.91m 를 넉넉히 감싼다.
    max_depth_diff_m: float = 0.45
    # ★ 게이트 폭 검증 (0821 재산출). 통로 폭은 **줄자 실측 82cm** 이다.
    #   옛 창 35~160cm 는 너무 넓어 엉뚱한 쌍이 통과했다 — 콘이 9~11개 보이는
    #   장면에서 좌우를 잘못 짝지어 115cm·136cm 짜리 '게이트'가 만들어졌고,
    #   차가 그걸 쫓다 통로에서 +40cm(여유 25.9cm) 벗어나 콘에 걸렸다
    #   (cone_0821_0016 t36~66 실측).
    #   82cm 기준 ±17cm 창으로 조이면 갇힌 구간의 가짜 게이트 71%가 걸러진다.
    #   대가: 게이트 성립률이 절반쯤 준다. 그건 policy 의 콘 목표 유지
    #   (PolicyConfig.cone_hold_s)가 받는다 — 틀린 게이트를 쫓는 것보다
    #   직전 목표를 붙드는 편이 낫다.
    #   ※ 트랙 통로 폭이 바뀌면 **이 두 값만** 고치면 된다.
    min_gate_m: float = 0.65
    max_gate_m: float = 0.98
    # ★ 조준할 게이트의 최소 깊이 [m] (0820 밤 신설).
    #   옛 코드는 무조건 **가장 가까운** 쌍을 골랐는데, 실측(race_0820_2343
    #   라바콘 347프레임) 게이트 깊이가 중앙 33.8cm / 54%가 35cm 이하였다 —
    #   즉 '지금 지나야 할 게이트'가 아니라 **이미 차 옆에 있는 콘**이 뽑혔다.
    #   퓨어 퍼수트는 전방주시가 짧을수록 같은 오차에 조향이 폭발한다:
    #       횡오차 12cm(중앙값)에서  전방주시 35cm → 30.4도
    #                                       55cm → 13.4도
    #                                       75cm →  7.3도
    #   그래서 매 프레임 ±30 이 뒤집혔고, 그 진동이 곡선감속을 불러 속도까지
    #   깎았다. 진동과 저속이 같은 뿌리다.
    #   이 값 이상 앞에 있는 쌍을 우선하고, 없으면 종전대로 가장 가까운 쌍을
    #   쓴다 (하드 필터가 아니라 선호라 통로 끝에서도 게이트를 잃지 않는다).
    prefer_depth_m: float = 0.55
    # 근접 콘을 보는 전방 창 [m] — 이 안의 콘은 '지금 지나는 중'이다.
    # ★ 0.70 → 0.45 (0821). 라이다가 차체 중앙 부근이고 앞 범퍼까지 ≈0.25m
    #   이므로, 0.45m 를 넘으면 '옆'이 아니라 '앞'이다.
    near_window_m: float = 0.45
    # ★ 근접 판정을 **실제 거리**로 한다 (0821). 옛 코드는 횡거리 |y| 만 보고
    #   전방거리 x 를 무시해서, x=62cm·y=20cm 인 **앞쪽** 콘을 '옆에 20cm' 로
    #   셌다. 실측(cone_0821_0037): 반발이 걸린 콘 116개 중 **58%가 전방
    #   45cm 초과** — 옆이 아니라 앞에 있는 콘이었다. 불필요한 밀어내기가
    #   그만큼 있었다는 뜻이다.
    #   이제 hypot(x,y) 가 이 값 안일 때만 '옆에 있다'고 본다. 다만 **보고하는
    #   값은 여전히 횡거리 |y|** 다 — 부딪히느냐를 결정하는 건 횡여유지
    #   대각선 거리가 아니기 때문이다. (실제 거리를 보고하면 x=25·y=5cm 인
    #   임박 충돌이 '25cm 여유' 로 둔갑해 반발이 꺼진다.)
    near_radius_m: float = 0.45
    # ── 같은 열 오인 방지 (0821 저녁 신설) ────────────────────────
    # ★ 실주행 cone_0821_0220 실측: 고른 게이트의 **50.2%가 같은 열의 두
    #   콘**이었다. 통로가 굽으면 한 열이 y=0 을 가로질러 두 콘의 y 부호가
    #   갈리고, 한 칸 걸러 두 개는 0.80m 라 폭 창 65~98cm 에 그대로 들어온다.
    #   그러면 차는 **한 열의 틈**을 통로로 알고 그리로 빠져나가고, 나가면서
    #   콘을 친다 (사용자 목격: "라바콘 사이 틈으로 빠져나오고 그러면서
    #   라바콘에 부딪혀"). 가짜 게이트는 폭 중앙 90cm·깊이 64cm 였고
    #   주축과의 나란함 |cos| 중앙이 0.95 였다 — 통로를 가로지르는 게 아니라
    #   통로를 따라 누운 쌍이다.
    # 그래서 쌍의 방향이 콘 배치의 **주축과 수직**에 가까운지 본다. 주축은
    # 콘 배치에서 나오므로 차가 통로 어디에 있든 흔들리지 않는다.
    # 같은 bag 에서 임계별 실측 (성립률 / 가짜 잔존 / 폭 IQR):
    #     검사 없음   95.9%  50.2%  78~97cm
    #     0.75        73.0%   0.0%  70~84cm   ← 채택
    #     0.85        80.3%  21.3%  70~87cm
    #     0.90        85.7%  33.5%  71~87cm
    # 0.75 에서 가짜가 완전히 사라지고 폭 분포가 줄자 82cm 로 모인다.
    # ('두 콘이 주축선 반대편' 조건도 같이 시험했지만 가짜를 45.9% 나
    #  통과시켜 사실상 무효였고, 성립률만 6%p 더 깎았다 — 뺐다.)
    # 대가: 성립률 95.9 → 73.0%. 잃은 프레임은 cone_hold_s 의 목표 유지가
    # 받는다 — **가짜 게이트를 쫓는 것보다 직전 목표를 붙드는 편이 낫다**.
    # 그게 이 구간에서 이미 검증된 판단이다 (min_gate_m 주석 참조).
    pair_perp_max: float = 0.75    # |cos|. 0.75 = 수직에서 41° 까지 허용
    pair_min_cones: int = 4        # 이보다 적으면 주축을 못 믿어 검사를 건너뛴다
    # ── 줄 소속 판정 (0821 밤 2차 신설) ──────────────────────────
    # ★ 위의 주축 검사는 **쌍**만 본다. 그런데 실패는 쌍이 아니라 **한 줄만
    #   보일 때** 났다. 통로 바깥으로 밀리면 안쪽 줄이 시야에서 사라지고
    #   바깥 줄만 남는데, 그 줄이 곡률 때문에 차 축을 가로질러 올라온다.
    #   가상 게이트는 여태 **y 부호로 줄을 판정**했으므로 오른쪽 줄 콘을
    #   '왼쪽 줄'로 읽고 통로 중심을 그 콘의 오른쪽 반폭에 놓았다 —
    #   즉 **통로 바깥**이다. 차는 그리로 간다.
    #   폐루프 실측(cone_corridor_sim, R1.2m 통로 e=-30cm):
    #       보이는 콘 4개 전부 오른쪽 줄 — (0.33,-0.05) (0.83,+0.14)
    #                                     (1.22,+0.50) (1.48,+0.96)
    #       옛 게이트: 폭 82cm 횡 **-26.7cm** (더 바깥) → 2m 이탈
    #   y 부호는 **x 가 작을 때만** 믿을 수 있다. 곡률이 콘을 축 너머로
    #   밀어내는 양은 x²/(2R) 이라 x=0.6m·R=1.0m 에서 18cm 뿐이지만
    #   x=1.2m 면 72cm 로 반폭(41cm)을 넘긴다.
    #   그래서 **사슬의 최근접 콘**에서 줄의 쪽을 정하고 그 줄 전체에 물려준다.
    row_link_m: float = 0.60       # 이 안이면 같은 줄 (통로 폭 82cm 보다 작다)
    row_anchor_x_m: float = 0.60   # 이 안의 콘만 y 부호를 믿는다
    # ★ 기준 콘이 축에 너무 붙으면 부호 자체를 못 믿는다. 통로 밖으로 밀리고
    #   요까지 틀어지면 옆줄의 최근접 콘이 y=0 을 스치며 지나간다 —
    #   실측(R1.2m 통로, e=-25cm·요 -20°)에서 기준 콘이 y=+0.03 으로 넘어와
    #   오른쪽 줄 전체가 '왼쪽 줄' 이 됐다. 이 띠 안에서는 **직전 스캔의 쪽**을
    #   그대로 쓴다. 줄은 움직이지 않으므로 8.8Hz 사이에 소속이 바뀔 수 없다.
    row_anchor_min_y_m: float = 0.10
    # ── 통로 추종 (0821 밤 신설) ──────────────────────────────────
    # ★ 게이트 '쌍'을 못 만들면 통로를 아예 안 따라가던 것이 이탈의 주범이다.
    #   실측(cone_0821_0307): 게이트 성립 52%, **통로 이탈 14회 중 10회가
    #   게이트가 없는 프레임**이었다. 그때는 카메라 단측 추정으로 넘어가는데,
    #   그게 라이다의 근접 콘 반발과 반대로 밀어 상쇄된 장면도 있었다
    #   (t=15.14 오른쪽 콘 2.0cm 인데 조향 +0.7).
    #
    #   콘이 3개만 있어도 통로 방향은 알 수 있다. 주축(PCA)이 통로가 뻗은
    #   방향이고, 콘 열은 중심선에서 반폭만큼 떨어져 있다. 한 줄만 보이면
    #   **그 줄에서 차 쪽으로 반폭만큼** 옮긴 선이 중심선이다 — 차가 통로
    #   안이면 정확하고, 밖으로 밀렸으면 안쪽으로 끌어당긴다. 둘 다 옳다.
    # ── 옆 콘과의 모순 검사 (0821 밤 3차 신설) ────────────────────
    # ★ 실주행 cone_0821_0651 t=10.5~12.5 실측. **차 오른쪽 12cm 에 콘이
    #   있는데 게이트는 '통로 중심이 왼쪽 76cm' 라고 했다.** 차는 그 목표를
    #   쫓아 좌 풀락(-42)으로 2초를 돌았고, 그동안 오른쪽 줄을 긁었다
    #   (차체 여유 -7.5cm, 접촉 스캔 17개).
    #
    #   이건 그 자체로 모순이다. 오른쪽 12cm 의 콘이 오른쪽 줄이라면 통로
    #   중심은 -0.12+0.41 = +0.29m 여야 한다. +0.76m 라면 오른쪽 줄이
    #   +0.35m — 즉 **차의 왼쪽**에 있다는 말이고, 그럼 오른쪽 12cm 의 콘은
    #   설명되지 않는다.
    #
    #   그래서 **가까운 관측이 먼 관측을 이긴다**로 푼다. 옆 콘은 거리가
    #   짧아 각도오차의 영향이 작고, 무엇보다 **지금 부딪히려는 그 물체**다.
    #   게이트 중심이 옆 콘이 허용하는 범위를 넘으면 그 범위로 당긴다.
    #   (버리지 않고 당기는 이유: 버리면 목표유지로 떨어지는데, 그때도
    #    조향은 직전 값을 붙들어 풀락이 유지된다 — 당겨야 방향이 바뀐다.)
    #
    #   실측 판별력 (같은 bag, 라바콘 구간 유효 게이트 142개):
    #       걸린 게이트 24개(17%) — **전부 t=8.7~13.2 접촉 구간에 몰림**
    #       접촉 스캔 17개 중 10개가 이 검사에 걸린 프레임
    #   여유(margin)를 0 으로 두면 정상 프레임까지 건드리므로 조금 준다.
    near_veto_margin_m: float = 0.15
    # 쌍을 이룰 수 있는 콘의 횡거리 상한 [m]. 통로 안에 있을 수 없는 위치의
    # 콘(벽·잡물·다른 구간)을 짝지어 엉뚱한 게이트를 만드는 것을 막는다.
    # 값은 아래 스윕으로 정한다 — 좁히면 곡선에서 정상 반대편 줄까지 자른다.
    pair_band_m: float = 0.85
    corridor_half_m: float = 0.41   # 중심선에서 콘 열까지 (실측 82cm의 절반)
    virtual_window_m: float = 0.95  # 가상 게이트를 만들 때 볼 전방 거리
    virtual_side_margin_m: float = 0.25  # 통로 반폭 + 이만큼까지만 통로 콘


@dataclass
class LidarGate:
    valid: bool = False
    depth_m: float = 0.0        # 게이트까지 전방 거리
    width_m: float = 0.0        # 좌우 콘 사이 간격
    lateral_m: float = 0.0      # 게이트 중앙의 횡위치 (좌측 +)
    cones: int = 0              # 이번 스캔의 콘 후보 수
    # ★ 차 바로 옆의 콘까지 횡거리 [m] (없으면 inf) — 0821 신설.
    #   게이트는 **앞쪽** 쌍을 조준하는데, 그 사이 차 옆을 지나는 콘은
    #   아무도 보지 않았다. cone_0821_0025 t10.1 실측: 게이트는 깊이 34cm
    #   앞에서 폭 75cm 통로를 보고 '횡편차 +36cm' 라 했는데, 같은 순간
    #   **차 오른쪽 1cm 에 콘이 있었다**. 그게 접촉이다.
    near_left_m: float = float("inf")
    near_right_m: float = float("inf")
    # ★ 이번 스캔에서 본 콘 위치 (x 전방, y 좌측) [m] — 0821 밤.
    #   마지막 관문(충돌 방지 클램프)이 쓴다. 추정을 다 맞히는 대신,
    #   **명령이 콘을 관통하지 못하게** 막는 편이 일관성에 낫다.
    points: Tuple[Tuple[float, float], ...] = ()
    # ★ 차에 **가장 가까운 콘이 속한 줄**의 쪽 (+1 왼쪽 / -1 오른쪽 / 0 모름).
    #   다음 스캔에 되먹여 기준 콘이 축에 붙었을 때의 판정을 이어받는다
    #   (LidarConeConfig.row_anchor_min_y_m 주석).
    near_row_side: int = 0


def _cluster(
    pts: Sequence[Tuple[float, float]], gap: float, min_points: int
) -> List[Tuple[float, float, float, int]]:
    """각도 순으로 정렬된 점들을 인접 거리로 묶는다 → (cx, cy, 폭, 점수)."""
    out: List[Tuple[float, float, float, int]] = []
    cur: List[Tuple[float, float]] = []

    def flush() -> None:
        if len(cur) < min_points:
            return
        xs = [p[0] for p in cur]
        ys = [p[1] for p in cur]
        w = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        out.append((sum(xs) / len(xs), sum(ys) / len(ys), w, len(cur)))

    for p in pts:
        if not cur:
            cur = [p]
            continue
        if math.hypot(p[0] - cur[-1][0], p[1] - cur[-1][1]) <= gap:
            cur.append(p)
        else:
            flush()
            cur = [p]
    flush()
    return out


def _fit_quadratic(pts: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    """전방 u 에 대해 v = a·u² + b·u + c 를 맞춰 (a, b) 를 낸다 (u 중심화)."""
    n = len(pts)
    if n < 3:
        return None
    su = sum(p[0] for p in pts) / n
    xs = [p[0] - su for p in pts]
    ys = [p[1] for p in pts]
    s0, s1, s2 = float(n), sum(xs), sum(x * x for x in xs)
    s3 = sum(x ** 3 for x in xs)
    s4 = sum(x ** 4 for x in xs)
    t0 = sum(ys)
    t1 = sum(x * y for x, y in zip(xs, ys))
    t2 = sum(x * x * y for x, y in zip(xs, ys))
    m = ((s4, s3, s2), (s3, s2, s1), (s2, s1, s0))
    det = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
           - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
           + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    if abs(det) < 1e-14:
        return None
    da = (t2 * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
          - m[0][1] * (t1 * m[2][2] - m[1][2] * t0)
          + m[0][2] * (t1 * m[2][1] - m[1][1] * t0))
    db = (m[0][0] * (t1 * m[2][2] - m[1][2] * t0)
          - t2 * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
          + m[0][2] * (m[1][0] * t0 - t1 * m[2][0]))
    # a 는 u 중심화 좌표 기준이지만 2차 계수는 평행이동에 불변이다.
    # b 는 u=su 지점의 기울기 → 차 앞(u≈0)의 기울기로 되돌린다.
    a = da / det
    b = db / det - 2.0 * a * su
    return a, b


def _cone_clusters(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    angle_offset: float,
    cfg: LidarConeConfig,
) -> List[Tuple[float, float, float, int]]:
    """LaserScan 원시값 → 콘 후보 클러스터 (x, y, 폭, 점수).

    ★ 0821 밤 재작성. 두 가지가 벽을 콘으로 만들고 있었다.

    ① **ROI 로 먼저 자르면 안 된다.** 옛 코드는 x/y 창으로 점을 거른 뒤
       묶었다. 그러면 창을 가로지르는 **벽이 잘려** 짧은 조각이 되고,
       그 조각이 폭 검사(≤0.30m)를 통과해 콘이 된다. 이제 스캔 전체를
       묶은 **다음에** 클러스터 중심으로 ROI 를 건다 — 벽은 하나의 긴
       클러스터라 폭에서 걸린다.

    ② **콘은 고립돼 있다.** 콘은 양옆이 뚝 끊긴다(뒤 배경까지 거리 계단).
       벽 조각은 양옆으로 벽이 이어져 거리가 매끄럽다. 각진 벽 모서리가
       콘처럼 보이는 것도 이 검사로 걸러진다.
       (사용자 목격: "이탈한 구간에 각진 벽이 있는데 거길 라바콘으로
        인식한 것 같다" — 3회 중 1회만 이탈해 주행마다 결과가 갈렸다.)
    """
    raw: List[Tuple[float, float, float, float]] = []   # (bearing, x, y, r)
    for i, r in enumerate(ranges):
        if not math.isfinite(r):
            continue
        if not (cfg.min_range_m < r < cfg.max_range_m):
            continue
        a = angle_min + i * angle_increment + angle_offset
        raw.append((a, r * math.cos(a), r * math.sin(a), float(r)))
    n = len(raw)
    if n < cfg.min_points:
        return []

    # 빔 순서대로 인접 거리로 묶는다 (ROI 는 나중에)
    spans: List[Tuple[int, int]] = []
    start = 0
    for k in range(1, n):
        if math.hypot(raw[k][1] - raw[k - 1][1], raw[k][2] - raw[k - 1][2]) > cfg.cluster_gap_m:
            spans.append((start, k - 1))
            start = k
    spans.append((start, n - 1))

    beam = abs(angle_increment) or 1e-3
    out: List[Tuple[float, float, float, int]] = []
    for (a0, a1) in spans:
        cnt = a1 - a0 + 1
        if cnt < cfg.min_points:
            continue
        xs = [raw[k][1] for k in range(a0, a1 + 1)]
        ys = [raw[k][2] for k in range(a0, a1 + 1)]
        w = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if w > cfg.max_width_m:
            continue
        cx, cy = sum(xs) / cnt, sum(ys) / cnt
        if cx < cfg.min_x_m or abs(cy) > cfg.max_abs_y_m:
            continue
        if cfg.isolation_step_m > 0.0:
            rm = sum(raw[k][3] for k in range(a0, a1 + 1)) / cnt
            if not (_side_free(raw, a0, -1, rm, beam, cfg)
                    and _side_free(raw, a1, +1, rm, beam, cfg)):
                continue
        if w < cfg.min_width_m:
            continue                    # 너무 얇다 = 다리
        out.append((cx, cy, w, cnt))

    # 서로 merge_dist_m 안에 붙은 후보는 하나의 물체로 본다.
    # 다리 여러 개가 각각 콘으로 세어지는 것을 막는다.
    merged: List[Tuple[float, float, float, int]] = []
    for c in sorted(out, key=lambda t: -t[3]):
        if any(math.hypot(c[0] - m[0], c[1] - m[1]) < cfg.merge_dist_m
               for m in merged):
            continue
        merged.append(c)
    return merged


def _side_free(
    raw: Sequence[Tuple[float, float, float, float]],
    edge: int,
    direction: int,
    mean_r: float,
    beam: float,
    cfg: LidarConeConfig,
) -> bool:
    """클러스터 한쪽이 '끊겨' 있는가 (콘의 조건). 벽은 이어진다."""
    k = edge + direction
    if k < 0 or k >= len(raw):
        return True                       # 스캔 끝 = 더 볼 것이 없다
    d_ang = abs(raw[k][0] - raw[edge][0])
    if d_ang > cfg.isolation_gap_beams * beam:
        return True                       # 사이가 비었다(무반사) = 열린 공간
    return abs(raw[k][3] - mean_r) > cfg.isolation_step_m


def _row_sides(
    cones: Sequence[Tuple[float, float, float, int]],
    cfg: LidarConeConfig,
    hint: int = 0,
) -> List[int]:
    """각 콘이 어느 **줄**인가 — +1 왼쪽 / -1 오른쪽 / 0 모름.

    y 부호로 줄을 정하면 굽은 통로에서 뒤집힌다 (LidarConeConfig.row_link_m
    주석의 실측). 대신 두 단계로 푼다.

      ① 가까운 콘끼리 사슬로 묶는다. 같은 줄의 이웃은 콘 간격만큼(실측 배치
         에서 26~54cm) 떨어져 있고, **마주보는 줄**은 통로 폭(82cm) 만큼
         떨어져 있다 — row_link_m=0.60m 이 그 사이를 가른다.
      ② 사슬의 **가장 가까운 콘**의 y 부호를 그 사슬 전체의 쪽으로 쓴다.
         곡률이 콘을 축 너머로 미는 양 x²/(2R) 은 x 가 작을수록 작아지므로,
         차 옆을 지나는 콘의 좌/우는 통로가 아무리 굽어도 뒤집히지 않는다.

    기준 콘이 row_anchor_x_m 보다 멀면 **모름(0)** 을 낸다 — 통로 진입 전처럼
    양쪽 줄이 다 멀리 있는 장면이다. 그때는 부르는 쪽이 옛 규칙으로 되돌아간다.
    """
    n = len(cones)
    if n == 0:
        return []

    parent = list(range(n))

    def _root(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i in range(n):
        for j in range(i + 1, n):
            d = math.hypot(cones[i][0] - cones[j][0], cones[i][1] - cones[j][1])
            if d > cfg.row_link_m:
                continue
            ri, rj = _root(i), _root(j)
            if ri != rj:
                parent[ri] = rj

    anchor: dict = {}
    for i in range(n):
        r = _root(i)
        cur = anchor.get(r)
        if cur is None or cones[i][0] < cones[cur][0]:
            anchor[r] = i

    # 차에 가장 가까운 콘이 속한 사슬 — 직전 쪽(hint)을 물려줄 대상이다
    nearest = min(range(n), key=lambda i: cones[i][0] ** 2 + cones[i][1] ** 2)
    near_root = _root(nearest)

    out = [0] * n
    for i in range(n):
        r = _root(i)
        a = cones[anchor[r]]
        if a[0] > cfg.row_anchor_x_m:
            continue                      # 기준 콘이 너무 멀다 — 판정하지 않는다
        if abs(a[1]) < cfg.row_anchor_min_y_m and r == near_root and hint:
            # 기준 콘이 축에 붙었다 — 부호가 한 스캔 만에 뒤집힐 수 있다.
            # 줄은 움직이지 않으니 직전 스캔의 쪽을 잇는다.
            out[i] = hint
        else:
            out[i] = 1 if a[1] > 0.0 else -1
    return out


def find_cone_gate(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    angle_offset: float = 0.0,
    config: Optional[LidarConeConfig] = None,
    prev_row_side: int = 0,
) -> LidarGate:
    """LaserScan 원시값 → 콘 게이트. rclpy 의존 없음(테스트 가능).

    prev_row_side 는 **직전 스캔**이 낸 LidarGate.near_row_side 다. 기준 콘이
    축에 붙어 좌/우를 못 가릴 때만 쓰인다 (row_anchor_min_y_m 주석). 신선하지
    않으면 0 을 넘겨라 — 옛 스캔의 쪽을 잇는 것보다 모른다고 하는 편이 낫다.
    """
    cfg = config or LidarConeConfig()

    cones = _cone_clusters(ranges, angle_min, angle_increment, angle_offset, cfg)
    if len(cones) < 2:
        return LidarGate(cones=len(cones))

    # 콘 배치의 주축 = 통로가 뻗은 방향 (pair_perp_max 주석 참조)
    axis: Optional[Tuple[float, float, float, float]] = None
    if len(cones) >= cfg.pair_min_cones:
        n_c = len(cones)
        mx0 = sum(c[0] for c in cones) / n_c
        my0 = sum(c[1] for c in cones) / n_c
        sxx = sum((c[0] - mx0) ** 2 for c in cones)
        syy = sum((c[1] - my0) ** 2 for c in cones)
        sxy = sum((c[0] - mx0) * (c[1] - my0) for c in cones)
        tr = sxx + syy
        disc = max(0.0, tr * tr / 4.0 - (sxx * syy - sxy * sxy))
        lam = tr / 2.0 + math.sqrt(disc)
        ax, ay = (sxy, lam - sxx) if abs(sxy) > 1e-9 else (1.0, 0.0)
        an = math.hypot(ax, ay)
        if an > 1e-9:
            axis = (mx0, my0, ax / an, ay / an)

    def _crosses_corridor(a, b) -> bool:
        """이 쌍이 통로를 **가로지르는가** (같은 열의 두 콘이 아닌가)."""
        if axis is None:
            return True                      # 주축을 못 믿으면 검사하지 않는다
        _, _, ux, uy = axis
        seg = math.hypot(b[0] - a[0], b[1] - a[1])
        if seg < 1e-6:
            return False
        return abs(((b[0] - a[0]) * ux + (b[1] - a[1]) * uy) / seg) <= cfg.pair_perp_max

    # 콘마다 줄 소속을 먼저 정한다 (_row_sides 주석). 쌍 판정과 가상 게이트가
    # 같은 판정을 쓴다 — 굽은 통로에서 y 부호가 뒤집혀도 흔들리지 않는다.
    sides = _row_sides(cones, cfg, prev_row_side)

    # 좌우 마주보는 쌍 중 **가장 가까운** 것 = 지금 지나야 할 게이트
    best: Optional[Tuple[float, float, float]] = None
    near: Optional[Tuple[float, float, float]] = None   # 폴백용 최근접 쌍
    for i in range(len(cones)):
        for j in range(i + 1, len(cones)):
            a, b = cones[i], cones[j]
            sa, sb = sides[i], sides[j]
            if sa != 0 and sb != 0:
                # 줄을 안다 — 부호가 아니라 **줄**로 판정한다. 굽은 통로에서는
                # 진짜 쌍도 y 부호가 같아질 수 있고(둘 다 축 한쪽으로 밀림),
                # 반대로 같은 줄의 두 콘이 부호를 달리하기도 한다.
                if sa == sb:                             # 같은 줄 — 게이트 아님
                    continue
            else:
                # 줄을 모른다 (기준 콘이 멀다) — 옛 규칙으로 되돌아간다
                if a[1] * b[1] >= 0.0:                   # 같은 쪽이면 게이트 아님
                    continue
                if not _crosses_corridor(a, b):          # 같은 열의 두 콘이다
                    continue
            if abs(a[0] - b[0]) > cfg.max_depth_diff_m:  # 깊이가 다르면 다른 쌍
                continue
            # ★ 두 콘 모두 **통로 밴드 안**이어야 한다 (0821 밤 3차).
            #   가상 게이트 경로엔 이 제한(virtual_side_margin_m)이 있는데
            #   쌍 경로엔 없었다. 실측 cone_0821_0651 t=11.10: 차 왼쪽
            #   **1.20m** 의 콘을 +0.33m 콘과 짝지어 '폭 87cm 게이트' 를
            #   만들고 중심을 +0.76m 로 보고했다 — 82cm 통로 안에 있을 수
            #   없는 위치다. 차는 그 목표를 좌 풀락으로 쫓다 오른쪽 줄을
            #   긁었다 (여유 -7.5cm).
            if abs(a[1]) > cfg.pair_band_m or abs(b[1]) > cfg.pair_band_m:
                continue
            gap = abs(a[1] - b[1])
            if not (cfg.min_gate_m <= gap <= cfg.max_gate_m):
                continue
            depth = (a[0] + b[0]) / 2.0
            cand = (depth, gap, (a[1] + b[1]) / 2.0)
            # 우선: prefer_depth_m 이상 앞의 쌍 중 가장 가까운 것
            if depth >= cfg.prefer_depth_m:
                if best is None or depth < best[0]:
                    best = cand
            # 폴백: 그런 쌍이 하나도 없을 때만 쓰는 '가장 가까운 쌍'
            elif near is None or depth < near[0]:
                near = cand

    # 차 옆(전방 near_window_m 안)의 좌/우 최근접 콘 — 게이트와 무관하게 잰다
    def _near(sign: float) -> float:
        best = float("inf")
        for cx, cy, _w, _n in cones:
            if cy * sign <= 0.0:
                continue
            # ① '옆에 있는가' 는 실제 거리로 판정한다 (앞쪽 콘 배제)
            if not (cfg.min_x_m <= cx <= cfg.near_window_m):
                continue
            if math.hypot(cx, cy) > cfg.near_radius_m:
                continue
            # ② 그러나 '여유가 얼마인가' 는 횡거리다. 여기에 실제 거리를
            #    넣으면 바로 옆 5cm 콘이 '29cm 여유' 로 보고돼 반발이 안 걸린다.
            lat = abs(cy)
            if lat < best:
                best = lat
        return best

    nl = _near(+1.0)
    nr = _near(-1.0)
    # 다음 스캔에 물려줄 값 — 가장 가까운 콘이 속한 줄의 쪽
    _ni = min(range(len(cones)), key=lambda i: cones[i][0] ** 2 + cones[i][1] ** 2)
    near_row_side = sides[_ni]
    # 클램프가 쓸 콘 좌표 — 가까운 것부터 (앞 2.0m 안)
    pts_out = tuple(
        (round(c[0], 3), round(c[1], 3))
        for c in sorted(cones, key=lambda c: c[0] * c[0] + c[1] * c[1])
        if c[0] <= 2.0
    )[:24]

    if best is None:
        best = near

    if best is None:
        # ★ 가상 게이트 (0821 밤). 좌우 쌍을 못 만들면 여태 통로를 **아예
        #   안 따라갔다** — 그게 이탈의 주범이다. 실측(cone_0821_0307):
        #   게이트 성립 52%, 통로 이탈 14회 중 **10회가 게이트 없는 프레임**.
        #   그때는 카메라 단측 추정으로 넘어가는데, 그게 라이다의 근접 콘
        #   반발과 반대로 밀어 상쇄된 장면도 있었다 (t=15.14 오른쪽 콘
        #   2.0cm 인데 조향 +0.7).
        #
        #   한 줄만 보여도 통로 위치는 안다: 그 줄에서 **통로 쪽으로** 반폭
        #   옮긴 곳이 중심선이다. 차가 통로 안이면 정확하고, 밖으로 밀렸으면
        #   안쪽으로 끌어당긴다 — 둘 다 가야 할 방향이다.
        #
        #   ★ '통로 쪽'이 어느 쪽인가는 y 부호가 아니라 **줄 소속**(_row_sides)
        #     이 답한다. 통로가 굽으면 먼 콘은 좌우가 뒤집혀 보인다(오른쪽 줄이
        #     y>0 이 된다) — 여기가 통로 밖으로 밀린 차를 더 밀어내던 자리다.
        #   ★ 차 앞 virtual_window_m 안의 콘만 쓴다 (먼 콘일수록 곡률 왜곡이
        #     크고, 조준점으로도 부적합하다).
        w = cfg.corridor_half_m
        # 통로 폭 안에 있는 콘만 쓴다. 옆으로 멀리 떨어진 콘(다른 줄, 벽
        # 근처 잡물)을 기준 삼으면 목표가 통로 밖으로 나간다 —
        # 실측에서 y=+0.95m 콘을 잡아 목표가 +54cm 로 나온 장면이 있었다.
        lim = w + cfg.virtual_side_margin_m
        win = [i for i, c in enumerate(cones)
               if cfg.min_x_m <= c[0] <= cfg.virtual_window_m and abs(c[1]) <= lim]
        # ★ 줄 소속은 **사슬의 최근접 콘**에서 온다 — y 부호가 아니다.
        #   여기가 통로 바깥으로 밀린 차를 더 밀어내던 자리다 (row_link_m 주석).
        vl = min((cones[i] for i in win if sides[i] > 0),
                 key=lambda c: c[0], default=None)
        vr = min((cones[i] for i in win if sides[i] < 0),
                 key=lambda c: c[0], default=None)
        # 줄을 모르는 콘은 옛 규칙(y 부호)으로 채운다 — 통로 진입 전 장면.
        if vl is None:
            vl = min((cones[i] for i in win if sides[i] == 0 and cones[i][1] > 0.05),
                     key=lambda c: c[0], default=None)
        if vr is None:
            vr = min((cones[i] for i in win if sides[i] == 0 and cones[i][1] < -0.05),
                     key=lambda c: c[0], default=None)
        # ★ 줄마다 **따로** 통로 중심을 추정하고 평균한다 (0821 밤 3차).
        #   옛 코드는 두 콘의 y 중점을 썼다. 그건 **같은 깊이**일 때만 옳다 —
        #   굽은 통로에서 vl 과 vr 의 깊이가 다르면 두 y 는 통로 폭이 아니라
        #   '폭 + 곡률로 벌어진 양' 만큼 떨어져 있어, 중점이 통로 밖으로 나간다.
        #   (그걸 막으려고 span > 2w 일 때 당기는 보정이 붙어 있었는데,
        #    보정 자체가 '어느 콘이 가까운가' 로 갈려 목표가 뛰는 원인이었다.)
        #
        #   줄에서 반폭만큼 통로 쪽으로 옮긴 점은 **그 콘의 깊이에서** 정확하다.
        #   왼쪽 줄이면 오른쪽으로, 오른쪽 줄이면 왼쪽으로 옮긴다.
        #   좌우가 다 보이면 두 추정을 평균한다 — 진짜 쌍(같은 깊이·82cm)이면
        #   두 값이 같아 옛 중점과 정확히 일치하고, 깊이가 어긋나면 두 점의
        #   가운데를 조준한다. 어느 줄이 더 가깝든 값이 같아 뒤집히지 않는다.
        #
        #   폭은 **추정한 통로 폭(2w)** 으로 보고한다. 관측한 값이 아니라
        #   가정한 값이기 때문이다 — 여기서 두 콘의 y 차이를 폭이라고 실으면
        #   policy 가 그 값으로 통로 반폭을 학습해(_cone_half_m) 같은 줄의
        #   좁은 틈을 통로 폭으로 배운다.
        est = []
        if vl is not None:
            est.append((vl[0], vl[1] - w))
        if vr is not None:
            est.append((vr[0], vr[1] + w))
        if est:
            best = (sum(e[0] for e in est) / len(est),
                    2.0 * w,
                    sum(e[1] for e in est) / len(est))
    if best is None:
        return LidarGate(cones=len(cones), near_left_m=nl, near_right_m=nr,
                         points=pts_out, near_row_side=near_row_side)

    # ★ 옆 콘과의 모순 검사 (near_veto_margin_m 주석의 실측). 옆에 콘이
    #   붙어 있는데 목표가 그 반대편으로 크게 나가 있으면, 옆 콘이 허용하는
    #   범위까지 당긴다. 옆 콘이 없으면(inf) 아무 제약도 걸지 않는다.
    lat = best[2]
    w2 = cfg.corridor_half_m + cfg.near_veto_margin_m
    if math.isfinite(nr):
        lat = min(lat, -nr + w2)        # 오른쪽 옆 콘 → 목표가 그보다 왼쪽일 수 없다
    if math.isfinite(nl):
        lat = max(lat, nl - w2)         # 왼쪽 옆 콘 → 목표가 그보다 오른쪽일 수 없다
    best = (best[0], best[1], lat)

    return LidarGate(
        valid=True,
        depth_m=best[0],
        width_m=best[1],
        lateral_m=best[2],
        near_left_m=nl,
        near_right_m=nr,
        cones=len(cones),
        points=pts_out,
        near_row_side=near_row_side,
    )


__all__ = ["LidarConeConfig", "LidarGate", "find_cone_gate"]

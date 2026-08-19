"""차선 검출 — BEV 변환 + (흰색 HSV ∪ Canny ∪ Sobel) + 슬라이딩 윈도우 + 2차 폴리핏.

기본 파라미터는 팀이 실차에서 이미 튜닝한 값(`segi/my_driver_lanefollow.py`)을
그대로 가져왔다. BEV 원근 좌표, 흰색 임계, 차선폭 450px 는 자이카 Y모델 카메라
장착 높이·화각에 맞춰진 값이라 처음부터 다시 찾을 이유가 없다.

바꾼 것:
  · 트랙바(cv2.getTrackbarPos) 의존 제거 → 파라미터 주입. 대회 당일 GUI 없이
    돌려야 하고, 창이 닫히면 조용히 기본값으로 되돌아가는 사고를 막는다.
  · 한쪽 차선만 보일 때 차선폭 히스토리로 반대쪽을 추정하는 로직 유지.
  · lane_side(지금 어느 차선에 있나) 추가 — 추월 방향 3순위 판단에 쓴다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

from ..core.types import LaneReading


@dataclass
class LaneFit:
    """관측 구간을 기억하는 2차 곡선 — 범위 밖은 **접선으로 선형 외삽**한다.

    왜 필요한가: 2차식을 관측 범위 밖까지 그대로 대입하면 조금만 벗어나도
    발산한다. 코너에서 차선은 ROI 위쪽(먼 쪽)에만 남는 일이 잦은데, 그때
    근거리 값 x(y_near) 를 2차식으로 외삽하면 수백 px 씩 튄다 — 그게 그대로
    조향 슬램이 된다. 관측 범위 끝의 접선으로 잇는 것이 물리적으로도 맞다
    (본 것 너머의 곡률은 모르니 직진으로 가정).
    """

    coef: np.ndarray            # [a, b, c] — x = a·y² + b·y + c
    y_lo: float                 # 관측된 y 최소 (BEV 위쪽 = 먼 쪽)
    y_hi: float                 # 관측된 y 최대 (BEV 아래쪽 = 차에 가까움)
    # 밴드 밝은 구간의 중앙값 폭 [px] (rowscan 전용, 히스토그램 경로는 0).
    # 테이프 차선(4~5cm ≈ 30~40px)과 바닥 이음새(5~10px)를 가르는 단서 —
    # 같은 물리 선으로 보이는 두 후보 중 하나를 골라야 할 때 넓은 쪽을 남긴다.
    width: float = 0.0

    def at(self, y: float) -> float:
        yc = min(max(float(y), self.y_lo), self.y_hi)
        val = float(np.polyval(self.coef, yc))
        if yc != y:
            slope = 2.0 * float(self.coef[0]) * yc + float(self.coef[1])
            val += slope * (float(y) - yc)
        return val

    @property
    def a(self) -> float:
        return float(self.coef[0])

    @property
    def b(self) -> float:
        return float(self.coef[1])

    @property
    def span(self) -> float:
        return self.y_hi - self.y_lo


@dataclass
class LaneConfig:
    # ── BEV 원근 변환 (프레임 비율) ──
    src_bottom_y: float = 0.90
    src_top_y: float = 0.60
    src_bottom_inset: float = 0.10
    src_top_inset: float = 0.40
    dst_inset: float = 0.20

    # ── 이진화 방식 (0818 밤 교체) ────────────────────────────────
    # "stripe" = 폭 정합 top-hat (기본). "legacy" = 절대 HSV 임계 ∪ Canny ∪ Sobel.
    #
    # 왜 바꿨나 (0818 bag 실측):
    #   ⓐ 주행 중 BEV 의 **프레임 최대 밝기가 V 229~237** 인데 흰색 임계는
    #      V≥243 이었다. 흰 마스크가 상시 공집합이었고 검출은 사실상 Canny/
    #      Sobel 단독으로 돌아갔다 — 가구·벽-바닥 경계·바닥 이음새가 전부
    #      '차선'이 된다.
    #   ⓑ 세션 내내 평균 V 가 109~214 로 요동한다(자동노출). **고정 절대
    #      임계는 프레임마다 다른 문제를 푼다** — 임계 재튜닝으로는 못 고친다.
    #   ⓒ Canny/Sobel 을 끄면 검출률은 내려가지만 오프셋 도약 95% 가
    #      129→108px 로 떨어졌다. 엣지 채널은 잡음 공급원이 맞았다.
    #
    # top-hat 이 푸는 방식: 차선은 '밝다'가 아니라 **'양옆 바닥보다 밝은,
    # 폭이 정해진 줄무늬'** 다. 원본에서 (가로 열림)을 뺀 값은
    #   · 조명 절대값에 무관 (국소 대비만 본다 → 노출 요동에 면역)
    #   · 계단형 엣지(벽-바닥 경계)에 응답 ~0 → 원리적으로 배제
    #   · 커널보다 넓은 밝은 면(반사 웅덩이·과노출 영역)도 응답 0
    binarize_mode: str = "stripe"
    # 이보다 가로로 넓은 구조는 '배경'으로 보고 뺀다. BEV 에서 차선 두께는
    # 20~40px 이므로 그 2~3배가 안전하다. 너무 키우면 배경 제거가 약해진다.
    stripe_kernel_px: int = 81
    # 임계 = max(절대하한, 상위분위수 × 비율). 상대항이 조명 적응을 담당하고
    # 절대항이 '균일한 바닥에서 잡음이 폭주하는 것'을 막는다.
    stripe_pct: float = 99.0
    stripe_rel_frac: float = 0.45
    stripe_abs_min: float = 12.0

    # ── 흰색 차선 HSV (binarize_mode="legacy" 전용 — 화석) ──
    white_h: Tuple[int, int] = (0, 179)
    white_s: Tuple[int, int] = (0, 20)
    white_v: Tuple[int, int] = (243, 255)

    # ── 엣지 (legacy 전용) ──
    canny_low: int = 50
    canny_high: int = 150
    sobel_thresh: int = 50

    # ── 결합 방식 (0818 밤 교체) ──────────────────────────────────
    # "rowscan" = 행 밴드 스캔 + 폴리라인 추적 (기본). "histogram" = 옛 방식.
    #
    # 왜 바꿨나: 컬럼 히스토그램 + 슬라이딩 윈도우는 **BEV 에서 차선이 거의
    # 수직**임을 가정한다. 컬럼별 세로 합이 크려면 선이 서 있어야 하기 때문
    # 이다. 이 트랙은 최소 회전반경 1.01m 짜리 급코너라 코너에서 차선이 눕고,
    # 그러면 컬럼당 픽셀 수가 붕괴한다.
    #   실측: 코너 프레임에서 흰 선이 아주 깨끗하게 이진화됐는데 컬럼당 최대
    #   16행 → 게이트(25행) 탈락 → left_ok=right_ok=False → **차선 없음**.
    #   주행 프레임의 17% 가 이 게이트에서 탈락했고 그게 곧 코너 구간이다.
    #   "직선 1바퀴는 되는데 코너만 실패"의 정확한 기전.
    # 근본 원인은 단위 혼동이었다 — min_peak = 255*(min_pix//2) 는 min_pix
    # ('윈도우당 픽셀 수' 50) 를 '컬럼당 행 수' 25 로 재사용한 것이다.
    # 행 스캔은 각 행에서 밝은 구간의 중심을 찾아 위로 이어붙이므로 곡률과
    # 무관하다.
    assoc_mode: str = "rowscan"
    scan_band_px: int = 8           # 행 밴드 높이 (한 밴드 = 한 점)
    run_min_px: int = 3             # 밴드 안 밝은 구간의 허용 폭
    run_max_px: int = 70
    track_gate_px: float = 45.0     # 다음 밴드에서 이 거리 안의 후보만 잇는다
    track_max_miss: int = 2         # 연속 미스 허용 (점선·가림 대응)
    track_min_points: int = 4       # 폴리라인으로 인정할 최소 점 수
    track_min_span_frac: float = 0.15   # 세로로 ROI 의 이만큼은 뻗어야 한다
    # 폭 변동(std/median) 상한 — 차선은 폭이 일정하고 반사 웅덩이 테두리는
    # 들쭉날쭉하다. 합성 실측: 진짜 차선 0.19 / 웅덩이 1.26.
    track_width_cv_max: float = 0.60

    # ── 좌/우 정체성 (0818 밤 신설) ───────────────────────────────
    # 옛 코드는 '피크가 BEV 화면의 어느 절반에 있나'로만 좌/우를 정했다.
    # 코너에서 한 줄이 중앙을 넘어가면 라벨이 뒤집히고 목표가 차선폭만큼 튄다.
    #   실측: 좌/우 조합이 **유지**되는 구간의 오프셋 도약은 중앙 4px 인데,
    #   조합이 **바뀌는** 순간은 중앙 57px·최대 422px(≈차선폭 전체).
    #   즉 점프의 원인은 검출 실패가 아니라 라벨 뒤집힘이었다.
    # → 직전 프레임에서 그 선이 있던 x 와의 연속성으로 먼저 잇고, 새로 나타난
    #   선만 차량 중심 기준으로 좌/우를 준다.
    identity_gate_px: float = 90.0

    # ── 슬라이딩 윈도우 (assoc_mode="histogram" 전용) ──
    n_windows: int = 9
    margin: int = 100
    min_pix: int = 50
    min_contour_area: int = 500

    # ── 콘 주황 / 노란 중앙선 / 반사 (0818 낮 실차 프레임 분석) ──
    # 콘의 주황과 그 엣지가 차선 이진화에 새어 곡률을 오염시켰다(라바콘 구간
    # 조향 붕괴의 근원). 주황·노랑은 차선 이진화에서 **빼고**, 노랑은 별도
    # 중앙선 폴백으로 쓴다. 반사 웅덩이(유리창 빛)는 '뚱뚱한 블롭' 필터로
    # 제거한다 — 차선은 BEV 에서 두께 20~40px 의 가늘고 긴 구조다.
    orange_h: Tuple[int, int] = (0, 21)
    orange_s_min: int = 90
    orange_v_min: int = 80
    yellow_h: Tuple[int, int] = (22, 38)
    yellow_s_min: int = 70
    yellow_v_min: int = 110
    # ── 노란 중앙선을 '주역'으로 (0818 오후, 사용자 확인) ──
    # 트랙은 2차선이고 노란 점선이 도로 한가운데다. 규정상 차선 구분이 없어
    # (p.27) **노란선을 밟고 달려도 무방**하며, 실제 주행도 그렇게 한다.
    # 즉 노란선은 '차선 하나'가 아니라 **주행 목표선 그 자체**다.
    #   실측(run5 1196프레임): 노란선 79% / 흰 선 양쪽 2% / 흰 선 전무 32%.
    #   흰 선은 곡선에서 BEV 밖으로 밀려나지만 노란선은 한가운데라 남는다.
    #   게다가 반사광은 무채색이라 노란색 게이트를 통과하지 못한다.
    center_primary: bool = True
    # 출발·결승 체커존에는 **노란 호**가 가로로 칠해져 있다 (실측: 큰 노란
    # 조각의 58%가 가로지름). 이걸 중앙선으로 물면 결승선마다 홱 꺾는다 —
    # 긴 축이 진행방향(BEV 세로)에서 이 각도 안에 있는 조각만 중앙선으로 본다.
    center_max_tilt_deg: float = 50.0
    center_min_area: int = 60
    # 점선 조각이 세로로 이만큼(ROI 높이 비율) 퍼져 있어야 2차(곡률) 피팅을
    # 신뢰한다. 한 조각만 보이면 곡률은 잡음이므로 1차(직선)로 떨어뜨린다.
    center_curv_span_frac: float = 0.25
    # 흰 선 양쪽이 다 보일 때 노란선이 그 중점에서 이 비율 이상 떨어져 있으면
    # 중앙선이 아니라 딴 표식이다 — 그 프레임은 흰 선 기준으로 되돌린다.
    center_cross_check_frac: float = 0.35
    # 흰 선 ↔ 노란 중앙선 거리 [BEV px]. 흰 선만 보이는 프레임에서도 **같은
    # 물리 선**(도로 중앙)을 겨냥하려고 실측 학습한다. 학습 범위를 벗어난 값은
    # 오검이다.
    # ★ 0818 저녁 재실측 (run5 주행 247프레임, 현행 검출기로 직접 계측):
    #   오른 흰선−노란선 중앙값 332px (n=56, IQR 325~340) / 노란선−왼 흰선
    #   335px (n=24, IQR 328~343). 근거리 행과 원거리 행이 같다(334/340) —
    #   ROI 안에서 축척은 균일하다. 옛 기본값 190 은 다른 것을 잰 값이었고
    #   max 330 은 진짜 표본의 절반(50/84)을 오검으로 기각해 학습을 276 에서
    #   묶었다 → 단측 프레임(주행의 45%)마다 목표가 진짜 중앙에서 55~140px
    #   벽 쪽으로 밀렸다. 반차선 0.41m(줄자) 기준 ≈ 810 px/m.
    half_road_default_px: float = 335.0
    half_road_min_px: float = 200.0
    half_road_max_px: float = 430.0
    half_road_alpha: float = 0.12
    # ── 양쪽 흰 선 '쌍'의 타당성 (0818 저녁) ──
    # ROI 폭(640px)이 도로 전폭(≈670px)보다 좁아 근거리 행에서 흰 선 둘을
    # 동시에 보는 것은 원리적으로 거의 불가능하다. 즉 "양쪽 검출"의 대부분은
    # 한쪽이 가짜다 — 바닥 비닐 이음새(x≈300, #159~162 실측), 벽 밑단, 노란선
    # 테두리 헤일로. 그 가짜 쌍의 중점을 목표로 삼고 진짜 노란선을 교차검증
    # 으로 기각하면 직선에서 벽 쪽으로 조향한다 (+80 vs 진짜 −170).
    # → 근거리 간격이 이 범위일 때만 '진짜 쌍'으로 보고 중점·교차검증·폭
    #   히스토리에 쓴다. 아니면 span 이 긴 한쪽 + half_road 로 간다.
    pair_width_min_px: float = 520.0
    pair_width_max_px: float = 900.0
    # 중앙선 후보 기하 게이트: 중앙선은 흰 선 **사이**에만 있다. 이 여유[px]
    # 안쪽에 있는 후보만 인정한다 (오른 흰선 바깥의 올리브 바닥 조각이 노란
    # 게이트를 통과해 목표선이 되던 #112/#163/#220 차단 — 기하 불가능 픽 6→0).
    center_inside_margin_px: float = 20.0
    # 검출기 출력 위생 — 물리적으로 말이 안 되는 기울기·곡률은 여기서 자른다.
    # 짧은 피팅(점선 한 조각, 벽 줄무늬)의 2차 계수는 잡음이라, 자르지 않으면
    # 헤딩 항 단독이 서보 포화를 넘는 프레임이 41% 였다 (0818 저녁 실측).
    # 제어단 게이트(0818 아침 철회)와 달리 이건 '입력 잡음을 지각에서 잡는' 것.
    #   slope 0.6 ≈ 31° 어긋남 (그 이상은 이미 차선을 잃은 것)
    #   curv 3.5e-3 ≈ 반경 0.45m (트랙 최소 R≈1.0m → 1.5e-3, 여유 2배)
    max_heading_slope: float = 0.6
    max_curvature_px: float = 3.5e-3
    # 목표선의 프레임간 최대 이동 [px]. 30fps·1m/s 실제 횡이동은 ~15px 이라
    # 이보다 큰 도약은 검출 오류다 (체커존 노란 호 실측 최대 1142px 점프).
    target_max_step_px: float = 70.0
    # 이 프레임 수 이상 목표선을 놓쳤다가 재획득하면 제한 없이 스냅한다.
    target_snap_after_lost: int = 5
    exclude_dilate_px: int = 9      # 주황·노랑 제외 시 엣지 헤일로까지 덮는 팽창
    glare_max_side_px: int = 55     # minAreaRect 짧은 변이 이보다 크면 반사 웅덩이
    # …이면서 채움비가 이 이상일 때만 지운다. 채움비 조건이 없으면 휜 차선이
    # (곡선의 minAreaRect 는 뚱뚱하므로) 통째로 지워진다 — 0818 실측 확인.
    glare_min_fill: float = 0.55
    # ── 분기점(지름길 입구) 우측 우선 (0818 run4 랩3: 무단 지름길 진입) ──
    # 좌우 선 간격이 이보다 크면 두 갈래 길을 보고 있는 것이다. 본선은 항상
    # 오른쪽 가지 — 왼쪽 선을 버리고 오른쪽 선 기준으로 강등해 지름길(좌회전
    # 신호 없이는 규정 위반)로 새지 않게 한다.
    # 0818 저녁: 정상 도로 전폭이 ≈670px 로 재실측돼 650 은 정상 도로에서도
    # 발동하던 값이었다 → 950 (두 갈래 = 도로 하나 반 이상).
    fork_width_px: float = 950.0

    # ── 기하 ──
    # ★ 0818 저녁 재실측: 흰 선↔흰 선 = 반차선 335px × 2 ≈ **670px**.
    #   낮에 넣었던 335 는 옛 검출기가 노란 중앙선을 '흰 선'으로 잡아 잰
    #   흰↔노랑 거리였다. 이 값은 ① 곡률 피드포워드 게인의 축척(control.
    #   k_curvature: s_lat = width/road_width_m → 670/0.825 ≈ 812 px/m, 실측과
    #   일치) ② 좌/우 퇴화 병합 문턱(0.4× = 268px — 이음새 가짜선을 진짜와
    #   같은 물리 선으로 보고 span 긴 쪽만 남긴다) ③ lane_side 문턱에 쓰인다.
    default_lane_width_px: float = 670.0
    # 전방주시점 — ROI 높이 비율 (0=맨 위·가장 멂, 1=맨 아래·차 바로 앞).
    # ★ 0817 첫 자율주행: 기존 lookahead_row=10(맨 꼭대기 ≈2m 앞)은 코너가
    #   화면 위에 등장하는 순간 조향을 시작시켜 코너 진입 전에 꺾고, 정작
    #   코너 안에선 그 점이 화면 밖으로 나가는 실사고(사용자 관찰과 일치).
    #   0.6m/s 에서 2m 는 3초 앞. 중간 거리(0.55)로 내린다 — 밤 분석으로 재산출.
    lookahead_frac: float = 0.55
    width_history: int = 20


class LaneDetector:
    def __init__(self, config: Optional[LaneConfig] = None) -> None:
        self.config = config or LaneConfig()
        self._width_hist: deque[float] = deque(maxlen=self.config.width_history)
        self._cx_hist: deque[float] = deque(maxlen=10)
        # detect() 가 마지막으로 만든 BEV ROI (콘 검출 재사용용)
        self.last_bev_roi: Optional[np.ndarray] = None
        # _binarize 가 만든 노란 마스크: _last_yellow = 전부(이진화 제외용),
        # _center_yellow = 진행방향 조각만(중앙선 피팅용, 체커존 호 배제)
        self._last_yellow: Optional[np.ndarray] = None
        self._center_yellow: Optional[np.ndarray] = None
        # 목표선 일관성 상태 — 흰 선↔노란선 거리 학습값, 직전 목표, 소실 카운트
        self._half_road: float = self.config.half_road_default_px
        self._prev_target: Optional[float] = None
        self._lost_frames: int = 0
        # 좌/우 정체성 연속성 — 직전 프레임에서 각 선이 있던 근거리 x.
        # '화면 어느 절반'이 아니라 이 값과의 거리로 같은 선인지 판정한다.
        self._prev_left_x: Optional[float] = None
        self._prev_right_x: Optional[float] = None
        self.debug: Optional[np.ndarray] = None
        self._M: Optional[np.ndarray] = None
        self._M_shape: Optional[Tuple[int, int]] = None

    # ── BEV ───────────────────────────────────────────────────────
    def _warp_matrix(self, w: int, h: int) -> np.ndarray:
        if self._M is not None and self._M_shape == (w, h):
            return self._M
        c = self.config
        src = np.float32(
            [
                [w * c.src_bottom_inset, h * c.src_bottom_y],
                [w * (1 - c.src_bottom_inset), h * c.src_bottom_y],
                [w * (1 - c.src_top_inset), h * c.src_top_y],
                [w * c.src_top_inset, h * c.src_top_y],
            ]
        )
        dst = np.float32(
            [
                [w * c.dst_inset, h],
                [w * (1 - c.dst_inset), h],
                [w * (1 - c.dst_inset), 0],
                [w * c.dst_inset, 0],
            ]
        )
        self._M = cv2.getPerspectiveTransform(src, dst)
        self._M_shape = (w, h)
        return self._M

    def warp(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        return cv2.warpPerspective(
            frame, self._warp_matrix(w, h), (w, h), flags=cv2.INTER_LINEAR
        )

    def warp_matrix(self) -> Optional[np.ndarray]:
        """마지막 detect() 가 쓴 호모그래피 — 콘 원거리 베이스 투영에 재사용."""
        return self._M

    # ── 이진화 ────────────────────────────────────────────────────
    def _stripe_mask(self, roi: np.ndarray) -> np.ndarray:
        """폭 정합 top-hat — '양옆보다 밝은, 폭이 정해진 줄무늬'만 남긴다.

        top-hat = 원본 − (가로 방향 열림). 가로로 stripe_kernel_px 보다 넓은
        구조는 열림에서 살아남아 그대로 빠지므로 응답이 0 이 된다. 그래서
        벽·바닥 같은 넓은 면, 반사 웅덩이, 과노출 영역이 통째로 사라지고
        차선처럼 가는 것만 남는다. 계단형 엣지(벽-바닥 경계)도 밝은 쪽이
        넓은 면이라 응답이 없다 — Canny 가 절대 못 하던 구분이다.

        임계는 상대(상위 분위수 × 비율) 와 절대(하한) 중 큰 쪽이다. 상대항이
        노출 요동을 흡수하고, 절대항이 '아무것도 없는 균일한 바닥에서 잡음이
        상위 분위수를 차지해 폭주하는 것'을 막는다.
        """
        c = self.config
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        kern = cv2.getStructuringElement(
            cv2.MORPH_RECT, (max(3, int(c.stripe_kernel_px) | 1), 1)
        )
        resp = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kern)
        pos = resp[resp > 0]
        thr = c.stripe_abs_min
        if pos.size:
            thr = max(
                thr, float(np.percentile(pos, c.stripe_pct)) * c.stripe_rel_frac
            )
        mask = (resp >= thr).astype(np.uint8) * 255
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def _binarize(self, roi: np.ndarray) -> np.ndarray:
        c = self.config
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        if c.binarize_mode == "stripe":
            mask_w = self._stripe_mask(roi)
        else:
            mask_w = cv2.inRange(
                hsv,
                np.array([c.white_h[0], c.white_s[0], c.white_v[0]]),
                np.array([c.white_h[1], c.white_s[1], c.white_v[1]]),
            )
            mask_w = cv2.morphologyEx(
                mask_w, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)
            )

        # 반사 웅덩이 제거는 **흰 마스크 단계**에서 한다 (0818 2판).
        # 최종 이진화 단계에서 지우면 CLOSE 가 진짜 차선과 웅덩이를 한
        # 덩어리로 붙인 뒤라 차선까지 통째로 지워진다 (bag 재검증에서 흰 선
        # 단독 적중률 급락으로 발각). 차선은 가늘고(BEV 두께 20~40px) 웅덩이는
        # 뚱뚱하다 — minAreaRect 짧은 변으로 가른다.
        cnts_w, _ = cv2.findContours(
            mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        glare = np.zeros_like(mask_w)
        for cnt in cnts_w:
            (_gx, _gy), (rw, rh), _ang = cv2.minAreaRect(cnt)
            # ★ 짧은 변만으로 가르면 **코너에서 휜 차선을 통째로 지운다** —
            #   곡선의 minAreaRect 는 원래 뚱뚱하기 때문이다 (0818 실측:
            #   이 필터만 켜도 양쪽 검출 27.1%→0.8% 붕괴). 채움비를 같이 본다.
            #   웅덩이는 사각형을 꽉 채우고(fill 高), 휜 차선은 대각선으로
            #   가로지를 뿐이라 성기다(fill 低).
            rect_area = max(1.0, rw * rh)
            fill = float(cv2.contourArea(cnt)) / rect_area
            if min(rw, rh) > c.glare_max_side_px and fill >= c.glare_min_fill:
                cv2.drawContours(mask_w, [cnt], -1, 0, -1)
                # 웅덩이의 '경계 엣지'(Canny/Sobel 발화)도 가짜 차선이 된다 —
                # 영역을 팽창시켜 뒤에서 엣지 채널에서도 통째로 뺀다.
                cv2.drawContours(glare, [cnt], -1, 255, -1)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, c.canny_low, c.canny_high)

        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        _, sobel_bin = cv2.threshold(
            cv2.convertScaleAbs(sobelx), c.sobel_thresh, 255, cv2.THRESH_BINARY
        )

        combined = cv2.bitwise_or(cv2.bitwise_or(mask_w, edges), sobel_bin)

        # ── 콘 주황·노란 중앙선 제외 (0818 실차) ──
        # 콘의 몸통과 그 경계 엣지가 '차선'으로 들어와 곡률을 오염시켰고,
        # 노란 중앙선이 좌/우 베이스 한쪽을 훔쳐 폭 추정을 무너뜨렸다.
        # 팽창으로 경계 헤일로(Canny 발화 지점)까지 덮어서 뺀다.
        # 노랑 마스크는 버리지 않고 중앙선 폴백 피팅(detect)에 쓴다.
        mask_o = cv2.inRange(
            hsv,
            np.array([c.orange_h[0], c.orange_s_min, c.orange_v_min]),
            np.array([c.orange_h[1], 255, 255]),
        )
        mask_y = cv2.inRange(
            hsv,
            np.array([c.yellow_h[0], c.yellow_s_min, c.yellow_v_min]),
            np.array([c.yellow_h[1], 255, 255]),
        )
        mask_y = cv2.morphologyEx(mask_y, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # 이진화 제외용은 노란 픽셀 '전부' (출발선 호도 흰 채널에서 빼야 한다)
        self._last_yellow = mask_y
        # 중앙선 피팅용은 **진행방향으로 뻗은 조각만** (체커존 가로 호 배제)
        keep = np.zeros_like(mask_y)
        cnts_y, _ = cv2.findContours(
            mask_y, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in cnts_y:
            if cv2.contourArea(cnt) < c.center_min_area:
                continue
            (_yx, _yy), (rw, rh), ang = cv2.minAreaRect(cnt)
            # 긴 축이 BEV 세로축에서 몇 도 기울었나 (0=진행방향, 90=가로지름)
            theta = abs(ang) if rw >= rh else abs(ang + 90.0)
            dev = min(theta, abs(theta - 180.0))
            tilt_from_vertical = abs(90.0 - dev)
            if tilt_from_vertical <= c.center_max_tilt_deg:
                cv2.drawContours(keep, [cnt], -1, 255, -1)
        self._center_yellow = keep
        kd = np.ones((c.exclude_dilate_px, c.exclude_dilate_px), np.uint8)
        excl = cv2.dilate(
            cv2.bitwise_or(cv2.bitwise_or(mask_o, mask_y), glare), kd
        )
        combined = cv2.bitwise_and(combined, cv2.bitwise_not(excl))

        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        binary = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kern)

        # 작은 잡음 컨투어 제거 (반사 웅덩이는 흰 마스크 단계에서 이미 제거)
        cnts, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in cnts:
            if cv2.contourArea(cnt) < c.min_contour_area:
                cv2.drawContours(binary, [cnt], -1, 0, -1)
        return binary

    # ── 행 밴드 스캔 + 폴리라인 추적 (0818 밤 신설) ────────────────
    @staticmethod
    def _band_runs(
        mask: np.ndarray, y0: int, y1: int, w_min: int, w_max: int
    ) -> "list[tuple[float, float]]":
        """밴드 [y0,y1) 안 밝은 구간들의 (중심 x, 폭). 폭이 맞는 것만 남긴다.

        차선은 BEV 에서 폭이 정해진 구조다. 폭 게이트 하나로 얇은 잡음(1~2px)
        과 넓은 면(벽·반사)이 같이 걸러진다 — 컬럼 히스토그램에는 없던 방어다.
        """
        row = (mask[y0:y1] > 0).any(axis=0).astype(np.int8)
        d = np.diff(np.concatenate(([0], row, [0])))
        starts = np.flatnonzero(d == 1)
        ends = np.flatnonzero(d == -1)
        return [
            (0.5 * (float(s) + float(e) - 1.0), float(e - s))
            for s, e in zip(starts, ends)
            if w_min <= (e - s) <= w_max
        ]

    @staticmethod
    def _predict_x(pts: "list[tuple[float, float, float]]", y: float) -> float:
        """직전 두 점의 기울기로 다음 밴드의 x 를 외삽 — 곡선 추종의 핵심.

        기울기를 쓰기 때문에 차선이 아무리 누워도(코너) 따라간다. 컬럼
        히스토그램이 '수직 가정'에서 무너지던 바로 그 지점이다.
        """
        if len(pts) >= 2:
            y1, x1 = pts[-1][0], pts[-1][1]
            y2, x2 = pts[-2][0], pts[-2][1]
            if y1 != y2:
                return x1 + (x1 - x2) * ((y - y1) / (y1 - y2))
        return pts[-1][1]

    def _scan_tracks(
        self, mask: np.ndarray
    ) -> "list[list[tuple[float, float, float]]]":
        """아래(차 가까움)에서 위로 밴드를 훑으며 밝은 구간을 폴리라인으로 잇는다.

        각 점은 (y, x, 폭). 폭을 들고 다니는 이유는 아래 _fit_tracks 의 폭
        일관성 판정 때문이다.
        """
        c = self.config
        h = mask.shape[0]
        band = max(2, int(c.scan_band_px))
        tracks: "list[dict]" = []
        for y1 in range(h, 0, -band):
            y0 = max(0, y1 - band)
            y = 0.5 * (y0 + y1 - 1)
            cands = self._band_runs(mask, y0, y1, c.run_min_px, c.run_max_px)
            taken: "set[int]" = set()
            for t in tracks:
                if t["dead"]:
                    continue
                pred = self._predict_x(t["pts"], y)
                best = None
                for i, (x, wd) in enumerate(cands):
                    if i in taken:
                        continue
                    dist = abs(x - pred)
                    if dist <= c.track_gate_px and (best is None or dist < best[0]):
                        best = (dist, i, x, wd)
                if best is None:
                    # 점선·가림은 몇 밴드 비는 게 정상이다 — 바로 끊지 않는다.
                    t["miss"] += 1
                    if t["miss"] > c.track_max_miss:
                        t["dead"] = True
                else:
                    taken.add(best[1])
                    t["pts"].append((y, best[2], best[3]))
                    t["miss"] = 0
            for i, (x, wd) in enumerate(cands):
                if i not in taken:
                    tracks.append({"pts": [(y, x, wd)], "miss": 0, "dead": False})
        return [t["pts"] for t in tracks if len(t["pts"]) >= c.track_min_points]

    def _fit_tracks(self, mask: np.ndarray, h_roi: int) -> "list[LaneFit]":
        """폴리라인 → LaneFit. 세로 뻗음 + **폭 일관성**을 통과한 것만."""
        c = self.config
        out: "list[LaneFit]" = []
        for pts in self._scan_tracks(mask):
            arr = np.asarray(pts, dtype=float)
            ys, xs, ws = arr[:, 0], arr[:, 1], arr[:, 2]
            y_lo, y_hi = float(ys.min()), float(ys.max())
            if (y_hi - y_lo) < c.track_min_span_frac * h_roi:
                continue
            # ★ 폭 일관성 — 차선은 폭이 일정한 줄무늬이고, 반사 웅덩이의
            #   top-hat 응답은 테두리라 폭이 들쭉날쭉하다. 합성 실측:
            #   진짜 차선 std/median 0.19 vs 웅덩이 1.26 (6.6배 분리).
            #   길이·밝기로는 안 갈라지던 것이 이걸로 갈라진다.
            med = float(np.median(ws))
            if med <= 0.0:
                continue
            if float(np.std(ws)) / med > c.track_width_cv_max:
                continue
            # 점 수가 적으면 2차 곡률은 잡음이다 — 직선으로 떨어뜨려 곡률 0.
            if len(pts) >= 5:
                coef = np.polyfit(ys, xs, 2)
            else:
                b1, b0 = np.polyfit(ys, xs, 1)
                coef = np.array([0.0, b1, b0])
            out.append(LaneFit(coef=coef, y_lo=y_lo, y_hi=y_hi, width=med))
        return out

    def _assign_identity(
        self, fits: "list[LaneFit]", h_roi: int, w_roi: int
    ) -> "tuple[Optional[LaneFit], Optional[LaneFit]]":
        """좌/우 정체성 — **화면 절반이 아니라 직전 프레임과의 연속성**으로.

        옛 코드는 '피크가 BEV 의 어느 절반에 있나'로만 좌/우를 정했다. 코너
        에서 한 줄이 화면 중앙을 넘어가면 라벨이 뒤집히고 목표가 차선폭만큼
        튄다 (실측: 조합 유지 구간 도약 중앙 4px vs 조합 전환 순간 57px·
        최대 422px). 직전 프레임에서 그 선이 있던 x 에 먼저 잇고, 남은 것만
        차량 중심 기준으로 좌/우를 준다.
        """
        c = self.config
        y_near = float(h_roi - 1)
        cand = [(f.at(y_near), f) for f in fits]
        left = right = None
        used: "set[int]" = set()

        for side in (-1, 1):
            prev = self._prev_left_x if side < 0 else self._prev_right_x
            if prev is None:
                continue
            best = None
            for i, (x, f) in enumerate(cand):
                if i in used:
                    continue
                dist = abs(x - prev)
                if dist <= c.identity_gate_px and (best is None or dist < best[0]):
                    best = (dist, i, f)
            if best is not None:
                used.add(best[1])
                if side < 0:
                    left = best[2]
                else:
                    right = best[2]

        center = w_roi / 2.0
        if left is None:
            # 차 왼쪽에 있는 선들 중 **차에 가장 가까운**(= x 가 가장 큰) 것
            pool = [(x, i) for i, (x, _f) in enumerate(cand)
                    if i not in used and x < center]
            if pool:
                _x, i = max(pool)
                used.add(i)
                left = cand[i][1]
        if right is None:
            pool = [(x, i) for i, (x, _f) in enumerate(cand)
                    if i not in used and x >= center]
            if pool:
                _x, i = min(pool)
                used.add(i)
                right = cand[i][1]

        # 퇴화 방지: 좌우가 너무 가까우면 같은 물리 선을 둘로 센 것이거나
        # 하나가 가짜(바닥 이음새·테두리 헤일로)다. 테이프는 넓고 이음새는
        # 가늘다 — 넓은 쪽, 같으면 길게 관측된 쪽을 남긴다 (0818 저녁 #160:
        # 이음새 x≈300 이 '왼선'이 되어 진짜 오른선과 가짜 쌍을 만들었다).
        if left is not None and right is not None:
            if (right.at(y_near) - left.at(y_near)) < 0.4 * c.default_lane_width_px:
                if (left.width, left.span) >= (right.width, right.span):
                    right = None
                else:
                    left = None

        self._prev_left_x = left.at(y_near) if left is not None else None
        self._prev_right_x = right.at(y_near) if right is not None else None
        return left, right

    def _associate_rowscan(
        self, binary: np.ndarray, h_roi: int, w_roi: int
    ) -> "tuple[Optional[LaneFit], Optional[LaneFit]]":
        return self._assign_identity(self._fit_tracks(binary, h_roi), h_roi, w_roi)

    # ── 옛 결합 (컬럼 히스토그램 + 슬라이딩 윈도우) — 대조군 보존 ────
    def _associate_histogram(
        self, binary: np.ndarray, h_roi: int, w_roi: int
    ) -> "tuple[Optional[LaneFit], Optional[LaneFit]]":
        c = self.config
        hist = np.sum(binary[h_roi // 2 :, :], axis=0)
        mid = w_roi // 2
        left_base = int(np.argmax(hist[:mid]))
        right_base = int(np.argmax(hist[mid:]) + mid)

        min_peak = 255 * (c.min_pix // 2)
        left_ok = bool(hist[:mid].max() >= min_peak)
        right_ok = bool(hist[mid:].max() >= min_peak)
        if (
            left_ok
            and right_ok
            and (right_base - left_base) < 0.4 * c.default_lane_width_px
        ):
            if hist[left_base] >= hist[right_base]:
                right_ok = False
            else:
                left_ok = False

        win_h = h_roi // c.n_windows
        ys, xs = binary.nonzero()
        lx, rx = left_base, right_base
        idx_l, idx_r = [], []
        for i in range(c.n_windows):
            y_lo = h_roi - (i + 1) * win_h
            y_hi = h_roi - i * win_h
            good_l = (
                (ys >= y_lo) & (ys < y_hi)
                & (xs >= lx - c.margin) & (xs < lx + c.margin)
            ).nonzero()[0]
            good_r = (
                (ys >= y_lo) & (ys < y_hi)
                & (xs >= rx - c.margin) & (xs < rx + c.margin)
            ).nonzero()[0]
            idx_l.append(good_l)
            idx_r.append(good_r)
            if len(good_l) > c.min_pix:
                lx = int(np.mean(xs[good_l]))
            if len(good_r) > c.min_pix:
                rx = int(np.mean(xs[good_r]))

        def fit(idx_list, ok):
            idx = np.concatenate(idx_list) if idx_list else np.array([], dtype=int)
            if not ok or len(idx) <= c.min_pix:
                return None
            yy, xx = ys[idx], xs[idx]
            return LaneFit(
                coef=np.polyfit(yy, xx, 2),
                y_lo=float(yy.min()),
                y_hi=float(yy.max()),
            )

        return fit(idx_l, left_ok), fit(idx_r, right_ok)

    # ── 메인 ──────────────────────────────────────────────────────
    def detect(self, frame: Optional[np.ndarray]) -> LaneReading:
        if frame is None:
            return LaneReading()

        c = self.config
        h, w = frame.shape[:2]
        warped = self.warp(frame)
        roi = warped[h // 2 :, :]
        center_fit = None    # 노란 중앙선 폴백이 채워질 수 있다 (아래)
        # 콘 검출이 같은 BEV ROI 를 쓴다 — 호출부가 warp 를 또 돌리지 않도록
        # 여기서 캐시한다 (프레임당 워프 2회 → 1회, 리뷰 E1).
        self.last_bev_roi = roi
        h_roi, w_roi = roi.shape[:2]
        binary = self._binarize(roi)

        if c.assoc_mode == "rowscan":
            lf, rf = self._associate_rowscan(binary, h_roi, w_roi)
        else:
            lf, rf = self._associate_histogram(binary, h_roi, w_roi)

        yv = float(c.lookahead_frac) * float(h_roi - 1)
        y_near = float(h_roi - 1)
        mean_width = (
            float(np.mean(self._width_hist))
            if self._width_hist
            else c.default_lane_width_px
        )

        # LaneFit.at 은 관측 범위 밖을 **접선으로 선형 외삽**한다. 코너에서
        # 차선이 ROI 위쪽에만 남았을 때 2차식을 그대로 근거리까지 대입하면
        # 수백 px 씩 발산하고, 그게 그대로 조향 슬램이 된다.
        def at(fit, y):
            return fit.at(y)

        # 분기점 강등: 좌우 '선' 간격이 정상 차선폭을 한참 넘으면 두 갈래
        # 길(지름길 입구)이다 — 왼쪽 가지를 버리고 본선(오른쪽) 기준으로 간다.
        # (0818 run4 랩3: 신호 없이 지름길에 진입 — 규정 p.35 위반 리스크)
        if (
            lf is not None
            and rf is not None
            and (at(rf, y_near) - at(lf, y_near)) > c.fork_width_px
        ):
            lf = None

        # ── 목표선 = 노란 중앙선 (0818 오후, 사용자 확인) ──────────────
        # 이 트랙은 2차선이고 노란 점선이 도로 한가운데다. 규정상 차선 구분이
        # 없어(p.27) **노란선을 밟고 달리는 것이 실제 주행 방식**이다 — 한
        # 차선 안에 정확히 머무는 쪽이 오히려 더 자주 실패한다(사용자).
        # 따라서 노란선이 곧 목표선이다.
        #
        # ★ 진짜 핵심은 '어느 선을 보든 같은 물리 선을 겨냥한다'는 일관성이다.
        #   기존 코드는 흰 오른선만 보이면 목표를 (오른선 − 반차선폭)=오른차선
        #   중앙에, 노란선이 보이면 도로 중앙에 두었다. 프레임마다 목표가 다른
        #   선으로 갈아타면서 실측 |Δoffset| 95%가 147~164px, 최대 738px 튀었고
        #   그 점프가 그대로 조향 슬램이 됐다 (run5 "차선 인식을 잘 못한다").
        #   → 흰 선↔노란선 거리(half_road)를 학습해 흰 선만 보일 때도 노란선
        #     위치로 환산한다. 목표선은 언제나 도로 중앙 하나뿐이다.
        wl = at(lf, y_near) if lf is not None else None
        wr = at(rf, y_near) if rf is not None else None
        # 양쪽 흰 선이 '진짜 쌍'인가 — 간격이 도로 전폭 대역일 때만 (설정 주석).
        # ROI 가 도로보다 좁아 이 조건은 드물게만 참이고, 참이 아닌 "양쪽"은
        # 한쪽이 가짜(이음새·벽 밑단)라 중점도 교차검증도 믿을 수 없다.
        pair_ok = (
            wl is not None
            and wr is not None
            and c.pair_width_min_px <= (wr - wl) <= c.pair_width_max_px
        )

        # 노란 중앙선 후보는 흰 선 사이에서만 고른다 (기하 게이트, 설정 주석).
        cf_target = (
            self._fit_center(c, lo=wl, hi=wr) if c.center_primary else None
        )
        cy = at(cf_target, y_near) if cf_target is not None else None

        # 교차검증: 진짜 흰 선 쌍이 보이는데 노란선이 그 중점에서 크게 벗어나면
        # 중앙선이 아니라 딴 표식(체커존 호 등)이다 — 그 프레임은 버린다.
        # (가짜 쌍으로 진짜 노란선을 기각하던 것이 직선 우측 이탈의 기전이라
        #  pair_ok 일 때만 검사한다 — 0818 저녁 #159~162 실측.)
        if cy is not None and pair_ok:
            if abs(cy - (wl + wr) / 2.0) > c.center_cross_check_frac * max(
                1.0, wr - wl
            ):
                cy, cf_target = None, None

        # 흰 선↔노란선 거리 학습 (둘 다 보이는 프레임에서만, EMA)
        for d in (
            (wr - cy) if (cy is not None and wr is not None) else None,
            (cy - wl) if (cy is not None and wl is not None) else None,
        ):
            if d is not None and c.half_road_min_px <= d <= c.half_road_max_px:
                a_hr = c.half_road_alpha
                self._half_road = a_hr * d + (1.0 - a_hr) * self._half_road

        half = self._half_road
        if cy is not None:
            raw_near, raw_far = cy, at(cf_target, yv)
            center_fit = cf_target
        elif pair_ok:
            # 진짜 흰 선 쌍이 보이면 **그 중점**이 도로 중앙이다. 학습값
            # half_road 로 한쪽에서 환산하면 학습 오차가 그대로 편향이 된다.
            # half_road 는 단측 프레임 전용 폴백이다.
            raw_near = (wl + wr) / 2.0
            raw_far = (at(lf, yv) + at(rf, yv)) / 2.0
        elif wr is not None and (wl is None or (rf.width, rf.span) >= (lf.width, lf.span)):
            # 단측(또는 타당하지 않은 쌍 → 관측이 긴 쪽 하나만 신뢰)
            raw_near, raw_far = wr - half, at(rf, yv) - half
        elif wl is not None:
            raw_near, raw_far = wl + half, at(lf, yv) + half
        else:
            raw_near = raw_far = (
                float(np.mean(self._cx_hist)) if self._cx_hist else w_roi / 2.0
            )

        if cy is None and wl is None and wr is None:
            self._lost_frames += 1
        else:
            self._lost_frames = 0

        # 시간 일관성 게이트: 30fps·1m/s 에서 한 프레임의 실제 횡이동은 ~15px
        # 이다. 그보다 훨씬 큰 도약은 물리가 아니라 검출 오류(체커존 노란 호,
        # 옆 구간 점선)다 — 목표를 끌어당겨 조향 슬램을 막는다. 오래 놓쳤다가
        # 다시 잡은 경우(재획득)는 제한 없이 스냅한다.
        if (
            self._prev_target is not None
            and self._lost_frames <= c.target_snap_after_lost
        ):
            d = raw_near - self._prev_target
            if abs(d) > c.target_max_step_px:
                pull = (c.target_max_step_px if d > 0 else -c.target_max_step_px) - d
                raw_near += pull
                raw_far += pull
        self._prev_target = raw_near

        # 폭 히스토리는 '진짜 쌍'이 보일 때만 갱신 (곡률 게인 축척용).
        # 가짜 쌍(135~445px)을 넣으면 평균이 265 로 무너져 피드포워드 게인이
        # 2.5배 약해진다 (0818 저녁 실측) — 그게 코너 언더스티어의 한 축이었다.
        if pair_ok:
            self._width_hist.append(wr - wl)
            mean_width = float(np.mean(self._width_hist))

        xl_near, xr_near = raw_near - mean_width / 2.0, raw_near + mean_width / 2.0
        xl_far, xr_far = raw_far - mean_width / 2.0, raw_far + mean_width / 2.0

        center = w_roi / 2.0
        path_near = (xl_near + xr_near) / 2.0
        path_far = (xl_far + xr_far) / 2.0
        self._cx_hist.append(path_near)

        # ── 목표선의 기울기·곡률 (코너 제어의 핵심 입력) ──────────────
        # BEV 는 y 가 아래로 갈수록 차에 가깝다. '전방' 좌표 f = -y 로 보면
        #   x(f) 의 1차 미분 = -dx/dy,  2차 미분 = d²x/dy² = 2a
        # 부호 규약: + 면 전방에서 목표선이 오른쪽으로 간다 → 우조향(+).
        # 목표선이 노란 중앙선이면 기울기·곡률도 **그 선에서** 나와야 한다
        # (흰 선은 곡선에서 한쪽만 남아 폴리핏 끝단이 흔들린다).
        if center_fit is not None:
            fits = [center_fit]
        elif pair_ok:
            fits = [lf, rf]
        elif rf is not None and (lf is None or (rf.width, rf.span) >= (lf.width, lf.span)):
            fits = [rf]          # 목표선을 만든 바로 그 선에서 방향을 읽는다
        elif lf is not None:
            fits = [lf]
        else:
            fits = []
        if fits:
            # ★ 접선 평가점 = 관측 범위 안으로 클램프한 y (LaneFit.at 과 같은
            #   규약). 옛 코드는 -(2a·y_near+b) 로 관측 밖까지 2차식을 외삽했다
            #   — 위치는 접선 외삽인데 방향만 포물선 외삽이라 서로 다른 선을
            #   말했고, 짧은 피팅에서 |Δ|×k_heading 가 p90 85cmd 였다 (0818 저녁).
            heading_slope = float(np.mean([
                -(2.0 * f.a * min(max(y_near, f.y_lo), f.y_hi) + f.b) for f in fits
            ]))
            curvature_px = 2.0 * float(np.mean([f.a for f in fits]))
        else:
            heading_slope = 0.0
            curvature_px = 0.0
        # 출력 위생 — 물리적으로 불가능한 값은 잡음이다 (설정 주석)
        heading_slope = max(-c.max_heading_slope, min(c.max_heading_slope, heading_slope))
        curvature_px = max(-c.max_curvature_px, min(c.max_curvature_px, curvature_px))

        # (curvature 계산은 소비자가 없어 제거했다 — 리뷰 S4.
        #  필요해지면 폴리핏 최고차항 평균으로 복원하면 된다.)

        # 지금 어느 차선에 있나 — 점선(가운데) 기준 좌/우.
        # 규정 p.27 상 1·2차선 구분은 없지만 추월 방향 3순위 판단에 쓴다.
        offset = path_near - center
        lane_side = 0
        if abs(offset) > 0.18 * mean_width:
            lane_side = 1 if offset > 0 else -1

        return LaneReading(
            offset_px=float(offset),
            lookahead_px=float(path_far - center),
            heading_slope=float(heading_slope),
            curvature_px=float(curvature_px),
            left_found=lf is not None,
            right_found=rf is not None,
            center_found=center_fit is not None,
            lane_width_px=float(mean_width),
            lane_side=lane_side,
        )

    # ── 노란 중앙선 피팅 (주행 목표선 그 자체 — center_primary) ──────
    def _fit_center(
        self,
        c: LaneConfig,
        lo: Optional[float] = None,
        hi: Optional[float] = None,
    ) -> Optional[LaneFit]:
        """노란 중앙선(점선) 피팅 — 흰 선과 같은 행 스캔·추적을 쓴다.

        점선에 행 스캔이 특히 잘 맞는다: 추적기가 track_max_miss 밴드까지
        빈 구간을 견디므로 **끊긴 조각들을 하나의 선으로 이어 준다.** 옛
        슬라이딩 윈도우는 조각 사이가 비면 창이 그 자리에 멈춰 서서 다음
        조각을 놓쳤다.

        lo / hi: 이번 프레임의 왼/오른 흰 선 근거리 x (없으면 None). 중앙선은
        흰 선 **사이**에만 있으므로 그 밖의 후보는 버린다 — 오른 흰선 바깥의
        올리브 바닥(노란 게이트 통과)이 span 이 길어 목표선으로 뽑히던 것
        (#112 +247 / #163 +133 / #220) 을 막는다 (0818 저녁, 기하 불가능 6→0).
        """
        mask = self._center_yellow
        if mask is None or not mask.any():
            return None
        h_roi, w_roi = mask.shape[:2]

        if c.assoc_mode != "rowscan":
            return self._fit_center_histogram(c, mask, h_roi, w_roi)

        fits = self._fit_tracks(mask, h_roi)
        y_near = float(h_roi - 1)
        m = c.center_inside_margin_px
        if lo is not None:
            fits = [f for f in fits if f.at(y_near) >= lo + m]
        if hi is not None:
            fits = [f for f in fits if f.at(y_near) <= hi - m]
        if not fits:
            return None

        # ★ 후보 선택은 '가장 큰 노란 덩어리'가 아니라 **직전 목표선 근처**다.
        #   전역 최대를 고르면 출발·결승 체커존에서 가로로 칠해진 노란 호를
        #   물어 목표가 최대 1142px 튄다 (실측). 직전 목표 근처를 먼저 보고,
        #   거기 아무것도 없을 때만 가장 길게 뻗은 선으로 되돌아간다.
        if self._prev_target is not None:
            near = [
                f for f in fits
                if abs(f.at(y_near) - self._prev_target) <= c.margin
            ]
            if near:
                return max(near, key=lambda f: f.span)
        return max(fits, key=lambda f: f.span)

    def _fit_center_histogram(
        self, c: LaneConfig, mask: np.ndarray, h_roi: int, w_roi: int
    ) -> Optional[LaneFit]:
        """옛 중앙선 피팅 (컬럼 히스토그램 + 슬라이딩 윈도우) — 대조군 보존."""
        hist = np.sum(mask, axis=0)
        if hist.max() < 255 * (c.min_pix // 4):
            return None
        base = int(np.argmax(hist))
        if self._prev_target is not None:
            lo = int(max(0, self._prev_target - c.margin))
            hi = int(min(w_roi, self._prev_target + c.margin))
            if hi > lo:
                local = hist[lo:hi]
                if local.max() >= 255 * (c.min_pix // 4):
                    base = lo + int(np.argmax(local))
        ys, xs = mask.nonzero()
        win_h = h_roi // c.n_windows
        bx = base
        idx = []
        for i in range(c.n_windows):
            y_lo = h_roi - (i + 1) * win_h
            y_hi = h_roi - i * win_h
            good = (
                (ys >= y_lo) & (ys < y_hi)
                & (xs >= bx - c.margin) & (xs < bx + c.margin)
            ).nonzero()[0]
            idx.append(good)
            if len(good) > c.min_pix // 2:     # 점선은 성기다 — 절반이면 충분
                bx = int(np.mean(xs[good]))
        pts = np.concatenate(idx) if idx else np.array([], dtype=int)
        if len(pts) <= c.min_pix:
            return None
        yy, xx = ys[pts], xs[pts]
        span = float(yy.max() - yy.min())
        if span < c.center_curv_span_frac * h_roi:
            b1, b0 = np.polyfit(yy, xx, 1)
            coef = np.array([0.0, b1, b0])
        else:
            coef = np.polyfit(yy, xx, 2)
        return LaneFit(coef=coef, y_lo=float(yy.min()), y_hi=float(yy.max()))


__all__ = ["LaneConfig", "LaneDetector"]

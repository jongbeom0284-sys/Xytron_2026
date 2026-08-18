"""4구 신호등 인식 (규정 p.17~p.20, p.35~p.36).

램프 배치는 규정 도면에 확정돼 있다 — 좌→우 [빨강, 노랑, 좌회전←, 녹색].
점등 조합도 규정이 못박아 준다:
    출발      빨강        → 녹색            (직진 출발)
    2랩 시작  녹색        → 녹색+좌회전     (직좌 동시신호) 또는 녹색 유지
    3랩 시작  녹색                          (직진만)

그래서 '색'으로 푸는 게 아니라 **'네 번째 램프가 켜졌나 / 세 번째가 켜졌나'**
로 푼다. 색 기반 판정은 라바콘 주황을 빨강으로 오인하는 알려진 함정이 있는데,
위치 기반은 애초에 그 문제를 만들지 않는다.

구현 출처: mastic-choi/UMK `perception/traffic_signal.py` 의 HoughCircles + 배치검증
+ 조합 전수탐색 방식을 따랐다. 반사광으로 원이 4개보다 많이 잡히는 상황이 실차에서
흔하기 때문에 그 완화 처리가 특히 중요하다.

랩 마커 (`gantry_span_px`) — 2026-08-13 재설계:
  대회장(강의실형 실내)의 천장 조명이 HoughCircles 에 '일렬·등간격 원 조합'
  으로 잡힌다는 것이 run4 bag 포렌식으로 확정됐다 — 검증 span 상위 12개
  에피소드의 피크 프레임을 전수 육안 판정한 결과 12개 전부 조명이었고
  겐트리는 0개였다. 원 기반 span 은 이 대회장에서 랩 마커로 **불성립**한다.

  대신 **채도 있는 초록 램프 블롭의 가로폭**을 쓴다. 주행 중 신호등은 항상
  초록 계열(GREEN/GREEN_LEFT)이 점등돼 있고, 조명은 무채색(형광등) 또는
  주황(목재구간 펜던트)이라 '채도 초록'이 겐트리 고유 시그니처다.
  실측(run4, 38~42s 접근): H 75~95 / S 60~120 / V 130~255,
  블롭 폭 7px(원거리) → 14px → 35px(통과 직전).

  점등 분류(state)와는 독립이다 — 어떤 램프가 켜졌는지 못 읽어도 겐트리
  통과 사실은 랩 마커로 잡혀야 한다 (core/race_context.py LapTracker).

점등 분류의 채도 게이트 (같은 날 추가):
  '밝기만'으로 lit 을 정하면 천장 조명 원이 무작위 RED/GREEN/GREEN_LEFT 를
  만든다 (run4 에서 695프레임 오분류 실측 — 가짜 GREEN_LEFT 는 지름길
  오발동으로 직결). 그래서 lit 판정에 밝기 AND 해당 색 채도를 요구한다.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

from ..core.types import SignalReading, SignalState


@dataclass
class SignalConfig:
    """실차 튜닝 대상. config/params.yaml 에서 덮어쓴다."""

    # ROI (프레임 비율). 신호등은 화면 상단에 걸린다.
    # 2026-08-17 실차 bag 실측으로 확장: 출발 대기 시 램프가 y 0.49~0.53,
    # 통과 접근 시 x 0.02~0.65 에 잡힌다 — 구 ROI(0.05~0.45 / 0.20~0.85)는
    # 출발 대기 램프를 세로로, 접근 램프를 가로로 잘라먹고 있었다.
    roi_top: float = 0.03
    roi_bottom: float = 0.60
    roi_left: float = 0.02
    roi_right: float = 0.98

    min_radius: int = 8
    max_radius: int = 26
    # HoughCircles 파라미터
    hough_param1: int = 40
    hough_param2: int = 20
    hough_min_dist: int = 18

    # 점등 판정: 램프 밝기가 4개 평균보다 이만큼 밝으면 켜진 것
    bright_margin: float = 15.0
    # 원이 이보다 많이 잡히면 노이즈로 보고 포기 (조합 폭발 방지)
    max_candidates: int = 9

    # ── 채도 게이트 (run4 bag 실측 기반, 모듈 주석 참조) ──
    # 초록 램프 실측 H 75~95 — 약간의 여유를 둔다.
    green_h: Tuple[int, int] = (68, 98)
    # 빨강은 H 가 0 근처에서 감긴다: 0~red_h_max 또는 red_h_wrap~180
    red_h_max: int = 12
    red_h_wrap: int = 168
    chroma_min_s: int = 55          # 형광등(무채색) 배제 임계
    lamp_min_v: int = 110
    # lit 판정 시 램프 패치에서 해당 색 픽셀이 차지해야 하는 최소 비율
    color_frac_min: float = 0.20

    # ── 초록 램프 블롭 (랩 마커) ──
    # run4 bag 전수 A/B(설정 4종 × 11,056프레임)로 확정한 값. 이 조합에서
    # 진짜 통과 32px vs 최악 오검 14px 로 2배 이상 분리된다.
    blob_min_px: int = 8            # 이보다 작은 블롭은 노이즈
    blob_min_v: int = 170           # 램프는 발광체 — 창밖 수풀(V 낮음) 배제
    blob_fill_min: float = 0.55     # 원형 채움비 — 너덜한 반사 배제
    blob_max_aspect: float = 2.0    # 가로판(비상구 표지 등) 배제
    # 소등 램프 검사: 진짜 신호등이면 초록 왼쪽에 꺼진 램프(어두운 원)들이
    # 있다 (규정 배치 좌→우 [빨,노,좌,녹] — 초록 계열은 항상 오른쪽).
    # 단독 초록 발광체(비상구 유도등)를 구조적으로 걸러낸다.
    unlit_v_max: int = 90
    unlit_dark_frac_min: float = 0.15

    # ── 과노출 심 경로 (2026-08-17, kobot 새 카메라 실측 `run_0817_1211`) ──
    # 새 카메라(usb_cam 자동노출)는 실내가 전체적으로 과노출이라 점등 램프
    # 심이 거의 흰색으로 날아간다: 실측 심 H 78~85 / S 28~67(요동) / V 240~253.
    # 채도 게이트(S≥55)가 심을 통과 못 시키므로, 별도 경로를 둔다:
    #   "V≥lamp_core_v 흰 심(채도 무관) + 심 둘레 링(1.1r~1.9r)의 초록비율".
    # 진짜 램프 링초록 0.13~0.67 vs 형광등·반사 0.00~0.15 (동일 bag 전수 실측).
    lamp_core_v: int = 220          # 과노출 심 밝기 하한.
    # ⚠️ 210 으로 내리지 말 것 (0817 실측): 프린지(V 200~220)가 심 블롭에
    # 흡수되어 링 밴드가 프린지 밖을 훑게 된다 — 링 0.42 → 0.06 로 붕괴.
    # 원거리 소형 심 감도는 core_min_px=2 와 confirm_s 단축이 담당한다.
    ring_s_min: int = 30            # 링 픽셀이 '유채색'으로 인정될 채도 하한
    # 링 초록 hue. ★ 0818 실차 오검으로 (55,100)→(72,98): 이 대회장 바닥이
    # 황록색(H≈60~66)이라 바닥 위 흰 물체(종이·박스)가 전부 '흰 심+초록 링'
    # 으로 잡혀 유령 랩·가짜 지름길을 만들었다 (auto_0818_1153 t=35/50s).
    # 72~98 이면 진짜 램프 프린지(H 78~85)는 살고 바닥은 죽는다 (실측 분리).
    ring_h_green: Tuple[int, int] = (72, 98)
    ring_green_min: float = 0.20    # 링 초록비율 임계 (실측 분리대 0.15/0.20)
    ring_red_min: float = 0.20      # RED 점등판정용 링 빨강비율 임계
    core_min_px: int = 2            # 과노출 심 최소 가로폭. 출발 대기 거리
    #                                 (4~5m)에서 심이 2~4px 라 3 이면 플리커
    #                                 — 링·y대역·소등 게이트가 노이즈를 막는다
    core_max_px: int = 30           # 심 가로폭 상한 — 실측: 접근 창에서 ≤8px,
    #                                 그 이상 큰 흰 덩어리는 조명/근접 블로우아웃
    # 프린지는 심(V≥220) 가장자리 '바로 밖' 1~2px 에 있다 — 밴드를 심에
    # 붙인다 (실측: 밴드 0.9r~1.7r 에서 진짜 0.42~0.62, 1.1r~1.9r 은 0.03~0.25
    # 로 프린지를 벗어난다).
    ring_in_scale: float = 0.9
    ring_out_scale: float = 1.7
    # 과노출 화면에서는 소등 하우징도 V 117~172 로 '어둡지 않다' — 심 경로의
    # 소등 검사는 완화 임계로 따로 본다 (실측: 진짜 좌스트립 V10퍼센타일
    # 117~137 vs 책상 오검 대부분 149~215).
    washed_unlit_v_max: int = 150
    # ── 출발 전용 원거리 경로 (0817 실차 출발선 실측, 신호등 8~10m) ──
    # 그 거리에선 점등 심(7x6px)에 프린지가 없고 하우징 전체가 흰색으로
    # 날아가 링·소등 검사가 전부 실패한다. 남는 시그니처는 심의 '균형
    # 초록 끼': 실측 램프 G−R=+9.5 / G−B=+7.9 vs 같은 프레임 다른 흰
    # 후보 21개는 min(G−R,G−B) ≤ +5.0 (청록 반사는 G−B 만 크고, 형광등은
    # G−R 이 음수). min(G−R, G−B) ≥ 6 이 유일 분리자.
    start_tint_min: float = 6.0
    # 초록 심 페어 → GREEN_LEFT 상태 폴백 스위치. ⚠️ 기본 봉인(False):
    # 0817 두 bag 전수 조사에서 진짜 직좌 접근은 0발화, 책상/운반/주행 중
    # 오검 다수 (최악: 진짜 겐트리 접근 순간 6px 페어 오검 — 직진 랩
    # 지름길 오발동 경로). 페어는 span(랩 마커)에만 기여한다. 게이트
    # 재설계·재검증 후에만 켤 것.
    pair_green_left_enabled: bool = False
    # 심 후보의 세로 하한 (프레임 높이 비율). ★ 근/원거리 구분의 핵심 —
    # 겐트리 램프 높이와 카메라 피치가 정하는 기하. 실측 대역:
    #   0817 오전 bag: 통과·출발 램프 y_full 187~219 (0.39~0.46)
    #   0817 오후 bag: 겐트리 위치가 옮겨져(이동식) 접근 램프 0.32~0.37
    #     — 0.34 컷이 GREEN_LEFT 접근 전 구간을 전멸시킨 실사고. 0.30 으로.
    # 창밖 수풀 하늘 구멍(0.03~0.26, 랩마다 오발화 실측)·천장 반사는 여전히
    # 이 위라 전멸한다. ⚠️ 카메라 재장착·겐트리 이동 시 반드시 재실측.
    washed_min_y_frac: float = 0.30

    # 배치 검증 — min/max_radius 에서 파생되는 값이라 "N배" 관계를 그대로 둔다.
    @property
    def vert_diff_max(self) -> int:
        return self.max_radius * 2

    @property
    def horiz_diff_max(self) -> int:
        return self.max_radius * 11

    @property
    def min_gap(self) -> int:
        return self.min_radius * 3


class SignalDetector:
    """한 프레임 → SignalReading. 프레임 간 디바운스는 상위(FSM)가 한다."""

    def __init__(self, config: Optional[SignalConfig] = None) -> None:
        self.config = config or SignalConfig()
        self.roi: Optional[np.ndarray] = None
        self.last: SignalReading = SignalReading()

    # ── 기하 검증 ─────────────────────────────────────────────────
    def _shape_ok(self, circles: Sequence) -> Tuple[bool, str]:
        cfg = self.config
        xs = sorted(int(c[0]) for c in circles)
        ys = sorted(int(c[1]) for c in circles)

        if (ys[-1] - ys[0]) > cfg.vert_diff_max:
            return False, f"vert_spread={ys[-1] - ys[0]}>{cfg.vert_diff_max}"
        if (xs[-1] - xs[0]) > cfg.horiz_diff_max:
            return False, f"horiz_spread={xs[-1] - xs[0]}>{cfg.horiz_diff_max}"
        for i in range(len(xs) - 1):
            if (xs[i + 1] - xs[i]) < cfg.min_gap:
                return False, f"gap[{i}]={xs[i + 1] - xs[i]}<{cfg.min_gap}"
        return True, ""

    def _pick_best_4(self, circles: Sequence) -> Tuple[Optional[List], str]:
        """원이 4개보다 많이 잡혔을 때 가장 신호등다운 4개 조합을 고른다.

        여러 조합이 배치검증을 통과하면 세로 퍼짐이 가장 작은 것을 택한다 —
        같은 높이에 나란할수록 진짜 신호등일 확률이 높다는 가정.
        """
        best, best_score = None, None
        for combo in itertools.combinations(circles, 4):
            ok, _ = self._shape_ok(combo)
            if not ok:
                continue
            ys = [c[1] for c in combo]
            score = max(ys) - min(ys)
            if best_score is None or score < best_score:
                best, best_score = combo, score
        if best is None:
            return None, "no_valid_4subset"
        return list(best), ""

    @staticmethod
    def _brightness(gray: np.ndarray, x: int, y: int, r: int) -> float:
        """램프 중심 패치의 평균 밝기. 반경의 절반만 써서 테두리를 배제한다."""
        half = max(1, r // 2)
        y0, y1 = max(0, y - half), y + half
        x0, x1 = max(0, x - half), x + half
        patch = gray[y0:y1, x0:x1]
        if patch.size == 0:
            return 0.0
        return float(np.mean(patch))

    def _color_frac(self, hsv: np.ndarray, x: int, y: int, r: int, color: str) -> float:
        """램프 패치에서 해당 색(채도 포함) 픽셀의 비율.

        '밝기만'으로 lit 을 정하면 무채색 천장 조명이 켜진 램프로 오인된다.
        점등 램프는 반드시 유채색이므로 색 비율을 함께 요구한다.
        """
        cfg = self.config
        half = max(1, r // 2)
        y0, y1 = max(0, y - half), y + half
        x0, x1 = max(0, x - half), x + half
        patch = hsv[y0:y1, x0:x1]
        if patch.size == 0:
            return 0.0
        hh, ss, vv = patch[..., 0], patch[..., 1], patch[..., 2]
        sat = (ss >= cfg.chroma_min_s) & (vv >= cfg.lamp_min_v)
        if color == "red":
            hue = (hh <= cfg.red_h_max) | (hh >= cfg.red_h_wrap)
        else:  # green
            hue = (hh >= cfg.green_h[0]) & (hh <= cfg.green_h[1])
        return float(np.count_nonzero(sat & hue)) / float(hh.size)

    def _unlit_left_ok(
        self, v: np.ndarray, x: int, y: int, bw: int, bh: int,
        v_max: Optional[int] = None,
    ) -> bool:
        """소등 램프 검사 — 블롭 왼쪽 스트립에 어두운(꺼진 램프) 픽셀이
        충분히 있어야 한다 (규정 배치 좌→우 [빨,노,좌,녹] — 초록은 항상
        오른쪽이므로 왼쪽에 꺼진 하우징이 존재한다). 단독 초록 발광체
        (비상구 유도등 등)를 구조적으로 걸러낸다. 왼쪽이 프레임 밖이면
        (통과 직전 초근접) 이 블롭은 버리지만, 그 전 접근 프레임들에서
        이미 목격이 확정되므로 랩 카운팅에는 문제가 없다.

        v_max: '어둡다'의 임계. 기본은 unlit_v_max(채도 경로). 과노출 심
        경로는 washed_unlit_v_max 를 넘긴다 — 과노출 화면에선 소등 하우징도
        V 117~172 라 90 기준으로는 진짜 신호등이 전멸한다 (0817 실측)."""
        cfg = self.config
        thresh = cfg.unlit_v_max if v_max is None else v_max
        x0s = max(0, x - int(3.6 * bw))
        x1s = max(0, x - max(1, int(0.3 * bw)))
        y0s = max(0, y - int(0.2 * bh))
        y1s = min(v.shape[0], y + int(1.2 * bh))
        strip = v[y0s:y1s, x0s:x1s]
        if strip.size == 0:
            return False
        dark = np.count_nonzero(strip < thresh) / float(strip.size)
        return dark >= cfg.unlit_dark_frac_min

    def _ring_color_frac(
        self, hsv: np.ndarray, cx: int, cy: int, r: int, color: str
    ) -> float:
        """심 둘레 고리(1.1r~1.9r)에서 해당 색(유채색) 픽셀 비율.

        과노출로 심이 하얗게 날아가도 발광체 가장자리에는 원색 프린지가
        남는다 — 0817 bag 실측: 진짜 초록 램프 0.13~0.67, 조명/반사 ≤0.15.
        """
        cfg = self.config
        r_out = max(int(r * cfg.ring_out_scale), r + 2)
        r_in = max(int(r * cfg.ring_in_scale), 1)
        h_img, w_img = hsv.shape[:2]
        x0, x1 = max(0, cx - r_out), min(w_img, cx + r_out + 1)
        y0, y1 = max(0, cy - r_out), min(h_img, cy + r_out + 1)
        if x1 <= x0 or y1 <= y0:
            return 0.0
        patch = hsv[y0:y1, x0:x1]
        yy, xx = np.ogrid[y0:y1, x0:x1]
        d2 = (yy - cy) ** 2 + (xx - cx) ** 2
        ring = (d2 <= r_out * r_out) & (d2 > r_in * r_in)
        total = int(np.count_nonzero(ring))
        if total == 0:
            return 0.0
        hh, ss = patch[..., 0], patch[..., 1]
        sat = ss >= cfg.ring_s_min
        if color == "red":
            hue = (hh <= cfg.red_h_max) | (hh >= cfg.red_h_wrap)
        else:
            hue = (hh >= cfg.ring_h_green[0]) & (hh <= cfg.ring_h_green[1])
        return float(np.count_nonzero(hue & sat & ring)) / float(total)

    def _washed_core_blobs(self, hsv: np.ndarray) -> List[dict]:
        """과노출 심 경로의 후보 블롭 수집: 흰 심(V≥lamp_core_v) + 초록 링.

        kobot 새 카메라(자동노출)에서 점등 램프 심은 S 게이트를 통과하지
        못한다 (SignalConfig 의 과노출 심 섹션 실측 참조). 원형성·종횡비는
        여기서 걸고, 소등 램프 검사 결과는 unlit_ok 플래그로 남긴다 —
        직좌 동시점등(GREEN_LEFT)에서는 오른쪽 램프(녹색)의 왼쪽 이웃이
        '켜진' 화살표라 소등 검사가 정당하게 실패하기 때문에, 페어 판정이
        그 예외를 흡수한다 (왼쪽 멤버의 소등 검사로 구조를 보증).
        """
        cfg = self.config
        v = hsv[:, :, 2]
        roi_h = float(max(1, hsv.shape[0]))
        mask = (v >= cfg.lamp_core_v).astype(np.uint8) * 255
        # 2x2 open — 6~8px 심을 보존하면서 1px 노이즈만 제거 (3x3 은 과침식)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        n, _labels, stats, cent = cv2.connectedComponentsWithStats(mask)
        out: List[dict] = []
        for i in range(1, n):
            x, y, bw, bh, area = (int(stats[i][k]) for k in range(5))
            if bw < cfg.core_min_px or area < 4:
                continue
            if bw > cfg.core_max_px:
                # 대형 흰 덩어리(조명·근접 블로우아웃) — 램프 심이 아니다.
                continue
            aspect = max(bw / max(1, bh), bh / max(1, bw))
            if aspect > cfg.blob_max_aspect:
                continue
            if area / float(bw * bh) < cfg.blob_fill_min:
                continue
            cx, cy = int(cent[i][0]), int(cent[i][1])
            # 세로 하한 — 목격 거리 램프의 기하 대역 밖(수평선 위)은 창문
            # 수풀·원거리·천장 반사다 (washed_min_y_frac 주석의 실측).
            y_full_frac = cfg.roi_top + (cy / roi_h) * (cfg.roi_bottom - cfg.roi_top)
            if y_full_frac < cfg.washed_min_y_frac:
                continue
            r = max(bw, bh) // 2 + 1
            if self._ring_color_frac(hsv, cx, cy, r, "green") < cfg.ring_green_min:
                continue
            out.append(dict(
                x=x, y=y, bw=bw, bh=bh, cx=cx, cy=cy,
                unlit_ok=self._unlit_left_ok(
                    v, x, y, bw, bh, v_max=cfg.washed_unlit_v_max
                ),
            ))
        return out

    def _start_blob_px(self, roi_bgr: np.ndarray, hsv: np.ndarray) -> int:
        """출발 전용 원거리 초록 심 폭 [px]. 없으면 0. (랩 마커와 무관!)

        흰 심(V≥lamp_core_v, 3~30px, 원형, 수평선 아래) 중 '균형 초록 끼'
        (min(G−R, G−B) ≥ start_tint_min)를 띠는 것 — SignalConfig 의
        출발 전용 경로 실측 참조. 링·소등 검사는 요구하지 않는다.
        """
        cfg = self.config
        v = hsv[:, :, 2]
        roi_h = float(max(1, hsv.shape[0]))
        mask = (v >= cfg.lamp_core_v).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        n, labels, stats, cent = cv2.connectedComponentsWithStats(mask)
        best = 0
        for i in range(1, n):
            x, y, bw, bh, area = (int(stats[i][k]) for k in range(5))
            if bw < 3 or bw > cfg.core_max_px or area < 6:
                continue
            aspect = max(bw / max(1, bh), bh / max(1, bw))
            if aspect > cfg.blob_max_aspect:
                continue
            if area / float(bw * bh) < cfg.blob_fill_min:
                continue
            cy = int(cent[i][1])
            y_full_frac = cfg.roi_top + (cy / roi_h) * (cfg.roi_bottom - cfg.roi_top)
            if y_full_frac < cfg.washed_min_y_frac:
                continue
            m = labels == i
            px = roi_bgr[m].astype(np.float32)
            if px.size == 0:
                continue
            b_mean, g_mean, r_mean = float(px[:, 0].mean()), float(px[:, 1].mean()), float(px[:, 2].mean())
            if min(g_mean - r_mean, g_mean - b_mean) < cfg.start_tint_min:
                continue
            best = max(best, bw)
        return best

    def _washed_pair(self, blobs: List[dict]) -> Optional[Tuple[dict, dict]]:
        """직좌 동시점등 시그니처: 초록 심 블롭 2개가 나란히.

        규정 배치 좌→우 [빨,노,좌,녹] 에서 직좌 신호는 3·4번 램프가 함께
        켜진다 — 과노출에서는 '흰 심+초록 링' 2개가 한 램프 간격으로 나란한
        모양이 된다 (0817 오후 bag f320 실측). 이 대회장은 배경 원 잡음
        (의자 바퀴 등 25~45개)으로 4구 원 분류가 불가능해, 원 없이 이 페어가
        GREEN_LEFT 의 근거다. 왼쪽 멤버는 소등 검사(빨·노 꺼짐)를 통과해야
        한다 — 구조 보증은 왼쪽이 담당하고 오른쪽은 면제(이웃이 켜진 화살표).
        """
        cfg = self.config
        cands = sorted(blobs, key=lambda b: b["cx"])
        for i in range(len(cands) - 1):
            left, right = cands[i], cands[i + 1]
            if not left["unlit_ok"]:
                continue
            w = max(left["bw"], right["bw"])
            if max(left["bw"], right["bw"]) > 2.5 * min(left["bw"], right["bw"]):
                continue
            gap = right["cx"] - left["cx"]
            if not (1.2 * w <= gap <= 4.5 * w):
                continue
            if abs(left["cy"] - right["cy"]) > 1.5 * max(left["bh"], right["bh"]):
                continue
            return left, right
        return None

    def _washed_core_span(self, hsv: np.ndarray) -> int:
        """과노출 심 경로의 랩 마커 span. (블롭 수집은 detect() 와 공유)"""
        blobs = self._washed_core_blobs(hsv)
        best = max((b["bw"] for b in blobs if b["unlit_ok"]), default=0)
        pair = self._washed_pair(blobs)
        if pair is not None:
            best = max(best, pair[0]["bw"], pair[1]["bw"])
        return best

    def _chroma_span(self, hsv: np.ndarray) -> int:
        """채도 초록 블롭 경로의 랩 마커 span — 정상 노출 카메라용 (run4 실측).

        원 검출과 완전히 독립이다 — 천장 조명이 원 조합을 오염시켜도
        램프 블롭은 겐트리에서만 나온다는 것이 run4 bag 실측의 결론이다.
        """
        cfg = self.config
        # 단일 inRange 콜 — split+불리언 조합(배열 패스 ~11회)과 의미 동일하나
        # SIMD 한 번으로 끝난다 (30fps 핫패스, 리뷰 E4).
        mask = cv2.inRange(
            hsv,
            (cfg.green_h[0], cfg.chroma_min_s, cfg.blob_min_v),
            (cfg.green_h[1], 255, 255),
        )
        v = hsv[:, :, 2]
        roi_h = float(max(1, hsv.shape[0]))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        n, _labels, stats, cent = cv2.connectedComponentsWithStats(mask)
        best = 0
        for i in range(1, n):
            x, y, bw, bh, area = (int(stats[i][k]) for k in range(5))
            if area < cfg.blob_min_px:
                continue
            aspect = max(bw / max(1, bh), bh / max(1, bw))
            if aspect > cfg.blob_max_aspect:
                continue
            if area / float(bw * bh) < cfg.blob_fill_min:
                continue
            # 수평선 컷 — 기하 조건이라 채도 경로에도 동일하게 적용한다
            cy = float(cent[i][1])
            y_full_frac = cfg.roi_top + (cy / roi_h) * (cfg.roi_bottom - cfg.roi_top)
            if y_full_frac < cfg.washed_min_y_frac:
                continue
            if not self._unlit_left_ok(v, x, y, bw, bh):
                continue
            best = max(best, bw)
        return best

    # ── 메인 ──────────────────────────────────────────────────────
    def detect(self, frame: Optional[np.ndarray]) -> SignalReading:
        if frame is None:
            self.last = SignalReading(reject_reason="no_frame")
            return self.last

        cfg = self.config
        h, w = frame.shape[:2]
        t, b = int(h * cfg.roi_top), int(h * cfg.roi_bottom)
        l, r = int(w * cfg.roi_left), int(w * cfg.roi_right)
        self.roi = frame[t:b, l:r]

        # 랩 마커는 원 검출과 독립적으로 블롭에서 계산한다.
        # 과노출 블롭은 한 번만 수집해 span·페어 판정에 공유한다.
        hsv = cv2.cvtColor(self.roi, cv2.COLOR_BGR2HSV)
        washed_blobs = self._washed_core_blobs(hsv)
        pair = self._washed_pair(washed_blobs)
        washed_span = max(
            (bb["bw"] for bb in washed_blobs if bb["unlit_ok"]), default=0
        )
        if pair is not None:
            washed_span = max(washed_span, pair[0]["bw"], pair[1]["bw"])
        gantry_span = max(self._chroma_span(hsv), washed_span)
        start_blob = self._start_blob_px(self.roi, hsv)

        # 4구 원 분류가 무너졌을 때의 대체 판정 후보 — 단 페어→상태 승격은
        # 기본 봉인 상태다 (pair_green_left_enabled 주석의 0817 FP 실측).
        fallback_state = (
            SignalState.GREEN_LEFT
            if (pair is not None and cfg.pair_green_left_enabled)
            else SignalState.UNKNOWN
        )

        gray = cv2.cvtColor(self.roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        found = cv2.HoughCircles(
            blur,
            cv2.HOUGH_GRADIENT,
            1,
            cfg.hough_min_dist,
            param1=cfg.hough_param1,
            param2=cfg.hough_param2,
            minRadius=cfg.min_radius,
            maxRadius=cfg.max_radius,
        )

        if found is None:
            self.last = SignalReading(
                state=fallback_state,
                reject_reason="no_circles", gantry_span_px=gantry_span,
                start_blob_px=start_blob,
            )
            return self.last

        circles = np.round(found[0, :]).astype(int)
        n = len(circles)

        if n > cfg.max_candidates:
            self.last = SignalReading(
                state=fallback_state,
                circle_count=n,
                reject_reason=f"circle_count={n}(>{cfg.max_candidates}, noisy)",
                gantry_span_px=gantry_span,
                start_blob_px=start_blob,
            )
            return self.last
        if n < 4:
            self.last = SignalReading(
                state=fallback_state,
                circle_count=n,
                reject_reason=f"circle_count={n}(<4)",
                gantry_span_px=gantry_span,
                start_blob_px=start_blob,
            )
            return self.last

        if n == 4:
            ok, reason = self._shape_ok(circles)
            chosen = list(circles) if ok else None
        else:
            chosen, reason = self._pick_best_4(circles)

        if chosen is None:
            self.last = SignalReading(
                state=fallback_state,
                circle_count=n,
                reject_reason=f"circle_count={n} {reason}",
                gantry_span_px=gantry_span,
                start_blob_px=start_blob,
            )
            return self.last

        # 좌→우 정렬 = [빨강, 노랑, 좌회전, 녹색]
        ordered = sorted(chosen, key=lambda c: c[0])
        bright = [self._brightness(gray, int(x), int(y), int(rr)) for x, y, rr in ordered]
        avg = float(np.mean(bright))
        lit = [bv - avg > cfg.bright_margin for bv in bright]

        # 채도 게이트 — 무채색으로 밝은 것(천장 조명·반사)은 켜진 램프가 아니다.
        # 가짜 GREEN_LEFT 한 번이면 지름길 오발동(=차선이탈)이라 여기서 막는다.
        # 과노출 카메라 보완(0817): 심이 하얗게 날아가 색 비율이 0 에 가까운
        # 진짜 점등은 '흰 심 + 해당 색 링 프린지'로도 인정한다.
        def _lit_chroma(idx: int, color: str) -> bool:
            x, y, rr = int(ordered[idx][0]), int(ordered[idx][1]), int(ordered[idx][2])
            if not lit[idx]:
                return False
            if self._color_frac(hsv, x, y, rr, color) >= cfg.color_frac_min:
                return True
            # 과노출 심 경로: 패치 안이 충분히 하얗게 포화됐고 링에 원색이 남는가
            half = max(1, rr // 2)
            patch_v = hsv[max(0, y - half): y + half, max(0, x - half): x + half, 2]
            if patch_v.size == 0:
                return False
            white = float(np.count_nonzero(patch_v >= cfg.lamp_core_v)) / patch_v.size
            if white < 0.30:
                return False
            ring_min = cfg.ring_red_min if color == "red" else cfg.ring_green_min
            # Hough 반지름은 프린지 외곽에 잡히기도 한다 — 링 밴드를 몇 개
            # 스케일에서 훑어 최댓값을 쓴다 (프린지 위치 오차에 강건).
            frac = max(
                self._ring_color_frac(hsv, x, y, max(2, int(rr * s)), color)
                for s in (0.6, 0.8, 1.0)
            )
            return frac >= ring_min

        red_lit = _lit_chroma(0, "red")
        left_lit = _lit_chroma(2, "green")
        green_lit = _lit_chroma(3, "green")

        # 규정에 실제로 존재하는 조합만 상태로 매핑한다.
        if green_lit and left_lit:
            state = SignalState.GREEN_LEFT   # 직좌 동시신호 → 지름길 가능
        elif green_lit:
            state = SignalState.GREEN
        elif red_lit:
            state = SignalState.RED
        else:
            # 원 분류가 점등을 못 읽었으면 블롭 페어 판정이 대신한다.
            state = fallback_state

        self.last = SignalReading(
            state=state,
            lamp_brightness=tuple(round(v, 1) for v in bright),
            circle_count=n,
            reject_reason="" if state is not SignalState.UNKNOWN else "no_lamp_lit",
            gantry_span_px=gantry_span,
                start_blob_px=start_blob,
        )
        return self.last

    # ── 디버그 오버레이 ───────────────────────────────────────────
    def annotate(self) -> Optional[np.ndarray]:
        """튜닝용 시각화. 대회 당일에는 호출하지 않는다 (GUI 가 프레임을 잡아먹음)."""
        if self.roi is None:
            return None
        vis = self.roi.copy()
        r = self.last
        color = {
            SignalState.GREEN: (0, 255, 0),
            SignalState.GREEN_LEFT: (0, 255, 255),
            SignalState.RED: (0, 0, 255),
            SignalState.UNKNOWN: (150, 150, 150),
        }[r.state]
        cv2.putText(
            vis,
            f"{r.state.value} n={r.circle_count} span={r.gantry_span_px}",
            (4, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            color,
            1,
            cv2.LINE_AA,
        )
        if r.reject_reason:
            cv2.putText(
                vis, r.reject_reason, (4, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1, cv2.LINE_AA,
            )
        return vis


__all__ = ["SignalConfig", "SignalDetector"]

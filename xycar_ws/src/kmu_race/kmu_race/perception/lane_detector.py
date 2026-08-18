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
class LaneConfig:
    # ── BEV 원근 변환 (프레임 비율) ──
    src_bottom_y: float = 0.90
    src_top_y: float = 0.60
    src_bottom_inset: float = 0.10
    src_top_inset: float = 0.40
    dst_inset: float = 0.20

    # ── 흰색 차선 HSV ──
    white_h: Tuple[int, int] = (0, 179)
    white_s: Tuple[int, int] = (0, 20)
    white_v: Tuple[int, int] = (243, 255)

    # ── 엣지 ──
    canny_low: int = 50
    canny_high: int = 150
    sobel_thresh: int = 50

    # ── 슬라이딩 윈도우 ──
    n_windows: int = 9
    margin: int = 100
    min_pix: int = 50
    min_contour_area: int = 500

    # ── 기하 ──
    default_lane_width_px: float = 450.0
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

    # ── 이진화 ────────────────────────────────────────────────────
    def _binarize(self, roi: np.ndarray) -> np.ndarray:
        c = self.config
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask_w = cv2.inRange(
            hsv,
            np.array([c.white_h[0], c.white_s[0], c.white_v[0]]),
            np.array([c.white_h[1], c.white_s[1], c.white_v[1]]),
        )
        mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, c.canny_low, c.canny_high)

        sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        _, sobel_bin = cv2.threshold(
            cv2.convertScaleAbs(sobelx), c.sobel_thresh, 255, cv2.THRESH_BINARY
        )

        combined = cv2.bitwise_or(cv2.bitwise_or(mask_w, edges), sobel_bin)
        kern = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        binary = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, kern)

        # 작은 잡음 컨투어 제거
        cnts, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for cnt in cnts:
            if cv2.contourArea(cnt) < c.min_contour_area:
                cv2.drawContours(binary, [cnt], -1, 0, -1)
        return binary

    # ── 메인 ──────────────────────────────────────────────────────
    def detect(self, frame: Optional[np.ndarray]) -> LaneReading:
        if frame is None:
            return LaneReading()

        c = self.config
        h, w = frame.shape[:2]
        warped = self.warp(frame)
        roi = warped[h // 2 :, :]
        # 콘 검출이 같은 BEV ROI 를 쓴다 — 호출부가 warp 를 또 돌리지 않도록
        # 여기서 캐시한다 (프레임당 워프 2회 → 1회, 리뷰 E1).
        self.last_bev_roi = roi
        h_roi, w_roi = roi.shape[:2]
        binary = self._binarize(roi)

        # 히스토그램으로 좌/우 베이스 찾기
        hist = np.sum(binary[h_roi // 2 :, :], axis=0)
        mid = w_roi // 2
        left_base = int(np.argmax(hist[:mid]))
        right_base = int(np.argmax(hist[mid:]) + mid)

        # argmax 는 전부 0 이어도 0 을 반환한다 — 피크에 실체가 없으면 그쪽
        # 탐색을 포기한다 (x=0 가장자리 잡음을 차선으로 피팅하는 사고 방지).
        min_peak = 255 * (c.min_pix // 2)
        left_ok = bool(hist[:mid].max() >= min_peak)
        right_ok = bool(hist[mid:].max() >= min_peak)
        # 중앙 근처 라인 하나가 좌/우 양쪽 베이스로 잡히는 퇴화 방지 —
        # 베이스 간격이 차선폭의 40% 미만이면 같은 물리 라인이다. 강한 쪽만.
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
                (ys >= y_lo) & (ys < y_hi) & (xs >= lx - c.margin) & (xs < lx + c.margin)
            ).nonzero()[0]
            good_r = (
                (ys >= y_lo) & (ys < y_hi) & (xs >= rx - c.margin) & (xs < rx + c.margin)
            ).nonzero()[0]
            idx_l.append(good_l)
            idx_r.append(good_r)
            if len(good_l) > c.min_pix:
                lx = int(np.mean(xs[good_l]))
            if len(good_r) > c.min_pix:
                rx = int(np.mean(xs[good_r]))

        left_idx = np.concatenate(idx_l) if idx_l else np.array([], dtype=int)
        right_idx = np.concatenate(idx_r) if idx_r else np.array([], dtype=int)

        lf = (
            np.polyfit(ys[left_idx], xs[left_idx], 2)
            if left_ok and len(left_idx) > c.min_pix
            else None
        )
        rf = (
            np.polyfit(ys[right_idx], xs[right_idx], 2)
            if right_ok and len(right_idx) > c.min_pix
            else None
        )

        yv = float(c.lookahead_frac) * float(h_roi - 1)
        y_near = float(h_roi - 1)
        mean_width = (
            float(np.mean(self._width_hist))
            if self._width_hist
            else c.default_lane_width_px
        )

        def at(fit, y):
            return fit[0] * y * y + fit[1] * y + fit[2]

        if lf is not None and rf is not None:
            xl_far, xr_far = at(lf, yv), at(rf, yv)
            xl_near, xr_near = at(lf, y_near), at(rf, y_near)
            width = xr_near - xl_near
            if 0.4 * c.default_lane_width_px < width < 2.0 * c.default_lane_width_px:
                self._width_hist.append(width)
        elif lf is not None:
            xl_far = at(lf, yv)
            xr_far = xl_far + mean_width
            xl_near = at(lf, y_near)
            xr_near = xl_near + mean_width
        elif rf is not None:
            xr_far = at(rf, yv)
            xl_far = xr_far - mean_width
            xr_near = at(rf, y_near)
            xl_near = xr_near - mean_width
        else:
            prev = float(np.mean(self._cx_hist)) if self._cx_hist else w_roi / 2.0
            xl_far = xl_near = prev - mean_width / 2.0
            xr_far = xr_near = prev + mean_width / 2.0

        center = w_roi / 2.0
        path_near = (xl_near + xr_near) / 2.0
        path_far = (xl_far + xr_far) / 2.0
        self._cx_hist.append(path_near)

        # ── 목표선의 기울기·곡률 (코너 제어의 핵심 입력) ──────────────
        # BEV 는 y 가 아래로 갈수록 차에 가깝다. '전방' 좌표 f = -y 로 보면
        #   x(f) 의 1차 미분 = -dx/dy,  2차 미분 = d²x/dy² = 2a
        # 부호 규약: + 면 전방에서 목표선이 오른쪽으로 간다 → 우조향(+).
        fits = [f for f in (lf, rf) if f is not None]
        if fits:
            a = float(np.mean([f[0] for f in fits]))
            b = float(np.mean([f[1] for f in fits]))
            heading_slope = -(2.0 * a * y_near + b)
            curvature_px = 2.0 * a
        else:
            heading_slope = 0.0
            curvature_px = 0.0

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
            lane_width_px=float(mean_width),
            lane_side=lane_side,
        )


__all__ = ["LaneConfig", "LaneDetector"]

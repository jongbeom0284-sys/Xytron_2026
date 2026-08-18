"""라바콘 검출 — 좌우 두 줄 사이의 '게이트'를 찾는다 (규정 p.23~p.25).

규정 도면은 콘이 **좌우 두 줄**로 놓이고 그 사이를 뱀처럼 통과하는 형태다.
그래서 흔한 구현 방식인 '주황 마스크 히스토그램의 argmax 하나를 추종'은
위험하다 — 픽셀이 많은 한쪽 줄만 따라가게 되고, 콘을 건드리면 개당 3초다.

여기서는 BEV 에서 가까운 밴드를 잘라 주황 컨투어의 중심들을 x 로 정렬하고,
**y 가 비슷한(같은 깊이) 인접 콘 쌍 중 가장 넓은 간격**을 게이트로 삼아 그
중점을 게이트 중앙으로 보고한다. y 근접 조건이 없으면 곡선에서 같은 줄의
앞뒤 콘 사이가 게이트로 오인되어 콘 줄 한가운데로 조향하게 된다(리뷰 실측).
한쪽 줄만 보이면 gate_valid=False 로 보고하고, 상위(policy)가 반발 조향으로
대체한다.

주황 HSV 기본값은 팀이 실차에서 튜닝한 값(`segi/my_driver_labacorn.py` 의
[0,108,224]~[28,255,255])을 기준으로 잡았다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..core.types import ConeReading


@dataclass
class ConeConfig:
    # 주황 HSV
    lower: Tuple[int, int, int] = (0, 108, 224)
    upper: Tuple[int, int, int] = (28, 255, 255)

    # 게이트를 찾을 밴드 — BEV ROI 하단에서 이 비율만큼. 가까운 콘만 본다.
    band_top: float = 0.35
    band_bottom: float = 1.0

    # 콘 하나로 인정할 최소 컨투어 면적
    min_area: int = 90
    # 게이트 계산을 건너뛸 노이즈 하한. 주의: 구간 '진입' 임계는 FSM 의
    # cone_min_orange_px(900) 하나다 — 여기 값을 그보다 높이면 FSM 노브를
    # 내려도 게이트가 안 생겨 소리 없이 무효가 된다 (리뷰 S2). 항상 FSM
    # 임계보다 충분히 낮게 유지할 것.
    min_section_px: int = 300

    # 게이트로 인정할 좌우 콘 사이 최소 간격 [px].
    # 차폭이 통과 가능한 폭이어야 한다. BEV px_per_m 기준으로 잡을 것.
    min_gate_px: float = 110.0
    # 게이트 후보가 너무 넓으면 콘 두 줄이 아니라 서로 다른 구간을 본 것이다.
    max_gate_px: float = 620.0
    # 게이트 쌍의 y(깊이) 근접 허용치 [px]. 같은 줄의 앞뒤 콘(=y 차이 큼)이
    # 게이트로 오인되는 것을 막는다.
    gate_y_tol_px: float = 70.0


class ConeDetector:
    def __init__(self, config: Optional[ConeConfig] = None) -> None:
        self.config = config or ConeConfig()
        self.mask: Optional[np.ndarray] = None
        self._last_gate: float = 0.0

    def detect(self, bev_roi: Optional[np.ndarray]) -> ConeReading:
        """입력은 LaneDetector 와 같은 BEV ROI (warped 하단 절반)."""
        if bev_roi is None:
            return ConeReading()

        cfg = self.config
        h, w = bev_roi.shape[:2]
        hsv = cv2.cvtColor(bev_roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(cfg.lower), np.array(cfg.upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        self.mask = mask

        total_px = int(np.count_nonzero(mask))
        if total_px < cfg.min_section_px:
            return ConeReading(detected=False, orange_px=total_px)

        y0 = int(h * cfg.band_top)
        y1 = int(h * cfg.band_bottom)
        band = mask[y0:y1, :]

        cnts, _ = cv2.findContours(
            band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        center = w / 2.0
        cones: List[Tuple[float, float, int]] = []   # (cx, cy, area)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < cfg.min_area:
                continue
            m = cv2.moments(cnt)
            if m["m00"] == 0:
                continue
            cones.append((m["m10"] / m["m00"], m["m01"] / m["m00"], int(area)))

        # 좌/우를 화면 중앙으로 가르지 않는다. 콘 통로가 곡선에서 화면 한쪽으로
        # 통째로 밀리면 그 방식은 양쪽 줄을 모두 '왼쪽'으로 분류해 버린다.
        # 대신 콘들을 x 로 정렬해 **y(깊이)가 비슷한 인접 쌍 중 가장 넓은
        # 간격**을 찾는다. y 조건이 없으면 곡선에서 같은 줄의 앞뒤 콘 사이가
        # 게이트로 오인된다 — 게이트는 좌우 '마주보는' 콘 사이여야 한다.
        gate_valid = False
        gate = self._last_gate
        gate_depth = 0.0
        gate_width = 0.0

        if len(cones) >= 2:
            pts = sorted((cx, cy) for cx, cy, _a in cones)
            best_gap, best_pair = 0.0, None
            for i, (ax, ay) in enumerate(pts):
                for j in range(i + 1, len(pts)):
                    bx, by = pts[j]
                    if abs(ay - by) > cfg.gate_y_tol_px:
                        continue
                    # 게이트는 '빈 틈'이어야 한다 — 사이에 다른 콘이 서 있으면
                    # 그 틈으로는 지나갈 수 없다.
                    if any(ax < mx < bx for mx, _my in pts[i + 1 : j]):
                        continue
                    gap = bx - ax
                    if gap > best_gap:
                        best_gap, best_pair = gap, (ax, bx, (ay + by) / 2.0)
            if best_pair and cfg.min_gate_px <= best_gap <= cfg.max_gate_px:
                inner_l, inner_r, pair_y = best_pair
                gate = (inner_l + inner_r) / 2.0 - center
                self._last_gate = gate
                gate_valid = True
                # ROI 하단(=차에 가장 가까움)에서 게이트까지의 깊이
                gate_depth = float(h - (y0 + pair_y))
                gate_width = float(best_gap)

        # 좌/우 콘 픽셀은 게이트를 기준으로 나눈다 (게이트가 없으면 화면 중앙).
        split = center + (gate if gate_valid else 0.0)
        left_px = sum(a for cx, _cy, a in cones if cx < split)
        right_px = sum(a for cx, _cy, a in cones if cx >= split)

        return ConeReading(
            detected=True,
            orange_px=total_px,
            left_cone_px=int(left_px),
            right_cone_px=int(right_px),
            gate_center_px=float(gate),
            gate_valid=gate_valid,
            gate_depth_px=gate_depth,
            gate_width_px=gate_width,
        )


__all__ = ["ConeConfig", "ConeDetector"]

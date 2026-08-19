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
    # 0818 bag 실측: 실통로 633px 관측 → 620 은 진짜 게이트를 기각했다.
    max_gate_px: float = 780.0
    # 게이트 쌍의 y(깊이) 근접 허용치 [px]. 같은 줄의 앞뒤 콘(=y 차이 큼)이
    # 게이트로 오인되는 것을 막는다. 0818: 곡선에서 마주보는 쌍의 깊이차가
    # 80px 로 관측돼 70 → 110 (같은 줄 인접 콘의 깊이차는 250~400px 라 안전).
    gate_y_tol_px: float = 110.0

    # ── 원거리 콘: 원본 프레임 베이스 투영 (0818 실차 실측으로 추가) ──
    # BEV ROI 는 차 발밑 ~1.1m 만 본다. 실제 통로에선 좌우 쌍이 그 안에 동시에
    # 들어오는 일이 드물어 run5 에서 주황 1만px 를 보고도 게이트가 한 번도 안
    # 섰다 (성립률 0~37%). 콘의 '바닥 접점'은 도로 평면 위 점이라 차선과 같은
    # 호모그래피로 정확히 투영된다 — 원본 프레임에서 콘을 찾아 바닥 중점을
    # BEV 평면 좌표로 쏘면 ROI 밖(더 먼) 콘까지 게이트 후보가 된다.
    far_enabled: bool = True
    far_y_top_frac: float = 0.40     # 원본 프레임 탐색 세로 대역 (수평선 아래)
    far_y_bot_frac: float = 0.95
    far_min_area: int = 120
    # 근접(BEV) 마스크는 팀 튜닝 V≥224 를 유지하지만, 원본 프레임은 조명이
    # 어두운 프레임(실측 평균 116~131)에서 콘 V 가 224 를 못 넘는다 — 완화.
    far_v_min: int = 130
    far_merge_px: float = 70.0       # 근접 검출과 중복 병합 반경 [BEV px]
    # 투영점 채택 한계: 깊이(ROI 위쪽으로) 이만큼까지만 게이트 후보로 쓴다.
    # 더 먼 콘은 다른 구간/보관 무더기일 수 있고 조향에도 불필요하다.
    # 0818 bag 재검증: 480(≈1.1m)은 통로 콘 대부분을 잘라 게이트 23%에
    # 그쳤다 — 900(≈2m)으로. 조준 거리는 policy 의 lookahead 클램프가 자른다.
    far_max_depth_px: float = 900.0
    # 구간 유지 판정에 더할 원본 프레임 주황 픽셀의 상한 기여 — 원거리
    # 픽셀 수는 게이트 좌표와 달리 척도가 달라(원근) 그대로 합치면 과대 계상.
    far_px_cap: int = 4000


class ConeDetector:
    def __init__(self, config: Optional[ConeConfig] = None) -> None:
        self.config = config or ConeConfig()
        self.mask: Optional[np.ndarray] = None
        self._last_gate: float = 0.0

    def detect(
        self,
        bev_roi: Optional[np.ndarray],
        frame: Optional[np.ndarray] = None,
        warp_m: Optional[np.ndarray] = None,
    ) -> ConeReading:
        """입력은 LaneDetector 와 같은 BEV ROI (warped 하단 절반).

        frame + warp_m 을 주면 원본 프레임의 콘 바닥 접점을 같은 호모그래피로
        BEV 평면에 투영해 ROI 밖(더 먼) 콘까지 게이트 후보로 쓴다 (0818).
        """
        if bev_roi is None:
            return ConeReading()

        cfg = self.config
        h, w = bev_roi.shape[:2]
        hsv = cv2.cvtColor(bev_roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(cfg.lower), np.array(cfg.upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        self.mask = mask

        near_px = int(np.count_nonzero(mask))

        # ── 원거리: 원본 프레임 → 콘 바닥 접점 → BEV 평면 투영 ──
        far_pts: List[Tuple[float, float]] = []
        far_px = 0
        if cfg.far_enabled and frame is not None and warp_m is not None:
            fh, fw = frame.shape[:2]
            yf0 = int(fh * cfg.far_y_top_frac)
            yf1 = int(fh * cfg.far_y_bot_frac)
            sub = frame[yf0:yf1]
            hsv_f = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
            mask_f = cv2.inRange(
                hsv_f,
                np.array([cfg.lower[0], cfg.lower[1], cfg.far_v_min]),
                np.array(cfg.upper),
            )
            mask_f = cv2.morphologyEx(
                mask_f, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
            )
            far_px = min(cfg.far_px_cap, int(np.count_nonzero(mask_f)))
            cnts_f, _ = cv2.findContours(
                mask_f, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            bases = []
            for cnt in cnts_f:
                if cv2.contourArea(cnt) < cfg.far_min_area:
                    continue
                bx, by, bw_, bh_ = cv2.boundingRect(cnt)
                bases.append((bx + bw_ / 2.0, by + bh_ + yf0))
            if bases:
                proj = cv2.perspectiveTransform(
                    np.float32(bases).reshape(-1, 1, 2), warp_m
                ).reshape(-1, 2)
                for px_, py_ in proj:
                    roi_y = float(py_) - fh / 2.0     # ROI = warped 하단 절반
                    depth = h - roi_y
                    if depth <= 0 or depth > cfg.far_max_depth_px:
                        continue
                    if not (-0.5 * w <= px_ <= 1.5 * w):
                        continue
                    far_pts.append((float(px_), roi_y))

        total_px = near_px + far_px
        if total_px < cfg.min_section_px:
            return ConeReading(detected=False, orange_px=total_px)

        y0 = int(h * cfg.band_top)
        y1 = int(h * cfg.band_bottom)
        band = mask[y0:y1, :]

        cnts, _ = cv2.findContours(
            band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        center = w / 2.0
        cones: List[Tuple[float, float, int]] = []   # (cx, cy, area) — ROI 좌표
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < cfg.min_area:
                continue
            m = cv2.moments(cnt)
            if m["m00"] == 0:
                continue
            cones.append(
                (m["m10"] / m["m00"], y0 + m["m01"] / m["m00"], int(area))
            )

        # 원거리 투영점 병합 — 근접 검출과 겹치면 근접(면적 증거 있음)을 남긴다
        for fx, fy in far_pts:
            dup = any(
                abs(fx - cx) < cfg.far_merge_px and abs(fy - cy) < cfg.far_merge_px
                for cx, cy, _a in cones
            )
            if not dup:
                cones.append((fx, fy, cfg.far_min_area))

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
            # ★ 쌍 선택 기준 (0818 변경): '가장 넓은 틈'이 아니라 **가장 가까운
            # (y 큰) 유효 쌍**. 원거리 투영으로 여러 깊이의 쌍이 동시에 보이는
            # 지금은 당장 지나야 할 게이트가 조준점이어야 한다 — 두세 게이트
            # 앞을 조준하면 슬라럼에서 통로를 가로지른다. 같은 깊이면 넓은 틈.
            best_key, best_pair, best_gap = None, None, 0.0
            for i, (ax, ay) in enumerate(pts):
                for j in range(i + 1, len(pts)):
                    bx, by = pts[j]
                    if abs(ay - by) > cfg.gate_y_tol_px:
                        continue
                    # 게이트는 '빈 틈'이어야 한다 — 사이에 다른 콘이 서 있으면
                    # 그 틈으로는 지나갈 수 없다. ★ 단 '같은 깊이'의 콘만
                    # 가로막는다 — 1m 더 먼 콘은 이 게이트를 막지 못한다
                    # (0818: 다중 깊이 도입 후 y 무시 검사가 모든 쌍을 죽였다).
                    pair_y0 = (ay + by) / 2.0
                    if any(
                        ax < mx < bx
                        and abs(my - pair_y0) <= cfg.gate_y_tol_px
                        for mx, my in pts[i + 1 : j]
                    ):
                        continue
                    gap = bx - ax
                    if not (cfg.min_gate_px <= gap <= cfg.max_gate_px):
                        continue
                    pair_y = pair_y0
                    key = (pair_y, gap)
                    if best_key is None or key > best_key:
                        best_key = key
                        best_gap, best_pair = gap, (ax, bx, pair_y)
            if best_pair:
                inner_l, inner_r, pair_y = best_pair
                gate = (inner_l + inner_r) / 2.0 - center
                self._last_gate = gate
                gate_valid = True
                # ROI 하단(=차에 가장 가까움)에서 게이트까지의 깊이
                gate_depth = float(h - pair_y)
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

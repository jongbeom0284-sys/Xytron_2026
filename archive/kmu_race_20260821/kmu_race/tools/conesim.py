"""라바콘 구간까지 넣은 폐루프 시뮬레이터.

lanesim 은 차선만 그린다 — 콘이 없어 CONE_DRIVE 경로가 한 번도 안 돈다.
여기서는 트랙의 한 구간에 좌·우 두 줄 콘을 세우고, **카메라 프레임에 원뿔 모양으로**
그려 넣어 실제 ConeDetector 를 원거리 투영 경로까지 그대로 태운다.

콘 렌더링 근사
  · 바닥 접점은 지면 호모그래피로 정확히 투영한다 (원거리 경로가 쓰는 바로 그 점).
  · 원뿔 몸통 높이는 거리에 반비례하는 픽셀 높이로 근사한다 — 실차 bag 에서
    1m 앞 콘이 약 150px 였던 것에 맞췄다 (CONE_PX_AT_1M).
  · 색은 검출기 게이트(H≤28, S≥108, V≥150)를 통과하는 주황으로 칠한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..core.policy import DrivePolicy, PolicyConfig
from ..core.race_context import RaceContext
from ..core.race_fsm import Mode
from ..core.types import ConeReading, LidarReading, Observation, SignalReading
from ..perception.cone_detector import ConeDetector
from ..perception.lane_detector import LaneDetector
from .lanesim import (
    ACT_DELAY_S, CMD_TO_RAD, READY, SENSE_DELAY_S, SERVO_MAX_RAD, SERVO_TAU_S,
    SPEED_TO_MPS, WHEELBASE_M, BevRenderer, SimGeometry, SimResult, Track,
)

CONE_BGR = (25, 95, 235)      # 주황 — HSV ≈ (14, 227, 235), 검출기 게이트 통과
CONE_PX_AT_1M = 150.0         # 1m 앞 콘의 화면 높이 [px] (실차 bag 근사)
CONE_BASE_M = 0.15            # 바닥 지름 [m]


@dataclass
class ConeSection:
    """트랙 위 라바콘 구간 정의."""
    start_m: float = 3.0          # 중심선 시작점에서 몇 m 지점부터
    length_m: float = 4.0         # 구간 길이
    half_gap_m: float = 0.40      # 중심선에서 좌·우 콘까지 거리
    spacing_m: float = 0.60       # 같은 줄 콘 간격
    stagger_m: float = 0.30       # 좌·우 줄의 진행방향 어긋남 (지그재그)

    def place(self, center: np.ndarray) -> List[Tuple[float, float]]:
        """월드 좌표 콘 위치 목록."""
        out: List[Tuple[float, float]] = []
        i0 = int(self.start_m * 100)
        i1 = min(len(center) - 1, int((self.start_m + self.length_m) * 100))
        step = max(1, int(self.spacing_m * 100))
        off = int(self.stagger_m * 100)
        for side, shift in ((+1.0, 0), (-1.0, off)):
            for k in range(i0 + shift, i1, step):
                px, py, pth = center[k]
                out.append((px - side * self.half_gap_m * -math.sin(pth),
                            py - side * self.half_gap_m * math.cos(pth)))
        return out


def _draw_cones(frame: np.ndarray, renderer: BevRenderer, M_inv: np.ndarray,
                cones_car: List[Tuple[float, float]], near_m: float) -> None:
    """차량 기준 (fwd, lat) 콘들을 카메라 프레임에 원뿔로 그린다.

    ★ near_m 보다 가까운 콘은 그리지 않는다 — ROI 맨 아랫줄이 보는 거리가 near_m
      이라 실제 카메라도 그 안쪽은 못 본다. 예전엔 0.12m 까지 그려서 코앞 콘 하나가
      1000px 삼각형으로 화면을 덮었고, 그 덩어리가 좌/우 픽셀 분류를 한쪽으로 쏠리게
      해 단측 반발 조향을 폭주시켰다 — 시뮬만의 인공물이었다.
    """
    h, w = frame.shape[:2]
    for fwd, lat in sorted(cones_car, key=lambda c: -c[0]):     # 먼 것부터
        if fwd <= near_m:
            continue
        u, v = renderer.car_to_warp(fwd, lat)
        p = cv2.perspectiveTransform(np.float32([[[u, v]]]), M_inv).reshape(2)
        bx, by = float(p[0]), float(p[1])
        if not (-w < bx < 2 * w and -h < by < 2 * h):
            continue
        ph = min(1.5 * h, CONE_PX_AT_1M / fwd)                  # 화면 높이 (상한)
        pw = ph * (CONE_BASE_M / 0.30)                          # 밑변 폭
        tri = np.array([[bx, by - ph], [bx - pw / 2, by], [bx + pw / 2, by]], np.int32)
        cv2.fillPoly(frame, [tri], CONE_BGR, cv2.LINE_AA)


def simulate_cones(
    track: Track,
    section: ConeSection,
    *,
    speed_mps: float = 0.6,
    geo: Optional[SimGeometry] = None,
    policy: Optional[DrivePolicy] = None,
    hz: float = 30.0,
    max_seconds: float = 120.0,
) -> Tuple[SimResult, dict]:
    geo = geo or SimGeometry()
    det = LaneDetector()
    cone_det = ConeDetector()
    renderer = BevRenderer(det, geo)
    M = det._warp_matrix(geo.frame_w, geo.frame_h)
    M_inv = np.linalg.inv(M)
    policy = policy or DrivePolicy(PolicyConfig())
    policy.reset()
    ctx = RaceContext()
    ctx.race_started_at = -10.0

    center = track.centerline()
    half = track.lane_width_m / 2.0
    cones_world = section.place(center)

    th = center[0][2]
    x, y = center[0][0], center[0][1]
    speed_scale = speed_mps / (20.0 * SPEED_TO_MPS)
    v = 0.0
    pose_hist = [(x, y, th)]
    cmd_hist = [0.0]
    delta_actual = 0.0
    sense_lag = max(0, int(round(SENSE_DELAY_S * hz)))
    act_lag = max(0, int(round(ACT_DELAY_S * hz)))
    dt = 1.0 / hz

    errs: List[float] = []
    trace: List[Tuple[float, float, float]] = []
    speeds: List[float] = []
    stats = dict(cone_frames=0, gate_frames=0, cone_mode_frames=0,
                 orange_max=0, min_cone_dist=9.9)
    max_cmd = 0.0
    lost = 0
    n = 0
    departed = False
    progress = 0
    in_cone = 0

    for step in range(int(max_seconds * hz)):
        hi = min(len(center), progress + 400)
        d = np.hypot(center[progress:hi, 0] - x, center[progress:hi, 1] - y)
        if len(d) == 0:
            break
        j = int(np.argmin(d)) + progress
        progress = j
        cx, cy, cth = center[j]
        err = -((x - cx) * -math.sin(cth) + (y - cy) * math.cos(cth))
        errs.append(err)
        trace.append((x, y, err))
        if abs(err) > half:
            departed = True
        if j >= len(center) - 5:
            break

        sx, sy, sth = pose_hist[max(0, len(pose_hist) - 1 - sense_lag)]

        def to_car(px: float, py: float) -> Tuple[float, float]:
            dx, dy = px - sx, py - sy
            return (dx * math.cos(sth) + dy * math.sin(sth),
                    -(-dx * math.sin(sth) + dy * math.cos(sth)))

        lines = []
        for side in (-1.0, +1.0):
            pts = []
            for k in range(j, min(len(center), j + 200)):
                px, py, pth = center[k]
                lx = px - side * half * -math.sin(pth)
                ly = py - side * half * math.cos(pth)
                fwd, lat = to_car(lx, ly)
                if 0.05 < fwd < 1.6 and abs(lat) < 1.2:
                    pts.append((fwd, lat))
            if len(pts) >= 2:
                lines.append(np.asarray(pts))

        dashes: List[np.ndarray] = []
        if track.center_line:
            period = max(0.02, track.center_dash_m + track.center_gap_m)
            cur: List[Tuple[float, float]] = []
            for k in range(j, min(len(center), j + 200)):
                px, py, _ = center[k]
                on = (k * 0.01) % period < track.center_dash_m
                fwd, lat = to_car(px, py)
                if on and 0.05 < fwd < 1.6 and abs(lat) < 1.2:
                    cur.append((fwd, lat))
                elif cur:
                    if len(cur) >= 2:
                        dashes.append(np.asarray(cur))
                    cur = []
            if len(cur) >= 2:
                dashes.append(np.asarray(cur))

        frame = renderer.render(lines, dashes)
        cones_car = [to_car(px, py) for px, py in cones_world]
        cones_car = [(f, l) for f, l in cones_car
                     if geo.near_m < f < 3.0 and abs(l) < 2.0]
        if cones_car:
            _draw_cones(frame, renderer, M_inv, cones_car, geo.near_m)
            stats["min_cone_dist"] = min(stats["min_cone_dist"],
                                         min(f for f, _ in cones_car))

        lane = det.detect(frame)
        if not lane.valid:
            lost += 1
        n += 1
        bev = det.warp(frame)
        bev_roi = bev[bev.shape[0] // 2:, :]
        cone = cone_det.detect(bev_roi, frame=frame, warp_m=M)
        stats["orange_max"] = max(stats["orange_max"], int(cone.orange_px))
        if cone.orange_px > 900:
            stats["cone_frames"] += 1
        if cone.gate_valid:
            stats["gate_frames"] += 1

        # FSM 진입/이탈 근사 — 3프레임 연속 게이트로 들어가고, 콘이 끊기면 나온다
        if cone.gate_valid and cone.orange_px > 900:
            in_cone = min(in_cone + 1, 99)
        elif cone.orange_px <= 900:
            in_cone = max(0, in_cone - 1)
        mode = Mode.CONE_DRIVE if in_cone >= 3 else Mode.LANE_DRIVE
        if mode is Mode.CONE_DRIVE:
            stats["cone_mode_frames"] += 1

        obs = Observation(
            now=step * dt, lane=lane, signal=SignalReading(), cone=cone,
            lidar=LidarReading(), yaw_deg=math.degrees(th), ego_speed_mps=v,
            camera_ok=True, lidar_ok=True,
        )
        cmd = policy.update(mode, obs, ctx, READY)
        max_cmd = max(max_cmd, abs(cmd.steer))

        v = cmd.speed * SPEED_TO_MPS * speed_scale
        speeds.append(v)
        cmd_hist.append(cmd.steer)
        applied = cmd_hist[max(0, len(cmd_hist) - 1 - act_lag)]
        delta_target = max(-SERVO_MAX_RAD, min(SERVO_MAX_RAD, applied * CMD_TO_RAD))
        alpha = dt / max(1e-3, SERVO_TAU_S + dt)
        delta_actual += alpha * (delta_target - delta_actual)
        th -= (v / WHEELBASE_M) * math.tan(delta_actual) * dt
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        pose_hist.append((x, y, th))

    res = SimResult(
        track=track.name, speed_mps=speed_mps,
        max_abs_err_m=max((abs(e) for e in errs), default=0.0),
        rms_err_m=float(np.sqrt(np.mean(np.square(errs)))) if errs else 0.0,
        departed=departed, max_steer_cmd=max_cmd,
        finished_frac=progress / max(1, len(center) - 1),
        lane_lost_frac=lost / max(1, n),
        mean_speed_mps=float(np.mean(speeds)) if speeds else 0.0,
        trace=trace,
    )
    stats["cones"] = cones_world
    stats["frames"] = n
    return res, stats


__all__ = ["ConeSection", "simulate_cones", "CONE_BGR"]

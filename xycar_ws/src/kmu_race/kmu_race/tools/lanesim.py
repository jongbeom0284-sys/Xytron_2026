"""차선주행 폐루프 시뮬레이터 — 실차 없이 코너링을 객관 측정한다.

왜 필요한가 (0817):
  첫 자율주행에서 코너를 못 돌았는데, 원인이 인지인지 제어인지 게인인지
  현장에서 한 판씩 돌려 보며 좁히는 건 배터리·시간·트랙 점유 모두 비싸다.
  차선은 기하가 단순하므로(평면 위 두 줄), BEV 를 직접 합성해 **진짜 검출기와
  진짜 정책**에 물리고 자전거 모델로 되먹이면 코너링을 정량 평가할 수 있다.

체인 (실차와 동일 순서):
  트랙 기하 → BEV 합성 → (역워프) 카메라 영상 → LaneDetector → DrivePolicy
  → 조향 cmd → [motor_adapter 모델: δ = cmd×0.0068, 서보 포화 ±16.5°]
  → 자전거 모델(축거 0.30m) → 새 자세 → 반복

BEV 축척은 실차 미실측이라 **스윕 대상**이다 (px_per_m 420~600). 특정 값에서만
되는 게인은 실차에서 깨지므로, 전 구간에서 이탈 0 이어야 채택한다.

    python -m kmu_race.tools.lanesim              # 기본 시나리오 전부
    python -m kmu_race.tools.lanesim --sweep      # 축척·속도 스윕
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..core.control import WHEELBASE_M
from ..core.policy import DrivePolicy, PolicyConfig
from ..core.race_context import RaceContext
from ..core.race_fsm import Mode
from ..core.safety import SafetyDecision, SafetyLevel
from ..core.types import ConeReading, LidarReading, Observation, SignalReading
from ..perception.lane_detector import LaneDetector

# motor_adapter 실측 체인 (팀원 kobot_vehicle)
CMD_TO_RAD = 0.0068            # δ[rad] = cmd × 0.0068
SERVO_MAX_RAD = math.radians(16.5)   # vesc.yaml servo_min/max 0.15~0.85 포화
READY = SafetyDecision(SafetyLevel.OK, "", True)


# ──────────────────────────────────────────────────────────────────
# 트랙 기하 — 중심선을 (호/직선) 구간의 연속으로 정의한다
# ──────────────────────────────────────────────────────────────────
@dataclass
class Segment:
    kind: str          # "straight" | "arc"
    length: float      # 직선 길이 [m] 또는 호 길이 [m]
    radius: float = 0.0    # 호 반경 [m], + 좌회전 / - 우회전


@dataclass
class Track:
    name: str
    segments: List[Segment]
    lane_width_m: float = 0.80     # 주행로 폭 (규정 도면 기준 실측값)

    def centerline(self, step: float = 0.01) -> np.ndarray:
        """중심선 점열 [(x, y, heading)] — 원점에서 +x 방향으로 시작."""
        pts = []
        x, y, th = 0.0, 0.0, 0.0
        for seg in self.segments:
            n = max(1, int(seg.length / step))
            for _ in range(n):
                if seg.kind == "arc" and abs(seg.radius) > 1e-6:
                    dth = step / seg.radius
                    th += dth
                x += step * math.cos(th)
                y += step * math.sin(th)
                pts.append((x, y, th))
        return np.asarray(pts, dtype=np.float64)


def make_tracks() -> List[Track]:
    """본선 트랙에서 실제로 만나는 형상들 (규정 도면 p.6/p.7 기반)."""
    return [
        Track("직선", [Segment("straight", 6.0)]),
        # 둥근 사각 루프의 코너 — 실측 반경 대역
        Track("완만한 코너 R=1.5m", [
            Segment("straight", 1.0), Segment("arc", 2.4, 1.5), Segment("straight", 1.0)]),
        Track("급코너 R=1.0m", [
            Segment("straight", 1.0), Segment("arc", 1.6, 1.0), Segment("straight", 1.0)]),
        Track("최악 코너 R=0.8m", [
            Segment("straight", 1.0), Segment("arc", 1.3, 0.8), Segment("straight", 1.0)]),
        # S자 (우변 구간 + 라바콘 슬라럼 형상)
        Track("S자 R=1.2m", [
            Segment("straight", 0.8), Segment("arc", 1.9, 1.2),
            Segment("arc", 1.9, -1.2), Segment("straight", 0.8)]),
        Track("연속 S자 (슬라럼)", [
            Segment("straight", 0.5),
            Segment("arc", 1.3, 1.0), Segment("arc", 1.3, -1.0),
            Segment("arc", 1.3, 1.0), Segment("arc", 1.3, -1.0),
            Segment("straight", 0.5)]),
    ]


# ──────────────────────────────────────────────────────────────────
# BEV 합성 → 카메라 영상 (검출기 호모그래피의 역변환)
# ──────────────────────────────────────────────────────────────────
@dataclass
class SimGeometry:
    """BEV 축척 — 실차 미실측이라 스윕한다."""
    px_per_m_lat: float = 560.0   # 가로: 차선폭 450px ÷ 0.8m ≈ 562
    px_per_m_fwd: float = 510.0   # 세로: ROI 240px ÷ 0.47m ≈ 510
    near_m: float = 0.25          # ROI 맨 아랫줄이 보는 전방 거리
    frame_w: int = 640
    frame_h: int = 480


class BevRenderer:
    def __init__(self, det: LaneDetector, geo: SimGeometry) -> None:
        self.geo = geo
        M = det._warp_matrix(geo.frame_w, geo.frame_h)
        self.M_inv = np.linalg.inv(M)

    def car_to_warp(self, fwd: float, lat: float) -> Tuple[float, float]:
        """차량 기준 (전방 m, 좌우 m; 오른쪽 +) → 워프영상 픽셀."""
        g = self.geo
        v_roi = (g.frame_h // 2 - 1) - (fwd - g.near_m) * g.px_per_m_fwd
        v_warp = v_roi + g.frame_h // 2
        u = g.frame_w / 2.0 + lat * g.px_per_m_lat
        return u, v_warp

    def render(self, lines_car: List[np.ndarray]) -> np.ndarray:
        """차량 기준 좌표의 차선 점열들 → 카메라 영상 (BGR).

        ⚠️ 역워프하면 원본 사다리꼴 밖이 검게 남는데, 그 경계가 Canny 에
        강한 엣지로 잡혀 '가짜 차선'이 된다 (실차 영상엔 없는 인공물이라
        시뮬 결론을 통째로 오염시킨다 — 0817 S자 8m 발산의 정체).
        그래서 **선만** 워프해서 균일한 바닥 위에 합성한다.
        """
        g = self.geo
        lines_bev = np.zeros((g.frame_h, g.frame_w), np.uint8)
        for pts in lines_car:
            uv = [self.car_to_warp(f, l) for f, l in pts]
            uv = [(int(round(u)), int(round(v))) for u, v in uv
                  if -3000 < u < 3000 and -3000 < v < 3000]
            if len(uv) >= 2:
                cv2.polylines(lines_bev, [np.asarray(uv, np.int32)], False,
                              255, 8, cv2.LINE_AA)
        mask = cv2.warpPerspective(
            lines_bev, self.M_inv, (g.frame_w, g.frame_h), flags=cv2.INTER_LINEAR
        )
        frame = np.full((g.frame_h, g.frame_w, 3), 168, np.uint8)   # 균일한 바닥
        frame[mask > 60] = (250, 250, 250)
        return frame


# ──────────────────────────────────────────────────────────────────
# 시뮬레이션 본체
# ──────────────────────────────────────────────────────────────────
@dataclass
class SimResult:
    track: str
    speed_mps: float
    max_abs_err_m: float
    rms_err_m: float
    departed: bool
    max_steer_cmd: float
    finished_frac: float
    lane_lost_frac: float
    mean_speed_mps: float = 0.0
    trace: List[Tuple[float, float, float]] = field(default_factory=list)


SPEED_TO_MPS = 0.08     # params.yaml — 명령속도 → m/s (실차 체인 설계값)

# ── 지연 모델 (★ 반드시 켜 둘 것) ──────────────────────────────────
# 타팀 UMK 는 지연 없는 시뮬로 게인을 격자탐색해 실차에 올렸다가 "술 취한 듯"
# 좌우 진동으로 되돌렸다 (그들 기록, 8/17). 지연 없는 시뮬은 실제보다 훨씬
# 높은 게인을 통과시킨다 — 우리도 같은 함정에 빠지지 않도록 체인의 지연을
# 명시적으로 모델링한다.
SENSE_DELAY_S = 0.05     # 노출+USB+디코드 (30fps 카메라 ≈ 1.5프레임)
ACT_DELAY_S = 0.04       # ROS2 pub/sub + motor_adapter + VESC 왕복
SERVO_TAU_S = 0.06       # 조향 서보 1차 지연 (RC 서보 60°/0.1s 급)


def simulate(
    track: Track,
    *,
    speed_mps: float = 0.6,     # speed_scale 로 환산되는 '순항 목표'
    use_policy_speed: bool = True,
    geo: Optional[SimGeometry] = None,
    policy: Optional[DrivePolicy] = None,
    hz: float = 30.0,
    start_lateral: float = 0.0,
    max_seconds: float = 60.0,
) -> SimResult:
    geo = geo or SimGeometry()
    det = LaneDetector()
    renderer = BevRenderer(det, geo)
    policy = policy or DrivePolicy(PolicyConfig())
    policy.reset()
    ctx = RaceContext()
    ctx.race_started_at = -10.0     # 출발 직진 홀드는 지나간 것으로

    center = track.centerline()
    half = track.lane_width_m / 2.0

    # 차 초기 자세 = 중심선 시작점에서 옆으로 start_lateral
    th0 = center[0][2]
    x = center[0][0] - start_lateral * math.sin(th0)
    y = center[0][1] + start_lateral * math.cos(th0)
    th = th0

    # 실차와 동일하게 **정책이 속도를 정한다**. speed_mps 는 목표 순항속도로
    # 해석해 speed_scale 로 환산한다 (LANE_DRIVE 기본 20 cmd = 1.6 m/s).
    speed_scale = speed_mps / (20.0 * SPEED_TO_MPS)
    v = 0.0
    speeds: List[float] = []
    # 지연 버퍼: 인지는 과거 자세를 보고, 명령은 나중에 바퀴에 닿는다
    pose_hist: List[Tuple[float, float, float]] = [(x, y, th)]
    cmd_hist: List[float] = [0.0]
    delta_actual = 0.0
    sense_lag = max(0, int(round(SENSE_DELAY_S * hz)))
    act_lag = max(0, int(round(ACT_DELAY_S * hz)))

    dt = 1.0 / hz
    errs: List[float] = []
    trace: List[Tuple[float, float, float]] = []
    max_cmd = 0.0
    lost = 0
    n = 0
    departed = False
    progress = 0

    for step in range(int(max_seconds * hz)):
        # ── 가장 가까운 중심선 점 (진행 인덱스는 단조 증가) ──
        seek_hi = min(len(center), progress + 400)
        d = np.hypot(center[progress:seek_hi, 0] - x, center[progress:seek_hi, 1] - y)
        if len(d) == 0:
            break
        j = int(np.argmin(d)) + progress
        progress = j
        cx, cy, cth = center[j]
        # 부호 있는 횡오차 (오른쪽 +)
        err = -((x - cx) * -math.sin(cth) + (y - cy) * math.cos(cth))
        errs.append(err)
        trace.append((x, y, err))
        if abs(err) > half:
            departed = True
        if j >= len(center) - 5:
            break

        # ── 차량 기준 차선 점열 만들기 — **지연된 과거 자세**로 본다 ──
        sx, sy, sth = pose_hist[max(0, len(pose_hist) - 1 - sense_lag)]
        lines = []
        for side in (-1.0, +1.0):
            pts = []
            for k in range(j, min(len(center), j + 200)):
                px, py, pth = center[k]
                lx = px - side * half * -math.sin(pth)
                ly = py - side * half * math.cos(pth)
                dx, dy = lx - sx, ly - sy
                fwd = dx * math.cos(sth) + dy * math.sin(sth)
                lat = -(-dx * math.sin(sth) + dy * math.cos(sth))
                if 0.05 < fwd < 1.6 and abs(lat) < 1.2:
                    pts.append((fwd, lat))
            if len(pts) >= 2:
                lines.append(np.asarray(pts))

        frame = renderer.render(lines)
        lane = det.detect(frame)
        if not lane.valid:
            lost += 1
        n += 1

        obs = Observation(
            now=step * dt,
            lane=lane,
            signal=SignalReading(),
            cone=ConeReading(),
            lidar=LidarReading(),
            yaw_deg=math.degrees(th),
            ego_speed_mps=v,
            camera_ok=True,
            lidar_ok=True,
        )
        cmd = policy.update(Mode.LANE_DRIVE, obs, ctx, READY)
        max_cmd = max(max_cmd, abs(cmd.steer))

        # ── 실차 모터 체인 + 자전거 모델 ──
        v = (
            cmd.speed * SPEED_TO_MPS * speed_scale
            if use_policy_speed
            else speed_mps
        )
        speeds.append(v)
        cmd_hist.append(cmd.steer)
        applied = cmd_hist[max(0, len(cmd_hist) - 1 - act_lag)]
        delta_target = max(
            -SERVO_MAX_RAD, min(SERVO_MAX_RAD, applied * CMD_TO_RAD)
        )
        # 서보 1차 지연 — 명령이 즉시 바퀴각이 되지 않는다
        alpha = dt / max(1e-3, SERVO_TAU_S + dt)
        delta_actual += alpha * (delta_target - delta_actual)
        th -= (v / WHEELBASE_M) * math.tan(delta_actual) * dt   # +cmd = 우회전
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        pose_hist.append((x, y, th))

    arr = np.abs(np.asarray(errs)) if errs else np.asarray([0.0])
    return SimResult(
        track=track.name,
        speed_mps=speed_mps,
        mean_speed_mps=float(np.mean(speeds)) if speeds else 0.0,
        max_abs_err_m=float(arr.max()),
        rms_err_m=float(np.sqrt((arr ** 2).mean())),
        departed=departed,
        max_steer_cmd=max_cmd,
        finished_frac=progress / max(1, len(center) - 1),
        lane_lost_frac=lost / max(1, n),
        trace=trace,
    )


def run_all(speeds=(0.6, 1.0), geo: Optional[SimGeometry] = None) -> List[SimResult]:
    out = []
    for tr in make_tracks():
        for v in speeds:
            out.append(simulate(tr, speed_mps=v, geo=geo))
    return out


def print_table(results: List[SimResult], title: str = "") -> bool:
    if title:
        print(f"\n=== {title} ===")
    print(f"{'트랙':<22}{'순항목표':>8}{'실주행':>8}{'최대오차':>10}{'RMS':>8}"
          f"{'최대조향':>9}{'완주':>7}  판정")
    ok_all = True
    for r in results:
        ok = (not r.departed) and r.finished_frac > 0.95
        ok_all &= ok
        print(f"{r.track:<22}{r.speed_mps:>7.1f}{r.mean_speed_mps:>8.2f}"
              f"{r.max_abs_err_m:>9.3f}m{r.rms_err_m:>7.3f}m{r.max_steer_cmd:>9.1f}"
              f"{r.finished_frac*100:>6.0f}%"
              f"  {'OK' if ok else '이탈' if r.departed else '미완주'}")
    return ok_all


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="차선주행 폐루프 시뮬레이터")
    p.add_argument("--sweep", action="store_true", help="BEV 축척·속도 스윕")
    p.add_argument("--speeds", default="0.6,1.0")
    args = p.parse_args(argv)

    speeds = tuple(float(s) for s in args.speeds.split(","))
    if not args.sweep:
        ok = print_table(run_all(speeds), "기본 축척 (lat 560 / fwd 510 px/m)")
        print("\n종합:", "✅ 전 시나리오 통과" if ok else "❌ 실패 시나리오 있음")
        return

    all_ok = True
    for lat, fwd in ((450.0, 420.0), (560.0, 510.0), (620.0, 600.0)):
        geo = SimGeometry(px_per_m_lat=lat, px_per_m_fwd=fwd)
        ok = print_table(run_all(speeds, geo), f"축척 lat={lat:.0f} fwd={fwd:.0f} px/m")
        all_ok &= ok
    print("\n스윕 종합:", "✅ 전 축척에서 통과" if all_ok else "❌ 축척 의존 실패 있음")


if __name__ == "__main__":
    main()

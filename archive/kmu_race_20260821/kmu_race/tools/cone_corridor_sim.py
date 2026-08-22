"""라이다 구동 라바콘 통로 폐루프 시뮬 — 실제 find_cone_gate + _cone_steer 를 태운다.

conesim 은 카메라 콘 경로만 돌린다. 실주행은 라이다 게이트가 1순위이므로
(perception/lidar_cones.py) 그쪽을 태워야 의미가 있다.

플랜트는 lanesim 과 같은 체인:
  발행 cmd → 구동지연 0.04s → δ_target = cmd × 0.0068 (서보 ±0.288rad 클램프)
  → 1차 지연 τ=0.06s → 자전거 모델 (축거 0.30m)
관측은 센싱지연 0.05s, 라이다 8.8Hz, 제어 50Hz.
"""
import sys, math, argparse

def build(pkg):
    m = __import__(pkg + ".perception.lidar_cones", fromlist=["x"])
    P = __import__(pkg + ".core.policy", fromlist=["x"])
    F = __import__(pkg + ".core.race_fsm", fromlist=["x"])
    X = __import__(pkg + ".core.race_context", fromlist=["x"])
    S = __import__(pkg + ".core.safety", fromlist=["x"])
    T = __import__(pkg + ".core.types", fromlist=["x"])
    L = __import__(pkg + ".perception.lidar_tracker", fromlist=["x"])
    return dict(LC=m, P=P, F=F, X=X, S=S, T=T, OFF=L.LidarConfig().angle_offset)

N_BEAM, A_MIN, A_INC = 500, -math.pi, 0.01259155385196209
WB, C2R, SERVO = 0.30, 0.0068, 0.288
SENSE, ACT, TAU = 0.05, 0.04, 0.06
CONE_R = 0.075                      # 콘 밑면 반지름 [m]

def centerline(kind, R, length=None, ds=0.005):
    """(x, y, th) 중심선. kind: 'straight' | 'arc' | 's'

    ★ 길이를 반드시 제한한다. 반경 1~2m 짜리 호를 길게 감으면 나선이 겹쳐
      라이다에 **다른 바퀴의 콘**이 들어와 게이트가 엉킨다 (harness 인공물).
      한 바퀴의 90% 를 넘지 않게 자른다.
    """
    if length is None:
        length = 5.0 if kind == "straight" else min(5.0, 0.9 * 2 * math.pi * R)
    n = int(length / ds)
    pts, x, y, th = [], 0.0, 0.0, 0.0
    for i in range(n):
        pts.append((x, y, th))
        if kind == "arc":       k = 1.0 / R
        elif kind == "s":       k = (1.0 / R) * (1 if (i // 400) % 2 == 0 else -1)
        else:                   k = 0.0
        th += k * ds; x += math.cos(th) * ds; y += math.sin(th) * ds
    return pts

def place_cones(center, half, spacing, stagger):
    out, step = [], max(1, int(spacing / 0.005))
    off = int(stagger / 0.005)
    for side, sh in ((+1.0, 0), (-1.0, off)):
        for k in range(sh, len(center), step):
            x, y, th = center[k]
            out.append((x - side * half * math.sin(th), y + side * half * math.cos(th)))
    return out

def synth_scan(cones, cx, cy, cth, off):
    """차 기준으로 콘들을 라이다 ranges 로 만든다 (원기둥 옆면)."""
    rng = [0.0] * N_BEAM
    for (wx, wy) in cones:
        dx, dy = wx - cx, wy - cy
        bx = dx * math.cos(cth) + dy * math.sin(cth)
        by = -dx * math.sin(cth) + dy * math.cos(cth)
        d = math.hypot(bx, by)
        if d < 0.05 or d > 2.6:
            continue
        a = math.atan2(by, bx)
        half = math.asin(min(0.99, CONE_R / max(CONE_R, d)))
        i0 = int(round((a - off - A_MIN) / A_INC))
        k = max(1, int(half / A_INC))
        for j in range(i0 - k, i0 + k + 1):
            ang = A_MIN + (j % N_BEAM) * A_INC + off
            da = ang - a
            if abs(da) > half:
                continue
            # 원기둥 표면까지의 거리
            r = d * math.cos(da) - math.sqrt(max(0.0, CONE_R**2 - (d*math.sin(da))**2))
            idx = j % N_BEAM
            if rng[idx] <= 0.0 or r < rng[idx]:
                rng[idx] = r
    return rng

def run(pkg, kind, R, *, speed_cmd=None, half=0.41, spacing=0.40, stagger=0.20,
        e0=0.10, drop=None, **over):
    M = build(pkg)
    pcfg = M["P"].PolicyConfig()
    for k, v in over.items():
        obj = pcfg
        parts = k.split("__")
        for part in parts[:-1]:
            obj = getattr(obj, part)
        assert hasattr(obj, parts[-1]), f"없는 설정: {k}"
        setattr(obj, parts[-1], v)
    pol = M["P"].DrivePolicy(pcfg)
    ctx = M["X"].RaceContext()
    ready = M["S"].SafetyDecision(M["S"].SafetyLevel.OK, "", True)
    Mode, T = M["F"].Mode, M["T"]
    center = centerline(kind, R)
    path_len = len(center) * 0.005
    cones = place_cones(center, half, spacing, stagger)
    # 초기 자세: 중심선 위, 횡오차 e0
    x0, y0, th0 = center[0]
    cx = x0 - e0 * math.sin(th0); cy = y0 + e0 * math.cos(th0); cth = th0
    dt, t = 0.02, 0.0
    delta = 0.0; gate = None; obs_q = []; cmd_hist = []
    worst = 0.0; worst_cone = 9.9; sat = 0; n = 0; hits = 0; v_sum = 0.0
    gate_ok = 0; travel = 0.0; last_i = 0
    next_scan = 0.0
    prev_side = 0
    dur = 1.6 * path_len / 1.1
    while t < dur:
        if t >= next_scan:
            next_scan += 1.0 / 8.8
            rng = synth_scan(cones, cx, cy, cth, M["OFF"])
            try:
                g = M["LC"].find_cone_gate(rng, A_MIN, A_INC, angle_offset=M["OFF"],
                                           prev_row_side=prev_side)
                prev_side = g.near_row_side
            except Exception:
                g = None
            # 게이트 끊김 재현 — 실측(cone_0821_0037) 성립 0.91s / 끊김 0.34s
            if drop:
                on, off = drop
                if (t % (on + off)) >= on:
                    g = None
            obs_q.append((t, g))
        while len(obs_q) > 1 and obs_q[1][0] <= t - SENSE:
            obs_q.pop(0)
        gate = obs_q[0][1] if obs_q else None
        cone = T.ConeReading(
            detected=bool(gate and gate.valid), orange_px=0,
            gate_valid=False,
            lidar_gate_valid=bool(gate and gate.valid),
            lidar_gate_width_m=gate.width_m if gate else 0.0,
            lidar_gate_depth_m=gate.depth_m if gate else 0.0,
            lidar_gate_lateral_m=gate.lateral_m if gate else 0.0,
            lidar_cones=gate.cones if gate else 0,
            lidar_near_left_m=gate.near_left_m if gate else float("inf"),
            lidar_near_right_m=gate.near_right_m if gate else float("inf"),
            # ★ 0821 밤: 이걸 안 실어서 **마지막 관문(_cone_guard)이 시뮬에서
            #   한 번도 안 돌고 있었다**. race_node 는 lg.points 를 항상 싣는다
            #   — 배선이 다르면 시뮬이 실차보다 쉬운 문제를 푼다.
            lidar_cone_pts=getattr(gate, "points", ()) if gate else ()) \
            if hasattr(T.ConeReading, "__dataclass_fields__") else None
        o = T.Observation(now=t, lane=T.LaneReading() if hasattr(T, "LaneReading") else None,
                          cone=cone, lidar=T.LidarReading(stamp=t),
                          camera_ok=True, lidar_ok=True,
                          ego_speed_mps=abs(0.08 * (speed_cmd or 14)))
        c = pol.update(Mode.CONE_DRIVE, o, ctx, ready)
        cmd = c.steer
        v = 0.08 * (speed_cmd if speed_cmd is not None else c.speed)
        cmd_hist.append(cmd)
        lag = int(round(ACT / dt))
        applied = cmd_hist[max(0, len(cmd_hist) - 1 - lag)]
        dtarget = max(-SERVO, min(SERVO, applied * C2R))
        delta += (dtarget - delta) * (1.0 - math.exp(-dt / TAU))
        cth -= (v / WB) * math.tan(delta) * dt   # lanesim 과 같은 부호 규약
        cx += v * math.cos(cth) * dt; cy += v * math.sin(cth) * dt
        # 중심선까지 횡거리
        # 진행방향으로만 전진하며 최근접점을 찾는다 (되감김 방지)
        best, bd = last_i, 1e9
        for i in range(last_i, min(len(center), last_i + 400)):
            d2 = (center[i][0]-cx)**2 + (center[i][1]-cy)**2
            if d2 < bd: bd, best = d2, i
        last_i = best
        px, py, pth = center[best]
        e = -(cx - px) * math.sin(pth) + (cy - py) * math.cos(pth)
        worst = max(worst, abs(e))
        # 차체는 원이 아니라 사각형이다 (앞 0.30m / 뒤 0.20m / 반폭 0.15m).
        # 원으로 재면 '앞 20cm 콘'을 충돌로 오판한다.
        dmin = 9.9
        for wx, wy in cones:
            dx, dy = wx - cx, wy - cy
            bx = dx * math.cos(cth) + dy * math.sin(cth)
            by = -dx * math.sin(cth) + dy * math.cos(cth)
            ox = max(-0.20 - bx, bx - 0.30, 0.0)
            oy = max(abs(by) - 0.15, 0.0)
            clear = math.hypot(ox, oy) - CONE_R
            dmin = min(dmin, clear)
        worst_cone = min(worst_cone, dmin)
        if dmin <= 0.0:
            hits += 1
        if abs(cmd) >= 41.5: sat += 1
        if gate and gate.valid: gate_ok += 1
        n += 1; v_sum += v; t += dt; travel += v * dt
        if best >= len(center) - 60:      # 통로 끝
            break
    return dict(worst_e=worst, min_cone=worst_cone, sat=sat/max(1,n),
                hit=hits > 0, v=v_sum/max(1,n), gate=gate_ok/max(1,n),
                travel=travel, path=path_len)

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--path", default="/tmp/ab6")
    a = ap.parse_args()
    sys.path.insert(0, a.path)
    print(f"{'통로':>10} {'길이':>6} | {'현행(deg)':>30} | {'수정(cmd)':>30}")
    print(f"{'':>10} {'':>6} | {'이탈   최근접  포화  주행':>30} | {'이탈   최근접  포화  주행':>30}")
    print("-"*80)
    cases = [("직선", "straight", 99.0), ("R 3.0m","arc",3.0), ("R 2.0m","arc",2.0),
             ("R 1.5m","arc",1.5), ("R 1.2m","arc",1.2), ("R 1.0m","arc",1.0),
             ("S자 R1.5","s",1.5)]
    for nm, kind, R in cases:
        row = []; plen = 0.0
        for pkg in ("kmu_race_deg", "kmu_race_cmd"):
            r = run(pkg, kind, R); plen = r["path"]
            flag = "✗" if (r["hit"] or r["worst_e"] > 0.259) else "✓"
            row.append(f"{flag}{r['worst_e']*100:6.1f}cm {r['min_cone']*100:5.1f}cm"
                       f" {r['sat']*100:4.0f}% {r['travel']:4.1f}m")
        print(f"{nm:>10} {plen:5.1f}m | {row[0]:>30} | {row[1]:>30}")

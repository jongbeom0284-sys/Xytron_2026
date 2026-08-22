"""bag 리플레이 — 실차 시간을 노트북 시간으로 바꾸는 도구.

실차 접근이 희소 자원이라(연습주행 07.01~08.21) 트랙에서 튜닝하면 하루에 몇 번
못 돌린다. 수동주행 bag 을 한 번 받아 두면 같은 입력을 초 단위로 무한히 다시
돌릴 수 있다.

    # 차에서 한 번 (수동주행으로 3바퀴)
    # raw 는 ~26MB/s 라 3바퀴에 4GB+. compressed 로 녹화한다 (replay 가 둘 다 읽음).
    ros2 bag record /image_raw/compressed /scan /imu -o run1

    # 이후 노트북에서
    ros2 run kmu_race replay run1                    # 퍼셉션 값 요약
    ros2 run kmu_race replay run1 --fsm              # FSM 까지 통째로 재생
    ros2 run kmu_race replay run1 --sweep-gantry     # ★ 랩 임계값 자동 탐색
    ros2 run kmu_race replay run1 --csv out.csv      # 프레임별 값 덤프

--sweep-gantry 가 이 도구의 핵심이다. 랩 카운팅은 이 대회 최대 리스크다 —
하나 놓치면 3바퀴를 못 채우고 하나 더 세면 2바퀴에 멈춘다. 둘 다 실격이다.
3바퀴 bag 을 넣으면 min_gantry_span_px 를 훑어 가며 **정확히 4회 목격**이
나오는 구간을 찾아 준다. 트랙에서 시행착오로 맞출 값이 아니다.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
from dataclasses import dataclass
from typing import Dict, Iterator, List, Optional, Tuple


# ──────────────────────────────────────────────────────────────────
# bag 읽기
# ──────────────────────────────────────────────────────────────────
def _open_reader(path: str):
    import rosbag2_py

    last_err = None
    for storage_id in ("sqlite3", "mcap", ""):
        try:
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions("", ""),
            )
            return reader
        except Exception as exc:                       # noqa: BLE001
            last_err = exc
    raise RuntimeError(f"bag 을 열 수 없다: {path}\n{last_err}")


def bag_topics(path: str) -> Dict[str, str]:
    """bag 의 {토픽: 타입} 맵. 토픽 자동 보정과 사전 경고에 쓴다."""
    reader = _open_reader(path)
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def read_bag(path: str) -> Iterator[Tuple[str, object, float]]:
    """(topic, msg, t_seconds) 를 시간 순으로 뱉는다."""
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = _open_reader(path)
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in type_map:
            continue
        try:
            msg = deserialize_message(data, get_message(type_map[topic]))
        except Exception:                              # noqa: BLE001
            continue
        yield topic, msg, stamp_ns * 1e-9


def _decode_image(bridge, msg):
    """Image 든 CompressedImage 든 BGR ndarray 로 만든다.

    실차 카메라(usb_cam + image_transport)는 raw `/image_raw` 와
    `/image_raw/compressed` 를 같이 낸다. raw 는 640x480 bgr8 기준 ~26MB/s 라
    3바퀴면 4GB 를 넘어가므로, 녹화는 compressed 로 하는 것이 현실적이다
    (8/10 run4 bag 이 실제로 그렇게 녹화됐다). 여기서 둘 다 받아 준다.
    """
    if hasattr(msg, "encoding"):                       # sensor_msgs/Image
        return bridge.imgmsg_to_cv2(msg, "bgr8")
    return bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # CompressedImage


# ──────────────────────────────────────────────────────────────────
# 프레임별 퍼셉션 결과
# ──────────────────────────────────────────────────────────────────
@dataclass
class Sample:
    t: float
    gantry_span: int = 0
    signal: str = "UNKNOWN"
    reject: str = ""
    lane_offset: float = 0.0
    lane_valid: bool = False
    cone_px: int = 0
    gate_center: float = 0.0
    gate_valid: bool = False
    front_dist: float = float("inf")
    yaw: float = 0.0


def run_perception(
    path: str, *, image_topic: str, scan_topic: str, imu_topic: str, stride: int
) -> List[Sample]:
    """bag 전체를 퍼셉션에 통과시킨다. 이미지 프레임 하나당 Sample 하나."""
    from cv_bridge import CvBridge

    from ..core.heading import GyroHeading
    from ..perception.cone_detector import ConeDetector
    from ..perception.lane_detector import LaneDetector
    from ..perception.lidar_tracker import LidarTracker
    from ..perception.signal_detector import SignalDetector

    bridge = CvBridge()
    sig_det, lane_det = SignalDetector(), LaneDetector()
    cone_det, lidar_trk = ConeDetector(), LidarTracker()

    # ── 토픽 사전 점검: raw 가 없으면 compressed 로 자동 보정 ──
    topics = bag_topics(path)
    if image_topic not in topics:
        alt = image_topic.rstrip("/") + "/compressed"
        if alt in topics:
            print(f"ℹ️  {image_topic} 없음 → {alt} (CompressedImage) 사용")
            image_topic = alt
        else:
            print(f"⚠️  bag 에 이미지 토픽이 없다: {image_topic} — "
                  f"보유 토픽: {', '.join(sorted(topics))}")
    if imu_topic not in topics:
        print(f"⚠️  bag 에 {imu_topic} 이 없다 — yaw 가 0 으로 고정된다 "
              f"(지름길 완료·요 교차검증 튜닝 불가). 다음 녹화엔 /imu 포함할 것")

    samples: List[Sample] = []
    front = float("inf")
    # 헤딩 = 자이로 z 적분 (실차 race_node 와 동일 경로 — core/heading.py).
    # 녹화 규약상 bag 첫 수 초는 차 정지 상태라 바이어스 캘리브레이션이 성립한다.
    heading = GyroHeading()
    yaw_now = 0.0
    n_img = 0
    n_imu = 0

    for topic, msg, t in read_bag(path):
        if topic == scan_topic:
            r = lidar_trk.update(
                msg.ranges,
                angle_min=msg.angle_min,
                angle_increment=msg.angle_increment,
                now=t,
            )
            front = r.front_dist
        elif topic == imu_topic:
            n_imu += 1
            g = msg.angular_velocity
            yaw_now = heading.update(g.x, g.y, g.z, t)
        elif topic == image_topic:
            n_img += 1
            if stride > 1 and (n_img % stride) != 0:
                continue
            try:
                frame = _decode_image(bridge, msg)
            except Exception:                          # noqa: BLE001
                continue
            sig = sig_det.detect(frame)
            lane = lane_det.detect(frame)
            cone = cone_det.detect(lane_det.last_bev_roi)
            samples.append(
                Sample(
                    t=t,
                    gantry_span=sig.gantry_span_px,
                    signal=sig.state.value,
                    reject=sig.reject_reason,
                    lane_offset=lane.offset_px,
                    lane_valid=lane.valid,
                    cone_px=cone.orange_px,
                    gate_center=cone.gate_center_px,
                    gate_valid=cone.gate_valid,
                    front_dist=front,
                    yaw=yaw_now,
                )
            )

    # 토픽은 등록돼 있는데 메시지가 0건인 경우 (녹화 시작 시 IMU 죽어 있던 bag)
    if n_imu == 0 and imu_topic in topics:
        print(f"⚠️  {imu_topic} 토픽은 있는데 메시지가 0건 — yaw 가 0 으로 "
              f"고정된다. 녹화 당시 IMU 가 죽어 있었다는 뜻이다")
    return samples


# ──────────────────────────────────────────────────────────────────
# 랩 임계값 스윕 — 이 도구의 핵심
# ──────────────────────────────────────────────────────────────────
def count_sightings(samples: List[Sample], threshold: int) -> int:
    """주어진 임계값으로 이 bag 을 재생했을 때 목격이 몇 번 잡히는가."""
    from ..core.race_context import LapTracker, LapTrackerConfig

    trk = LapTracker(LapTrackerConfig(min_gantry_span_px=threshold))
    for s in samples:
        trk.update(gantry_span_px=s.gantry_span, now=s.t, yaw_deg=s.yaw)
    return trk.sightings


def find_good_thresholds(
    samples: List[Sample], expect: int
) -> Tuple[List[int], int, int]:
    """목격 횟수가 정확히 expect 인 임계값들. 반환 (good, lo, hi)."""
    spans = [s.gantry_span for s in samples if s.gantry_span > 0]
    if not spans:
        return [], 0, 0
    lo, hi = min(spans), max(spans)
    step = max(1, (hi - lo) // 40)
    good = [
        thr
        for thr in range(max(2, lo), hi + 1, step)
        if count_sightings(samples, thr) == expect
    ]
    return good, lo, hi


def sweep_gantry(samples: List[Sample], expect: int) -> None:
    """min_gantry_span_px 를 훑어 목격 횟수가 기대치와 맞는 구간을 찾는다.

    격자와 판정은 find_good_thresholds 와 반드시 같은 코드여야 한다 —
    pytest 가 보증하는 격자와 현장 도구의 격자가 달라지면 랩 임계값
    (실격 직결)이 검증 안 된 복사본으로 정해진다 (리뷰 R3).
    """
    good, lo, hi = find_good_thresholds(samples, expect)
    spans = [s.gantry_span for s in samples if s.gantry_span > 0]
    if not spans:
        print("❌ 초록 블롭이 한 번도 안 잡혔다 — 신호등이 초록 점등 상태로 "
              "화면에 들어오는 bag 인지부터 확인할 것.")
        return

    print(f"\n관측된 span 범위: {lo} ~ {hi}  ({len(spans)}프레임)")
    print(f"목표 목격 횟수: {expect}회 "
          f"(출발 1 + 랩마커 {expect - 1})\n")
    print(f"  {'임계':>6}  {'목격':>4}  판정")
    print(f"  {'-' * 6}  {'-' * 4}  {'-' * 30}")

    step = max(1, (hi - lo) // 40)
    for thr in range(max(2, lo), hi + 1, step):
        n = count_sightings(samples, thr)
        if n == expect:
            verdict = "✅ 정확"
        elif n < expect:
            verdict = f"랩 누락 ({expect - n}회 놓침) → 3바퀴 미완주"
        else:
            verdict = f"중복 카운트 ({n - expect}회 초과) → 조기 종료"
        print(f"  {thr:>6}  {n:>4}  {verdict}")

    print()
    if good:
        best = good[len(good) // 2]
        print(f"✅ 정확한 구간: {good[0]} ~ {good[-1]}")
        print(f"   권장값 = {best}  (구간 중앙 — 양쪽 여유가 가장 크다)")
        print(f"   → core/race_context.py 의 "
              f"LapTrackerConfig.min_gantry_span_px = {best}")
        if len(good) < 3:
            print(f"   ⚠️  안전 구간이 좁다({len(good)}칸). 조명이 바뀌면 깨질 수 있으니")
            print(f"      오전·오후 bag 을 각각 받아 교집합을 쓸 것")
    else:
        print("❌ 어떤 임계값으로도 목격 횟수가 안 맞는다.")
        print("   원인 후보:")
        print("     · bag 이 실제로 3바퀴가 아니다 → --expect 로 맞출 것")
        print("     · 겐트리 검출이 불안정 → SignalConfig 반지름/ROI 조정")
        print("     · release_frames / min_gap_s 가 안 맞음")


# ──────────────────────────────────────────────────────────────────
# FSM 전체 재생
# ──────────────────────────────────────────────────────────────────
def replay_fsm(samples: List[Sample]) -> None:
    """녹화된 주행을 FSM+정책에 통째로 흘려 본다.

    수동주행 bag 이라 조향 명령은 실제 궤적과 무관하다. 여기서 보는 건
    **상태 전이가 제때 일어나는가** — 특히 랩 카운팅과 지름길 판단이다.
    """
    from ..core.policy import DrivePolicy
    from ..core.race_context import RaceContext
    from ..core.race_fsm import Mode, RaceFSM
    from ..core.safety import SafetyDecision, SafetyLevel
    from ..core.types import (
        ConeReading,
        LaneReading,
        LidarReading,
        Observation,
        SignalReading,
        SignalState,
    )

    ready = SafetyDecision(SafetyLevel.OK, "", True)
    fsm, ctx, pol = RaceFSM(), RaceContext(), DrivePolicy()
    t0 = samples[0].t
    prev = fsm.state

    print("\n[ FSM 재생 ]")
    for s in samples:
        obs = Observation(
            now=s.t,
            signal=SignalReading(
                state=SignalState(s.signal)
                if s.signal in SignalState.__members__
                else SignalState.UNKNOWN,
                gantry_span_px=s.gantry_span,
            ),
            lane=LaneReading(
                offset_px=s.lane_offset,
                lookahead_px=s.lane_offset,
                left_found=s.lane_valid,
                right_found=s.lane_valid,
                lane_width_px=450.0,
            ),
            lidar=LidarReading(front_dist=s.front_dist, stamp=s.t),
            cone=ConeReading(
                detected=s.cone_px > 0,
                orange_px=s.cone_px,
                gate_center_px=s.gate_center,
                gate_valid=s.gate_valid,
            ),
            yaw_deg=s.yaw,
            ego_speed_mps=1.0,
            camera_ok=True,
            lidar_ok=math.isfinite(s.front_dist),
        )
        fsm.maneuver_done = pol.maneuver_done
        tr = fsm.step(obs, ctx, ready)
        pol.update(fsm.state, obs, ctx, ready)
        if tr.changed:
            print(
                f"  {s.t - t0:7.2f}s  {tr.source.value:<12} → "
                f"{tr.target.value:<12}  lap{ctx.lap.current_lap}/3  {tr.reason}"
            )
        prev = fsm.state

    print(f"\n  최종 상태     : {fsm.state.value}")
    print(f"  목격 횟수     : {ctx.lap.sightings}  (기대 4 = 출발 1 + 랩 3)")
    print(f"  지름길        : {'사용' if ctx.shortcut_used else '미사용'}")
    if ctx.race_started_at is not None:
        print(f"  주행시간      : {ctx.elapsed(samples[-1].t):.1f}s / 240s")
    if fsm.state is not Mode.FINISH:
        print("  ⚠️  FINISH 에 도달하지 못했다 — --sweep-gantry 로 임계값을 먼저 잡을 것")


# ──────────────────────────────────────────────────────────────────
def summarize(samples: List[Sample]) -> None:
    if not samples:
        print("❌ 이미지 프레임이 없다. --image-topic 을 확인할 것.")
        return
    t0, t1 = samples[0].t, samples[-1].t
    print(f"\n[ 요약 ]  {len(samples)}프레임 / {t1 - t0:.1f}초 "
          f"({len(samples) / max(1e-6, t1 - t0):.1f} fps)")

    spans = [s.gantry_span for s in samples if s.gantry_span > 0]
    print(f"  겐트리 검출   : {len(spans)}프레임 "
          + (f"span {min(spans)}~{max(spans)}" if spans else "(없음)"))

    states: Dict[str, int] = {}
    for s in samples:
        key = s.signal if s.signal != "UNKNOWN" else f"UNKNOWN({s.reject})"
        states[key] = states.get(key, 0) + 1
    print("  신호 판정     :")
    for k, v in sorted(states.items(), key=lambda kv: -kv[1])[:6]:
        print(f"      {v:5d}  {k}")

    lane_ok = sum(1 for s in samples if s.lane_valid)
    print(f"  차선 검출     : {lane_ok}/{len(samples)} "
          f"({100 * lane_ok / len(samples):.0f}%)")
    cones = [s.cone_px for s in samples]
    gates = sum(1 for s in samples if s.gate_valid)
    print(f"  주황 픽셀     : 최대 {max(cones)}  "
          f"900 초과 {sum(1 for c in cones if c > 900)}프레임")
    print(f"  콘 게이트 성립: {gates}프레임")
    yaws = [s.yaw for s in samples]
    print(f"  요 누적       : {yaws[-1] - yaws[0]:+.0f}° "
          f"(한 바퀴≈±360°, 3바퀴≈±1080°)")


def dump_csv(samples: List[Sample], path: str) -> None:
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(
            ["t", "gantry_span", "signal", "reject", "lane_offset", "lane_valid",
             "cone_px", "gate_center", "gate_valid", "front_dist", "yaw"]
        )
        for s in samples:
            w.writerow(
                [f"{s.t:.3f}", s.gantry_span, s.signal, s.reject,
                 f"{s.lane_offset:.1f}", int(s.lane_valid), s.cone_px,
                 f"{s.gate_center:.1f}", int(s.gate_valid),
                 f"{s.front_dist:.3f}", f"{s.yaw:.1f}"]
            )
    print(f"\n💾 {path} 에 {len(samples)}행 저장")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="kmu_race bag 리플레이")
    p.add_argument("bag", help="ros2 bag 디렉터리")
    p.add_argument("--image-topic", default="/image_raw")
    p.add_argument("--scan-topic", default="/scan")
    p.add_argument("--imu-topic", default="/imu")
    p.add_argument("--stride", type=int, default=1, help="N프레임마다 1장 처리")
    p.add_argument("--fsm", action="store_true", help="FSM 까지 재생")
    p.add_argument(
        "--sweep-gantry", action="store_true",
        help="min_gantry_span_px 자동 탐색 (3바퀴 bag 필요)",
    )
    p.add_argument(
        "--expect", type=int, default=4,
        help="기대 목격 횟수 (기본 4 = 출발 1 + 랩 3)",
    )
    p.add_argument("--csv", help="프레임별 값을 CSV 로 덤프")
    args, _ = p.parse_known_args(argv)

    if not os.path.isdir(args.bag):
        print(f"❌ bag 디렉터리가 없다: {args.bag}")
        sys.exit(1)

    print(f"읽는 중: {args.bag}")
    samples = run_perception(
        args.bag,
        image_topic=args.image_topic,
        scan_topic=args.scan_topic,
        imu_topic=args.imu_topic,
        stride=args.stride,
    )
    summarize(samples)
    if args.sweep_gantry:
        sweep_gantry(samples, args.expect)
    if args.fsm:
        replay_fsm(samples)
    if args.csv:
        dump_csv(samples, args.csv)


if __name__ == "__main__":
    main()

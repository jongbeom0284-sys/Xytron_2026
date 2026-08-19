"""차선 검출 회귀 벤치 — bag 을 넣으면 '좋아졌나 나빠졌나'가 숫자로 나온다.

왜 이게 검출기보다 먼저인가 (0818 실사고):
    낮에 넣은 수정(주황·노랑 제외 + 글레어 필터)이 같은 bag 에서 양쪽 검출을
    27.1% → 0.8% 로 떨어뜨렸는데 **아무도 몰랐다.** 눈으로 몇 프레임 보고
    "좋아진 것 같다"로 판단하면 이런 개악이 그대로 차에 올라간다.
    D-7 에 가장 위험한 건 나쁜 검출기가 아니라 **좋아졌는지 모르는 상태**다.

무엇을 재는가 — 정답 라벨 없이 쓸 수 있는 대리 지표 4종:
    ① 검출률       : 목표선을 잡은 프레임 비율. 높을수록 좋다.
    ② 양쪽 관측률   : 흰 선 좌우를 동시에 본 비율. 폭·곡률 추정의 근거라
                     높을수록 좋지만, 이 트랙은 구조적으로 낮다(코너에서
                     바깥 선이 화각 밖). 절대값보다 **버전 간 비교**가 목적.
    ③ 오프셋 도약   : 프레임 간 |Δoffset|. 실제 차는 순간이동하지 않으므로
                     큰 도약 = 검출 오류다. 30fps·1m/s 에서 실제 횡이동은
                     ~15px. **95% 분위수가 핵심 지표** — 이게 조향 슬램이 된다.
    ④ 좌우 뒤바뀜   : 같은 물리 선이 프레임마다 좌↔우로 라벨이 바뀐 횟수.
                     한 번 뒤바뀌면 목표가 차선폭만큼 튄다(실측 최대 422px).

사용:
    # 단일 설정
    ros2 run kmu_race lanebench ~/kmu_bags/auto_0818_1153 --stride 12

    # A/B — 기준(현행 배포본)과 후보를 같은 프레임에 돌려 비교
    ros2 run kmu_race lanebench ~/kmu_bags/auto_0818_1153 --stride 12 \
        --variant baseline:legacy --variant cand:default

    # 큰 bag 의 일부만 (파일 인덱스로 자르기 — 58분 bag 전체는 6분 걸린다)
    ros2 run kmu_race lanebench ~/kmu_bags/auto_0818_1153 --files 48,50,52,55,57

주의: **녹화 중인 bag 은 열지 말 것** (sqlite 잠금으로 recorder 즉사).
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Iterable, List, Optional, Sequence

import numpy as np

from ..core.types import LaneReading
from ..perception.lane_detector import LaneConfig, LaneDetector


# ──────────────────────────────────────────────────────────────────
# 지표 (순수 함수 — bag/cv2 없이 단위 테스트 가능)
# ──────────────────────────────────────────────────────────────────
@dataclass
class BenchResult:
    name: str
    n: int = 0
    detect_rate: float = 0.0
    both_rate: float = 0.0
    center_rate: float = 0.0
    jump_med: float = 0.0
    jump_p95: float = 0.0
    jump_max: float = 0.0
    side_flips: int = 0
    curv_jump_p95: float = 0.0
    ms_per_frame: float = 0.0

    def line(self) -> str:
        return (
            f"{self.name:<22} {self.detect_rate:5.1f}% {self.both_rate:5.1f}% "
            f"{self.center_rate:5.1f}% {self.jump_med:6.1f} {self.jump_p95:7.1f} "
            f"{self.jump_max:8.1f} {self.side_flips:6d} {self.curv_jump_p95:9.5f} "
            f"{self.ms_per_frame:7.2f}"
        )

    @staticmethod
    def header() -> str:
        return (
            f"{'설정':<22} {'검출':>6} {'양쪽':>6} {'중앙선':>6} "
            f"{'도약중앙':>6} {'도약95%':>7} {'도약최대':>8} "
            f"{'뒤바뀜':>6} {'곡률도약95%':>9} {'ms/frame':>7}"
        )


def score(name: str, readings: Sequence[LaneReading], ms: float = 0.0) -> BenchResult:
    """LaneReading 시퀀스 → 지표. 검출기와 무관하므로 합성 데이터로도 검증된다."""
    n = len(readings)
    if n == 0:
        return BenchResult(name=name)

    valid = sum(1 for r in readings if r.valid)
    both = sum(1 for r in readings if r.left_found and r.right_found)
    center = sum(1 for r in readings if r.center_found)

    # 도약: '직전에도 잡혔고 지금도 잡힌' 구간에서만 센다. 놓쳤다 재획득하는
    # 순간의 점프는 검출기 품질이 아니라 재획득이라 별개 사건이다.
    jumps: List[float] = []
    curv_jumps: List[float] = []
    for a, b in zip(readings, readings[1:]):
        if a.valid and b.valid:
            jumps.append(abs(b.offset_px - a.offset_px))
            curv_jumps.append(abs(b.curvature_px - a.curvature_px))

    # 좌우 뒤바뀜: 단측 관측이 좌 → 우 (또는 그 반대) 로 직접 넘어간 횟수.
    # 양쪽 관측을 거쳐 넘어간 것은 정상 전이라 세지 않는다.
    flips = 0
    last_side: Optional[int] = None
    for r in readings:
        if r.left_found and r.right_found:
            side: Optional[int] = 0
        elif r.left_found:
            side = -1
        elif r.right_found:
            side = 1
        else:
            side = None
        if side in (-1, 1) and last_side in (-1, 1) and side != last_side:
            flips += 1
        if side is not None:
            last_side = side

    def pct(v: Iterable[float], q: float) -> float:
        arr = np.asarray(list(v), dtype=float)
        return float(np.percentile(arr, q)) if arr.size else 0.0

    return BenchResult(
        name=name,
        n=n,
        detect_rate=100.0 * valid / n,
        both_rate=100.0 * both / n,
        center_rate=100.0 * center / n,
        jump_med=pct(jumps, 50),
        jump_p95=pct(jumps, 95),
        jump_max=max(jumps) if jumps else 0.0,
        side_flips=flips,
        curv_jump_p95=pct(curv_jumps, 95),
        ms_per_frame=ms,
    )


# ──────────────────────────────────────────────────────────────────
# 설정 변형 (A/B 용)
# ──────────────────────────────────────────────────────────────────
def _legacy_config() -> LaneConfig:
    """0818 이전 동작 재현 — 절대 흰색 임계 + Canny/Sobel + 컬럼 히스토그램.

    '새 검출기가 정말 나은가'의 대조군이다. 이 값들은 되돌리기용 화석이지
    운영 파라미터가 아니다.
    """
    c = LaneConfig()
    c.binarize_mode = "legacy"
    c.assoc_mode = "histogram"
    c.white_v = (243, 255)
    c.default_lane_width_px = 450.0
    return c


VARIANTS: Dict[str, Callable[[], LaneConfig]] = {
    "default": LaneConfig,                      # 현재 기본값 (= 차에 올라갈 것)
    "legacy": _legacy_config,                   # 0818 이전 재현
}


def resolve_variant(spec: str) -> "tuple[str, LaneConfig]":
    """'이름:변형' 또는 '변형' → (표시이름, LaneConfig)."""
    if ":" in spec:
        name, key = spec.split(":", 1)
    else:
        name = key = spec
    if key not in VARIANTS:
        raise SystemExit(
            f"❌ 모르는 변형 '{key}' — 가능한 값: {', '.join(sorted(VARIANTS))}"
        )
    return name, VARIANTS[key]()


# ──────────────────────────────────────────────────────────────────
# 프레임 소스
# ──────────────────────────────────────────────────────────────────
def frames_from_bag(
    path: str, *, image_topic: str, stride: int, limit: int = 0
) -> Iterable[np.ndarray]:
    """bag 의 이미지 프레임. raw 가 없으면 compressed 로 자동 보정한다."""
    from cv_bridge import CvBridge

    from .replay import _decode_image, bag_topics, read_bag

    topics = bag_topics(path)
    if image_topic not in topics:
        alt = image_topic.rstrip("/") + "/compressed"
        if alt in topics:
            print(f"ℹ️  {image_topic} 없음 → {alt}")
            image_topic = alt
        else:
            raise SystemExit(f"❌ bag 에 이미지 토픽이 없다: {image_topic}")

    bridge = CvBridge()
    n = 0
    out = 0
    for topic, msg, _t in read_bag(path):
        if topic != image_topic:
            continue
        n += 1
        if stride > 1 and (n % stride) != 0:
            continue
        try:
            frame = _decode_image(bridge, msg)
        except Exception:                              # noqa: BLE001
            continue
        yield frame
        out += 1
        if limit and out >= limit:
            return


def frames_from_dir(path: str, *, stride: int, limit: int = 0) -> Iterable[np.ndarray]:
    """PNG/JPG 디렉터리. ROS 없이(맥 로컬) 회귀를 돌릴 때 쓴다."""
    import cv2

    names = sorted(
        f for f in os.listdir(path)
        if f.lower().endswith((".png", ".jpg", ".jpeg"))
    )
    out = 0
    for i, fn in enumerate(names):
        if stride > 1 and (i % stride) != 0:
            continue
        img = cv2.imread(os.path.join(path, fn), 1)
        if img is None:
            continue
        yield img
        out += 1
        if limit and out >= limit:
            return


def filter_bag_files(path: str, spec: str) -> str:
    """--files 48,50 → 그 db3 만 담은 임시 bag 디렉터리 경로를 만들어 돌려준다.

    58분 bag 전체를 매번 도는 건 6분이라 반복 튜닝에 못 쓴다. rosbag2 는
    metadata.yaml 의 relative_file_paths 로 조각을 고르므로, 원본은 그대로
    두고 **심볼릭 링크 + 축약 metadata** 만 새로 만든다 (원본 무변경).
    """
    import shutil

    import yaml

    meta_src = os.path.join(path, "metadata.yaml")
    if not os.path.isfile(meta_src):
        raise SystemExit(f"❌ metadata.yaml 이 없다 (녹화가 안 끝난 bag?): {path}")
    with open(meta_src) as fh:
        meta = yaml.safe_load(fh)
    info = meta["rosbag2_bagfile_information"]
    want = {s.strip() for s in spec.split(",") if s.strip()}
    keep = [
        p for p in info["relative_file_paths"]
        if any(p.endswith(f"_{w}.db3") for w in want)
    ]
    if not keep:
        raise SystemExit(f"❌ --files {spec} 에 해당하는 db3 가 없다")

    dst = os.path.join(
        os.environ.get("TMPDIR", "/tmp"),
        f"lanebench_{os.path.basename(path.rstrip('/'))}_{'_'.join(sorted(want))}",
    )
    shutil.rmtree(dst, ignore_errors=True)
    os.makedirs(dst)
    for rel in keep:
        os.symlink(os.path.join(os.path.abspath(path), rel), os.path.join(dst, rel))
    info["relative_file_paths"] = keep
    with open(os.path.join(dst, "metadata.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh)
    print(f"ℹ️  db3 {len(keep)}개만 사용 → {dst}")
    return dst


# ──────────────────────────────────────────────────────────────────
# 실행
# ──────────────────────────────────────────────────────────────────
def run(
    frames: Sequence[np.ndarray], variants: Sequence["tuple[str, LaneConfig]"]
) -> List[BenchResult]:
    results = []
    for name, cfg in variants:
        det = LaneDetector(cfg)
        readings: List[LaneReading] = []
        t0 = time.perf_counter()
        for f in frames:
            readings.append(det.detect(f))
        ms = 1000.0 * (time.perf_counter() - t0) / max(1, len(frames))
        results.append(score(name, readings, ms))
    return results


def report(results: Sequence[BenchResult], baseline: Optional[str] = None) -> None:
    print()
    print(BenchResult.header())
    print("-" * len(BenchResult.header()))
    for r in results:
        print(r.line())
    base = next((r for r in results if r.name == baseline), None)
    if base is None and len(results) > 1:
        base = results[0]
    if base is not None and len(results) > 1:
        print()
        print(f"[{base.name} 대비]")
        for r in results:
            if r is base:
                continue
            print(
                f"  {r.name:<20} 검출 {r.detect_rate - base.detect_rate:+5.1f}%p  "
                f"양쪽 {r.both_rate - base.both_rate:+5.1f}%p  "
                f"도약95% {r.jump_p95 - base.jump_p95:+7.1f}px  "
                f"뒤바뀜 {r.side_flips - base.side_flips:+d}회"
            )
    print()
    print("해석: 검출·양쪽은 높을수록, 도약·뒤바뀜은 낮을수록 좋다.")
    print("      도약95% 가 실제 조향 슬램의 크기다 — 이 값이 제일 중요하다.")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(description="차선 검출 회귀 벤치")
    p.add_argument("source", help="ros2 bag 디렉터리 또는 이미지 디렉터리")
    p.add_argument("--image-topic", default="/image_raw")
    p.add_argument("--stride", type=int, default=12, help="N프레임마다 1장 (기본 12)")
    p.add_argument("--limit", type=int, default=0, help="최대 프레임 수 (0=제한없음)")
    p.add_argument(
        "--files", default="",
        help="db3 인덱스만 골라 쓴다 (예: 48,50,52) — 큰 bag 반복 튜닝용",
    )
    p.add_argument(
        "--variant", action="append", default=[],
        help=f"'이름:변형' 반복 지정. 변형: {', '.join(sorted(VARIANTS))}",
    )
    p.add_argument("--baseline", default="", help="증감 기준으로 삼을 변형 이름")
    args = p.parse_args(argv)

    if not os.path.isdir(args.source):
        raise SystemExit(f"❌ 디렉터리가 아니다: {args.source}")

    specs = args.variant or ["legacy", "default"]
    variants = [resolve_variant(s) for s in specs]

    src = args.source
    is_bag = os.path.isfile(os.path.join(src, "metadata.yaml")) or any(
        f.endswith((".db3", ".mcap")) for f in os.listdir(src)
    )
    if is_bag:
        if args.files:
            src = filter_bag_files(src, args.files)
        frames = list(
            frames_from_bag(
                src, image_topic=args.image_topic, stride=args.stride, limit=args.limit
            )
        )
    else:
        frames = list(frames_from_dir(src, stride=args.stride, limit=args.limit))

    if not frames:
        raise SystemExit("❌ 프레임을 하나도 읽지 못했다")
    print(f"프레임 {len(frames)}장 · 변형 {len(variants)}종")

    report(run(frames, variants), args.baseline or None)


if __name__ == "__main__":                             # pragma: no cover
    main()

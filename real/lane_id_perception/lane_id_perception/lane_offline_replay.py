from __future__ import annotations

import argparse
import csv
from pathlib import Path
import time

import cv2

from .lane_config import LaneVisionConfig
from .lane_vision import LaneDetector


def iter_images(input_path: Path):
    if input_path.is_dir():
        for path in sorted(input_path.glob("*")):
            if path.suffix.lower() in {".png", ".jpg", ".jpeg", ".bmp"}:
                yield path, cv2.imread(str(path), cv2.IMREAD_COLOR)
    else:
        cap = cv2.VideoCapture(str(input_path))
        index = 0
        while cap.isOpened():
            ok, frame = cap.read()
            if not ok:
                break
            yield Path(f"frame_{index:06d}.jpg"), frame
            index += 1
        cap.release()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Mode C offline lane replay.")
    parser.add_argument("input", type=Path, help="Directory of frames or video file.")
    parser.add_argument("--output", type=Path, default=Path("lane_replay_out"))
    parser.add_argument("--no-debug-images", action="store_true")
    parser.add_argument("--initial-lane-width-m", type=float, default=1.35)
    parser.add_argument("--vehicle-width-m", type=float, default=0.42)
    parser.add_argument("--safety-margin-m", type=float, default=0.08)
    args = parser.parse_args(argv)

    cfg = LaneVisionConfig(
        initial_lane_width_m=args.initial_lane_width_m,
        vehicle_width_m=args.vehicle_width_m,
        safety_margin_m=args.safety_margin_m,
    )
    detector = LaneDetector(cfg)
    args.output.mkdir(parents=True, exist_ok=True)
    frames_dir = args.output / "frames"
    if not args.no_debug_images:
        frames_dir.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "lane_replay.csv"

    fields = [
        "frame_id",
        "source",
        "state",
        "left_lane_confidence",
        "right_lane_confidence",
        "overall_lane_confidence",
        "estimated_lane_width",
        "lateral_error_near",
        "lateral_error_far",
        "heading_error",
        "estimated_curvature",
        "min_safe_clearance_m",
    ]
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for frame_id, (path, frame) in enumerate(iter_images(args.input), start=1):
            if frame is None:
                continue
            result = detector.process(frame, capture_timestamp=time.time(), frame_id=frame_id, make_debug=not args.no_debug_images)
            writer.writerow(
                {
                    "frame_id": frame_id,
                    "source": str(path),
                    "state": result.state,
                    "left_lane_confidence": f"{result.left_lane_confidence:.6f}",
                    "right_lane_confidence": f"{result.right_lane_confidence:.6f}",
                    "overall_lane_confidence": f"{result.overall_lane_confidence:.6f}",
                    "estimated_lane_width": f"{result.estimated_lane_width:.6f}",
                    "lateral_error_near": f"{result.lateral_error_near:.6f}",
                    "lateral_error_far": f"{result.lateral_error_far:.6f}",
                    "heading_error": f"{result.heading_error:.6f}",
                    "estimated_curvature": f"{result.estimated_curvature:.6f}",
                    "min_safe_clearance_m": f"{result.min_safe_clearance_m:.6f}",
                }
            )
            if result.debug_image is not None and not args.no_debug_images:
                cv2.imwrite(str(frames_dir / f"{frame_id:06d}_lane_debug.jpg"), result.debug_image)
    print(f"offline replay csv: {csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

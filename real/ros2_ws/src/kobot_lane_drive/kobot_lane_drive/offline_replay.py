from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from .clrernet_engine import CLRerNetEngine
from .lane_selector import select_left_lane


def _image_paths(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    return [path for path in sorted(source.iterdir()) if path.suffix.lower() in extensions]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vendor-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    engine = CLRerNetEngine(args.vendor_root, args.config, args.checkpoint, device=args.device)
    records: list[dict[str, object]] = []
    for path in _image_paths(args.input):
        source = cv2.imread(str(path))
        if source is None:
            raise RuntimeError(f"Cannot read image: {path}")
        inference = engine.infer(source)
        lane = select_left_lane(inference.lanes, source.shape[1], source.shape[0])
        overlay = source.copy()
        if lane.points is not None:
            points = np.round(lane.points).astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(overlay, [points], False, (255, 255, 255), 4, cv2.LINE_AA)
        cv2.putText(overlay, f"LEFT {lane.state} | {inference.elapsed_ms:.1f} ms", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 80), 1, cv2.LINE_AA)
        cv2.putText(overlay, "OUTPUT GENERATED - ACCURACY UNVERIFIED", (8, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 180, 255), 1, cv2.LINE_AA)
        cv2.imwrite(str(args.output / path.name), overlay)
        records.append({
            "image": path.name,
            "state": lane.state,
            "source_lane_index": lane.source_index,
            "source_lane_count": lane.source_lane_count,
            "inference_ms": round(inference.elapsed_ms, 2),
        })
    (args.output / "summary.json").write_text(
        json.dumps({"accuracy": "unverified", "motor_output": False, "records": records},
                   ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

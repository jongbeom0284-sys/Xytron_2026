from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from .fast_lane import detect_fast_lane


def evaluate_directory(image_directory: Path, overlay_directory: Path | None = None) -> dict[str, object]:
    image_paths = sorted(
        path
        for path in image_directory.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg", ".png"}
    )
    if not image_paths:
        raise ValueError(f"No images found in {image_directory}")
    if overlay_directory is not None:
        overlay_directory.mkdir(parents=True, exist_ok=True)

    states: Counter[str] = Counter()
    processing_times: list[float] = []
    confidences: list[float] = []
    for index, path in enumerate(image_paths):
        frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError(f"Could not read {path}")
        save_overlay = overlay_directory is not None and index % 20 == 0
        result = detect_fast_lane(frame, render_debug=save_overlay)
        states[result.state] += 1
        processing_times.append(result.processing_ms)
        confidences.append(result.confidence)
        if save_overlay and result.debug_image is not None:
            cv2.imwrite(str(overlay_directory / path.name), result.debug_image)

    values = np.asarray(processing_times, dtype=np.float64)
    return {
        "image_directory": str(image_directory),
        "frame_count": len(image_paths),
        "states": dict(sorted(states.items())),
        "valid_ratio": round(1.0 - states["LOST"] / len(image_paths), 5),
        "confidence_mean": round(float(np.mean(confidences)), 4),
        "processing_ms": {
            "mean": round(float(np.mean(values)), 3),
            "p50": round(float(np.percentile(values, 50)), 3),
            "p95": round(float(np.percentile(values, 95)), 3),
            "p99": round(float(np.percentile(values, 99)), 3),
            "max": round(float(np.max(values)), 3),
        },
        "motor_output": False,
        "metric_calibrated": False,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Benchmark calibration-free fast lane detection on recorded frames"
    )
    parser.add_argument("image_directory", type=Path)
    parser.add_argument("--overlay-directory", type=Path)
    arguments = parser.parse_args()
    report = evaluate_directory(
        arguments.image_directory,
        arguments.overlay_directory,
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

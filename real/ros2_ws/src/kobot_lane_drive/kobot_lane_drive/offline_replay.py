from __future__ import annotations

import argparse
import json
from dataclasses import asdict
from pathlib import Path

import cv2

from .clrernet_engine import CLRerNetEngine
from .image_reference_controller import (
    ImageReferenceController,
    capture_pixel_reference,
)
from .lane_selector import select_left_lane
from .overlay import draw_perception_overlay


def _image_paths(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    extensions = {".jpg", ".jpeg", ".png", ".bmp"}
    return [
        path for path in sorted(source.iterdir()) if path.suffix.lower() in extensions
    ]


def _read_image(path: Path):
    source = cv2.imread(str(path))
    if source is None:
        raise RuntimeError(f"Cannot read image: {path}")
    return source


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vendor-root", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--reference-image", type=Path)
    parser.add_argument("--near-y-ratio", type=float, default=0.85)
    parser.add_argument("--far-y-ratio", type=float, default=0.60)
    parser.add_argument("--near-pid-kp", type=float, default=1.0)
    parser.add_argument("--far-pid-kp", type=float, default=0.65)
    parser.add_argument("--maximum-steering-rad", type=float, default=0.34)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    engine = CLRerNetEngine(
        args.vendor_root, args.config, args.checkpoint, device=args.device
    )
    controller: ImageReferenceController | None = None
    reference_payload: dict[str, object] | None = None
    if args.reference_image is not None:
        reference_source = _read_image(args.reference_image)
        reference_inference = engine.infer(reference_source)
        reference_lane = select_left_lane(
            reference_inference.lanes,
            reference_source.shape[1],
            reference_source.shape[0],
        )
        if reference_lane.points is None:
            raise RuntimeError("REFERENCE_LEFT_LANE_LOST")
        reference = capture_pixel_reference(
            reference_lane.points,
            reference_source.shape[1],
            reference_source.shape[0],
            near_y_ratio=args.near_y_ratio,
            far_y_ratio=args.far_y_ratio,
        )
        controller = ImageReferenceController(
            reference,
            near_tunings=(args.near_pid_kp, 0.0, 0.0),
            far_tunings=(args.far_pid_kp, 0.0, 0.0),
            maximum_steering_rad=args.maximum_steering_rad,
        )
        reference_payload = {
            "source_image": str(args.reference_image),
            **asdict(reference),
            "reference_ready": True,
            "accuracy": "unverified",
        }
        (args.output / "pixel_reference.json").write_text(
            json.dumps(reference_payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    records: list[dict[str, object]] = []
    for path in _image_paths(args.input):
        source = _read_image(path)
        inference = engine.infer(source)
        lane = select_left_lane(inference.lanes, source.shape[1], source.shape[0])
        preview = None
        error = None
        if lane.points is None:
            error = "LEFT_LANE_LOST"
        elif controller is None:
            error = "REFERENCE_NOT_READY"
        else:
            try:
                preview = controller.calculate(
                    lane.points, source.shape[1], source.shape[0], dt=0.1
                )
            except Exception as exc:  # noqa: BLE001 - record every blocked frame
                controller.reset()
                error = str(exc)

        overlay = draw_perception_overlay(
            source, lane, preview, inference.elapsed_ms, error
        )
        cv2.imwrite(str(args.output / path.name), overlay)
        measurement = None if preview is None else preview.measurement
        records.append(
            {
                "image": path.name,
                "state": "READY" if error is None else "BLOCKED",
                "left_lane": lane.state,
                "source_lane_index": lane.source_index,
                "source_lane_count": lane.source_lane_count,
                "near_x_ratio": (
                    None if measurement is None else round(measurement.near_x_ratio, 6)
                ),
                "far_x_ratio": (
                    None if measurement is None else round(measurement.far_x_ratio, 6)
                ),
                "near_error_ratio": (
                    None
                    if measurement is None
                    else round(measurement.near_error_ratio, 6)
                ),
                "far_error_ratio": (
                    None
                    if measurement is None
                    else round(measurement.far_error_ratio, 6)
                ),
                "steering_preview_rad": (
                    None if preview is None else round(preview.steering_angle_rad, 6)
                ),
                "inference_ms": round(inference.elapsed_ms, 2),
                "error": error,
            }
        )
    (args.output / "summary.json").write_text(
        json.dumps(
            {
                "accuracy": "unverified",
                "motor_output": False,
                "control_mode": "fixed_camera_pixel_reference",
                "reference": reference_payload,
                "records": records,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()

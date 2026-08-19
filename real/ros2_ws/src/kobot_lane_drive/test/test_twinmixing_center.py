from pathlib import Path

import numpy as np

from kobot_lane_drive.twinmixing_center import (
    TwinMixingCenterConfig,
    extract_center_path,
)


def _synthetic_masks(width=640, height=480, center_shift=0):
    driving = np.zeros((height, width), dtype=bool)
    lane = np.zeros_like(driving)
    for y in range(int(height * 0.45), int(height * 0.98)):
        progress = (y - height * 0.45) / (height * 0.53)
        center = int(width * 0.5 + center_shift * progress)
        half_gap = max(3, int(4 + 15 * progress))
        outer = int(width * (0.08 + 0.34 * (1.0 - progress)))
        driving[y, outer : center - half_gap] = True
        driving[y, center + half_gap : width - outer] = True
        lane[y, center - 1 : center + 2] = True
    return driving, lane


def test_extracts_middle_gap_between_two_drive_areas():
    driving, lane = _synthetic_masks(center_shift=70)
    result = extract_center_path(
        driving,
        lane,
        TwinMixingCenterConfig(minimum_points=12),
    )
    assert result.found
    assert result.state == "TRACKED"
    assert result.center_offset_px is not None
    assert result.center_offset_px > 15
    assert result.confidence >= 0.35


def test_rejects_single_unseparated_drive_area():
    driving = np.zeros((480, 640), dtype=bool)
    driving[220:470, 80:560] = True
    result = extract_center_path(driving)
    assert not result.found
    assert result.state == "LOST"


def test_runtime_node_never_publishes_motor_topic():
    package_root = Path(__file__).parents[1]
    source = (
        package_root / "kobot_lane_drive" / "twinmixing_center_node.py"
    ).read_text(encoding="utf-8")
    assert '"/xycar_motor"' not in source
    assert '"motor_output": False' in source

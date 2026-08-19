from pathlib import Path


def test_sensor_launch_uses_twinmixing_and_never_starts_motor_bridge():
    package_root = Path(__file__).parents[1]
    source = (package_root / "launch" / "sensors.launch.py").read_text(
        encoding="utf-8"
    )

    assert "lane_preview" not in source
    assert "package='kobot_lane_drive', executable='twinmixing_center_drive'" in source
    assert "twinmixing_center_drive_bridge" not in source
    assert "white_left" not in source
    assert "/xycar_motor" not in source


def test_legacy_lane_preview_is_not_installed():
    package_root = Path(__file__).parents[1]
    setup_source = (package_root / "setup.py").read_text(encoding="utf-8")

    assert "lane_preview" not in setup_source
    assert not (package_root / "kobot_vehicle" / "lane_preview_node.py").exists()

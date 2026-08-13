from pathlib import Path


def test_perception_runtime_has_no_motor_publish_interface():
    package_root = Path(__file__).parents[1] / "kobot_lane_drive"
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in package_root.glob("*.py")
        if path.name != "drive_bridge_node.py"
    )

    assert "/xycar_motor" not in source
    assert "/ackermann_cmd" not in source
    assert "/commands/motor/speed" not in source
    assert "/commands/servo/position" not in source


def test_drive_bridge_requires_explicit_arming():
    package_root = Path(__file__).parents[1]
    bridge = (package_root / "kobot_lane_drive" / "drive_bridge_node.py").read_text(
        encoding="utf-8"
    )
    launch = (package_root / "launch" / "track_test.launch.py").read_text(
        encoding="utf-8"
    )
    assert 'declare_parameter("armed", False)' in bridge
    assert 'default_value="false"' in launch
    assert "IfCondition(armed)" in launch

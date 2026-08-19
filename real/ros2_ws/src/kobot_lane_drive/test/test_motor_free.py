from pathlib import Path


def test_perception_runtime_has_no_motor_publish_interface():
    package_root = Path(__file__).parents[1] / "kobot_lane_drive"
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in package_root.glob("*.py")
        if path.name not in {
            "drive_bridge_node.py",
            "fast_lane_drive_bridge_node.py",
            "white_left_drive_bridge_node.py",
        }
    )

    assert "/xycar_motor" not in source
    assert "/ackermann_cmd" not in source
    assert "/commands/motor/speed" not in source
    assert "/commands/servo/position" not in source


def test_perception_runtime_uses_no_metric_camera_calibration():
    package_root = Path(__file__).parents[1]
    node = (package_root / "kobot_lane_drive" / "lane_perception_node.py").read_text(
        encoding="utf-8"
    )
    config = (package_root / "config" / "lane_drive.yaml").read_text(
        encoding="utf-8"
    )
    manifest = (package_root / "package.xml").read_text(encoding="utf-8")

    assert "CameraInfo" not in node
    assert "IPM" not in node
    assert "tf2_ros" not in node
    assert "target_distance_m" not in config
    assert "ipm_library" not in manifest
    assert "reference_ready: false" in config


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


def test_fast_lane_drive_bridge_requires_explicit_arming():
    package_root = Path(__file__).parents[1]
    bridge = (
        package_root / "kobot_lane_drive" / "fast_lane_drive_bridge_node.py"
    ).read_text(encoding="utf-8")
    launch = (
        package_root / "launch" / "fast_lane_track_test.launch.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("armed", False)' in bridge
    assert 'default_value="false"' in launch
    assert "IfCondition(armed)" in launch
    assert 'self.create_publisher(Float32MultiArray, "/xycar_motor", 10)' in bridge


def test_white_left_drive_bridge_requires_explicit_arming():
    package_root = Path(__file__).parents[1]
    bridge = (
        package_root / "kobot_lane_drive" / "white_left_drive_bridge_node.py"
    ).read_text(encoding="utf-8")
    launch = (
        package_root / "launch" / "white_left_record_drive.launch.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("armed", False)' in bridge
    assert 'default_value="false"' in launch
    assert "IfCondition(armed)" in launch
    assert 'self.create_publisher(\n            Float32MultiArray,\n            "/xycar_motor"' in bridge


def test_white_left_steering_uses_verified_angle_multiplier_and_limit():
    package_root = Path(__file__).parents[1]
    node = (
        package_root / "kobot_lane_drive" / "white_left_drive_node.py"
    ).read_text(encoding="utf-8")
    bridge = (
        package_root / "kobot_lane_drive" / "white_left_drive_bridge_node.py"
    ).read_text(encoding="utf-8")
    config = (package_root / "config" / "white_left_drive.yaml").read_text(
        encoding="utf-8"
    )

    assert 'declare_parameter("steering_kp", 0.42)' in node
    assert 'declare_parameter("steering_output_gain", 4.0)' in node
    assert 'declare_parameter("single_lane_distance_ratio", 180.0 / 640.0)' in node
    assert '* float(self.get_parameter("steering_output_gain").value)' in node
    assert 'declare_parameter("max_steering_rad", 0.476)' in node
    assert 'declare_parameter("max_steering_rad", 0.476)' in bridge
    assert "steering_kp: 0.42" in config
    assert "steering_output_gain: 4.0" in config
    assert "single_lane_distance_ratio: 0.28125" in config
    assert config.count("max_steering_rad: 0.476") == 2


def test_white_lane_control_uses_both_sides_and_stops_when_lost():
    package_root = Path(__file__).parents[1]
    node = (
        package_root / "kobot_lane_drive" / "white_left_drive_node.py"
    ).read_text(encoding="utf-8")
    bridge = (
        package_root / "kobot_lane_drive" / "white_left_drive_bridge_node.py"
    ).read_text(encoding="utf-8")

    assert "select_lane_control" in node
    assert 'payload["used_lane"] = used_lane' in node
    assert '"mode": "HOLD"' in node
    assert '"reason": "both_lanes_lost_hold_last"' in node
    assert 'READY_MODES = {"TRACKING", "HOLD"}' in bridge
    assert 'maximum_speed = float(self.get_parameter("max_speed_mps").value)' in bridge

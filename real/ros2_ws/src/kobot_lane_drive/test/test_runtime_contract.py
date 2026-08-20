"""Cone planner/bridge launch 계약을 ROS 없이 검증한다."""

import ast
import math
from pathlib import Path

import pytest
import yaml

from kobot_lane_drive.drive_command import limit_forward_speed


@pytest.mark.parametrize(
    ("planned", "maximum", "expected"),
    [
        (0.80, 0.55, 0.55),  # straight profile is capped
        (0.70, 0.55, 0.55),  # corner profile is capped too
        (0.50, 0.55, 0.50),  # slower fallback ratio is preserved
        (-0.20, 0.80, 0.0),  # cone drive cannot request reverse
        (0.80, -1.0, 0.0),
        (math.nan, 0.80, 0.0),
        (0.80, math.inf, 0.0),
    ],
)
def test_forward_speed_contract(planned, maximum, expected):
    assert limit_forward_speed(planned, maximum) == pytest.approx(expected)


def _source(package_root: Path, relative: str) -> str:
    return (package_root / relative).read_text(encoding="utf-8")


def _declared_parameters(source: str) -> set[str]:
    return {
        node.args[0].value
        for node in ast.walk(ast.parse(source))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and isinstance(node.args[0].value, str)
    }


def test_launch_passes_one_cap_to_planner_and_bridge(package_root):
    launch_source = _source(package_root, "launch/cone_corridor_drive.launch.py")
    tree = ast.parse(launch_source)

    # One LaunchConfiguration is deliberately reused by both Node dictionaries.
    assignments = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "max_speed_mps"
            for target in node.targets
        )
    ]
    assert len(assignments) == 1

    max_speed_keys = [
        key.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Dict)
        for key in node.keys
        if isinstance(key, ast.Constant) and key.value == "max_speed_mps"
    ]
    assert max_speed_keys == ["max_speed_mps", "max_speed_mps"]
    assert 'default_value="false"' in launch_source
    assert 'LaunchConfiguration("config_file")' in launch_source
    assert '"cone_corridor_robust.yaml"' in launch_source
    assert '"CONE_RUNTIME profile="' in launch_source


def test_planner_status_reports_the_capped_preview_speed(package_root):
    node_source = _source(
        package_root, "kobot_lane_drive/cone_corridor_drive_node.py"
    )
    assert 'declare_parameter("max_speed_mps", 1.50)' in node_source
    assert "cmd_speed = limit_forward_speed(planned_speed, maximum_speed)" in node_source
    assert '"speed_mps": round(cmd_speed, 2)' in node_source
    assert '"planned_speed_mps": round(planned_speed, 2)' in node_source
    assert "cmd_speed," in node_source  # preview payload uses the same value


def test_cone_bridge_yaml_section_matches_launch_node_name(package_root):
    launch_source = _source(package_root, "launch/cone_corridor_drive.launch.py")
    yaml_source = _source(package_root, "config/twinmixing_center.yaml")

    assert 'name="cone_corridor_drive_bridge"' in launch_source
    assert "\ncone_corridor_drive_bridge:\n  ros__parameters:\n" in yaml_source
    assert "\ncone_corridor_drive:\n  ros__parameters:\n" in yaml_source


def test_auto_enable_is_opt_in_in_node_yaml_and_launch(package_root):
    node_source = _source(
        package_root, "kobot_lane_drive/cone_corridor_drive_node.py"
    )
    yaml_source = _source(package_root, "config/twinmixing_center.yaml")
    launch_source = _source(package_root, "launch/cone_corridor_drive.launch.py")

    assert 'declare_parameter("auto_enable", False)' in node_source
    assert "    auto_enable: false" in yaml_source
    assert 'auto_enable = LaunchConfiguration("auto_enable")' in launch_source
    assert 'default_value="false"' in launch_source


def test_robust_profile_is_complete_and_self_contained(package_root):
    profile = yaml.safe_load(
        _source(package_root, "config/cone_corridor_robust.yaml")
    )
    planner = profile["cone_corridor_drive"]["ros__parameters"]
    bridge = profile["cone_corridor_drive_bridge"]["ros__parameters"]

    planner_declarations = _declared_parameters(
        _source(package_root, "kobot_lane_drive/cone_corridor_drive_node.py")
    )
    bridge_declarations = _declared_parameters(
        _source(
            package_root,
            "kobot_lane_drive/twinmixing_center_drive_bridge_node.py",
        )
    )
    assert set(planner) == planner_declarations
    assert set(bridge) == bridge_declarations

    assert planner["cone_drive_speed_mps"] == pytest.approx(1.50)
    assert planner["cone_corner_speed_mps"] == pytest.approx(1.00)
    assert planner["cone_fallback_speed_mps"] == pytest.approx(1.00)
    assert planner["cone_path_hold_speed_mps"] == pytest.approx(1.00)
    assert planner["cone_path_sharp_hold_speed_mps"] == pytest.approx(1.00)
    assert planner["max_speed_mps"] == pytest.approx(1.50)
    assert planner["auto_enable"] is False
    assert bridge["max_speed_mps"] == pytest.approx(1.50)
    assert bridge["armed"] is False


def test_setup_installs_the_robust_profile(package_root):
    setup_source = _source(package_root, "setup.py")
    assert '"config/cone_corridor_robust.yaml"' in setup_source


def test_enable_heartbeat_lease_is_opt_in_and_harness_enables_it(package_root):
    launch_source = _source(package_root, "launch/cone_corridor_drive.launch.py")
    bridge_source = _source(
        package_root, "kobot_lane_drive/twinmixing_center_drive_bridge_node.py"
    )
    harness_source = _source(package_root, "scripts/cone_active_run.py")
    profile = yaml.safe_load(
        _source(package_root, "config/cone_corridor_robust.yaml")
    )
    bridge = profile["cone_corridor_drive_bridge"]["ros__parameters"]

    assert 'declare_parameter("enable_heartbeat_timeout_sec", 0.0)' in bridge_source
    assert "run_enabled = self._enable_lease.active(now)" in bridge_source
    assert "self._status_ready and run_enabled" in bridge_source
    assert bridge["enable_heartbeat_timeout_sec"] == pytest.approx(0.0)
    assert 'LaunchConfiguration("enable_heartbeat_timeout_sec")' in launch_source
    assert 'default_value="0.0"' in launch_source
    assert "HARNESS_ENABLE_HEARTBEAT_TIMEOUT_SEC = 0.30" in harness_source
    assert '"enable_heartbeat_timeout_sec:="' in harness_source


def test_harness_owns_signals_and_requires_tracking_streak(package_root):
    harness_source = _source(package_root, "scripts/cone_active_run.py")

    init_at = harness_source.index(
        "rclpy.init(signal_handler_options=SignalHandlerOptions.NO)"
    )
    sigint_at = harness_source.index(
        "signal.signal(signal.SIGINT, interrupt_handler)"
    )
    assert init_at < sigint_at
    assert 'signal.signal(signal.SIGHUP, interrupt_handler)' in harness_source
    assert "TRACKING_READY_SEC = 0.5" in harness_source
    assert "TRACKING_STATUS_MAX_GAP_SEC = 0.25" in harness_source
    assert 'status.get("mode") != "TRACKING"' in harness_source
    assert 'status.get("motor_output") is not False' in harness_source
    assert 'parsed.get("motor_output") is False' in harness_source
    assert "tracking_since_mono_ns" in harness_source


def test_harness_progressively_pins_command_endpoints_before_ready(package_root):
    harness_source = _source(package_root, "scripts/cone_active_run.py")

    assert "start_progressive_command_guard" in harness_source
    assert "assert_progressive_command_endpoints" in harness_source
    assert '"reason": "gid_replaced"' in harness_source
    assert '"reason": "duplicate"' in harness_source
    assert '"reason": "external_node"' in harness_source
    assert '"reason": "invalid_gid"' in harness_source
    assert "(STATUS_TOPIC, \"/cone_corridor_drive\"): STRING_TYPE" in harness_source
    assert "(PREVIEW_TOPIC, \"/cone_corridor_drive\"): ARRAY_TYPE" in harness_source
    assert "progressive_command_endpoints_complete" in harness_source


def test_harness_pins_bridge_enable_subscription_without_blocking_bag(package_root):
    harness_source = _source(package_root, "scripts/cone_active_run.py")

    assert "assert_or_pin_bridge_enable_subscription" in harness_source
    assert 'identity.node == bridge_node' in harness_source
    assert '"bridge_enable_subscription_gid_replaced"' in harness_source
    assert '"bridge_enable_subscription_disappeared"' in harness_source
    assert "allow_missing_before_pin=False" in harness_source

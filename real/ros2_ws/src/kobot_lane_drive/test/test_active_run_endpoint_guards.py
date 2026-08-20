"""Behavioral tests for the ROS-graph guards without requiring ROS locally."""

from collections import Counter
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import time

import pytest


def _load_harness(package_root: Path, monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy.spin_once = lambda *_args, **_kwargs: None
    node_module = ModuleType("rclpy.node")
    node_module.Node = type("Node", (), {})
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = SimpleNamespace(TRANSIENT_LOCAL=1)
    qos_module.QoSProfile = type("QoSProfile", (), {})
    qos_module.ReliabilityPolicy = SimpleNamespace(RELIABLE=1)
    signals_module = ModuleType("rclpy.signals")
    signals_module.SignalHandlerOptions = SimpleNamespace(NO=0)
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = type("Bool", (), {})
    std_msgs_msg.Float32MultiArray = type("Float32MultiArray", (), {})
    std_msgs_msg.String = type("String", (), {})
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.qos": qos_module,
        "rclpy.signals": signals_module,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = f"cone_active_run_test_{id(monkeypatch)}"
    path = package_root / "scripts/cone_active_run.py"
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, module_name, module)
    spec.loader.exec_module(module)
    return module


def _fake_recorder(harness, publishers=None, subscriptions=None):
    recorder = object.__new__(harness.RunRecorder)
    recorder.progressive_command_expectations = None
    recorder.progressive_command_pins = None
    recorder.command_publisher_whitelist = None
    recorder.bridge_enable_subscription_pin = None
    recorder._enable_publisher = object()
    recorder.stop_requested = False
    recorder.stop_requested_mono_ns = None
    recorder.first_zero_after_stop_mono_ns = None
    recorder.get_fully_qualified_name = lambda: "/test_recorder"
    recorder.event = lambda *_args, **_kwargs: None
    false_transitions = []
    recorder.set_enabled = (
        lambda enabled, event_name: false_transitions.append((enabled, event_name))
    )
    publishers = publishers or {}
    subscriptions = subscriptions or {}
    recorder.publisher_counter = lambda topic: Counter(publishers.get(topic, {}))
    recorder.subscription_counter = (
        lambda topic: Counter(subscriptions.get(topic, {}))
    )
    return recorder, false_transitions


def _identity(harness, node, gid, topic_type):
    return harness.EndpointIdentity(node=node, gid=gid, topic_type=topic_type)


def test_progressive_and_exact_guards_pin_status_preview_and_fail_closed(
    package_root, monkeypatch
):
    harness = _load_harness(package_root, monkeypatch)
    recorder_enable = _identity(
        harness, "/test_recorder", "01", harness.BOOL_TYPE
    )
    planner_enable = _identity(
        harness, "/cone_corridor_drive", "02", harness.BOOL_TYPE
    )
    bridge_motor = _identity(
        harness, "/cone_corridor_drive_bridge", "03", harness.ARRAY_TYPE
    )
    planner_status = _identity(
        harness, "/cone_corridor_drive", "04", harness.STRING_TYPE
    )
    planner_preview = _identity(
        harness, "/cone_corridor_drive", "05", harness.ARRAY_TYPE
    )
    publishers = {
        harness.ENABLE_TOPIC: Counter({recorder_enable: 1}),
        harness.MOTOR_TOPIC: Counter(),
        harness.STATUS_TOPIC: Counter(),
        harness.PREVIEW_TOPIC: Counter(),
    }
    recorder, false_transitions = _fake_recorder(harness, publishers=publishers)
    recorder.start_progressive_command_guard()

    publishers[harness.ENABLE_TOPIC][planner_enable] = 1
    publishers[harness.MOTOR_TOPIC][bridge_motor] = 1
    publishers[harness.STATUS_TOPIC][planner_status] = 1
    publishers[harness.PREVIEW_TOPIC][planner_preview] = 1
    recorder.assert_progressive_command_endpoints()
    assert recorder.progressive_command_endpoints_complete()
    recorder.capture_command_publisher_whitelist()
    assert set(recorder.command_publisher_whitelist) == {
        harness.ENABLE_TOPIC,
        harness.MOTOR_TOPIC,
        harness.STATUS_TOPIC,
        harness.PREVIEW_TOPIC,
    }

    replacement = _identity(
        harness, "/cone_corridor_drive", "99", harness.STRING_TYPE
    )
    publishers[harness.STATUS_TOPIC] = Counter({replacement: 1})
    with pytest.raises(RuntimeError, match="endpoint violation"):
        recorder.assert_command_publisher_whitelist()
    assert false_transitions[-1][0] is False


@pytest.mark.parametrize("gid", ["", "00000000"])
def test_progressive_guard_rejects_empty_or_all_zero_gid(
    package_root, monkeypatch, gid
):
    harness = _load_harness(package_root, monkeypatch)
    invalid = _identity(harness, "/test_recorder", gid, harness.BOOL_TYPE)
    publishers = {harness.ENABLE_TOPIC: Counter({invalid: 1})}
    recorder, false_transitions = _fake_recorder(harness, publishers=publishers)
    with pytest.raises(RuntimeError, match="invalid_gid"):
        recorder.start_progressive_command_guard()
    assert false_transitions[-1][0] is False


def test_bridge_subscription_pin_ignores_bag_but_rejects_replacement(
    package_root, monkeypatch
):
    harness = _load_harness(package_root, monkeypatch)
    bridge = _identity(
        harness, "/cone_corridor_drive_bridge", "11", harness.BOOL_TYPE
    )
    bag = _identity(harness, "/rosbag2_recorder", "12", harness.BOOL_TYPE)
    subscriptions = {harness.ENABLE_TOPIC: Counter({bridge: 1, bag: 1})}
    recorder, false_transitions = _fake_recorder(
        harness, subscriptions=subscriptions
    )
    assert recorder.assert_or_pin_bridge_enable_subscription(
        allow_missing_before_pin=True
    )
    assert recorder.bridge_enable_subscription_pin == bridge
    assert recorder.assert_or_pin_bridge_enable_subscription(
        allow_missing_before_pin=False
    )

    replacement = _identity(
        harness, "/cone_corridor_drive_bridge", "13", harness.BOOL_TYPE
    )
    subscriptions[harness.ENABLE_TOPIC] = Counter({replacement: 1, bag: 1})
    with pytest.raises(RuntimeError, match="endpoint violation"):
        recorder.assert_or_pin_bridge_enable_subscription(
            allow_missing_before_pin=False
        )
    assert false_transitions[-1][0] is False


def test_tracking_ready_requires_explicit_motor_output_false(
    package_root, monkeypatch
):
    harness = _load_harness(package_root, monkeypatch)
    recorder, _false_transitions = _fake_recorder(harness)
    now_ns = time.monotonic_ns()
    recorder.latest_status_mono_ns = now_ns
    recorder.latest_preview_mono_ns = now_ns
    recorder.tracking_since_mono_ns = now_ns - int(0.6e9)
    recorder.latest_status = {
        "mode": "TRACKING",
        "found": True,
        "motor_output": False,
    }
    assert recorder.tracking_ready(now_ns - int(1e9))

    recorder.latest_status["motor_output"] = True
    assert not recorder.tracking_ready(now_ns - int(1e9))

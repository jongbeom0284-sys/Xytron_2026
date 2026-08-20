#!/usr/bin/env python3
"""Record one explicitly enabled cone-corridor vehicle run.

The launch process is allowed to finish discovery while the enable gate keeps
publishing ``false``.  Only after the planner, bridge, fresh tracking status,
and rosbag recorder are ready does the measured active interval begin.

This script never searches for or terminates sensor, dashboard, VESC, or motor
stack processes.  It refuses conflicting command publishers and only signals
the launch/rosbag process groups that it created itself.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
from typing import Any, Iterable, TextIO

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float32MultiArray, String


ENABLE_TOPIC = "/kobot_lane_drive/record_drive/enabled"
STATUS_TOPIC = "/kobot_lane_drive/cone_corridor/status"
PREVIEW_TOPIC = "/kobot_lane_drive/cone_corridor/drive_preview"
MOTOR_TOPIC = "/xycar_motor"
BOOL_TYPE = "std_msgs/msg/Bool"
ARRAY_TYPE = "std_msgs/msg/Float32MultiArray"
STRING_TYPE = "std_msgs/msg/String"
EXPECTED_NODES = {"/cone_corridor_drive", "/cone_corridor_drive_bridge"}
TRACKING_READY_SEC = 0.5
TRACKING_STATUS_MAX_GAP_SEC = 0.25
HARNESS_ENABLE_HEARTBEAT_TIMEOUT_SEC = 0.30
BAG_TOPICS = (
    "/scan",
    STATUS_TOPIC,
    PREVIEW_TOPIC,
    ENABLE_TOPIC,
    MOTOR_TOPIC,
    "/sensors/core",
)


def iso_now() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def endpoint_name(endpoint: Any) -> str:
    namespace = str(getattr(endpoint, "node_namespace", "/") or "/")
    name = str(getattr(endpoint, "node_name", "?") or "?")
    return f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"


def endpoint_gid(endpoint: Any) -> str:
    """Return a stable printable DDS endpoint identity on Humble RMWs."""

    raw_gid = getattr(endpoint, "endpoint_gid", b"")
    try:
        return bytes(raw_gid).hex()
    except (TypeError, ValueError):
        return repr(raw_gid)


@dataclass(frozen=True, order=True)
class EndpointIdentity:
    node: str
    gid: str
    topic_type: str

    @classmethod
    def from_info(cls, endpoint: Any) -> "EndpointIdentity":
        return cls(
            node=endpoint_name(endpoint),
            gid=endpoint_gid(endpoint),
            topic_type=str(getattr(endpoint, "topic_type", "")),
        )

    def label(self) -> str:
        short_gid = self.gid[:16] if self.gid else "no-gid"
        return f"{self.node}[{self.topic_type};gid={short_gid}]"

    def has_valid_gid(self) -> bool:
        return bool(self.gid) and any(character != "0" for character in self.gid)


EndpointCounter = Counter[EndpointIdentity]


@dataclass
class ManagedProcess:
    """A child session whose process group was created by this script."""

    process: subprocess.Popen[str]
    pgid: int
    label: str


def positive_float(raw: str) -> float:
    value = float(raw)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return value


class RunRecorder(Node):
    """Persistent false/true/false gate plus human-readable JSONL loggers."""

    def __init__(self, run_dir: Path) -> None:
        super().__init__(f"cone_active_run_recorder_{os.getpid()}")
        self.run_dir = run_dir
        self._event_file = (run_dir / "events.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self._status_file = (run_dir / "status.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self._preview_file = (run_dir / "preview.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )
        self._motor_file = (run_dir / "motor.jsonl").open(
            "w", encoding="utf-8", buffering=1
        )

        self.latest_status: dict[str, Any] | None = None
        self.latest_status_mono_ns: int | None = None
        self.tracking_since_mono_ns: int | None = None
        self.latest_preview_mono_ns: int | None = None
        self.latest_motor: list[float] | None = None
        self.latest_motor_mono_ns: int | None = None
        self.first_nonzero_motor_mono_ns: int | None = None
        self.last_nonzero_motor_mono_ns: int | None = None
        self.first_zero_after_stop_mono_ns: int | None = None
        self.stop_requested_mono_ns: int | None = None
        self.nonzero_motor_messages = 0
        self.stop_requested = False

        self.command_publisher_whitelist: dict[str, EndpointCounter] | None = None
        self.progressive_command_expectations: dict[
            tuple[str, str], str
        ] | None = None
        self.progressive_command_pins: dict[
            tuple[str, str], EndpointIdentity | None
        ] | None = None
        self.bag_subscription_whitelist: dict[str, EndpointCounter] | None = None
        self.bag_node_name: str | None = None
        self.bridge_enable_subscription_pin: EndpointIdentity | None = None

        self.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(
            Float32MultiArray, PREVIEW_TOPIC, self._on_preview, 10
        )
        self.create_subscription(Float32MultiArray, MOTOR_TOPIC, self._on_motor, 10)

        self._enable_publisher = None
        self._enable_value = False
        self._last_enable_publish_ns = 0

    def start_enable_gate(self) -> None:
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._enable_publisher = self.create_publisher(Bool, ENABLE_TOPIC, qos)
        self.set_enabled(False, event_name="enable_gate_started_false")

    def set_enabled(self, enabled: bool, event_name: str) -> None:
        if self._enable_publisher is None:
            raise RuntimeError("enable gate publisher has not been started")
        self._enable_value = bool(enabled)
        # A short burst makes the transition prompt without spawning a new ROS
        # CLI process.  Periodic publishing continues in spin_for().
        message = Bool(data=self._enable_value)
        for _ in range(3):
            self._enable_publisher.publish(message)
        self._last_enable_publish_ns = time.monotonic_ns()
        self.event(event_name, enabled=self._enable_value)

    def publish_enable_if_due(self, period_sec: float = 0.05) -> None:
        if self._enable_publisher is None:
            return
        now_ns = time.monotonic_ns()
        if now_ns - self._last_enable_publish_ns < int(period_sec * 1e9):
            return
        self._enable_publisher.publish(Bool(data=self._enable_value))
        self._last_enable_publish_ns = now_ns

    def spin_once_gated(self, timeout_sec: float = 0.02) -> None:
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        self.publish_enable_if_due()

    def spin_for(self, duration_sec: float) -> None:
        deadline_ns = time.monotonic_ns() + int(max(0.0, duration_sec) * 1e9)
        while time.monotonic_ns() < deadline_ns:
            remaining_sec = (deadline_ns - time.monotonic_ns()) / 1e9
            self.spin_once_gated(min(0.02, max(0.0, remaining_sec)))

    def graph_nodes(self) -> set[str]:
        result: set[str] = set()
        for name, namespace in self.get_node_names_and_namespaces():
            if namespace == "/":
                result.add(f"/{name}")
            else:
                result.add(f"{namespace.rstrip('/')}/{name}")
        return result

    def publisher_names(self, topic: str) -> list[str]:
        return sorted(
            {endpoint_name(info) for info in self.get_publishers_info_by_topic(topic)}
        )

    @staticmethod
    def endpoint_labels(counter: EndpointCounter) -> list[str]:
        labels: list[str] = []
        for identity, count in sorted(counter.items()):
            labels.extend([identity.label()] * count)
        return labels

    def publisher_counter(self, topic: str) -> EndpointCounter:
        return Counter(
            EndpointIdentity.from_info(info)
            for info in self.get_publishers_info_by_topic(topic)
        )

    def subscription_counter(self, topic: str) -> EndpointCounter:
        return Counter(
            EndpointIdentity.from_info(info)
            for info in self.get_subscriptions_info_by_topic(topic)
        )

    def assert_or_pin_bridge_enable_subscription(
        self, *, allow_missing_before_pin: bool
    ) -> bool:
        """Pin the bridge's enable subscription and keep its DDS identity.

        Rosbag later adds another subscriber to this topic, so this guard is
        deliberately scoped to the bridge node instead of asserting a global
        subscription count.  Once the bridge endpoint has appeared, an empty
        GID, disappearance, duplicate, replacement, or type change fails
        closed through the persistent enable publisher.
        """

        bridge_node = "/cone_corridor_drive_bridge"
        matching = Counter(
            {
                identity: count
                for identity, count in self.subscription_counter(
                    ENABLE_TOPIC
                ).items()
                if identity.node == bridge_node
            }
        )
        total = sum(matching.values())
        violations: list[dict[str, Any]] = []
        if total == 0:
            if self.bridge_enable_subscription_pin is not None:
                violations.append(
                    {
                        "reason": "bridge_enable_subscription_disappeared",
                        "expected": self.bridge_enable_subscription_pin.label(),
                    }
                )
            elif not allow_missing_before_pin:
                violations.append(
                    {
                        "reason": "bridge_enable_subscription_missing",
                        "node": bridge_node,
                    }
                )
            if violations:
                self._fail_command_endpoints(
                    "bridge_enable_subscription_violation", violations
                )
            return False
        if total != 1:
            violations.append(
                {
                    "reason": "bridge_enable_subscription_duplicate",
                    "endpoints": self.endpoint_labels(matching),
                }
            )
        identity = next(iter(matching))
        if identity.topic_type != BOOL_TYPE:
            violations.append(
                {
                    "reason": "bridge_enable_subscription_type_mismatch",
                    "endpoint": identity.label(),
                    "expected_type": BOOL_TYPE,
                }
            )
        if not identity.has_valid_gid():
            violations.append(
                {
                    "reason": "bridge_enable_subscription_invalid_gid",
                    "endpoint": identity.label(),
                }
            )
        pinned = self.bridge_enable_subscription_pin
        if pinned is not None and identity != pinned:
            violations.append(
                {
                    "reason": "bridge_enable_subscription_gid_replaced",
                    "expected": pinned.label(),
                    "current": identity.label(),
                }
            )
        if violations:
            self._fail_command_endpoints(
                "bridge_enable_subscription_violation", violations
            )
        if pinned is None:
            self.bridge_enable_subscription_pin = identity
            self.event(
                "bridge_enable_subscription_pinned", endpoint=identity.label()
            )
        return True

    def _fail_command_endpoints(self, event_name: str, details: Any) -> None:
        self.event(event_name, differences=details)
        if self._enable_publisher is not None:
            self.stop_requested = True
            self.stop_requested_mono_ns = time.monotonic_ns()
            self.first_zero_after_stop_mono_ns = None
            self.set_enabled(False, event_name="endpoint_violation_enable_false")
        raise RuntimeError(
            f"command publisher endpoint violation: "
            + json.dumps(details, ensure_ascii=False)
        )

    def start_progressive_command_guard(self) -> None:
        """Allow only expected planner/bridge/recorder publishers and pin GIDs."""

        recorder_name = self.get_fully_qualified_name()
        self.progressive_command_expectations = {
            (ENABLE_TOPIC, recorder_name): BOOL_TYPE,
            (ENABLE_TOPIC, "/cone_corridor_drive"): BOOL_TYPE,
            (MOTOR_TOPIC, "/cone_corridor_drive_bridge"): ARRAY_TYPE,
            (STATUS_TOPIC, "/cone_corridor_drive"): STRING_TYPE,
            (PREVIEW_TOPIC, "/cone_corridor_drive"): ARRAY_TYPE,
        }
        self.progressive_command_pins = {
            key: None for key in self.progressive_command_expectations
        }
        self.assert_progressive_command_endpoints()

    def assert_progressive_command_endpoints(self) -> None:
        """Monitor the preflight-to-ready 0->1 endpoint transition.

        An expected endpoint may be absent until its node starts.  Its first
        valid DDS GID is pinned.  From then on disappearance, replacement,
        duplicate endpoints, external node names, and type mismatches all fail
        closed.  This closes the gap before the exact runtime whitelist exists.
        """

        expectations = self.progressive_command_expectations
        pins = self.progressive_command_pins
        if expectations is None or pins is None:
            raise RuntimeError("progressive command endpoint guard is not initialized")

        guarded_topics = {topic for topic, _node_name in expectations}
        current = {
            topic: self.publisher_counter(topic) for topic in guarded_topics
        }
        next_pins = dict(pins)
        violations: list[dict[str, Any]] = []

        for topic, endpoints in current.items():
            for identity, count in endpoints.items():
                if not identity.has_valid_gid():
                    violations.append(
                        {
                            "reason": "invalid_gid",
                            "topic": topic,
                            "endpoint": identity.label(),
                            "count": count,
                        }
                    )
                    continue
                expected_type = expectations.get((topic, identity.node))
                if expected_type is None:
                    violations.append(
                        {
                            "reason": "external_node",
                            "topic": topic,
                            "endpoint": identity.label(),
                            "count": count,
                        }
                    )
                elif identity.topic_type != expected_type:
                    violations.append(
                        {
                            "reason": "type_mismatch",
                            "topic": topic,
                            "endpoint": identity.label(),
                            "expected_type": expected_type,
                        }
                    )

        for key, expected_type in expectations.items():
            topic, node_name = key
            matching = Counter(
                {
                    identity: count
                    for identity, count in current[topic].items()
                    if identity.node == node_name
                }
            )
            total = sum(matching.values())
            pinned = pins[key]
            if total > 1:
                violations.append(
                    {
                        "reason": "duplicate",
                        "topic": topic,
                        "node": node_name,
                        "endpoints": self.endpoint_labels(matching),
                    }
                )
                continue
            if total == 0:
                if pinned is not None:
                    violations.append(
                        {
                            "reason": "disappeared_after_pin",
                            "topic": topic,
                            "endpoint": pinned.label(),
                        }
                    )
                continue

            identity = next(iter(matching))
            if identity.topic_type != expected_type:
                continue  # type_mismatch was already recorded above
            if pinned is None:
                next_pins[key] = identity
            elif identity != pinned:
                violations.append(
                    {
                        "reason": "gid_replaced",
                        "topic": topic,
                        "expected": pinned.label(),
                        "current": identity.label(),
                    }
                )

        if violations:
            self._fail_command_endpoints(
                "progressive_command_endpoint_violation", violations
            )

        newly_pinned = [
            identity.label()
            for key, identity in next_pins.items()
            if identity is not None and pins[key] is None
        ]
        self.progressive_command_pins = next_pins
        if newly_pinned:
            self.event("progressive_command_endpoints_pinned", endpoints=newly_pinned)

    def progressive_endpoint_pinned(self, topic: str, node_name: str) -> bool:
        pins = self.progressive_command_pins or {}
        return pins.get((topic, node_name)) is not None

    def progressive_command_endpoints_complete(self) -> bool:
        pins = self.progressive_command_pins
        return pins is not None and all(identity is not None for identity in pins.values())

    def capture_command_publisher_whitelist(self) -> None:
        """Freeze the fully appeared progressive endpoints as an exact set."""

        self.assert_progressive_command_endpoints()
        if not self.progressive_command_endpoints_complete():
            missing = [
                {"topic": topic, "node": node_name}
                for (topic, node_name), identity in (
                    self.progressive_command_pins or {}
                ).items()
                if identity is None
            ]
            raise RuntimeError(f"command endpoints not complete: {missing}")
        pinned_by_topic: dict[str, EndpointCounter] = {
            topic: Counter()
            for topic, _node_name in (self.progressive_command_pins or {})
        }
        for (topic, _node_name), identity in (
            self.progressive_command_pins or {}
        ).items():
            if identity is not None:
                pinned_by_topic[topic][identity] += 1
        current = {
            topic: self.publisher_counter(topic) for topic in pinned_by_topic
        }
        if current != pinned_by_topic:
            self._fail_command_endpoints(
                "command_endpoint_capture_mismatch",
                {
                    topic: {
                        "pinned": self.endpoint_labels(pinned_by_topic[topic]),
                        "current": self.endpoint_labels(current[topic]),
                    }
                    for topic in pinned_by_topic
                    if current[topic] != pinned_by_topic[topic]
                },
            )
        self.command_publisher_whitelist = pinned_by_topic
        self.event(
            "command_endpoint_whitelist_captured",
            publishers={
                topic: self.endpoint_labels(counter)
                for topic, counter in pinned_by_topic.items()
            },
        )

    def assert_command_publisher_whitelist(self, fail_closed: bool = True) -> None:
        expected = self.command_publisher_whitelist
        if expected is None:
            raise RuntimeError("command publisher whitelist has not been captured")
        current = {
            topic: self.publisher_counter(topic) for topic in expected
        }
        if current == expected:
            return
        details = {
            topic: {
                "expected": self.endpoint_labels(expected[topic]),
                "current": self.endpoint_labels(current[topic]),
            }
            for topic in expected
            if current[topic] != expected[topic]
        }
        if fail_closed and self._enable_publisher is not None:
            self._fail_command_endpoints("command_endpoint_violation", details)
        self.event("command_endpoint_violation", differences=details)
        raise RuntimeError(
            "command publisher endpoint whitelist changed: "
            + json.dumps(details, ensure_ascii=False)
        )

    def discover_bag_subscriptions(
        self,
        baseline: dict[str, EndpointCounter],
    ) -> bool:
        """Capture one newly appearing subscriber node on every bag topic."""

        newly_seen: dict[str, EndpointCounter] = {}
        common_nodes: set[str] | None = None
        for topic in BAG_TOPICS:
            new_endpoints = self.subscription_counter(topic) - baseline[topic]
            newly_seen[topic] = new_endpoints
            nodes = {identity.node for identity in new_endpoints.elements()}
            common_nodes = nodes if common_nodes is None else common_nodes & nodes
        candidates = sorted(common_nodes or set())
        rosbag_candidates = [name for name in candidates if "rosbag" in name.lower()]
        if len(rosbag_candidates) == 1:
            bag_node = rosbag_candidates[0]
        elif len(candidates) == 1:
            bag_node = candidates[0]
        else:
            return False

        captured: dict[str, EndpointCounter] = {}
        for topic, endpoints in newly_seen.items():
            selected = Counter(
                {
                    identity: count
                    for identity, count in endpoints.items()
                    if identity.node == bag_node
                }
            )
            if not selected:
                return False
            captured[topic] = selected
        self.bag_node_name = bag_node
        self.bag_subscription_whitelist = captured
        self.event(
            "bag_subscription_handshake_complete",
            bag_node=bag_node,
            subscriptions={
                topic: self.endpoint_labels(counter)
                for topic, counter in captured.items()
            },
        )
        return True

    def assert_bag_subscriptions_present(self) -> None:
        expected = self.bag_subscription_whitelist
        if expected is None:
            raise RuntimeError("bag subscription handshake has not completed")
        missing: dict[str, list[str]] = {}
        for topic, required in expected.items():
            current = self.subscription_counter(topic)
            absent = required - current
            if absent:
                missing[topic] = self.endpoint_labels(absent)
        if missing:
            raise RuntimeError(
                "rosbag subscription endpoint disappeared: "
                + json.dumps(missing, ensure_ascii=False)
            )

    def tracking_ready(self, not_before_ns: int) -> bool:
        if self.latest_status_mono_ns is None or self.latest_preview_mono_ns is None:
            return False
        if self.latest_status_mono_ns < not_before_ns:
            return False
        if self.latest_preview_mono_ns < not_before_ns:
            return False
        now_ns = time.monotonic_ns()
        if (
            now_ns - self.latest_status_mono_ns
            > int(TRACKING_STATUS_MAX_GAP_SEC * 1e9)
        ):
            self.tracking_since_mono_ns = None
            return False
        freshness_ns = now_ns - min(
            self.latest_status_mono_ns, self.latest_preview_mono_ns
        )
        if freshness_ns > int(0.5 * 1e9):
            return False
        status = self.latest_status or {}
        if (
            status.get("mode") != "TRACKING"
            or status.get("found") is not True
            or status.get("motor_output") is not False
        ):
            return False
        if self.tracking_since_mono_ns is None:
            return False
        continuous_since_ns = max(self.tracking_since_mono_ns, not_before_ns)
        return (
            now_ns - continuous_since_ns
            >= int(TRACKING_READY_SEC * 1e9)
        )

    def reset_launch_readiness_cache(self) -> None:
        self.latest_status = None
        self.latest_status_mono_ns = None
        self.tracking_since_mono_ns = None
        self.latest_preview_mono_ns = None
        self.event("launch_readiness_cache_reset")

    def event(self, event_name: str, **fields: Any) -> None:
        payload = self.timestamp_fields()
        payload.update({"event": event_name, **fields})
        self._event_file.write(json.dumps(payload, ensure_ascii=False) + "\n")

    def timestamp_fields(self) -> dict[str, Any]:
        return {
            "wall_time": iso_now(),
            "wall_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "ros_time_ns": self.get_clock().now().nanoseconds,
        }

    def _on_status(self, message: String) -> None:
        now_mono_ns = time.monotonic_ns()
        previous_status_ns = self.latest_status_mono_ns
        try:
            parsed: dict[str, Any] = json.loads(message.data)
        except (TypeError, ValueError):
            parsed = {"_invalid_json": message.data}
        self.latest_status = parsed
        self.latest_status_mono_ns = now_mono_ns
        if (
            previous_status_ns is not None
            and now_mono_ns - previous_status_ns
            > int(TRACKING_STATUS_MAX_GAP_SEC * 1e9)
        ):
            self.tracking_since_mono_ns = None
        if (
            parsed.get("mode") == "TRACKING"
            and parsed.get("found") is True
            and parsed.get("motor_output") is False
        ):
            if self.tracking_since_mono_ns is None:
                self.tracking_since_mono_ns = now_mono_ns
        else:
            self.tracking_since_mono_ns = None
        record = self.timestamp_fields()
        record["status"] = parsed
        self._status_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _on_preview(self, message: Float32MultiArray) -> None:
        self.latest_preview_mono_ns = time.monotonic_ns()
        record = self.timestamp_fields()
        record["data"] = [float(value) for value in message.data]
        self._preview_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def _on_motor(self, message: Float32MultiArray) -> None:
        now_mono_ns = time.monotonic_ns()
        values = [float(value) for value in message.data]
        self.latest_motor = values
        self.latest_motor_mono_ns = now_mono_ns
        has_speed = len(values) >= 2
        speed = values[1] if has_speed else float("nan")
        if has_speed and abs(speed) > 1.0e-6:
            self.nonzero_motor_messages += 1
            self.last_nonzero_motor_mono_ns = now_mono_ns
            if self.first_nonzero_motor_mono_ns is None:
                self.first_nonzero_motor_mono_ns = now_mono_ns
                self.event("first_nonzero_motor_command", command=values)
        elif (
            has_speed
            and abs(speed) <= 1.0e-6
            and self.stop_requested
            and self.stop_requested_mono_ns is not None
            and now_mono_ns - self.stop_requested_mono_ns >= int(0.01 * 1e9)
            and self.first_zero_after_stop_mono_ns is None
        ):
            self.first_zero_after_stop_mono_ns = now_mono_ns
            self.event("first_zero_motor_after_stop", command=values)

        record = self.timestamp_fields()
        record["data"] = values
        self._motor_file.write(json.dumps(record, ensure_ascii=False) + "\n")

    def close_logs(self) -> None:
        for stream in (
            self._event_file,
            self._status_file,
            self._preview_file,
            self._motor_file,
        ):
            stream.flush()
            stream.close()


def start_process(
    command: list[str], log_file: TextIO, label: str
) -> ManagedProcess:
    process = subprocess.Popen(
        command,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        pgid = os.getpgid(process.pid)
    except ProcessLookupError:
        # start_new_session=True guarantees pid==pgid while the process exists.
        pgid = process.pid
    if pgid != process.pid:
        process.terminate()
        raise RuntimeError(
            f"refusing unmanaged {label} process group: pid={process.pid}, pgid={pgid}"
        )
    return ManagedProcess(process=process, pgid=pgid, label=label)


def process_group_exists(managed: ManagedProcess) -> bool:
    # poll() reaps an exited leader, but group liveness is deliberately checked
    # independently because ros2 launch children can outlive that leader.
    managed.process.poll()
    try:
        os.killpg(managed.pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def wait_for_process_group_exit(managed: ManagedProcess, timeout_sec: float) -> bool:
    deadline_ns = time.monotonic_ns() + int(timeout_sec * 1e9)
    while time.monotonic_ns() < deadline_ns:
        if not process_group_exists(managed):
            return True
        time.sleep(0.05)
    return not process_group_exists(managed)


def stop_own_process_group(
    managed: ManagedProcess | None,
) -> bool:
    if managed is None:
        return True
    # Signal the recorded PGID even when the ros2 CLI leader has already
    # exited.  Only start_process() can create a ManagedProcess, so broad
    # process discovery and unrelated sensor/VESC groups are never addressed.
    escalation: Iterable[tuple[signal.Signals, float]] = (
        (signal.SIGINT, 6.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 2.0),
    )
    for signum, wait_sec in escalation:
        if not process_group_exists(managed):
            return True
        try:
            os.killpg(managed.pgid, signum)
        except ProcessLookupError:
            return True
        except PermissionError as exc:
            print(
                f"ERROR: cannot signal owned {managed.label} PGID {managed.pgid}: {exc}",
                file=sys.stderr,
            )
            return False
        if wait_for_process_group_exit(managed, wait_sec):
            return True
    print(
        f"ERROR: owned {managed.label} PGID {managed.pgid} survived SIGKILL",
        file=sys.stderr,
    )
    return False


def wait_for_clean_graph(node: RunRecorder, settle_sec: float) -> None:
    node.spin_for(settle_sec)
    existing_nodes = EXPECTED_NODES.intersection(node.graph_nodes())
    motor_publishers = node.publisher_names(MOTOR_TOPIC)
    enable_publishers = node.publisher_names(ENABLE_TOPIC)
    status_publishers = node.publisher_names(STATUS_TOPIC)
    preview_publishers = node.publisher_names(PREVIEW_TOPIC)
    conflicts: list[str] = []
    if existing_nodes:
        conflicts.append("existing cone nodes: " + ", ".join(sorted(existing_nodes)))
    if motor_publishers:
        conflicts.append(
            f"existing {MOTOR_TOPIC} publishers: " + ", ".join(motor_publishers)
        )
    if enable_publishers:
        conflicts.append(
            f"existing {ENABLE_TOPIC} publishers: " + ", ".join(enable_publishers)
        )
    if status_publishers:
        conflicts.append(
            f"existing {STATUS_TOPIC} publishers: " + ", ".join(status_publishers)
        )
    if preview_publishers:
        conflicts.append(
            f"existing {PREVIEW_TOPIC} publishers: " + ", ".join(preview_publishers)
        )
    if conflicts:
        raise RuntimeError(
            "command conflict detected; this script did not terminate anything:\n  - "
            + "\n  - ".join(conflicts)
        )


def wait_for_recorder_endpoint_pin(
    node: RunRecorder,
    timeout_sec: float,
) -> None:
    recorder_name = node.get_fully_qualified_name()
    deadline_ns = time.monotonic_ns() + int(timeout_sec * 1e9)
    while time.monotonic_ns() < deadline_ns:
        node.spin_once_gated()
        node.assert_progressive_command_endpoints()
        if node.progressive_endpoint_pinned(ENABLE_TOPIC, recorder_name):
            node.event(
                "recorder_enable_endpoint_pinned_before_launch",
                node=recorder_name,
            )
            return
    raise TimeoutError("recorder enable endpoint was not discovered before launch")


def wait_for_launch_ready(
    node: RunRecorder,
    launch_process: ManagedProcess,
    launch_started_ns: int,
    timeout_sec: float,
    require_tracking: bool,
) -> None:
    deadline_ns = time.monotonic_ns() + int(timeout_sec * 1e9)
    while time.monotonic_ns() < deadline_ns:
        node.spin_once_gated()
        # During node startup only expected planner/bridge endpoints may make a
        # single 0->1 transition.  Their first GIDs are pinned immediately.
        node.assert_progressive_command_endpoints()
        if launch_process.process.poll() is not None:
            raise RuntimeError(
                "cone launch exited before readiness "
                f"(code={launch_process.process.returncode})"
            )
        nodes_ready = EXPECTED_NODES.issubset(node.graph_nodes())
        bridge_matched = node.assert_or_pin_bridge_enable_subscription(
            allow_missing_before_pin=True
        )
        fresh_messages = (
            node.latest_status_mono_ns is not None
            and node.latest_preview_mono_ns is not None
            and node.latest_status_mono_ns >= launch_started_ns
            and node.latest_preview_mono_ns >= launch_started_ns
            and time.monotonic_ns()
            - min(node.latest_status_mono_ns, node.latest_preview_mono_ns)
            <= int(0.5 * 1e9)
        )
        path_ready = node.tracking_ready(launch_started_ns)
        command_endpoints_ready = node.progressive_command_endpoints_complete()
        if (
            nodes_ready
            and bridge_matched
            and fresh_messages
            and command_endpoints_ready
        ):
            if path_ready or not require_tracking:
                node.event(
                    "launch_ready",
                    nodes=sorted(EXPECTED_NODES),
                    tracking_ready=path_ready,
                    latest_status=node.latest_status,
                )
                return
    status = node.latest_status or {}
    raise TimeoutError(
        "launch readiness timed out while enable stayed false; "
        f"nodes={sorted(node.graph_nodes())}, latest_mode={status.get('mode')}, "
        f"found={status.get('found')}, "
        f"command_endpoints_complete={node.progressive_command_endpoints_complete()}"
    )


def wait_for_bag_ready(
    node: RunRecorder,
    bag_process: ManagedProcess,
    bag_dir: Path,
    subscription_baseline: dict[str, EndpointCounter],
    timeout_sec: float,
) -> None:
    deadline_ns = time.monotonic_ns() + int(timeout_sec * 1e9)
    while time.monotonic_ns() < deadline_ns:
        node.spin_once_gated()
        node.assert_command_publisher_whitelist()
        if bag_process.process.poll() is not None:
            raise RuntimeError(
                "rosbag exited before readiness "
                f"(code={bag_process.process.returncode})"
            )
        output_created = bag_dir.is_dir() and any(bag_dir.iterdir())
        if node.bag_subscription_whitelist is None:
            node.discover_bag_subscriptions(subscription_baseline)
        if output_created and node.bag_subscription_whitelist is not None:
            node.assert_bag_subscriptions_present()
            node.event(
                "bag_ready",
                bag_dir=str(bag_dir),
                graph_handshake=True,
                bag_node=node.bag_node_name,
            )
            return
    raise TimeoutError(
        "rosbag readiness timed out before both output creation and graph "
        "subscription handshake completed"
    )


def assert_runtime_integrity(
    node: RunRecorder,
    launch_process: ManagedProcess,
    bag_process: ManagedProcess,
) -> None:
    if launch_process.process.poll() is not None:
        raise RuntimeError("cone launch exited while the run harness was active")
    if bag_process.process.poll() is not None:
        raise RuntimeError("rosbag exited while the run harness was active")
    node.assert_command_publisher_whitelist()
    node.assert_or_pin_bridge_enable_subscription(
        allow_missing_before_pin=False
    )
    node.assert_bag_subscriptions_present()


def spin_runtime_for(
    node: RunRecorder,
    duration_sec: float,
    launch_process: ManagedProcess,
    bag_process: ManagedProcess,
) -> None:
    deadline_ns = time.monotonic_ns() + int(max(0.0, duration_sec) * 1e9)
    while time.monotonic_ns() < deadline_ns:
        remaining_sec = (deadline_ns - time.monotonic_ns()) / 1e9
        node.spin_once_gated(min(0.02, max(0.0, remaining_sec)))
        assert_runtime_integrity(node, launch_process, bag_process)


def wait_for_operator(
    node: RunRecorder,
    launch_process: ManagedProcess,
    bag_process: ManagedProcess,
) -> None:
    print("READY: vehicle remains disabled. Press ENTER to start; type q then ENTER to abort.")
    while True:
        node.spin_once_gated(0.05)
        assert_runtime_integrity(node, launch_process, bag_process)
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            continue
        line = sys.stdin.readline()
        if line == "":
            raise RuntimeError("stdin closed; use --no-prompt for deliberate automation")
        response = line.strip().lower()
        if response in {"q", "quit", "abort"}:
            raise KeyboardInterrupt
        return


def request_false_and_confirm_motor_zero(
    node: RunRecorder,
    timeout_sec: float,
    event_name: str,
    validate_command_endpoints: bool = False,
) -> tuple[int, int]:
    """Switch the persistent gate false and require a later bridge zero."""

    node.stop_requested = True
    node.first_zero_after_stop_mono_ns = None
    node.stop_requested_mono_ns = time.monotonic_ns()
    requested_ns = node.stop_requested_mono_ns
    node.set_enabled(False, event_name=event_name)
    deadline_ns = requested_ns + int(timeout_sec * 1e9)
    while time.monotonic_ns() < deadline_ns:
        node.spin_once_gated(0.02)
        if validate_command_endpoints:
            node.assert_command_publisher_whitelist()
        confirmed_ns = node.first_zero_after_stop_mono_ns
        if confirmed_ns is not None and confirmed_ns > requested_ns:
            node.event(
                "motor_zero_confirmed",
                delay_sec=(confirmed_ns - requested_ns) / 1e9,
                command=node.latest_motor,
            )
            return requested_ns, confirmed_ns
    raise TimeoutError(
        f"no actual {MOTOR_TOPIC} [*, 0] message within {timeout_sec:.3f}s "
        "after enable=false"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Wait for cone launch and rosbag readiness, then record an exact "
            "false -> true -> false active interval."
        )
    )
    parser.add_argument("--duration", type=positive_float, default=15.0)
    parser.add_argument("--speed", type=positive_float, default=0.55)
    parser.add_argument("--countdown", type=float, default=3.0)
    parser.add_argument("--ready-timeout", type=positive_float, default=30.0)
    parser.add_argument("--stop-timeout", type=positive_float, default=2.0)
    parser.add_argument("--stop-settle", type=positive_float, default=1.0)
    parser.add_argument(
        "--output-root", type=Path, default=Path.home() / "cone_runs"
    )
    parser.add_argument("--config", type=Path)
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="start after countdown without waiting for ENTER",
    )
    parser.add_argument(
        "--allow-not-tracking",
        action="store_true",
        help="allow activation with fresh status/preview even when found is false",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.speed > 2.16:
        raise SystemExit("--speed must not exceed the platform limit of 2.16 m/s")
    if args.countdown < 0.0:
        raise SystemExit("--countdown must be zero or greater")
    if args.config is not None:
        args.config = args.config.expanduser().resolve()
        if not args.config.is_file():
            raise SystemExit(f"config file does not exist: {args.config}")

    stamp = datetime.now().astimezone().strftime("%Y%m%d-%H%M%S")
    run_dir = args.output_root.expanduser().resolve() / f"cone-active-{stamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    bag_dir = run_dir / "bag"

    launch_log = (run_dir / "launch.log").open("w", encoding="utf-8", buffering=1)
    bag_log = (run_dir / "bag.log").open("w", encoding="utf-8", buffering=1)
    launch_process: ManagedProcess | None = None
    bag_process: ManagedProcess | None = None
    node: RunRecorder | None = None
    active_started_ns: int | None = None
    active_stopped_ns: int | None = None
    motor_zero_confirmed = False
    motor_zero_confirmed_ns: int | None = None
    interrupted = False
    failure: str | None = None
    cleanup_started = False

    def record_failure(message: str) -> None:
        nonlocal failure
        failure = message if failure is None else f"{failure}; {message}"

    def interrupt_handler(_signum: int, _frame: Any) -> None:
        if cleanup_started:
            return
        raise KeyboardInterrupt

    # Humble otherwise installs its own SIGINT/SIGTERM handlers during init and
    # can overwrite the harness fail-closed handler.  Disable that behavior,
    # then install one owner for INT/TERM/HUP.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, interrupt_handler)
    signal.signal(signal.SIGTERM, interrupt_handler)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, interrupt_handler)
    try:
        node = RunRecorder(run_dir)
        node.event(
            "session_created",
            requested_duration_sec=args.duration,
            max_speed_mps=args.speed,
            ros_domain_id=os.environ.get("ROS_DOMAIN_ID"),
            ros_localhost_only=os.environ.get("ROS_LOCALHOST_ONLY"),
            bag_topics=list(BAG_TOPICS),
            enable_heartbeat_timeout_sec=HARNESS_ENABLE_HEARTBEAT_TIMEOUT_SEC,
        )
        print(f"RUN_DIR={run_dir}")

        # Do not resolve conflicts by killing processes: report and stop here.
        wait_for_clean_graph(node, settle_sec=1.0)
        node.start_enable_gate()  # false exists before armed launch starts
        node.start_progressive_command_guard()
        wait_for_recorder_endpoint_pin(node, timeout_sec=args.ready_timeout)

        launch_command = [
            "ros2",
            "launch",
            "kobot_lane_drive",
            "cone_corridor_drive.launch.py",
            "armed:=true",
            "auto_enable:=false",
            f"max_speed_mps:={args.speed:.6g}",
            (
                "enable_heartbeat_timeout_sec:="
                f"{HARNESS_ENABLE_HEARTBEAT_TIMEOUT_SEC:.2f}"
            ),
        ]
        if args.config is not None:
            launch_command.append(f"config_file:={args.config}")
        node.assert_progressive_command_endpoints()
        node.reset_launch_readiness_cache()
        launch_process = start_process(launch_command, launch_log, "cone launch")
        launch_started_ns = time.monotonic_ns()
        node.event(
            "launch_started",
            pid=launch_process.process.pid,
            pgid=launch_process.pgid,
            command=launch_command,
        )
        wait_for_launch_ready(
            node,
            launch_process,
            launch_started_ns,
            args.ready_timeout,
            require_tracking=not args.allow_not_tracking,
        )
        # Exact node names and endpoint counts are checked before capturing the
        # DDS GIDs.  Every later check must match this immutable identity set.
        node.capture_command_publisher_whitelist()
        node.assert_command_publisher_whitelist()

        bag_subscription_baseline = {
            topic: node.subscription_counter(topic) for topic in BAG_TOPICS
        }
        bag_command = ["ros2", "bag", "record", "-o", str(bag_dir), *BAG_TOPICS]
        bag_process = start_process(bag_command, bag_log, "rosbag")
        node.event(
            "bag_started",
            pid=bag_process.process.pid,
            pgid=bag_process.pgid,
            command=bag_command,
        )
        wait_for_bag_ready(
            node,
            bag_process,
            bag_dir,
            bag_subscription_baseline,
            args.ready_timeout,
        )
        # Give discovery/subscription handshakes a short pre-roll while false.
        spin_runtime_for(node, 0.75, launch_process, bag_process)

        if not args.no_prompt:
            wait_for_operator(node, launch_process, bag_process)
        if args.countdown > 0.0:
            print(f"Starting in {args.countdown:.1f}s (still disabled)...")
            spin_runtime_for(node, args.countdown, launch_process, bag_process)

        # Mandatory immediate pre-activation recheck.  A new dashboard/AI
        # publisher cannot be admitted merely because preflight once passed.
        assert_runtime_integrity(node, launch_process, bag_process)
        if not args.allow_not_tracking and not node.tracking_ready(launch_started_ns):
            raise RuntimeError(
                "tracking became stale or found=false before activation; enable stayed false"
            )

        active_started_ns = time.monotonic_ns()
        node.set_enabled(True, event_name="active_enable_true")
        print(f"ACTIVE for exactly {args.duration:.3f}s")
        active_deadline_ns = active_started_ns + int(args.duration * 1e9)
        next_summary_ns = active_started_ns
        while time.monotonic_ns() < active_deadline_ns:
            remaining_sec = (active_deadline_ns - time.monotonic_ns()) / 1e9
            node.spin_once_gated(min(0.02, max(0.0, remaining_sec)))
            # Endpoint counters include DDS GIDs.  Any added/replaced command
            # publisher calls set_enabled(False) inside this assertion before
            # the exception unwinds to cleanup.
            assert_runtime_integrity(node, launch_process, bag_process)
            if time.monotonic_ns() >= next_summary_ns:
                status = node.latest_status or {}
                print(
                    "  t={:.1f}s mode={} path={} side={} steer={} speed={}".format(
                        (time.monotonic_ns() - active_started_ns) / 1e9,
                        status.get("mode"),
                        status.get("path_state"),
                        status.get("primary_side"),
                        status.get("steering_deg"),
                        status.get("speed_mps"),
                    )
                )
                next_summary_ns += int(1e9)

        active_stopped_ns, motor_zero_confirmed_ns = (
            request_false_and_confirm_motor_zero(
                node,
                args.stop_timeout,
                event_name="active_enable_false",
                validate_command_endpoints=True,
            )
        )
        motor_zero_confirmed = True
        node.event(
            "active_interval_complete",
            requested_duration_sec=args.duration,
            measured_enable_duration_sec=(active_stopped_ns - active_started_ns) / 1e9,
        )
        print("STOP CONFIRMED: enable=false and /xycar_motor speed=0 received")
        spin_runtime_for(node, args.stop_settle, launch_process, bag_process)
    except KeyboardInterrupt:
        interrupted = True
        print("INTERRUPTED: forcing enable=false before closing our run", file=sys.stderr)
    except Exception as exc:  # keep cleanup and the partial dataset
        record_failure(f"{type(exc).__name__}: {exc}")
        print(f"ERROR: {failure}", file=sys.stderr)
    finally:
        # A second Ctrl-C/SIGTERM must not tear through fail-closed cleanup.
        cleanup_started = True
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if hasattr(signal, "SIGHUP"):
            signal.signal(signal.SIGHUP, signal.SIG_IGN)
        try:
            if node is not None and node._enable_publisher is not None:
                if active_started_ns is not None and active_stopped_ns is None:
                    active_stopped_ns = (
                        node.stop_requested_mono_ns or time.monotonic_ns()
                    )
                if launch_process is not None and not motor_zero_confirmed:
                    try:
                        _, motor_zero_confirmed_ns = (
                            request_false_and_confirm_motor_zero(
                                node,
                                args.stop_timeout,
                                event_name="cleanup_enable_false",
                            )
                        )
                        motor_zero_confirmed = True
                    except Exception as exc:
                        record_failure(
                            f"motor zero confirmation failed before child shutdown: {exc}"
                        )
                elif not motor_zero_confirmed:
                    # No armed child launch was created, but keep the durable
                    # gate false without demanding a nonexistent bridge sample.
                    node.set_enabled(False, event_name="cleanup_enable_false_no_bridge")
        finally:
            # Nested finally blocks guarantee bag cleanup even if launch group
            # escalation itself encounters an unexpected error.  PGIDs were
            # created and recorded by this process; no discovered PID is used.
            try:
                try:
                    launch_group_stopped = stop_own_process_group(launch_process)
                    if not launch_group_stopped:
                        record_failure("owned cone launch process group survived cleanup")
                except Exception as exc:
                    record_failure(f"cone launch process-group cleanup failed: {exc}")
            finally:
                try:
                    bag_group_stopped = stop_own_process_group(bag_process)
                    if not bag_group_stopped:
                        record_failure("owned rosbag process group survived cleanup")
                except Exception as exc:
                    record_failure(f"rosbag process-group cleanup failed: {exc}")

        summary: dict[str, Any] = {
            "run_dir": str(run_dir),
            "completed_at": iso_now(),
            "requested_duration_sec": args.duration,
            "max_speed_mps": args.speed,
            "interrupted": interrupted,
            "failure": failure,
            "motor_zero_confirmed_before_child_shutdown": motor_zero_confirmed,
            "bag_graph_subscription_handshake": (
                node is not None and node.bag_subscription_whitelist is not None
            ),
            "active_enable_duration_sec": (
                (active_stopped_ns - active_started_ns) / 1e9
                if active_started_ns is not None and active_stopped_ns is not None
                else None
            ),
        }
        if node is not None:
            summary.update(
                {
                    "nonzero_motor_messages": node.nonzero_motor_messages,
                    "first_nonzero_motor_after_enable_sec": (
                        (node.first_nonzero_motor_mono_ns - active_started_ns) / 1e9
                        if active_started_ns is not None
                        and node.first_nonzero_motor_mono_ns is not None
                        else None
                    ),
                    "last_nonzero_motor_after_enable_sec": (
                        (node.last_nonzero_motor_mono_ns - active_started_ns) / 1e9
                        if active_started_ns is not None
                        and node.last_nonzero_motor_mono_ns is not None
                        else None
                    ),
                    "zero_after_stop_delay_sec": (
                        (
                            motor_zero_confirmed_ns
                            - node.stop_requested_mono_ns
                        )
                        / 1e9
                        if motor_zero_confirmed_ns is not None
                        and node.stop_requested_mono_ns is not None
                        else None
                    ),
                    "latest_status": node.latest_status,
                }
            )
            try:
                node.event("session_closed", summary=summary)
            except Exception:
                pass
            node.destroy_node()
            node.close_logs()
        if rclpy.ok():
            rclpy.shutdown()
        launch_log.close()
        bag_log.close()
        (run_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(f"RESULT={run_dir / 'summary.json'}")

    return 1 if failure is not None else 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())

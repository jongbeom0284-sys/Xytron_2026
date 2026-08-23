"""Integrated cone controller test harness without camera/lane missions.

This node deliberately reuses :class:`race_last.cone_link.ConeLink`, which is
the cone controller embedded in ``yolop_drive``.  It does not start YOLOP,
traffic-light or vehicle perception.  ``Cone_Perception`` supplies the same
``/cone/reading`` JSON used on race day and this node is the sole motor-command
publisher during an isolated cone test.

The motor stack is intentionally launched separately.  ``armed`` defaults to
false and any other publisher discovered on the motor topic causes a latched
zero-output state.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .cone_link import ConeLink


STOP = (0.0, 0.0)


class IntegratedConeTestNode(Node):
    """Run the integrated ``ConeLink`` controller as a standalone ROS node."""

    def __init__(self) -> None:
        super().__init__('integrated_cone_test')

        self.declare_parameter('armed', False)
        self.declare_parameter('reading_topic', '/cone/reading')
        self.declare_parameter('motor_topic', '/xycar_motor')
        self.declare_parameter(
            'preview_topic', '/race_last/cone_test/drive_preview')
        self.declare_parameter('status_topic', '/race_last/cone_test/status')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('mps_per_unit', 0.08)
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('acceleration_limit_mps2', 4.0)
        self.declare_parameter('max_steer', 40.0)
        self.declare_parameter('cone_speed_scale', 0.7)
        self.declare_parameter('cone_approach_depth_m', 1.2)
        self.declare_parameter('cone_wall_steer', True)
        self.declare_parameter('cone_wall_gain', 1.5)

        self.armed = bool(self.get_parameter('armed').value)
        self.reading_topic = str(self.get_parameter('reading_topic').value)
        self.motor_topic = str(self.get_parameter('motor_topic').value)
        preview_topic = str(self.get_parameter('preview_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.mps_per_unit = float(self.get_parameter('mps_per_unit').value)
        self.max_speed_mps = float(
            self.get_parameter('max_speed_mps').value)
        self.acceleration_limit_mps2 = float(
            self.get_parameter('acceleration_limit_mps2').value)
        self.max_steer = float(self.get_parameter('max_steer').value)
        self.cone_speed_scale = float(
            self.get_parameter('cone_speed_scale').value)

        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be finite and positive')
        if not math.isfinite(self.mps_per_unit) or self.mps_per_unit <= 0.0:
            raise ValueError('mps_per_unit must be finite and positive')
        if not math.isfinite(self.max_speed_mps) or self.max_speed_mps < 0.0:
            raise ValueError('max_speed_mps must be finite and non-negative')
        if (not math.isfinite(self.acceleration_limit_mps2)
                or self.acceleration_limit_mps2 <= 0.0):
            raise ValueError(
                'acceleration_limit_mps2 must be finite and positive')
        if not math.isfinite(self.max_steer) or self.max_steer <= 0.0:
            raise ValueError('max_steer must be finite and positive')
        if (not math.isfinite(self.cone_speed_scale)
                or self.cone_speed_scale < 0.0):
            raise ValueError(
                'cone_speed_scale must be finite and non-negative')

        self._lock = threading.Lock()
        self._reading: Optional[dict] = None
        self._last_command = STOP
        self._last_basis = 'WAITING'
        self._publisher_conflict = False
        self._reading_conflict = False
        self._last_status_at = -1e9

        self._link = ConeLink(
            self.get_logger(),
            approach_depth_m=float(
                self.get_parameter('cone_approach_depth_m').value),
            wall_steer=bool(self.get_parameter('cone_wall_steer').value),
            wall_gain=float(self.get_parameter('cone_wall_gain').value),
        )
        # ConeLink's inherited SpeedPolicy normally ramps at 22 legacy units/s,
        # only about 1.85 m/s^2 after the cone scale. Match the isolated test's
        # controller ramp to the explicit physical acceleration limit.
        output_mps_per_policy_unit = max(
            1e-6, self.mps_per_unit * self.cone_speed_scale)
        self._link._policy.speed.config.ramp_up_per_s = (
            self.acceleration_limit_mps2 / output_mps_per_policy_unit)

        self.create_subscription(
            String, self.reading_topic, self._on_reading, 10)
        self.pub_preview = self.create_publisher(
            Float32MultiArray, preview_topic, 10)
        self.pub_status = self.create_publisher(String, status_topic, 10)

        # Check before creating our endpoint so an already-running dashboard,
        # YOLOP node or another test controller cannot be hidden by our own
        # publisher in the count.
        existing = self.get_publishers_info_by_topic(self.motor_topic)
        if self.armed and existing:
            names = ', '.join(
                sorted({f'{i.node_namespace}/{i.node_name}' for i in existing}))
            raise RuntimeError(
                f'{self.motor_topic} already has publisher(s): {names}')

        self.pub_motor = (
            self.create_publisher(Float32MultiArray, self.motor_topic, 10)
            if self.armed else None
        )
        self.create_timer(1.0 / rate_hz, self._tick)
        self.create_timer(0.20, self._check_motor_publishers)

        if self.armed:
            self.get_logger().warning(
                'INTEGRATED CONE TEST ARMED: publishing isolated commands to '
                f'{self.motor_topic}; max {self.max_speed_mps:.2f} m/s, '
                f'accel {self.acceleration_limit_mps2:.2f} m/s^2')
        else:
            self.get_logger().warning(
                'Integrated cone test is PREVIEW ONLY (armed=false)')

    def _on_reading(self, message: String) -> None:
        try:
            body = json.loads(message.data)
            if not isinstance(body, dict):
                raise ValueError('reading must be a JSON object')
        except (json.JSONDecodeError, TypeError, ValueError):
            self.get_logger().warning('invalid /cone/reading JSON ignored')
            return
        with self._lock:
            self._reading = body

    def _check_motor_publishers(self) -> None:
        reading_publishers = self.get_publishers_info_by_topic(
            self.reading_topic)
        unexpected_readers = [
            info for info in reading_publishers
            if info.node_name != 'cone_perception'
        ]
        if ((len(reading_publishers) > 1 or unexpected_readers)
                and not self._reading_conflict):
            self._reading_conflict = True
            names = ', '.join(sorted(
                f'{i.node_namespace}/{i.node_name}'
                for i in reading_publishers))
            self.get_logger().error(
                f'CONE READING PUBLISHER CONFLICT ({names}); '
                'output latched at zero')
            self._publish_motor(*STOP)

        if self.pub_motor is None or self._publisher_conflict:
            return
        foreign = [
            info for info in self.get_publishers_info_by_topic(self.motor_topic)
            if info.node_name != self.get_name()
        ]
        if not foreign:
            return
        self._publisher_conflict = True
        names = ', '.join(
            sorted({f'{i.node_namespace}/{i.node_name}' for i in foreign}))
        self.get_logger().error(
            f'MOTOR PUBLISHER CONFLICT ({names}); output latched at zero')
        self._publish_motor(*STOP)

    def _tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            reading = self._reading

        command = self._link.tick(
            now,
            reading,
            ego_mps=max(0.0, self._last_command[1]) * self.mps_per_unit,
            # Camera is intentionally absent in this isolated LiDAR test.  The
            # race stack's cone safety decision is being tested, not camera
            # availability degradation.
            camera_ok=True,
        )

        if command is None:
            steer, speed_units = STOP
            if self._link.approach:
                basis = 'APPROACH_WAIT'
            else:
                basis = 'WAITING_OR_EXITED'
        else:
            steer = max(-self.max_steer, min(self.max_steer,
                                             float(command[0])))
            planned_units = max(0.0,
                                float(command[1]) * self.cone_speed_scale)
            cap_units = self.max_speed_mps / self.mps_per_unit
            speed_units = min(planned_units, cap_units)
            basis = f'CONE_{self._link.wall_basis or "POLICY"}'

        if self._publisher_conflict or self._reading_conflict:
            steer, speed_units = STOP
            basis = ('MOTOR_PUBLISHER_CONFLICT' if self._publisher_conflict
                     else 'CONE_READING_CONFLICT')

        if not all(math.isfinite(v) for v in (steer, speed_units)):
            steer, speed_units = STOP
            basis = 'NONFINITE_COMMAND'

        self._last_command = (steer, speed_units)
        self._last_basis = basis
        self._publish_preview(steer, speed_units)
        self._publish_motor(steer, speed_units)
        self._publish_status(now, reading, steer, speed_units, basis)

    def _publish_preview(self, steer: float, speed_units: float) -> None:
        self.pub_preview.publish(Float32MultiArray(
            data=[float(steer), float(speed_units)]))

    def _publish_motor(self, steer: float, speed_units: float) -> None:
        if self.pub_motor is None:
            return
        self.pub_motor.publish(Float32MultiArray(
            data=[float(steer), float(speed_units)]))

    def _publish_status(self, now: float, reading: Optional[dict],
                        steer: float, speed_units: float, basis: str) -> None:
        if now - self._last_status_at < 0.20:
            return
        self._last_status_at = now
        data = reading or {}
        payload = {
            'armed': self.armed,
            'active': bool(self._link.active),
            'approach': bool(self._link.approach),
            'basis': basis,
            'wall_basis': str(self._link.wall_basis or ''),
            'publisher_conflict': self._publisher_conflict,
            'reading_conflict': self._reading_conflict,
            'cones': int(data.get('cones', 0)),
            'gate_valid': bool(data.get('valid', False)),
            'gate_depth_m': float(data.get('depth', 0.0)),
            'gate_width_m': float(data.get('width', 0.0)),
            'gate_lateral_m': float(data.get('lateral', 0.0)),
            'steer': float(steer),
            'speed_units': float(speed_units),
            'speed_mps': float(speed_units * self.mps_per_unit),
            'acceleration_limit_mps2': self.acceleration_limit_mps2,
        }
        self.pub_status.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        if self.pub_motor is not None and rclpy.ok():
            try:
                self._publish_motor(*STOP)
            except Exception:  # context may already be shutting down
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = IntegratedConeTestNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

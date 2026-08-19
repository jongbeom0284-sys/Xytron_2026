from pathlib import Path


def _sources():
    package_root = Path(__file__).parents[1]
    server = (package_root / "kobot_vehicle" / "dashboard_server.py").read_text(
        encoding="utf-8"
    )
    app = (package_root / "dashboard" / "app.js").read_text(encoding="utf-8")
    index = (package_root / "dashboard" / "index.html").read_text(
        encoding="utf-8"
    )
    return server, app, index


def test_recovery_api_covers_individual_media_and_all_sensor_restarts():
    server, _, _ = _sources()

    assert "'lane': (" in server
    assert "'video': (" in server
    assert "'media': ('camera', 'lane', 'video')" in server
    assert "'all': ('camera', 'lane', 'video', 'lidar', 'ultrasonic', 'imu')" in server
    assert "path.startswith('/api/recovery/component/')" in server
    assert "path.startswith('/api/recovery/group/')" in server
    assert "self.devices.reconnect_many(names)" in server


def test_real_component_restart_has_motor_and_recording_safety_gates():
    server, _, _ = _sources()

    assert "if record_state in ('starting', 'running', 'paused', 'ending')" in server
    assert "if self.recording:" in server
    assert "if self.motor.active:" in server
    assert "if self.motor.stack_pids():" in server
    assert "self.motor.shutdown_stack()" in server
    assert "self.publish_zero(0.6)" in server
    assert "self._stop_stack()" in server


def test_browser_refresh_closes_old_mjpeg_connections_without_ros_api():
    _, app, index = _sources()

    assert 'id="open-recovery"' in index
    assert 'data-recovery-action="stream"' in app
    assert "image.removeAttribute('src')" in app
    assert "refreshStreamConnections()" in app
    assert "closeStreamConnections();" in app
    assert "/api/recovery/component/${component}" in app
    assert "/api/recovery/group/${group}" in app
    assert "모터 스택이 켜져 있으면 먼저 0속도" in app

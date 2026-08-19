from pathlib import Path


def _sources():
    package_root = Path(__file__).parents[1]
    server = (package_root / "kobot_vehicle" / "dashboard_server.py").read_text(
        encoding="utf-8"
    )
    app = (package_root / "dashboard" / "app.js").read_text(encoding="utf-8")
    return server, app


def test_record_drive_speed_is_selected_before_start_and_capped_at_half_mps():
    server, app = _sources()

    assert "MIN_SPEED_MPS = 0.20" in server
    assert "MAX_SPEED_MPS = 0.50" in server
    assert "DEFAULT_SPEED_MPS = 0.50" in server
    assert "f'max_speed_mps:={self.speed_mps:.2f}'" in server
    assert "[0.2,0.3,0.4,0.5]" in app
    assert "JSON.stringify({speed_mps:recordDriveSettings.speed})" in app


def test_record_drive_sessions_are_unique_and_pause_keeps_the_open_session():
    server, _ = _sources()

    assert "strftime('%Y%m%d_%H%M%S_%f')[:-3]" in server
    assert "candidate.mkdir(parents=True, exist_ok=False)" in server
    assert "recording_path / 'record-drive.log'" in server
    assert "self.node.pause_recording()" in server
    assert "self.node.resume_recording()" in server
    assert "recording = self.node.stop_recording()" in server


def test_drive_start_rejects_foreign_motor_publishers():
    server, _ = _sources()

    assert "def foreign_motor_publishers(self):" in server
    assert "self.get_publishers_info_by_topic('/xycar_motor')" in server
    assert server.count("self.node.ensure_exclusive_motor_publisher()") >= 3
    assert "다른 /xycar_motor 발행자를 먼저 종료하세요" in server

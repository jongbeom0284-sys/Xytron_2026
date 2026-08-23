"""Static safety contract for the isolated integrated-cone launch."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_cone_drive_test_launch_is_isolated():
    text = (ROOT / 'launch' / 'cone_drive_test.launch.py').read_text(
        encoding='utf-8')
    assert "executable='cone_perception'" in text
    assert "executable='integrated_cone_test'" in text
    assert "executable='xycar_lidar_node'" in text
    assert "executable='motor_logger'" in text
    assert "executable='yolop_drive'" not in text
    assert "executable='dashboard_server'" not in text
    assert 'motor.launch.py' not in text
    assert "'max_speed_mps', default_value='1.5'" in text
    assert "'acceleration_limit_mps2', default_value='4.0'" in text
    assert "'cone_speed_scale', default_value='1.05'" in text


def test_test_controller_defaults_disarmed_and_stops_without_cone_command():
    text = (ROOT / 'race_last' / 'integrated_cone_test_node.py').read_text(
        encoding='utf-8')
    assert "declare_parameter('armed', False)" in text
    assert 'if command is None:' in text
    assert 'steer, speed_units = STOP' in text
    assert 'get_publishers_info_by_topic(self.motor_topic)' in text
    assert 'get_publishers_info_by_topic(\n            self.reading_topic)' in text
    assert "'publisher_conflict': self._publisher_conflict" in text
    assert "'reading_conflict': self._reading_conflict" in text


def test_force_enabled_cone_perception_ignores_traffic_gate():
    text = (ROOT / 'race_last' / 'Cone_Perception.py').read_text(
        encoding='utf-8')
    signal = text.split('def _on_signal(self, msg):', 1)[1]
    signal = signal.split('def _on_scan(self, msg):', 1)[0]
    assert 'if self.force:' in signal
    assert 'self.enabled = True' in signal
    assert signal.index('if self.force:') < signal.index("if state == 'green':")

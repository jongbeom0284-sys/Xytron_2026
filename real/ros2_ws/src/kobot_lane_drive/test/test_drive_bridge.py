import pytest
from kobot_lane_drive.drive_command import make_legacy_command


def test_stale_or_invalid_state_always_stops():
    assert make_legacy_command(0.1, 0.3, 0.0068, 0.08, ready=False, fresh=True) == [0.0, 0.0]
    assert make_legacy_command(0.1, 0.3, 0.0068, 0.08, ready=True, fresh=False) == [0.0, 0.0]
    assert make_legacy_command(None, 0.3, 0.0068, 0.08, ready=True, fresh=True) == [0.0, 0.0]


def test_converts_preview_radians_to_existing_motor_adapter_contract():
    command = make_legacy_command(0.068, 0.32, 0.0068, 0.08, ready=True, fresh=True)
    assert command[0] == pytest.approx(-10.0)
    assert command[1] == pytest.approx(4.0)


def test_clamps_legacy_command():
    assert make_legacy_command(-1.0, 9.0, 0.0068, 0.08, True, True) == [50.0, 50.0]

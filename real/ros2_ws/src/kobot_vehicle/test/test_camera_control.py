from types import SimpleNamespace

import pytest

from kobot_vehicle.camera_control import CameraControl


class FakeRunner:
    def __init__(self):
        self.commands = []

    def __call__(self, command):
        self.commands.append(command)
        return SimpleNamespace(stdout='''
brightness: 0
exposure_time_absolute: 40
gamma: 150
auto_exposure: 1 (Manual Mode)
''')


def test_status_reports_whitelisted_controls_and_ranges():
    runner = FakeRunner()
    result = CameraControl(runner=runner).status()

    assert result['manual_exposure'] is True
    assert result['values'] == {'brightness': 0, 'exposure': 40, 'gamma': 150}
    assert result['controls']['brightness']['min'] == -64
    assert result['controls']['gamma']['max'] == 500
    assert runner.commands == [[
        'v4l2-ctl', '-d', '/dev/videoCAM',
        '--get-ctrl=brightness,exposure_time_absolute,gamma,auto_exposure',
    ]]


def test_apply_uses_fixed_argument_list_and_manual_exposure():
    runner = FakeRunner()
    CameraControl(runner=runner).apply({
        'gamma': 200,
        'brightness': -5,
        'exposure': 55,
    })

    assert runner.commands[0] == [
        'v4l2-ctl', '-d', '/dev/videoCAM',
        '--set-ctrl=brightness=-5,auto_exposure=1,exposure_time_absolute=55,gamma=200',
    ]
    assert runner.commands[1][3].startswith('--get-ctrl=')


@pytest.mark.parametrize('payload', [
    {'gamma': 501},
    {'brightness': -65},
    {'exposure': 0},
    {'gamma': True},
    {'command': 'rm -rf /'},
    {},
])
def test_apply_rejects_invalid_or_unknown_values(payload):
    runner = FakeRunner()
    with pytest.raises(ValueError):
        CameraControl(runner=runner).apply(payload)
    assert runner.commands == []

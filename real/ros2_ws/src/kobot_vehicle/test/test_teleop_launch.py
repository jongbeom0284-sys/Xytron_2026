from pathlib import Path


def test_teleop_launch_composes_existing_hardware_launches_safely():
    package_root = Path(__file__).parents[1]
    source = (package_root / "launch" / "teleop.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"sensors.launch.py"' in source
    assert '"motor.launch.py"' in source
    assert '"start_sensors"' in source
    assert '"armed"' in source
    assert source.count('default_value="false"') >= 2
    assert 'launch_arguments={"armed": armed}.items()' in source

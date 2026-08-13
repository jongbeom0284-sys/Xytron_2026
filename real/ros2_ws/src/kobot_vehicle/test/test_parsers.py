from kobot_vehicle.imu_node import parse_imu_line
from kobot_vehicle.ultrasonic_node import parse_ultrasonic_line


def test_ultrasonic_valid_and_clamped():
    assert parse_ultrasonic_line('1,2,3,4,5,6,141,x') == [1,2,3,4,5,6,0,0]


def test_ultrasonic_rejects_short_line():
    assert parse_ultrasonic_line('1,2,3') is None


def test_imu_line():
    parsed = parse_imu_line('#YPRAG=1,2,3,4,5,6,7,8,9')
    assert parsed is not None
    assert len(parsed['quaternion']) == 4

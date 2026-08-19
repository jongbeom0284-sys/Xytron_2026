"""인식 계층이 모터를 직접 건드리지 않는다는 계약을 지킨다.

실제 모터 출력은 승인 잠금 bridge 한 곳에만 둔다. 인식·판단 모듈이 /xycar_motor 를
직접 발행하면 dashboard 의 경쟁 퍼블리셔 검사가 주행 시작을 거부한다.
"""

from pathlib import Path


BRIDGE = "twinmixing_center_drive_bridge_node.py"


def test_perception_runtime_has_no_motor_publish_interface():
    package_root = Path(__file__).parents[1] / "kobot_lane_drive"
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in package_root.glob("*.py")
        if path.name != BRIDGE
    )

    assert "/xycar_motor" not in source
    assert "/ackermann_cmd" not in source
    assert "/commands/motor/speed" not in source
    assert "/commands/servo/position" not in source


def test_twinmixing_drive_bridge_requires_explicit_arming():
    package_root = Path(__file__).parents[1]
    bridge = (package_root / "kobot_lane_drive" / BRIDGE).read_text(encoding="utf-8")
    launch = (
        package_root / "launch" / "twinmixing_center_record_drive.launch.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("armed", False)' in bridge
    assert 'default_value="false"' in launch
    assert "IfCondition(armed)" in launch


def test_only_the_bridge_publishes_the_motor_topic():
    package_root = Path(__file__).parents[1] / "kobot_lane_drive"
    publishers = [
        path.name
        for path in package_root.glob("*.py")
        if "/xycar_motor" in path.read_text(encoding="utf-8")
    ]
    assert publishers == [BRIDGE]

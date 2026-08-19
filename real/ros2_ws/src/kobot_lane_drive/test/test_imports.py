"""패키지의 모든 모듈이 실제로 import 되는지 확인한다.

문자열만 검사하는 시험은 옮겨진 의존을 못 잡는다. 2026-08-19 재편에서
drive_command.py 를 legacy 로 내리는 바람에 승인 잠금 bridge 가 주행 시작 직후
ModuleNotFoundError 로 죽었고, 그 사실이 실차 주행에서야 드러났다.
"""

import importlib
import pkgutil

import pytest

import kobot_lane_drive

# ROS 런타임이나 CUDA 가 있어야만 import 되는 모듈은 시험 환경에서 건너뛴다.
NEEDS_RUNTIME = {"twinmixing_center_node", "twinmixing_center_drive_bridge_node"}


def _module_names():
    return [name for _, name, _ in pkgutil.iter_modules(kobot_lane_drive.__path__)]


@pytest.mark.parametrize("name", _module_names())
def test_module_imports(name):
    if name in NEEDS_RUNTIME:
        pytest.importorskip("rclpy")
    importlib.import_module(f"kobot_lane_drive.{name}")


def test_gated_bridge_imports_are_all_local():
    """브리지가 쓰는 모듈이 전부 살아있는 패키지 안에 있는지 확인한다."""
    import ast
    from pathlib import Path

    source = (
        Path(__file__).parents[1]
        / "kobot_lane_drive"
        / "twinmixing_center_drive_bridge_node.py"
    ).read_text(encoding="utf-8")
    package = Path(__file__).parents[1] / "kobot_lane_drive"
    for node in ast.walk(ast.parse(source)):
        if isinstance(node, ast.ImportFrom) and node.level == 1:
            assert (package / f"{node.module}.py").is_file(), (
                f"bridge 가 import 하는 {node.module} 이 패키지에 없다"
            )

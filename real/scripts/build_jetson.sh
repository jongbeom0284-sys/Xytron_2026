#!/usr/bin/env bash
# kobot 실차 워크스페이스 빌드.
#
# TwinMixing 추론은 별도 venv(~/.venvs/twinmixing)의 Torch 를 쓴다. colcon 자체는
# 시스템 python3 로 돌리고, 런타임 PYTHONPATH 주입은 sensors.launch.py 가 담당한다.
set -euo pipefail

REPOSITORY="${KOBOT_REPOSITORY:-/home/xytron/2026_Xytron}"
WORKSPACE="$REPOSITORY/real/ros2_ws"
TORCH_SITE="${KOBOT_TORCH_SITE:-/home/xytron/.venvs/twinmixing/lib/python3.10/site-packages}"

test "$(hostname)" = kobot
test -d "$WORKSPACE/src/kobot_lane_drive"
test -d "$TORCH_SITE"

set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WORKSPACE"
colcon build --symlink-install "$@"

set +u
source "$WORKSPACE/install/setup.bash"
set -u

echo "--- 설치된 실행 파일 ---"
ls "$WORKSPACE/install/kobot_lane_drive/lib/kobot_lane_drive"
ls "$WORKSPACE/install/kobot_vehicle/lib/kobot_vehicle"

echo "--- 런타임 import 확인 ---"
PYTHONPATH="$TORCH_SITE:${PYTHONPATH:-}" python3 - <<'PY'
import rclpy
import torch
from kobot_lane_drive.twinmixing_center import extract_center_path
from kobot_lane_drive.geometry import move_towards
print("build runtime imports OK", "torch", torch.__version__, "cuda", torch.cuda.is_available())
PY

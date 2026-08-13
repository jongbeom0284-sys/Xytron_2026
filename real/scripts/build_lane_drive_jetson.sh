#!/usr/bin/env bash
set -euo pipefail

REPOSITORY=/home/xytron/2026_Xytron
WORKSPACE="$REPOSITORY/real/ros2_ws"
VENV="$REPOSITORY/real/.venv"

test "$(hostname)" = kobot
cd "$REPOSITORY"
test "$(git branch --show-current)" = main
test "$(git rev-parse HEAD)" = "$(git ls-remote origin refs/heads/main | cut -f1)"
test -x "$VENV/bin/python"

set +u
source /opt/ros/humble/setup.bash
set -u
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
cd "$WORKSPACE"
"$VENV/bin/python" -m colcon build --symlink-install \
    --packages-select ipm_interfaces ipm_library kobot_lane_drive

set +u
source "$WORKSPACE/install/setup.bash"
set -u
head -n 1 "$WORKSPACE/install/kobot_lane_drive/lib/kobot_lane_drive/lane_perception"
"$VENV/bin/python" - <<'PY'
import rclpy
import shapely
import torch
from ipm_library.ipm import IPM
print("build runtime imports OK", torch.__version__, shapely.__version__, IPM)
PY

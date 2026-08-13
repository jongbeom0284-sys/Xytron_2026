#!/usr/bin/env bash
set -euo pipefail

SOURCE_VENV=/home/xytron/clrernet_eval/venv
TARGET_VENV=/home/xytron/2026_Xytron/real/.venv
MIGRATING_VENV=/home/xytron/2026_Xytron/real/.venv.migrating
SOURCE_WEIGHT=/home/xytron/clrernet_eval/weights/clrernet_culane_dla34_ema.pth
TARGET_WEIGHT=/home/xytron/2026_Xytron/real/models/clrernet_culane_dla34_ema.pth
EXPECTED_WEIGHT_SHA256=cce8b552bc8301e31186da192fe4213043e9351b35651ef5d40014ce3040c787
NMS_SOURCE=/home/xytron/2026_Xytron/real/ros2_ws/src/kobot_lane_drive/third_party/CLRerNet/libs/models/layers/nms

test "$(hostname)" = kobot
test -d "$SOURCE_VENV"
test -f "$SOURCE_WEIGHT"
test -d "$NMS_SOURCE"
test ! -e "$TARGET_VENV"
test ! -e "$MIGRATING_VENV"

actual_source_hash="$(sha256sum "$SOURCE_WEIGHT" | cut -d' ' -f1)"
test "$actual_source_hash" = "$EXPECTED_WEIGHT_SHA256"

cp -a "$SOURCE_VENV" "$MIGRATING_VENV"
/usr/bin/python3 -m venv --upgrade "$MIGRATING_VENV"
source /opt/ros/humble/setup.bash
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
"$MIGRATING_VENV/bin/python" -m pip install --no-deps --force-reinstall "$NMS_SOURCE"
"$MIGRATING_VENV/bin/python" - <<'PY'
import cv2
import mmcv
import mmdet
import nms
import numpy
import rclpy
import scipy
import shapely
import torch
import torchvision

assert torch.cuda.is_available()
print("runtime imports OK", torch.__version__, torchvision.__version__, shapely.__version__)
PY
mv "$MIGRATING_VENV" "$TARGET_VENV"

if test -e "$TARGET_WEIGHT"; then
    actual_target_hash="$(sha256sum "$TARGET_WEIGHT" | cut -d' ' -f1)"
    test "$actual_target_hash" = "$EXPECTED_WEIGHT_SHA256"
else
    cp -p "$SOURCE_WEIGHT" "$TARGET_WEIGHT"
fi
sha256sum "$TARGET_WEIGHT"

echo "Migration copied and verified. Original clrernet_eval was not removed."

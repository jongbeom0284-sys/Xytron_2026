#!/usr/bin/env bash
set -u

REPOSITORY=/home/xytron/2026_Xytron
set +u
source /home/xytron/kobot_env.sh
set -u

echo "HOST=$(hostname)"
cd "$REPOSITORY" || exit 1
echo "BRANCH=$(git branch --show-current)"
echo "HEAD=$(git rev-parse HEAD)"
echo "REMOTE_MAIN=$(git ls-remote origin refs/heads/main | cut -f1)"
echo "WORKTREE"
git status --short

echo "CAMERA"
ros2 topic info /image_raw -v
timeout 6 ros2 topic hz /image_raw
ros2 topic echo /camera_info --once

echo "TF"
ros2 topic info /tf_static -v
ros2 run tf2_ros tf2_echo base_link default_cam --once

echo "MOTOR_DEVICE"
ls -l /dev/ttyMOTOR 2>&1
fuser -v /dev/ttyMOTOR 2>&1 || true

echo "MOTOR_PROCESSES"
ps -eo pid,args | grep -Ei 'vesc|motor|ackermann|kmu_race|track_drive|figure_eight|lane_drive' | grep -v grep || true

echo "MOTOR_TOPICS"
ros2 topic info /xycar_motor -v
ros2 topic info /ackermann_cmd -v

echo "AUTONOMOUS_CONFLICTS"
if ps -eo args | grep -Ei 'kmu_race|track_drive|figure_eight|lane_drive_bridge' | grep -v grep; then
    echo "BLOCKED: another autonomous or test publisher is running"
    exit 2
fi
echo "No known autonomous publisher process detected. User approval is still required."

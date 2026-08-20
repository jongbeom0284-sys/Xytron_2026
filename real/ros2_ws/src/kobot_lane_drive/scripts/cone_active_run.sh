#!/usr/bin/env bash
set -euo pipefail

# This wrapper changes only the current shell environment.  It does not start,
# stop, restart, or kill the LiDAR/VESC/motor services.
ENV_FILE="${CONE_TEST_ENV:-${HOME}/cone_test_env.sh}"
if [[ ! -r "${ENV_FILE}" ]]; then
  echo "Missing ROS test environment: ${ENV_FILE}" >&2
  echo "Set CONE_TEST_ENV to the reviewed environment script." >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck disable=SC1090
source "${ENV_FILE}"

echo "PACKAGE=$(ros2 pkg prefix kobot_lane_drive)"
exec python3 "${SCRIPT_DIR}/cone_active_run.py" "$@"

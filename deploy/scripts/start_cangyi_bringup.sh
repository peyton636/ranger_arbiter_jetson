#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${1:-/opt/cangyirobot}"
MODE="${2:-control}"
CAN_PORT="${3:-can0}"
ARM_TYPE="${4:-piper}"
EFFECTOR_TYPE="${5:-none}"

if [[ ! -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
  echo "Missing setup file: ${WORKSPACE_DIR}/install/setup.bash"
  exit 1
fi

# shellcheck disable=SC1090
source "${WORKSPACE_DIR}/install/setup.bash"

exec ros2 launch cangyi_bringup bringup.launch.py \
  mode:="${MODE}" \
  can_port:="${CAN_PORT}" \
  arm_type:="${ARM_TYPE}" \
  effector_type:="${EFFECTOR_TYPE}"

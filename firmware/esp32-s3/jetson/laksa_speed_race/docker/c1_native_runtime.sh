#!/usr/bin/env bash
# Source only the isolated native C1 runtime and fail before launch on mismatch.
set -eo pipefail

ACKERMANN_MSGS_PREFIX="${ACKERMANN_MSGS_PREFIX:-/tmp/laksa-c1-native/ackermann_root/opt/ros/humble}"
LAKSA_C1_WORKSPACE="${LAKSA_C1_WORKSPACE:-/tmp/laksa-c1.2-runtime}"
LAKSA_C1_VENV="${LAKSA_C1_VENV:-${LAKSA_C1_WORKSPACE}/venv}"
F1TENTH_GYM_CHECKOUT="${F1TENTH_GYM_CHECKOUT:-/tmp/laksa-c1-native/f1tenth_gym}"
F1TENTH_GYM_PYDEPS="${F1TENTH_GYM_PYDEPS:-/tmp/laksa-c1-native/pydeps}"
ACKERMANN_MSGS_SETUP="${ACKERMANN_MSGS_PREFIX}/share/ackermann_msgs/local_setup.bash"

if [[ ! -f "${ACKERMANN_MSGS_SETUP}" ]]; then
  echo "missing ackermann_msgs runtime setup: ${ACKERMANN_MSGS_SETUP}" >&2
  exit 2
fi
if [[ ! -f "${LAKSA_C1_WORKSPACE}/install/setup.bash" ]]; then
  echo "missing isolated C1 setup: ${LAKSA_C1_WORKSPACE}/install/setup.bash" >&2
  exit 2
fi
if [[ ! -f "${F1TENTH_GYM_CHECKOUT}/f1tenth_gym/__init__.py" ]]; then
  echo "missing pinned f1tenth_gym checkout: ${F1TENTH_GYM_CHECKOUT}" >&2
  exit 2
fi
if [[ ! -d "${F1TENTH_GYM_PYDEPS}/gymnasium" ]]; then
  echo "missing isolated f1tenth_gym dependencies: ${F1TENTH_GYM_PYDEPS}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${ACKERMANN_MSGS_SETUP}"
source "${LAKSA_C1_WORKSPACE}/install/setup.bash"
export PYTHONPATH="${F1TENTH_GYM_CHECKOUT}:${F1TENTH_GYM_PYDEPS}:${LAKSA_C1_VENV}/lib/python3.10/site-packages:${PYTHONPATH:-}"
set -u

exec ros2 run laksa_speed_race c1_closed_loop_qualification \
  --workspace "${LAKSA_C1_WORKSPACE}" \
  --ackermann-prefix "${ACKERMANN_MSGS_PREFIX}" \
  --f1tenth-gym-checkout "${F1TENTH_GYM_CHECKOUT}" \
  "$@"

#!/usr/bin/env bash
# FGC launcher: sources the ROS2 environment before running the binary so that
# the ROS shared libraries (librclcpp, librcl, libmavros_msgs, ...) resolve even
# when launched from a desktop icon (which has no ROS env in LD_LIBRARY_PATH).
set -eo pipefail

# ROS2 distro to source (override with ROS_DISTRO before launching if needed).
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${ROS_SETUP}" ]]; then
    # ROS setup scripts reference unset vars, so keep nounset off while sourcing.
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
else
    echo "FGC: ROS setup not found at ${ROS_SETUP}" >&2
fi

# Resolve repo root from this script's location: custom/deploy/linux/ -> repo root.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# Prefer Debug, fall back to Release.
BIN="${REPO_ROOT}/build/Debug/FGC"
[[ -x "${BIN}" ]] || BIN="${REPO_ROOT}/build/Release/FGC"

exec "${BIN}" "$@"

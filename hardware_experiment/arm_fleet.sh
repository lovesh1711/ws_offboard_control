#!/usr/bin/env bash
#
# arm_fleet.sh — simultaneous, multi-threaded arm of the whole fleet.
#
# Runs on the LAPTOP (central station). Sources ROS + this workspace, then runs
# the multi_sim `arm_all` node (one process, MultiThreadedExecutor, one callback
# group per drone) against fleet.txt — all drones arm together, hold, disarm.
#
# !!! PROPS OFF !!!  (force-arm spins the motors). Both agents must be running
# on the Pis, and both drones powered.
#
# Usage:
#   ./hardware_experiment/arm_fleet.sh                       # force-arm, 5 s hold
#   ./hardware_experiment/arm_fleet.sh -p force:=false -p hold_s:=8.0
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="${WS_DIR:-$(dirname "$SCRIPT_DIR")}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export MISSION_CONFIG_FILE="${MISSION_CONFIG_FILE:-$SCRIPT_DIR/fleet.txt}"

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_DIR/install/local_setup.bash"
set -u

echo "[arm_fleet] fleet ($MISSION_CONFIG_FILE):"
grep -vE '^\s*#|^\s*$' "$MISSION_CONFIG_FILE" | sed 's/^/    /'
echo "[arm_fleet] PROPS OFF — arming all drones simultaneously (multi-threaded)..."

if [[ "$#" -gt 0 ]]; then
  ros2 run multi_sim arm_all --ros-args "$@"
else
  ros2 run multi_sim arm_all
fi

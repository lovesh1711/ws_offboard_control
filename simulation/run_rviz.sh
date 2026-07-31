#!/usr/bin/env bash
#
# run_rviz.sh — 3-D path plot for the mission in RViz 2.
#
# Starts the trajectory/marker publisher (rviz_paths.py) and opens RViz with the
# mission layout. Reads the same /tmp/mission_config.txt that record_demo.sh
# wrote, so run this ALONGSIDE record_demo.sh (order doesn't matter).
#
set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # this simulation/ folder
WS_DIR="${WS_DIR:-$(dirname "$SCRIPT_DIR")}"                 # colcon workspace root (parent)
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
export MISSION_CONFIG_FILE="${MISSION_CONFIG_FILE:-/tmp/mission_config.txt}"

cleanup() {
  pkill -9 -f 'rviz_paths.py' 2>/dev/null || true
  pkill -9 -f 'static_transform_publisher.*map' 2>/dev/null || true
}
trap 'cleanup; exit 0' INT TERM

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_DIR/install/local_setup.bash"
set -u

# make the 'map' frame exist in TF so RViz always renders the markers
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base >/dev/null 2>&1 &

echo "[run_rviz] starting trajectory/marker publisher (rviz_paths.py)"
python3 "$SCRIPT_DIR/rviz_paths.py" &

echo "[run_rviz] opening RViz (Fixed Frame = map, MarkerArray on /viz/markers)"
rviz2 -d "$SCRIPT_DIR/mission.rviz"

cleanup

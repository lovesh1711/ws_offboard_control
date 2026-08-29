#!/usr/bin/env bash
#
# record_demo.sh — one-command SITL demo for the paper video.
#
# Spawns 3 drones near the base, takes them off SIMULTANEOUSLY, and fans them into
# two quadrants at different cruise altitudes. It does NOT screen-record; use
# Gazebo's built-in Video Recorder (GUI: the vertical "..." menu -> "Video
# Recorder"). For a labelled 3-D path plot with moving drone labels, run RViz
# alongside via ./run_rviz.sh (which reads the same /tmp/mission_config.txt).
#
set -uo pipefail

# ----------------------------- configuration -------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"   # this simulation/ folder
WS_DIR="${WS_DIR:-$(dirname "$SCRIPT_DIR")}"                 # colcon workspace root (parent)
PX4_DIR="${PX4_DIR:-/home/lovesh/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
AGENT_PORT="${AGENT_PORT:-8888}"
AUTOSTART="${AUTOSTART:-4001}"
SIM_MODEL="${SIM_MODEL:-gz_x500}"
GZ_WORLD="${GZ_WORLD:-demo}"                              # world name (matches <world name=...>)
WORLD_FILE="${WORLD_FILE:-$SCRIPT_DIR/worlds/$GZ_WORLD.sdf}"  # custom city world for the video
GZ_WAIT_TIMEOUT="${GZ_WAIT_TIMEOUT:-60}"
CONFIG_FILE="/tmp/mission_config.txt"
LOG_DIR="/tmp/mission_logs"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
export MISSION_CONFIG_FILE="$CONFIG_FILE"

# ---- 3-drone demo: spawn (x y) | world target (x y) | cruise alt | colour (r g b) ----
NAMES=(1 2 3)
SPAWN_X=( 2   2   -5 )
SPAWN_Y=( 2   -2   0)
TGT_X=(  10  10  12 )
TGT_Y=(  -10 10   5)
ALT=(     10   5  8 )

# ----------------------------- cleanup -------------------------------------
kill_all() {
  echo
  echo "[record_demo] cleaning up ..."
  pkill -9 -f 'mission_setpoints' 2>/dev/null || true
  pkill -9 -f 'bin/px4'           2>/dev/null || true
  pkill -9 -f 'px4_sitl'          2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent'    2>/dev/null || true
  pkill -9 -f 'gz sim'            2>/dev/null || true
  pkill -9 -f 'ruby .*gz'         2>/dev/null || true
  sleep 1
  echo "[record_demo] done."
}
trap 'kill_all; exit 0' INT TERM
kill_all

# ----------------------------- checks --------------------------------------
[[ -x "$PX4_BIN" ]]            || { echo "ERROR: px4 not found: $PX4_BIN"; exit 1; }
command -v MicroXRCEAgent >/dev/null || { echo "ERROR: MicroXRCEAgent not in PATH"; exit 1; }
[[ -f "$WS_DIR/install/local_setup.bash" ]] || { echo "ERROR: workspace not built (colcon build in $WS_DIR)"; exit 1; }
mkdir -p "$LOG_DIR"

# ----------------------------- write demo config ---------------------------
: > "$CONFIG_FILE"
echo "# ns sysid wait target_x target_y cruise_alt spawn_x spawn_y   (targets local NED, wait=0 -> simultaneous)" >> "$CONFIG_FILE"
for idx in "${!NAMES[@]}"; do
  i="${NAMES[$idx]}"
  ltx=$(awk -v t="${TGT_X[$idx]}" -v s="${SPAWN_X[$idx]}" 'BEGIN{printf "%.3f", t - s}')
  lty=$(awk -v t="${TGT_Y[$idx]}" -v s="${SPAWN_Y[$idx]}" 'BEGIN{printf "%.3f", t - s}')
  echo "/px4_$i $((i + 1)) 0 $ltx $lty ${ALT[$idx]} ${SPAWN_X[$idx]} ${SPAWN_Y[$idx]}" >> "$CONFIG_FILE"
done
echo "[record_demo] fleet config ($CONFIG_FILE):"; sed 's/^/    /' "$CONFIG_FILE"

# ----------------------------- source ROS ----------------------------------
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_DIR/install/local_setup.bash"
set -u

# ----------------------------- launch agent + gazebo -----------------------
echo "[record_demo] starting micro-XRCE-DDS agent"
MicroXRCEAgent udp4 -p "$AGENT_PORT" > "$LOG_DIR/agent.log" 2>&1 &
sleep 2

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
[[ -f "$WORLD_FILE" ]] || { echo "ERROR: world file not found: $WORLD_FILE (run: python3 worlds/make_demo_world.py)"; kill_all; exit 1; }
echo "[record_demo] starting Gazebo (world: $GZ_WORLD, file: $WORLD_FILE)"
gz sim -r "$WORLD_FILE" > "$LOG_DIR/gz.log" 2>&1 &

echo -n "[record_demo] waiting for Gazebo world to load"
waited=0
until gz topic -l 2>/dev/null | grep -qE "/world/$GZ_WORLD|/clock"; do
  sleep 1; echo -n "."
  waited=$((waited + 1))
  if [[ "$waited" -ge "$GZ_WAIT_TIMEOUT" ]]; then
    echo; echo "ERROR: Gazebo did not come up — see $LOG_DIR/gz.log"; kill_all; exit 1
  fi
done
echo " up."; sleep 2

# ----------------------------- spawn all drones (standalone) ---------------
for idx in "${!NAMES[@]}"; do
  i="${NAMES[$idx]}"
  echo "[record_demo] spawning drone $i at (${SPAWN_X[$idx]}, ${SPAWN_Y[$idx]}) -> target (${TGT_X[$idx]}, ${TGT_Y[$idx]}) alt ${ALT[$idx]}"
  (
    cd "$PX4_DIR" || exit 1
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD="$GZ_WORLD" \
    PX4_SYS_AUTOSTART="$AUTOSTART" \
    PX4_GZ_MODEL_POSE="${SPAWN_X[$idx]},${SPAWN_Y[$idx]}" \
    PX4_SIM_MODEL="$SIM_MODEL" \
      "$PX4_BIN" -d -i "$i" > "$LOG_DIR/px4_$i.log" 2>&1
  ) &
  sleep 3
done

echo -n "[record_demo] waiting for telemetry from all drones"
last="${NAMES[-1]}"
waited=0
until ros2 topic list 2>/dev/null | grep -q "/px4_${last}/fmu/out/vehicle_local_position"; do
  sleep 1; echo -n "."
  waited=$((waited + 1))
  if [[ "$waited" -ge 40 ]]; then echo; echo "[record_demo] (topic wait timed out — continuing)"; break; fi
done
echo

# ----------------------------- frame + record with Gazebo, then run --------
echo "============================================================================"
echo "[record_demo] ${#NAMES[@]} drones are on the ground."
echo "[record_demo] 1) Orbit/zoom the Gazebo camera to frame the scene."
echo "[record_demo] 2) In Gazebo, open the '...' menu (top-right) -> 'Video Recorder'"
echo "[record_demo]    and press its record button."
read -rp "[record_demo] 3) Then press ENTER here to launch the mission (drones take off)... "

echo "[record_demo] launching mission node"
# SITL here runs PX4 1.15 (unversioned /fmu/out topics); the nodes default to the
# 1.17 "_v1" names, so tell them to use the unversioned ones for the sim.
PX4_OUT_SUFFIX="${PX4_OUT_SUFFIX-}" MISSION_CONFIG_FILE="$CONFIG_FILE" ros2 run multi_sim mission_setpoints > "$LOG_DIR/node.log" 2>&1 &

echo "============================================================================"
echo "[record_demo] Mission running. Stop Gazebo's recorder when done, then press"
echo "[record_demo] Ctrl+C here to shut everything down."
wait

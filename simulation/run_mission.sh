#!/usr/bin/env bash
#
# run_mission.sh — interactive multi-drone launcher for the multi_sim package.
#
# Prompts for the number of drones and, per drone, its start (x y), target (x y)
# and wait_time, then:
#   1. cleans up any leftover Gazebo / PX4 / agent processes,
#   2. launches the micro-XRCE-DDS agent,
#   3. starts PX4 SITL instance 1 (which brings up Gazebo) and instances 2..N
#      (standalone, attaching to the same Gazebo world) at the requested poses,
#   4. runs the mission_setpoints node with the entered config.
#
# Ctrl+C (or the node exiting) tears the whole stack down. QGC is NOT launched.
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
CRUISE_ALT="${CRUISE_ALT:-5.0}"          # cruise altitude (m) used for every drone
CONFIG_FILE="${MISSION_CONFIG_FILE:-/tmp/mission_config.txt}"
LOG_DIR="${LOG_DIR:-/tmp/mission_logs}"
GZ_WAIT_TIMEOUT="${GZ_WAIT_TIMEOUT:-60}" # seconds to wait for Gazebo to come up
GZ_WORLD="${GZ_WORLD:-default}"          # Gazebo world (under PX4 Tools/simulation/gz/worlds)
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"

export MISSION_CONFIG_FILE="$CONFIG_FILE"

# ----------------------------- cleanup helper ------------------------------
kill_all() {
  echo
  echo "[run_mission] cleaning up Gazebo / PX4 / agent / node ..."
  pkill -9 -f 'mission_setpoints'  2>/dev/null || true
  pkill -9 -f 'bin/px4'            2>/dev/null || true
  pkill -9 -f 'px4_sitl'           2>/dev/null || true
  pkill -9 -f 'MicroXRCEAgent'     2>/dev/null || true
  pkill -9 -f 'gz sim'             2>/dev/null || true
  pkill -9 -f 'ruby .*gz'          2>/dev/null || true
  pkill -9 -f 'gz_x500'            2>/dev/null || true
  sleep 1
  echo "[run_mission] done."
}

# clean on exit / Ctrl+C, and once up-front to clear any leftovers
trap 'kill_all; exit 0' INT TERM
kill_all

# ----------------------------- sanity checks -------------------------------
[[ -x "$PX4_BIN" ]]            || { echo "ERROR: px4 binary not found/executable: $PX4_BIN"; exit 1; }
command -v MicroXRCEAgent >/dev/null || { echo "ERROR: MicroXRCEAgent not in PATH"; exit 1; }
[[ -f "$ROS_SETUP" ]]         || { echo "ERROR: ROS setup not found: $ROS_SETUP"; exit 1; }
[[ -f "$WS_DIR/install/local_setup.bash" ]] || { echo "ERROR: workspace not built: run 'colcon build' in $WS_DIR"; exit 1; }

# ----------------------------- collect input -------------------------------
read -rp "Number of drones: " N
[[ "$N" =~ ^[0-9]+$ && "$N" -ge 1 ]] || { echo "ERROR: enter a positive integer"; exit 1; }

declare -a SX SY TX TY WT
for ((i = 1; i <= N; i++)); do
  echo "--- Drone $i (namespace /px4_$i, MAV_SYS_ID $((i + 1))) ---"
  read -rp "  start  x y : " sx sy
  read -rp "  target x y : " tx ty
  read -rp "  wait_time (s): " wt
  SX[i]="$sx"; SY[i]="$sy"; TX[i]="$tx"; TY[i]="$ty"; WT[i]="$wt"
done

# ----------------------------- write config --------------------------------
# Node targets are in each drone's LOCAL frame (origin = its spawn point),
# so local_target = world_target - start.
mkdir -p "$LOG_DIR"
{
  echo "# ns sysid wait_time target_x target_y cruise_alt   (targets in local NED, relative to spawn)"
  for ((i = 1; i <= N; i++)); do
    ltx=$(awk -v t="${TX[i]}" -v s="${SX[i]}" 'BEGIN{printf "%.3f", t - s}')
    lty=$(awk -v t="${TY[i]}" -v s="${SY[i]}" 'BEGIN{printf "%.3f", t - s}')
    echo "/px4_$i $((i + 1)) ${WT[i]} $ltx $lty $CRUISE_ALT"
  done
} > "$CONFIG_FILE"

echo
echo "[run_mission] config written to $CONFIG_FILE:"
sed 's/^/    /' "$CONFIG_FILE"
echo "[run_mission] logs: $LOG_DIR/   (tail -f $LOG_DIR/px4_1.log)"
echo

# ----------------------------- source ROS ----------------------------------
# ROS/ament setup scripts reference unset variables, so relax nounset while sourcing
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1091
source "$WS_DIR/install/local_setup.bash"
set -u

# ----------------------------- launch agent --------------------------------
echo "[run_mission] starting micro-XRCE-DDS agent on udp4:$AGENT_PORT"
MicroXRCEAgent udp4 -p "$AGENT_PORT" > "$LOG_DIR/agent.log" 2>&1 &
sleep 2

# ----------------------------- launch Gazebo (standalone, once) ------------
# Start the Gazebo server+GUI ourselves (from PX4's local models, no download)
# and let EVERY PX4 instance attach in standalone mode. This avoids the race
# where the gazebo-launching PX4 instance tries to spawn its model before the
# server is ready (the "gz_bridge: Service call timed out" failure).
export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
echo "[run_mission] starting Gazebo (world: $GZ_WORLD)"
gz sim -r "$PX4_DIR/Tools/simulation/gz/worlds/$GZ_WORLD.sdf" > "$LOG_DIR/gz.log" 2>&1 &

# wait until the Gazebo world has actually loaded before spawning vehicles
echo -n "[run_mission] waiting for Gazebo world to load"
waited=0
until gz topic -l 2>/dev/null | grep -qE "/world/$GZ_WORLD|/clock"; do
  sleep 1; echo -n "."
  waited=$((waited + 1))
  if [[ "$waited" -ge "$GZ_WAIT_TIMEOUT" ]]; then
    echo; echo "ERROR: Gazebo did not come up within ${GZ_WAIT_TIMEOUT}s — see $LOG_DIR/gz.log"
    kill_all; exit 1
  fi
done
echo " up."
sleep 2

# ----------------------------- launch PX4 instances (all standalone) -------
for ((i = 1; i <= N; i++)); do
  echo "[run_mission] starting PX4 instance $i (standalone) at pose (${SX[i]}, ${SY[i]})"
  (
    cd "$PX4_DIR" || exit 1
    PX4_GZ_STANDALONE=1 \
    PX4_GZ_WORLD="$GZ_WORLD" \
    PX4_SYS_AUTOSTART="$AUTOSTART" \
    PX4_GZ_MODEL_POSE="${SX[i]},${SY[i]}" \
    PX4_SIM_MODEL="$SIM_MODEL" \
      "$PX4_BIN" -d -i "$i" > "$LOG_DIR/px4_$i.log" 2>&1
  ) &
  sleep 4
done

# ----------------------------- wait for telemetry --------------------------
echo -n "[run_mission] waiting for /px4_${N}/fmu/out/vehicle_local_position"
waited=0
until ros2 topic list 2>/dev/null | grep -q "/px4_${N}/fmu/out/vehicle_local_position"; do
  sleep 1; echo -n "."
  waited=$((waited + 1))
  if [[ "$waited" -ge 30 ]]; then echo; echo "[run_mission] (timeout waiting for topics — continuing anyway)"; break; fi
done
echo

# ----------------------------- ready: user starts the node manually --------
echo "============================================================================"
echo "[run_mission] Simulation is up with $N drone(s)."
echo "[run_mission] Now, in ANOTHER terminal, start the mission with:"
echo
echo "    cd $WS_DIR"
echo "    source $ROS_SETUP"
echo "    source $WS_DIR/install/local_setup.bash"
[[ "$CONFIG_FILE" == "/tmp/mission_config.txt" ]] \
  && echo "    ros2 run multi_sim mission_setpoints" \
  || echo "    MISSION_CONFIG_FILE=$CONFIG_FILE ros2 run multi_sim mission_setpoints"
echo
echo "[run_mission] Keep THIS terminal open. Press Ctrl+C here to stop the whole simulation."
echo "============================================================================"

# keep the sim (agent + gazebo + px4) alive until Ctrl+C (trap -> kill_all)
wait

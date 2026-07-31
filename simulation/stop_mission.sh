#!/usr/bin/env bash
#
# stop_mission.sh — force-kill every Gazebo / PX4 / agent / node process left
# behind by a mission run. Use this instead of the manual
# "ps ax | grep gz" + "kill -9" dance when a window doesn't close.
#
echo "[stop_mission] killing mission processes ..."
pkill -9 -f 'mission_setpoints'  2>/dev/null || true
pkill -9 -f 'bin/px4'            2>/dev/null || true
pkill -9 -f 'px4_sitl'           2>/dev/null || true
pkill -9 -f 'MicroXRCEAgent'     2>/dev/null || true
pkill -9 -f 'gz sim'             2>/dev/null || true
pkill -9 -f 'ruby .*gz'          2>/dev/null || true
pkill -9 -f 'gz_x500'            2>/dev/null || true
sleep 1

echo "[stop_mission] survivors (should be empty):"
pgrep -a -f 'gz sim|bin/px4|MicroXRCEAgent|mission_setpoints' || echo "    none — all clear."

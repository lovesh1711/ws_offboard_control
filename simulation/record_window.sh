#!/usr/bin/env bash
#
# record_window.sh — record a single window to an .mkv. Press q or Ctrl+C to stop.
#
# By default it finds the RViz window by name (needs xdotool) so you can't misclick
# and grab the whole screen. If xdotool isn't installed, it falls back to click-to-
# select (xwininfo) and warns if you accidentally pick the desktop.
#
# Usage:
#   ./record_window.sh                 # auto-find RViz, record it
#   ./record_window.sh out.mkv         # custom output file
#   ./record_window.sh out.mkv Gazebo  # match a different window name
#
set -uo pipefail
OUT="${1:-$HOME/mission_videos/rviz_$(date +%Y%m%d_%H%M%S).mkv}"
NAME="${2:-RViz}"
FPS="${FPS:-30}"
: "${DISPLAY:=:0}"

command -v ffmpeg >/dev/null || { echo "install ffmpeg:  sudo apt install -y ffmpeg"; exit 1; }
mkdir -p "$(dirname "$OUT")"

if command -v xdotool >/dev/null; then
  WID=$(xdotool search --name "$NAME" 2>/dev/null | tail -1)
  [[ -n "$WID" ]] || { echo "No window matching '$NAME'. Is RViz open?"; exit 1; }
  xdotool windowactivate "$WID" 2>/dev/null; sleep 0.3
  eval "$(xdotool getwindowgeometry --shell "$WID")"   # sets X Y WIDTH HEIGHT
  W="$WIDTH"; H="$HEIGHT"
else
  command -v xwininfo >/dev/null || { echo "install x11-utils:  sudo apt install -y x11-utils"; exit 1; }
  echo "(tip: 'sudo apt install -y xdotool' lets this auto-find the RViz window)"
  echo "Click the RViz window you want to record..."
  INFO=$(xwininfo)
  X=$(awk '/Absolute upper-left X/{print $NF}' <<<"$INFO")
  Y=$(awk '/Absolute upper-left Y/{print $NF}' <<<"$INFO")
  W=$(awk '/^  Width/{print $NF}'  <<<"$INFO")
  H=$(awk '/^  Height/{print $NF}' <<<"$INFO")
fi

W=$((W - W % 2)); H=$((H - H % 2))   # libx264 needs even dimensions
SCR_W=$(xdpyinfo 2>/dev/null | awk '/dimensions:/{split($2,a,"x"); print a[1]; exit}')
if [[ -n "${SCR_W:-}" && "$W" -ge "$SCR_W" ]]; then
  echo "WARNING: width ($W) spans the whole screen — you likely selected the desktop,"
  echo "         not the RViz window. Recording anyway; Ctrl+C to abort and retry."
fi

echo "Recording ${W}x${H} at (${X},${Y}) -> $OUT"
echo "Don't move/resize the window while recording. Press q or Ctrl+C to stop."
ffmpeg -y -f x11grab -framerate "$FPS" -video_size "${W}x${H}" -i "${DISPLAY}+${X},${Y}" \
       -c:v libx264 -preset veryfast -pix_fmt yuv420p "$OUT"
echo "Saved: $OUT"

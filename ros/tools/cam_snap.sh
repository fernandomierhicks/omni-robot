#!/usr/bin/env bash
# The picture server for the console's Cameras tab (omni_console/camera_snap.py) on the Pi ("omni-cam", rpi.md).
# From Git Bash, in the repo:
#
#   bash ros/tools/cam_snap.sh start [ARGS]   # copy it to the Pi and start it there in the background (port 8086)
#   bash ros/tools/cam_snap.sh stop           # stop it (the cameras are released)
#   bash ros/tools/cam_snap.sh status         # running or not, and what /cameras says
#   bash ros/tools/cam_snap.sh log            # the end of its log on the Pi
#   bash ros/tools/cam_snap.sh fake           # test patterns on this PC (Windows sim venv), no cameras, foreground
#
# ARGS go to camera_snap.py: --port 8086, --idle 20. Runs in ~/.venvs/cam (Picamera2 and pyrealsense2 both there).
# A camera opens at its first picture and closes after --idle seconds without one: ball_view and the camera head can
# still have it, but not while the tab is taking pictures.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
PORT=${CAM_SNAP_PORT:-8086}
SSH=(ssh -o BatchMode=yes)
DIR=omni_ws/cam_snap

# by pid file, not pkill -f (it would match the remote shell's own command line, rpi.md Gotchas)
stop() { "${SSH[@]}" "$HOST" "if [ -f ~/$DIR/pid ] && kill \$(cat ~/$DIR/pid) 2>/dev/null; then echo '== stopped'; else echo '== not running'; fi; rm -f ~/$DIR/pid"; }

case "${1:-}" in
    start)
        shift
        "${SSH[@]}" "$HOST" "mkdir -p ~/$DIR"
        scp -q -o BatchMode=yes "$REPO/ros/src/omni_console/omni_console/camera_snap.py" "$HOST:$DIR/"
        stop >/dev/null
        # a simple command in the background (no "cd &&" list: its subshell would hold the ssh session open)
        "${SSH[@]}" "$HOST" "cd ~/$DIR || exit 1; setsid nohup ~/.venvs/cam/bin/python camera_snap.py --port $PORT $* \
            > camera_snap.log 2>&1 < /dev/null & echo \$! > ~/$DIR/pid"
        sleep 2
        "${SSH[@]}" "$HOST" "kill -0 \$(cat ~/$DIR/pid) 2>/dev/null" \
            || { echo "== it didn't start:"; "${SSH[@]}" "$HOST" "tail -20 ~/$DIR/camera_snap.log"; exit 1; }
        echo "== camera_snap on $HOST: http://$HOST.local:$PORT/cameras   (the console's Robot > Cameras tab)"
        ;;
    stop) stop ;;
    status)
        "${SSH[@]}" "$HOST" "[ -f ~/$DIR/pid ] && kill -0 \$(cat ~/$DIR/pid) 2>/dev/null && echo '== running, pid' \$(cat ~/$DIR/pid) || echo '== not running'"
        curl -s -m 5 "http://$HOST.local:$PORT/cameras" && echo || true
        ;;
    log) "${SSH[@]}" "$HOST" "tail -40 ~/$DIR/camera_snap.log" ;;
    fake)
        shift
        exec /c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe "$REPO/ros/src/omni_console/omni_console/camera_snap.py" --fake --port "$PORT" "$@"
        ;;
    *) sed -n '2,13p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

#!/usr/bin/env bash
# The ball view on the Pi ("omni-cam", rpi.md), opened in a browser on the PC. From Git Bash, in the repo:
#
#   bash characterization/ball_view/bv.sh picam [ARGS]       # Arducam / Camera Module 3 (position from the ball's size)
#   bash characterization/ball_view/bv.sh realsense [ARGS]   # D435 colour + depth (position from the depth map)
#   bash characterization/ball_view/bv.sh synthetic          # a rendered ball, no camera (the Pi's CPU, for timing)
#   bash characterization/ball_view/bv.sh stop               # stop a view left running
#   bash characterization/ball_view/bv.sh deploy             # just copy the code to the Pi
#   bash characterization/ball_view/bv.sh fetch              # the page's saved settings, Pi -> characterization/ball_view/settings/
#   bash characterization/ball_view/bv.sh test               # this folder's tests on the PC
#
# ARGS go to ball_view.py: --mode wide, --lens wide, --size 640x480, --fps 60, --numpy, --port 8085 ...
# It deploys the detector (ros/src/omni_catch) and this folder to ~/omni_ws first, starts the server on the Pi, opens
# http://omni-cam.local:8085/ and runs in the foreground: Ctrl-C stops it. Only one process can hold a camera.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
PORT=${BV_PORT:-8085}
SSH=(ssh -o BatchMode=yes)

deploy() {
    tar -C "$REPO" -cf - --exclude='__pycache__' --exclude='.pytest_cache' --exclude='build' \
        ros/src/omni_catch/omni_catch characterization/ball_view \
    | "${SSH[@]}" "$HOST" "set -e; rm -rf ~/.bv_stage; mkdir -p ~/.bv_stage ~/omni_ws/src/omni_catch; \
        tar -C ~/.bv_stage -xf -; \
        rsync -a ~/.bv_stage/ros/src/omni_catch/omni_catch/ ~/omni_ws/src/omni_catch/omni_catch/; \
        rsync -a --delete ~/.bv_stage/characterization/ball_view/ ~/omni_ws/ball_view/; rm -rf ~/.bv_stage"
}

stop() { "${SSH[@]}" "$HOST" 'pkill -f "python3? .*ball_view[.]py" || true; pkill -f "[.]venvs/cam/bin/python.*ball_view" || true'; }

case "${1:-}" in
    bucket) shift; exec bash "$REPO/characterization/ball_view/bucket.sh" start "$@" ;;
    bucket-demo) shift; exec bash "$REPO/characterization/ball_view/bucket.sh" demo "$@" ;;
    picam|realsense|synthetic)
        src=$1; shift
        deploy
        stop
        (sleep 4; cmd.exe /c start "" "http://$HOST.local:$PORT/" >/dev/null 2>&1 || true) &
        echo "== ball view ($src) on $HOST: http://$HOST.local:$PORT/   (Ctrl-C stops it)"
        tty=(); [ -t 0 ] && tty=(-t)          # a terminal: Ctrl-C reaches the Pi; the Launcher has no terminal (Stop button)
        "${SSH[@]}" "${tty[@]}" "$HOST" "cd ~/omni_ws/ball_view && exec ~/.venvs/cam/bin/python ball_view.py --source $src --port $PORT $*"
        ;;
    stop) stop ;;
    deploy) deploy ;;
    fetch)
        mkdir -p "$REPO/characterization/ball_view/settings"
        scp -q -o BatchMode=yes "$HOST:omni_ws/ball_view_settings/*.json" "$REPO/characterization/ball_view/settings/"             && ls -1 "$REPO/characterization/ball_view/settings/" | tail -5             && echo "== saved in characterization/ball_view/settings/ (<source>.json is the latest per camera)"
        ;;
    test)
        cd "$REPO/characterization/ball_view" && /c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -m pytest -q
        ;;
    *) sed -n '2,15p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

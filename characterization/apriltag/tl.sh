#!/usr/bin/env bash
# The AprilTag lock bench (tag_lock.py) on the Pi ("omni-cam", rpi.md). From Git Bash, in the repo:
#
#   bash characterization/apriltag/tl.sh run [ARGS]    # copy it over, stop the Cameras tab's server (it holds the
#                                                       # cameras), run tag_lock.py ARGS there, fetch the results
#   bash characterization/apriltag/tl.sh fetch          # results/ from the Pi into characterization/apriltag/results
#
# ARGS: --size 0.14 --tape 1.346 --seconds 6 --only NAME... (tag_lock.py's docstring). OMNI_PI=omni-cam-eth uses the
# cable. Restart the server afterwards with: bash ros/tools/cam_snap.sh start
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
SSH=(ssh -o BatchMode=yes)
DIR=omni_ws/apriltag

fetch() { mkdir -p "$HERE/results"; scp -q -o BatchMode=yes "$HOST:$DIR/results/*.json" "$HERE/results/" && ls -t "$HERE/results" | head -3; }

case "${1:-}" in
    run)
        shift
        "${SSH[@]}" "$HOST" "mkdir -p ~/$DIR/results"
        scp -q -o BatchMode=yes "$HERE/tag_lock.py" "$REPO/ros/src/omni_console/omni_console/camera_snap.py" \
            "$REPO/ros/src/omni_console/omni_console/apriltag_view.py" "$REPO/ros/src/omni_slam/omni_slam/tags.py" "$HOST:$DIR/"
        OMNI_PI=$HOST bash "$REPO/ros/tools/cam_snap.sh" stop || true
        # each argument quoted for the remote shell (--light takes words)
        "${SSH[@]}" "$HOST" "cd ~/$DIR && ~/.venvs/cam/bin/python tag_lock.py --when $(date +%Y%m%d_%H%M%S) $(printf '%q ' "$@")"
        fetch
        ;;
    fetch) fetch ;;
    *) sed -n '2,10p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

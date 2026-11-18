#!/usr/bin/env bash
# The lidar bench on the Pi ("omni-cam", rpi.md). From Git Bash:
#
#   bash characterization/lidar/lidar.sh capture SESSION [ARGS]   # lidar_capture.py in the background on the Pi
#   bash characterization/lidar/lidar.sh status SESSION            # its log so far
#   bash characterization/lidar/lidar.sh fetch SESSION             # raw captures -> $LIDAR_DATA/SESSION (outside Dropbox)
#   bash characterization/lidar/lidar.sh ssh
#
# The Pi holds a copy (~/omni_ws/lidar: this folder's scripts + the robot's omni_console/rplidar.py), never the master;
# capture deploys first. Raw captures stay outside Dropbox (AGENTS.md): ~/lidar_data on the Pi, C:/Users/ferna/lidar_data
# here. Then, on Windows: python characterization/lidar/lidar_timing.py $LIDAR_DATA/SESSION (writes results/).
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
LIDAR_DATA=${LIDAR_DATA:-/c/Users/ferna/lidar_data}
SSH=(ssh -o BatchMode=yes)

remote() { "${SSH[@]}" "$HOST" "$*"; }

deploy() {
    tar -C "$REPO" -cf - characterization/lidar/lidar_capture.py ros/src/omni_console/omni_console/rplidar.py \
    | "${SSH[@]}" "$HOST" "set -e; mkdir -p ~/omni_ws/lidar; t=\$(mktemp -d); tar -C \$t -xf -;
        cp \$t/characterization/lidar/lidar_capture.py \$t/ros/src/omni_console/omni_console/rplidar.py ~/omni_ws/lidar/;
        rm -rf \$t"
}

case "${1:-}" in
    capture)
        s=${2:?session}; deploy
        remote "mkdir -p ~/lidar_data/$s; cd ~/omni_ws/lidar; nohup ~/.venvs/cam/bin/python -u lidar_capture.py \
            --session $s ${*:3} > ~/lidar_data/$s/capture.log 2>&1 < /dev/null & echo started pid \$!"
        ;;
    status) remote "cat ~/lidar_data/${2:?session}/capture.log; pgrep -af '[l]idar_capture' || echo '(not running)'" ;;
    fetch)
        s=${2:?session}; mkdir -p "$LIDAR_DATA"
        remote "tar -C ~/lidar_data -cf - $s" | tar -C "$LIDAR_DATA" -xf -
        echo "fetched -> $LIDAR_DATA/$s ($(ls "$LIDAR_DATA/$s" | wc -l) files)"
        ;;
    ssh) exec ssh "$HOST" ;;
    *) sed -n '2,12p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

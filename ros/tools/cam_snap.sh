#!/usr/bin/env bash
# The picture server for the console's Cameras tab (omni_console/camera_snap.py) on the Pi ("omni-cam", rpi.md),
# which also serves the lidar to the Test > Lidar page (omni_console/rplidar.py). From Git Bash, in the repo:
#
#   bash ros/tools/cam_snap.sh start [ARGS]   # copy it to the Pi, install it as a service there and (re)start it
#   bash ros/tools/cam_snap.sh stop           # stop it until the next start or boot (the cameras are released)
#   bash ros/tools/cam_snap.sh status         # running or not, and what /cameras says
#   bash ros/tools/cam_snap.sh log            # the end of its log on the Pi
#   bash ros/tools/cam_snap.sh remove         # stop it and take the service off (no longer started at boot)
#   bash ros/tools/cam_snap.sh fake           # test patterns on this PC (Windows sim venv), no cameras, foreground
#
# It runs as the systemd service omni-cam-snap (since Nov 12): started at every boot and restarted if it dies, so the
# console's pages work whenever the Pi is on; it costs nothing until a page asks. ARGS go to camera_snap.py (--idle 20,
# --webcam DEV, --lidar PORT|none) and are kept in ~/omni_ws/cam_snap/env for later boots; start with none to clear
# them. Runs in ~/.venvs/cam (Picamera2, pyrealsense2, OpenCV, pyserial and, for the AprilTag overlay,
# pupil-apriltags: `~/.venvs/cam/bin/pip install pupil-apriltags`, done Nov 4).
# A camera opens at its first picture and closes after --idle seconds without one (or a live stream): ball_view and
# the camera head can still have it, but not while the tab is taking pictures. The lidar likewise: its motor runs
# only while the Lidar page scans or spins it, and the port is released --idle s after the page goes. Benches that
# want a camera for themselves (the tag bench) stop it first. OMNI_PI=omni-cam-eth uses the cable.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
PORT=${CAM_SNAP_PORT:-8086}
SSH=(ssh -o BatchMode=yes)
DIR=omni_ws/cam_snap
UNIT=omni-cam-snap

# the Pi's address as this PC reaches it (the alias's HostName: omni-cam-eth is 192.168.0.101, omni-cam is omni-cam.local)
addr() { ssh -G "$HOST" 2>/dev/null </dev/null | awk '$1 == "hostname" {print $2; exit}'; }

# an instance started by the old pid-file way (before the service), by pid, not pkill -f (rpi.md Gotchas)
old_stop() { echo "if [ -f ~/$DIR/pid ]; then kill \$(cat ~/$DIR/pid) 2>/dev/null; rm -f ~/$DIR/pid; fi"; }

case "${1:-}" in
    start)
        shift
        "${SSH[@]}" "$HOST" "mkdir -p ~/$DIR"
        # the server, its AprilTag overlay, the SLAM's tag pose (omni_slam/tags.py) it uses, and the lidar driver
        scp -q -o BatchMode=yes "$REPO/ros/src/omni_console/omni_console/camera_snap.py" \
            "$REPO/ros/src/omni_console/omni_console/apriltag_view.py" "$REPO/ros/src/omni_slam/omni_slam/tags.py" \
            "$REPO/ros/src/omni_console/omni_console/rplidar.py" "$HOST:$DIR/"
        # the unit: as fernando, unbuffered into the log file; systemd splits $CAM_SNAP_ARGS into words
        "${SSH[@]}" "$HOST" "set -e; $(old_stop); printf 'CAM_SNAP_ARGS=%s\n' '$*' > ~/$DIR/env; \
            sudo tee /etc/systemd/system/$UNIT.service > /dev/null <<EOF
[Unit]
Description=omni camera_snap: the console's Cameras and Lidar pages (port $PORT)
After=network.target

[Service]
User=\$(id -un)
WorkingDirectory=\$HOME/$DIR
Environment=PYTHONUNBUFFERED=1
EnvironmentFile=-\$HOME/$DIR/env
ExecStart=\$HOME/.venvs/cam/bin/python \$HOME/$DIR/camera_snap.py --port $PORT \\\$CAM_SNAP_ARGS
StandardOutput=append:\$HOME/$DIR/camera_snap.log
StandardError=append:\$HOME/$DIR/camera_snap.log
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
            sudo systemctl daemon-reload; sudo systemctl enable --quiet $UNIT; sudo systemctl restart $UNIT"
        sleep 3
        "${SSH[@]}" "$HOST" "systemctl is-active --quiet $UNIT" \
            || { echo "== it didn't start:"; "${SSH[@]}" "$HOST" "tail -20 ~/$DIR/camera_snap.log; systemctl status --no-pager $UNIT | tail -5"; exit 1; }
        echo "== camera_snap on $HOST (service $UNIT, started at boot): http://$(addr):$PORT/cameras, /lidar"
        echo "   (the console's See > Cameras and Test > Lidar: Server http://$(addr):$PORT)"
        ;;
    stop) "${SSH[@]}" "$HOST" "$(old_stop); sudo systemctl stop $UNIT 2>/dev/null; echo '== stopped (it starts again at the next boot, or with start)'" ;;
    status)
        "${SSH[@]}" "$HOST" "echo \"== service $UNIT: \$(systemctl is-active $UNIT), at boot: \$(systemctl is-enabled $UNIT 2>/dev/null || echo no)\""
        curl -s -m 5 "http://$(addr):$PORT/cameras" && echo || true
        ;;
    log) "${SSH[@]}" "$HOST" "tail -40 ~/$DIR/camera_snap.log" ;;
    remove) "${SSH[@]}" "$HOST" "$(old_stop); sudo systemctl disable --now $UNIT 2>/dev/null; sudo rm -f /etc/systemd/system/$UNIT.service; sudo systemctl daemon-reload; echo '== removed'" ;;
    fake)
        shift
        exec /c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe "$REPO/ros/src/omni_console/omni_console/camera_snap.py" --fake --port "$PORT" "$@"
        ;;
    *) sed -n '2,20p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

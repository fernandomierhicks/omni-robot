#!/usr/bin/env bash
# The D435 on the Pi ("omni-cam", rpi.md): run this folder's steps there. From Git Bash:
#
#   bash characterization/realsense/rs_pi.sh setup           # once: venv ~/.venvs/rs + pyrealsense2 (aarch64 wheel)
#   bash characterization/realsense/rs_pi.sh usb             # does the Pi see it, at what USB speed (no librealsense)
#   bash characterization/realsense/rs_pi.sh info            # rs_info.py: device, firmware, modes, geometry
#   bash characterization/realsense/rs_pi.sh timing [ARGS]   # rs_timing.py: fps, drops, jitter, latency per mode
#   bash characterization/realsense/rs_pi.sh check [S]       # both, then the fetched results: a one-screen verdict
#   bash characterization/realsense/rs_pi.sh run SCRIPT ...  # any other step, e.g. run rs_depth_plane.py
#   bash characterization/realsense/rs_pi.sh fetch           # copy the Pi's results/ to results/omni-cam/
#   bash characterization/realsense/rs_pi.sh ssh
#
# Like pi.sh: the Pi holds a copy of this folder (~/omni_ws/realsense), never the master; every command
# deploys first. Results are written on the Pi and fetched into results/omni-cam/ (the Windows ones stay in
# results/); `RS_RESULTS=results/omni-cam python rs_report.py` reports on them.
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
DIR=characterization/realsense
RW='~/omni_ws/realsense'
PY='~/.venvs/rs/bin/python'
SSH=(ssh -o BatchMode=yes)

remote() { "${SSH[@]}" "$HOST" "$*"; }

deploy() {
    tar -C "$REPO/characterization" -cf - --exclude='__pycache__' --exclude='.pytest_cache' --exclude='results' \
        --exclude='*.ps1' --exclude='*.cmd' realsense \
    | "${SSH[@]}" "$HOST" "set -e; mkdir -p ~/omni_ws; tar -C ~/omni_ws -xf -"
}

fetch() {
    mkdir -p "$REPO/$DIR/results/omni-cam"
    "${SSH[@]}" "$HOST" "cd $RW 2>/dev/null && tar -cf - results 2>/dev/null" \
        | tar -C "$REPO/$DIR/results/omni-cam" --strip-components=1 -xf - 2>/dev/null || true
    echo "fetched -> $DIR/results/omni-cam ($(ls "$REPO/$DIR/results/omni-cam" | wc -l) files)"
}

usb() {
    remote 'echo "== lsusb"; lsusb | grep -i -E "8086|intel|realsense" || echo "no Intel device on the bus";
            echo "== lsusb -t (speed per port: 5000M = USB 3, 480M = USB 2)"; lsusb -t;
            echo "== kernel"; sudo dmesg | grep -i -E "8086|realsense|usb 2-|uvc" | tail -12;
            echo "== video nodes"; ls /dev/video* 2>/dev/null | tr "\n" " "; echo;
            echo "== power"; vcgencmd get_throttled 2>/dev/null || true;
            echo "== usb autosuspend / max current"; grep -H . /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true;
            vcgencmd get_config usb_max_current_enable 2>/dev/null || true'
}

case "${1:-}" in
    setup)
        remote 'set -e; [ -d ~/.venvs/rs ] || python3 -m venv --system-site-packages ~/.venvs/rs;
                ~/.venvs/rs/bin/pip install -q --upgrade pip; ~/.venvs/rs/bin/pip install -q pyrealsense2;
                ~/.venvs/rs/bin/python -c "import pyrealsense2 as rs, numpy; print(\"pyrealsense2\", rs.__version__, \"numpy\", numpy.__version__)"'
        ;;
    usb) usb ;;
    info) deploy; remote "cd $RW && $PY rs_info.py ${*:2}"; fetch ;;
    timing) deploy; remote "cd $RW && $PY rs_timing.py ${*:2}"; fetch ;;
    check) deploy; remote "cd $RW && $PY rs_pi_check.py ${*:2}"; fetch ;;
    run) deploy; remote "cd $RW && $PY ${*:2}"; fetch ;;
    fetch) fetch ;;
    ssh) exec ssh "$HOST" ;;
    *) sed -n '2,15p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

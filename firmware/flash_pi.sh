#!/usr/bin/env bash
# Build the robot's firmware on the PC and flash the Teensy 4.1 plugged into the Pi (attended sessions only).
# From Git Bash:  bash firmware/flash_pi.sh [env]        (env: arm_bench (default: a bare bench Teensy), arm, catch)
# If the console's robot step is running (mcu_link holds the Teensy's port) it is stopped for the flash and started
# again with the same arguments. Needs teensy_loader_cli on the Pi (apt teensy-loader-cli; sudo: no udev rule).
# The build workspace is outside Dropbox (PlatformIO's lock files fail in a Dropbox folder: Errno 22).
set -euo pipefail
HOST=${OMNI_PI:-omni-cam}
ENV=${1:-arm_bench}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PIO="$USERPROFILE/.platformio/penv/Scripts/pio.exe"
SSH=/c/Windows/System32/OpenSSH/ssh.exe
SCP=/c/Windows/System32/OpenSSH/scp.exe
export PLATFORMIO_WORKSPACE_DIR=${PLATFORMIO_WORKSPACE_DIR:-$USERPROFILE/pio_ws/omni_fw}

(cd "$HERE" && "$PIO" run -e "$ENV" | grep -E 'build id|FLASH:|RAM1:|SUCCESS|FAILED|error')
HEX="$PLATFORMIO_WORKSPACE_DIR/build/$ENV/firmware.hex"
"$SSH" -o BatchMode=yes "$HOST" 'mkdir -p ~/fw_teensy'
"$SCP" -q -o BatchMode=yes "$HEX" "$HOST:fw_teensy/firmware.hex"
"$SSH" -o BatchMode=yes "$HOST" 'bash -s' <<'EOF'
set -e
api() { curl -s -m 5 "localhost:8080/api/$1" "${@:2}"; }
STATE=$(api state || true)
RUNNING=$(python3 -c 'import json,sys
try:
    c = json.loads(sys.argv[1]).get("current") or {}
except Exception:
    c = {}
print(json.dumps({"step": c["step"], "args": c.get("args") or {}}) if c.get("step") and c.get("state") in ("running", "starting", None) else "")' "$STATE")
if [ -n "$RUNNING" ]; then
    echo "stopping the console's step: $RUNNING"
    api stop -X POST -H 'Content-Type: application/json' -d '{}' >/dev/null
    for i in $(seq 1 40); do pgrep -f omni_cpp/mcu_link >/dev/null || break; sleep 0.25; done
fi
P=$(ls /dev/serial/by-id/usb-Teensyduino_* 2>/dev/null | head -1 || true)
if [ -n "$P" ]; then
    if fuser "$P" >/dev/null 2>&1; then echo "port busy:"; fuser -v "$P"; exit 1; fi
    stty -F "$P" 134 || true                    # 134 baud: the Teensy reboots into its HalfKay bootloader
    sleep 1
else
    echo "no Teensy serial port: press its button if it is already in the bootloader"
fi
sudo teensy_loader_cli --mcu=TEENSY41 -w -v ~/fw_teensy/firmware.hex
for i in $(seq 1 20); do ls /dev/serial/by-id/usb-Teensyduino_* >/dev/null 2>&1 && break; sleep 0.5; done
ls -l /dev/serial/by-id/ | grep -i teensy || echo "the Teensy's port did not come back"
if [ -n "$RUNNING" ]; then
    sleep 1
    echo "starting the console's step again"
    api start -X POST -H 'Content-Type: application/json' -d "$RUNNING"; echo
fi
EOF

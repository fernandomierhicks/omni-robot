#!/usr/bin/env bash
# Build this bench firmware on the PC and flash the UNO R4 WiFi plugged into the Pi (attended sessions only).
# From Git Bash:  bash firmware/bench/omni_fw_uno_r4/flash_pi.sh [env]     (env: uno_r4_wifi (default) or profile:
#                 platformio.ini)
# Needs Arduino's bossac for linux-aarch64 on the Pi at ~/tools/bossac/bin/bossac (README.md here).
# The board's port: the ESP32-S3 bridge's (a sketch with NO_USB) or the RA4M1's own (this firmware): any Arduino one.
set -euo pipefail
HOST=${OMNI_PI:-omni-cam}
ENV=${1:-uno_r4_wifi}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PIO="$USERPROFILE/.platformio/penv/Scripts/pio.exe"

(cd "$HERE" && "$PIO" run -e "$ENV" | grep -E 'build id|RAM:|Flash:|SUCCESS|FAILED|error')
ssh -o BatchMode=yes "$HOST" 'mkdir -p ~/fw_bench'
scp -q -o BatchMode=yes "$HERE/.pio/build/$ENV/firmware.bin" "$HERE/link_probe.py" "$HERE/teleop_bench.py" \
    "$HERE/ros_bench.sh" "$HERE/console_bench.sh" "$HOST:fw_bench/"
# Flashing goes through the ESP32-S3 bridge (2341:1002): a 1200-baud touch on it resets the RA4M1 into its ROM
# bootloader, bossac writes and resets. While this firmware runs, the USB is the RA4M1's own (2341:006D): a touch on
# it makes the firmware stop the motors and hand the USB back to the bridge (src/main.cpp), then the bridge is touched.
# Refuses if a process holds the port (mcu_link): stop it first
ssh -o BatchMode=yes "$HOST" 'set -e; touch1200() { stty -F $1 1200; (exec 3<>$1; sleep 0.1) || true; }; \
    P=$(ls /dev/serial/by-id/usb-Arduino* | head -1); echo "port $P"; \
    if fuser $P >/dev/null 2>&1; then echo "port busy:"; fuser -v $P; exit 1; fi; \
    if lsusb | grep -q 2341:006d; then \
        echo "native USB: handing over to the bridge"; touch1200 $P; \
        for i in $(seq 1 20); do sleep 0.5; lsusb | grep -q 2341:1002 && break; done; sleep 1; \
        P=$(ls /dev/serial/by-id/usb-Arduino_UNO_WiFi_R4_CMSIS-DAP* | head -1) || { echo "no bridge"; exit 1; }; fi; \
    touch1200 $P; sleep 3; \
    B=$(ls -t /dev/ttyACM* | head -1); echo "bootloader on $B ($(lsusb | grep -o "2341:[0-9a-f]*" | tr "\n" " "))"; \
    ~/tools/bossac/bin/bossac --port=$(basename $B) -U -e -w -R ~/fw_bench/firmware.bin 2>&1 | grep -E "Write|Done|rror|No device"; \
    sleep 3; ls -l /dev/serial/by-id/ | grep -i arduino'

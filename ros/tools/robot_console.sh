#!/usr/bin/env bash
# The operator console on the robot computer, as on the real robot (omni_console/systemd/README.md): systemd starts
# omni_console's supervisor at boot; it serves the page on port 8080 (and https on 8443) and starts and stops the
# robot's software from the page (Robot > System: config/robot_steps.json, step "robot" = omni_bringup
# hardware.launch.py). From Git Bash, in the repo, with the workspace already deployed and built there:
#
#   bash ros/tools/robot_console.sh install    # write the unit from systemd/omni-console.service, enable, start
#   bash ros/tools/robot_console.sh status     # the service, and what the page server answers
#   bash ros/tools/robot_console.sh log        # the supervisor's log (each step's own log: ~/.omni/logs/steps/)
#   bash ros/tools/robot_console.sh restart | stop | remove
#
# OMNI_PI (default omni-cam): the robot computer. OMNI_AUTOSTART=robot starts the robot's software at boot too (the
# real robot's setting); without it (the Pi bench: the MCU isn't always plugged in) nothing starts until the page asks.
# The supervisor alone costs next to nothing: it sleeps until the page asks (the page draws on the viewing device).
set -euo pipefail

HOST=${OMNI_PI:-omni-cam}
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
SSH=(ssh -o BatchMode=yes)
UNIT=omni-console
AUTOSTART=${OMNI_AUTOSTART:-}

addr() { ssh -G "$HOST" 2>/dev/null </dev/null | awk '$1 == "hostname" {print $2; exit}'; }

case "${1:-}" in
    install)
        scp -q -o BatchMode=yes "$REPO/ros/src/omni_console/systemd/omni-console.service" "$HOST:/tmp/$UNIT.service"
        # the repo's unit is the Jetson's (user omni, --autostart robot): this computer's user and home, and the
        # autostart only when asked. The page talks to the robot through rosbridge, so the robot's DDS stays on this
        # computer (localhost discovery) and can't meet another machine's ROS graph on the LAN.
        "${SSH[@]}" "$HOST" "AUTOSTART='$AUTOSTART' bash -s" <<'EOF'
set -e
[ -f ~/omni_ws/install/omni_console/share/omni_console/config/robot_steps.json ] \
    || { echo "omni_console isn't built in ~/omni_ws (deploy and build first)"; exit 1; }
sed -e "s/^User=omni$/User=$(id -un)/" -e "s#/home/omni#$HOME#g" /tmp/omni-console.service > /tmp/omni-console.unit
if [ -z "$AUTOSTART" ]; then sed -i 's/ --autostart robot//' /tmp/omni-console.unit
else sed -i "s/--autostart robot/--autostart $AUTOSTART/" /tmp/omni-console.unit; fi
sed -i 's/^Environment=PYTHONUNBUFFERED=1$/&\nEnvironment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST/' /tmp/omni-console.unit
sudo install -m 644 /tmp/omni-console.unit /etc/systemd/system/omni-console.service
rm -f /tmp/omni-console.service /tmp/omni-console.unit
# The robot step runs as this (regular) user under a system service, not a login session: with logind's default
# RemoveIPC=yes, the user's /dev/shm is deleted when their last SSH session ends, cutting Fast DDS's shared memory from
# under the running nodes (Nov 12: the console's service stopped answering). Keep it.
if [ ! -f /etc/systemd/logind.conf.d/omni-keep-ipc.conf ]; then
    sudo mkdir -p /etc/systemd/logind.conf.d
    printf '[Login]\nRemoveIPC=no\n' | sudo tee /etc/systemd/logind.conf.d/omni-keep-ipc.conf >/dev/null
    sudo systemctl restart systemd-logind
fi
sudo systemctl daemon-reload
sudo systemctl enable --quiet omni-console
if ss -ltnH 'sport = :8080' | grep -q . && ! systemctl is-active --quiet omni-console; then
    echo "== enabled for boot, but port 8080 is taken now (another console?): not started. It starts at the next boot,"
    echo "   or with: bash ros/tools/robot_console.sh restart, once 8080 is free."
    ss -ltnpH 'sport = :8080' 2>/dev/null || true
else
    sudo systemctl restart omni-console
    sleep 3
    systemctl is-active --quiet omni-console && echo "== started" || { journalctl -u omni-console -n 20 --no-pager; exit 1; }
fi
EOF
        echo "== the console on $HOST: http://$(addr):8080   (service $UNIT, started at boot${AUTOSTART:+, autostart $AUTOSTART})"
        ;;
    status)
        "${SSH[@]}" "$HOST" "ipc=\$(systemd-analyze cat-config systemd/logind.conf 2>/dev/null | grep -E '^RemoveIPC=' | tail -1); \
            echo \"== service $UNIT: \$(systemctl is-active $UNIT), at boot: \$(systemctl is-enabled $UNIT 2>/dev/null || echo no), \${ipc:-RemoveIPC=yes (the default: install sets no)}\""
        curl -s -m 20 "http://$(addr):8080/api/state"           # .local can take ~11 s to resolve from Windows | head -c 600 && echo || echo "(the page server doesn't answer)"
        ;;
    log) "${SSH[@]}" "$HOST" "journalctl -u $UNIT -n 60 --no-pager" ;;
    restart) "${SSH[@]}" "$HOST" "sudo systemctl restart $UNIT && echo '== restarted'" ;;
    stop) "${SSH[@]}" "$HOST" "sudo systemctl stop $UNIT && echo '== stopped (it starts again at the next boot)'" ;;
    remove) "${SSH[@]}" "$HOST" "sudo systemctl disable --now $UNIT 2>/dev/null; sudo rm -f /etc/systemd/system/$UNIT.service; sudo systemctl daemon-reload; echo '== removed'" ;;
    *) sed -n '2,16p' "${BASH_SOURCE[0]}"; exit 1 ;;
esac

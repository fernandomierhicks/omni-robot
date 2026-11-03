#!/usr/bin/env bash
# The whole arm robot on the Pi (side:=robot), the sim on the PC (WSL, side:=sim), over the Ethernet cable; the Pi's CPU
# per process and phase (design.md 4.10, "The whole arm robot on the Pi 4"). Run from Git Bash:
#
#   bash characterization/pi_camera/arm_split.sh LABEL [launch args...]
#
# env  PHASES (MANUAL,TIDY,PATROL: ros/tools/arm_load.py's phases)  PHASE (s per phase, 120)  NAV_PROFILE (pi; full | lean | pi)
#      CLK ("clock:=wall" or "clock:=sim rtf:=0.7 clock_period:=0.01")  FACE (1; 0 = no face display on the Pi)
#      PROFILE (1: cProfile arm_node into logs/<tag>_arm_node.prof)  PC_IP / PI_IP / PI (the cable's addresses, 192.168.0.x)
# Needs once (admin PowerShell on the PC, rpi.md): Hyper-V firewall rules for TCP 5761-5773 and UDP 7400-32999 from the
# cable's network. Results: results/pi_arm_<tag>.json (+ logs/<tag>_*); cut per phase with ros/tools/arm_load_report.py.
# Runs on its own ROS domain (43) and ports (5771 / 5773): it can run beside an interactive stack. It stops only its own
# processes, and clears the Pi's stale Fast DDS shared-memory files (left by killed processes they hang the next run's
# lifecycle nodes: slam_toolbox never configures and Nav2 never gets a map frame).
set -uo pipefail
LABEL=${1:?usage: arm_split.sh LABEL [launch args...]}; shift
EXTRA="$*"
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WREPO=$(echo "$REPO" | sed -E 's#^/([a-zA-Z])/#/mnt/\L\1/#')
PI=${PI:-omni-cam-eth}
PC_IP=${PC_IP:-192.168.0.100}; PI_IP=${PI_IP:-192.168.0.101}
DOM=43; MCU=5771; SERVO=5773
PHASE=${PHASE:-120}; PHASES=${PHASES:-MANUAL,TIDY,PATROL}; SECS=${SECS:-3000}
ARGS="map:=house objects:=toys n_objects:=6 ${CLK:-clock:=wall} nav_profile:=${NAV_PROFILE:-pi} mcu_port:=$MCU servo_port:=$SERVO console:=false viewer:=false foxglove:=false fast:=true $EXTRA"
TAG="$(printf '%s' "$LABEL" | tr -c 'A-Za-z0-9' '_' | head -c 40)_$(date +%H%M%S)"
RES="$REPO/characterization/pi_camera/results"
mkdir -p "$RES/logs"
SSH=(ssh -o BatchMode=yes -o ServerAliveInterval=10)
wsl_side() { MSYS_NO_PATHCONV=1 /c/Windows/System32/wsl.exe -d Ubuntu-24.04 -- env PC_REPO="$WREPO" PI_PEER="$PI_IP" bash "$WREPO/characterization/pi_camera/wsl_side.sh" "$@"; }
echo "== $LABEL ($TAG): $ARGS"

"${SSH[@]}" $PI 'rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*'
# the robot side on the Pi first (mcu_link retries until the bridge listens)
PIENV="source /opt/ros/jazzy/setup.bash; cd ~/omni_ws && source install/setup.bash && export ${PROFILE:+OMNI_PROFILE=/tmp/arm_node.prof} ROS_DOMAIN_ID=$DOM ROS_STATIC_PEERS=$PC_IP OMNI_FIRMWARE_DIR=~/omni_ws/firmware"
"${SSH[@]}" $PI "$PIENV && (setsid nohup ros2 launch omni_bringup arm.launch.py side:=robot mcu_host:=$PC_IP $ARGS >/tmp/pi_arm.log 2>&1 & echo \$! >/tmp/pi_arm.pid)"
sleep 5
# the sim side in WSL, in its own process group so it can be stopped alone
wsl_side start "$TAG" $ARGS
sleep 25
# the face's display on the Pi (headless: SDL's dummy drivers; espeak voice)
[ "${FACE:-1}" = 1 ] && "${SSH[@]}" $PI "$PIENV && (SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy setsid nohup python3 -m omni_face.display --listen none --voice espeak >/tmp/pi_face.log 2>&1 &)"
# the Pi's monitor (host + per-process CPU, no ROS: it hung in rclpy start-up on an overloaded Pi) and the phase driver
"${SSH[@]}" $PI "$PIENV && python3 tools/pi_keepup.py pi_camera/results/pi_arm_$TAG.json --no-ros --seconds $SECS --window 5 --raw --label '$LABEL'" > "$RES/logs/${TAG}_keepup.txt" 2>&1 &
KEEP=$!
wsl_side drive "$TAG" --phase "$PHASE" --phases "$PHASES" 2>&1 | tee "$RES/logs/${TAG}_driver.txt"
sleep 3; "${SSH[@]}" $PI 'kill -INT $(pgrep -f "[t]ools/pi_keepup.py")'; wait $KEEP

# stop our own processes only
"${SSH[@]}" $PI 'pkill -INT -f "[o]mni_face.display"; sleep 1; pkill -9 -f "[o]mni_face.display"; kill -INT -- -$(cat /tmp/pi_arm.pid) 2>/dev/null; sleep 6; kill -9 -- -$(cat /tmp/pi_arm.pid) 2>/dev/null; rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*; true'
wsl_side stop "$TAG"
scp -q -o BatchMode=yes "$PI:omni_ws/pi_camera/results/pi_arm_$TAG.json" "$RES/" 2>/dev/null
scp -q -o BatchMode=yes "$PI:/tmp/pi_arm.log" "$RES/logs/${TAG}_pi.log" 2>/dev/null
scp -q -o BatchMode=yes "$PI:/tmp/arm_node.prof" "$RES/logs/${TAG}_arm_node.prof" 2>/dev/null
scp -q -o BatchMode=yes "$PI:/tmp/pi_face.log" "$RES/logs/${TAG}_face.log" 2>/dev/null
wsl_side copy "$TAG"
cat "$RES/logs/${TAG}_keepup.txt"
echo "== results: $RES/pi_arm_$TAG.json, logs/${TAG}_*"

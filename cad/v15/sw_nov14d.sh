#!/bin/bash
# Nov 14 round c (revised): the beam's front screws up from below. Part STEPs, the SolidWorks rebuild and its checks, then the build123d checks
cd "$(dirname "$0")"
start=$(date +%s)
/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B tools/sw_robot_parts.py > sw_parts_run.log 2>&1
rc=$?; echo "sw_robot_parts -> exit $rc ($(( $(date +%s) - start )) s)"
[ $rc -ne 0 ] && { tail -5 sw_parts_run.log; echo "CHAIN STOPPED"; exit $rc; }
./regen_nov14b.sh > regen_d.out 2>&1 &
./sw_nov14b.sh || { echo "CHAIN STOPPED"; wait; exit 1; }
cd tools
for st in "test paths" "interference"; do s=$(date +%s); /c/Users/ferna/cadenv/Scripts/python.exe sw_robot.py $st > "../sw_b_check_${st// /_}.log" 2>&1; echo "SW [$st] -> exit $? ($(( $(date +%s) - s )) s)"; done
echo "SW CHECKS DONE"
wait
cat ../regen_d.out
echo "ALL DONE"

#!/bin/bash
# Nov 14 round c: beam front screws, motor-plate harness hole (no tie tunnel), taller tower tabs. Part STEPs, then the SolidWorks rebuild and
# its checks; the build123d checks / exports run alongside once the part STEPs are written
cd "$(dirname "$0")"
start=$(date +%s)
/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B tools/sw_robot_parts.py > sw_parts_run.log 2>&1
rc=$?; echo "sw_robot_parts -> exit $rc ($(( $(date +%s) - start )) s)"; tail -2 sw_parts_run.log | cut -c1-200
[ $rc -ne 0 ] && { echo "CHAIN STOPPED"; exit $rc; }
./regen_nov14b.sh > regen_c.out 2>&1 &
./sw_nov14b.sh || { echo "CHAIN STOPPED"; wait; exit 1; }
cd tools
for st in "test paths" "interference"; do s=$(date +%s); /c/Users/ferna/cadenv/Scripts/python.exe sw_robot.py $st > "../sw_b_check_${st// /_}.log" 2>&1; echo "SW [$st] -> exit $? ($(( $(date +%s) - s )) s)"; done
echo "SW CHECKS DONE"
wait
cat ../regen_c.out
echo "ALL DONE"

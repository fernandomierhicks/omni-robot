#!/bin/bash
# Nov 14: the harness on the mid deck. Part STEPs (+ the harness group), the SolidWorks groups that changed (deck relief, harness), the top
# assembly and its checks; alongside: harness.py all, then the pod / deck / robot STEPs
cd "$(dirname "$0")"
PY=/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe
SW=/c/Users/ferna/cadenv/Scripts/python.exe
s=$(date +%s); $PY -B tools/sw_robot_parts.py > sw_parts_run.log 2>&1; rc=$?
echo "sw_robot_parts -> exit $rc ($(( $(date +%s) - s )) s)"; [ $rc -ne 0 ] && { tail -8 sw_parts_run.log; echo "CHAIN STOPPED"; exit $rc; }
( cd tools; for st in "harness.py all" "pod_mid.py all" "chassis_mid.py all" "robot_step.py"; do set -- $st; s=$(date +%s); $PY -B $st > ../regen_${1%.py}.log 2>&1; echo "$st -> exit $? ($(( $(date +%s) - s )) s)"; done; echo "BUILD123D DONE" ) > regen_h.out 2>&1 &
cd tools
n=0
for stages in "import" "group_frame group_mounts" "group_bumper group_fasteners" "group_harness" "robot" "test paths" "interference"; do
  n=$((n+1)); s=$(date +%s)
  $SW sw_robot.py $stages > ../sw_h_$n.log 2>&1; rc=$?
  echo "SW [$stages] -> exit $rc ($(( $(date +%s) - s )) s)"
  if [ $rc -ne 0 ]; then tail -15 ../sw_h_$n.log; echo "SW CHAIN STOPPED"; break; fi
  [ "$stages" = "robot" ] && echo "SW ASSEMBLY DONE"
done
wait
cat ../regen_h.out
echo "ALL DONE"

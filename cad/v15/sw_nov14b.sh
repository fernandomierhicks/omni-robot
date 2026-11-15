#!/bin/bash
# Nov 14 round b: rebuild the SolidWorks assembly (changed parts: pods, wheel, motor, deck, bumper, fins, fasteners). One process per call
# (SolidWorks leaks memory); stops at the first failure. upper / electronics / arm groups are unchanged and not rebuilt
PY=/c/Users/ferna/cadenv/Scripts/python.exe
cd "$(dirname "$0")/tools"
n=0
for stages in "import" "roller wheel motor unsprung static" "pod" "group_frame group_mounts" "group_bumper group_reference" "group_fasteners" "robot"; do
  n=$((n+1)); start=$(date +%s)
  $PY sw_robot.py $stages > ../sw_b_$n.log 2>&1
  rc=$?
  echo "SW [$stages] -> exit $rc ($(( $(date +%s) - start )) s)"
  if [ $rc -ne 0 ]; then tail -15 ../sw_b_$n.log; echo "SW CHAIN STOPPED"; exit $rc; fi
done
echo "SW ASSEMBLY DONE"

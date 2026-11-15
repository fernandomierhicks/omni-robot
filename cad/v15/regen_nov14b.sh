#!/bin/bash
# Nov 14 round b: TPU light gray, motor plug up, outer fork r 21 at the axle, tower rails rounded, deck: open between the arms, accessory
# holes, straight side fins, fin tie slots. Sequential (memory)
PY=/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe
cd "$(dirname "$0")/tools"
for step in "pod_mid.py all" "chassis_mid.py all" "robot_step.py"; do
  set -- $step
  name=${1%.py}
  start=$(date +%s)
  $PY -B $step > ../regen_${name}.log 2>&1
  echo "$step -> exit $? ($(( $(date +%s) - start )) s)"
done
echo REGEN DONE

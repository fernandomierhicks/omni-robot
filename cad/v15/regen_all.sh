#!/bin/bash
# v15: regenerate every STEP the robot needs (Nov 14 2026), sequentially (parallel builds ran out of memory before)
PY=/c/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe
cd "$(dirname "$0")/tools"
for step in "wheel_motor.py all" "coupon_687.py all" "coupon_627.py all" "pod_mid.py all" "chassis_mid.py all" "upper.py export" "robot_step.py"; do
  set -- $step
  name=${1%.py}
  $PY -B $step > ../regen_${name}.log 2>&1
  echo "$step -> exit $?"
done
echo REGEN DONE

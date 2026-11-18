# Superseded study: P4 pods on hollow-triangle chassis nodes (Nov 14 2026)

Not adopted. The first v15 question: how cad/v14's Pod_reimagined_4 (side frames plus an inboard beam) would sit on four hollow printed
triangles, one per pod. It grew the robot to Ø481, moved the wheels and fins, and gave no way to take a pod out with the second floor on.
Fernando's mid deck (the rest of cad/v15) replaced it the same day.

Kept for reference, frozen as it was on Nov 14:

- `tools/chassis_tri.py`: the triangles and the robot-level checks.
- `tools/pod_p4.py`: P4 with a shorter back (its side frames and the deck stand-in). v15's `tools/pod_p4.py` is now only the fork and beam.
- `tools/chassis.py`, `tools/first_floor.py`: v14's frame, fins and lidar constants as v15 had them (the mid deck's fins are `tools/fins.py` now).
- `validation/`, `step_export/`: its checks, renders and STEPs.

These scripts are not maintained against `cad/v15/tools`, which has moved on (no module patching, layout F in `base.py`, the pod libraries
trimmed), so they do not run as they are. The results are in the outputs here.

Main findings (from its checks):

| | |
| --- | --- |
| Robot diameter | Ø481 (v14 Ø450): the outer frames' back corners set it, not the wheels |
| Slide | −25 (v14 −20): 16.8 mm between the paired pods |
| Triangles | 1612 cm³ (v14's frame 1141) |
| Pod service | a closed triangle stops a pod sliding out radially |
| Hollow | cut from the rollers at one angle, not the wheel's revolved envelope: its clearances near the wheels are not reliable |

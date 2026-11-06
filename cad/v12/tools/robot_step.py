"""One STEP of the whole v12 robot for review (Nov 4 2026, round 14): every printed part in the robot frame, at the pods' nominal pose.
Printed parts only (Fernando's STEP rule: no fasteners, bearings, bought or vendor parts).

    python -B robot_step.py        (omni-robot-sim interpreter; writes step_export/robot_assembly.step)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis as C                                                    # noqa: E402
import first_floor as FF                                               # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
from step_colour import write_step                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'robot_assembly.step'
POD_PRINTED = ('pod_fork_outer', 'pod_fork_inner', 'pod_frame_inner', 'pod_keel_outer', 'pod_sleeve', 'wheel_hub', 'wheel_cartridge',
               'roller_core', 'roller_tread')


def main():
    items = {}
    parts, _, _, _ = C.build(FF.pod_geometry())
    for k, s in parts.items():
        items[f'chassis/{k}'] = (s, C.colour(k))
    sh, _ = L.posed(L.build_static()[0], P.wheel_unit(), 0.0)
    pod = {k: s for k, s in sh.items() if k.startswith(POD_PRINTED)}
    for i in range(4):
        for k, s in pod.items():
            items[f'{B.NAMES[i]}/{k}'] = (B.place3d(s, i), L.colour(k))
    printed, _, _, _, _ = U.build()
    for k, s in printed.items():
        items[f'upper/{k}'] = (s, U.colour(k))
    write_step(items, OUT, 'robot_v12')
    print('wrote', OUT, len(items), 'parts', flush=True)


if __name__ == '__main__':
    main()

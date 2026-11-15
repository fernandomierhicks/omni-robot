"""One STEP of the whole v15 robot for review: every printed part in the robot frame, at the pods' nominal pose. Printed parts only
(Fernando's STEP rule: no fasteners, bearings, bought or vendor parts).

    python -B robot_step.py        (omni-robot-sim interpreter; writes step_export/robot_assembly.step)

v15 (Nov 14 2026): the MID-DECK robot: chassis_mid.py's deck (four pieces), bumper, lidar pedestal and fins; pod_mid.py's pods (fork, hangers,
beam, towers, rings, shims, retainer and the wheel's printed parts) at layout F; upper.py's second floor and everything on it (unchanged from
v14 in place: the floor stays at z 176.5). The hardware-complete assembly of the base is step_export/chassis_mid/v15_mid_base_assembly.step.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import chassis_mid as CM                                               # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import pod_rx as R                                                     # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
import palette as PAL                                                  # noqa: E402
from step_colour import write_step                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'robot_assembly.step'
POD_PRINTED = ('pod_fork_outer', 'pod_fork_inner', 'rx_hanger', 'rx_beam', 'rx_tower', 'rx_ring', 'rx_shim', 'pod_retainer', 'wheel_hub',
               'wheel_cartridge', 'roller_core', 'roller_sleeve', 'roller_tread')


def main():
    items = {}
    fp = CM.pod_build()[4]
    parts, _, _ = CM.build(fp)
    for k, s in parts.items():
        items[f'base/{k}'] = (s, PAL.ORANGE if k.startswith('bumper') else PAL.WHITE)
    for n, (st, mv) in CM.pods_placed(0.0).items():
        for k, s in {**st, **mv}.items():
            if k.startswith(POD_PRINTED):
                items[f'{n}/{k}'] = (s, R.colour(k))
    printed, bought, _, _, _ = U.build()
    for k, s in printed.items():
        items[f'upper/{k}'] = (s, U.colour(k))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    write_step(items, OUT, 'robot_v15')
    print('wrote', OUT, len(items), 'parts', flush=True)


if __name__ == '__main__':
    main()

"""One STEP of the whole robot for review (v12 round 14; v13 Nov 5: the D435 on the riser): every printed part in the robot frame, at the
pods' nominal pose. Printed parts only (Fernando's STEP rule: no fasteners, bearings, bought or vendor parts).

    python -B robot_step.py        (omni-robot-sim interpreter; writes step_export/robot_assembly.step and, v13, step_export/
                                    d435_mount_review.step: the printed parts round the mount plus the D435's own model and its plug + lead
                                    envelope, which the mount cannot be judged without; drop it if only printed parts are wanted)
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
    printed, bought, _, _, _ = U.build()
    for k, s in printed.items():
        items[f'upper/{k}'] = (s, U.colour(k))
    write_step(items, OUT, 'robot_v13')
    print('wrote', OUT, len(items), 'parts', flush=True)
    front = {k: v for k, v in items.items() if v[0].bounding_box().max.X > 120.0 and not k.split('/')[-1].startswith(('roller', 'wheel', 'pod_'))}
    front['vendor/d435'] = (bought['d435'], (0.15, 0.15, 0.18))
    front['vendor/plug_d435'] = (bought['plug_d435'], (0.10, 0.10, 0.10))
    write_step(front, OUT.parent / 'd435_mount_review.step', 'd435_mount_v13')
    print('wrote', OUT.parent / 'd435_mount_review.step', len(front), 'parts', flush=True)


if __name__ == '__main__':
    main()

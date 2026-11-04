"""Part STEPs + manifest for the SolidWorks assemblies of the v11 wheel pod (sw_build.py builds them). omni-robot-sim interpreter.

    python -B sw_parts.py

Writes cad/v11/solidworks/step_parts/<part>.step (one coloured part each) and manifest.json. Parts used once are written in the pod
frame (inserted at the origin); the roller core/tread are written in their own frame with one pose per instance. Fasteners are not
modelled except the four spring plungers (M4 bolt + washer + nut as one body); bearings and springs are included so the mates have real faces.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

SW = W.ROOT / 'solidworks'
STEPS = SW / 'step_parts'
colour = P.colour


def main():
    STEPS.mkdir(parents=True, exist_ok=True)
    for f in STEPS.glob('*.step'):
        f.unlink()
    wm, _ = W.build_all()
    parts, cinfo = P.build_static()
    once = {k: wm[k] for k in ('wheel_hub', 'wheel_cartridge', 'bearing_6804_left', 'ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1',
                               'xt30_pin_2', 'lead_pos', 'lead_neg')}
    # one-body 6804 ring in the cartridge (the reference STEP has separate races and balls: SolidWorks opens it as an assembly)
    once['bearing_6804_left'] = W.along_x(R_BRG, X_PLATE_FLOOR, X_PLATE_FLOOR + 7.0) - W.along_x(10.0, X_PLATE_FLOOR - 1, X_PLATE_FLOOR + 8.0)
    once.update(pod_fork=parts['fork'], axle_stub=parts['stub'], pod_frame_outer=parts['frame_outer'], pod_frame_inner=parts['frame_inner'])
    for n in P.FRAMES:
        once['bearing_627_' + n] = parts['b627_' + n]
    for k, (xt, y) in P.PLUNGERS.items():
        once[f'spring_plunger_{k}'] = P.plunger(xt, y)
        once[f'die_spring_{k}'] = P.die_spring(xt, y, P.Z_CEIL - P.Z_SB)
    for k, s in once.items():
        write_part(s, colour(k), STEPS / (k + '.step'), k)
    local = dict(roller_core=wm['_roller_core'], roller_tread=wm['_roller_tread'])
    for k, s in local.items():
        write_part(s, colour(k), STEPS / (k + '.step'), k)
    man = dict(
        once=sorted(once), local=sorted(local),
        rollers={f'{row}_{k}': M.tolist() for row, k, M in W.roller_frames()},
        roller=dict(bore_r=BORE_D / 2, journal_end=HT + JOURNAL, thrust_face=ZF, hub_hole_r=CLR_D / 2),
        motor=dict(rotor_face_x=MOTOR_X[0] + 1.0, rotor_spigot_r=9.7, stator_recess_r=10.0),
        pivot=[PIVOT_Y, PIVOT_Z], b627_bore_r=B627_BORE / 2, pin_r=B627_BORE / 2 - 0.02, poses_deg=P.POSES,
        plungers=dict(axes={k: list(v) for k, v in P.PLUNGERS.items()}, shank_r=SHANK_R, guide_r=GUIDE_R, pad_z=PAD_Z,
                      dome_r=P.DOME_R, pad_half=PAD_HALF, foot_top_z=PLATE_TOP_Z + FOOT_T,
                      lift={k: {pk: P.plunger_lift(a, y)[0] for pk, (_, y) in P.PLUNGERS.items()} for k, a in P.POSES.items()}),
    )
    (SW / 'manifest.json').write_text(json.dumps(man, indent=1), encoding='utf-8')
    print('wrote', len(list(STEPS.glob('*.step'))), 'part STEPs and the manifest to', SW)


if __name__ == '__main__':
    main()

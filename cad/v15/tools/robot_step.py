"""One STEP of the whole v15 robot for review (step_export/robot_assembly.step): every printed part in the robot frame at the pods' nominal pose,
**with every fastener and bearing as a real model in magenta** (cad/solidworks.md 15: assembly STEPs show the bearings and fasteners; the
per-part print STEPs in step_export/<group>/ stay printed parts only). Other bought parts (motors, electronics, the battery, the SO-101) are
left out: SolidWorks (solidworks/V15_robot.SLDASM) shows everything.

    python -B robot_step.py        (omni-robot-sim interpreter)

The STEP is read back afterwards and its solid count and bounding box written to validation/robot/robot_step.json, with the same for the
base's hardware-complete assembly (step_export/chassis_mid/v15_mid_base_assembly.step).
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import chassis_mid as CM                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod_rx as R                                                     # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
import world as WD                                                     # noqa: E402
from step_colour import write_step                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'robot_assembly.step'
VAL = W.ROOT / 'validation' / 'robot'
POD_PRINTED = ('pod_fork_outer', 'pod_fork_inner', 'rx_hanger', 'rx_beam', 'rx_pin', 'rx_lug', 'rx_tower', 'rx_ring', 'pod_retainer', 'wheel_hub',
               'wheel_cartridge', 'roller_core', 'roller_sleeve', 'roller_tread')
POD_HARDWARE = ('bolt', 'nut', 'bearing', 'plunger', 'spring')        # the springs and plunger bolts are bought too: magenta as the rest


def reimport(path):
    """(solids, bounding box) of a STEP read back"""
    import build123d as b
    s = b.import_step(str(path))
    bb = s.bounding_box()
    return dict(file=str(path.relative_to(W.ROOT)), solids=len(s.solids()), bbox_min=[round(v, 1) for v in tuple(bb.min)],
                bbox_max=[round(v, 1) for v in tuple(bb.max)])


def pods_placed():
    """CM.pods_placed(0, labelled=True); Nov 19 (Fernando: the STEPs as the SolidWorks assembly): each M3 plunger as its seven parts
    (sw_robot_parts.m3_pieces: cap yellow, sleeve cyan, the steel magenta) and its bore's two coil springs, in place of the pod build's fused
    plunger and spring tube"""
    import sw_robot_parts as SP
    import base as B
    m3 = {}
    pieces = SP.m3_pieces()
    for pk, (xs, y) in SP.PLUNGERS.items():
        at = b_location(xs, y)
        for k, (s, c) in pieces.items():
            m3[f'{k}_{pk}'] = (s.moved(at), c)
        m3[f'spring_{pk}'] = (SP.m3_springs(xs, y), H.MAGENTA)
    out = {}
    for i, (n, (st, mv)) in enumerate(CM.pods_placed(0.0, labelled=True).items()):
        st = {k: v for k, v in st.items() if not k.startswith(('plunger_', 'spring_'))}
        st.update({k: (B.place3d(s, i), c) for k, (s, c) in m3.items()})
        out[n] = (st, mv)
    return out


def b_location(x, y):
    import build123d as b
    return b.Location((x, y, 0))


def main():
    items = {}
    parts, hw, _ = WD.base()
    for k, s in parts.items():
        items[f'base/{k}'] = (s, CM.colour(k))
    for k, s in hw.items():
        items[f'base/{k}'] = (s, H.MAGENTA)
    for n, (st, mv) in pods_placed().items():                          # Nov 15: each pod's parts with its number and IN / OUT
        for k, s in {**st, **mv}.items():
            if isinstance(s, tuple):                                   # an M3 plunger's piece in its own colour
                items[f'{n}/{k}'] = s
            elif k.startswith(POD_PRINTED):
                items[f'{n}/{k}'] = (s, R.colour(k))
            elif k.startswith(POD_HARDWARE):
                items[f'{n}/{k}'] = (H.step_model(k, s), H.MAGENTA)
    printed, _, _, uhw, _ = WD.upper()
    for k, s in printed.items():
        items[f'upper/{k}'] = (s, U.colour(k))
    for n, r in enumerate(uhw):
        items[f'upper/{r["name"]}#{n}'] = (H.solid(r), H.MAGENTA)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    write_step(items, OUT, 'robot_v15')
    print('wrote', OUT, len(items), 'items', flush=True)
    VAL.mkdir(parents=True, exist_ok=True)
    rep = dict(written=len(items), robot=reimport(OUT))
    base = W.ROOT / 'step_export' / 'chassis_mid' / 'v15_mid_base_assembly.step'
    if base.exists():
        rep['base_assembly'] = reimport(base)
    (VAL / 'robot_step.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps(rep), flush=True)


if __name__ == '__main__':
    main()

"""One STEP of the whole v15 robot for review (step_export/robot_assembly.step): every printed part in the robot frame at the pods' nominal pose,
**with every fastener and bearing as a real model in magenta** (cad/solidworks.md 15: assembly STEPs show the bearings and fasteners; the
per-part print STEPs in step_export/<group>/ stay printed parts only). Other bought parts (motors, electronics, the battery, the SO-101) are
left out: SolidWorks (solidworks/V16_robot.SLDASM) shows everything.

    python -B robot_step.py        (omni-robot-sim interpreter)

The STEP is read back afterwards and its solid count and bounding box written to validation/robot/robot_step.json, with the same for the
base's hardware-complete assembly (step_export/chassis_mid/v16_mid_base_assembly.step).
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


CACHE = W.ROOT / 'cache' / 'robot_step'                                # Nov 19: the base's and the upper's items (BREP + colour), kept by
                                                                       # each full run so `robot_step.py pods` rebuilds only the pods


def save_cache(items):
    from OCP.BRepTools import BRepTools
    CACHE.mkdir(parents=True, exist_ok=True)
    for f in CACHE.glob('*.brep'):
        f.unlink()
    index = {}
    for n, (k, (s, rgb)) in enumerate(items.items()):
        f = CACHE / f'{n:04d}.brep'
        BRepTools.Write_s(s.wrapped, str(f))
        index[k] = [f.name, list(rgb)]
    (CACHE / 'index.json').write_text(json.dumps(index, indent=0), encoding='utf-8')


def load_cache():
    import build123d as b
    from OCP.BRep import BRep_Builder
    from OCP.BRepTools import BRepTools
    from OCP.TopoDS import TopoDS_Shape
    idx = CACHE / 'index.json'
    if not idx.exists():
        raise RuntimeError(f'no {idx}: run `robot_step.py` once in full first')
    out = {}
    for k, (fn, rgb) in json.loads(idx.read_text(encoding='utf-8')).items():
        sh = TopoDS_Shape()
        BRepTools.Read_s(sh, str(CACHE / fn), BRep_Builder())
        out[k] = (b.Compound(sh) if sh.ShapeType().name not in ('TopAbs_SOLID',) else b.Solid(sh), tuple(rgb))
    return out


def main(mode='all'):
    """mode 'pods' (Nov 19, quick.sh): the base and the upper from the last full run's cache, the pods rebuilt; no read-back"""
    items = {}
    if mode == 'pods':
        cached = load_cache()
        items.update({k: v for k, v in cached.items() if k.startswith('base/')})
    else:
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
    if mode == 'pods':
        items.update({k: v for k, v in cached.items() if k.startswith('upper/')})
    else:
        printed, _, _, uhw, _ = WD.upper()
        for k, s in printed.items():
            items[f'upper/{k}'] = (s, U.colour(k))
        for n, r in enumerate(uhw):
            items[f'upper/{r["name"]}#{n}'] = (H.solid(r), H.MAGENTA)
        save_cache({k: v for k, v in items.items() if k.startswith(('base/', 'upper/'))})
    OUT.parent.mkdir(parents=True, exist_ok=True)
    write_step(items, OUT, 'robot_v16')
    print('wrote', OUT, len(items), 'items', flush=True)
    if mode == 'pods':
        return
    VAL.mkdir(parents=True, exist_ok=True)
    rep = dict(written=len(items), robot=reimport(OUT))
    base = W.ROOT / 'step_export' / 'chassis_mid' / 'v16_mid_base_assembly.step'
    if base.exists():
        rep['base_assembly'] = reimport(base)
    (VAL / 'robot_step.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
    print(json.dumps(rep), flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')

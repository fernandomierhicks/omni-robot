"""cad/v14 687 fit coupon (Nov 13 2026, Fernando: a very compact coupon for a very tight fit of the 687, 7 x 14 x 5, in the roller core and on its sleeve).
build123d; bench print, not part of the robot.

    python -B coupon_687.py [build|check|export|render|all]        (omni-robot-sim interpreter)

One 40 x 40 x 12 block, four cells in a 2 x 2 grid, each numbered on the top (engraved). Cell n has:
 * a core pocket, Ø C687_POCKET[n], opening on the bed side (as the real core's lower pocket prints, upright on its end), 5.6 deep under a
   45 deg cone roof (no support). Set the bearing on a flat bench and press the block down over it: it stops flush with the face. 4.0 mm of PLA
   round each pocket, the core's 4.15 wall from pocket to key slot, so the fit reads as in the roller;
 * a sleeve stub on the top, as the real sleeve's end: a Ø8.6 x 0.8 shoulder (the inner race lands on it), a Ø C687_PIN[n] x 6 pin with the
   sleeve's 0.2 lead-in, and the Ø3.2 sleeve bore through (the same 1.9 wall, so it gives like the sleeve). Press a bearing on to the shoulder;
 * two Ø1.8 push-out holes from the top onto the pocket bearing's outer ring (r 6-7), wholly inside the pocket so they never notch its
   wall: push the bearing out with a 1.5 mm hex key, side to side.
Cell 1 is the drawn roller (pocket 14.10, sleeve 7.00); every cell after it is 0.05 tighter on both (13.95 / 7.15 in cell 4). Pick the cell whose
pocket takes the bearing with a firm press and no crack, and the one whose pin needs a firm press and holds; put the two diameters into
dims.BRG_POCKET_FIT and dims.BRG_D's sleeve (wheel_motor.sleeve_solid). Print in the roller core's filament and settings, cells' axes vertical.
Dimensions: dims.C687_*. Nominal CAD; not printed.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import palette as PAL                                                  # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part                                     # noqa: E402

OUT = W.ROOT / 'step_export' / 'coupon_687'
VAL = W.ROOT / 'validation' / 'coupon_687'
HALF = C687_PITCH / 2 + max(C687_POCKET) / 2 + C687_WALL              # block half width (20.05)
CORNER_R = 4.0
# cell number -> centre: 1 2 on the top row (+y), 3 4 below; numbers read upright with +y up
CELLS = {1: (-C687_PITCH / 2, C687_PITCH / 2), 2: (C687_PITCH / 2, C687_PITCH / 2), 3: (-C687_PITCH / 2, -C687_PITCH / 2), 4: (C687_PITCH / 2, -C687_PITCH / 2)}
PIN_TOP = C687_TOP + C687_SHOULDER + C687_PIN_LEN


def cone_roof(rp):
    """the pocket's 45 deg roof: from the pocket wall at its depth in to the sleeve bore"""
    h = rp - SLEEVE_ID / 2
    return b.Cone(rp, SLEEVE_ID / 2, h, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN)).moved(b.Location((0, 0, C687_POCKET_D)))


def outward(n):
    cx, cy = CELLS[n]
    return np.array([math.copysign(1, cx), math.copysign(1, cy)]) / math.sqrt(2)


def build():
    blk = b.Box(2 * HALF, 2 * HALF, C687_TOP, align=(b.Align.CENTER, b.Align.CENTER, b.Align.MIN))
    blk = b.fillet([e for e in blk.edges() if e.geom_type.name == 'LINE' and abs(e.position_at(1).Z - e.position_at(0).Z) > 1], CORNER_R)
    for n, (cx, cy) in CELLS.items():
        rp, rpin = C687_POCKET[n - 1] / 2, C687_PIN[n - 1] / 2
        L = b.Location((cx, cy, 0))
        pin = W.zcyl(BRG_IR_LAND / 2, C687_TOP - 0.01, C687_TOP + C687_SHOULDER) + W.zcyl(rpin, C687_TOP + C687_SHOULDER - 0.01, PIN_TOP)
        top = [e for e in pin.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().Z - PIN_TOP) < 1e-3]
        blk = blk + b.chamfer(top, SLEEVE_LEAD).moved(L)
        blk = blk - (W.zcyl(rp, -1, C687_POCKET_D) + cone_roof(rp) + W.zcyl(SLEEVE_ID / 2, -1, PIN_TOP + 1)).moved(L)
        u = outward(n)
        for s in (1, -1):
            px, py = np.array([cx, cy]) + s * C687_PUSH_R * np.array([-u[1], u[0]])
            blk = blk - W.zcyl(C687_PUSH_D / 2, BRG_B, C687_TOP + 1).moved(b.Location((px, py, 0)))
        lx, ly = np.array([cx, cy]) + 9.0 * u
        txt = b.extrude(b.Text(str(n), font_size=C687_LABEL[0], align=(b.Align.CENTER, b.Align.CENTER)), amount=C687_LABEL[1] + 0.1)
        blk = blk - txt.moved(b.Location((lx, ly, C687_TOP - C687_LABEL[1])))
    blk.label = 'coupon_687'
    return {'coupon_687': blk}


def bearing_at(n, where):
    """a 687 envelope in cell n: 'pocket' = pressed flush with the bed face, 'pin' = on the shoulder"""
    cx, cy = CELLS[n]
    z = BRG_B / 2 if where == 'pocket' else C687_TOP + C687_SHOULDER + BRG_B / 2
    return W.bearing_687().moved(b.Location((cx, cy, z)))


def check(shapes):
    c = shapes['coupon_687']
    bb = c.bounding_box()
    rep = dict(part=dict(solids=len(c.solids()), valid=bool(c.is_valid), volume_cm3=round(c.volume / 1000, 2), bbox=[round(v, 2) for v in bb.size]), cells={})
    for n, (cx, cy) in CELLS.items():
        rp_meas = W.min_dist(b.Vertex(cx, cy, 0.5), c)                                                    # on the axis, 0.5 into the pocket
        rpin_meas = 5.0 - W.min_dist(b.Vertex(cx + 5.0, cy, C687_TOP + C687_SHOULDER + 3.0), c)          # beside the pin, halfway up
        pocket_b, pin_b = bearing_at(n, 'pocket'), bearing_at(n, 'pin')
        # walls: to the block's faces and to the neighbouring pockets
        wall_out = HALF - (abs(cx) + C687_POCKET[n - 1] / 2)
        web = C687_PITCH - (C687_POCKET[n - 1] + max(C687_POCKET)) / 2
        u = outward(n)
        push_ok = []
        for s in (1, -1):
            px, py = np.array([cx, cy]) + s * C687_PUSH_R * np.array([-u[1], u[0]])
            rod = W.zcyl(0.75, BRG_B - 0.1, C687_TOP + 2).moved(b.Location((px, py, 0)))
            push_ok.append(round(W.overlap(rod, c), 4) == 0 and W.overlap(rod, pocket_b) > 0)        # a 1.5 key passes and meets the bearing
        rep['cells'][n] = dict(
            pocket_d=C687_POCKET[n - 1], pocket_d_measured=round(2 * rp_meas, 3), interference_d_on_14=round(BRG_DO - C687_POCKET[n - 1], 3),
            pocket_bearing_overlap_mm3=round(W.overlap(pocket_b, c), 3), 
            pin_d=C687_PIN[n - 1], pin_d_measured=round(2 * rpin_meas, 3), interference_d_on_7=round(C687_PIN[n - 1] - BRG_D, 3),
            pin_bearing_overlap_mm3=round(W.overlap(pin_b, c), 3),
            shoulder_on_inner_race_only=bool(BRG_IR_LAND / 2 < 5.0), wall_to_face=round(wall_out, 2), push_hole_inside_pocket_by=round(C687_POCKET[n - 1] / 2 - C687_PUSH_R - C687_PUSH_D / 2, 3), web_to_neighbour_min=round(web, 2),
            roof_over_cone_apex=round(C687_TOP - (C687_POCKET_D + C687_POCKET[n - 1] / 2 - SLEEVE_ID / 2), 2),
            pin_wall=round((C687_PIN[n - 1] - SLEEVE_ID) / 2, 2), push_out_holes_reach_race=all(push_ok))
    rep['bearings_clear_each_other'] = all(W.overlap(bearing_at(i, 'pin'), bearing_at(j, 'pin')) == 0 for i in CELLS for j in CELLS if i < j)
    rep['fits_bed'] = max(bb.size.X, bb.size.Y) <= 256
    return rep


def export(shapes):
    OUT.mkdir(parents=True, exist_ok=True)
    rgb = PAL.colour('roller_core') or (0.38, 0.66, 0.38)
    write_part(shapes['coupon_687'], rgb, OUT / 'coupon_687.step', 'coupon_687')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    c = shapes['coupon_687']
    fig, axs = plt.subplots(1, 2, figsize=(14, 5.2))
    for ax, row in zip(axs, ((1, 2), (3, 4))):
        y = CELLS[row[0]][1]
        items = [(c, (0.45, 0.45, 0.48))] + [(bearing_at(n, w), (0.85, 0.55, 0.15)) for n in row for w in ('pocket', 'pin')]
        polys = []
        for s, col in items:
            x = s.moved(b.Location((0, -y, 0))).rotate(b.Axis.X, -90) & b.Box(200, 200, 0.4)   # the plane y = cell row, seen from -y
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.02, 0.2)
            Vv = np.array([[q.X, q.Y] for q in v])
            polys += [(Vv[tri], col) for tri in np.array(t)]
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.set_xlim(-22, 22)
        ax.set_ylim(-2, 21)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title('cells ' + ' and '.join(f'{n} (pocket Ø{C687_POCKET[n - 1]:.2f}, pin Ø{C687_PIN[n - 1]:.2f})' for n in row), fontsize=9)
        ax.set_xlabel('x (mm); bed at z = 0, 687s orange')
    fig.tight_layout()
    VAL.mkdir(parents=True, exist_ok=True)
    fig.savefig(VAL / 'coupon_687_section.png', dpi=110)
    plt.close(fig)
    # top view: numbers, pins, push-out holes
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    x = c & b.Box(200, 200, 0.2).moved(b.Location((0, 0, C687_TOP - 0.15)))
    v, t = x.tessellate(0.02, 0.2)
    Vv = np.array([[q.X, q.Y] for q in v])
    ax.add_collection(PolyCollection([Vv[tri] for tri in np.array(t)], facecolors=(0.45, 0.45, 0.48), edgecolors='none'))
    for n, (cx, cy) in CELLS.items():
        ax.add_patch(plt.Circle((cx, cy), BRG_IR_LAND / 2, fc=(0.3, 0.5, 0.8)))
        ax.add_patch(plt.Circle((cx, cy), C687_POCKET[n - 1] / 2, fill=False, ls='--', lw=.7))
    ax.set_xlim(-22, 22)
    ax.set_ylim(-22, 22)
    ax.set_aspect('equal')
    ax.set_title('top, 0.15 under the face: engraved numbers, push-out holes;\nblue: sleeve stubs, dashed: pockets below', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'coupon_687_top.png', dpi=110)
    plt.close(fig)


def main(stage):
    shapes = build()
    c = shapes['coupon_687']
    print('coupon_687', len(c.solids()), c.is_valid, round(c.volume / 1000, 2), [round(v, 2) for v in c.bounding_box().size], flush=True)
    if stage in ('check', 'all'):
        VAL.mkdir(parents=True, exist_ok=True)
        rep = check(shapes)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'all')

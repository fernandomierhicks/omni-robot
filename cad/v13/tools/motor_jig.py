"""cad/v13 bench jig for bolting the AK40-10's rotor to the wheel hub (Nov 10 2026, Fernando). build123d, two printed parts; not part of the robot.

    python -B motor_jig.py [build|check|export|render|all]        (omni-robot-sim interpreter)

The joint has no spigot: three M2.5 at the bottom of a 40 mm deep bore are all that tie the rotor to the hub, and the rotor turns freely. So the motor stands
output side up and the hub comes down over it, with the screws going in from above:
 * CRADLE (jig_motor_cradle): holds the can in a sleeve whose outside centres the hub's drum bore, so both are on one axis and the hub's three flange holes
   lie on the same circle as the rotor's three threaded holes. An open slot takes the XT30 and its leads and stops the stator turning.
 * GUIDE CAP (jig_motor_guide): replaces the cartridge for the job. A stem down the access bore, clocked to the flange holes by three pegs in the cartridge's
   pilots. Over the three flange holes it has one narrow probe bore and two bores wide enough for a screw head and driver.
Only the angle is then left, and it needs no sight of the rotor: put a length of 1.75 mm filament, its end cut to a point, down the probe bore and turn the WHEEL
(the rotor stays put in its gearbox) until the filament drops into a threaded hole. It then pins the hub to the rotor. Drive the two screws through the wide bores,
pull the filament, lift the cap, turn it 120 deg, drop it back on and drive the third.
Everything is nominal CAD; nothing has been printed. Dimensions in dims.py (JIG_*).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hardware as H                                                   # noqa: E402
import pod_lean as L                                                   # noqa: E402
import print_beds as PB                                                # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

along_x, at_angle, overlap, min_dist, near = W.along_x, W.at_angle, W.overlap, W.min_dist, W.near
OUT = W.ROOT / 'step_export' / 'motor_jig'
VAL = W.ROOT / 'validation' / 'motor_jig'
JIG_COLOURS = {'jig_motor_cradle': (0.20, 0.55, 0.85), 'jig_motor_guide': (0.30, 0.75, 0.45), 'filament': (0.90, 0.20, 0.20)}
X_REAR, X_OUTPUT = MOTOR_X[1], MOTOR_X[0]
PROBE_AZ, GUIDE_AZ = FLANGE_ANGLES[0], FLANGE_ANGLES[1:]
PRINTED = ('jig_motor_cradle', 'jig_motor_guide')
HARNESS = ('xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg')


def colour(name):
    return JIG_COLOURS.get(name) or W.colour(name)


def harness_slot(shapes):
    """(azimuth of the slot's centre line, its half width) from the plug, pins and leads where they stand outside the can"""
    P = np.vstack([W.points(shapes[k], 0.05) for k in HARNESS])
    P = P[np.hypot(P[:, 1], P[:, 2]) > MOTOR_R - 1.0]
    az = math.atan2(P[:, 2].mean(), P[:, 1].mean())
    half = float(np.abs(-P[:, 1] * math.sin(az) + P[:, 2] * math.cos(az)).max())
    return math.degrees(az), half + JIG_SLOT_CLR


def build_cradle(shapes):
    """base disc under the can, 45 deg foot, sleeve: the can inside it, the drum bore outside it; one open slot, top to bottom, for the XT30 and leads"""
    ri, ro = MOTOR_R + JIG_CLR, DRUM_IN - JIG_CLR
    xb = X_REAR + JIG_BASE_T
    rc = ro + (X_REAR - JIG_CONE_X)
    prof = [(xb, JIG_BORE_R), (xb, JIG_BASE_R - 0.8), (xb - 0.8, JIG_BASE_R), (X_REAR, JIG_BASE_R), (X_REAR, rc), (JIG_CONE_X, ro), (JIG_SLEEVE_TOP + 1.0, ro),
            (JIG_SLEEVE_TOP, ro - 1.0), (JIG_SLEEVE_TOP, ri + 1.0), (JIG_SLEEVE_TOP + 1.0, ri), (X_REAR, ri), (X_REAR, JIG_BORE_R)]
    c = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    az, half = harness_slot(shapes)
    x0, x1 = JIG_SLEEVE_TOP - 1.0, xb + 1.0
    slot = b.Box(x1 - x0, 80.0, 2 * half).moved(b.Location(((x0 + x1) / 2, JIG_BORE_R - 5.0 + 40.0, 0))).rotate(b.Axis.X, az)   # runs into the base's opening: a C, no sliver
    return c - slot, dict(slot_azimuth_deg=round(az, 1), slot_width_mm=round(2 * half, 1))


def build_guide():
    """plate on the hub's left end, boss in the cartridge counterbore, stem down the access bore; pegs into the cartridge pilots; bores on the flange holes"""
    xt = X_LEFT - JIG_CAP_T
    xboss = X_PLATE_FLOOR - 0.3                                        # the plate rests on the hub's end face, the boss stops short of the counterbore floor
    xs = FLANGE_FRONT - JIG_STEM_GAP
    prof = [(xt, 0.0), (xt, JIG_CAP_R - 0.8), (xt + 0.8, JIG_CAP_R), (X_LEFT, JIG_CAP_R), (X_LEFT, DRUM_IN - 0.2), (xboss, DRUM_IN - 0.2), (xboss, ACCESS_R - JIG_CLR),
            (xs - 0.5, ACCESS_R - JIG_CLR), (xs, ACCESS_R - JIG_CLR - 0.5), (xs, 0.0)]
    g = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    for a in CART_ANGLES:
        y, z = at_angle(CART_R, a)
        tip = b.Cone(JIG_PEG_D / 2, JIG_PEG_D / 2 - 0.4, 0.4).rotate(b.Axis.Y, 90).moved(b.Location((xboss + JIG_PEG_L + 0.2, y, z)))
        g = g + along_x(JIG_PEG_D / 2, xboss - 0.5, xboss + JIG_PEG_L, y, z) + tip
    def lead_in(r, y, z, h):
        return b.Cone(r + h, r, h).rotate(b.Axis.Y, 90).moved(b.Location((xt + h / 2, y, z)))
    for a in GUIDE_AZ:
        y, z = at_angle(FLANGE_R, a)
        g = g - along_x(JIG_GUIDE_D / 2, xt - 0.1, xs + 0.1, y, z) - lead_in(JIG_GUIDE_D / 2, y, z, 1.0)
    y, z = at_angle(FLANGE_R, PROBE_AZ)
    g = g - along_x(JIG_PROBE_D / 2, xt - 0.1, xs + 0.1, y, z) - lead_in(JIG_PROBE_D / 2, y, z, 1.5)
    return g - along_x(JIG_GUIDE_D / 2, xs - JIG_HEAD_RELIEF, xs + 0.1, y, z)


def hub_screw(a):
    return H.solid(H.pan((FLANGE_FRONT, *at_angle(FLANGE_R, a)), (1, 0, 0), 5.0, 'bolt_hub_rotor', size=2.5))


def filament():
    y, z = at_angle(FLANGE_R, PROBE_AZ)
    return along_x(0.875, X_LEFT - JIG_CAP_T - 30.0, X_OUTPUT + 1.0, y, z)


def build_all():
    shapes, _ = W.build_all()
    shapes['jig_motor_cradle'], info = build_cradle(shapes)
    shapes['jig_motor_guide'] = build_guide()
    return shapes, info


def slide(s, dx):
    return s.moved(b.Location((dx, 0, 0)))


def check(shapes, info):
    cradle, guide, hub = shapes['jig_motor_cradle'], shapes['jig_motor_guide'], shapes['wheel_hub']
    rollers = [k for k in shapes if k.startswith(('roller_core_', 'roller_tread_'))]
    motor = ('ak40_stator', 'ak40_rotor') + HARNESS
    rep = dict(cradle=info)
    for k, up in (('jig_motor_cradle', -1), ('jig_motor_guide', 1)):
        s = shapes[k]
        rep[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1), bbox=[round(v, 1) for v in tuple(s.bounding_box().size)],
                      overhang_mm2_as_printed=L.overhang_mm2(s, up))
    rep['cradle_overlap_mm3'] = {k: round(overlap(cradle, shapes[k]), 3) for k in motor + ('wheel_hub',)}
    rep['cradle_gap_mm'] = {k: round(min_dist(cradle, shapes[k]), 2) for k in motor + ('wheel_hub',)}
    rep['cradle_to_rollers_min_mm'] = round(min(min_dist(cradle, shapes[k]) for k in rollers), 2)
    rep['motor_drop_into_cradle_overlap_mm3'] = {f'{d:g}': round(sum(overlap(cradle, slide(shapes[k], -d)) for k in motor), 3) for d in (5, 10, 20, 30, 40, 50, 60)}
    rep['hub_drop_over_cradle_overlap_mm3'] = {f'{d:g}': round(overlap(cradle, slide(hub, -d)), 3) for d in (5, 10, 15, 20, 30, 40)}
    screws = {a: hub_screw(a) for a in FLANGE_ANGLES}
    orient = {}
    for turn in (0, 120, 240):
        g = guide.rotate(b.Axis.X, turn) if turn else guide
        r = dict(guide_hub_overlap_mm3=round(overlap(g, hub), 3), guide_hub_gap_mm=round(min_dist(g, hub), 3), guide_rotor_gap_mm=round(min_dist(g, shapes['ak40_rotor']), 2),
                 screw_overlap_mm3={f'{a:g}': round(overlap(g, s), 3) for a, s in screws.items()})
        r['lift_out_overlap_mm3'] = {f'{d:g}': round(overlap(slide(g, -d), hub), 3) for d in (1, 3, 6, 10, 20, 40)}
        orient[f'cap turned {turn}'] = r
    rep['guide'] = orient
    f = filament()
    rep['filament_overlap_mm3'] = dict(guide=round(overlap(f, guide), 3), hub=round(overlap(f, hub), 3), rotor_tip_in_its_hole=round(overlap(f, shapes['ak40_rotor']), 3))
    rep['screw_vs_hub_overlap_mm3 (shank in Ø2.7, head on the flange)'] = {f'{a:g}': round(overlap(s, hub), 3) for a, s in screws.items()}
    rep['screw_driver_depth_mm (cap top to a seated head top)'] = round((FLANGE_FRONT - 2.0) - (X_LEFT - JIG_CAP_T), 1)
    rep['filament_min_length_mm (cap top to rotor face, plus 30 to hold)'] = round(X_OUTPUT - (X_LEFT - JIG_CAP_T) + 30.0, 0)
    return rep


def export(shapes):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for k in PRINTED:
        write_part(shapes[k], colour(k), OUT / 'parts' / (k + '.step'), k)
    keep = PRINTED + ('wheel_hub', 'ak40_stator', 'ak40_rotor') + HARNESS
    write_step({k: (shapes[k], colour(k)) for k in keep}, OUT / 'motor_jig_assembly.step', 'motor_jig')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    for f in VAL.glob('*.png'):
        f.unlink()
    keep = ('wheel_hub', 'ak40_stator', 'ak40_rotor', 'jig_motor_cradle', 'jig_motor_guide', 'xt30_housing', 'lead_pos', 'lead_neg')
    extra = {f'bolt_{i}': hub_screw(a) for i, a in enumerate(GUIDE_AZ)}
    extra['filament'] = filament()
    # section through the axis and the probe bore, x (print and assembly axis) up the page: the cap on top, the cradle at the bottom
    fig, ax = plt.subplots(figsize=(10, 12))
    sl = b.Box(400, 400, 0.5)
    for k, s in {**{k: shapes[k] for k in keep}, **extra}.items():
        x = s.rotate(b.Axis.X, -PROBE_AZ) & sl
        if x is None or not x.faces():
            continue
        v, t = x.tessellate(0.1, 0.2)
        Pv = np.array([[q.Y, -q.X] for q in v])
        ax.add_collection(PolyCollection([Pv[tri] for tri in np.array(t)], facecolors=[colour(k)], edgecolors='none'))
    ax.set_xlim(-75, 75)
    ax.set_ylim(-60, 60)
    ax.set_aspect('equal')
    ax.grid(True, lw=.3)
    ax.set_title('section through the axis and the probe bore: guide cap (green) in the hub (orange), motor in the cradle (blue), filament (red)', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'section_assembled.png', dpi=100)
    plt.close(fig)
    up = {'jig_motor_guide': -95.0, 'wheel_hub': -45.0, 'ak40_stator': 0.0, 'ak40_rotor': 0.0, 'xt30_housing': 0.0, 'lead_pos': 0.0, 'lead_neg': 0.0, 'jig_motor_cradle': 0.0}
    apart = {k: slide(shapes[k], d) for k, d in up.items()}
    # x up the page: rotate the parts so the iso helper's z is the assembly axis
    flip = {k: s.rotate(b.Axis.Y, 90) for k, s in apart.items()}
    L.iso(flip, VAL / 'iso_exploded.png', 35, 18, 'exploded, as assembled on the bench: cap, hub, motor in the cradle', colour_fn=colour, tol=0.25)
    L.iso({'jig_motor_cradle': shapes['jig_motor_cradle'].rotate(b.Axis.Y, 90)}, VAL / 'iso_cradle.png', 35, 28, 'cradle', colour_fn=colour, tol=0.15)
    L.iso({'jig_motor_guide': shapes['jig_motor_guide'].rotate(b.Axis.Y, -90)}, VAL / 'iso_guide.png', 35, 28, 'guide cap, as printed: pegs, stem, one probe bore and two screw bores', colour_fn=colour, tol=0.1)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    shapes, info = build_all()
    print('built', info, flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes, info)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')

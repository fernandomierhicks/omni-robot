"""Smoother-rolling wheel, round 1 (Nov 12 2026, Fernando: the printed v13 wheel, rolled by hand with its rollers locked,
still shakes a hollow wood floor, rocking from three rollers to one or two). Fernando, Nov 12: print this tread the next
time rollers are printed (the relief is near TPU print precision; kept because it costs nothing). Not yet in the pod or
robot assemblies. wheel_quiet.py (the 687 roller cartridges and the round-3 hub) is imported unchanged; only the TPU tread differs.

    python -B wheel_quiet_2.py [build|check|export|render|all|compare]   (omni-robot-sim interpreter; compare = the
                                                                          tread before / after picture alone)

Why (simulation/v11 sil/studies/wheel_wobble.py, results/wheel_wobble_2026-11-12.md):
  * Upright, the wheel is round: every tread is the R70 circle about the wheel axis, so the hub height varies < 1 um.
  * Leaned by beta, it is not: the rows sit at x -+17.25, so one row's axles rise 17.25 sin(beta) and the other's drop,
    whatever the roller shape (a lean turns the bottom roller about its own axis). Every 30 deg the hub climbs onto, or
    drops off, the other row: 0.30 mm at 0.5 deg. Held only at the hub (a hand), nothing stops the lean: the wheel rocks
    -+3.9 deg between a roller of one row and the ends of the two rollers of the other row (the tripod) and the hub bobs
    1.3 mm. On the robot the fork holds the lean (k_roll ~300 N m/rad est): 0.2 deg of rocking, 0.1-0.25 mm of bob.
  * Even upright and held, the support hands over roller -> two rollers -> roller every 30 deg, and the TPU contact's
    stiffness jumps with it (one contact sinks ~0.22 mm under a quarter of the robot, two share it and sink 0.14), like a
    gear's mesh stiffness. That bob drives the hub on its tread (~70 Hz) and is the robot's main floor ripple at 1 m/s.
  * The lean step's height is set by the row spacing alone; overlap, end shape and tread hardness do not change it.

Round 1: tip relief (as on gear teeth). The last RELIEF_LEN of each tread end leaves the R70 circle on a tangent arc of
radius RELIEF_R, falling RELIEF_DEPTH below it by the end (the R2 end fillet then starts lower). Each roller then takes up
and gives back its load over a few millimetres of travel, not in a step. Unloaded and upright, the hub dips by the
relief at the handover point (~0.2 mm, less than the tread's own sink under load). Only the tread changes: core, sleeve,
687s and hub are wheel_quiet's (step_export/wheel_quiet/parts), the tread's bore, keys and detent are v13's.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_motor as wm                                                # noqa: E402
import wheel_quiet as wq                                                # noqa: E402
from dims import *                                                      # noqa: E402,F401,F403
from step_colour import write_part, write_step                          # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'step_export' / 'wheel_quiet_2'
VAL = ROOT / 'validation' / 'wheel_quiet_2'

# ---- round 1 (mm) ------------------------------------------------------------------------------------------------------------
RELIEF_LEN = 7.0                           # relief zone at each tread end (local z from HT - RELIEF_LEN to HT)
RELIEF_DEPTH = 0.2                         # its drop below the R70 circle at the end face (before the end fillet). wheel_wobble.py
                                           # refine: 0.2 / 7 is the best all-round of 0.2-0.4 x 6-9 mm (deeper wins at 1 m/s on
                                           # a 0.5 deg lean but loses at light load and walking pace)
RELIEF_R = 1.0 / (1.0 / R_WHEEL + 2.0 * RELIEF_DEPTH / RELIEF_LEN ** 2)   # tangent arc with that drop (to second order)
Z0 = HT - RELIEF_LEN                       # where the relief starts
Z_HANDOVER = R_WHEEL * math.sin(math.radians(15.0))   # where the two rows hand over upright (30 deg pitch / 2)


def _relief_centre():
    """the relief arc's centre in the roller's (r, z) plane (r from the roller axis): on the R70 normal at Z0"""
    q = math.sqrt(R_WHEEL ** 2 - Z0 ** 2)
    return (q - R_AXIS) - RELIEF_R * q / R_WHEEL, Z0 - RELIEF_R * Z0 / R_WHEEL


def rt_relief(z):
    """relieved tread radius about the roller axis at local z (no end fillet)"""
    z = abs(z)
    if z <= Z0:
        return rt(z)
    cr, cz = _relief_centre()
    return cr + math.sqrt(RELIEF_R ** 2 - (z - cz) ** 2)


def tread_relief():
    """wm.tread_solid() with the relieved outer profile: R70 arc between -+Z0, tangent RELIEF_R arcs to the ends, R2 end
    fillets; straight bore, keys and detent rib unchanged"""
    H = HT
    V = b.Vector
    zm = (Z0 + H) / 2
    edges = [wm.arc(V(rt(Z0), -Z0, 0), V(rt(0), 0, 0), V(rt(Z0), Z0, 0)),
             wm.arc(V(rt(Z0), Z0, 0), V(rt_relief(zm), zm, 0), V(rt_relief(H), H, 0)),
             wm.line(V(rt_relief(H), H, 0), V(CORE_R, H, 0)), wm.line(V(CORE_R, H, 0), V(CORE_R, -H, 0)),
             wm.line(V(CORE_R, -H, 0), V(rt_relief(H), -H, 0)),
             wm.arc(V(rt_relief(H), -H, 0), V(rt_relief(zm), -zm, 0), V(rt(Z0), -Z0, 0))]
    t = b.revolve(b.Plane.XZ * b.Face(b.Wire(edges)), axis=b.Axis.Z)
    t = b.fillet(wm.end_circles(t, rt_relief(H)), FILLET_TREAD)
    t = b.chamfer(wm.end_circles(t, CORE_R), CORE_CHAMFER)
    for a in (0, 120, 240):
        t = t + b.Box(KEY_DEPTH + 0.5, KEY_W, 2 * H).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 0.5) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return t + wm.detent_ring(CORE_R + 0.01, CORE_R - DETENT_RIB)


def build_all():
    shapes, info = wq.build_all()
    tr = tread_relief()
    shapes['_roller_tread_relief'] = tr
    for row, k, M in wm.roller_frames():
        shapes.pop(f'roller_tread_{row}_{k}')
        shapes[f'roller_tread_relief_{row}_{k}'] = tr.moved(wm.loc(M))
    for k, s in shapes.items():
        s.label = k.lstrip('_')
        s.color = b.Color(*wq.colour(k))
    return shapes, info


def section_radius(shape, z):
    """the solid's outer radius about the roller axis at local z, from a thin section (CAD check of the profile)"""
    sl = shape & b.Box(60, 60, 0.002).moved(b.Location((0, 0, z)))
    v, _ = sl.tessellate(0.002, 0.05)
    return max(math.hypot(q.X, q.Y) for q in v)


def check(shapes):
    rep = {}
    tr, tv = shapes['_roller_tread_relief'], shapes['_roller_tread']
    rep['roller_tread_relief'] = dict(solids=len(tr.solids()), valid=bool(tr.is_valid), volume_mm3=round(tr.volume, 1),
                                      v13_volume_mm3=round(tv.volume, 1))
    rep['relief'] = dict(len=RELIEF_LEN, depth_at_end=RELIEF_DEPTH, arc_radius=round(RELIEF_R, 2), starts_at_z=round(Z0, 2),
                         handover_z=round(Z_HANDOVER, 2))
    # profile, CAD sections against the formula, drop below the v13 tread (the R70 circle except in the end fillet)
    prof = {}
    for z in (0.0, Z0, (Z0 + Z_HANDOVER) / 2, Z_HANDOVER, 18.5, 19.0):
        cad = section_radius(tr, z)
        prof[f'z {z:.2f}'] = dict(cad=round(cad, 3), formula=round(rt_relief(z), 3) if z < HT - 1.6 else None,
                                  below_v13=round(section_radius(tv, z) - cad, 3))
    rep['profile_mm (radius about the roller axis; below_v13 = the relief there)'] = prof
    rep['unloaded_upright_dip_at_handover_mm'] = round(section_radius(tv, Z_HANDOVER) - section_radius(tr, Z_HANDOVER), 3)
    # the tread's interfaces are v13's: the same overlap with the core (keys in the keyways, detent rib in the groove)
    core = shapes['_roller_core_687']
    rep['tread_core_overlap_mm3 (want = v13)'] = dict(relief=round(wm.overlap(tr, core), 3), v13=round(wm.overlap(tv, core), 3))
    hub = shapes['wheel_hub_687']
    gaps = {}
    for rk in ('1_1', '1_2', '2_1', '2_2'):
        gaps[rk] = dict(relief=round(wm.min_dist(hub, shapes[f'roller_tread_relief_{rk}']), 3))
    rep['tread_to_hub_min_gap_mm (wheel_quiet: 0.95)'] = gaps
    rep['tread_hub_overlap_mm3 (want 0)'] = round(sum(wm.overlap(hub, s) for k, s in shapes.items()
                                                      if k.startswith('roller_tread_relief_') and wm.near(hub, s)), 3)
    return rep


def export(shapes):
    parts = OUT / 'parts'
    parts.mkdir(parents=True, exist_ok=True)
    for f in list(parts.glob('*.step')) + list(OUT.glob('*.step')):
        f.unlink()
    write_part(shapes['_roller_tread_relief'], wq.colour('roller_tread'), parts / 'roller_tread_relief.step', 'roller_tread_relief')
    skip = ('bearing_', '_')            # no bearings or fasteners in any STEP; '_' = unplaced copies
    write_step({k: (s, wq.colour(k)) for k, s in shapes.items() if not k.startswith(skip)}, OUT / 'wheel_quiet_2_assembly.step',
               'wheel_quiet_2')


def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tr, tv = shapes['_roller_tread_relief'], shapes['_roller_tread']
    zs = np.linspace(0, HT - 0.01, 140)
    rv = np.array([section_radius(tv, z) for z in zs])
    rr = np.array([section_radius(tr, z) for z in zs])
    fig, axs = plt.subplots(1, 2, figsize=(15, 5.5))
    ax = axs[0]
    ax.plot(zs, rv, color='0.5', label='v13 tread (R70 + R2 end fillet)')
    ax.plot(zs, rr, color='C3', label=f'round 1: R{RELIEF_R:.1f} tangent relief over the last {RELIEF_LEN:g} mm')
    ax.axvline(Z_HANDOVER, color='k', ls=':', lw=1)
    ax.text(Z_HANDOVER - 0.2, rv.min() + 0.3, 'upright row handover\n(z 18.1, 15 deg)', ha='right', fontsize=8)
    ax.set_xlabel('local z along the roller (mm, 0 = middle)')
    ax.set_ylabel('tread radius about the roller axis (mm)')
    ax.set_title('one half of the tread profile (CAD sections)')
    ax.legend(fontsize=8)
    ax.grid(lw=.3)
    ax = axs[1]
    ax.plot(zs, (rv - rr) * 1000, color='C3')
    ax.axvline(Z_HANDOVER, color='k', ls=':', lw=1)
    ax.axvline(Z0, color='C3', ls='--', lw=1)
    ax.set_xlim(Z0 - 3, HT)
    ax.set_xlabel('local z (mm)')
    ax.set_ylabel('relief = v13 radius - round 1 radius (um)')
    ax.set_title('the relief (beyond z 18 the R2 fillet starts on both)')
    ax.grid(lw=.3)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_2_profile.png', dpi=90)
    plt.close(fig)
    # roller 1_1 cut along its axle, both treads overlaid
    M = {(r, k): M for r, k, M in wm.roller_frames()}[(1, 1)]
    inv = wm.loc(M).inverse()
    from matplotlib.collections import PolyCollection
    fig, ax = plt.subplots(figsize=(12, 6))
    for name, col in (('_roller_tread', (0.75, 0.75, 0.75)), ('roller_tread_relief_1_1', (0.20, 0.24, 0.32)),
                      ('roller_core_687_1_1', (0.38, 0.66, 0.38)), ('roller_sleeve_687_1_1', (0.15, 0.55, 0.85))):
        s = shapes[name] if name.startswith('_') else shapes[name].moved(inv)
        x = s.rotate(b.Axis.Y, 90) & b.Box(400, 400, 0.6)
        if x is None or not x.faces():
            continue
        v, t = x.tessellate(0.03, 0.1)
        Vv = np.array([[q.X, q.Y] for q in v])
        ax.add_collection(PolyCollection([Vv[tri] for tri in np.array(t)], facecolors=[col], edgecolors='none'))
    ax.set_xlim(-24, 24)
    ax.set_ylim(-2, 20)
    ax.set_aspect('equal')
    ax.grid(lw=.3)
    ax.set_title(f'roller cut along its axle (upper half): v13 tread grey behind, relieved tread dark ({RELIEF_DEPTH:g} mm relief: '
                 'look at the ends), core green, sleeve blue', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_2_section.png', dpi=90)
    plt.close(fig)


def compare():
    """before / after picture of the TPU tread alone (CAD sections of wm.tread_solid() and tread_relief()): the whole
    roller at true scale with a zoom on one end, and both profiles' drop below the R70 rolling circle along the roller"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    tv, tr = wm.tread_solid(), tread_relief()
    zh = np.concatenate([np.linspace(0, 17.5, 36), np.linspace(17.55, HT - 0.005, 60)])
    rv = np.array([section_radius(tv, z) for z in zh])
    rr = np.array([section_radius(tr, z) for z in zh])
    z = np.concatenate([-zh[::-1], zh[1:]])
    rv, rr = np.concatenate([rv[::-1], rv[1:]]), np.concatenate([rr[::-1], rr[1:]])
    circ = np.array([rt(min(abs(q), HT)) for q in z])          # the R70 rolling circle (no end fillet)
    BEF, AFT = '0.45', 'C3'

    fig = plt.figure(figsize=(15, 9.5))
    gs = fig.add_gridspec(2, 3, height_ratios=[1, 1.05], width_ratios=[1, 1, 0.9])
    for i, (r, col, name) in enumerate(((rv, BEF, 'BEFORE: v13 tread (R70 arc + R2 end fillet)'),
                                         (rr, AFT, f'AFTER: wheel_quiet_2 ({RELIEF_DEPTH:g} mm tip relief over the last '
                                                   f'{RELIEF_LEN:g} mm)'))):
        ax = fig.add_subplot(gs[0, i])
        ax.fill_between(z, CORE_R, r, color=col, alpha=0.35, lw=0)
        ax.fill_between(z, -r, -CORE_R, color=col, alpha=0.35, lw=0)
        ax.plot(z, r, color=col, lw=1.5)
        ax.plot(z, -r, color=col, lw=1.5)
        ax.plot(z, circ, 'k:', lw=0.8)
        ax.axhline(0, color='k', lw=0.5, ls='-.')
        ax.set_aspect('equal')
        ax.set_xlim(-21, 21)
        ax.set_ylim(-20, 20)
        ax.set_title(name, fontsize=10)
        ax.set_xlabel('along the roller (mm)')
        ax.set_ylabel('radius about the roller axis (mm)')
        ax.grid(lw=.3)
        ax.text(0, 0.6, 'TPU tread section, true scale (bore Ø%.1f; keys, rib not drawn)' % (2 * CORE_R), ha='center', fontsize=7)
    ax = fig.add_subplot(gs[0, 2])
    ax.plot(z, rv, color=BEF, lw=2, label='before (v13)')
    ax.plot(z, rr, color=AFT, lw=2, label='after')
    ax.plot(z, circ, 'k:', lw=0.8, label='R70 rolling circle')
    for zz, t in ((Z0, 'relief starts\nz 12.5'), (Z_HANDOVER, 'row handover\nz 18.1')):
        ax.axvline(zz, color='k', ls='--' if zz == Z0 else ':', lw=0.8)
        ax.text(zz - 0.15, 14.15, t, ha='right', fontsize=7)
    ax.set_xlim(11, HT + 0.2)
    ax.set_ylim(14.0, 17.3)
    ax.set_aspect('equal')
    ax.set_title('zoom on one end, true scale', fontsize=10)
    ax.set_xlabel('along the roller (mm)')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(lw=.3)

    ax = fig.add_subplot(gs[1, :])
    ax.plot(z, (circ - rv) * 1000, color=BEF, lw=2, label='before (v13): on the circle until the R2 end fillet')
    ax.plot(z, (circ - rr) * 1000, color=AFT, lw=2,
            label=f'after: leaves the circle at |z| {Z0:g} on a tangent R{RELIEF_R:.1f} arc')
    for s in (-1, 1):
        ax.axvline(s * Z0, color='k', ls='--', lw=0.8)
        ax.axvline(s * Z_HANDOVER, color='k', ls=':', lw=0.8)
    hv = (circ - rr)[np.argmin(abs(z - Z_HANDOVER))] * 1000
    ax.annotate(f'{hv:.0f} µm at the handover', (Z_HANDOVER, hv), (Z_HANDOVER - 7, 250), fontsize=9,
                arrowprops=dict(arrowstyle='->', lw=0.8))
    ax.text(0, 330, 'drop below the R70 rolling circle (µm; + = inside it, the floor sees less tread). '
                    'The rows hand over at z ±18.1 (upright); the R2 end fillet takes both treads away beyond ~±18',
            ha='center', fontsize=9)
    ax.set_ylim(-20, 400)
    ax.set_xlim(-HT - 0.3, HT + 0.3)
    ax.invert_yaxis()
    ax.set_xlabel('along the roller (mm, 0 = middle)')
    ax.set_ylabel('drop below the R70 circle (µm)')
    ax.legend(fontsize=9, loc='lower center')
    ax.grid(lw=.3)
    fig.suptitle('TPU roller tread: before / after the tip relief (CAD sections)', fontsize=12)
    fig.tight_layout()
    fig.savefig(VAL / 'wheel_quiet_2_before_after.png', dpi=100)
    plt.close(fig)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    if stage == 'compare':
        compare()
        print('rendered', VAL / 'wheel_quiet_2_before_after.png', flush=True)
        return
    shapes, info = build_all()
    print('build', info, flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes)
        rep['build'] = info
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1))
        print(json.dumps(rep, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')

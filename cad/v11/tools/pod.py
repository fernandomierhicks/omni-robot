"""cad/v11 compact suspension pod for the wheel + motor (Nov 3 2026), build123d, STEP review only. Dimensions in dims.py.

    python -B pod.py [build|check|export|render|all]        (omni-robot-sim interpreter)

A module that drops into a cutout in the flat robot plate (designed later) and is held by 6 vertical M3 screws from the top:
 * two side frames (printed, each lying on its flat outer face): a foot on the plate, a spring tower over the fork arm, a keel down through a
   plate notch that carries the Ø7 pivot shaft at plate height, level with the axle (80.6 mm arm, design.md 2), and the droop / bump ledges.
 * fork (one printed part): outer arm with a bolt-in axle stub into the wheel's left 6804, inner arm bolted straight to the AK40-10's rear
   face, beam behind the wheel, 2 x 627, a pad under each spring plunger.
 * springs: one ISO 10243 die spring per side (Ø10 x 38, green or blue by robot mass) on an M5 x 50 button-head plunger that presses on the
   fork's pad; no damper. The ledges stop the tail at -3 / +15 mm.
Mirror-hand pods (opposite diagonals) are not generated yet. Everything is nominal CAD; nothing has been printed.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b

sys.path.insert(0, str(Path(__file__).resolve().parent))
import wheel_motor as W                                                # noqa: E402
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                         # noqa: E402

loc, cyl, along_x, V, arc, line, prism_x = W.loc, W.cyl, W.along_x, W.V, W.arc, W.line, W.prism_x
overlap, near, min_dist, points = W.overlap, W.near, W.min_dist, W.points
ROOT = W.ROOT
OUT = ROOT / 'step_export' / 'pod'
VAL = ROOT / 'validation' / 'pod'
PIVOT_AXIS = b.Axis((0, PIVOT_Y, PIVOT_Z), (1, 0, 0))
EXTRA_COLOURS = {'pod_frame': (0.25, 0.68, 0.74), 'pod_fork': (0.95, 0.75, 0.15), 'axle_stub': (0.66, 0.46, 0.86), 'die_spring': (0.15, 0.65, 0.25),
                 'spring_plunger': (1.0, 0.0, 1.0), 'pivot_shaft': (0.75, 0.75, 0.78), 'bearing': (0.30, 0.30, 0.32)}


def colour(name):
    for k, v in EXTRA_COLOURS.items():
        if name.startswith(k):
            return v
    return W.colour(name)


# ---- geometry helpers ----------------------------------------------------------------------------------------------------
def box(x0, x1, y0, y1, z0, z1):
    return b.Box(x1 - x0, y1 - y0, z1 - z0).moved(b.Location(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)))


def prism(edges, x0, x1):
    return prism_x(edges, (x1 - x0) / 2).moved(b.Location(((x0 + x1) / 2, 0, 0)))


def hull2c(c1, r1, c2, r2):
    """y-z outline of the convex hull of two circles (real arcs and tangent lines)"""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    d = c2 - c1
    D = np.linalg.norm(d)
    phi, al = math.atan2(d[1], d[0]), math.acos((r1 - r2) / D)
    u = d / D
    t = lambda c, r, s: c + r * np.array([math.cos(phi + s * al), math.sin(phi + s * al)])   # noqa: E731
    a1, a2, b1, b2 = t(c1, r1, 1), t(c2, r2, 1), t(c1, r1, -1), t(c2, r2, -1)
    return [line(V(*a1), V(*a2)), arc(V(*a2), V(*(c2 + r2 * u)), V(*b2)), line(V(*b2), V(*b1)), arc(V(*b1), V(*(c1 - r1 * u)), V(*a1))]


def rot_pt(p, a):
    """(y, z) turned by a deg about the pivot axis (right hand about +x, as Shape.rotate)"""
    y, z = p[0] - PIVOT_Y, p[1] - PIVOT_Z
    c, s = math.cos(math.radians(a)), math.sin(math.radians(a))
    return np.array([PIVOT_Y + y * c - z * s, PIVOT_Z + y * s + z * c])


def angle_for(dz):
    """fork angle that moves the axle dz up (bisection; negative angles raise the axle)"""
    lo, hi = -40.0, 40.0
    for _ in range(80):
        m = (lo + hi) / 2
        if rot_pt((0.0, 0.0), m)[1] > dz:
            lo = m
        else:
            hi = m
    return (lo + hi) / 2


A_DROOP, A_NOM, A_BUMP = angle_for(-DROOP), 0.0, angle_for(BUMP)
POSES = {'droop': A_DROOP, 'nominal': A_NOM, 'bump': A_BUMP}

# ---- springs and plungers ------------------------------------------------------------------------------------------------
Y_S = PIVOT_Y - SPRING_ARM
X_T = ((OUTER_ARM_X[0] + OUTER_ARM_X[1]) / 2, (INNER_ARM_X[0] + INNER_ARM_X[1]) / 2)     # spring axes, outer / inner
DOME_R = ((BOLT_HEAD_D / 2) ** 2 + BOLT_HEAD_K ** 2) / (2 * BOLT_HEAD_K)


def spring_force(mass):
    """per spring at nominal: the wheel's share of the robot's weight through the lever, two springs"""
    return mass * 9.81 / 4 * PIVOT_Y / SPRING_ARM / 2


K_DESIGN, MAXW_DESIGN = SPRINGS[SPRING_DESIGN]
C_NOM = spring_force(DESIGN_MASS) / K_DESIGN
Z_SB = PAD_Z + BOLT_HEAD_K + WASHER_T                   # spring bottom seat (on the washer), nominal
Z_CEIL = Z_SB + SPRING_L0 - C_NOM                       # spring top seat in the tower
Z_TT = Z_CEIL + GUIDE_LEN                               # tower top
Z_TBOT = PAD_Z + TOWER_CLEAR                            # tower underside
BOLT_L = math.ceil((Z_TT + NUT_GAP + NUT_T + 1.0 - (PAD_Z + BOLT_HEAD_K)) / 5.0) * 5.0


def plunger_lift(a):
    """how far the plunger rises at fork angle a: its dome (a sphere on the vertical axis) stays tangent to the turned pad plane"""
    p0 = rot_pt((Y_S, PAD_Z), a)
    n = rot_pt((Y_S, PAD_Z + 1.0), a) - p0
    zc0 = PAD_Z + DOME_R
    zc = p0[1] + (DOME_R - n[0] * (Y_S - p0[0])) / n[1]             # n . ((Y_S, zc) - p0) = DOME_R
    contact = np.array([Y_S, zc]) - DOME_R * n
    along = float((contact - p0) @ np.array([n[1], -n[0]]))      # contact point from the pad centre, along the pad
    return zc - zc0, along


# ---- fork --------------------------------------------------------------------------------------------------------------------
def lead_azimuth():
    """azimuth of the XT30 pins (the leads leave there): the plug housing's centroid"""
    _, housing, _ = W.motor_and_plug()
    c = housing.center()
    return math.degrees(math.atan2(c.Z, c.Y))


def build_fork():
    piv = (PIVOT_Y, PIVOT_Z)
    f = prism(hull2c((0, 0), OUTER_BOSS_R, piv, LOBE_R), *OUTER_ARM_X)
    f = f + prism(hull2c((0, 0), INNER_PLATE_R, piv, LOBE_R), *INNER_ARM_X)
    y, z = W.at_angle(LEAD_NOTCH_R + 12.0, lead_azimuth())             # notch where the leads pass (they move with the fork)
    f = f - along_x(12.0, INNER_ARM_X[0] - 0.1, INNER_ARM_X[1] + 0.1, y, z)
    beam = prism(hull2c(piv, LOBE_R, (BEAM_TAIL[0], PIVOT_Z), BEAM_TAIL[1]), OUTER_ARM_X[0], INNER_ARM_X[1])
    f = f + (beam - along_x(WHEEL_CLEAR_R, OUTER_ARM_X[1], INNER_ARM_X[0]))
    for ax in (OUTER_ARM_X, INNER_ARM_X):                              # spring pads
        f = f + box(ax[0], ax[1], Y_S - PAD_HALF, Y_S + PAD_HALF, 5.0, PAD_Z)
    f = f - along_x(SHAFT_CLR / 2, OUTER_ARM_X[0] - 1, INNER_ARM_X[1] + 1, PIVOT_Y, PIVOT_Z)
    f = f - along_x(B627_OD / 2, OUTER_ARM_X[0] - 0.1, OUTER_ARM_X[0] + B627_W, PIVOT_Y, PIVOT_Z)
    f = f - along_x(B627_OD / 2, INNER_ARM_X[1] - B627_W, INNER_ARM_X[1] + 0.1, PIVOT_Y, PIVOT_Z)
    head_x = INNER_ARM_X[0] + REAR_SCREW_L - REAR_ENGAGE                # motor rear face: 4 x M3 x 12 in counterbores
    for a in REAR_HOLES_AZ:
        y, z = W.at_angle(REAR_HOLES_R, a)
        f = f - along_x(M3_CLR / 2, INNER_ARM_X[0] - 0.1, INNER_ARM_X[1] + 0.1, y, z) - along_x(REAR_CBORE_D / 2, head_x, INNER_ARM_X[1] + 0.1, y, z)
    f = f - along_x(STUB_SHOULDER_D / 2 + 0.2, OUTER_ARM_X[0] - 0.1, OUTER_ARM_X[1] + 0.1)
    for a in (90, 210, 330):
        y, z = W.at_angle(STUB_SCREW_R, a)
        f = f - along_x(M3_PILOT / 2, OUTER_ARM_X[0] - 0.1, OUTER_ARM_X[0] + 9.0, y, z)
    return f


def build_stub():
    x_inner_race = X_PLATE_FLOOR                                  # left 6804 from x -36.5 to -29.5 in the cartridge
    xs0 = x_inner_race - STUB_SHOULDER_GAP
    s = along_x(STUB_D / 2, xs0 - 0.01, x_inner_race + 7.0)
    s = s + along_x(STUB_SHOULDER_D / 2, OUTER_ARM_X[0], xs0)
    s = s + along_x(STUB_FLANGE_D / 2, OUTER_ARM_X[0] - STUB_FLANGE_T, OUTER_ARM_X[0])
    for a in (90, 210, 330):
        y, z = W.at_angle(STUB_SCREW_R, a)
        s = s - along_x(M3_CLR / 2, OUTER_ARM_X[0] - STUB_FLANGE_T - 0.1, OUTER_ARM_X[0] + 0.1, y, z)
    return s


# ---- side frames -------------------------------------------------------------------------------------------------------------
FRAMES = {'outer': dict(x=(OUTER_ARM_X[0] - FRAME_GAP - FRAME_T, OUTER_ARM_X[0] - FRAME_GAP), inward=1, xt=X_T[0]),
          'inner': dict(x=(INNER_ARM_X[1] + FRAME_GAP, INNER_ARM_X[1] + FRAME_GAP + FRAME_T), inward=-1, xt=X_T[1])}


def ledge_x(fr):
    x0, x1 = fr['x']
    return (x1, x1 + LEDGE_W) if fr['inward'] > 0 else (x0 - LEDGE_W, x0)


def stops(fork):
    """droop ledge underside = the fork's highest point under it at droop; bump ledge top = its lowest point at bump"""
    out = {}
    for name, fr in FRAMES.items():
        lx = ledge_x(fr)
        probe = box(lx[0], lx[1], STOP_Y[0], STOP_Y[1], -60, 60)
        hi = (fork.rotate(PIVOT_AXIS, A_DROOP) & probe).bounding_box().max.Z
        lo = (fork.rotate(PIVOT_AXIS, A_BUMP) & probe).bounding_box().min.Z
        out[name] = (float(lo), float(hi))
    return out


def build_frame(name, z_bump, z_droop):
    fr = FRAMES[name]
    x0, x1 = fr['x']
    s, xt = fr['inward'], fr['xt']
    xc = (x0 + x1) / 2
    top_foot = PLATE_TOP_Z + FOOT_T
    z_bot = z_bump - 6.0
    f = box(x0, x1, FOOT_Y[0], FOOT_Y[1], PLATE_TOP_Z, top_foot)                          # foot on the plate
    f = f + box(x0, x1, Y_S - 9.0, Y_S + 9.0, PLATE_TOP_Z, Z_TT)                            # spring column
    tx = (x1, xt + 8.0) if s > 0 else (xt - 8.0, x0)
    f = f + box(tx[0], tx[1], Y_S - 9.0, Y_S + 9.0, Z_TBOT, Z_TT)                         # tower over the fork arm
    f = f + box(x0, x1, KEEL_Y[0], KEEL_Y[1], z_bot, max(top_foot, z_droop + 6.0))         # keel (through the plate notch)
    lx = ledge_x(fr)
    f = f + box(lx[0], lx[1], STOP_Y[0], STOP_Y[1], z_droop, z_droop + 6.0)               # droop ledge over the tail
    f = f + box(lx[0], lx[1], STOP_Y[0], STOP_Y[1], z_bot, z_bump)                        # bump ledge under the tail
    f = f - cyl((SPRING_HOLE_D + 0.6) / 2, Z_CEIL - Z_TBOT + 0.1, (xt, Y_S, (Z_CEIL + Z_TBOT) / 2 - 0.05))
    f = f - cyl(2.7, GUIDE_LEN + 0.2, (xt, Y_S, Z_CEIL + GUIDE_LEN / 2))
    f = f - along_x(SHAFT_D / 2 + 0.05, x0 - 1, x1 + 1, PIVOT_Y, PIVOT_Z)
    if name == 'inner':                                                                       # cross pin through the keel and the shaft
        top = PIVOT_Z + SHAFT_D / 2
        f = f - cyl(M3_CLR / 2, top - z_bot + 0.1, (xc, PIVOT_Y, (top + z_bot) / 2)) - cyl(M3_PILOT / 2, 7.0, (xc, PIVOT_Y, top + 3.5))
    for y in MOUNT_Y:
        f = f - cyl(M3_CLR / 2, FOOT_T + 0.2, (xc, y, PLATE_TOP_Z + FOOT_T / 2))
    return f, dict(x=[round(x0, 2), round(x1, 2)], z_bottom=round(z_bot, 2), z_top=round(Z_TT, 2), ledges=dict(bump_top=round(z_bump, 2), droop_underside=round(z_droop, 2)))


def build_shaft():
    x0, x1 = FRAMES['outer']['x'][0], FRAMES['inner']['x'][1]
    xc = sum(FRAMES['inner']['x']) / 2
    return along_x(SHAFT_D / 2, x0, x1, PIVOT_Y, PIVOT_Z) - cyl(M3_CLR / 2, SHAFT_D + 1, (xc, PIVOT_Y, PIVOT_Z))


def bearings_627():
    ring = lambda x0: along_x(B627_OD / 2, x0, x0 + B627_W, PIVOT_Y, PIVOT_Z) - along_x(SHAFT_D / 2, x0 - 1, x0 + B627_W + 1, PIVOT_Y, PIVOT_Z)  # noqa: E731
    return ring(OUTER_ARM_X[0]), ring(INNER_ARM_X[1] - B627_W)


def plunger(xt):
    """M5 x BOLT_L button head + washer + nyloc as one body, at nominal (dome apex on the pad)"""
    cap = b.Sphere(DOME_R).moved(b.Location((0, 0, DOME_R))) & b.Cylinder(BOLT_HEAD_D / 2, BOLT_HEAD_K).moved(b.Location((0, 0, BOLT_HEAD_K / 2)))
    z_head = BOLT_HEAD_K
    p = cap + cyl(2.5, BOLT_L, (0, 0, z_head + BOLT_L / 2))
    p = p + (cyl(WASHER_OD / 2, WASHER_T, (0, 0, z_head + WASHER_T / 2)) - cyl(2.7, WASHER_T + 1, (0, 0, z_head + WASHER_T / 2)))
    zn = Z_TT + NUT_GAP - PAD_Z
    p = p + (b.extrude(b.RegularPolygon(4.0, 6), NUT_T).moved(b.Location((0, 0, zn))) - cyl(2.5, NUT_T + 1, (0, 0, zn + NUT_T / 2)))
    return p.moved(b.Location((xt, Y_S, PAD_Z)))


def die_spring(xt, length):
    z0 = Z_SB
    return cyl(SPRING_OD / 2, length, (xt, Y_S, z0 + length / 2)) - cyl(SPRING_ID / 2, length + 1, (xt, Y_S, z0 + length / 2))


# ---- assembly at a pose ----------------------------------------------------------------------------------------------------------
WHEEL_KEEP = ('wheel_hub', 'wheel_cartridge', 'ak40_stator', 'ak40_rotor', 'xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg', 'bearing_6804_left')


def wheel_unit():
    wm, _ = W.build_all()
    unit = {k: wm[k] for k in WHEEL_KEEP}
    unit.update({k: v for k, v in wm.items() if k.startswith(('roller_core_', 'roller_tread_', 'bolt_'))})
    return unit


def build_static():
    fork = build_fork()
    st = stops(fork)
    parts = dict(fork=fork, stub=build_stub(), shaft=build_shaft(), b627=bearings_627())
    info = {}
    for name in FRAMES:
        parts['frame_' + name], info[name] = build_frame(name, *st[name])
    return parts, info


def posed(parts, unit, a):
    moving = dict(unit, pod_fork=parts['fork'], axle_stub=parts['stub'], bearing_627_outer=parts['b627'][0], bearing_627_inner=parts['b627'][1])
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    out['pod_frame_outer'], out['pod_frame_inner'] = parts['frame_outer'], parts['frame_inner']
    out['pivot_shaft'] = parts['shaft']
    lift, _ = plunger_lift(a)
    for side, xt in zip(('outer', 'inner'), X_T):
        out[f'spring_plunger_{side}'] = plunger(xt).moved(b.Location((0, 0, lift)))
        out[f'die_spring_{side}'] = die_spring(xt, Z_CEIL - Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, lift


# ---- checks ------------------------------------------------------------------------------------------------------------------
def spring_table():
    rows = {}
    lift_b, lift_d = plunger_lift(A_BUMP)[0], plunger_lift(A_DROOP)[0]
    for kg in (10, 12.5, 15, 20):
        F = spring_force(kg)
        r = dict(force_nominal_N=round(F, 1))
        for colour_, (k, maxw) in SPRINGS.items():
            c = F / k
            r[colour_] = dict(wheel_rate_N_per_mm=round(2 * k * (SPRING_ARM / PIVOT_Y) ** 2, 2), compression_nominal=round(c, 1),
                              compression_bump=round(c + lift_b, 1), max_working=maxw, ok=bool(c + lift_b <= maxw and c + lift_d > 0.5),
                              shim_mm=round(c - C_NOM, 1))
        rows[f'{kg} kg'] = r
    return rows


def check(parts, unit):
    rep = dict(poses_deg={k: round(v, 3) for k, v in POSES.items()},
               springs=dict(part='ISO 10243 Ø10 x 38 (HPC R203106 green / R204106 blue)', design=f'{SPRING_DESIGN} at {DESIGN_MASS} kg', arm_mm=SPRING_ARM,
                            plunger=f'M5 x {BOLT_L:g} button head, M5 washer, M5 nyloc', table=spring_table(),
                            note='shim_mm: spacer under the spring seat to keep the nominal ride height with that spring and mass (negative: the pod rides higher)'))
    static = ('pod_frame_outer', 'pod_frame_inner', 'pivot_shaft')
    rotating = [k for k in unit if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_6804'))]
    harness = ('xt30_housing', 'lead_pos', 'lead_neg')
    for name, a in POSES.items():
        sh, lift = posed(parts, unit, a)
        _, along = plunger_lift(a)
        r = dict(axle=[round(float(v), 2) for v in rot_pt((0, 0), a)], plunger_lift=round(lift, 2), contact_from_pad_centre=round(along, 2),
                 spring_length=round(Z_CEIL - Z_SB - lift, 2))
        ov = {}
        moving = [k for k in sh if k not in static and not k.startswith(('spring_plunger', 'die_spring'))]
        for st in static:
            for k in moving:
                if st == 'pivot_shaft' and k.startswith('bearing_627'):
                    continue
                if near(sh[st], sh[k]):
                    v = overlap(sh[st], sh[k])
                    if v > 0.01:
                        ov[f'{st} / {k}'] = round(v, 2)
        for side in ('outer', 'inner'):
            for k in sh:
                if k.startswith(('spring_plunger', 'die_spring')) or not near(sh[f'spring_plunger_{side}'], sh[k]):
                    continue
                v = overlap(sh[f'spring_plunger_{side}'], sh[k])
                if v > 0.05:
                    ov[f'spring_plunger_{side} / {k}'] = round(v, 2)
        r['interference_mm3'] = ov
        gaps = dict(frames_to_fork=round(min(min_dist(sh[f], sh['pod_fork']) for f in static[:2]), 2),
                    frames_to_rotating=round(min(min_dist(sh[f], sh[k]) for f in static[:2] for k in rotating if near(sh[f], sh[k], 8)), 2),
                    frames_to_harness=round(min(min_dist(sh[f], sh[k]) for f in static[:2] for k in harness), 2))
        r['gaps_mm'] = gaps
        if name == 'nominal':
            r['fork_to_rotating'] = round(min(min_dist(sh['pod_fork'], sh[k]) for k in rotating if near(sh['pod_fork'], sh[k], 6)), 2)
            r['fork_to_harness'] = round(min(min_dist(sh['pod_fork'], sh[k]) for k in harness), 2)
            r['stub_to_rotating'] = round(min(min_dist(sh['axle_stub'], sh[k]) for k in rotating if k != 'bearing_6804_left' and near(sh['axle_stub'], sh[k], 6)), 2)
        rep[name] = r
    # what the plate needs: everything that crosses its thickness, over the travel
    slab = box(-200, 200, -200, 200, PLATE_TOP_Z - PLATE_T_ASSUMED, PLATE_TOP_Z)
    lo, hi = np.array([1e9, 1e9]), np.array([-1e9, -1e9])
    for a in POSES.values():
        sh, _ = posed(parts, unit, a)
        for k, s in sh.items():
            if k.startswith(('pod_frame', 'pivot_shaft', 'spring_', 'die_')) or not near(s, slab, 0):
                continue
            x = s & slab
            if x is None or not x.solids():
                continue
            bb = x.bounding_box()
            lo, hi = np.minimum(lo, [bb.min.X, bb.min.Y]), np.maximum(hi, [bb.max.X, bb.max.Y])
    rep['plate'] = dict(top_z_above_axle=PLATE_TOP_Z, top_above_floor=round(PLATE_TOP_Z + R_WHEEL, 1),
                        cutout_xy_bbox_moving_parts=[[round(float(v), 1) for v in lo], [round(float(v), 1) for v in hi]],
                        keel_notches={n: dict(x=FRAMES[n]['x'], y=list(KEEL_Y)) for n in FRAMES},
                        mount_screws={n: [[round(sum(FRAMES[n]['x']) / 2, 2), y] for y in MOUNT_Y] for n in FRAMES},
                        note=f'plate thickness assumed {PLATE_T_ASSUMED} mm for the cutout; frames stand on the plate top; M3 from the top into Ø2.8 pilots in the plate (or nuts under it)')
    rep['printed_parts'] = {k: dict(solids=len(parts[p].solids()), valid=bool(parts[p].is_valid), volume_cm3=round(parts[p].volume / 1000, 1),
                                    bbox=[round(v, 1) for v in tuple(parts[p].bounding_box().size)])
                            for k, p in (('pod_frame_outer', 'frame_outer'), ('pod_frame_inner', 'frame_inner'), ('pod_fork', 'fork'), ('axle_stub', 'stub'))}
    return rep


# ---- export / render -------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_', 'bearing_', 'pivot_shaft', 'spring_plunger')
PRINTED = (('pod_frame_outer', 'frame_outer'), ('pod_frame_inner', 'frame_inner'), ('pod_fork', 'fork'), ('axle_stub', 'stub'))


def export(parts, unit):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for n, p in PRINTED:
        write_part(parts[p], colour(n), OUT / 'parts' / (n + '.step'), n)
    sh, _ = posed(parts, unit, 0.0)
    write_step({k: (v, colour(k)) for k, v in sh.items() if not k.startswith(STEP_SKIP)}, OUT / 'pod_assembly.step', 'wheel_pod')


def render(parts, unit):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    def section(sh, x0, ax, title):
        sl = box(x0 - 0.3, x0 + 0.3, -400, 400, -400, 400)
        polys = []
        for k, s in sh.items():
            if not near(s, sl, 0):
                continue
            x = s & sl
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.15, 0.2)
            P = np.array([[q.Y, q.Z] for q in v])
            polys += [(P[tri], colour(k)) for tri in np.array(t)]
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.axhline(-R_WHEEL, color='k', lw=0.8)
        ax.axhline(PLATE_TOP_Z, color='0.4', lw=0.8, ls='--')
        ax.set_xlim(-80, 130)
        ax.set_ylim(-75, 95)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(title, fontsize=9)

    def iso(items, out, az_deg, el, title):
        a, e = np.radians(az_deg), np.radians(el)
        Rm = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]]) @ np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        polys = []
        for k, s in items.items():
            v, t = s.tessellate(0.4, 0.4)
            P3 = np.array([[q.X, q.Y, q.Z] for q in v]) @ Rm.T
            col = np.array(colour(k))
            for tri in np.array(t):
                P = P3[tri]
                n = np.cross(P[1] - P[0], P[2] - P[0])
                nn = np.linalg.norm(n)
                if nn == 0 or n[1] > 0:
                    continue
                polys.append((P[:, 1].mean(), P[:, [0, 2]], tuple(col * (0.45 + 0.55 * abs(n[1] / nn)))))
        polys.sort(key=lambda p: -p[0])
        fig, ax = plt.subplots(figsize=(11, 9))
        ax.add_collection(PolyCollection([p[1] for p in polys], facecolors=[p[2] for p in polys], edgecolors='none'))
        ax.autoscale()
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(title)
        fig.savefig(out, dpi=95)
        plt.close(fig)

    for f in list(VAL.glob('iso_*.png')) + list(VAL.glob('section_*.png')):
        f.unlink()
    sh, _ = posed(parts, unit, 0.0)
    keep = {k: v for k, v in sh.items() if not k.startswith(('bolt_', 'roller_core'))}
    iso(keep, VAL / 'iso_pod_motor_side.png', 235, 22, 'pod, motor side (nominal)')
    iso(keep, VAL / 'iso_pod_outer_side.png', 55, 22, 'pod, outer side (nominal)')
    iso({k: sh[k] for k in sh if k.startswith(('pod_fork', 'axle_stub', 'pod_frame', 'pivot_shaft', 'spring_plunger', 'die_spring'))},
        VAL / 'iso_frames_fork.png', 215, 30, 'side frames, fork, shaft, spring plungers (wheel hidden)')
    for xs, nm in ((X_T[0], 'outer_spring_plane'), (0.0, 'mid_plane'), (FRAMES['outer']['x'][0] + 0.5, 'outer_frame')):
        fig, axs = plt.subplots(1, 3, figsize=(18, 6))
        for ax, (pname, a) in zip(axs, POSES.items()):
            sh, lift = posed(parts, unit, a)
            section(sh, xs, ax, f'{pname}: x = {xs:g}, plunger lift {lift:+.1f} (y toward the pivot, z up; dashed: plate top)')
        fig.tight_layout()
        fig.savefig(VAL / f'section_{nm}.png', dpi=90)
        plt.close(fig)


def main(stage):
    for d in (OUT, VAL):
        d.mkdir(parents=True, exist_ok=True)
    parts, finfo = build_static()
    print('frames', finfo, 'spring nominal compression', round(C_NOM, 2), 'tower top', round(Z_TT, 2), 'bolt', BOLT_L, flush=True)
    for k in ('fork', 'stub', 'frame_outer', 'frame_inner'):
        print(k, len(parts[k].solids()), parts[k].is_valid, round(parts[k].volume), flush=True)
    if stage == 'build':
        return
    unit = wheel_unit()
    if stage in ('check', 'all'):
        rep = check(parts, unit)
        rep['frames'] = finfo
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k != 'springs'}, indent=1), flush=True)
        print(json.dumps(rep['springs']['table'], indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, unit)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, unit)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')

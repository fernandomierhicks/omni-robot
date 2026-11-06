"""cad/v11 compact suspension pod for the wheel + motor (Nov 3 2026), build123d, STEP review only. Dimensions in dims.py.

    python -B pod.py [build|check|export|render|all]        (omni-robot-sim interpreter)

A module that drops into a cutout in the flat robot plate (designed later) and is held by 6 vertical M3 screws from the top:
 * two side frames (printed, each lying on its flat outer face): a foot on the plate, a spring tower over the fork arm, a keel down through a
   plate notch with a printed pivot pin at plate height, level with the axle (80.6 mm arm, design.md 2), and the hard stops at the
   springs: bump rails under the tower onto the fork pad, a droop hook under the fork arm.
 * fork (one printed part): outer arm with a bolt-in axle stub into the wheel's left 6804, inner arm bolted straight to the AK40-10's rear
   face, beam behind the wheel, a pad under each spring pair, a 627 in each arm. Each 627 turns on its frame's pin (an M3 x 16 core).
 * springs: two yellow 8 x 25 mm 3D-printer bed springs per side, side by side, each on an M4 button-head plunger that presses on the
   fork's pad; no damper. The stops hold the axle to -3 / +15 mm.
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
                 'spring_plunger': (1.0, 0.0, 1.0), 'bearing': (0.30, 0.30, 0.32)}


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
Y_S = PIVOT_Y - SPRING_ARM                               # centre of each side's spring pair
X_T = ((OUTER_ARM_X[0] + OUTER_ARM_X[1]) / 2, (INNER_ARM_X[0] + INNER_ARM_X[1]) / 2)     # spring axes, outer / inner
Y_PAIR = tuple(Y_S + (i - (SPRINGS_PER_SIDE - 1) / 2) * SPRING_PITCH for i in range(SPRINGS_PER_SIDE))   # 'a' toward the axle, 'b' the pivot
PLUNGERS = {f'{side}_{"ab"[i]}': (xt, y) for side, xt in zip(('outer', 'inner'), X_T) for i, y in enumerate(Y_PAIR)}
DOME_R = ((BOLT_HEAD_D / 2) ** 2 + BOLT_HEAD_K ** 2) / (2 * BOLT_HEAD_K)


def spring_force(mass):
    """per spring at nominal: the wheel's share of the robot's weight through the lever, shared by all the pod's springs"""
    return mass * 9.81 / 4 * PIVOT_Y / SPRING_ARM / (2 * SPRINGS_PER_SIDE)


K_DESIGN, MAXW_DESIGN = SPRINGS[SPRING_DESIGN]
C_NOM = spring_force(DESIGN_MASS) / K_DESIGN
Z_SB = PAD_Z + BOLT_HEAD_K + WASHER_T                   # spring bottom seat (on the washer), nominal
Z_CEIL = Z_SB + SPRING_L0 - C_NOM                       # spring top seat in the tower
Z_TT = Z_CEIL + GUIDE_LEN                               # tower top
Z_TBOT = PAD_Z + TOWER_CLEAR                            # tower underside
BOLT_L = math.ceil((Z_TT + NUT_GAP + NUT_T + 1.0 - (PAD_Z + BOLT_HEAD_K)) / 5.0) * 5.0


def plunger_lift(a, y=Y_S):
    """how far the plunger on axis y rises at fork angle a: its dome (a sphere on the vertical axis) stays tangent to the turned pad plane;
    also where it touches, from the point of the pad that was under it at nominal"""
    p0 = rot_pt((y, PAD_Z), a)
    n = rot_pt((y, PAD_Z + 1.0), a) - p0
    zc0 = PAD_Z + DOME_R
    zc = p0[1] + (DOME_R - n[0] * (y - p0[0])) / n[1]               # n . ((y, zc) - p0) = DOME_R
    contact = np.array([y, zc]) - DOME_R * n
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
    beam = along_x(LOBE_R, OUTER_ARM_X[0], INNER_ARM_X[1], *piv)
    f = f + (beam - along_x(WHEEL_CLEAR_R, OUTER_ARM_X[1], INNER_ARM_X[0]))
    for ax in (OUTER_ARM_X, INNER_ARM_X):                              # spring pads
        f = f + box(ax[0], ax[1], Y_S - PAD_HALF, Y_S + PAD_HALF, 5.0, PAD_Z)
    for xa, s in ((OUTER_ARM_X[0], 1), (INNER_ARM_X[1], -1)):           # 627 seats in the arms' outer faces; the relief keeps the seat
        xs = xa + s * (BRG_RECESS + B627_W)                             # bottom off the inner race
        f = f - along_x(B627_OD / 2, *sorted((xa - s * 0.1, xs)), PIVOT_Y, PIVOT_Z)
        f = f - along_x(SEAT_RELIEF_D / 2, *sorted((xs - s * 0.1, xs + s * SEAT_RELIEF_T)), PIVOT_Y, PIVOT_Z)
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


def tower_x(fr):
    x0, x1 = fr['x']
    return (x1, fr['xt'] + TOWER_REACH) if fr['inward'] > 0 else (fr['xt'] - TOWER_REACH, x0)


def rails_x(fr):
    """the two bump rails under a tower: its x span less the spring washers' band"""
    lo, hi = tower_x(fr)
    w = WASHER_OD / 2 + RAIL_CLEAR
    return (lo, fr['xt'] - w), (fr['xt'] + w, hi)


def stops(fork):
    """the stop faces, as straight y-z lines z = z0 + m (y - y0), from the fork itself: the droop hook's top is the arm's underside at droop
    (min z in thin slices at the hook's ends); the bump rails' underside is the pad's top at bump, from the pad's front edge at bump back
    to the tower's back (the pad's top is a plane, so its line at bump lies above it at every other pose there)"""
    out = {}
    ya, yb = Y_S - STOP_HALF, Y_S + STOP_HALF
    pad_front = rot_pt((Y_S - PAD_HALF, PAD_Z), A_BUMP)
    pad_back = rot_pt((Y_S + PAD_HALF, PAD_Z), A_BUMP)
    m_b = (pad_back[1] - pad_front[1]) / (pad_back[0] - pad_front[0])
    for name, fr in FRAMES.items():
        lx = ledge_x(fr)
        down = fork.rotate(PIVOT_AXIS, A_DROOP)
        za, zb = ((down & box(lx[0], lx[1], y - 0.1, y + 0.1, -60, PLATE_TOP_Z)).bounding_box().min.Z for y in (ya, yb))
        out[name] = dict(hook=((ya, float(za)), (yb, float(zb))),
                         rail=((float(pad_front[0]) + 0.5, float(pad_front[1] + m_b * 0.5)), (Y_S + TOWER_HALF, float(pad_front[1] + m_b * (Y_S + TOWER_HALF - pad_front[0])))))
    return out


def yz_poly(pts, x0, x1):
    """closed y-z polygon extruded over x0..x1"""
    return prism([line(V(*p), V(*q)) for p, q in zip(pts, pts[1:] + pts[:1])], x0, x1)


def yz_quad(p0, p1, dz, x0, x1):
    """prism between the y-z segment p0-p1 and the same segment shifted dz (up if dz > 0), over x0..x1"""
    return yz_poly([p0, p1, (p1[0], p1[1] + dz), (p0[0], p0[1] + dz)], x0, x1)


def build_frame(name, st):
    fr = FRAMES[name]
    x0, x1 = fr['x']
    s, xt = fr['inward'], fr['xt']
    xc = (x0 + x1) / 2
    top_foot = PLATE_TOP_Z + FOOT_T
    (ha, hb), (ra, rb) = st['hook'], st['rail']
    z_bot = min(ha[1], hb[1]) - HOOK_T                                                     # keel bottom = the droop hook's bottom
    yc0, yc1 = Y_S - TOWER_HALF, Y_S + TOWER_HALF
    # side plate (y-z profile through the frame's thickness): foot on the plate, spring column, web + gusset, keel through the plate notch
    pts = [(FOOT_Y[0], PLATE_TOP_Z), (KEEL_Y[0], PLATE_TOP_Z), (KEEL_Y[0], z_bot), (KEEL_Y[1], z_bot), (KEEL_Y[1], PLATE_TOP_Z),
           (FOOT_Y[1], PLATE_TOP_Z), (FOOT_Y[1], top_foot), (GUSSET_END_Y, top_foot),
           (yc1, GUSSET_TOP_Z), (yc1, Z_TT), (yc0, Z_TT), (yc0, top_foot), (FOOT_Y[0], top_foot)]
    f = prism([line(V(*p), V(*q)) for p, q in zip(pts, pts[1:] + pts[:1])], x0, x1)
    for r, at in ((FILLET_COL, [(yc0, top_foot)]), (FILLET_GUSSET, [(yc1, GUSSET_TOP_Z), (GUSSET_END_Y, top_foot)])):
        f = f.fillet(r, [e for e in f.edges() if abs(e.length - FRAME_T) < 1e-3 and any(abs(e.center().Y - y) < 1e-3 and abs(e.center().Z - z) < 1e-3 for y, z in at)])
    tx = tower_x(fr)
    f = f + box(tx[0], tx[1], yc0, yc1, Z_TBOT, Z_TT)                                       # tower over the fork arm
    for rx in rails_x(fr):                                                                   # bump rails: land on the pad at bump
        f = f + yz_poly([ra, rb, (rb[0], Z_TBOT + 0.5), (ra[0], Z_TBOT + 0.5)], *rx)
    lx = ledge_x(fr)
    f = f + yz_quad(ha, hb, -HOOK_T, *lx)                                                    # droop hook under the arm
    for y in Y_PAIR:
        f = f - cyl((SPRING_HOLE_D + 0.6) / 2, Z_CEIL - Z_TBOT + 0.1, (xt, y, (Z_CEIL + Z_TBOT) / 2 - 0.05))
        f = f - cyl(GUIDE_R, GUIDE_LEN + 0.2, (xt, y, Z_CEIL + GUIDE_LEN / 2))
    xi, xo = (x1, x0) if s > 0 else (x0, x1)                                                 # keel's inner (fork side) and outer faces
    xa = xi + s * FRAME_GAP                                                                   # fork arm's outer face
    x_tip = xa + s * (BRG_RECESS + B627_W - 0.2)                                              # pin end, 0.2 short of the 627's far face
    f = f + along_x(PIVOT_BOSS_D / 2, *sorted((xi - s * 0.5, xa)), PIVOT_Y, PIVOT_Z)        # boss onto the 627's inner race
    f = f + along_x(B627_BORE / 2 - 0.02, *sorted((xa - s * 0.1, x_tip)), PIVOT_Y, PIVOT_Z) # pin in the inner race
    f = f - along_x(PIVOT_HEAD_D / 2, *sorted((xo - s * 0.1, xo + s * PIVOT_HEAD_RECESS)), PIVOT_Y, PIVOT_Z)   # M3 head counterbore
    f = f - along_x(M3_PILOT / 2, *sorted((xo, x_tip + s * 0.1)), PIVOT_Y, PIVOT_Z)       # M3 pilot through the keel, boss and pin
    plastic = abs(x_tip - xo) - PIVOT_HEAD_RECESS
    for y in MOUNT_Y:
        f = f - cyl(M3_CLR / 2, FOOT_T + 0.2, (xc, y, PLATE_TOP_Z + FOOT_T / 2))
    r2 = lambda p: [round(v, 2) for v in p]                                                  # noqa: E731
    return f, dict(x=[round(x0, 2), round(x1, 2)], z_bottom=round(z_bot, 2), z_top=round(Z_TT, 2),
                   droop_hook_top=[r2(ha), r2(hb)], bump_rail_underside=[r2(ra), r2(rb)],
                   pivot=f'Ø7 x {abs(x_tip - xa):.1f} pin on a Ø10 x {FRAME_GAP:g} boss; M3 x {PIVOT_SCREW_L:g} all threaded, its tip '
                         f'{plastic - PIVOT_SCREW_L:.1f} mm short of the pin end; head {PIVOT_HEAD_RECESS - 2.5:.1f} mm under the outer face')


def bearing_627(name):
    """the 627 (one ring body) in the fork arm next to this frame"""
    fr = FRAMES[name]
    s = fr['inward']
    a = (fr['x'][1] if s > 0 else fr['x'][0]) + s * (FRAME_GAP + BRG_RECESS)
    c = a + s * B627_W
    return along_x(B627_OD / 2, *sorted((a, c)), PIVOT_Y, PIVOT_Z) - along_x(B627_BORE / 2, min(a, c) - 1, max(a, c) + 1, PIVOT_Y, PIVOT_Z)


def plunger(xt, y):
    """M4 x BOLT_L button head + washer + nyloc as one body, at nominal (dome apex on the pad)"""
    cap = b.Sphere(DOME_R).moved(b.Location((0, 0, DOME_R))) & b.Cylinder(BOLT_HEAD_D / 2, BOLT_HEAD_K).moved(b.Location((0, 0, BOLT_HEAD_K / 2)))
    cap = cap - b.extrude(b.Plane.XY.offset(-0.1) * b.RegularPolygon(2.5 / math.sqrt(3), 6), amount=1.4)   # round 13: its 2.5 hex socket
    z_head = BOLT_HEAD_K
    p = cap + cyl(SHANK_R, BOLT_L, (0, 0, z_head + BOLT_L / 2))
    p = p + (cyl(WASHER_OD / 2, WASHER_T, (0, 0, z_head + WASHER_T / 2)) - cyl(GUIDE_R, WASHER_T + 1, (0, 0, z_head + WASHER_T / 2)))
    zn = Z_TT + NUT_GAP - PAD_Z
    nut = b.extrude(b.RegularPolygon(NUT_AF / math.sqrt(3), 6), NUT_T).moved(b.Location((0, 0, zn)))
    p = p + (nut - cyl(SHANK_R, NUT_T + 1, (0, 0, zn + NUT_T / 2)))
    return p.moved(b.Location((xt, y, PAD_Z)))


def die_spring(xt, y, length):
    z0 = Z_SB
    return cyl(SPRING_OD / 2, length, (xt, y, z0 + length / 2)) - cyl(SPRING_ID / 2, length + 1, (xt, y, z0 + length / 2))


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
    parts = dict(fork=fork, stub=build_stub())
    info = {}
    for name in FRAMES:
        parts['b627_' + name] = bearing_627(name)
        parts['frame_' + name], info[name] = build_frame(name, st[name])
    return parts, info


def posed(parts, unit, a):
    moving = dict(unit, pod_fork=parts['fork'], axle_stub=parts['stub'], bearing_627_outer=parts['b627_outer'], bearing_627_inner=parts['b627_inner'])
    out = {k: (v.rotate(PIVOT_AXIS, a) if a else v) for k, v in moving.items()}
    out['pod_frame_outer'], out['pod_frame_inner'] = parts['frame_outer'], parts['frame_inner']
    lifts = {}
    for k, (xt, y) in PLUNGERS.items():
        lifts[k] = lift = plunger_lift(a, y)[0]
        out[f'spring_plunger_{k}'] = plunger(xt, y).moved(b.Location((0, 0, lift)))
        out[f'die_spring_{k}'] = die_spring(xt, y, Z_CEIL - Z_SB - lift).moved(b.Location((0, 0, lift)))
    return out, lifts


# ---- checks ------------------------------------------------------------------------------------------------------------------
def spring_table():
    rows = {}
    lift_b = max(plunger_lift(A_BUMP, y)[0] for y in Y_PAIR)           # the plunger farther from the pivot moves most
    lift_d = min(plunger_lift(A_DROOP, y)[0] for y in Y_PAIR)
    for kg in (10, 12.5, 15, 20):
        F = spring_force(kg)
        r = dict(force_nominal_N=round(F, 1))
        for colour_, (k, maxw) in SPRINGS.items():
            c = F / k
            r[colour_] = dict(wheel_rate_N_per_mm=round(2 * sum(k * ((PIVOT_Y - y) / PIVOT_Y) ** 2 for y in Y_PAIR), 2), compression_nominal=round(c, 1),
                              compression_bump=round(c + lift_b, 1), max_working=maxw, ok=bool(c + lift_b <= maxw and c + lift_d > 0.5),
                              shim_mm=round(c - C_NOM, 1))
        rows[f'{kg} kg'] = r
    return rows


def check(parts, unit):
    rep = dict(poses_deg={k: round(v, 3) for k, v in POSES.items()},
               springs=dict(part=f'{2 * SPRINGS_PER_SIDE} x yellow 8 x 25 mm 3D-printer bed spring (JIS extra-light Ø8 / Ø4 x 25; rate {K_DESIGN} N/mm ASSUMED, VERIFY)',
                            design=f'{SPRING_DESIGN} at {DESIGN_MASS} kg', arm_mm=SPRING_ARM, pair_y=[round(y, 2) for y in Y_PAIR],
                            plunger=f'M4 x {BOLT_L:g} button head (ISO 7380), M4 washer, M4 nyloc', table=spring_table(),
                            note='shim_mm: spacer under the spring seat to keep the nominal ride height with that spring and mass (negative: the pod rides higher)'))
    static = ('pod_frame_outer', 'pod_frame_inner')
    rotating = [k for k in unit if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_6804'))]
    harness = ('xt30_housing', 'lead_pos', 'lead_neg')
    for name, a in POSES.items():
        sh, lifts = posed(parts, unit, a)
        r = dict(axle=[round(float(v), 2) for v in rot_pt((0, 0), a)], plunger_lift={k: round(v, 2) for k, v in lifts.items()},
                 contact_from_pad_point={k: round(plunger_lift(a, y)[1], 2) for k, (_, y) in PLUNGERS.items()},
                 spring_length={k: round(Z_CEIL - Z_SB - v, 2) for k, v in lifts.items()},
                 nut_gap_to_tower={k: round(NUT_GAP + v, 2) for k, v in lifts.items()},
                 pad_margin=round(PAD_HALF - max(abs(y - Y_S + plunger_lift(a, y)[1]) for _, y in PLUNGERS.values()), 2))
        ov = {}
        moving = [k for k in sh if k not in static and not k.startswith(('spring_plunger', 'die_spring'))]
        for st in static:
            for k in moving:
                if near(sh[st], sh[k]):
                    v = overlap(sh[st], sh[k])
                    if v > 0.01:
                        ov[f'{st} / {k}'] = round(v, 2)
        for pk in PLUNGERS:
            for k in sh:
                if k.endswith(pk) or not near(sh[f'spring_plunger_{pk}'], sh[k]):
                    continue
                v = overlap(sh[f'spring_plunger_{pk}'], sh[k])
                if v > 0.05:
                    ov[f'spring_plunger_{pk} / {k}'] = round(v, 2)
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
            if k.startswith(('pod_frame', 'spring_', 'die_')) or not near(s, slab, 0):
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
    rep['printed_parts'] = {k: dict(solids=len(parts[p].solids()), valid=bool(parts[p].is_valid), volume_cm3=round(parts[p].volume / 1000, 2),
                                    bbox=[round(v, 1) for v in tuple(parts[p].bounding_box().size)])
                            for k, p in PRINTED}
    return rep


# ---- export / render -------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_', 'bearing_', 'spring_plunger')
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
    iso({k: sh[k] for k in sh if k.startswith(('pod_fork', 'axle_stub', 'pod_frame', 'bearing_627', 'spring_plunger', 'die_spring'))},
        VAL / 'iso_frames_fork.png', 215, 30, 'side frames, fork, pivot 627s, spring plungers (wheel hidden)')
    for xs, nm in ((X_T[0], 'outer_spring_plane'), (FRAMES['outer']['x'][1] + 2.5, 'outer_stops'), (0.0, 'mid_plane'),
                   (FRAMES['outer']['x'][0] + 0.5, 'outer_frame')):
        fig, axs = plt.subplots(1, 3, figsize=(18, 6))
        for ax, (pname, a) in zip(axs, POSES.items()):
            sh, lifts = posed(parts, unit, a)
            la, lb = lifts['outer_a'], lifts['outer_b']
            section(sh, xs, ax, f'{pname}: x = {xs:g}, plunger lifts {la:+.1f} / {lb:+.1f} (y toward the pivot, z up; dashed: plate top)')
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

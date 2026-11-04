"""cad/v11 compact wheel + motor, round 8 (Nov 3 2026), build123d, STEP review only. Dimensions in dims.py. The suspension pod is pod.py.

    python -B wheel_motor.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Round 8, Fernando's sketch: each axle head pocket is a round shank hole, a round head counterbore and a 45 deg cone opening outward along the
axle (pocket_tool), instead of the double-teardrop tunnel that left cantilevered slivers of web; the cone stops at 5 mm because the two axles
in a spoke cross just past their heads and a longer cone cuts into the neighbour's head seat.

Round 7, after Fernando's review of the round-6 hub: the Ø3.4 shank hole between each thrust face and head seat is a teardrop like the tunnel
(its round roof was the last overhang at the pockets, including where it breaks into the neighbouring roller's driver tunnel); R0.8 on the
spoke webs' x-end edges and the drum's left end; the TPU tread snaps into a V groove round the core's middle (a 0.4 mm rib in its bore), so it
cannot walk along the core but still pulls off by hand.

Round 6, after Fernando's first prints: Ø8 double-teardrop head pockets / driver tunnels (no support inside, bigger driver); spoke webs 26 mm deep
and the roller cut is the roller's revolved envelope + drop-in channel, so the webs wrap the roller ends at the thin neck; straight roller core
with open-ended keys so the TPU tread slides off.

Round 5: no motor mount: the pod's fork bolts straight to the motor's rear face. Motor turned MOTOR_ROT about the axis so the XT30 points back and
slightly up (az ~160), away from the pod's pivot and shocks; the hub's flange holes follow. Second 45 deg cone in the hub cavity (left wall), so the
cavity needs no internal support printed either end down. STEP colours on every face (step_colour.py): SolidWorks ignored build123d's colours.

Round 4 (option 1, chosen by Fernando): the motor sits MOTOR_SHIFT_X outboard so its radial XT30 plug and leads are outside the hub, >= 4 mm from
anything rotating. The hub ends at x 29 (drum only) and rides on the motor output bearing + the left 6804; the right 6804 is gone.

Round 3, after Fernando's review of round 2:
 * real AK40-10 with its mated XT30(2+2) plug (components/ak40-10/AK40-10 with connector.STEP); the two power leads leave the pin ends radially
   and bend 90 deg toward +x (lead OD assumed, VERIFY). The plug mates RADIALLY: see the harness check.
 * the flat thrust-face cut now runs across the whole roller end (the hub touched the roller end face beyond the journal), blended into the
   tread clearance with R3 at the thin spoke neck. Band and webs are built from real lines and arcs.
 * roller bore 3.6 -> 3.4.
 * one script + dims.py; the STEP assembly is coloured by part type; printed parts only in parts/ (no fasteners or bearings in any STEP).
Everything is nominal CAD; nothing has been printed.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from OCP.gp import gp_Trsf

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dims import *                                                     # noqa: E402,F401,F403
from step_colour import write_part, write_step                                     # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
REF = ROOT / 'reference'
MOTOR_STEP = ROOT.parents[1] / 'components' / 'ak40-10' / 'AK40-10 with connector.STEP'
OUT = ROOT / 'step_export' / 'wheel_motor'
VAL = ROOT / 'validation' / 'wheel_motor'

COLOURS = {                         # STEP colours (SolidWorks shows them per component); first matching prefix wins
    'wheel_hub': (0.93, 0.52, 0.20), 'wheel_cartridge': (0.66, 0.46, 0.86), 'pod_chassis': (0.25, 0.68, 0.74), 'pod_fork': (0.95, 0.75, 0.15), 'axle_stub': (0.66, 0.46, 0.86), 'shock': (0.85, 0.15, 0.55),
    'roller_core': (0.38, 0.66, 0.38), 'roller_tread': (0.20, 0.24, 0.32), 'ak40_stator': (0.42, 0.43, 0.46), 'ak40_rotor': (0.78, 0.78, 0.80),
    'xt30_housing': (0.96, 0.82, 0.10), 'xt30_pin': (0.85, 0.62, 0.20), 'lead_pos': (0.85, 0.10, 0.10), 'lead_neg': (0.08, 0.08, 0.08),
    'bolt': (1.0, 0.0, 1.0), 'bearing': (0.30, 0.30, 0.30), 'motor_proxy': (0.62, 0.63, 0.66),
}


def colour(name):
    name = name.lstrip('_')
    for k, v in COLOURS.items():
        if name.startswith(k):
            return v
    return (0.7, 0.7, 0.7)


# ---- helpers --------------------------------------------------------------------------------------------------------------
def loc(M):
    t = gp_Trsf()
    t.SetValues(*[float(v) for r in np.asarray(M)[:3] for v in r])
    return b.Location(t)


def cyl(r, length, centre):
    """cylinder along local z, centred at `centre`"""
    return b.Cylinder(r, length).moved(b.Location(centre))


def along_x(r, x0, x1, y=0.0, z=0.0):
    return b.Cylinder(r, x1 - x0).rotate(b.Axis.Y, 90).moved(b.Location(((x0 + x1) / 2, y, z)))


def at_angle(r, deg):
    a = math.radians(deg)
    return r * math.cos(a), r * math.sin(a)             # (y, z)


def V(y, z):
    """point in a roller's local y-z plane"""
    return b.Vector(0, y, z)


def arc(p0, pm, p1):
    return b.Edge.make_three_point_arc(p0, pm, p1)


def line(p0, p1):
    return b.Edge.make_line(p0, p1)


def prism_x(edges, half):
    """closed y-z profile, extruded +-half along x"""
    return b.extrude(b.Face(b.Wire(edges)), amount=half, both=True)


def overlap(a, c):
    x = a & c
    return 0.0 if x is None else float(x.volume)


def near(a, c, gap=0.5):
    p, q = a.bounding_box(), c.bounding_box()
    return all(tuple(p.min)[i] - gap <= tuple(q.max)[i] and tuple(q.min)[i] - gap <= tuple(p.max)[i] for i in range(3))


def min_dist(a, c):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    d = BRepExtrema_DistShapeShape(a.wrapped, c.wrapped)
    d.Perform()
    return d.Value()


def points(shape, tol=0.05):
    v, _ = shape.tessellate(tol, 0.1)
    return np.array([tuple(q) for q in v])


# ---- rollers --------------------------------------------------------------------------------------------------------------
def end_circles(s, r):
    return [e for e in s.edges() if e.geom_type.name == 'CIRCLE' and abs(abs(e.center().Z) - HT) < 1e-3 and abs(e.radius - r) < 1e-3]


def core_solid():
    """straight PLA core (round 6): the tread slides on and off along the axis; three keyways open at both ends"""
    H, br = HT, BORE_D / 2
    core = cyl(CORE_R, 2 * H, (0, 0, 0))
    core = b.chamfer(end_circles(core, CORE_R), CORE_CHAMFER) - cyl(br, 2 * H + 1, (0, 0, 0))
    for sgn in (-1, 1):
        zc = sgn * (H + JOURNAL / 2)
        core = core + (cyl(JOURNAL_R, JOURNAL, (0, 0, zc)) - cyl(br, JOURNAL + 1, (0, 0, zc)))
    for a in (0, 120, 240):
        core = core - b.Box(KEY_DEPTH + 1, KEY_W, 2 * H + 2).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 1) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return core - detent_ring(CORE_R + 0.5, CORE_R - DETENT_DEPTH)


def detent_ring(r_base, r_tip):
    """45 deg V ring round the roller axis at its middle, from radius r_base (the base, 2 x its depth wide) to r_tip"""
    h = abs(r_base - r_tip)
    edges = [line(b.Vector(r_base, -h, 0), b.Vector(r_tip, 0, 0)), line(b.Vector(r_tip, 0, 0), b.Vector(r_base, h, 0)),
             line(b.Vector(r_base, h, 0), b.Vector(r_base, -h, 0))]
    return b.revolve(b.Plane.XZ * b.Face(b.Wire(edges)), axis=b.Axis.Z)


def tread_solid():
    """TPU tread: outside the R_WHEEL profile, straight bore on the core, keys along its whole length"""
    H = HT
    edges = [arc(b.Vector(rt(H), -H, 0), b.Vector(rt(0), 0, 0), b.Vector(rt(H), H, 0)), line(b.Vector(rt(H), H, 0), b.Vector(CORE_R, H, 0)),
             line(b.Vector(CORE_R, H, 0), b.Vector(CORE_R, -H, 0)), line(b.Vector(CORE_R, -H, 0), b.Vector(rt(H), -H, 0))]
    t = b.revolve(b.Plane.XZ * b.Face(b.Wire(edges)), axis=b.Axis.Z)
    t = b.fillet(end_circles(t, rt(H)), FILLET_TREAD)
    t = b.chamfer(end_circles(t, CORE_R), CORE_CHAMFER)
    for a in (0, 120, 240):
        t = t + b.Box(KEY_DEPTH + 0.5, KEY_W, 2 * H).moved(b.Location((CORE_R - KEY_DEPTH + (KEY_DEPTH + 0.5) / 2, 0, 0))).rotate(b.Axis.Z, a)
    return t + detent_ring(CORE_R + 0.01, CORE_R - DETENT_RIB)          # snaps into the core's groove


def roller_frames():
    """[(row, k, 4x4 local -> pod)]; even rollers are turned 180 deg about local y so the two axles that share a spoke both enter from it"""
    out = []
    for row in (1, 2):
        x = -D_ROW if row == 1 else D_ROW
        for k in range(1, N_PER_ROW + 1):
            th = math.radians(60 * (k - 1) + (0 if row == 1 else 30))
            c, s = math.cos(th), math.sin(th)
            M = np.eye(4)
            M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = [1, 0, 0], [0, c, s], [0, -s, c], [x, R_AXIS * c, R_AXIS * s]
            if k % 2 == 0:
                M = M @ np.diag([-1.0, 1.0, -1.0, 1.0])
            out.append((row, k, M))
    return out


# ---- hub ---------------------------------------------------------------------------------------------------------------------
def band_tool():
    """the cut each roller makes in the spokes. Its profile in the local y-z plane: the thrust faces z = +-ZF run across the whole roller end and
    blend (BLEND_R) into the tread clearance arc (R_WHEEL + BAND_C about the wheel axis, mirrored: centre at local y = +R_AXIS). Round 6: that
    profile is revolved about the roller axis (the roller's clearance envelope), plus an open channel above the axis (radial drop-in) as wide as
    the envelope, so the spoke outside the roller end is kept and wraps it; before, the profile was prismed through x."""
    big = R_WHEEL + BAND_C
    zc = ZF - BLEND_R
    yc = R_AXIS - math.sqrt((big - BLEND_R) ** 2 - zc ** 2)
    out = np.array([yc - R_AXIS, zc]) / math.hypot(yc - R_AXIS, zc)
    pts = {}
    for s in (1, -1):
        c = np.array([yc, s * zc])
        o = np.array([out[0], s * out[1]])
        tb = c + BLEND_R * o
        mid = c + BLEND_R * (o + np.array([0, s])) / np.linalg.norm(o + np.array([0, s]))
        pts[s] = (V(yc, s * ZF), V(*mid), V(*tb))
    edges = [line(V(0, -ZF), V(0, ZF)), line(V(0, ZF), pts[1][0]), arc(*pts[1]),
             arc(pts[1][2], V(R_AXIS - big, 0), pts[-1][2]), arc(pts[-1][2], pts[-1][1], pts[-1][0]), line(pts[-1][0], V(0, -ZF))]
    env = b.revolve(b.Face(b.Wire(edges)), axis=b.Axis.Z)
    half = big - R_AXIS
    top = 200.0
    return env + b.Box(2 * half, top, 2 * ZF).moved(b.Location((0, top / 2, 0)))


def tunnel_tool(z0, z1, d=TUNNEL_D):
    """axle head pocket + driver tunnel (or the shank hole, d = CLR_D) along local z from z0 to z1: Ø d with 45 deg points toward local +-x
    (pod x, the print axis)"""
    r = d / 2
    q = r / math.sqrt(2)
    sec = b.Circle(r) + b.Polygon((r * math.sqrt(2), 0), (q, q), (-q, q), (-r * math.sqrt(2), 0), (-q, -q), (q, -q), align=None)
    return b.extrude(b.Plane.XY.offset(z0) * sec, amount=z1 - z0)


def pocket_tool():
    """round 8 (Fernando's sketch): round Ø3.4 shank hole from the thrust face to the seat, round head counterbore, then a 45 deg cone
    opening outward along the axle, so every face beyond the head slopes at 45 deg or less whichever way the hub prints (the round-6
    double-teardrop tunnel left cantilevered slivers of web either side of it); then the driver / head path on along the axle"""
    r = HEAD_POCKET_D / 2
    p = cyl(CLR_D / 2, SEAT - ZF + 1.0, (0, 0, (ZF - 1.0 + SEAT) / 2)) + cyl(r, B_OUT - SEAT + 0.01, (0, 0, (SEAT + B_OUT) / 2))
    p = p + b.Cone(r, r + FLARE_L, FLARE_L).moved(b.Location((0, 0, B_OUT + FLARE_L / 2)))
    return p + cyl(r, 60.0, (0, 0, B_OUT + FLARE_L + 30.0 - 0.01))


def web(zc):
    """spoke web under one block: convex hull of the block circle (local y 0, z zc) and two feet 1 mm inside the drum, prismed through x
    (build_hub clips it to the row's x range)"""
    P = np.array([0.0, zc])
    feet = []
    for dz in (-WEB_REACH, WEB_REACH):
        z = max(-DRUM_OUT + 1, min(DRUM_OUT - 1, zc + dz))
        feet.append(np.array([-R_AXIS + math.sqrt(DRUM_OUT ** 2 - z * z) - 1.0, z]))
    def tangent(E, side):
        d = E - P
        phi, a = math.atan2(d[1], d[0]), math.acos(BLOCK_R / np.linalg.norm(d))
        return P + BLOCK_R * np.array([math.cos(phi + side * a), math.sin(phi + side * a)])
    t_lo, t_hi = tangent(feet[0], -1), tangent(feet[1], 1)
    if t_lo[1] > t_hi[1]:
        t_lo, t_hi = tangent(feet[0], 1), tangent(feet[1], -1)
    edges = [line(V(*feet[0]), V(*feet[1])), line(V(*feet[1]), V(*t_hi)), arc(V(*t_hi), V(BLOCK_R, zc), V(*t_lo)), line(V(*t_lo), V(*feet[0]))]
    return prism_x(edges, D_ROW + WEB_OUT)


def web_x(row):
    """pod x range of a row's spokes"""
    return (-(D_ROW + WEB_OUT), -WEB_GAP / 2) if row == 1 else (WEB_GAP / 2, min(D_ROW + WEB_OUT, X_RIGHT))


def build_hub():
    # drum, left end wall (cartridge counterbore + access bore) running out on a 45 deg cone, flange plate joined to the drum by a 45 deg cone,
    # open right end around the motor can: the cavity between the cones needs no support printed either end down
    prof = [(X_LEFT, DRUM_IN + 0.2), (X_LEFT, DRUM_OUT), (X_RIGHT, DRUM_OUT), (X_RIGHT, DRUM_IN), (FLANGE_BACK, DRUM_IN), (FLANGE_BACK, 0.0),
            (FLANGE_FRONT, 0.0), (FLANGE_FRONT, ACCESS_R), (FLANGE_FRONT - (DRUM_IN - ACCESS_R), DRUM_IN),
            (WALL_BACK + (DRUM_IN - ACCESS_R), DRUM_IN), (WALL_BACK, ACCESS_R), (X_PLATE_FLOOR, ACCESS_R), (X_PLATE_FLOOR, DRUM_IN + 0.2)]
    hub = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    zb, za = (ZF + B_OUT) / 2, -(ZF + A_OUT) / 2
    unit = cyl(BLOCK_R, B_OUT - ZF, (0, 0, zb)) + cyl(BLOCK_R, A_OUT - ZF, (0, 0, za)) + web(zb) + web(za)
    keepout = b.Cylinder(DRUM_OUT - 0.5, 120.0).rotate(b.Axis.Y, 90)   # nothing of the spokes inside r 32.5 (the drum wall is r 30..33)
    frames = roller_frames()
    info = dict(edge_round_failed=0)
    for row, k, M in frames:
        x0, x1 = web_x(row)
        spoke = (unit.moved(loc(M)) - keepout) & b.Box(x1 - x0, 200, 200).moved(b.Location(((x0 + x1) / 2, 0, 0)))
        # round 7: round the spoke's x-end edges before it joins the drum (OCC will not fillet them on the finished hub); not the edges buried in the drum
        ends = [e for e in spoke.edges() if any(all(abs(e.position_at(t).X - p) < 1e-3 for t in (0, 0.5, 1)) for p in (x0, x1))
                and math.hypot(e.position_at(0.5).Y, e.position_at(0.5).Z) > DRUM_OUT]
        try:
            spoke = b.fillet(ends, HUB_EDGE_R)
        except Exception:                   # noqa: BLE001
            info['edge_round_failed'] += 1
        hub = hub + spoke
    hub, info['root_fillet'] = fillet_roots(hub)
    band = band_tool()
    for row, k, M in frames:
        hub = hub - band.moved(loc(M))
    solids = sorted(hub.solids(), key=lambda s: -s.volume)
    hub = solids[0]
    info['slivers_dropped_mm3'] = [round(s.volume, 2) for s in solids[1:]]
    try:                                                                    # drum's left end (outside and inside edges)
        hub = b.fillet([e for e in hub.edges() if e.geom_type.name == 'CIRCLE' and abs(e.center().X - X_LEFT) < 1e-3 and e.radius > DRUM_IN], HUB_EDGE_R)
    except Exception:                       # noqa: BLE001
        info['edge_round_failed'] += 1
    for row, k, M in frames:
        pilot = cyl(PILOT_D / 2, PILOT_DEPTH, (0, 0, -(ZF + PILOT_DEPTH / 2)))
        hub = hub - (pocket_tool() + pilot).moved(loc(M))
    for a in FLANGE_ANGLES:
        y, z = at_angle(FLANGE_R, a)
        hub = hub - along_x(M25_CLEAR / 2, FLANGE_FRONT - 0.1, FLANGE_BACK + 0.1, y, z)
    for a in CART_ANGLES:
        y, z = at_angle(CART_R, a)
        hub = hub - along_x(1.4, X_PLATE_FLOOR - 0.1, X_PLATE_FLOOR + 10.0, y, z)
    return hub, info


def fillet_roots(hub, radii=(3.0, 2.5, 2.0, 1.5)):
    """fillet where the spoke webs' x faces meet the drum's outside cylinder"""
    planes = [x for row in (1, 2) for x in web_x(row) if abs(x - X_RIGHT) > 0.05]       # a web face flush with the drum end has no root edge
    arcs = [e for e in hub.edges() if e.geom_type.name == 'CIRCLE' and abs(e.radius - DRUM_OUT) < 0.02
            and any(abs(e.position_at(0.5).X - p) < 0.05 for p in planes)]
    err = None
    for r in radii:
        try:
            out = b.fillet(arcs, r)
            if out.is_valid and len(out.solids()) == 1:
                return out, dict(edges=len(arcs), radius=r)
        except Exception as ex:             # noqa: BLE001
            err = str(ex)[:80]
    return hub, dict(edges=len(arcs), radius=None, error=err)


# ---- cartridge (flush plate + boss carrying the left 6804) -------------------------------------------------------------------
def build_cartridge():
    """flush plate whose lip holds the left 6804's outer race, and a boss the bearing presses into from the open (+x) side: no floor, so no
    trapped cavity, and it prints plate-down without support. The 6804 floats axially (the motor's output bearing locates the wheel)."""
    x0, x1 = X_LEFT, X_PLATE_FLOOR
    brg_back = x1 + 7.0
    # plate face, plate outer, plate back, boss outer, boss end (open), bearing seat, lip over the outer race (r 13..16)
    prof = [(x0, 13.0), (x0, DRUM_IN - 0.3), (x1, DRUM_IN - 0.3), (x1, 17.6), (brg_back, 17.6), (brg_back, R_BRG + BRG_FIT), (x1, R_BRG + BRG_FIT),
            (x1, 13.0)]
    c = b.revolve(b.Plane.XY * b.Polygon(*prof, align=None), axis=b.Axis.X)
    for a in CART_ANGLES:
        y, z = at_angle(CART_R, a)
        c = c - along_x(1.7, x0 - 0.1, x1 + 0.1, y, z) - b.Cone(2.75, 1.7, 1.7).rotate(b.Axis.Y, 90).moved(b.Location((x0 + 0.85, y, z)))
    return c, (x1 + brg_back) / 2


# ---- motor, plug, leads ------------------------------------------------------------------------------------------------------
def motor_and_plug():
    """the real AK40-10 (surface model) and its XT30(2+2) plug: housing + the two power pins, turned MOTOR_ROT about the axis"""
    asm = b.import_step(MOTOR_STEP)
    T = b.Location(MOTOR_STEP_SHIFT)
    def place(s):
        return s.moved(T).rotate(b.Axis.X, MOTOR_ROT)
    motor = place(asm.children[0])
    solids = sorted(asm.children[1].solids(), key=lambda s: -s.volume)
    return motor, place(solids[0]), [place(s) for s in solids[1:]]


def build_motor(housing, pins):
    """own solid AK40-10: the maker's STEP is surfaces only (B-spline faces, no solid), which neither SolidWorks mates nor volume checks can use.
    Dimensions measured off that STEP: can Ø53 from the output side + 1 to the rear face, rear face 4 x Ø2.8 x 4.3 at r 23.5 (mounting) and
    4 x Ø4 recessed case screws, front 6 x Ø2.6 at r 23.75; rotor = output disk Ø37 x 1 with 3 x Ø2.05 (M2.5) at r 13.5 and 3 x Ø2 dowel holes at
    r 12.2, a spigot into the can. The XT30 socket is a pocket of the plug's own shape. Stator and rotor are separate so the wheel can turn."""
    xo, xr = MOTOR_X
    st = along_x(MOTOR_R, xo + 1.0, xr) - along_x(10.0, xo + 0.9, xo + 2.0)
    for a in REAR_HOLES_AZ:
        st = st - along_x(1.4, xr - REAR_HOLE_DEPTH, xr + 0.1, *at_angle(REAR_HOLES_R, a))
    for a in (-132, -42, 72, 162):
        st = st - along_x(2.0, xr - 0.5, xr + 0.1, *at_angle(23.5, a + MOTOR_ROT)) - along_x(1.1, xr - 1.4, xr, *at_angle(23.5, a + MOTOR_ROT))
    for a in range(-180, 180, 60):
        st = st - along_x(1.3, xo + 0.9, xo + 4.0, *at_angle(23.75, a + MOTOR_ROT))
    st = st - housing
    for p in pins:
        st = st - p
    ro = along_x(18.5, xo, xo + 1.0) + along_x(9.7, xo + 0.95, xo + 1.9)
    for a in FLANGE_ANGLES:
        ro = ro - along_x(1.025, xo - 0.1, xo + 2.0, *at_angle(FLANGE_R, a))
    for a in (90, -30, -150):
        ro = ro - along_x(1.0, xo - 0.1, xo + 1.6, *at_angle(12.2, a + MOTOR_ROT))
    return st, ro


def motor_proxy():
    """solid stand-in for the shell motor in volume checks and sections"""
    return along_x(MOTOR_R, MOTOR_X[0] + 1.0, MOTOR_X[1]) + along_x(18.5, MOTOR_X[0], MOTOR_X[0] + 1.1)


def build_leads(pins):
    """one lead per power pin: from inside the pin's outer end straight on along the pin, sharp 90 deg bend toward +x, short run"""
    out = []
    xh = np.array([1.0, 0, 0])
    for p in pins:
        P = points(p, 0.02)
        u, s, vt = np.linalg.svd(P - P.mean(0))
        d = vt[0] if vt[0] @ P.mean(0) > 0 else -vt[0]            # pin axis, pointing outward
        d[0] = 0.0
        d /= np.linalg.norm(d)
        proj = P @ d
        end = P[proj > proj.max() - 0.02].mean(0)
        p0, p1 = end - d * 1.0, end + d * LEAD_STRAIGHT
        cb = p1 + xh * LEAD_BEND_R
        p2 = cb + d * LEAD_BEND_R
        pm = cb + LEAD_BEND_R * (d - xh) / math.sqrt(2)
        path = b.Wire([line(b.Vector(*p0), b.Vector(*p1)), arc(b.Vector(*p1), b.Vector(*pm), b.Vector(*p2)),
                       line(b.Vector(*p2), b.Vector(LEAD_RUN_TO, p2[1], p2[2]))])
        prof = b.Plane(origin=path.position_at(0), z_dir=path.tangent_at(0)) * b.Circle(LEAD_D / 2)
        out.append(b.sweep(prof, path))
    out.sort(key=lambda s: s.center().Y)
    return out


# ---- assembly --------------------------------------------------------------------------------------------------------------------
def build_all():
    shapes = {}
    hub, info = build_hub()
    shapes['wheel_hub'] = hub
    cart, x_brg_left = build_cartridge()
    shapes['wheel_cartridge'] = cart
    _, housing, pins = motor_and_plug()
    shapes['ak40_stator'], shapes['ak40_rotor'] = build_motor(housing, pins)
    shapes['xt30_housing'] = housing
    for i, p in enumerate(pins, 1):
        shapes[f'xt30_pin_{i}'] = p
    shapes['lead_pos'], shapes['lead_neg'] = build_leads(pins)
    core, tread = core_solid(), tread_solid()
    shapes['_roller_core'], shapes['_roller_tread'] = core, tread           # unplaced, for the print exports
    bolt = b.import_step(REF / 'bolt_M3_50_PAN.step')
    for row, k, M in roller_frames():
        shapes[f'roller_core_{row}_{k}'] = core.moved(loc(M))
        shapes[f'roller_tread_{row}_{k}'] = tread.moved(loc(M))
        shapes[f'bolt_{row}_{k}'] = bolt.moved(loc(M @ np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, SEAT], [0, 0, 0, 1.0]])))
    brg = b.import_step(REF / 'bearing_6804.step')
    shapes['bearing_6804_left'] = brg.moved(loc(np.array([[0, 0, 1, x_brg_left], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]])))
    flat = b.import_step(REF / 'bolt_M3_12_FLAT.step')
    for i, a in enumerate(CART_ANGLES, 1):
        y, z = at_angle(CART_R, a)
        shapes[f'bolt_cartridge_{i}'] = flat.moved(loc(np.array([[0, 0, -1, X_LEFT], [0, 1, 0, y], [1, 0, 0, z], [0, 0, 0, 1.0]])))
    for k, s in shapes.items():
        s.label = k.lstrip('_')
        s.color = b.Color(*colour(k.lstrip('_')))
    return shapes, info


# ---- checks --------------------------------------------------------------------------------------------------------------------
def neck_widths(hub):
    """row-2 spoke at azimuth 0: material width along the tangent through the spoke centre, and x thickness, at several radii"""
    from OCP.BRepClass3d import BRepClass3d_SolidClassifier
    from OCP.gp import gp_Pnt
    from OCP.TopAbs import TopAbs_IN
    def inside(p):
        return BRepClass3d_SolidClassifier(hub.wrapped, gp_Pnt(*p), 1e-6).State() == TopAbs_IN
    def run(p, axis, step=0.1, lim=30):
        n = 0.0
        for sgn in (1, -1):
            d = 0.0
            while d < lim and inside(tuple(p[i] + (sgn * d if i == axis else 0) for i in range(3))):
                d += step
            n += d
        return n - step
    out = {}
    for rho in (34, 36, 38, 40, 42, 44, 46, 50, 54, 58):
        p = (D_ROW, rho, 0.0)
        out[rho] = dict(tangential=round(run(p, 2), 2), x=round(run(p, 0), 2)) if inside(p) else None
    return out


def check(shapes):
    rep = {}
    hub = shapes['wheel_hub']
    bb = hub.bounding_box()
    rep['hub'] = dict(solids=len(hub.solids()), valid=bool(hub.is_valid), volume_mm3=round(hub.volume), x_range=[round(bb.min.X, 2), round(bb.max.X, 2)],
                      diameter_over_rollers=round(2 * max(np.hypot(points(shapes['roller_tread_1_1'], 0.05)[:, 1], points(shapes['roller_tread_1_1'], 0.05)[:, 2])), 2))
    prox = motor_proxy()
    solid_names = [k for k in shapes if not k.startswith(('_', 'wheel_hub'))]
    ov = {}
    for k in solid_names:
        if near(hub, shapes[k]):
            v = overlap(hub, shapes[k])
            if v > 0.01:
                ov[k] = round(v, 2)
    rep['hub_overlaps_mm3 (axle threads in their pilots by design)'] = ov
    rep['hub_to_roller_min_gap_mm'] = {f'{part}_{rk}': round(min_dist(hub, shapes[f'{part}_{rk}']), 3)
                                       for part in ('roller_core', 'roller_tread') for rk in ('1_1', '1_2', '2_1', '2_2')}
    keys = [k for k in shapes if k.startswith(('roller_core_', 'roller_tread_'))]
    gaps = {}
    for i, k in enumerate(keys):
        for kk in keys[i + 1:]:
            if k.split('_')[-2:] != kk.split('_')[-2:] and near(shapes[k], shapes[kk], 1.6):
                d = min_dist(shapes[k], shapes[kk])
                if d < 2.0:
                    gaps[f'{k} / {kk}'] = round(d, 2)
    rep['roller_neighbour_gaps_mm_lt2 (closest 6)'] = dict(sorted(gaps.items(), key=lambda kv: kv[1])[:6])
    rep['rollers_vs_motor_proxy_overlap_mm3'] = round(sum(overlap(prox, shapes[k]) for k in keys if near(prox, shapes[k], 0)), 2)
    # harness: plug housing, pins and leads against everything that is not the motor
    harness = {}
    hz = ['xt30_housing', 'xt30_pin_1', 'xt30_pin_2', 'lead_pos', 'lead_neg']
    rotating = [k for k in shapes if k.startswith(('wheel_', 'roller_', 'bolt_', 'bearing_'))]
    worst = (1e9, '')
    for h in hz:
        for k in rotating:
            if near(shapes[h], shapes[k], HARNESS_CLEAR + 1):
                d = min_dist(shapes[h], shapes[k])
                worst = min(worst, (d, f'{h} - {k}'))
    harness[f'closest rotating part (want >= {HARNESS_CLEAR:g})'] = dict(gap=round(worst[0], 2) if worst[1] else 'none within 5 mm', pair=worst[1])
    rep['harness'] = harness
    xs = [s.bounding_box() for k, s in shapes.items() if not k.startswith(('_', 'lead_'))]
    rep['x_range_hub_to_motor'] = [round(min(q.min.X for q in xs), 2), round(max(q.max.X for q in xs), 2)]
    rep['motor_x_output_rear'] = list(MOTOR_X)
    tun = {}
    for row, k, M in roller_frames():
        tool = cyl(AXLE_HEAD_D / 2 - 0.05, 70.0, (0, 0, SEAT + 35.0)).moved(loc(M))
        hits = {}
        for kk in shapes:
            if kk.startswith(('roller_', 'bolt_', 'wheel_cartridge')) and not kk.endswith(f'_{row}_{k}') and near(tool, shapes[kk], 0):
                v = overlap(tool, shapes[kk])
                if v > 0.05:
                    hits[kk] = round(v, 2)
        tun[f'{row}_{k}'] = hits
    rep['axle_insertion_tunnel_hits_mm3'] = tun
    drop = {}
    frames = {(r, k): M for r, k, M in roller_frames()}
    for rk in ((1, 1), (1, 2), (2, 1), (2, 2)):
        M = frames[rk]
        for d in (0.0, 10.0, 20.0, 30.0):
            T = np.eye(4)
            T[:3, 3] = M[:3, 1] * d
            drop[f'{rk[0]}_{rk[1]} +{d:g}'] = round(sum(overlap(hub, shapes[f'roller_{p}_{rk[0]}_{rk[1]}'].moved(loc(T))) for p in ('core', 'tread')), 2)
    rep['roller_radial_drop_in_overlap_mm3'] = drop
    rep['spoke_neck_row2_az0_mm'] = neck_widths(hub)
    rep['roller_angular_overlap_deg'] = round(2 * math.degrees(math.asin(HT / R_WHEEL)) - 30, 2)
    rep['roller_bore_d'] = BORE_D
    return rep


# ---- export --------------------------------------------------------------------------------------------------------------------
STEP_SKIP = ('bolt_', 'bearing_', '_')          # no fasteners or bearings in any STEP; '_' = unplaced print copies


PRINTED = ('wheel_hub', 'wheel_cartridge', '_roller_core', '_roller_tread')


def export(shapes):
    parts = OUT / 'parts'
    for f in parts.glob('*.step'):
        f.unlink()
    for k in PRINTED:
        n = k.lstrip('_')
        write_part(shapes[k], colour(n), parts / (n + '.step'), n)
    write_step({k: (s, colour(k)) for k, s in shapes.items() if not k.startswith(STEP_SKIP)}, OUT / 'wheel_motor_assembly.step', 'wheel_motor')


# ---- render --------------------------------------------------------------------------------------------------------------------
def render(shapes):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection

    def cut(selected, transform, name, xlim, ylim, size, title):
        slab_ = b.Box(300, 300, 0.6)
        polys = []
        for k, s in shapes.items():
            if not selected(k):
                continue
            x = transform(s) & slab_
            if x is None or not x.faces():
                continue
            v, t = x.tessellate(0.12, 0.2)
            Vv = np.array([[q.X, q.Y] for q in v])
            polys += [(Vv[tri], colour(k)) for tri in np.array(t)]
        fig, ax = plt.subplots(figsize=size)
        ax.add_collection(PolyCollection([p[0] for p in polys], facecolors=[p[1] for p in polys], edgecolors='none'))
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect('equal')
        ax.grid(True, lw=.3)
        ax.set_title(title)
        fig.savefig(VAL / (name + '.png'), dpi=100)
        plt.close(fig)

    solid = lambda k: not k.startswith('_')                                                    # noqa: E731
    cut(lambda k: solid(k) and not k.startswith('roller_tread_'), lambda s: s, 'section_axis_plane_z0', (-48, 72), (-68, 68), (13, 9),
        'axis plane z = 0 (x along the wheel axis)')
    cut(lambda k: k == 'wheel_hub' or k.startswith(('roller_core', 'roller_tread')), lambda s: s.rotate(b.Axis.Y, 90).moved(b.Location((0, 0, D_ROW))),
        'section_row2_plane', (-75, 75), (-75, 75), (10, 10), 'across the row-2 spokes at x = +17.25 (hub orange, cores green, treads dark)')
    # across roller 2_1's axis through its head pocket, local x (pod x, the print axis) up the page: the double-teardrop driver tunnel
    M21 = {(r, k): M for r, k, M in roller_frames()}[(2, 1)]
    to_local = b.Location((0, 0, -(SEAT + HEAD_DEPTH / 2))) * loc(M21).inverse()
    cut(lambda k: k == 'wheel_hub', lambda s: s.moved(to_local).rotate(b.Axis.Z, 90), 'section_head_pocket', (-25, 25), (-20, 20), (8, 6.5),
        'across a roller axle at the head pocket (pod x = print axis vertical): Ø8 tunnel, 45 deg points')
    # round 7: one roller cut along its axis (axis up the page): the tread's rib in the core's V groove at the middle
    roller = lambda k: k in ('_roller_core', '_roller_tread')                                   # noqa: E731
    along = lambda s: s.rotate(b.Axis.Z, 30).rotate(b.Axis.X, -90)                             # noqa: E731  (cut between the keys)
    cut(roller, along, 'section_roller_detent', (-18, 18), (-23, 23), (7, 8.5), 'roller along its axis: TPU tread (dark) on the core (green)')
    cut(roller, along, 'section_roller_detent_zoom', (9, 17), (-3, 3), (8, 6.5), 'tread rib in the core groove (0.5 deep, 45 deg; rib 0.4)')
    # plane along x and the pin axis, through the first pin's centre (the pins are parallel, not radial)
    P = points(shapes['xt30_pin_1'], 0.02)
    d = np.linalg.svd(P - P.mean(0))[2][0]
    az = math.degrees(math.atan2(d[2], d[1]))
    az = az - 180 if az > 90 else az + 180 if az < -90 else az
    off = float(b.Vector(*P.mean(0)).rotate(b.Axis.X, -az).Z)
    cut(lambda k: solid(k) and not k.startswith(('bolt_', 'bearing_')), lambda s: s.rotate(b.Axis.X, -az).moved(b.Location((0, 0, -off))), 'section_plug_plane',
        (-45, 72), (0, 60), (14, 8), f'plane along x and the XT30 pin axis (az {az:.1f} deg, {off:.1f} mm off the axis): plug yellow, pin gold, lead, hub orange')

    def iso(items, out, az_deg, el, title):
        a, e = np.radians(az_deg), np.radians(el)
        Rm = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]]) @ np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
        polys = []
        for k, s in items.items():
            v, t = s.tessellate(0.25, 0.3)
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

    iso({'wheel_hub': shapes['wheel_hub']}, VAL / 'iso_hub.png', 150, 28, 'printed hub, cartridge side')
    iso({k: shapes[k] for k in shapes if k.startswith(('wheel_hub', 'roller_core_2', 'roller_tread_2'))}, VAL / 'iso_hub_row2.png', 330, 28,
        'hub + row-2 rollers, motor side')


def main(stage):
    for d in (OUT / 'parts', VAL):
        d.mkdir(parents=True, exist_ok=True)
    shapes, info = build_all()
    print('hub build', info, flush=True)
    if stage in ('check', 'all'):
        rep = check(shapes)
        rep['hub_build'] = info
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1))
        print(json.dumps({k: v for k, v in rep.items() if not k.startswith('axle_insertion')}, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(shapes)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(shapes)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')

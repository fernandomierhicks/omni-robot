"""Printed parts of the catch module, PAYLOAD frame (origin on the payload-plate top
surface, x forward, z up). Ball path: funnel throat -> pocket -> sliding gate ->
drop tube -> J-bend -> left/right flywheel nip -> 52 deg launch.

Print notes
  channel_half_L/R : PETG, lay on the outer side wall; the ball-path faces print
                     against nothing and come out smooth (no supports in the channel).
  launcher_base    : PETG, flat; motor plates overhang 38 deg from vertical.
  flywheel_hub     : PETG 100%, 8 x M4 bolt+nut inertia ring.
  flywheel_tire    : TPU 95A, press fit on the hub.
  shutter, pinion  : PETG (or PLA), 100% infill on the teeth.
"""

from __future__ import annotations

import math

from build123d import (Align, Axis, Box, Cylinder, Location, Part, Plane, Polygon, Pos, Rot, extrude, RegularPolygon)

import dims as D
from base import M3_CLEAR, M3_INSERT, M4_CLEAR, M4_INSERT

T = D.WALL
W = D.TUBE_W
CX, CZ = D.J_C
A_END = math.pi / 2 + D.THETA                 # J sweep: from straight down to the launch direction
TOP = D.THROAT_Z
GATE_HINGE = (-W - 1.5, D.GATE_Z - 1.5)       # (y, z) of the trapdoor hinge axis (along x)
SPLIT_BOSSES = ((-W - T - 4.0, 130.0), (-W - T - 4.0, 205.0), (W + T + 4.0, 205.0))
BASE_X0, BASE_X1 = -88.0, 160.0               # launcher base extent (248 mm: fits a 256 mm bed)
BASE_T = 5.0                                  # launcher base plate thickness; channel halves sit on it


# ---------------------------------------------------------------------------
# helpers

def prism_xz(pts, y0, y1) -> Part:
    """Extrude a polygon given in the x-z plane between y0 and y1."""
    area = sum(x0 * z1 - x1 * z0 for (x0, z0), (x1, z1) in zip(pts, pts[1:] + pts[:1]))
    if area < 0:                       # extrude follows the face normal: keep it +Z (counter-clockwise)
        pts = list(reversed(pts))
    face = Polygon(*pts, align=None)
    solid = extrude(face, y1 - y0).rotate(Axis.X, 90)          # (x, y, z) -> (x, -z, y)
    return solid.moved(Pos(0, y1, 0))


def arc_pts(cx, cz, r, a0, a1, n=72):
    """Points on the J arc: P(a) = C + r(-cos a, -sin a), a from a0 to a1 (radians)."""
    return [(cx - r * math.cos(a0 + (a1 - a0) * k / n), cz - r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def flywheel_frame(side: int) -> Location:
    """Flywheel center frame: z along the wheel axis (launch-line normal), side = +1 left / -1 right."""
    return Location((D.NIP[0], side * D.FW_Y, D.NIP[1]), (0, -math.degrees(D.THETA), 0))


# ---------------------------------------------------------------------------
# channel halves

def channel_half(side: int) -> Part:
    """Half of the ball channel (side=+1: y >= 0, side=-1: y <= 0). Side wall + half of
    the pocket / drop tube / J walls, gate rails, break-beam hole, feet and top ears."""
    y_in, y_out = 0.0, W + T
    sgn = 1 if side > 0 else -1

    def span(a, b):
        lo, hi = sorted((sgn * a, sgn * b))
        return lo, hi

    # side wall: tube region + J region (outline walks the outer J arc back to its start)
    jo = arc_pts(CX, CZ, D.J_R + T, 0.0, A_END)
    exit_pt = jo[-1]
    outline = [(-W - T, TOP), (W + T, TOP), (W + T, CZ), (exit_pt[0], exit_pt[1] + 10.0)] + list(reversed(jo))[:-1]
    wall = prism_xz(outline, *span(W, W + T))
    # fill under the J between the side walls' lower edge and the base (stiffness + feet)
    under = [(jo[k][0], jo[k][1]) for k in range(len(jo))] + [(jo[-1][0], BASE_T), (-W - T, BASE_T)]
    wall += prism_xz(under, *span(W, W + T))
    # front and back walls of the pocket / drop tube (back wall continues into the J);
    # everything overlaps its neighbors by 0.5-1 mm so the booleans fuse into one solid
    wall += prism_xz([(W, CZ - 1), (W + T, CZ - 1), (W + T, TOP), (W, TOP)], *span(0, W + 0.5))
    wall += prism_xz([(-W - T, CZ - 1), (-W, CZ - 1), (-W, TOP), (-W - T, TOP)], *span(0, W + 0.5))
    # J outer wall (ring sector), half width
    ji = arc_pts(CX, CZ, D.J_R, 0.0, A_END)
    ring = ji + list(reversed(jo))
    wall += prism_xz(ring, *span(0, W + 0.5))
    # feet on the base (within the wall's y-span so the half prints flat on its side)
    for x0, x1 in ((-W - T - 18.0, -W - T + 1.0), (jo[-1][0] - 1.0, jo[-1][0] + 18.0)):
        wall += prism_xz([(x0, BASE_T), (x1, BASE_T), (x1, BASE_T + 6.0), (x0, BASE_T + 6.0)], *span(8, W + T))
    # ears at the top for the funnel collar
    for x0, x1 in ((-W - T - 16.0, -W - T + 1.0), (W + T - 1.0, W + T + 16.0)):
        wall += prism_xz([(x0, TOP - 6.0), (x1, TOP - 6.0), (x1, TOP), (x0, TOP)], *span(8, W + T))
    # split-line bosses (M3 through, nut on the other half)
    for x, z in SPLIT_BOSSES:
        wall += Cylinder(5.0, W + T, rotation=(90, 0, 0)).moved(Pos(x, sgn * (W + T) / 2, z))

    # --- cuts ---------------------------------------------------------------------
    # ball path: pocket + tube, J channel (lower half of the J disk), exit duct to the nip
    wall -= Box(2 * W, 2 * W, TOP - CZ + 20).moved(Pos(0, 0, (TOP + CZ) / 2 + 5))
    disk = Cylinder(D.J_R, 2 * W, rotation=(90, 0, 0)).moved(Pos(CX, 0, CZ))
    wall -= disk & Box(600, 2 * W, 300, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, CZ))
    ux, uz = D.U
    nx, nz = D.NRM
    ex, ez = D.J_E
    duct = [(ex - nx * W, ez - nz * W), (ex - nx * W + ux * 160, ez - nz * W + uz * 160),
            (ex + nx * W + ux * 160, ez + nz * W + uz * 160), (ex + nx * W, ez + nz * W)]
    wall -= prism_xz(duct, -W, W)
    # flywheel windows
    for s in (-1, 1):
        wall -= Cylinder(D.FW_R + 4.0, D.FW_W + 8.0).moved(flywheel_frame(s))
    # gate: hinge pin through the front/back walls, servo coupling through the back
    # wall, and a window in the -y wall so the open flap lies flush with the wall
    if side < 0:
        hy, hz = GATE_HINGE
        wall -= Cylinder(1.6, 200, rotation=(0, 90, 0)).moved(Pos(0, hy, hz))
        wall -= Cylinder(3.9, 2 * W + 2, rotation=(0, 90, 0)).moved(Pos(0, hy, hz))           # knuckle clearance
        wall -= Cylinder(6.0, 20, rotation=(0, 90, 0)).moved(Pos(-W - T / 2, hy, hz))          # coupling
        wall -= Box(2 * W - 1.0, T + 2, 82.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
            Pos(0, -W - T / 2, hz - 3.0))
    # break-beam: IR emitter / receiver across x at the held-ball height
    wall -= Cylinder(2.6, 2 * (W + T) + 20, rotation=(0, 90, 0)).moved(Pos(0, 0, D.HOLD_Z))
    # holes: feet, ears, bosses
    jx = jo[-1][0]
    for x in (-W - T - 9.0, jx + 9.0):
        wall -= Cylinder(M3_CLEAR / 2, 20).moved(Pos(x, sgn * (W + T - 8.0), BASE_T))
    for x in (-W - T - 8.0, W + T + 8.0):
        wall -= Cylinder(M3_INSERT / 2, 20).moved(Pos(x, sgn * (W + T - 8.0), TOP))
    for x, z in SPLIT_BOSSES:
        wall -= Cylinder(M3_CLEAR / 2, 3 * W, rotation=(90, 0, 0)).moved(Pos(x, 0, z))
    # keep this half only
    keep = Box(600, W + T + 30, 600, align=(Align.CENTER, Align.MIN if side > 0 else Align.MAX, Align.CENTER))
    wall = wall & keep
    wall.label = f"channel_half_{'L' if side > 0 else 'R'}"
    return wall


# ---------------------------------------------------------------------------

def launcher_base() -> Part:
    """Flat plate under the channel halves, carrying both tilted flywheel-motor plates.
    Bolts to the payload plate with 4 captive M4 thumbscrews; 2 dowel holes locate it."""
    from base import PIN_XY, THUMBSCREW_XY
    x0, x1 = BASE_X0, BASE_X1
    p = Box(x1 - x0, 190.0, 5.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(x0, 0, 0))
    # flywheel motor pedestals
    nvec = (D.NRM[0], 0.0, D.NRM[1])
    for s in (-1, 1):
        fw = flywheel_frame(s)
        s_top = -(D.FW_W / 2 + 2.0 + D.FM_H)           # motor base plane, along the wheel axis
        plate = Box(42.0, 42.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(fw * Pos(0, 0, s_top))
        c = (fw * Pos(0, 0, s_top - 2.5)).position
        col = Box(30.0, 30.0, c.Z + 10.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(c.X, c.Y, 0))
        above = Box(200, 200, 200, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(fw * Pos(0, 0, s_top))
        p += (col - above) + plate
        for dx in (-D.FM_MOUNT / 2, D.FM_MOUNT / 2):
            for dy in (-D.FM_MOUNT / 2, D.FM_MOUNT / 2):
                p -= Cylinder(M3_CLEAR / 2, 30).moved(fw * Pos(dx, dy, s_top))
        p -= Cylinder(5.0, 30).moved(fw * Pos(0, 0, s_top))
    # channel-half feet (M3 inserts) and payload interface
    jo_x = CX - (D.J_R + T) * math.cos(A_END)
    for sy in (-1, 1):
        for x in (-W - T - 9.0, jo_x + 9.0):
            p -= Cylinder(M3_INSERT / 2, 30).moved(Pos(x, sy * (W + T - 8.0), 5.0))
    for x, y in THUMBSCREW_XY:
        p -= Cylinder(M4_CLEAR / 2, 30).moved(Pos(x, y, 0))
    for x, y in SERVO_TOWER_HOLES:
        p -= Cylinder(M3_INSERT / 2, 30).moved(Pos(x, y, 5.0))
    for x, y in PIN_XY:
        p -= Cylinder(4.05 / 2, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, -0.01))
    p -= Cylinder(16.0, 30)                                            # payload connector
    p.label = "launcher_base"
    return p


def gate_flap() -> Part:
    """Trapdoor under the catch pocket, PAYLOAD frame, closed. Hinged on a Ø3 steel pin
    along x at the -y wall; opens 95 deg downward into the wall window so the ball drops
    straight through the full tube width. The back end of the knuckle passes through
    the back wall and carries a disc for a 25T aluminum servo horn (4 x M2 on Ø14,
    VERIFY for the horn). Glue a 3 mm EVA foam pad on top: it soaks up the catch impact."""
    hy, hz = GATE_HINGE
    zg = D.GATE_Z
    plate = Box(2 * W - 1.2, W - hy - 3.5 - 0.4, 3.0, align=(Align.CENTER, Align.MIN, Align.MAX)).moved(Pos(0, hy + 3.5, zg))
    knuckle = Cylinder(3.5, 2 * W - 1.2, rotation=(0, 90, 0)).moved(Pos(0, hy, hz))
    shaft = Cylinder(5.0, 12.0, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(-W - T - 8.0, hy, hz))
    disc = Cylinder(12.0, 3.0, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(-W - T - 11.0, hy, hz))
    p = plate + knuckle + shaft + disc
    p -= Cylinder(1.6, 2 * W + 2, rotation=(0, 90, 0)).moved(Pos(0, hy, hz))           # hinge pin
    for k in range(4):
        a = math.pi / 4 + k * math.pi / 2
        p -= Cylinder(1.1, 10, rotation=(0, 90, 0)).moved(Pos(-W - T - 9.0, hy + 7.0 * math.cos(a), hz + 7.0 * math.sin(a)))
    p -= Cylinder(4.2, 2.0, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(-W - T - 11.01, hy, hz))
    p.label = "gate_flap"
    return p


def servo_location() -> Location:
    """Gate servo output-shaft frame (PAYLOAD): shaft along +x on the hinge axis, body
    behind the back wall, long side vertical. Vendor servo frame: shaft +Z, body below."""
    hy, hz = GATE_HINGE
    return Location((-W - T - 11.0 - 2.0, hy, hz), (0, 90, 0))


SERVO_TOWER_HOLES = ((-80.0, -28.0), (-80.0, -53.0))


def servo_tower() -> Part:
    """Standing plate on the launcher base that holds the gate servo by its tabs."""
    sl = servo_location()
    L, Wd, H = D.SERVO
    x_face = (sl * Pos(0, 0, -10.0 - D.SERVO_TAB[1])).position.X  # tab underside rests on the plate
    zc = (sl * Pos(L / 2 - 10.0, 0, 0)).position.Z
    yc = sl.position.Y
    z_top = zc + D.SERVO_TAB[0] / 2 + 8.0
    plate = Box(4.0, 44.0, z_top - 5.0, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(x_face, yc, 5.0))
    plate -= Box(10.0, Wd + 0.6, L + 0.6).moved(Pos(x_face - 2.0, yc, zc))
    for dz in (-D.SERVO_HOLES[0] / 2, D.SERVO_HOLES[0] / 2):
        for dy in (-D.SERVO_HOLES[1] / 2, D.SERVO_HOLES[1] / 2):
            plate -= Cylinder(1.6, 20, rotation=(0, 90, 0)).moved(Pos(x_face, yc + dy, zc + dz))
    foot = Box(x_face - 4.0 - BASE_X0, 44.0, 5.0, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(x_face - 4.0, yc, 5.0))
    for x, y in SERVO_TOWER_HOLES:
        foot -= Cylinder(M3_CLEAR / 2, 20).moved(Pos(x, y, 5.0))
    rib = Box(20.0, 4.0, 40.0, align=(Align.MAX, Align.CENTER, Align.MIN)).moved(Pos(x_face - 4.0, yc, 5.0))
    p = plate + foot + rib
    p.label = "servo_tower"
    return p


# ---------------------------------------------------------------------------
# flywheels

def flywheel_hub() -> Part:
    """PETG hub, Ø58 x 28. Frame: axis Z, motor side at z = -14. M5 prop shaft through the
    bore with an M5 nylock on the outer face; 8 x M4 bolt+nut inertia ring at r = 21."""
    r_hub = D.FW_R - 6.0
    p = Cylinder(r_hub, D.FW_W)
    p -= Cylinder(2.6, 40)
    p -= Cylinder(4.6, 6.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(0, 0, D.FW_W / 2 + 0.01))   # nut
    p -= Cylinder(11.0, 1.5, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(0, 0, -D.FW_W / 2 - 0.01))  # bell top
    for k in range(8):
        a = 2 * math.pi * k / 8
        p -= Cylinder(M4_CLEAR / 2, 40).moved(Pos(21.0 * math.cos(a), 21.0 * math.sin(a), 0))
    p.label = "flywheel_hub"
    return p


def flywheel_tire() -> Part:
    """TPU 95A tire, OD 70 / ID 57.6 (0.2 mm interference on the hub), 28 wide."""
    p = Cylinder(D.FW_R, D.FW_W) - Cylinder(D.FW_R - 6.0 - 0.2, D.FW_W + 2)
    p.label = "flywheel_tire"
    return p


# ---------------------------------------------------------------------------
# funnel

def funnel_collar() -> Part:
    """Ring on the channel halves' top ears; the net's bottom hem is laced through the
    slots, and the six struts plug into the angled sockets. Frame: PAYLOAD."""
    z0 = TOP
    r_in = D.FUNNEL_THROAT_R
    p = Cylinder(r_in + 22.0, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(0, 0, z0))
    p -= Cylinder(r_in, 30).moved(Pos(0, 0, z0))
    p -= Box(2 * W, 2 * W, 30).moved(Pos(0, 0, z0))            # matches the pocket below
    for x in (-W - T - 8.0, W + T + 8.0):
        for y in (-(W + T - 8.0), W + T - 8.0):
            p -= Cylinder(M3_CLEAR / 2, 30).moved(Pos(x, y, z0))
    for k in range(24):                                         # lacing holes for the net hem
        a = 2 * math.pi * (k + 0.5) / 24
        p -= Cylinder(1.6, 30).moved(Pos((r_in + 6.0) * math.cos(a), (r_in + 6.0) * math.sin(a), z0))
    for k in range(6):                                          # strut sockets
        a = math.radians(30 + 60 * k)
        loc = strut_location(k)
        p -= Cylinder(4.1, 30.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(loc * Pos(0, 0, -6.0))
        p += Cylinder(7.0, 14.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(loc * Pos(0, 0, -2.0)) - \
            Cylinder(4.1, 30.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(loc * Pos(0, 0, -6.0))
    p.label = "funnel_collar"
    return p


def strut_ends(k: int):
    a = math.radians(30 + 60 * k)
    r0, z0 = D.FUNNEL_THROAT_R + 16.0, TOP + 6.0
    r1, z1 = D.FUNNEL_R + 2.0, D.RIM_Z - 4.0
    return (r0 * math.cos(a), r0 * math.sin(a), z0), (r1 * math.cos(a), r1 * math.sin(a), z1)


def strut_location(k: int) -> Location:
    (x0, y0, z0), (x1, y1, z1) = strut_ends(k)
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    tilt = math.degrees(math.atan2(math.hypot(dx, dy), dz))
    az = math.degrees(math.atan2(dy, dx))
    return Location((x0, y0, z0)) * Rot(0, 0, az) * Rot(0, tilt, 0)


def strut_length(k: int = 0) -> float:
    (x0, y0, z0), (x1, y1, z1) = strut_ends(k)
    return math.dist((x0, y0, z0), (x1, y1, z1))


RIM_RC = D.FUNNEL_R + 6.0                    # rim section centerline radius
CAM_ANGLE = 33.7                              # cameras sit on the rim at +/- this angle
CAM_TILT = 30.0                               # optical axes: forward, tilted up


def _wedge(a0, a1, z0, h):
    big = 400.0
    pts = [(0, 0)] + [(big * math.cos(math.radians(a)), big * math.sin(math.radians(a))) for a in (a0, (a0 + a1) / 2, a1)]
    return extrude(Polygon(*pts, align=None), h).moved(Pos(0, 0, z0))


def camera_pad_xy(side: int):
    return D.polar(RIM_RC, side * CAM_ANGLE)


def funnel_rim_arc(k: int) -> Part:
    """One of four 90-degree rim arcs (PETG), 12 x 10 section. Half-lap joints: the
    start of each arc is a top-half rabbet, the end carries a bottom-half tab with an
    M3 insert. Groove on the inner face for the net's top hem; strut sockets drilled
    along the strut axis; the two front arcs carry the stereo-camera pads."""
    r, z = RIM_RC, D.RIM_Z
    a0, a1 = 90 * k, 90 * k + 90
    ring = (Cylinder(r + 6.0, 10.0) - Cylinder(r - 6.0, 12.0)).moved(Pos(0, 0, z))
    ring -= (Cylinder(r - 3.0, 4.0) - Cylinder(r - 6.5, 4.0)).moved(Pos(0, 0, z))        # hem groove
    p = ring & _wedge(a0, a1 + 6.0, z - 6.0, 12.0)
    p -= ring & _wedge(a0 - 1.0, a0 + 6.0, z - 6.0, 6.0)                                 # rabbet (bottom half removed)
    p -= ring & _wedge(a1, a1 + 7.0, z, 6.0)                                             # tab (top half removed)
    for a, dia in ((a0 + 3.0, M3_CLEAR), (a1 + 3.0, M3_INSERT)):
        x, y = D.polar(r, a)
        p -= Cylinder(dia / 2, 30).moved(Pos(x, y, z))
    for s in range(6):                       # every socket: struts near a joint pass through both arcs
        p -= Cylinder(4.1, 24.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            strut_location(s) * Pos(0, 0, strut_length(s) - 10.0))
    for side in (1, -1):
        ang = (side * CAM_ANGLE) % 360
        if a0 <= ang < a1:
            x, y = camera_pad_xy(side)
            p += Cylinder(11.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(x, y, z + 5.0))
            for dx in (-6.0, 6.0):
                p -= Cylinder(M3_INSERT / 2, 8.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(Pos(x + dx, y, z + 9.01))
    p.label = f"funnel_rim_arc_{k}"
    return p


def _camera_holder_center():
    return (12.0, 0.0, 30.0)


def camera_location(side: int) -> Location:
    """OV9782 board frame (vendor: board back at z = 0, lens +Z) in the PAYLOAD frame."""
    x, y = camera_pad_xy(side)
    cx, cy, cz = _camera_holder_center()
    d = (math.cos(math.radians(CAM_TILT)), 0.0, math.sin(math.radians(CAM_TILT)))
    return Location((x + cx + 2.0 * d[0], y + cy, D.RIM_Z + 9.0 + cz + 2.0 * d[2]), (0, 90 - CAM_TILT, 0))


def camera_bracket(side: int) -> Part:
    """Bolts to a rim pad (2 x M3); holds an OV9782 board (38 x 38, 4 x M2 on 34 mm,
    VERIFY) with its optical axis forward and tilted up 30 deg. Both cameras face the
    same way: stereo baseline = 2 x 171 x sin(33.7) = 190 mm."""
    cx, cy, cz = _camera_holder_center()
    base = Cylinder(11.0, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    block = Box(18.0, 22.0, 16.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(-4.0, 0, 0))
    holder = Box(4.0, 42.0, 42.0).rotate(Axis.Y, -CAM_TILT).moved(Pos(cx, cy, cz))
    p = base + block + holder
    p -= Cylinder(8.5, 20.0).rotate(Axis.Y, 90 - CAM_TILT).moved(Pos(cx, cy, cz))       # lens
    for dy in (-17.0, 17.0):
        for dz in (-17.0, 17.0):
            off = Location((cx, cy, cz), (0, -CAM_TILT, 0)) * Pos(0, dy, dz)
            p -= Cylinder(1.1, 20.0).rotate(Axis.Y, 90).moved(off)
    for dx in (-6.0, 6.0):
        p -= Cylinder(M3_CLEAR / 2, 40.0).moved(Pos(dx, 0, 0))
        p -= Cylinder(3.0, 12.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Pos(dx, 0, 4.0))   # head clearance
    x, y = camera_pad_xy(side)
    p = p.moved(Pos(x, y, D.RIM_Z + 9.0))
    p.label = f"camera_bracket_{'L' if side > 0 else 'R'}"
    return p


def funnel_net_pattern(path) -> dict:
    """Flat sewing pattern for the conical net (annular sector), with 10 mm hem allowance."""
    a = D.FUNNEL_ANGLE
    s_top = (D.FUNNEL_R + 6.0) / math.sin(a) + 10.0
    s_bot = D.FUNNEL_THROAT_R / math.sin(a) - 10.0
    sweep = 360.0 * math.sin(a) + 4.0                       # + seam overlap
    n = 120
    outer = [(s_top * math.cos(math.radians(sweep * k / n)), s_top * math.sin(math.radians(sweep * k / n))) for k in range(n + 1)]
    inner = [(s_bot * math.cos(math.radians(sweep * k / n)), s_bot * math.sin(math.radians(sweep * k / n))) for k in range(n, -1, -1)]
    pts = outer + inner
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    minx, miny = min(xs) - 10, min(ys) - 10
    w, h = max(xs) - minx + 10, max(ys) - miny + 10
    d = " ".join(f"{'M' if i == 0 else 'L'}{x - minx:.2f},{h - (y - miny):.2f}" for i, (x, y) in enumerate(pts)) + " Z"
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w:.1f}mm" height="{h:.1f}mm" viewBox="0 0 {w:.1f} {h:.1f}">'
           f'<path d="{d}" fill="none" stroke="black" stroke-width="0.5"/>'
           f'<text x="{w / 2:.0f}" y="{h / 2:.0f}" font-size="8" text-anchor="middle">funnel net: cut 1, sew radial seam '
           f'({sweep - 4:.1f} deg + 4 deg overlap), 10 mm hems</text></svg>')
    path.write_text(svg)
    return dict(outer_radius=s_top, inner_radius=s_bot, sweep_deg=sweep)

"""v15 second-floor fins, lidar and pedestal (Nov 14 2026, PROPOSAL; replaces v14's chassis.py fins, which were patched onto the mid deck).

Library: chassis_mid.py builds the fins (build_fins) once it knows the deck's free plan and the heights of everything that stands on the deck;
upper.py reads the fins' top interface (FINS, FIN_DIMS, fin_xy, floor_pts, FIN_FLANGE).

What stays from v14 (the second floor's interface and the lidar's view, unchanged):
 * five fins, each in a vertical plane through the lidar's axis (x LIDAR_X): the spine behind it (bearing 180), two side fins (+-136.7, the
   rays under the arm riser's rear corners), two front fins (+-36). At the scan plane the lidar sees only a web edge-on: only a 5 mm web
   (a wedge as wide as its own shadow) crosses SUPPORT_PLANE_BAND; a 16 mm flange the fin's whole length under the second floor (z 176.5);
   the floor's M3 x 12 come down into it at the same points (floor_pts).
 * the fins print upside down on their flange, without supports.
What is new (Fernando's review, Nov 14: "redesign the fins"):
 * the lower edge is GENERATED, not drawn: a height map of everything above the deck (the pods' moving parts in five poses from droop to
   bump with the wheel's revolved envelope, plungers and springs; the towers, their bolts and nuts; the pedestal and the lidar; the battery
   straps over the deck) gives, along each fin, the lowest z its lower edge may come down to (+3 mm to moving parts, +2 to static, then
   R_EDGE all round). Where nothing stands and the deck is solid underneath, the fin stands on the deck (a foot); where the deck is open,
   its edge stays NOFOOT over the deck plane; long feet get lightening arches. Change the pods, the deck or what stands on it and the fins
   follow on the next build.
 * every foot is held by M3 x 25 pan heads up from under the deck (counterbored) into Ø2.8 pilots along the fin's print axis, placed where
   screw_ok (chassis_mid: off the seams, other screws, the battery, the stand pads, the bumper tabs) allows. No long bolts (v14's front fins
   had an M3 x 130, then x 90, from the floor's top: longer than any M3 Fernando has), no studs on the pods' columns (v14's side fins sat
   on pod_lean's spring column, which v15's pods no longer have).
 * the front fins get one floor screw (at the old long bolt's place, so the floor's hole is where it was).
"""
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import Point, Polygon, box as sbox

TOOLS = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS))
import base as B                                                       # noqa: E402
import pod as P                                                        # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL, M3_PILOT, M3_CLR, TIE_SLOT_L, TIE_SLOT_W     # noqa: E402

U_ = lambda a: np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])   # noqa: E731
cyl, box = W.cyl, P.box

# ---- the deck's height (pod_mid.DECK_Z + R_WHEEL; repeated here so upper.py need not build the pod) ---------------------------------------
Z0, Z1 = 80.0, 100.0                       # the deck's underside and top (robot z)

# ---- lidar (A2M12) and its pedestal ----------------------------------------------------------------------------------------------------------
LIDAR = dict(d=76.0, h=41.0, plane=163.0, plane_above_base=31.3)        # Nov 4: 31.3 from the vendor RPLIDAR model's Laser_link; VERIFY on the unit
LIDAR_BASE_Z = LIDAR['plane'] - LIDAR['plane_above_base']               # 131.7
LIDAR_X = 120.0                            # forward, where the scans matter
LIDAR_HOLES_R = 26.0                       # A2M12 base mounting holes: 4 x M2.5 on this circle (PLACEHOLDER, VERIFY on the unit)
PED_FLANGE_R, PED_R, PED_WALL, PED_TOP_R, PED_SCREW_R = 31.0, 20.0, 3.0, 36.5, 25.5   # the pedestal: flange on the deck, tube, top plate;
                                           # its four M3 x 14 down into the deck on r 25.5

# ---- the second floor (upper.py) ------------------------------------------------------------------------------------------------------------
FLOOR2_UNDER = 176.5                       # second floor underside (layout F; v14's height, kept)
FLOOR2_TOP = FLOOR2_UNDER + 5.0            # its top (upper.Z_TOP)

# ---- fins ------------------------------------------------------------------------------------------------------------------------------------
SUPPORT_PLANE_BAND = (148.0, 170.0)        # only the web crosses this band (robot z); the scan plane is 163
FIN_WEB, FIN_FLANGE = 5.0, 16.0            # web in the scan band; top flange under the floor
FIN_LEG = dict(spine=(4.0, 7.0), side=(4.0, 4.5), front=(4.0, 4.5))   # the lower body's half width from z 120 up / at the deck (it flares)
SPINE = dict(bearing=180.0, top=(44.0, 336.0),   # Nov 15 evening: from u 44 (was 50; the lidar's body is at u 38), so its nose reaches the deck as a foot
 floor=(80.0, 182.0, 230.0, 275.0, 328.0), web_from=54.3)   # web_from: no web in the
                                           # band nearer the lidar than this (v14's nose cove: the spine's shadow 5.3 deg, not 5.7)
SIDE = dict(bearing=136.7, top=(78.0, 238.0), floor=(86.0, 119.0, 155.0, 205.0, 232.0))   # the nose (u 78) under the arm riser's rear corner
FRONT = dict(bearing=36.0, top=(84.0, 109.0), floor=(105.0,), web=6.0)   # floor screw at u 105: v14's long bolt's place
FINS = (('support_spine', 'spine', 1), ('support_side_L', 'side', 1), ('support_side_R', 'side', -1),
        ('support_front_L', 'front', 1), ('support_front_R', 'front', -1))
FIN_DIMS = dict(spine=SPINE, side=SIDE, front=FRONT)
TIE_FIN_U = dict(side=(150.0, 185.0, 215.0), spine=(110.0, 140.0, 200.0, 230.0, 260.0, 290.0), front=(97.0,))   # zip-tie slots (u)
TIE_FIN_ABOVE = 6.0                        # a tie slot this far over the fin's lower edge (the tie goes through it and round that edge)
TIE_SCREW_KEEP = 7.0                       # no tie slot within this (along the fin) of a foot's screw
# the generated lower edge
CLR_MOVING, CLR_STATIC = 3.0, 2.0          # to the pods' moving parts (every pose) / to what stands still; in plan and in z
R_EDGE = 3.0                               # the cut-out round the forbidden region is rounded this much (and adds this much clearance)
NOFOOT = 8.0                               # over open deck the edge stays this far over the deck plane
FOOT_MIN = 12.0                            # a contact run shorter than this is not a foot
ARCH = dict(foot=18.0, span_max=64.0, span_min=26.0, h=22.0)   # lightening arches in long feet: feet this long, arches this wide and high
FIN_SCREW_L, FIN_PILOT_DEPTH = 25.0, 10.0  # feet: M3 x 25 up through the 20 mm deck (head counterbored 2.9): 7.9 of thread, a short
                                           # regular thread for a Ø2.8 pilot (cad/solidworks.md 18); the pilot runs on 2 mm past the tip
FLOOR_PILOT_DEPTH = 7.0                    # the floor's M3 x 12 down into the flange
SCREW_PITCH = 34.0                         # feet longer than 2 x this get a screw near each end and more between
FOOT_TWO = 22.0                            # Nov 15 evening: feet at least this long get two screws (one near each end), shorter ones one


def fin_axis(kind, side=1):
    bearing = side * FIN_DIMS[kind]['bearing']
    return np.array([LIDAR_X, 0.0]), U_(bearing), bearing


def fin_xy(kind, u, side=1):
    lc, d, _ = fin_axis(kind, side)
    return lc + d * u


def floor_pts(kind, side=1):
    """the second floor's screws down into a fin's flange (plan)"""
    return [tuple(fin_xy(kind, u, side)) for u in FIN_DIMS[kind]['floor']]


def fin_cross_section(kind):
    """across the fin against z: the lower body flaring toward the deck, only the web through the scan band, the flange under the floor"""
    (w_up, w_deck), w, f, (b0, b1) = FIN_LEG[kind], FIN_WEB / 2, FIN_FLANGE / 2, SUPPORT_PLANE_BAND
    half = [(w_deck, Z1 - 2.0), (w_up, 120.0), (w_up, b0 - 8.0), (w, b0), (w, b1), (f, b1 + 2.5), (f, FLOOR2_UNDER + 1.0)]
    return Polygon([(x, z) for x, z in half] + [(-x, z) for x, z in reversed(half)])


def wedge(u_near, web, u_far):
    """the fin's plan in the scan band (fin frame: u from the lidar's axis, v across): the cone of its own lidar shadow (web wide at u_near,
    so no wider a shadow than the plain web's), capped at the flange's width"""
    t = web / 2 / u_near
    return Polygon([(0.0, 0.0), (u_far, -u_far * t), (u_far, u_far * t)]).intersection(sbox(0.0, -FIN_FLANGE / 2, u_far, FIN_FLANGE / 2))


def lidar_env():
    return cyl(LIDAR['d'] / 2, LIDAR['h'], (LIDAR_X, 0, LIDAR_BASE_Z + LIDAR['h'] / 2))


def pedestal_screws():
    return [tuple(np.array([LIDAR_X, 0.0]) + U_(a) * PED_SCREW_R) for a in (45, 135, 225, 315)]


def build_pedestal():
    """flange on the deck (4 x M3 down into it), a tube, the lidar's top plate (its 4 x M2.5 pilots), the lead's slot out of the back"""
    z0, z1 = Z1, LIDAR_BASE_Z
    c = (LIDAR_X, 0.0)
    p = cyl(PED_FLANGE_R, 4.0, (*c, z0 + 2.0)) + cyl(PED_R, z1 - z0, (*c, (z0 + z1) / 2)) + cyl(PED_TOP_R, 5.0, (*c, z1 - 2.5))
    h = PED_TOP_R - PED_R                                              # Nov 14 audit: a 45 deg cone under the top plate (it printed as a
    p = p + b.Solid.make_cone(PED_R, PED_TOP_R, h, b.Plane(origin=(*c, z1 - 5.0 - h), z_dir=(0, 0, 1)))   # 16.5 mm overhang ring all round)
    p = p - cyl(PED_R - PED_WALL, z1 - z0 - 5.0 + 0.1, (*c, (z0 + z1 - 5.0) / 2))
    for x, y in pedestal_screws():
        p = p - cyl(M3_CLR / 2, 4.2, (x, y, z0 + 2.0))
    for a in (45, 135, 225, 315):
        x, y = np.array(c) + U_(a) * LIDAR_HOLES_R
        p = p - cyl(1.35, 5.2, (x, y, z1 - 2.5))
    p = p - box(c[0] - PED_TOP_R - 1, c[0] - PED_R + PED_WALL + 1, -6, 6, z1 - 30, z1 - 5)  # the lidar's lead out of the back (tube and cone)
    p = p - cyl(8.0, 5.2, (*c, z1 - 2.5))
    return p


# ---- height map: the top of everything that stands on the deck, on a 1 mm grid ------------------------------------------------------------
class HeightMap:
    """two layers (moving, static) of the highest z per 1 mm plan cell over the robot; cells at or under the deck's top are empty"""
    ORG, N = -245.0, 490

    def __init__(self):
        self.h = dict(moving=np.full((self.N, self.N), -np.inf), static=np.full((self.N, self.N), -np.inf))

    def cells(self, xy):
        ij = np.floor((np.asarray(xy, float) - self.ORG)).astype(int)
        ok = np.all((ij >= 0) & (ij < self.N), axis=1)
        return ij[ok], ok

    def add_points(self, pts, layer):
        pts = np.asarray(pts, float)
        ij, ok = self.cells(pts[:, :2])
        np.maximum.at(self.h[layer], (ij[:, 0], ij[:, 1]), pts[ok, 2])

    def add_shape(self, s, layer, density=2.0, rng=np.random.default_rng(1)):
        vs, ts = s.tessellate(0.3, 0.3)
        if not ts:
            return
        V = np.array([[q.X, q.Y, q.Z] for q in vs])
        T = np.array(ts, dtype=int).reshape(-1, 3)
        A, Bv, Cv = V[T[:, 0]], V[T[:, 1]], V[T[:, 2]]
        area = 0.5 * np.linalg.norm(np.cross(Bv - A, Cv - A), axis=1)
        idx = np.repeat(np.arange(len(T)), np.maximum(1, np.ceil(area * density)).astype(int))
        r1, r2 = rng.random(len(idx)), rng.random(len(idx))
        sq = np.sqrt(r1)
        pts = (1 - sq)[:, None] * A[idx] + (sq * (1 - r2))[:, None] * Bv[idx] + (sq * r2)[:, None] * Cv[idx]
        self.add_points(np.vstack([V, pts]), layer)

    def add_box(self, x0, x1, y0, y1, z, layer):
        xs, ys = np.arange(x0, x1 + 0.5, 0.5), np.arange(y0, y1 + 0.5, 0.5)
        X, Y = np.meshgrid(xs, ys)
        self.add_points(np.stack([X.ravel(), Y.ravel(), np.full(X.size, z)], 1), layer)

    def add_wheel(self, i, a, env):
        """the wheel's revolved envelope (pod_rx.wheel_envelope: (x, r) about the axle) of pod i at fork angle a: its top over every cell"""
        from shapely.geometry import LineString
        x0, _, x1, _ = env.bounds
        xg = np.arange(x0, x1 + 0.25, 0.25)
        rg = np.array([(lambda c: c.bounds[3] if not c.is_empty else -1.0)(env.intersection(LineString([(x, -1.0), (x, 500.0)]))) for x in xg])
        ya, za = (float(v) for v in P.rot_pt((0.0, 0.0), a))
        A, xh, yh, m = B.frame(i)
        s = -1.0 if m else 1.0
        g = self.ORG + 0.5 + np.arange(self.N)
        X, Y = np.meshgrid(g, g, indexing='ij')
        dX, dY = X - A[0], Y - A[1]
        xp, yp = dX * xh[0] + dY * xh[1], s * (dX * yh[0] + dY * yh[1])
        r = np.interp(xp, xg, rg, left=-1.0, right=-1.0)
        dy = yp - ya
        inside = (r > 0) & (np.abs(dy) < r)
        z = np.where(inside, za + np.sqrt(np.maximum(r * r - dy * dy, 0.0)) + R_WHEEL, -np.inf)
        self.h['moving'] = np.maximum(self.h['moving'], z)

    def finish(self):
        """empty the cells at or under the deck's top; dilate each layer by its clearance in plan; one 'required lower edge' map"""
        from scipy.ndimage import maximum_filter
        out = np.full((self.N, self.N), -np.inf)
        for layer, c in (('moving', CLR_MOVING), ('static', CLR_STATIC)):
            h = np.where(self.h[layer] > Z1 + 0.01, self.h[layer], -np.inf)
            r = int(math.ceil(c))
            yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
            h = maximum_filter(h, footprint=(xx * xx + yy * yy) <= c * c + 0.25, mode='constant', cval=-np.inf)
            out = np.maximum(out, h + c)
        self.req = out
        return self

    def required(self, pts):
        """the lowest z a fin's edge may take over these plan points"""
        ij, ok = self.cells(pts)
        return float(self.req[ij[:, 0], ij[:, 1]].max()) if len(ij) else -np.inf


# ---- the fins --------------------------------------------------------------------------------------------------------------------------------
def lower_edge(kind, side, hmap, deck_free):
    """per u (1 mm steps over the fin's length): the z its lower edge may come down to, and whether the deck can carry a foot there"""
    u0, u1 = FIN_DIMS[kind]['top']
    w_up, w_deck = FIN_LEG[kind]
    hw = max(w_up, w_deck)
    lc, d, _ = fin_axis(kind, side)
    nv = np.array([-d[1], d[0]])
    us = np.arange(u0, u1 + 1e-6, 1.0)
    vs = np.arange(-hw, hw + 1e-6, 0.5)
    req, ok = [], []
    free = deck_free.buffer(-0.5)
    for u in us:
        pts = [lc + d * (u + du) + nv * v for du in (-0.5, 0.0, 0.5) for v in vs]
        req.append(hmap.required(pts))
        strip = Polygon([tuple(lc + d * uu + nv * vv) for uu, vv in ((u - 0.5, -w_deck - 1), (u + 0.5, -w_deck - 1), (u + 0.5, w_deck + 1), (u - 0.5, w_deck + 1))])
        ok.append(free.contains(strip))
    req = np.array(req)
    zl = np.where(req > Z1, req, np.where(ok, Z1, Z1 + NOFOOT))
    return us, zl, np.array(ok)


def fin_profile(kind, side, hmap, deck_free):
    """the fin's outline in its plane (u, z): flange to deck, less the forbidden region under the generated lower edge (rounded R_EDGE), less
    the lightening arches in long feet. Returns (profile, report)"""
    u0, u1 = FIN_DIMS[kind]['top']
    us, zl, ok = lower_edge(kind, side, hmap, deck_free)
    outline = sbox(u0, Z1, u1, FLOOR2_UNDER)
    forbid = union_all([sbox(u - 0.5, Z1 - 5.0, u + 0.5, z) for u, z in zip(us, zl) if z > Z1 + 1e-6] or [Polygon()])
    prof = outline.difference(forbid.buffer(R_EDGE, 16)) if not forbid.is_empty else outline
    feet = contact_runs(prof)
    arches = []
    for a, c in feet:                                  # lightening arches in the long feet
        L_ = c - a
        if L_ < 2 * ARCH['foot'] + ARCH['span_min']:
            continue
        n = max(1, int(round((L_ - ARCH['foot']) / (ARCH['span_max'] + ARCH['foot']))))
        span = (L_ - (n + 1) * ARCH['foot']) / n
        while span > ARCH['span_max'] and n < 20:
            n += 1
            span = (L_ - (n + 1) * ARCH['foot']) / n
        if span < ARCH['span_min']:
            continue
        for k in range(n):
            s0 = a + ARCH['foot'] + k * (span + ARCH['foot'])
            h = min(ARCH['h'], 0.6 * span)
            pts = [(s0 + span / 2 + span / 2 * math.cos(t), Z1 + h * math.sin(t)) for t in np.linspace(0, math.pi, 25)]
            arches.append(Polygon(pts + [(s0, Z1 - 1.0), (s0 + span, Z1 - 1.0)]).buffer(0))
    if arches:
        prof = prof.difference(union_all(arches))
    if 'web_from' in FIN_DIMS[kind]:                  # no web in the scan band nearer the lidar than web_from (the flange stays)
        prof = prof.difference(sbox(u0 - 1.0, SUPPORT_PLANE_BAND[0] - 8.0, FIN_DIMS[kind]['web_from'], SUPPORT_PLANE_BAND[1] + 2.5))
    prof = prof.buffer(-1.5, 16).buffer(1.5, 16)       # nothing thinner than 3 mm in the plane, outside corners R1.5
    if prof.geom_type == 'MultiPolygon':
        prof = max(prof.geoms, key=lambda q: q.area)
    rep = dict(edge_min_z=round(float(zl.min()), 1), edge_max_z=round(float(zl.max()), 1), deck_ok_mm=int(ok.sum()), length_mm=int(len(us)),
               feet=[[round(a, 1), round(c, 1)] for a, c in contact_runs(prof)], arches=len(arches))
    return prof, rep


def contact_runs(prof):
    """the u intervals where the profile stands on the deck (z Z1)"""
    band = prof.intersection(sbox(-1e3, Z1 - 0.1, 1e3, Z1 + 0.3))
    runs = sorted((g.bounds[0], g.bounds[2]) for g in getattr(band, 'geoms', [band]) if not g.is_empty and g.area > 0.01)
    return [(a, c) for a, c in runs if c - a >= FOOT_MIN]


def foot_screws(kind, side, prof, screw_ok):
    """screw stations along the feet: one in a short foot, one near each end of a longer one and every SCREW_PITCH between; each moved along
    the foot (up to +-6) to the nearest place screw_ok allows; plan (x, y) and u"""
    out = []
    for a, c in contact_runs(prof):
        L_ = c - a
        if L_ < FOOT_TWO:
            want = [(a + c) / 2]
        elif L_ < 2 * SCREW_PITCH:                     # Nov 15 evening (Fernando: two per front fin, for its alignment with the lidar): a
            want = [a + 6.0, c - 6.0]                  # foot this long gets a screw near each end
        else:
            n = int(L_ // SCREW_PITCH)
            want = list(np.linspace(a + 9.0, c - 9.0, n))
        for u in want:
            for du in (0, 1.5, -1.5, 3, -3, 4.5, -4.5, 6, -6):
                uu = u + du
                if uu < a + 5.0 or uu > c - 5.0:
                    continue
                xy = fin_xy(kind, uu, side)
                if screw_ok(float(xy[0]), float(xy[1])):
                    out.append((float(uu), (float(xy[0]), float(xy[1]))))
                    break
    return out


def build_fin(kind, side, hmap, deck_free, screw_ok):
    """the fin solid (robot frame), its foot screws' plan points, its section at the scan plane (the lidar's shadow) and a report"""
    prof, rep = fin_profile(kind, side, hmap, deck_free)
    S = FIN_DIMS[kind]
    w = FIN_FLANGE / 2 + 1.0
    s = B.solid_of(prof, -w, w).rotate(b.Axis.X, 90)                                   # (u, z, across) -> (u, -across, z)
    u0, u1 = S['top']
    zp, (b0, b1) = LIDAR['plane'], SUPPORT_PLANE_BAND
    band = prof.intersection(sbox(-1e3, zp - 0.5, 1e3, zp + 0.5)).bounds
    cone = wedge(band[0], S.get('web', FIN_WEB), u1 + 2.0)
    s = s & (B.solid_of(fin_cross_section(kind), u0 - 2.0, u1 + 2.0).rotate(b.Axis.X, 90).rotate(b.Axis.Z, 90) + B.solid_of(cone, b0 - 8.0, b1 + 2.5))
    lc, d, bearing = fin_axis(kind, side)
    s = s.rotate(b.Axis.Z, bearing).moved(b.Location((*lc, 0)))
    screws = foot_screws(kind, side, prof, screw_ok)
    for _, (x, y) in screws:                                                           # the feet: M3 up through the deck into Ø2.8 pilots
        s = s - cyl(M3_PILOT / 2, FIN_PILOT_DEPTH, (x, y, Z1 + FIN_PILOT_DEPTH / 2 - 0.05))
    for x, y in floor_pts(kind, side):                                                 # the floor: M3 x 12 down into the flange
        s = s - cyl(M3_PILOT / 2, FLOOR_PILOT_DEPTH, (x, y, FLOOR2_UNDER - FLOOR_PILOT_DEPTH / 2 + 0.05))
    ties = 0
    for u in TIE_FIN_U.get(kind, ()):                                                  # zip-tie slots through the plate over its edge
        if any(abs(u - us) < TIE_SCREW_KEEP for us, _ in screws):                     # Nov 14 audit: a slot across a foot's pilot halved it
            continue
        col = prof.intersection(sbox(u - 0.01, -1e3, u + 0.01, 1e3))
        if col.is_empty:
            continue
        zc = col.bounds[1] + TIE_FIN_ABOVE
        if zc + TIE_SLOT_W / 2 > b0 - 8.0 or col.bounds[1] > Z1 + 0.5 and zc - TIE_SLOT_W / 2 < col.bounds[1] + 2.0:
            continue
        cx, cy = fin_xy(kind, u, side)
        cut = box(-TIE_SLOT_L / 2, TIE_SLOT_L / 2, -FIN_FLANGE, FIN_FLANGE, -TIE_SLOT_W / 2, TIE_SLOT_W / 2)
        s = s - cut.rotate(b.Axis.Z, bearing).moved(b.Location((cx, cy, zc)))
        ties += 1
    sec = cone.intersection(sbox(band[0], -FIN_FLANGE, band[2], FIN_FLANGE))           # the wedge at the scan plane
    sec = affinity.translate(affinity.rotate(sec, bearing, origin=(0, 0)), *lc)
    rep.update(screws=len(screws), screw_u=[round(u, 1) for u, _ in screws], tie_slots=ties)
    return s, [xy for _, xy in screws], sec, rep


def build_fins(hmap, deck_free, screw_ok):
    """{name: solid}, the feet's screw points, the scan-plane sections, the report"""
    parts, feet, sections, rep = {}, [], [], {}
    for name, kind, side in FINS:
        s, pts, sec, r = build_fin(kind, side, hmap, deck_free, screw_ok)
        parts[name] = s
        feet += pts
        sections.append((name, sec))
        rep[name] = r
    return parts, feet, sections, rep


def flange_plans():
    """plan polygons of the fins' flanges under the floor (upper.py)"""
    out = []
    for _, kind, side in FINS:
        u0, u1 = FIN_DIMS[kind]['top']
        a, d = fin_xy(kind, u0, side), fin_xy(kind, u1, side)
        n = np.array([-(d - a)[1], (d - a)[0]]) / np.linalg.norm(d - a) * FIN_FLANGE / 2
        out.append(Polygon([a + n, d + n, d - n, a - n]))
    return union_all(out)

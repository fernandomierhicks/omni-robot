"""cad/v15 MID-DECK base (PROPOSAL, Nov 14 2026, Fernando): the floor deck at z 80..100, above the pods' pivots (z 70) and under their
spring towers; the pods (pod_mid.py) hang under it, bolted up from below; the towers, the fins (fins.py), the lidar pedestal and the bumper
hang on it; the battery hangs under its centre. build123d + shapely, STEP review only; nothing printed.

    C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe -B chassis_mid.py [study|build|check|render|export|all]

 * Layout F (base.py): rolling lines tangent to r 154, wheels slid 20 mm (v14's wheel positions: kinematics, front gap, arm reach unchanged).
   Robot frame: origin on the floor under the centre, x forward, y left, z up.
 * The deck: a disc 20 mm deep in four quadrant pieces (one pod each; each fits the 256 bed), joined by dovetail half-laps across the seams;
   less each pod's opening (everything of the pod above the deck's underside, every pose, + pod_mid.CLR) and lightening pockets where nothing
   bears. The seams cross at SEAM_O, off the centre: the spine fin runs along y 0 and its screws must not land on a seam.
 * The fins (fins.py): their lower edges generated from a height map of everything on the deck; feet screwed up from below.
 * The battery (Nov 14 review): under the deck's centre, along x, its top against the deck's underside: four printed corner clips (an M3 up
   into the deck each) locate it, two straps through slots in the deck hold it up; the XT90 end toward the rear, the mated pair under the
   deck behind it, the board's lead up through LEAD_HOLE. Out: strap open, XT90 unplugged, the pack drops out downward (the pods' way).
 * Service stand (Nov 14 review): four pads on the deck's underside where nothing hangs or drops (STAND), each with a Ø8.4 socket for a peg;
   the stand height that lets a pod drop clear under the bumper is in the checks.
 * The bumper: four 3 mm quarters (z 40..80) hung on tabs under the deck's rim (an M3 up into the deck each).
"""
import json
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
import fins as F                                                       # noqa: E402
import pod_mid as PM                                                   # noqa: E402
import pod_rx as R                                                     # noqa: E402
import hardware as H                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import M3_CLR, M3_PILOT, R_WHEEL                             # noqa: E402
import palette as PAL                                                  # noqa: E402
import print_beds as PB                                                # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

VAL = W.ROOT / 'validation' / 'chassis_mid'
OUT = W.ROOT / 'step_export' / 'chassis_mid'
Z0, Z1 = PM.DECK_Z[0] + R_WHEEL, PM.DECK_Z[1] + R_WHEEL               # 80, 100
assert (Z0, Z1) == (F.Z0, F.Z1), 'fins.Z0 / Z1 must be the deck (pod_mid.DECK_Z + R_WHEEL)'
ZL = (Z0 + Z1) / 2                 # the half-lap line
CLR = PM.CLR
JOINT_GAP, SEAM_KEEP = 0.0, 22.0   # quadrant seams; no pocket within SEAM_KEEP of a seam (the laps go there). Nov 15 evening (Fernando): the
                                   # pieces meet along the seams (was a 0.2 gap), so their walls bear on each other and stiffen the deck; only
                                   # the dovetail sockets keep a clearance (TAB['clr'])
SEAM_O = (6.0, -10.0)              # the seams cross here, not at the centre: the spine fin (y 0) and the side fins' screws stay off them
SEAM_SCREW_KEEP = 7.0              # no screw this close to a seam line (its counterbore would straddle two pieces)
# the joints: dovetail half-laps across each seam. On every seam one piece is the LOWER one: its bottom half runs across the seam as a
# dovetail tab into a socket in the other piece's bottom half, whose top half covers it; 2 x M3 x 16 down through the top half into the tab.
# FL and RR are the lower pieces on all their seams, RL and FR drop onto them: the deck goes together from above
SEAMS = [(90.0, 'FL', 'RL'), (180.0, 'RR', 'RL'), (270.0, 'RR', 'FR'), (0.0, 'FL', 'FR')]    # (seam angle, lower piece, upper piece)
LAP_R = (45.0, 100.0, 155.0, 205.0)   # nominal radii of the laps on each seam (moved up to +-30 to land on solid deck). Nov 15 evening: 4 per seam
                                   # where the deck has room (was 3), the tabs 20 deep (was 14; 13 since Nov 16, the bed); 2 screws each, a flush hex recess for an optional
                                   # nut under each (the Ø2.8 pilot stays, so the screw still threads without one)
TAB = dict(neck=8.0, flare=11.0, depth=13.0, root=1.0, clr=0.05, r=0.5)   # half widths at the seam / at the far end, depth past the seam.
                                   # clr: the socket's clearance per face; Nov 17 (Fernando): 0.05 (was 0.1), the best fit on the printed
                                   # dovetail coupon (coupon_dovetail.py). r: Nov 17 (Fernando: a very small fillet on the outside sharp
                                   # corners so they slide in better): R0.5 on the tab's far corners and on the socket's two lips at the seam
                                   # Nov 16: depth 20 -> 13 so deck_FL and deck_RR (the lower pieces on both their seams: tabs past two
                                   # edges) fit the 256 bed with its 2 mm margin (FL was 243 x 258.6); 4 laps now fit on every seam
LAP_GAP = 6.0                      # two tabs on one seam at least this far apart
# Nov 15 evening (Fernando, "B"): a flush four-arm key over the seams' crossing, set 6 mm into the deck's top: its arms run along the diagonals
# into the four pieces and flare like dovetails, so the pieces cannot pull apart at the crossing; one M3 x 16 down through each arm into its piece
KEY = dict(depth=6.0, r0=6.0, r1=22.0, neck=4.0, flare=7.0, hub_r=8.0, screw_r=14.0, clr=0.05, keep=3.0)   # clr as TAB's (Nov 17, the coupon)
# Nov 15 evening (Fernando, "A"): no lightening pockets (at 15 % infill a closed deck costs little and is far stiffer); a few larger openings for
# access, zip ties and leads instead: up to WINDOW['n'] rounded slots per piece where the old pockets could go
WINDOW = dict(w=25.0, l=50.0, n=3, sep=55.0, step=3.0)
LAP_SEAT_KEEP = 1.0                # Nov 15: a tab (+ its clearance) this far off every pod's seat under the deck (the beams' and hangers' tops)
DECK_MARK = dict(size=16.0, keep=5.0)   # Nov 15 (Fernando): each deck piece's number (1 FL, 2 FR, 3 RL, 4 RR) engraved in its top, reading with the
                                   # robot's front up; this far from any hole, pocket, seat or edge
LAP_SCREW_L, LAP_SCREW_DU = 16.0, 4.5
RIM_KEEP, OPEN_KEEP, SCREW_KEEP, SEAT_KEEP = 12.0, 10.0, 8.0, 3.0      # solid deck kept: rim band, round each opening, round screws, round seats
HUB_KEEP, POCKET_MIN, POCKET_R, FILLET = 30.0, 400.0, 6.0, 8.0
FOOT_KEEP = 3.0                    # solid deck round each fin's foot
SLIVER_W = 14.0                    # Nov 14 (Fernando): strips narrower than this held at one end only (cantilevers that add no stiffness) go
BUMPER_T, BUMPER_Z = 3.0, (40.0, 80.0)   # Nov 14 (Fernando): its top at the deck's underside, tabs under it
# the bumper's attachment: tabs from the quarter's top inward under the deck's rim, each with an M3 x 16 pan head up through it (Ø3.4,
# counterbored in its underside) into a Ø2.8 pilot through the deck. A 45 deg gusset under each tab; the quarter prints upside down
BTAB = dict(r_in=216.0, w=14.0, t=6.0, gusset=8.0, screw_r=222.5, screw_L=16.0, per_quarter=3, end_keep=4.0, min_sep=15.0, step=0.5)
BTAB_CLR_MOVING, BTAB_CLR_STATIC = 3.0, 1.5      # plan clearance of a tab (and its gusset) to the pods' swept moving parts / their static parts
CB_D = 7.2
PED_PAD_R = F.PED_FLANGE_R + 6.0
QUAD = {'FL': (0.0, 90.0), 'RL': (90.0, 180.0), 'RR': (180.0, 270.0), 'FR': (270.0, 360.0)}   # the bumper quarters (angles)
# ---- the battery under the deck (Nov 14 review, Fernando: "move it down there") ------------------------------------------------------------
# Nov 16 (Fernando): the HRB 6S 3300 mAh (components/battery: 136 x 46 x 43, 558 g) replaces the OVONIC Roam 6S 5200 (157 x 47 x 46, 706 g).
# Same centre, so the straps keep their places on the deck (29 / 35.5 mm in from the pack's ends); laid with its 43 mm side vertical (z 37-80).
# Along x on the centreline: under the deck the pods leave a corridor 55 mm wide from x -135 to +135 (3 mm off every pod part, every pose).
# The XT90 end toward the rear (the HRB's connector VERIFY: XT90 ASSUMED): the mated pair behind the pack under the deck, the board's lead up
# through LEAD_HOLE beside the spine (forward, it would come up under the lidar's pedestal).
BATTERY = dict(size=(136.0, 46.0, 43.0), xy=(11.5, 0.0), mass=0.558,
               clip=dict(wall=2.5, h=30.0, arm=14.0, tab=3.0, tab_reach=11.0, screw_l=10.0),   # four corner Ls hung from the deck, tab out
               strap_u=(-39.25, 32.5), strap=(20.0, 1.5), slot=(22.0, 3.5),                    # two straps (Nov 15 evening: the front one moved
                                                                                                # back 6.75, off the seam-0 dovetail and the spine's nose)
               xt90=dict(x=(-118.0, -72.0), y=(-10.5, 10.5), z=(51.5, 62.5)), pigtail_r=4.5)  # the mated pair (harness.CONN XT90, ASSUMED)
LEAD_HOLE = (-128.0, -17.0, 12.0)  # (x, y, Ø): the board's 12 AWG XT90 lead up through the deck (rounded edges; no grommet)
# ---- service stand (Nov 14 review, Fernando: "features for a future service stand") ---------------------------------------------------------
STAND = dict(pad_r=12.0, socket_d=8.4, socket_h=Z1 - Z0, clr=5.0, dirs=(0.0, 90.0, 180.0, 270.0), spread=25.0, r=(110.0, 214.0))
# a pad per direction: the outermost point within +-spread deg of it where a Ø24 pad is solid deck (no pocket, hole or seam), 5 mm off every
# pod's drop-out column (everything of it, every pose), the battery's and the bumper's tabs' zone. A Ø8 peg in each socket locates the robot;
# the socket runs through the deck (cad/solidworks.md 18: every hole through its part).


def U(a):
    return np.array([math.cos(math.radians(a)), math.sin(math.radians(a))])


def sector(a0, a1, r):
    return Polygon([(0, 0)] + [tuple(r * U(t)) for t in np.linspace(a0, a1, 64)])


def rmax(g):
    return max(math.hypot(*c) for q in getattr(g, 'geoms', [g]) for c in q.exterior.coords)


def piece_box(n, big=600.0):
    ox, oy = SEAM_O
    return dict(FL=sbox(ox, oy, big, big), RL=sbox(-big, oy, ox, big), RR=sbox(-big, -big, ox, oy), FR=sbox(ox, -big, big, oy))[n]


COLOURS = dict(deck=(0.86, 0.86, 0.82), bumper=(0.90, 0.45, 0.15))


def colour(name):
    """display / STEP colour of a base part (the printed parts: their filament)"""
    n = name.split('/')[-1]
    if n.startswith('bumper'):
        return PAL.ORANGE
    if n.startswith(('bolt', 'nut')):
        return H.MAGENTA
    if n.startswith('battery'):
        return (0.85, 0.25, 0.25)
    if n.startswith('strap'):
        return (0.15, 0.15, 0.15)
    return PAL.colour(n) or PAL.WHITE


_POD = None


def pod_build():
    """the pod (pod frame) and its footprints, once"""
    global _POD
    if _POD is None:
        parts, rm, rs, _ = PM.build()
        unit = R.wheel_unit()
        R.wheel_envelope(unit)                                         # primes the envelope footprints() uses when its cache is stale
        fp = PM.footprints(parts, rm, rs, unit)
        _POD = (parts, rm, rs, unit, fp)
    return _POD


def pods_placed(a=0.0, labelled=False):
    """robot coordinates at fork angle a: {pod: (static dict, moving dict)}; the towers and their screws count as the deck's.
    labelled=True: each pod's big parts carry its number and IN / OUT (pod_mid.labelled; for the STEPs and SolidWorks, not needed by checks)"""
    parts, rm, rs, unit, _ = pod_build()
    out = {}
    for i, n in enumerate(B.NAMES):
        sh, moving, _ = PM.assembly(PM.labelled(parts, i) if labelled else parts, rm, rs, unit, a, deck=False)
        st = {k: B.place3d(v, i) for k, v in sh.items() if k not in moving}
        mv = {k: B.place3d(v, i) for k, v in sh.items() if k in moving}
        out[n] = (st, mv)
    return out


# ---- the battery (plan and solids) ---------------------------------------------------------------------------------------------------------
def battery_plan(grow=0.0):
    (l, w, _), (cx, cy) = BATTERY['size'], BATTERY['xy']
    return sbox(cx - l / 2 - grow, cy - w / 2 - grow, cx + l / 2 + grow, cy + w / 2 + grow)


def battery_zone():
    """everything of the battery's that hangs under the deck (pack, clips and their tabs, straps, XT90 pair, the lead to the hole), plan"""
    (l, w, _), (cx, cy) = BATTERY['size'], BATTERY['xy']
    c, x, (sw, st) = BATTERY['clip'], BATTERY['xt90'], BATTERY['strap']
    hx, hy, hd = LEAD_HOLE
    reach = c['wall'] + c['tab_reach']                                 # the clips' tabs reach out along x, their walls 2.5 round the pack
    pack = sbox(cx - l / 2 - reach, cy - w / 2 - c['wall'], cx + l / 2 + reach, cy + w / 2 + c['wall'])
    straps = union_all([sbox(cx + u - sw / 2, cy - w / 2 - 0.25 - st, cx + u + sw / 2, cy + w / 2 + 0.25 + st) for u in BATTERY['strap_u']])
    lead = Polygon([(x['x'][0], -BATTERY['pigtail_r']), (x['x'][0], BATTERY['pigtail_r']), (hx, hy + hd / 2), (hx, hy - hd / 2)]).convex_hull
    return union_all([pack, straps, sbox(x['x'][0], x['y'][0], x['x'][1], x['y'][1]), lead, Point(hx, hy).buffer(hd / 2)])


CORNERS = ((1, 1), (-1, 1), (-1, -1), (1, -1))


def clip_screw(sx, sy):
    """a clip's M3 (plan): in its tab beyond the end wall (along x), over the side wall's half"""
    (l, w, _), (cx, cy) = BATTERY['size'], BATTERY['xy']
    c = BATTERY['clip']
    return (cx + sx * (l / 2 + c['wall'] + c['tab_reach'] / 2), cy + sy * (w / 2 + c['wall'] - (c['arm'] + c['wall']) / 2))


def battery_clip_screws():
    return [clip_screw(sx, sy) for sx, sy in CORNERS]


def strap_slots():
    """the straps' slots through the deck (plan boxes): two per strap, right beside the pack's sides"""
    (l, w, _), (cx, cy) = BATTERY['size'], BATTERY['xy']
    sl, st = BATTERY['slot']
    return [sbox(cx + u - sl / 2, cy + sy * (w / 2 + 0.25), cx + u + sl / 2, cy + sy * (w / 2 + 0.25 + st)) if sy > 0 else
            sbox(cx + u - sl / 2, cy - w / 2 - 0.25 - st, cx + u + sl / 2, cy - w / 2 - 0.25)
            for u in BATTERY['strap_u'] for sy in (1, -1)]


def battery_parts():
    """printed: the four corner clips; bought (envelopes): the pack, its two straps; hardware: the clips' screws"""
    (l, w, h), (cx, cy) = BATTERY['size'], BATTERY['xy']
    c = BATTERY['clip']
    zt, zb = Z0, Z0 - h
    printed, bought, hw = {}, {}, {}
    bought['battery'] = P_box(cx - l / 2, cx + l / 2, cy - w / 2, cy + w / 2, zb, zt)
    for k, (sx, sy) in enumerate(CORNERS):                             # an L round each corner, 30 tall, hung from its tab (printed tab down)
        x_end, y_side = cx + sx * l / 2, cy + sy * w / 2
        z_low = zt - c['h']
        end = P_box(*sorted((x_end, x_end + sx * c['wall'])), *sorted((y_side + sy * c['wall'], y_side - sy * c['arm'])), z_low, zt)   # the end wall
        side = P_box(*sorted((x_end + sx * c['wall'], x_end - sx * c['arm'])), *sorted((y_side, y_side + sy * c['wall'])), z_low, zt)  # the side wall
        tab = P_box(*sorted((x_end + sx * c['wall'], x_end + sx * (c['wall'] + c['tab_reach']))),
                    *sorted((y_side + sy * c['wall'], y_side - sy * c['arm'])), zt - c['tab'], zt)
        px, py = clip_screw(sx, sy)
        clip = (end + side + tab).clean() - W.cyl(M3_CLR / 2, c['tab'] + 0.4, (px, py, zt - c['tab'] / 2))
        hw[f'bolt_battery_clip_{k}'] = H.pan_screw((px, py, zt - c['tab']), (0, 0, 1), c['screw_l'])
        printed[f'bracket_battery_clip_{k}'] = clip
    sw, stt = BATTERY['strap']
    for j, u in enumerate(BATTERY['strap_u']):
        x0, x1 = cx + u - sw / 2, cx + u + sw / 2
        loop = (P_box(x0, x1, cy - w / 2 - 0.25 - stt, cy + w / 2 + 0.25 + stt, zb - stt, Z1 + stt)
                - P_box(x0 - 1, x1 + 1, cy - w / 2 - 0.25, cy + w / 2 + 0.25, zb, Z1))
        bought[f'strap_battery_{j}'] = loop
    return printed, bought, hw


def P_box(x0, x1, y0, y1, z0, z1):
    return b.Solid.make_box(x1 - x0, y1 - y0, z1 - z0).moved(b.Location((x0, y0, z0)))


# ---- the height map the fins are generated from -----------------------------------------------------------------------------------------------
STAYS = ('rx_tower', 'bolt_tower', 'nut_tower', 'bolt_towerfix')     # a pod's parts that stand on the deck (static)
WHEEL_PARTS = ('roller_', 'bearing_687', 'wheel_', 'bolt_1', 'bolt_2', 'bolt_cart', 'tread')   # inside the wheel's revolved envelope


def height_map():
    """fins.HeightMap of the pods (moving parts in five poses, droop .. bump, with the wheel's envelope; plungers and springs; the towers),
    the pedestal, the lidar and the battery straps over the deck. Cached against the sources"""
    import pickle
    srcs = [TOOLS / n for n in ('fins.py', 'pod_mid.py', 'pod_p4.py', 'pod_rx.py', 'pod.py', 'pod_lean.py', 'wheel_motor.py', 'base.py', 'dims.py')]
    key = [round(s.stat().st_mtime) for s in srcs] + [repr(BATTERY), repr(WHEEL_PARTS), repr(STAYS)]
    cache = VAL / 'hmap.pkl'
    if cache.exists():
        k, hm = pickle.loads(cache.read_bytes())
        if k == key:
            return hm
    hm = F.HeightMap()
    env = R.wheel_envelope(None)
    poses = [R.A_DROOP, 0.0, R.A_BUMP / 3, 2 * R.A_BUMP / 3, R.A_BUMP]
    for a in poses:
        for i, (n, (st, mv)) in enumerate(pods_placed(a).items()):
            hm.add_wheel(i, a, env)
            for k, s in mv.items():
                if not k.startswith(WHEEL_PARTS):
                    hm.add_shape(s, 'moving')
            for k, s in st.items():
                if k.startswith(('plunger', 'spring')):
                    hm.add_shape(s, 'moving')
                elif a == 0.0 and k.startswith(STAYS):
                    hm.add_shape(s, 'static')
    hm.add_shape(F.build_pedestal(), 'static')
    hm.add_shape(F.lidar_env(), 'static')
    for slot in strap_slots():                                         # the straps over the deck between their slots
        x0, _, x1, _ = slot.bounds
        hm.add_box(x0, x1, BATTERY['xy'][1] - BATTERY['size'][1] / 2 - 4.0, BATTERY['xy'][1] + BATTERY['size'][1] / 2 + 4.0, Z1 + BATTERY['strap'][1] + 0.5, 'static')
    hm.finish()
    hm.h = None                                                        # keep only the required-edge map
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, hm)))
    return hm


# ---- the deck (plan) -------------------------------------------------------------------------------------------------------------------------
def outline(fp):
    """the deck's disc and the pods' openings (no pockets yet)"""
    openings = union_all([B.place2d(PM.opening(fp), i) for i in range(4)])
    hold = union_all([B.place2d(union_all([fp['pod_static'], fp['towers']]), i) for i in range(4)])
    r_hold, r_open = rmax(hold), rmax(openings)
    r_deck = math.ceil(max(r_hold + 1.5, r_open + 6.0))
    return dict(openings=openings, r_deck=r_deck, r_hold=round(r_hold, 1), r_open=round(r_open, 1), disc=Point(0, 0).buffer(r_deck, 256))


def pod_screw_points(fp):
    place = lambda pts: [tuple(B.place2d(Point(*p), i).coords[0]) for i in range(4) for p in pts]   # noqa: E731
    return place(fp['up_screws']), place(fp['tower_screws']), place(fp['tower_fix'])


def stand_pads(o, fp):
    """STAND: one pad per direction (see STAND), robot (x, y)"""
    S = STAND
    pods = union_all([B.place2d(union_all([fp['moving'], fp['pod_static']]), i) for i in range(4)]).buffer(S['clr'])
    bad = union_all([pods, o['openings'].buffer(S['clr']), battery_zone().buffer(S['clr']),
                     Point(0, 0).buffer(o['r_deck'] + 1).difference(Point(0, 0).buffer(BTAB['r_in'] - S['clr'])),
                     sbox(SEAM_O[0] - S['pad_r'] - 2, -500, SEAM_O[0] + S['pad_r'] + 2, 500), sbox(-500, SEAM_O[1] - S['pad_r'] - 2, 500, SEAM_O[1] + S['pad_r'] + 2)])
    ok_disc = o['disc']
    out = []
    for d in S['dirs']:
        best = None
        for r in np.arange(S['r'][1], S['r'][0], -2.0):
            for da in np.arange(0.0, S['spread'] + 0.1, 1.0):
                for sgn in ((1,) if da == 0 else (1, -1)):
                    x, y = r * U(d + sgn * da)
                    c = Point(x, y).buffer(S['pad_r'])
                    if ok_disc.contains(c) and not c.intersects(bad):
                        best = (float(x), float(y))
                        break
                if best:
                    break
            if best:
                break
        if best:
            out.append(best)
    return out


def layout(fp):
    o = outline(fp)
    openings, r_deck, disc = o['openings'], o['r_deck'], o['disc']
    up, tw, fix = pod_screw_points(fp)
    pads = stand_pads(o, fp)
    hx, hy, hd = LEAD_HOLE
    lead = Point(hx, hy).buffer(hd / 2)
    bzone = battery_zone()
    deck_free = disc.difference(openings).difference(lead.buffer(2.0))
    others = up + tw + fix + F.pedestal_screws() + battery_clip_screws()
    slots = union_all(strap_slots())
    # Nov 15: no screw head from below over a pod's alignment boss (its counterbore would meet the boss's pocket; one over the rest of a beam is
    # an order of work, as before: the pod comes out first)
    under_pods = union_all([B.place2d(fp['align_bottom'], i) for i in range(4)]).buffer(PM.ALIGN_CLR + CB_D / 2 + 1.0)
    key_zone = key_plan(KEY['clr'] + KEY['keep'])                      # Nov 15 evening: the centre key (nothing else in its recess)
    key_edge = key_plan().boundary

    def screw_ok(x, y):
        p = Point(x, y)
        if not deck_free.buffer(-5.0).contains(p) or slots.buffer(5.0).contains(p) or abs(x - SEAM_O[0]) < SEAM_SCREW_KEEP or abs(y - SEAM_O[1]) < SEAM_SCREW_KEEP:
            return False                                               # (over the battery is fine: the heads sit recessed in counterbores)
        if under_pods.contains(p) or any(p.distance(Point(q)) < 9.0 for q in key_screws()) or key_edge.distance(p) < M3_CLR / 2 + 1.5:
            return False                                               # (through the key, not across its edge: no half hole)
        if any(p.distance(Point(q)) < 9.0 for q in others) or any(p.distance(Point(q)) < STAND['pad_r'] + 6.0 for q in pads):
            return False
        return math.hypot(x, y) < BTAB['r_in'] - 6.0                   # inside the bumper tabs' band
    fins, feet, sections, fin_rep = F.build_fins(height_map(), deck_free, screw_ok)
    foot_plans = union_all([g for g in (B.proj(s, True, Z1 - 0.01, Z1 + 1.0) for s in fins.values()) if g is not None])
    seats = union_all([B.place2d(union_all([fp['seats'], fp['towers']]), i) for i in range(4)])
    keep = union_all([openings.buffer(OPEN_KEEP), disc.difference(Point(0, 0).buffer(r_deck - RIM_KEEP, 256)),
                      sbox(-r_deck, SEAM_O[1] - SEAM_KEEP, r_deck, SEAM_O[1] + SEAM_KEEP), sbox(SEAM_O[0] - SEAM_KEEP, -r_deck, SEAM_O[0] + SEAM_KEEP, r_deck),
                      Point(*SEAM_O).buffer(HUB_KEEP), seats.buffer(SEAT_KEEP), Point(F.LIDAR_X, 0).buffer(PED_PAD_R),
                      foot_plans.buffer(FOOT_KEEP), battery_zone().buffer(4.0), slots.buffer(6.0),
                      lead.buffer(6.0)] + [Point(p).buffer(STAND['pad_r'] + 4.0) for p in pads]
                     + [Point(p).buffer(SCREW_KEEP) for p in up + tw + fix + feet])
    keep = keep.union(key_zone)
    room = disc.difference(keep).buffer(-POCKET_R, 32).buffer(POCKET_R, 32)   # where the old lightening pockets went
    pockets = access_windows(room)                                     # Nov 15 evening: a few access windows there instead
    decks, pruned = {}, {}
    protect = union_all([seats.buffer(SEAT_KEEP), foot_plans, slots.buffer(3.0), lead.buffer(3.0)]
                        + [Point(p).buffer(SCREW_KEEP) for p in up + tw + fix + feet + F.pedestal_screws() + battery_clip_screws()]
                        + [Point(p).buffer(STAND['pad_r']) for p in pads])
    for n in B.NAMES:
        sec = piece_box(n).buffer(-JOINT_GAP / 2, join_style=2)
        g = disc.intersection(sec).difference(openings).difference(pockets)
        g = g.buffer(FILLET, 32).buffer(-FILLET, 32).intersection(sec).intersection(disc).difference(openings)
        g = g.buffer(-1.5).buffer(1.5)                                 # nothing under 3 mm
        if g.geom_type == 'MultiPolygon':
            g = max(g.geoms, key=lambda q: q.area)
        g, cut = prune_cantilevers(g, protect, sec)
        pruned[n] = cut
        decks[n] = g
    avoid = up + tw + fix + feet + F.pedestal_screws() + battery_clip_screws() + pads + [(hx, hy)]
    pod_seats = union_all([B.place2d(fp['seats'], i) for i in range(4)])   # Nov 15: no tab over a pod's seat (FR's beam lay under RR's tab)
    laps, lower = make_laps(decks, avoid + key_screws(), union_all([slots, foot_plans]),
                            no_tab=union_all([pod_seats.buffer(LAP_SEAT_KEEP), slots.buffer(3.0), key_zone]))   # Nov 15 evening: + the strap slots,
                                                                       # the key (the slot beside the seam-0 dovetail, Fernando)
    return dict(decks=decks, lower=lower, laps=laps, openings=openings, pockets=pockets, pruned=pruned, r_deck=r_deck, r_hold=o['r_hold'],
                r_open=o['r_open'], up=up, tw=tw, fix=fix, feet=feet, fins=fins, sections=sections, fin_rep=fin_rep, foot_plans=foot_plans,
                pads=pads, deck_free=deck_free)


def key_plan(grow=0.0):
    """the centre key's plan (robot frame): a hub round the seams' crossing and four arms along the diagonals, each flaring from KEY['neck'] at
    r0 to KEY['flare'] at r1 (half widths) like a dovetail, so a piece cannot pull away from the crossing; grown by `grow`"""
    K, O = KEY, np.array(SEAM_O)
    parts = [Point(*O).buffer(K['hub_r'], 32)]
    for a in (45.0, 135.0, 225.0, 315.0):
        d, n = U(a), U(a + 90.0)
        parts.append(Polygon([tuple(O + d * K['r0'] + n * s * K['neck']) for s in (1, -1)] + [tuple(O + d * K['r1'] + n * s * K['flare']) for s in (-1, 1)]))
    g = union_all(parts).buffer(-1.0, 16, join_style=2).buffer(1.0, 16)  # outside corners R1
    return g.buffer(grow, 16, join_style=2) if grow else g


def key_screws():
    """one M3 down through each south arm into its piece (FR, RR). The north arms (FL, RL) lie under the spine fin's first foot, which holds
    them down; the fin's foot screws may pass up through the key (holes cut in it)"""
    return [tuple(np.array(SEAM_O) + U(a) * KEY['screw_r']) for a in (225.0, 315.0)]


def access_windows(room):
    """Nov 15 evening (Fernando: fewer mass-saving pockets, some larger openings for access, zip ties and leads): per deck piece up to
    WINDOW['n'] rounded slots (w x l) where the old pockets could go, at least WINDOW['sep'] apart, the first nearest the piece's middle"""
    W_ = WINDOW

    def slot(w, l):
        base = Point(0, 0).buffer(w / 2, 32)
        return union_all([affinity.translate(base, dx, 0) for dx in (-(l - w) / 2, (l - w) / 2)]).convex_hull
    sizes = [(W_['w'], W_['l']), (22.0, 40.0), (20.0, 32.0), (24.0, 24.0)]   # the largest that fits each place (the old pockets' rooms are
    out = []                                                           # roughly triangular, ~50 across)
    for n in B.NAMES:
        reg = room.intersection(piece_box(n).buffer(-SEAM_KEEP))
        if reg.is_empty:
            continue
        c = reg.centroid
        cands = []
        x0, y0, x1, y1 = reg.bounds
        for rank, (sw, sl) in enumerate(sizes):
            shapes = [affinity.rotate(slot(sw, sl), a, origin=(0, 0)) for a in ((0, 45, 90, 135) if sl > sw else (0,))]
            for x in np.arange(x0, x1, W_['step']):
                for y in np.arange(y0, y1, W_['step']):
                    for s in shapes:
                        w = affinity.translate(s, x, y)
                        if reg.contains(w):
                            cands.append((rank, math.hypot(x - c.x, y - c.y), (x, y), w))
        cands = [(d, xy, w) for _, d, xy, w in sorted(cands, key=lambda t: (t[0], t[1]))]
        got = []
        for _, xy, w in cands:
            if len(got) >= W_['n']:
                break
            if all(math.hypot(xy[0] - q[0], xy[1] - q[1]) >= W_['sep'] for q, _ in got):
                got.append((xy, w))
        out += [w for _, w in got]
    return union_all(out) if out else Polygon()


def prune_cantilevers(g, protect, sec, w=SLIVER_W):
    """remove the strips of g narrower than w that hang from the rest at one end only (they stiffen nothing), unless they carry a seat or a
    screw; the strips that join two wider regions (the rings round the openings, the rim) stay. Returns the plan and what was cut (mm2)"""
    cut = 0.0
    for _ in range(3):                                                 # a removed finger can leave another one behind
        core = g.buffer(-w / 2, 16).buffer(w / 2, 16).intersection(g)
        thin = g.difference(core.buffer(0.05))
        drop = []
        for c in getattr(thin, 'geoms', [thin]):
            if c.area < 1.0 or c.intersects(protect):
                continue
            touch = c.buffer(0.3).intersection(core)
            n_touch = len(getattr(touch, 'geoms', [touch])) if not touch.is_empty else 0
            if n_touch <= 1:
                drop.append(c)
        if not drop:
            break
        dd = union_all(drop)
        cut += dd.area
        g = g.difference(dd.buffer(0.1)).buffer(-0.5).buffer(0.5)
        if g.geom_type == 'MultiPolygon':
            g = max(g.geoms, key=lambda q: q.area)
    return g, round(cut, 1)


def round_convex(g, r, where=None):
    """g with its outside (convex) corners rounded to r: everywhere, or only inside the region `where` (Nov 17, the dovetails)"""
    if r <= 0:
        return g
    o = g.buffer(-r, 16, join_style=1).buffer(r, 16, join_style=1)
    return o if where is None else g.difference(where.difference(o))


def tab_poly(theta, side, u):
    """the dovetail tab at radius u on the seam at angle theta, reaching `side` (+-1: across the seam toward +-normal)"""
    T = TAB
    loc = Polygon([(u - T['neck'], -T['root']), (u + T['neck'], -T['root']), (u + T['flare'], T['depth']), (u - T['flare'], T['depth'])])
    loc = round_convex(loc, T['r'])                                    # Nov 17: R0.5 on its corners (the root's lie inside the lower piece)
    loc = affinity.scale(loc, 1.0, side, origin=(0, 0))
    return affinity.translate(affinity.rotate(loc, theta, origin=(0, 0)), *SEAM_O)   # u from the seams' crossing


def make_laps(decks, avoid, avoid_geom=None, no_tab=None):
    """place the laps (first radius near each nominal where the tab and a margin round it are solid deck on both sides, clear of other
    screws and of avoid_geom: the strap slots, the fins' feet over the lap screws' heads; the tab itself off no_tab: the pods' seats under the
    deck, Nov 15); returns the laps and each piece's bottom-half plan"""
    lower = dict(decks)
    laps = []
    for theta, lo, hi in SEAMS:
        nrm, O = U(theta + 90.0), np.array(SEAM_O)
        rp = decks[hi].representative_point()
        side = 1.0 if rp.distance(Point(*(O + nrm * 50))) < rp.distance(Point(*(O - nrm * 50))) else -1.0
        both = decks[lo].union(decks[hi]).buffer(max(JOINT_GAP, 0.02), join_style=2)   # (touching pieces leave a hairline in the union)
        for u0 in LAP_R:
            for du in (0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25, 30, -30):
                tab = tab_poly(theta, side, u0 + du)
                if not both.buffer(-3.0).contains(tab) or (no_tab is not None and tab.buffer(TAB['clr']).intersects(no_tab)):
                    continue
                if any(l_['tab'].distance(tab) < LAP_GAP for l_ in laps if l_['seam'] == theta):   # Nov 15 evening: 4 per seam
                    continue
                d = U(theta)
                screws = [tuple(O + d * (u0 + du + k * LAP_SCREW_DU) + nrm * side * TAB['depth'] * 0.5) for k in (-1, 1)]
                if any(Point(p).distance(Point(q)) < 9.0 for p in screws for q in avoid):
                    continue
                if avoid_geom is not None and any(avoid_geom.distance(Point(p)) < CB_D / 2 + 1.0 for p in screws):
                    continue
                laps.append(dict(seam=theta, lower=lo, upper=hi, u=u0 + du, tab=tab, screws=screws))
                lower[lo] = lower[lo].union(tab)
                sock = tab.buffer(TAB['clr'], join_style=2)
                band = affinity.translate(affinity.rotate(sbox(-400, -3 * TAB['r'], 400, 3 * TAB['r']), theta, origin=(0, 0)), *SEAM_O)
                lips = sock.buffer(3 * TAB['r']).intersection(band)    # Nov 17: round only the socket's two outside corners at the seam
                lower[hi] = round_convex(lower[hi].difference(sock), TAB['r'], lips)
                break
    return laps, lower


def deck_marks(g, fp):
    """Nov 15 (Fernando): each deck piece's number (labels.NUMBER: 1 FL, 2 FR, 3 RL, 4 RR) engraved in its top, glyphs toward the robot's front
    (read from above with the front up), where a box round it keeps DECK_MARK['keep'] off every hole, pocket, seat, tower, fin foot and edge"""
    import labels as LB
    from shapely import affinity as AF
    keep = DECK_MARK['keep']
    pts = (g['up'] + g['tw'] + g['fix'] + g['feet'] + F.pedestal_screws() + battery_clip_screws() + [p for lap in g['laps'] for p in lap['screws']]
           + [xy for _, _, xy, _ in g.get('btabs', [])])
    busy = union_all([B.place2d(union_all([fp['seats'], fp['towers']]), i) for i in range(4)] + [g['foot_plans'], union_all(strap_slots()),
                     Point(*LEAD_HOLE[:2]).buffer(LEAD_HOLE[2] / 2), Point(F.LIDAR_X, 0).buffer(PED_PAD_R), key_plan(KEY['clr'])]
                     + [Point(p).buffer(CB_D / 2) for p in pts] + [Point(p).buffer(STAND['pad_r']) for p in g['pads']])
    out = {}
    for n, d in g['decks'].items():
        pieces, (w, h) = LB.layout(str(LB.NUMBER[n]), DECK_MARK['size'])
        free = d.buffer(-keep).difference(busy.buffer(keep))
        to_uv = lambda q: AF.affine_transform(q, [0, -1, 1, 0, 0, 0])   # noqa: E731  (x, y) -> (u, v) = (-y, x): u reads toward -y, v = +x
        reg = to_uv(free)
        c = reg.representative_point() if not reg.is_empty else None
        uv = LB.fit(reg, w, h, (c.x, c.y), margin=0.0, step=2.0, reach=250.0) if c is not None else None
        if uv is None:
            out[n] = dict(placed=False)
            continue
        x, y = uv[1], -uv[0]
        cut = LB._mark_solid(pieces, DECK_MARK['size'], (x, y, Z1), (0, 0, 1), (1, 0, 0), LB.MARK_DEPTH)
        out[n] = dict(placed=True, text=str(LB.NUMBER[n]), xy=(round(x, 1), round(y, 1)), cut=cut,
                      zone=sbox(x - h / 2, y - w / 2, x + h / 2, y + w / 2))
    return out


def owner(decks, x, y):
    return next((n for n, g in decks.items() if g.buffer(0.3).contains(Point(x, y))), None)


# ---- accessory holes (Nov 14, Fernando): zip-tie anchors and spare M3 pilots through every deck piece -----------------------------------------
ACC_PITCH = 15.0                   # M3 pilots on a 15 mm square lattice (robot x / y) wherever the deck has room
ACC_WALL, ACC_KEEP = 2.6, 3.0      # walls: ACC_WALL from a pilot or slot to the deck's edge (both halves), ACC_KEEP off seats, laps, fin
ACC_SCREW_KEEP = 6.5               # feet, the pedestal; ACC_SCREW_KEEP round the screws. Tie pairs first, on a 5 mm lattice at 0/45/90/135
ACC_TIE_STEP, ACC_TIE_GAP = 5.0, 22.0   # deg (cable direction), >= ACC_TIE_GAP apart; pilots then skip anything within 4 mm of a pair.
ACC_BATTERY_KEEP = 10.0            # Nov 14: none over the battery (a screw from the top would reach the pack) nor its straps, the stand pads


def accessory_holes(g, fp):
    """plan positions and cutters per piece: zip-tie pairs (two TIE_SLOT_L x TIE_SLOT_W slots TIE_PITCH apart across the cable: the tie goes
    down one, under the bridge, up the other) and Ø M3_PILOT pilots, all straight through the 20 mm deck (along the print axis)"""
    from dims import TIE_SLOT_L, TIE_SLOT_W, TIE_PITCH
    seats = union_all([B.place2d(union_all([fp['seats'], fp['towers']]), i) for i in range(4)])
    pts = g['up'] + g['tw'] + g['fix'] + g['feet'] + F.pedestal_screws() + [p for lap in g['laps'] for p in lap['screws']] + battery_clip_screws()
    avoid = union_all([seats.buffer(ACC_KEEP), Point(F.LIDAR_X, 0).buffer(PED_PAD_R + ACC_KEEP), g['foot_plans'].buffer(ACC_KEEP),
                       battery_zone().buffer(ACC_BATTERY_KEEP), union_all(strap_slots()).buffer(ACC_BATTERY_KEEP)]
                      + [lap['tab'].buffer(ACC_KEEP + TAB['clr']) for lap in g['laps']] + [Point(p).buffer(ACC_SCREW_KEEP) for p in pts]
                      + [Point(p).buffer(STAND['pad_r'] + ACC_KEEP) for p in g['pads']]
                      + [tab_plan(ang, g['r_deck'] + 0.5).buffer(ACC_KEEP) for _, ang, _, _ in g.get('btabs', [])]
                      + [m['zone'].buffer(ACC_KEEP) for m in g.get('marks', {}).values() if m.get('placed')]
                      + [key_plan(KEY['clr'] + ACC_KEEP)])
    cutters, count = {}, {}
    slot = sbox(-TIE_SLOT_L / 2, -TIE_SLOT_W / 2, TIE_SLOT_L / 2, TIE_SLOT_W / 2)       # cable along local x; the slots across it
    for n, d in g['decks'].items():
        ok = d.intersection(g['lower'][n]).buffer(-ACC_WALL).difference(avoid)
        x0, y0, x1, y1 = d.bounds
        cut, ties, taken, pil_pts = [], [], [], []
        for x in np.arange(math.floor(x0 / ACC_TIE_STEP) * ACC_TIE_STEP, x1, ACC_TIE_STEP):    # tie pairs
            for y in np.arange(math.floor(y0 / ACC_TIE_STEP) * ACC_TIE_STEP, y1, ACC_TIE_STEP):
                if any(math.hypot(x - tx, y - ty) < ACC_TIE_GAP for tx, ty in ties):
                    continue
                for deg in (0.0, 90.0, 45.0, 135.0):
                    sl = [affinity.translate(affinity.rotate(affinity.translate(slot, 0.0, k * TIE_PITCH), deg, origin=(0, 0)), x, y) for k in (-0.5, 0.5)]
                    if all(ok.contains(s) for s in sl):
                        for s in sl:
                            cut.append(B.solid_of(s, Z0 - 1.0, Z1 + 1.0))
                        taken.append(union_all(sl).convex_hull)
                        ties.append((x, y))
                        break
        for x in np.arange(math.floor(x0 / ACC_PITCH) * ACC_PITCH, x1, ACC_PITCH):              # pilots
            for y in np.arange(math.floor(y0 / ACC_PITCH) * ACC_PITCH, y1, ACC_PITCH):
                p = Point(x, y)
                if ok.contains(p.buffer(M3_PILOT / 2, 16)) and not any(t.distance(p) < 4.0 for t in taken):
                    cut.append(W.cyl(M3_PILOT / 2, Z1 - Z0 + 2.0, (float(x), float(y), (Z0 + Z1) / 2)))
                    pil_pts.append((float(x), float(y)))
        cutters[n], count[n] = cut, dict(tie_pairs=len(ties), m3_pilots=len(pil_pts))
    return dict(cutters=cutters, count=count)


# ---- the bumper's tabs ---------------------------------------------------------------------------------------------------------------------
def tab_plan(ang, rb):
    """a tab's plan (with its gusset: the same footprint) at angle ang: r_in .. the bumper's inside, w wide"""
    T = BTAB
    p = sbox(T['r_in'], -T['w'] / 2, rb + 0.5, T['w'] / 2)
    return affinity.rotate(p, ang, origin=(0, 0))


def bumper_tabs(g, fp):
    """per quarter, up to BTAB['per_quarter'] tab angles where the tab's plan keeps clear of the pods (moving: swept, all heights; static), the
    fin feet's counterbores and the stand pads, and its screw lands on deck ACC_WALL + 1.5 inside a piece; near the quarter's 1/6, 1/2, 5/6
    points, >= min_sep deg apart. Returns [(quarter, angle, (x, y) screw, deck piece)]"""
    T, rb = BTAB, g['r_deck'] + 0.5
    obst = union_all([B.place2d(fp['moving'], i).buffer(BTAB_CLR_MOVING) for i in range(4)]
                     + [B.place2d(fp['pod_static'], i).buffer(BTAB_CLR_STATIC) for i in range(4)]
                     + [Point(p).buffer(CB_D / 2 + 2.0) for p in g['feet']] + [Point(p).buffer(STAND['pad_r'] + 2.0) for p in g['pads']])
    out = []
    for n in B.NAMES:
        a0, a1 = QUAD[n]
        ok = []
        for ang in np.arange(a0 + T['end_keep'], a1 - T['end_keep'] + 1e-6, T['step']):
            if tab_plan(ang, rb).intersects(obst):
                continue
            sx, sy = T['screw_r'] * U(ang)
            piece = next((m for m, d in g['decks'].items() if d.buffer(-(M3_PILOT / 2 + ACC_WALL + 1.5)).contains(Point(sx, sy))), None)
            if piece:
                ok.append((float(ang), (float(sx), float(sy)), piece))
        chosen = []
        for f in (1 / 6, 1 / 2, 5 / 6):
            tgt = a0 + f * (a1 - a0)
            cand = [c for c in ok if all(abs(c[0] - q[0]) >= T['min_sep'] for q in chosen)]
            if cand and len(chosen) < T['per_quarter']:
                chosen.append(min(cand, key=lambda c: abs(c[0] - tgt)))
        out += [(n, a, xy, pc) for a, xy, pc in sorted(chosen)]
    return out


def bumper_quarter(n, tabs, rb):
    """a bumper quarter: the 3 mm wall (z BUMPER_Z) and its tabs (t thick under the deck, a 45 deg gusset under each), screw holes"""
    T = BTAB
    a0, a1 = QUAD[n]
    ring = sector(a0 + 0.3, a1 - 0.3, rb + BUMPER_T).difference(Point(0, 0).buffer(rb, 256))
    q = B.prism(ring, *BUMPER_Z)
    zt = BUMPER_Z[1]
    zb = zt - T['t']
    for qn, ang, (sx, sy), _ in tabs:
        if qn != n:
            continue
        sec = [(rb + 1.0, zb - T['gusset']), (rb + 1.0, zt), (T['r_in'], zt), (T['r_in'], zb), (rb - T['gusset'], zb)]   # radial section (r, z)
        tab = b.extrude(b.Plane.XZ * b.Polygon(*sec, align=None), amount=T['w'] / 2, both=True)
        tab = tab.rotate(b.Axis.Z, ang)
        try:
            tab = b.fillet([e for e in tab.edges() if e.center().Z > zb - 0.01 and math.hypot(e.center().X, e.center().Y) < rb - 0.5
                            and abs(e.position_at(1).Z - e.position_at(0).Z) > 1.0], 1.5)    # the tab's two inner vertical corners
        except Exception:                                              # noqa: BLE001
            pass
        q = q + tab
        q = q - W.cyl(M3_CLR / 2, T['t'] + 2.0, (sx, sy, zb + T['t'] / 2)) - W.cyl(CB_D / 2, 2.9 + 0.1, (sx, sy, zb + 2.9 / 2 - 0.05))
    return q.clean()


# ---- solids ------------------------------------------------------------------------------------------------------------------------------
def snap(g):
    """a plan outline snapped to a 0.01 mm grid before it is extruded (Nov 15 audit: the booleans left a 0.0003 mm hole in deck_FR's plan,
    four zero-area faces that failed the validity check once the part was meshed)"""
    import shapely
    return shapely.set_precision(g, 0.01)


def drill(parts, decks, cutter, every=False):
    """cut `cutter` from the deck piece(s) it actually overlaps (every=True: all of them, for a hole on or near a seam). Nov 14 audit: an exact
    overlap, not a bounding-box test: the lower pieces' dovetail tabs reach across the seams, so a box test let FL take FR's pilots"""
    for n in decks:
        s = parts[f'deck_{n}']
        if W.near(s, cutter, 0.0) and W.overlap(s, cutter) > 1e-3:
            parts[f'deck_{n}'] = s - cutter
            if not every:
                return


def build(fp):
    """the base's printed parts, its hardware, the bought items it carries (the battery, straps) and the layout"""
    g = layout(fp)
    decks = g['decks']
    parts, hw, info = {}, {}, {}
    for n, pl in decks.items():
        s = (B.prism(snap(g['lower'][n]), Z0, ZL) + B.prism(snap(pl), ZL, Z1)).clean()
        top = [e for e in s.edges() if all(abs(e.position_at(t).Z - Z1) < 1e-3 for t in (0, 0.5, 1))]
        info[n] = 'top edges sharp (refused)'
        for r in (1.5, 1.0, 0.6):
            try:
                out = b.fillet(top, r)
                if out.is_valid and len(out.solids()) == 1:
                    s, info[n] = out, f'top edges R{r:g}'
                    break
            except Exception:                                              # noqa: BLE001
                pass
        else:
            try:
                out = b.chamfer(top, 0.8)
                if out.is_valid:
                    s, info[n] = out, 'top edges chamfer 0.8'
            except Exception:                                              # noqa: BLE001
                pass
        parts[f'deck_{n}'] = s
    for k, lap in enumerate(g['laps']):                                # the laps: down through the upper piece's top half into the tab
        for j, (x, y) in enumerate(lap['screws']):
            lo, hi = f'deck_{lap["lower"]}', f'deck_{lap["upper"]}'
            parts[hi] = parts[hi] - W.cyl(M3_CLR / 2, Z1 - ZL + 0.2, (x, y, (Z1 + ZL) / 2)) - W.cyl(CB_D / 2, 3.0, (x, y, Z1 - 1.45))
            parts[lo] = parts[lo] - W.cyl(M3_PILOT / 2, ZL - Z0 + 0.2, (x, y, (ZL + Z0) / 2)) - H.hex_recess((x, y, Z0), (0, 0, 1))
            hw[f'bolt_lap_{k}_{j}'] = H.pan_screw((x, y, Z1 - 2.9), (0, 0, -1), LAP_SCREW_L)   # (Nov 15 evening: a flush hex recess under the
                                                                       # tab for an optional nut, with a longer screw; the pilot threads without)
    through = lambda d, x, y: W.cyl(d / 2, Z1 - Z0 + 1, (x, y, (Z0 + Z1) / 2))   # noqa: E731
    for x, y in g['up'] + g['fix']:                                    # hangers, beams, the towers' short screws: the threads in the deck
        drill(parts, decks, through(M3_PILOT, x, y))
    for x, y in g['tw']:                                               # the hangers' long bolts pass up through the deck into the towers
        drill(parts, decks, through(M3_CLR, x, y))
    for k, (x, y) in enumerate(g['feet']):                             # the fins' feet: M3 x 30 up through the deck (counterbored below)
        drill(parts, decks, through(M3_CLR, x, y) + W.cyl(CB_D / 2, 3.0, (x, y, Z0 + 1.45)), every=True)
        hw[f'bolt_fin_{k}'] = H.pan_screw((x, y, Z0 + 2.9), (0, 0, 1), F.FIN_SCREW_L)
    for k, (x, y) in enumerate(F.pedestal_screws()):                   # the pedestal's screws into the deck
        drill(parts, decks, through(M3_PILOT, x, y))                    # through the deck (every hole through its part)
        hw[f'bolt_pedestal_{k}'] = H.pan_screw((x, y, Z1 + 4.0), (0, 0, -1), 14.0)
    g['btabs'] = bumper_tabs(g, fp)                                   # the bumper's tabs under the rim, an M3 up into the deck each
    for k, (qn, ang, (x, y), piece) in enumerate(g['btabs']):
        parts[f'deck_{piece}'] = parts[f'deck_{piece}'] - through(M3_PILOT, x, y)
        hw[f'bolt_bumper_{k}'] = H.pan_screw((x, y, BUMPER_Z[1] - BTAB['t'] + 2.9), (0, 0, 1), BTAB['screw_L'])
    # Nov 15: the pods' alignment pockets (the towers' bosses into the top, the hangers' and beams' into the underside)
    for i in range(4):
        drill(parts, decks, B.solid_of(B.place2d(fp['align_top'], i).buffer(PM.ALIGN_CLR, 16), Z1 - PM.ALIGN_DEPTH, Z1 + 1.0), every=True)
        drill(parts, decks, B.solid_of(B.place2d(fp['align_bottom'], i).buffer(PM.ALIGN_CLR, 16), Z0 - 1.0, Z0 + PM.ALIGN_DEPTH), every=True)
    # Nov 15 evening: the centre key, flush in a recess over the seams' crossing; one M3 x 16 down through each arm into its piece (a flush hex
    # recess under the deck for an optional nut, as the laps)
    kz0 = Z1 - KEY['depth']
    drill(parts, decks, B.solid_of(key_plan(KEY['clr']), kz0, Z1 + 1.0), every=True)
    key = B.solid_of(key_plan(), kz0, Z1)
    try:
        key = b.fillet([e for e in key.edges() if all(abs(e.position_at(t).Z - Z1) < 1e-3 for t in (0, 0.5, 1))], 0.6)
    except Exception:                                                  # noqa: BLE001
        pass
    for k, (x, y) in enumerate(key_screws()):
        key = key - W.cyl(M3_CLR / 2, KEY['depth'] + 1.0, (x, y, (kz0 + Z1) / 2)) - W.cyl(CB_D / 2, 3.0, (x, y, Z1 - 1.45))
        drill(parts, decks, W.cyl(M3_PILOT / 2, kz0 - Z0 + 0.2, (x, y, (kz0 + Z0) / 2)) + H.hex_recess((x, y, Z0), (0, 0, 1)))
        hw[f'bolt_key_{k}'] = H.pan_screw((x, y, Z1 - 2.9), (0, 0, -1), LAP_SCREW_L)
    kp = key_plan()
    for x, y in g['feet']:                                             # fin feet's screws that pass up through the key
        if kp.buffer(M3_CLR / 2).intersects(Point(x, y)):
            key = key - W.cyl(M3_CLR / 2, KEY['depth'] + 1.0, (x, y, (kz0 + Z1) / 2))
    parts['deck_key'] = key
    # Nov 15: each piece's number in its top (before the accessory holes, which keep off it)
    g['marks'] = deck_marks(g, fp)
    for n, m in g['marks'].items():
        if m['placed']:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - m['cut']
    # the battery: clip screws' pilots, strap slots, the lead's hole (its edges chamfered), all through the deck
    for x, y in battery_clip_screws():
        drill(parts, decks, through(M3_PILOT, x, y))
    for s in strap_slots():
        drill(parts, decks, B.solid_of(s, Z0 - 1.0, Z1 + 1.0), every=True)
    hx, hy, hd = LEAD_HOLE
    lead_cut = (through(hd, hx, hy) + b.Solid.make_cone(hd / 2 + 1.5, hd / 2, 1.5, b.Plane(origin=(hx, hy, Z1 - 1.5 + 1e-3), z_dir=(0, 0, -1))).moved(b.Location((0, 0, 1.5)))
                + b.Solid.make_cone(hd / 2 + 1.5, hd / 2, 1.5, b.Plane(origin=(hx, hy, Z0 - 1e-3), z_dir=(0, 0, 1))))
    drill(parts, decks, lead_cut, every=True)
    # service stand: a socket for the stand's peg in each pad (from the underside, along the print axis)
    for x, y in g['pads']:
        drill(parts, decks, W.cyl(STAND['socket_d'] / 2, STAND['socket_h'] + 1.0, (x, y, Z0 + STAND['socket_h'] / 2)))
    acc = accessory_holes(g, fp)                                       # zip-tie pairs and spare M3 pilots on a grid
    for n, cutters in acc['cutters'].items():
        if cutters:
            parts[f'deck_{n}'] = parts[f'deck_{n}'] - b.Compound(cutters)
    g['accessory'] = acc['count']
    rb = g['r_deck'] + 0.5
    for n in B.NAMES:                                                  # bumper quarters hung on the deck's rim (one per quarter)
        parts[f'bumper_{n}'] = bumper_quarter(n, g['btabs'], rb)
    parts['pedestal'] = F.build_pedestal()
    parts.update(g['fins'])
    bp, bb, bh = battery_parts()
    parts.update(bp)
    hw.update(bh)
    g.update(rb=rb, info=info, bought=bb)
    return parts, hw, g


# ---- checks --------------------------------------------------------------------------------------------------------------------------------
def compound(d, skip=()):
    return b.Compound([v for k, v in d.items() if not k.startswith(skip)])


def stand_report(g, fp):
    """how high the stand must hold the deck for a pod to drop out and slide out under the bumper: the dropping part of a pod (everything but
    its towers, plungers, springs and their bolts) at droop, robot z"""
    st, mv = pods_placed(R.A_DROOP)['FL']
    drop = {k: v for k, v in {**st, **mv}.items() if not k.startswith(STAYS + ('plunger', 'spring') + WHEEL_PARTS)}
    bb = compound(drop).bounding_box()                                  # the wheel: its revolved envelope (padded boxes of turned rollers)
    env = R.wheel_envelope(None)
    ya, za = (float(v) for v in R.P.rot_pt((0.0, 0.0), R.A_DROOP))
    top = max(bb.max.Z, za + env.bounds[3] + R_WHEEL)
    bot = min(bb.min.Z, za - env.bounds[3] + R_WHEEL)
    h = top - bot
    lift = h - BUMPER_Z[0] + 5.0                                     # the pod, set down on the bench, passes 5 mm under the bumper
    return dict(pads_xy=[[round(v, 1) for v in p] for p in g['pads']], n_pads=len(g['pads']), pad_d=2 * STAND['pad_r'],
                socket=f"Ø{STAND['socket_d']:g} through the deck (a Ø8 peg from below)",
                pod_drop_part_height_mm=round(h, 1), lift_over_floor_mm=round(lift, 1), pad_height_over_bench_mm=round(Z0 + lift, 1),
                note='the robot on four pegs, its normal floor lifted by lift_over_floor_mm: a pod (or the battery) lowered onto the bench '
                     'slides out radially under the bumper between two pegs')


def check(parts, hw, g, fp):
    decks = g['decks']
    mv_plan = [B.place2d(fp['moving'], i) for i in range(4)]
    st_plan = [B.place2d(union_all([fp['pod_static'], fp['towers']]), i) for i in range(4)]
    r_m = max(rmax(q) for q in mv_plan)
    rep = dict(layout=dict(D=B.D_WHEEL, slide=B.SLIDE, deck_z=[Z0, Z1], deck_z_v14=[40.0, 62.0], r_deck=g['r_deck'], r_hold=g['r_hold'],
                           r_opening=g['r_open'], r_moving=round(r_m, 1), bumper_inside_r=g['rb'], robot_diameter=round(2 * (g['rb'] + BUMPER_T), 1),
                           robot_diameter_v14=450.3, bumper_to_moving_mm=round(g['rb'] - r_m, 1), opening_clearance_mm=CLR))
    pair = lambda i, j: round(union_all([mv_plan[i], st_plan[i]]).distance(union_all([mv_plan[j], st_plan[j]])), 1)   # noqa: E731
    rep['pods_plan_gaps_mm'] = dict(side_FL_RL=pair(0, 1), side_FR_RR=pair(3, 2), front_FL_FR=pair(0, 3), back_RL_RR=pair(1, 2))
    q = {}
    for i, n in enumerate(B.NAMES):
        d = decks[n]
        seat_h = B.place2d(fp['seats'], i)
        seat_t = B.place2d(fp['towers'], i)
        mine = lambda pts: [p for p in pts if owner(decks, *p) == n]                                                   # noqa: E731
        margin = lambda pts: round(min(d.exterior.distance(Point(p)) if d.contains(Point(p)) else -d.distance(Point(p)) for p in pts), 2) if pts else None   # noqa: E731
        up_i = [tuple(B.place2d(Point(*p), i).coords[0]) for p in fp['up_screws']]
        tw_i = [tuple(B.place2d(Point(*p), i).coords[0]) for p in fp['tower_screws']]
        # Nov 16: the bed fit of the piece's solid (its dovetail tabs reach past the plan outline `d`: FL measured 234.5 x 238.6 on the plan,
        # 243 x 258.6 as a solid)
        q[n] = dict(volume_cm3=round(parts[f'deck_{n}'].volume / 1000, 1), solids=len(parts[f'deck_{n}'].solids()), valid=bool(parts[f'deck_{n}'].is_valid),
                    bed=PB.bed_fit(parts[f'deck_{n}'], '+z'),
                    hanger_beam_tops_on_deck_pct=round(100 * seat_h.intersection(d).area / seat_h.area, 1),
                    tower_bases_on_deck_pct=round(100 * seat_t.intersection(d).buffer(0).area / max(seat_t.area, 1e-6), 1),
                    pod_up_screws_on_own_piece=f'{len(mine(up_i))} of {len(up_i)}', pod_up_screws_edge_margin_mm=margin(mine(up_i)),
                    tower_screws_edge_margin_mm=margin(mine(tw_i)), plan_gap_to_openings_mm=round(d.distance(g['openings'].buffer(-CLR)), 2),
                    top_edges=g['info'][n])
    rep['deck_pieces'] = q
    rep['cantilever_strips_removed_mm2'] = g['pruned']
    rep['joints'] = dict(kind='dovetail half-laps across the seams (lower piece: bottom half tab; upper piece: socket + its top half over it), '
                              f'2 x M3 x {LAP_SCREW_L:g} each, down from the top into the tab', order='FL and RR first, RL and FR dropped onto them',
                         seams_cross_at=SEAM_O, laps=[dict(seam_deg=l['seam'], lower=l['lower'], upper=l['upper'], r=l['u']) for l in g['laps']],
                         per_seam={str(t): sum(1 for l in g['laps'] if l['seam'] == t) for t, _, _ in SEAMS},
                         socket_clearance_per_face=TAB['clr'], fillet_r=TAB['r'],
                         fillets='the tab\'s far corners and the socket\'s two lips at the seam (vertical edges); the key: R1 outside corners')
    # Nov 15: every pod seat (hanger / beam top) on its own piece only, no tab over it; the alignment pockets inside one piece with walls
    seat_other = {}
    for i, n in enumerate(B.NAMES):
        seat = B.place2d(fp['seats'], i)
        for m_, d_ in g['lower'].items():
            if m_ != n:
                a_ = seat.intersection(d_).area
                if a_ > 0.5:
                    seat_other[f'{n} seats on {m_}'] = round(a_, 1)
    pockets = {}
    for i, n in enumerate(B.NAMES):
        for nm, key, plan_of in (('top', 'align_top', lambda m_: decks[m_]), ('underside', 'align_bottom', lambda m_: g['lower'][m_])):
            pk = B.place2d(fp[key], i).buffer(PM.ALIGN_CLR, 16)
            for pq in getattr(pk, 'geoms', [pk]):
                own = [m_ for m_ in decks if plan_of(m_).intersection(pq).area > 0.5]
                walls = [round(plan_of(m_).exterior.distance(pq) if plan_of(m_).contains(pq) else -1.0, 2) for m_ in own]
                tabs = [round(pq.distance(l_['tab']), 2) for l_ in g['laps'] if l_['tab'].distance(pq) < 6.0]
                pockets.setdefault(n, []).append(dict(face=nm, area_mm2=round(pq.area, 1), pieces=own, wall_to_edge_mm=walls, near_tabs_mm=tabs))
    rep['alignment_pockets'] = dict(depth=PM.ALIGN_DEPTH, per_pod=pockets, seats_on_another_piece_mm2=seat_other,
                                    note='a pocket must be in one piece (wall >= 0 inside it); seats_on_another_piece must be empty (Nov 15: FR\'s beam '
                                         'under RR\'s tab)')
    rep['deck_marks'] = {n: {k: v for k, v in m.items() if k not in ('cut', 'zone')} for n, m in g.get('marks', {}).items()}
    rep['accessory_holes'] = dict(per_piece=g.get('accessory'), pitch=ACC_PITCH, note='zip-tie pairs and M3 pilots (Ø2.8, through) on a 15 mm '
                                  'lattice where the deck has room; none over the battery, its straps, the stand pads')
    rep['deck_volume_cm3'] = dict(total=round(sum(v['volume_cm3'] for v in q.values()), 1), v14_frame=1141.3)
    # the fins, pedestal, bumper and battery against the pods (every pose)
    clash, near_ = {}, {}
    items = {k: v for k, v in parts.items() if k.startswith(('support_', 'pedestal', 'bumper_', 'bracket_battery'))}
    items.update({k: v for k, v in g['bought'].items()})
    for pn, a in R.POSES.items():
        for pod, (st, mv) in pods_placed(a).items():
            for nm, sh in items.items():
                for grp, dct in (('static', st), ('moving', mv)):
                    cp = compound(dct, ('roller_core', 'bearing_687', 'bolt_cart'))
                    if not W.near(sh, cp, 6.0):
                        continue
                    v = W.overlap(sh, cp) if W.near(sh, cp, 0.0) else 0.0
                    key = f'{nm} / {pod} {grp}'
                    if v > 0.01:
                        clash[f'{key} ({pn})'] = round(v, 1)
                    near_[key] = min(near_.get(key, 99.0), 0.0 if v > 0.01 else round(sh.distance_to(cp), 2))
    rep['base_items_vs_pods'] = dict(overlap_mm3=clash, min_gap_mm=near_, note='moving: wheel parts at one roller angle; the fins were cut '
                                     'from the wheel\'s revolved envelope + 3')
    rep['fins'] = dict(per_fin=g['fin_rep'], volume_cm3={k: round(v.volume / 1000, 1) for k, v in parts.items() if k.startswith('support_')},
                       feet_screws=len(g['feet']), feet_on=sorted({owner(decks, *xy) or 'NOT ON THE DECK' for xy in g['feet']}),
                       solids={k: (len(v.solids()), bool(v.is_valid)) for k, v in parts.items() if k.startswith('support_')},
                       rule=f'lower edge {F.CLR_MOVING:g} mm off the moving parts, {F.CLR_STATIC:g} off the static ones, + R{F.R_EDGE:g} rounding; '
                            f'feet on solid deck; M3 x {F.FIN_SCREW_L:g} up from below')
    fins_in = {k: v for k, v in parts.items() if k.startswith('support_')}
    ov = {}
    for a_, sa in fins_in.items():
        for c_, sc in parts.items():
            if c_ == a_ or c_.startswith('support_'):
                continue
            if W.near(sa, sc, 0.0) and W.overlap(sa, sc) > 0.05:
                ov[f'{a_} / {c_}'] = round(W.overlap(sa, sc), 2)
    rep['fins']['overlap_with_base_parts_mm3'] = ov
    rep['bumper'] = dict(z=BUMPER_Z, tabs=[dict(quarter=q_, deg=round(a, 1), screw_xy=[round(v, 1) for v in xy], into=f'deck_{pc}') for q_, a, xy, pc in g['btabs']],
                         per_quarter={n: sum(1 for q_, *_ in g['btabs'] if q_ == n) for n in B.NAMES},
                         solids={k: (len(v.solids()), bool(v.is_valid)) for k, v in parts.items() if k.startswith('bumper_')},
                         screw=f"M3 x {BTAB['screw_L']:g} pan up through the tab (counterbored) into a Ø2.8 pilot through the deck: {BTAB['screw_L'] - BTAB['t'] + 2.9:.1f} mm of thread")
    (l, w, h), (cx, cy) = BATTERY['size'], BATTERY['xy']
    bz = compound({k: v for k, v in dict(g['bought'], **{k: v for k, v in parts.items() if k.startswith('bracket_battery')}).items()})
    rep['battery'] = dict(at=dict(x=[cx - l / 2, cx + l / 2], y=[cy - w / 2, cy + w / 2], z=[Z0 - h, Z0]), lowest_z=round(bz.bounding_box().min.Z, 1),
                          clips=4, clip_screws=f"M3 x {BATTERY['clip']['screw_l']:g} up into the deck", straps=len(BATTERY['strap_u']),
                          lead_hole=dict(xy=LEAD_HOLE[:2], d=LEAD_HOLE[2], in_piece=owner(decks, *LEAD_HOLE[:2]),
                                         wall_to_opening_mm=round(g['openings'].distance(Point(*LEAD_HOLE[:2])) - LEAD_HOLE[2] / 2, 1)),
                          plan_gap_to_pods_mm=round(battery_zone().distance(union_all([B.place2d(union_all([fp['moving'], fp['pod_static']]), i) for i in range(4)])), 1),
                          out='strap open, XT90 unplugged under the deck, the pack drops out (robot on the stand)',
                          cg_drop_mm=round((181.5 + 2.0 + h / 2) - (Z0 - h / 2), 1), note='cg_drop: the pack centre vs v14 (on the second floor)')
    rep['printed'] = printed_report(parts)
    rep['bed_fit_failures'] = PB.bed_failures(rep['printed'])          # must be empty (cad/solidworks.md 18)
    rep['hole_audit'] = hole_audit(parts, hw, fp)
    rep['lidar'] = lidar_shadows(g)
    (VAL / 'lidar_shadows.json').write_text(json.dumps(dict(source='chassis_mid.py check: the generated fins at the scan plane', supports=rep['lidar']['supports']), indent=1))
    rep['computer_on_the_deck'] = computer_fit(fp)
    rep['stand'] = stand_report(g, fp)
    rep['lidar_pedestal'] = dict(height_mm=round(F.LIDAR_BASE_Z - Z1, 1), height_v14_mm=round(F.LIDAR_BASE_Z - 62.0, 1))
    top = max(compound(st).bounding_box().max.Z for st, _ in pods_placed(0.0).values())
    rep['heights'] = dict(pod_and_tower_top_z=round(top, 1), wheel_top_bump_z=round(R_WHEEL + float(R.P.rot_pt((0, 0), R.A_BUMP)[1]) + R_WHEEL, 1),
                          scan_plane_z=F.LIDAR['plane'], floor2_under=F.FLOOR2_UNDER, lowest_chassis_z=dict(deck=Z0, bumper=BUMPER_Z[0]))
    return rep


def printed_report(parts):
    """every printed base part: one valid solid, its print orientation (print_beds.py), overhang area as printed, bed fit (cad/solidworks.md 17-18)"""
    out = {}
    for k, s in parts.items():
        bd = PB.bed(k)
        if bd is None:
            out[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), bed='NO PRINT_BED ENTRY')
            continue
        up = H.up_axis(bd[0]) * (1.0 if bd[0][0] == '+' else -1.0)
        out[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), up=bd[0], supports=bd[1], overhang_mm2=R.overhang_mm2(s, up),
                      bed=PB.bed_fit(s, bd[0]))                         # best rotation (covers the diagonal), 2 mm margin
    for line in PB.bed_failures(out):
        print('BED FIT FAIL', line, flush=True)
    return out


def pod_screw_recs():
    """each pod's own screws into the deck (pod_mid.static_recs), in the robot frame (base.place3d's placement, mirror included)"""
    out = []
    for i, n in enumerate(B.NAMES):
        A, xh, yh, m = B.frame(i)
        sg = -1.0 if m else 1.0
        for r in PM.static_recs():
            p, d = np.asarray(r['p'], float), np.asarray(r['d'], float)
            pr = (A[0] + xh[0] * p[0] + yh[0] * sg * p[1], A[1] + xh[1] * p[0] + yh[1] * sg * p[1], p[2] + R_WHEEL)
            dr = (xh[0] * d[0] + yh[0] * sg * d[1], xh[1] * d[0] + yh[1] * sg * d[1], d[2])
            out.append(dict(r, p=tuple(float(v) for v in pr), d=tuple(float(v) for v in dr), name=f"{n}/{r['name']}"))
    return out


def hole_audit(parts, hw, fp):
    """hardware.audit over every base screw (fin feet, laps, pedestal, bumper tabs, battery clips) and each pod's screws into the deck, against
    the base's printed parts and the pods' static parts: thread per screw (>= 6 mm), half holes, pilots across layers, heads in parts, no hole;
    and the screws whose hole stops inside its printed part (cad/solidworks.md 18: every hole through)"""
    recs = [dict(r, name=k) for k, r in ((k, H.from_solid(s, k)) for k, s in hw.items()) if r is not None] + pod_screw_recs()
    st = {}
    for n, (s_, _) in pods_placed(0.0).items():
        st.update({f'{n}/{k}': v for k, v in s_.items() if k.startswith(('rx_hanger', 'rx_beam', 'rx_tower', 'rx_lug', 'rx_pin'))})
    pool = dict({k: v for k, v in parts.items()}, **st)
    base_up = H.bed_up_of()

    def up_of(name):
        """print axis in the robot frame: a pod part's (pod-frame) axis turned with its pod (Nov 15: the hangers and beam print along x)"""
        u = base_up(name)
        if u is None or '/' not in name:
            return u
        i = B.NAMES.index(name.split('/')[0])
        A, xh, yh, m = B.frame(i)
        sg = -1.0 if m else 1.0
        return np.array([xh[0] * u[0] + yh[0] * sg * u[1], xh[1] * u[0] + yh[1] * sg * u[1], u[2]])
    au = H.audit(recs, pool, up_of)
    th = {k: round(sum(v.values()), 1) for k, v in au['thread'].items()}
    nutted = ('bolt_tower_',)                                          # the long tower bolts end in hex nuts in the tabs' tops
    short = {k: v for k, v in th.items() if v < 5.9 and not k.split('/')[-1].startswith(nutted)}
    blind = {k: v['solid'] for k, v in H.blind_holes(recs, pool, up_of).items()}
    return dict(screws=len(recs), thread_min_mm=min(th.values()) if th else None, short_of_6mm=short, partial=au['partial'],
                across_layers=au['across_layers'], soft_across_layers=au['soft_across_layers'], heads_in_parts=au['head'], no_hole=au['no_hole'],
                blind_holes_mm_of_plastic=blind,
                note='partial: a hole that is not a full circle in a part (a seam, an edge); across layers: a thread-forming pilot > 45 deg off '
                     'the part\'s print axis; blind: the hole stops inside its printed part, mm of plastic left on its axis')


def lidar_shadows(g):
    """each fin's shadow at the scan plane as the lidar sees it (its section there: bearing span from the lidar's axis), and the total"""
    out = []
    for name, sec in g['sections']:
        pts = np.array([c for q in getattr(sec, 'geoms', [sec]) for c in q.exterior.coords])
        d = pts - np.array([F.LIDAR_X, 0.0])
        brg = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
        mid = math.degrees(math.atan2(np.sin(np.radians(brg)).mean(), np.cos(np.radians(brg)).mean()))
        off = ((brg - mid + 180) % 360) - 180
        out.append(dict(support=name, nearest=round(float(np.hypot(d[:, 0], d[:, 1]).min()), 1), bearing=round(mid, 1),
                        shadow_deg=round(float(off.max() - off.min()), 1)))
    return dict(scan_z=F.LIDAR['plane'], supports=out, blind_total_deg=round(sum(s['shadow_deg'] for s in out), 1),
                v14_blind_total_deg=round(5.3 + 2 * 3.7 + 2 * 4.1, 1))


def computer_fit(fp):
    """Nov 14 review (Fernando: can the computer move down too?): where the BMAX B8A Pro (126 x 112 x 52, its plugs' 12 mm on one face) fits on
    the deck's top: 3 mm off everything of the pods above the deck (every pose) and the towers, the pedestal; under the scan plane"""
    above = union_all([B.place2d(union_all([fp['above'], fp['towers']]), i) for i in range(4)]).buffer(3.0)
    occ = union_all([above, Point(F.LIDAR_X, 0).buffer(F.PED_TOP_R + 2.0)])
    disc = Point(0, 0).buffer(229 - 8)
    hits = []
    for ang in (0, 90):
        r = affinity.rotate(sbox(-63.0, -56.0, 63.0 + 12.0, 56.0), ang, origin=(0, 0))
        for x in range(-150, 151, 5):
            for y in range(-150, 151, 5):
                qq = affinity.translate(r, x, y)
                if disc.contains(qq) and not qq.intersects(occ):
                    hits.append((x, y, ang))
    best = min(hits, key=lambda t: math.hypot(t[0], t[1])) if hits else None
    return dict(fits=bool(hits), positions=len(hits), nearest_centre=best, top_z=round(Z1 + 3.0 + 52.0, 1), scan_plane=F.LIDAR['plane'],
                note='not moved (Nov 14): the spine fin runs through that spot and the bin deck over the second floor would come down with it; '
                     'see README')


# ---- outputs ---------------------------------------------------------------------------------------------------------------------------------
def render(parts, hw, g, fp):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    VAL.mkdir(parents=True, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(20, 10))
    for ax, show in ((axs[0], False), (axs[1], True)):
        for n, d in g['decks'].items():
            for qq in getattr(d, 'geoms', [d]):
                ax.fill(*qq.exterior.xy, color=COLOURS['deck'], ec='k', lw=0.6)
                for hole in qq.interiors:
                    ax.fill(*hole.xy, color='w', ec='k', lw=0.4)
            c = d.representative_point()
            ax.annotate(f'deck_{n}', (c.x, c.y), fontsize=9, ha='center')
        if show:
            for i in range(4):
                for key, col, al in (('moving', (0.95, 0.85, 0.5), 0.5), ('pod_static', (0.40, 0.60, 0.70), 0.8), ('towers', (0.25, 0.45, 0.60), 0.9)):
                    p = B.place2d(fp[key], i)
                    for qq in getattr(p, 'geoms', [p]):
                        ax.fill(*qq.exterior.xy, color=col, alpha=al)
        bz = battery_zone()
        for qq in getattr(bz, 'geoms', [bz]):
            ax.plot(*qq.exterior.xy, color=(0.85, 0.2, 0.2), lw=1.2, ls='--')
        for lap in g['laps']:
            ax.plot(*lap['tab'].exterior.xy, color='r', lw=0.8)
            for x, y in lap['screws']:
                ax.plot(x, y, 'r.', ms=4)
        for pts, sty in ((g['up'], 'm.'), (g['tw'], 'c.'), (g['feet'], 'b.')):
            for x, y in pts:
                ax.plot(x, y, sty, ms=4)
        for x, y in g['pads']:
            ax.add_patch(plt.Circle((x, y), STAND['pad_r'], fill=False, color='g', lw=1.5))
        for _, sec in g['sections']:
            for qq in getattr(sec, 'geoms', [sec]):
                ax.fill(*qq.exterior.xy, color='k')
        ax.add_patch(plt.Circle((0, 0), g['rb'] + BUMPER_T, fill=False, color=COLOURS['bumper'], lw=3))
        ax.add_patch(plt.Circle((0, 0), 225.15, fill=False, color='grey', ls='--', lw=0.8))
        ax.axhline(0, lw=0.4, color='k')
        ax.axvline(0, lw=0.4, color='k')
        ax.set_aspect('equal')
        lim = g['rb'] + 20
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.grid(lw=0.3)
    axs[0].set_title(f'v15 mid deck (z {Z0:.0f}-{Z1:.0f}): four pieces, bumper Ø{2 * (g["rb"] + BUMPER_T):.0f} (dashed: v14 Ø450); magenta: pod screws, '
                     'cyan: tower bolts, blue: fin feet, green: stand pads, red dashed: the battery under the deck', fontsize=9)
    axs[1].set_title('with the pods: yellow moving (swept), teal hangers + beam (under the deck), dark: towers (on it)', fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=80)
    plt.close(fig)
    pods = pods_placed(0.0)
    hide = ('roller_core', 'roller_sleeve', 'bearing_687', 'bolt_1', 'bolt_2', 'bolt_cartridge')
    allpods = {f'{n}/{k}': v for n, (st, mv) in pods.items() for k, v in {**st, **mv}.items() if not k.startswith(hide)}
    base = {k: v for k, v in dict(parts, **g['bought']).items() if not k.startswith('support_')}
    R.iso(dict(base, **allpods), VAL / 'iso_base.png', 215, 30, 'v15 mid deck with the pods (fins and second floor hidden)', size=(14, 10))
    R.iso(dict(parts, **g['bought'], **allpods), VAL / 'iso_with_fins.png', 215, 25, 'v15 mid deck, pods, pedestal and the generated fins', size=(14, 10))
    R.iso({k: v for k, v in parts.items() if k.startswith(('support_', 'deck_', 'pedestal'))}, VAL / 'iso_fins.png', 150, 25,
          'the fins on the deck (pods hidden): feet where the deck is free, arches over the pods', size=(14, 10))
    R.iso(dict(base, **allpods), VAL / 'iso_below.png', 200, -30, 'from below: pods, the battery under the centre, the stand pads', size=(14, 10))
    one = {k: v for k, v in parts.items() if k in ('deck_FL', 'bumper_FL')}
    st, mv = pods['FL']
    one.update({f'FL/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide)})
    R.iso(one, VAL / 'iso_FL.png', 200, 30, 'deck_FL with its pod, towers and bumper quarter', size=(12, 9))
    mod = {f'FL/{k}': v for k, v in {**st, **mv}.items() if not k.startswith(hide + STAYS + ('plunger', 'spring'))}
    R.iso(dict(mod, deck_FL=parts['deck_FL'].moved(b.Location((0, 0, 60)))), VAL / 'iso_FL_drop_out.png', 200, 20,
          'FL pod dropped out under its deck piece (the towers and springs stay on the deck: hidden here)', size=(12, 9))


def export(parts, hw, g):
    OUT.mkdir(parents=True, exist_ok=True)
    for f in OUT.glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        write_part(s, colour(k), OUT / f'{k}.step', k)
    items = {k: (v, colour(k)) for k, v in dict(parts, **g['bought']).items()}
    items.update({k: (v, H.MAGENTA) for k, v in hw.items()})
    for n, (st, mv) in pods_placed(0.0, labelled=True).items():
        for k, v in {**st, **mv}.items():
            items[f'{n}_{k}'] = (H.step_model(k, v), H.MAGENTA if k.startswith(H.STEP_HARDWARE) or k.startswith('plunger') else R.colour(k))
    write_step(items, OUT / 'v15_mid_base_assembly.step')
    return len(items)


_BUILT = None


def built():
    """(parts, hw, g) of the base, built once per process (world.py, robot_step.py, the harness)"""
    global _BUILT
    if _BUILT is None:
        _BUILT = build(pod_build()[4])
    return _BUILT


def main(stage):
    fp = pod_build()[4]
    if stage == 'study':
        o = outline(fp)
        print({k: o[k] for k in ('r_deck', 'r_hold', 'r_open')}, 'stand pads', stand_pads(o, fp))
        return
    parts, hw, g = build(fp)
    for k, v in parts.items():
        if k.startswith(('deck_', 'support_')):
            print(k, len(v.solids()), v.is_valid, round(v.volume / 1000, 1), 'cm3', flush=True)
    print(json.dumps(g['fin_rep'], default=str), flush=True)
    VAL.mkdir(parents=True, exist_ok=True)
    if stage in ('check', 'all'):
        rep = check(parts, hw, g, fp)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1, default=str))
        print(json.dumps(rep['layout']), json.dumps(rep['stand']), flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, g, fp)
        print('rendered', flush=True)
    if stage in ('export', 'all'):
        print('exported', export(parts, hw, g), flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'study')

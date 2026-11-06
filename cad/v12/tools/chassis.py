"""cad/v11 chassis for layout F (PROPOSAL, Nov 3 2026): the first floor as a frame instead of a plate. build123d, STEP review only.
Replaces first_floor.py's petals and spider (kept there for comparison); reuses its D435 cradle and pod footprints.

    python -B chassis.py [build|check|export|render|all]        (omni-robot-sim interpreter)

Why a frame: the 6 mm plate tied neighbouring pods through 19-25 mm wide necks (about as stiff as a ruler); its petals were really held
together by the pods. The first floor's jobs are to tie the four pods into one rigid chassis and hold the lidar, the D435 and the bumper; the
second floor stands on the pods themselves (their spring columns), so its weight goes straight to the wheels. Robot frame: x forward, z up.
 * RING: solid square-section members, 22 mm deep (z 40..62; print them with few walls and low infill), on |x| + |y| = 100..121 under the pods'
   inner frames; CROSS on the square's diagonals triangulates it; NODES at its corners run out to the rim.
 * Each pod is held, not just screwed: its inner frame stands in a pocket (0.2 mm clearance) whose 14 mm walls hug its centre-side face and its
   front end; its outer keel stands in a pocket on the side node (walls behind it and at its inboard end). Its pivot block bears on the ring.
   Round 12: all its screws come UP from under the members (3 into the frame's foot, 2 into the keel), and members and walls are clipped 1 mm
   off the path every pod part sweeps going out radially, so a pod comes out sideways with the second floor on (bumper quarter off).
 * Lidar (A2M12) on a pedestal on the front node at x 120, scan z 163: forward, where the scans matter, just behind the D435 cradle (the sim's
   x 140 would put its top plate into the cradle). Nothing of the pods reaches its height band.
 * Second-floor supports: three printed fins, each standing in a vertical plane through the lidar's axis, so at the scan plane the lidar sees
   only a 5 mm web edge-on however long the fin is. Each fin uses that length: a flange the whole length under the floor, two legs, an elliptical
   arch between them and coves out to the tips; legs flare toward the deck. The spine runs down the centreline behind the lidar (feet on the cross
   and the back node, under the battery and computer). The two side fins lie on the rays through the front pods' column-top pilots, which also
   pass under the arm riser's rear corners: nose under the corner, seat on the column top (dowels in its two pilots), foot on the ring by the side
   stem. That is the arm's front support; nothing stands in the lidar's front half. M3 x 30 up through the members into the feet; the floor
   screws down into the flanges. Printed upside down on the flange without supports.
 * Bumper: just for bumps (not structural): 3 mm PETG quarters, countersunk flat-head M3 from outside into the node arcs, lips on the arcs.
 * Nine frame parts (hub/cross, four ring edges, four nodes) joined by half-laps (11 + 11 mm, 2 x M3 each). Every part fits a 256 bed.
Clipped to stay 3 mm off everything that moves (droop..bump) and off each pod's drop-in path. Nothing printed or load-tested.

Round 13 (Nov 4, Fernando's review of v12):
 * the hub overlapped all four ring edges by 3.6 cm3 each: the hub-node half-laps (a +-20 deg sector) reached into the ring band, where the
   edge-node laps also are, and both gave their lower half to different parts. The check compared the chassis against the pods only, never
   its parts against each other; it now does (every pair), and asserts that no two lap zones meet;
 * splits are square to the members (the hub ends at |x|, |y| = HUB_SPLIT on the cross; the edges at |x -+ y| = EDGE_SPLIT across the ring),
   and every half-lap is a dovetail half-lap: each half's end is a dovetail in plan in the other part's socket, so the joint takes tension
   along the member, not only the screws; the parts still go together vertically;
 * the hub is beefier (cross 24 wide, was 16; disc r 34, was 20) and every inside corner of the frame is rounded (R FRAME_FILLET in plan);
 * hole audit (hardware.audit): no hole is a half circle (the pods' third foot screws straddled the hub / node split, the pedestal's ran along
   its tube wall, the D435's cut the cradle's cheeks) and no thread-forming pilot runs across a part's layers. The pods' screws go into hex
   nuts (pod_lean round 13); the bumper hangs on tabs at its foot, screwed from below into vertical pilots (were radial pilots in flat-printed
   nodes; the tabs print on the bed, the lips mid-height needed support); the column bolt comes up from below with the others;
 * pocket walls: plan outlines with rounded ends and corners, flush with the pad they stand on (were 1 mm proud of it), top edges rounded;
 * D435 cradle redrawn: base, a back plate on three webs, filleted; the camera's two M3 heads sit on the plate's back clear of everything;
   the USB-C plug is straight out of the camera's left end (as on the bench), its lead bending up.

Round 14 (Nov 4, Fernando's review of round 13):
 * no thin whispers of material: nothing in a frame part narrower than MIN_FEATURE (4 mm) in plan, and no corner sharper than ~75 deg
   (thin_of / no_thin): on the skeleton (rim arcs' needle tips), per part (a fillet's cusp cut off by a split line goes to the neighbour it
   belongs with: merge_thin), per layer of each part after the laps, and on the pocket walls where a split or the slide path cuts them.
   The ring's dovetails are smaller (their socket walls were 2.9 mm). The check reports what is left (thin_mm2);
 * every dovetail socket is DOVETAIL_CLR (0.1) bigger than its tail on each face (lap_layers);
 * the gap in the ring under each inner frame's pivot end is closed (the pod's pivot block dipped 1 mm under its foot: pod_lean round 14);
 * the D435 is tilted 7.5 deg down (was 30), the sim's robot_export.D435_TILT, at the sim's pose (190, 0, 111.5; round 13 had it at 123.5)
   with an L-shaped USB-C plug (Fernando), its head <= 8 mm off the camera, the lead straight up: 4.2 mm from the FR rollers at bump.

Round 15 (Nov 4, Fernando):
 * hub and the four ring edges are one part (ring + cross + disc): the nodes now start on their stems (NODE_SPLIT), one dovetail half-lap
   each; the hub alone was never joined to an edge (3.4 mm apart, both lapped to the nodes). Five frame parts (were nine);
 * the spine runs on forward past the centre to x 70 (the lidar is at the front), under the arm riser's rear edge, with a third foot on the
   hub's front cross arm; its scan-band web starts nearer the lidar (shadow 2.0 -> ~5 deg);
 * two front fins on node_front's arc (FRONT, bearing +-36 from the lidar), beside the D435 and behind its lenses (outside both FOVs),
   clear of the front pods' way out; each held by an M3 x 30 up into its foot and one long M3 from the second floor's top down through the
   fin into a pilot in the node, clamping floor, fin and node;
 * every fin's web in the scan band is a wedge in plan: as wide as its own lidar shadow (the 5 mm near end's), so the material is where the
   lidar is blind anyway (capped at the flange's 16 mm);
 * the side fins run on to u 238 with a second leg (a ~9 mm column in their shadow) on the side node beside the rear pod's keel pocket;
 * the D435's depth and RGB fields of view as envelopes (env_d435_fov_*; lens positions VERIFY).
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import build123d as b
from shapely import affinity, union_all
from shapely.geometry import LineString, Point, Polygon, box as sbox

sys.path.insert(0, str(Path(__file__).resolve().parent))
import base as B                                                       # noqa: E402
import first_floor as FF                                               # noqa: E402
import hardware as H                                                   # noqa: E402
import pod as P                                                        # noqa: E402
import pod_lean as L                                                   # noqa: E402
import wheel_motor as W                                                # noqa: E402
from dims import R_WHEEL, M3_PILOT, M3_CLR, DOVETAIL_CLR, MIN_FEATURE  # noqa: E402
from step_colour import write_part, write_step                         # noqa: E402

OUT = W.ROOT / 'step_export' / 'chassis'
VAL = W.ROOT / 'validation' / 'chassis'
U, cyl, box, pan_screw = FF.U, W.cyl, P.box, FF.pan_screw
Z0, Z1 = 40.0, FF.PLATE_Z[1]               # members: bottom (ground clearance) and top (the pods' feet)
ZL = (Z0 + Z1) / 2                         # half-lap line
RING_IN, RING_OUT = 100.0, 121.0           # |x| + |y| band of the ring (its outer edge is the inner arms' cutout)
CROSS_W, HUB_R = 24.0, 34.0                # round 13: beefier (16, 20)
FRAME_FILLET = 8.0                         # round 13: every inside corner of the frame rounded in plan
STEM_W = dict(front=50.0, back=36.0, side=30.0)
ARC_IN = 201.0
ARCS = dict(front=(-30.0, 30.0), left=(62.0, 118.0), back=(150.0, 210.0), right=(242.0, 298.0))   # rim arcs; front/back +-30 deg: 225 mm chords print
NODE_SPLIT = dict(front=120.0, left=126.0, back=130.0, right=126.0)   # round 15: hub + edges one part; each node starts here on its stem
                                           # (front: between the pedestal's screw rows; sides: the side fin's foot screw inside the lap's
                                           # rectangle, so the hub fits the bed turned 45 deg: at 142 it was 256 mm)
                                           # (round 13: hub to |x|, |y| 88 on the cross, edges to |x -+ y| 53 on the ring)
NODE_SPLIT_W = dict(front=40.0, left=25.0, back=28.0, right=25.0)     # half width of the hub's reach along each stem (front: the pedestal)
LAP = 12.0                                 # half-lap length either side of each split
TAIL = dict(ring=(2.5, 3.4, 5.0), cross=(4.0, 6.5, 7.0))   # round 13: dovetail on each lap end: neck and flare half widths, length.
                                                           # Round 14: ring 3.0 / 4.5 / 6 -> 2.5 / 3.4 / 5: the socket's walls beside the
                                                           # flare were 2.9 mm (the ring is 14.8 wide), now 4.0 (MIN_FEATURE)
CRUMB = 1.6                                # round 14: a thin piece counts from this area (mm2): a 90 deg corner's crumb at r 2 is 0.86, a 75 deg one's 1.6
LAP_SCREW = (6.0, -6.0, 9.0, -9.0, 3.0, -3.0)   # lap screws along the lap from its centre: the first two of these that land clear
NODES = ('front', 'left', 'back', 'right')
NODE_DEG = dict(front=0.0, left=90.0, back=180.0, right=270.0)
# pod pockets (pod frame; robot z = pod z + R_WHEEL)
POCKET_CLR, POCKET_WALL, POCKET_TOP = 0.2, 5.0, 6.0                    # wall top at pod z 6 = robot 76 (inner frame); keel walls to its top
# round 11 (Fernando): a brace per pod from the keel's pocket on the side node to the ring by the inner frame, parallel to the pivot axis
# behind the fork's base bar: it closes a triangle with the side stem and the rim arc, and ties the two pivot supports directly
BRACE = dict(x=(-43.5, 84.0), y=(96.0, 110.0))                          # pod frame; clipped 3 mm off everything that moves like every member
# round 12: each pod comes out radially (its -x) with the second floor on: its screws come up from under the members (heads 1.4 mm up in
# counterbores), and the members and pocket walls are clipped 1 mm off the path every pod part (all poses) sweeps going out
BELOW_CB = 3.9                             # counterbore from the members' underside for those heads (pan 6.5 x 2.5)
SLIDE_L, SLIDE_CLR = 260.0, 1.0
WALL_ROUND, WALL_CORNER, WALL_TOP_R = 2.4, 6.0, 1.5   # round 13: pocket walls' free ends / outer corner (plan) and top edges
# lidar
LIDAR_X = 120.0
PED_FLANGE_R, PED_R, PED_WALL, PED_TOP_R, PED_SCREW_R = 31.0, 20.0, 3.0, 36.5, 25.5   # round 13: screws outside the tube (were r 17, on its
                                           # wall); top plate r 36.5 (was 40: 2.0 mm from the FL rollers at bump)
# second-floor supports: printed fins, each in a vertical plane through the lidar's axis. u: distance from that axis along the fin's ray
FLOOR2_UNDER = 176.5                       # second floor underside (layout F; arm riser 62..205 x +-56 on top of it)
FLOOR2_TOP = FLOOR2_UNDER + 5.0            # its top (upper.Z_TOP)
COLUMN_TOP = P.Z_TT + R_WHEEL + 0.02       # the pods' spring-column top (inner frame), 126.9; the side fin bears on it (0.02 keeps the booleans clean)
SUPPORT_PLANE_BAND = (148.0, 170.0)        # only the web crosses this band (robot z); the scan plane is 163. Round 13: top 172 -> 170, so
                                           # the flange (from 172.5) holds the floor screws' 7 mm pilots (they ran into the 5 mm web: 1.1 mm wall)
FIN_WEB, FIN_FLANGE = 5.0, 16.0           # web in the scan band; top flange under the floor
FIN_LEG = dict(spine=(4.0, 7.0), side=(4.0, 4.5), front=(4.0, 4.5))   # leg half width from z 120 up / at the deck (the side foot stands
                                                                       # between the pods' forks)
SPINE = dict(bearing=180.0, top=(50.0, 336.0), feet=((70.0, 95.0), (155.0, 185.0), (300.0, 336.0)), apex=(120.0, 125.0), nose=166.0,
             screws=(74.0, 91.0, 163.0, 177.0, 310.0, 326.0), floor=(80.0, 182.0, 230.0, 275.0, 328.0))
             # round 13: floor screws off the battery tray (148 ..). Round 15 (Fernando): on forward past the centre (top from u 140, x -20, to
             # u 50, x 70: 8 mm under the arm riser's rear edge, 12 mm from the lidar), a third foot on the hub's front cross arm (x 25..50);
             # the new floor screw at x 40 sits in a dovetail tab on the floor's front seam (upper.SEAMS); none under the battery tray
             # (x -63.5..11.5)
SIDE = dict(bearing=136.7, top=(78.0, 238.0), column=(105.5, 133.5), arch=136.0, foot=(165.0, 175.0), leg_shift=-4.0, nose=166.0,
            tail_leg=(226.0, 238.0), tail_apex=135.0, screws=(170.0, 232.0), floor=(86.0, 119.0, 155.0, 205.0, 232.0))
            # bearing: the ray through both column pilots; its tip is under the riser's corner; round 13: floor screw 160 -> 155 (it was 1.7 mm
            # off the floor's side seam). Round 15 (Fernando: a column in its shadow): on to u 238 (was 225, a cove hanging at z 168) with a
            # second leg at u 226..238 on the side node, the legs shifted 4 mm off the rear pod (was 2.5); past u 248 the rear pod's keel
            # fills the shadow up to z 86, and the power stack's corner screw (-60, -164) stands at u 243.5
FRONT = dict(bearing=36.0, top=(84.0, 109.0), foot=(93.0, 109.0), knee=134.0, nose=166.0, web=6.0, screws=(97.5,), bolt=105.0, floor=())
            # round 15 (Fernando): two fins on node_front's arc (r 201..218) beside the D435, bearing +-36 from the lidar: behind its lens plane
            # and >= 25 mm outside its lenses sideways (no FOV), 5 mm off its side, the right one beside and above the USB lead (the fin
            # reaches in only above the knee, to u 84: at 80 it was 1.1 mm from the lead); +-38 leaves < 4 mm to the front pods' way out.
            # Web 6 at its near end (the long bolt's walls: 2.1 mm at u 105). The foot: M3 x 30 up from below; the bolt: M3 x LONG_BOLT_L down from the floor's top into the node
LONG_BOLT_L, LONG_BOLT_PILOT = 130.0, 11.3   # pan head on the floor top (181.5): tip at z 51.5, 10.5 mm into a vertical pilot in the node
FINS = (('support_spine', 'spine', 1), ('support_side_L', 'side', 1), ('support_side_R', 'side', -1),
        ('support_front_L', 'front', 1), ('support_front_R', 'front', -1))
FIN_DIMS = dict(spine=SPINE, side=SIDE, front=FRONT)
SPINE_FLOOR_OFF = 0.0                      # round 13: the spine's floor screws stay on its plane (off it they left the 16 mm flange for the 5 mm
                                           # web: 3.6 mm of thread); the floor's rear seam carries each one on a dovetail tab instead (upper.py)
FIN_SCREW_L, FIN_PILOT_DEPTH, FLOOR_PILOT_DEPTH = 30.0, 12.0, 7.0      # deck feet: M3 x 30 up through the member; floor: M3 x 12 down
PIN_D, PIN_L, PIN_PROUD = 3.0, 10.0, 4.0  # steel dowels pressed into the column's two support pilots, locating the side fin's foot
# bumper: impact only
BUMPER_T = 3.0
FF.MOUNTS_Q0 = (8.0, 14.0, 72.0, 84.0)    # round 12: 25 -> 17, the front / back arcs now end at 20 deg (clear of the pods' way out); round 13: 14 (the tab)
TAB_REACH, TAB_W, TAB_T, TAB_SCREW_L = 10.0, 16.0, 5.0, 10.0   # round 13: a tab at the bumper's foot per mount, in a recess under the node's
                                                                # arc, one M3 x 10 up from below into a vertical pilot
# D435 cradle (round 13; camera pose: first_floor.D435_C / D435_TILT). Round 13 put the camera 12 mm over the sim's ARM_D435 (z 111.5): with
# the straight USB-C plug (21 mm, as on the bench) its lead must clear the FR wheel at bump by 4 mm. Round 14: tilted 7.5 deg down, as the
# sim (robot_export.D435_TILT, design.md 4.11; was 30)
FF.D435_C = (190.0, 0.0, 111.5)
FF.D435_TILT = 7.5
CRADLE = dict(x=(155.0, 179.0), half_w=22.0, base_t=5.0, plate_t=4.0, plate_half_w=26.0, webs=((15.0, 22.0), (-3.5, 3.5), (-22.0, -15.0)),
              screws=((163.0, 9.25), (163.0, -9.25)), screw_l=10.0, cam_screw_l=8.0, edge_r=1.5)
USB_PLUG = dict(l=8.0, across=12.4, deep=6.5, from_back=3.25, lead_d=4.5, lead_in=3.5, rise=30.0)   # round 14 (Fernando): an L-shaped
                                           # USB-C plug, the lead straight up out of the head 3.5 from its end. Buy one whose head stands <= 8 mm
                                           # off the camera (VERIFY): 9 leaves 3.3 mm to the FR rollers at bump. Round 13: a straight plug, 21
                                           # long, lead bent up, which needed the camera at z 123.5
# round 15 (Fernando): the D435's fields of view, drawn as envelopes FOV_L long: (h deg, v deg, lens in the camera frame: x from the camera's
# centre along its view, y toward its left). Intel's D435 figures: depth 87 x 58 from the left imager (the depth origin, 4.2 mm behind the
# front glass), RGB 69 x 42. The lens positions (17.5 / 32.5 mm left of the centre) are VERIFY against the datasheet and the unit
D435_FOV = dict(depth=(87.0, 58.0, (12.5 - 4.2, 17.5)), rgb=(69.0, 42.0, (12.5 - 4.2, 32.5)))
FOV_L = 500.0
COLOURS = dict(FF.COLOURS, hub=(0.50, 0.52, 0.56), edge=(0.70, 0.72, 0.62), node=(0.62, 0.68, 0.78), support=(0.85, 0.85, 0.88),
               fov_depth=(0.30, 0.55, 0.95), fov_rgb=(0.35, 0.85, 0.40))


def colour(name):
    n = name.split('/')[-1]
    for k in ('hub', 'edge', 'node', 'support'):
        if n.startswith(k):
            return COLOURS[k]
    if n.startswith('env_d435_fov_'):
        return COLOURS['fov_' + n[13:]]
    if n.startswith('env'):
        return FF.COLOURS['env']
    return FF.colour(name)


def diamond(r):
    return Polygon([(r, 0), (0, r), (-r, 0), (0, -r)])


def sector(a0, a1, r=400.0):
    return Polygon([(0, 0)] + [tuple(r * U(t)) for t in np.linspace(a0, a1, 24)])


def prism(geom, z0, z1):
    """round 12: opened by 0.05 first; the side nodes' outline (valid in shapely) made an invalid solid after the slide-path clip"""
    return B.solid_of(geom.buffer(-0.05, join_style=2).buffer(0.05, join_style=2), z0, z1)


# ---- pod pockets (pod frame) ------------------------------------------------------------------------------------------------------------------
def rounded(g, r):
    """convex corners of a plan outline rounded to r"""
    return g.buffer(-r, 32, join_style=2).buffer(r, 32)


def frame_outline():
    """round 13, pod frame: the inner frame's pocket (foot + clearance + walls on its centre side and front end), corners rounded; the walls'
    outer faces and the pad under them share this outline"""
    x1 = L.FI_X[1] + POCKET_CLR
    return rounded(sbox(L.FI_X[0], L.FOOT_Y_L[0] - POCKET_CLR - POCKET_WALL, x1 + POCKET_WALL, L.FOOT_Y_L[1]), WALL_CORNER)


def keel_outline():
    kx, ky = L.KEEL_FOOT['x'], L.KEEL_FOOT['y']
    return rounded(sbox(kx[0], ky[0] + 2.0, kx[1] + POCKET_CLR + POCKET_WALL, ky[1] + POCKET_CLR + POCKET_WALL), WALL_CORNER)   # 2 mm: the base bar


def wall_solid(g, z0, z1):
    """a pocket wall from its plan (free ends rounded) with its top edges rounded"""
    g = g.buffer(-WALL_ROUND, 32).buffer(WALL_ROUND, 32)
    s = B.solid_of(g, z0, z1)
    s, _ = L.round_edges_each(s, [e for e in s.edges() if all(abs(e.position_at(t).Z - z1) < 1e-3 for t in (0, 0.5, 1))], WALL_TOP_R)
    return s


def frame_walls():
    """walls hugging the inner frame: along its centre-side face and across its front end (round 12: no horizontal screws through them)"""
    x1 = L.FI_X[1] + POCKET_CLR
    g = frame_outline().difference(sbox(L.FI_X[0] - 20, L.FOOT_Y_L[0] - POCKET_CLR, x1, L.FOOT_Y_L[1] + 20))
    return wall_solid(g, L.PLATE_TOP_L, POCKET_TOP), []


def keel_walls():
    kx, ky = L.KEEL_FOOT['x'], L.KEEL_FOOT['y']
    g = keel_outline().difference(sbox(kx[0] - 20, ky[0] - 20, kx[1] + POCKET_CLR, ky[1] + POCKET_CLR))
    return wall_solid(g, L.PLATE_TOP_L, L.KEEL_TOP_Z)                  # round 11: up to the keel's top (was its foot's, 7)


def slide_path():
    """round 12, pod frame plan: what every pod part (droop, nominal, bump) sweeps going out along -x by SLIDE_L, + SLIDE_CLR, in the members'
    band (robot z Z0..Z1) and the pocket walls' (Z1..Z1 + 30). Cached; rebuilt when the pod's sources change"""
    import pickle
    srcs = [Path(__file__).with_name(n) for n in ('pod.py', 'pod_lean.py', 'dims.py', 'wheel_motor.py', 'chassis.py')]
    key = [round(s.stat().st_mtime) for s in srcs]
    cache = VAL / 'slide_path.pkl'
    if cache.exists():
        k, out = pickle.loads(cache.read_bytes())
        if k == key:
            return out
    sp, _ = L.build_static()
    unit = P.wheel_unit()
    bands = dict(members=(Z0, Z1 - 0.05), walls=(Z1 + 0.05, Z1 + 30.0))
    secs = {nm: [] for nm in bands}
    for a in L.POSES.values():
        sh, _ = L.posed(sp, unit, a)
        for k, s in sh.items():
            for nm, (z0, z1) in bands.items():
                if nm == 'members' and not k.startswith(L.STATIC):        # round 13: moving parts to 0.5 over the members' top (the outer
                    z1 = Z1 + 0.5                                        # arm hangs 0.07 below it at droop and scraped the left node's arc)
                g = B.proj(s, True, z0 - R_WHEEL, z1 - R_WHEEL)
                if g is not None:
                    secs[nm].append(g)
    out = {}
    for nm, gs in secs.items():
        g = union_all(gs)
        quads = [g, affinity.translate(g, -SLIDE_L, 0)]
        for q in getattr(g, 'geoms', [g]):
            for ring in [q.exterior] + list(q.interiors):
                c = list(ring.coords)
                quads += [Polygon([a, b_, (b_[0] - SLIDE_L, b_[1]), (a[0] - SLIDE_L, a[1])]).buffer(0) for a, b_ in zip(c[:-1], c[1:])]
        out[nm] = union_all(quads).buffer(SLIDE_CLR, 16)
    VAL.mkdir(parents=True, exist_ok=True)
    cache.write_bytes(pickle.dumps((key, out)))
    return out


# ---- frame geometry (2D) ---------------------------------------------------------------------------------------------------------------------
def keel_foot():
    """the outer keel's foot in plan (pod frame), from 2.8 outboard of the keel (round 9's pad) to the foot's end"""
    (x0, x1), (y0, y1) = L.KEEL_FOOT['x'], L.KEEL_FOOT['y']
    return sbox(x0 - 2.8, y0, x1, y1)


def skeleton(fp, BR):
    rb = B.r_bumper(fp)
    rp = rb - 0.5
    cut = FF.placed(fp['cutout'])
    below = FF.placed(fp['bands']['below']).buffer(3.0)                 # moving parts in z 40..56, +3
    plate = FF.placed(fp['bands']['plate']).buffer(3.1)                 # round 13: moving parts in z 56..62 too (the rounded corners filled toward them)
    allowed = Point(0, 0).buffer(rp, 720).difference(cut).difference(below).difference(plate).difference(FF.placed(slide_path()['members']))
    ring = diamond(RING_OUT).difference(diamond(RING_IN))
    cross = union_all([sbox(-RING_IN - 2, -CROSS_W / 2, RING_IN + 2, CROSS_W / 2), sbox(-CROSS_W / 2, -RING_IN - 2, CROSS_W / 2, RING_IN + 2)])
    hub = Point(0, 0).buffer(HUB_R, 64)
    stems = [affinity.rotate(sbox(RING_IN, -STEM_W[k if k in ('front', 'back') else 'side'] / 2, rp, STEM_W[k if k in ('front', 'back') else 'side'] / 2),
                             NODE_DEG[k], origin=(0, 0)) for k in NODES]
    annulus = Point(0, 0).buffer(rp, 720).difference(Point(0, 0).buffer(ARC_IN, 720))
    arcs = [annulus.intersection(sector(*ARCS[k])) for k in NODES]
    keel_pad = union_all([FF.placed(keel_outline()), FF.placed(keel_foot())])   # round 13: flush with the pocket walls (was 1 mm proud)
    frame_pad = FF.placed(frame_outline())
    brace = FF.placed(sbox(BRACE['x'][0], BRACE['y'][0], BRACE['x'][1], BRACE['y'][1]))
    cradle = sbox(*B.proj(BR['bracket'], False).buffer(3).bounds)
    ped = Point(LIDAR_X, 0).buffer(PED_FLANGE_R + 3.0, 64)
    sk = union_all([ring, cross, hub] + stems + arcs + [brace, cradle, ped])
    sk = sk.buffer(FRAME_FILLET, 32).buffer(-FRAME_FILLET, 32)          # round 13: inside corners rounded
    sk = union_all([sk, keel_pad, frame_pad]).intersection(allowed)
    sk = sk.buffer(-0.3, 16, join_style=2).buffer(0.3, 16, join_style=2)   # round 12: drop the slide path's slivers (< 0.6 mm)
    sk = no_thin(sk)                                                    # round 14: and anything narrower than MIN_FEATURE, needle tips
    pieces = sorted(getattr(sk, 'geoms', [sk]), key=lambda g: -g.area)
    sk = union_all([g for g in pieces if g.area > 400])
    return sk, dict(rb=rb, rp=rp, cut=cut, allowed=allowed, pieces=[round(g.area) for g in pieces])


def strip(c, u, half_len, half_w):
    """rectangle centred on c, half_len along unit u, half_w across"""
    n = np.array([-u[1], u[0]])
    return Polygon([tuple(c + s * u * half_len + t * n * half_w) for s, t in ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def dovetail_lap(c, u, kind, clip):
    """round 13: a half-lap zone across a split (centre c on the member's axis, u along it): LAP either side, each end a dovetail in plan
    (narrow at the end line, flaring into the other part), clipped to the member"""
    n_, f_, d_ = TAIL[kind]
    nv = np.array([-u[1], u[0]])
    g = [strip(c, u, LAP, 60.0)]
    for s in (1, -1):
        e = c + s * u * LAP
        g.append(Polygon([tuple(e - s * u * 0.01 + nv * n_), tuple(e + s * u * d_ + nv * f_), tuple(e + s * u * d_ - nv * f_), tuple(e - s * u * 0.01 - nv * n_)]))
    return union_all(g).intersection(clip)


def clean(g, min_area=20.0):
    """round 13: drop slivers (pieces under min_area mm2, and hairlines) that the region and lap booleans leave at their boundaries"""
    g = g.buffer(-0.02, 8, join_style=2).buffer(0.02, 8, join_style=2)
    return union_all([q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and q.area >= min_area] or [Polygon()])


def polys(g):
    return [q for q in getattr(g, 'geoms', [g]) if q.geom_type == 'Polygon' and not q.is_empty]


def thin_of(g, r=MIN_FEATURE / 2):
    """round 14 (Fernando: no thin whispers of material): the pieces of g narrower than 2 r, and the tips of corners sharper than ~75 deg
    (CRUMB): g less its opening at r"""
    o = g.buffer(-r, 16).buffer(r, 16).buffer(0.1, 8)
    return union_all([q for q in polys(g.difference(o)) if q.area > CRUMB] or [Polygon()])


def no_thin(g, r=MIN_FEATURE / 2):
    """round 14: g without its thin pieces"""
    t = thin_of(g, r)
    return g if t.is_empty else clean(g.difference(t.buffer(0.15, 8)))


def merge_thin(regions, sk):
    """round 14: a thin piece of a part's plan (a fillet's cusp left where a split line cuts it, a sliver along a split) goes to the
    neighbouring part it shares most boundary with (dropped if it has none)"""
    for k in list(regions):
        for q in polys(thin_of(regions[k])):
            q = q.buffer(0.15, 8).intersection(regions[k])              # its own (buffered into a neighbour it overlapped a third part)
            best, most = None, 0.05
            for k2, r2 in regions.items():
                a = q.buffer(0.1).intersection(r2).area if k2 != k else 0.0
                if a > most:
                    best, most = k2, a
            regions[k] = clean(regions[k].difference(q))
            if best:
                regions[best] = clean(union_all([regions[best], q]))
    return regions


def split(sk):
    """the five parts' regions (2D) and the half-lap zones between them: (zone, lower part, upper part, centre, axis). Round 15: the hub is
    the ring, cross and disc in one (it fits the bed turned 45 deg), each node starts square across its stem at NODE_SPLIT, one dovetail
    half-lap each (round 13: hub, four ring edges and four nodes, 12 laps)"""
    reach = [affinity.rotate(sbox(0.0, -NODE_SPLIT_W[n], NODE_SPLIT[n], NODE_SPLIT_W[n]), NODE_DEG[n], origin=(0, 0)) for n in NODES]
    regions = {'hub': sk.intersection(union_all([diamond(RING_OUT + 2)] + reach))}
    rest = sk.difference(regions['hub'])
    for n in NODES:
        a = NODE_DEG[n]
        regions[f'node_{n}'] = rest.intersection(sector(a - 45, a + 45))
    laps = []
    for n in NODES:                                                     # hub -> node, across the stem
        u = U(NODE_DEG[n])
        c = u * NODE_SPLIT[n]
        laps.append((dovetail_lap(c, u, 'cross', strip(c, u, 60.0, NODE_SPLIT_W[n]).intersection(sk)), 'hub', f'node_{n}', c, u))
    regions = merge_thin({k: clean(v) for k, v in regions.items()}, sk)
    laps = [(clean(z), *rest_) for z, *rest_ in laps]
    for i, (z, *_) in enumerate(laps):                                  # round 13: no two laps may meet (the v12 overlap)
        for z2, *_ in laps[i + 1:]:
            if z.intersection(z2).area > 0.01:
                raise RuntimeError('lap zones overlap')
    return regions, laps


def lap_layers(regions, laps):
    """round 14: each frame part's plan in the lower (Z0..ZL) and upper (ZL..Z1) layer: its body, plus the lap halves it owns, less a socket
    DOVETAIL_CLR bigger than the other part's half in that layer (round 13: tail and socket the same size), without thin pieces"""
    lap_all = union_all([z for z, *_ in laps if not z.is_empty])
    out = {}
    for k, reg in regions.items():
        body = reg.difference(lap_all)
        lv = []
        for own, other in ((1, 2), (2, 1)):                             # laps tuple: (zone, lower part, upper part, ...)
            mine = [L_[0] for L_ in laps if L_[own] == k and not L_[0].is_empty]
            theirs = [L_[0] for L_ in laps if L_[other] == k and not L_[0].is_empty]
            g = union_all([body] + mine).buffer(0.02, 8, join_style=2).buffer(-0.02, 8, join_style=2)   # close the seams body / zone
            if theirs:
                g = g.difference(union_all(theirs).buffer(DOVETAIL_CLR, 8, join_style=2))
            lv.append(no_thin(clean(g)))
        out[k] = list(lv)
    for n in (0, 1):                                                    # butt joints (split lines outside the laps) DOVETAIL_CLR apart too:
        taken = Polygon()                                               # touching, each part's outline simplification (base.solid_of, 0.05)
        for k in out:                                                   # pushed it 0.4-0.5 mm3 into its neighbour (hub / edge_FR, side nodes)
            out[k][n] = no_thin(clean(out[k][n].difference(taken.buffer(DOVETAIL_CLR, 8))))
            taken = union_all([taken, out[k][n]])
    return {k: tuple(v) for k, v in out.items()}


def lap_screws(laps, avoid):
    """two M3 per lap on its axis, the first two of LAP_SCREW offsets whose counterbore stays 2.5 inside the lap's rectangle and 8 mm off
    every other hole (avoid)"""
    pts = []
    for z, lo, hi, c, u in laps:
        rect = z.intersection(strip(c, u, LAP, 60.0)).buffer(-(FF.CB_D / 2 + 1.0))
        got = []
        for t in LAP_SCREW:
            p = c + u * t
            if rect.contains(Point(*p)) and all(math.dist(p, q) >= 8.0 for q in avoid + [g for g, _, _ in got]):
                got.append((tuple(p), lo, hi))
            if len(got) == 2:
                break
        pts += got
    return pts


# ---- parts ----------------------------------------------------------------------------------------------------------------------------------------
def build_pedestal():
    z0, z1 = Z1, FF.LIDAR_BASE_Z
    c = (LIDAR_X, 0.0)
    p = cyl(PED_FLANGE_R, 4.0, (*c, z0 + 2.0)) + cyl(PED_R, z1 - z0, (*c, (z0 + z1) / 2)) + cyl(PED_TOP_R, 5.0, (*c, z1 - 2.5))
    p = p - cyl(PED_R - PED_WALL, z1 - z0 - 5.0 + 0.1, (*c, (z0 + z1 - 5.0) / 2))
    for a in (45, 135, 225, 315):
        x, y = np.array(c) + U(a) * PED_SCREW_R
        p = p - cyl(M3_CLR / 2, 4.2, (x, y, z0 + 2.0))
        x, y = np.array(c) + U(a) * FF.LIDAR_HOLES_R
        p = p - cyl(1.35, 5.2, (x, y, z1 - 2.5))
    p = p - box(c[0] - PED_R - 1, c[0] - PED_R + PED_WALL + 1, -6, 6, z1 - 30, z1 - 5)        # the lidar's lead out of the back of the tube
    p = p - cyl(8.0, 5.2, (*c, z1 - 2.5))
    return p


def bumper_tab(a, rp, rb, grow=0.0):
    """round 13: plan of the tab at the bumper's foot at mount angle a: from the bumper wall in under the node's arc"""
    return affinity.rotate(sbox(rp - TAB_REACH - grow, -TAB_W / 2 - grow, rb + 0.5, TAB_W / 2 + grow), a, origin=(0, 0))


def build_bumpers(rb, rp, sk):
    """round 13: each mount is a tab at the bumper's foot (z Z0..Z0 + TAB_T, printed on the bed) in a recess under the node's arc, one M3 x 10
    pan up from below through the tab (counterbored) into a vertical pilot in the arc. Off: the screws out from below, slide the quarter out
    radially. (Round 12: lips mid-height on the plate top, which needed support, and countersunk radial screws into horizontal pilots.)
    Returns the quarters, the screws and the recesses to cut from the nodes"""
    ro = rb + BUMPER_T
    ring = prism(Point(0, 0).buffer(ro, 720).difference(Point(0, 0).buffer(rb, 720)), *FF.BUMPER_Z)
    ring = ring - prism(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(-FF.NOSE_DIP_DEG, FF.NOSE_DIP_DEG, 9)]), FF.NOSE_DIP_TOP, FF.BUMPER_Z[1] + 1)
    out, hw, cuts = {}, {}, []
    mounts = FF.mount_angles()
    for q in range(4):
        s = ring & prism(Polygon([(0, 0)] + [tuple(1.5 * ro * U(t)) for t in np.linspace(90 * q, 90 * q + 90, 12)]), 0, 200)
        for a in mounts:
            if not (90 * q < a < 90 * q + 90):
                continue
            if affinity.rotate(sbox(rp - TAB_REACH - 0.2, -TAB_W / 2 - 1, rp - 0.5, TAB_W / 2 + 1), a, origin=(0, 0)).difference(sk).area > 0.01:
                raise RuntimeError(f'bumper tab at {a:g} deg has no arc over it')
            s = s + prism(bumper_tab(a, rp, rb), Z0, Z0 + TAB_T)
            x, y = U(a) * (rp - TAB_REACH / 2)
            s = s - cyl(M3_CLR / 2, TAB_T + 0.2, (x, y, Z0 + TAB_T / 2)) - cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z0 + FF.CB_DEPTH / 2 - 0.05))
            cuts.append((prism(bumper_tab(a, rp, rb, 0.2), Z0 - 0.1, Z0 + TAB_T + 0.2), (x, y)))
            hw[f'bolt_bumper_{a:g}'] = pan_screw((x, y, Z0 + FF.CB_DEPTH), (0, 0, 1), TAB_SCREW_L)
        out[f'bumper_q{q}'] = s
    return out, hw, cuts


D435_STEP = W.ROOT.parent / 'v10 human editable' / 'reference' / 'd435_source.step'   # X width, Y up, Z toward the lenses, front Z 0, back -25


def d435_real(loc):
    """round 13: the D435's own model in the camera frame (x along its view, back face x -12.5), moved by loc"""
    s = b.import_step(str(D435_STEP))
    s = b.Compound([q for q in s.solids()]) if not isinstance(s, b.Solid) else s
    T = np.eye(4)
    T[:3, :3] = [[0, 0, 1.0], [1.0, 0, 0], [0, 1.0, 0]]               # source (X, Y, Z) -> camera (y, z, x)
    T[:3, 3] = (FF.D435_SIZE[0] / 2, 0, 0)
    return s.moved(W.loc(T[:3, :])).moved(loc)


def d435_cradle():
    """round 13: the D435 on the front node. Camera frame (first_floor): x along its view (forward-down D435_TILT), y across, z its up.
    A 5 mm base on the node (2 x M3 x 10 down into vertical pilots, counterbored); a 4 mm back plate behind the camera, its two M3 x 8 heads
    on the plate's back (nothing behind them: a driver comes in along the camera's axis); three webs from the base up under the plate (the
    outer two flush with the base's sides, so base and web are one face); the plate's wings past the webs chamfered 45 deg underneath, so it
    prints on its base without support. Edges rounded (R CRADLE['edge_r'] where OCC takes it). The USB-C plug is an L out of the camera's
    -y end (centred 3.25 in from its back face, as the vendor model's port housing), its lead straight up toward the floor's grommet"""
    K = CRADLE
    cx, cy, cz = FF.D435_C
    t = math.radians(FF.D435_TILT)
    R = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])   # camera frame -> robot
    view, up = R[:, 0], R[:, 2]
    dpt, wid, hgt = FF.D435_SIZE
    c = np.array(FF.D435_C)
    loc = b.Location((cx, cy, cz), (0, 1, 0), FF.D435_TILT)
    to_r = lambda xc, zc: c + R @ np.array([xc, 0.0, zc])                                  # noqa: E731
    camera = box(-dpt / 2, dpt / 2, -wid / 2, wid / 2, -hgt / 2, hgt / 2).moved(loc)
    z0, zb = Z1, Z1 + K['base_t']
    xb0, xb1 = K['x']
    base = box(xb0, xb1, -K['half_w'], K['half_w'], z0, zb)
    xr = -dpt / 2 - K['plate_t']
    plate = box(xr, -dpt / 2, -K['plate_half_w'], K['plate_half_w'], -hgt / 2, hgt / 2).moved(loc)
    qb, pb, pt = to_r(-dpt / 2, -hgt / 2), to_r(xr, -hgt / 2), to_r(xr, hgt / 2)          # plate: front-bottom, back-bottom, back-top
    web_xz = [(xb0, z0), (qb[0], z0), (qb[0], qb[2]), (pt[0], pt[2]), (pb[0], pb[2])]
    br = base + plate
    for y0, y1 in K['webs']:
        br = br + b.extrude(b.Face(b.Wire.make_polygon([b.Vector(x, 0, z) for x, z in web_xz], close=True)), amount=y1 - y0).moved(b.Location((0, y1, 0)))
    for s_ in (1, -1):                                                  # 45 deg under the wings past the outer webs
        y0 = s_ * K['half_w']
        tri = b.extrude(b.Face(b.Wire.make_polygon([b.Vector(0, y0, pb[2] - 0.01), b.Vector(0, y0 + s_ * 20, pb[2] - 0.01),
                                                    b.Vector(0, y0 + s_ * 20, pb[2] + 20)], close=True)), amount=60.0).moved(b.Location((pb[0] - 20, 0, 0)))
        br = br - tri
    br = br - camera
    real = d435_real(loc)                                               # the camera's own model (0.25 mm deeper underneath than the box), 0.3 clear
    for dv in ((0, 0, 0), (-0.3, 0, 0), (0, 0, -0.3), (-0.3, 0, -0.3)):
        br = br - real.moved(b.Location(tuple(R @ np.array(dv))))
    br, rounded_ = L.round_edges_each(br, [e for e in br.edges() if not all(abs(e.position_at(q).Z - z0) < 1e-3 for q in (0, 0.5, 1))], K['edge_r'])
    hw = {}
    for n, cy_ in enumerate((-22.5, 22.5)):                             # the camera's two rear M3 (vendor: 45 apart, 5 deep)
        p = c - view * dpt / 2 + np.array([0, cy_, 0])
        br = br - b.Solid.make_cylinder(M3_CLR / 2, K['plate_t'] + 2.0, b.Plane(origin=tuple(p - view * (K['plate_t'] + 1.0)), z_dir=tuple(view)))
        hw[f'bolt_d435_{n}'] = pan_screw(tuple(p - view * K['plate_t']), tuple(view), K['cam_screw_l'])
    for n, (x, y) in enumerate(K['screws']):
        br = br - cyl(M3_CLR / 2, K['base_t'] + 0.2, (x, y, (z0 + zb) / 2)) - cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, zb - FF.CB_DEPTH / 2 + 0.05))
        hw[f'bolt_bracket_{n}'] = pan_screw((x, y, zb - FF.CB_DEPTH), (0, 0, -1), K['screw_l'])
    P_ = USB_PLUG                                                       # camera frame: the head out along -y from the end face
    xc = -dpt / 2 + P_['from_back']
    plug = box(xc - P_['deep'] / 2, xc + P_['deep'] / 2, -wid / 2 - P_['l'], -wid / 2, -P_['across'] / 2, P_['across'] / 2)
    a = c + R @ np.array([xc, -wid / 2 - P_['l'] + P_['lead_in'], P_['across'] / 2 - 0.5])   # the lead: robot up out of the head's top
    rise = b.Solid.make_cylinder(P_['lead_d'] / 2, P_['rise'], b.Plane(origin=tuple(a), z_dir=(0, 0, 1)))
    plug_r = plug.moved(loc) + rise
    return dict(bracket=br, camera=camera, plug=plug_r, deck_holes=list(K['screws']), hardware=hw, rounded=rounded_)


def d435_fov():
    """round 15 (Fernando): the D435's depth and RGB fields of view as solid frustums FOV_L long from each lens (envelopes, not parts)"""
    t = math.radians(FF.D435_TILT)
    R = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])   # camera frame -> robot (d435_cradle)
    out = {}
    for k, (h, v, (xl, yl)) in D435_FOV.items():
        o = np.array(FF.D435_C) + R @ np.array([xl, yl, 0.0])
        th, tv = math.tan(math.radians(h / 2)), math.tan(math.radians(v / 2))
        rects = []
        for dist in (0.5, FOV_L):                                       # a 0.5 mm deep start, so the loft has two real rectangles
            pts = [o + R @ np.array([dist, sy * dist * th, sz * dist * tv]) for sy, sz in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
            rects.append(b.Wire.make_polygon([b.Vector(*p) for p in pts], close=True))
        out[f'env_d435_fov_{k}'] = b.Solid.make_loft(rects)
    return out


def ellipse(c, a, bz, t0, t1, n=24):
    return [(c[0] + a * math.cos(math.radians(t)), c[1] + bz * math.sin(math.radians(t))) for t in np.linspace(t0, t1, n)]


def fin_profile(kind):
    """the fin's outline in its own plane (u, robot z): a top flange the whole length under the floor, legs on the chassis, elliptical arches
    between them and elliptical coves out to the tips; corners rounded"""
    zf, zd, zc = FLOOR2_UNDER, Z1, COLUMN_TOP
    if kind == 'spine':                    # feet on the hub's front arm, the cross by the hub and the back node, arched between; the nose a
        S = SPINE                          # cove down to the front foot
        (u0, u1), feet = S['top'], S['feet']
        pts = [(u0, zf), (u1, zf), (u1, zd), (feet[-1][0], zd)]
        for (a0, a1), (b0, _), apex in reversed(list(zip(feet[:-1], feet[1:], S['apex']))):
            pts += ellipse(((a1 + b0) / 2, zd), (b0 - a1) / 2, apex - zd, 0, 180)
        pts += [(feet[0][0], zd)] + ellipse((u0, zd), feet[0][0] - u0, S['nose'] - zd, 0, 90)
    elif kind == 'side':                   # nose under the arm riser's corner, bearing on the column top, an arch down to a foot on the ring
        S = SIDE                           # by the side stem, an arch on to the tail leg (round 15; was a cove out to a hanging tail)
        (u0, u1), (c0, c1), (f0, f1), ua, (t0, _) = S['top'], S['column'], S['foot'], S['arch'], S['tail_leg']
        pts = [(u0, zf), (u1, zf), (u1, zd), (t0, zd)]
        pts += ellipse(((f1 + t0) / 2, zd), (t0 - f1) / 2, S['tail_apex'] - zd, 0, 180)
        pts += [(f0, zd)] + ellipse((ua, zd), f0 - ua, zc - zd, 0, 90)
        pts += [(c0, zc)] + ellipse((c0, S['nose']), c0 - u0, S['nose'] - zc, 270, 180)
    else:                                  # front (round 15): a foot on node_front's arc, its inner face straight up to the knee, then a
        S = FRONT                          # corbel in to the nose under the floor (over the D435's end and its USB lead)
        (u0, u1), (f0, _) = S['top'], S['foot']
        pts = [(u0, zf), (u1, zf), (u1, zd), (f0, zd), (f0, S['knee'])]
        pts += ellipse((f0, S['nose']), f0 - u0, S['nose'] - S['knee'], 270, 180)
    g = Polygon(pts).buffer(0)
    return g.buffer(-1.0, 32).buffer(1.0, 32).buffer(3.0, 32).buffer(-3.0, 32)


def fin_cross_section(kind, side=1):
    """across the fin against z: legs flaring out toward the deck (the side fin's leg also shifted off the rear pod's fork), only the web
    through the scan band, the flange under the floor"""
    (w_up, w_deck), w, f, (b0, b1) = FIN_LEG[kind], FIN_WEB / 2, FIN_FLANGE / 2, SUPPORT_PLANE_BAND
    half = [(w_deck, Z1 - 2.0), (w_up, 120.0), (w_up, b0 - 8.0), (w, b0), (w, b1), (f, b1 + 2.5), (f, FLOOR2_UNDER + 1.0)]
    shift = side * (SIDE['leg_shift'] if kind == 'side' else 0.0)
    dz = lambda z: shift * max(0.0, 120.0 - z) / (120.0 - Z1 + 2.0)                    # noqa: E731
    return Polygon([(x + dz(z), z) for x, z in half] + [(-x + dz(z), z) for x, z in reversed(half)])


def fin_axis(kind, side=1):
    bearing = side * FIN_DIMS[kind]['bearing']
    return np.array([LIDAR_X, 0.0]), U(bearing), bearing


def fin_xy(kind, u, side=1):
    lc, d, _ = fin_axis(kind, side)
    return lc + d * u


def floor_pts(kind, side=1):
    """the floor's screws into a fin's flange (plan); round 13: the spine's alternate either side of its plane (the floor's rear seam)"""
    S = FIN_DIMS[kind]
    _, d, _ = fin_axis(kind, side)
    n = np.array([-d[1], d[0]])
    return [tuple(fin_xy(kind, u, side) + (n * SPINE_FLOOR_OFF * (1 - 2 * (k % 2)) if kind == 'spine' else 0.0)) for k, u in enumerate(S['floor'])]


def wedge(u_near, web, u_far):
    """round 15 (Fernando): the fin's plan in the scan band, fin frame (u from the lidar's axis, v across): the cone of its own lidar shadow
    (web wide at u_near, so no wider shadow than the plain web's), capped at the flange's width"""
    t = web / 2 / u_near
    return Polygon([(0.0, 0.0), (u_far, -u_far * t), (u_far, u_far * t)]).intersection(sbox(0.0, -FIN_FLANGE / 2, u_far, FIN_FLANGE / 2))


def build_fin(kind, side=1):
    """the fin solid in robot coordinates, its holes, hardware, its plan section at the scan plane, its deck screws and its long bolts"""
    prof = fin_profile(kind)
    S = FIN_DIMS[kind]
    w = FIN_FLANGE / 2 + 1.0
    s = B.solid_of(prof, -w, w).rotate(b.Axis.X, 90)                                  # (u, z, across) -> (u, -across, z)
    u0, u1 = S['top']
    zp, (b0, b1) = FF.LIDAR['plane'], SUPPORT_PLANE_BAND
    band = prof.intersection(sbox(-1e3, zp - 0.5, 1e3, zp + 0.5)).bounds
    web = S.get('web', FIN_WEB)
    cone = wedge(band[0], web, u1 + 2.0)                                                # round 15: through the band and its transitions
    s = s & (B.solid_of(fin_cross_section(kind, side), u0 - 2.0, u1 + 2.0).rotate(b.Axis.X, 90).rotate(b.Axis.Z, 90) + B.solid_of(cone, b0 - 8.0, b1 + 2.5))
    lc, d, bearing = fin_axis(kind, side)
    s = s.rotate(b.Axis.Z, bearing).moved(b.Location((*lc, 0)))
    nv = np.array([-d[1], d[0]])
    hw, feet, long_ = {}, [], []
    for n, u in enumerate(S['screws']):                                                 # deck feet: M3 x 30 up through the member into the leg
        x, y = fin_xy(kind, u, side)
        foot = s & box(x - 12, x + 12, y - 12, y + 12, Z1, Z1 + 1.0)                      # round 13: centred on the leg's foot (the side
        c = np.array([foot.center().X, foot.center().Y])                                  # fin's legs are shifted off its plane)
        if kind == 'front':                                                             # round 15: across only (the long bolt is 7.5 on)
            c = np.array([x, y]) + nv * float((c - np.array([x, y])) @ nv)
        x, y = c
        feet.append((x, y))
        s = s - cyl(M3_PILOT / 2, FIN_PILOT_DEPTH, (x, y, Z1 + FIN_PILOT_DEPTH / 2 - 0.05))
    if 'bolt' in S:                                                                     # round 15: the long M3 from the floor's top down
        x, y = fin_xy(kind, S['bolt'], side)                                             # through the fin into the node
        s = s - cyl(M3_CLR / 2, FLOOR2_UNDER - Z1 + 0.4, (x, y, (FLOOR2_UNDER + Z1) / 2))
        long_.append((x, y))
        hw[f'bolt_fin_long_{"L" if side > 0 else "R"}'] = pan_screw((x, y, FLOOR2_TOP), (0, 0, -1), LONG_BOLT_L)
    for x, y in floor_pts(kind, side):                                                  # the floor: M3 x 12 down into the flange
        s = s - cyl(M3_PILOT / 2, FLOOR_PILOT_DEPTH, (x, y, FLOOR2_UNDER - FLOOR_PILOT_DEPTH / 2 + 0.05))
    if kind == 'side':                                                                  # dowels in the column's two support pilots
        i = 0 if side > 0 else 3
        out = -B.frame(i)[1]                                                            # round 12: slots open toward the pod's way out
        for n, p in enumerate(L.SUPPORT_PILOTS):
            x, y = B.place2d(Point(*p), i).coords[0]
            slot = LineString([(x, y), (x + 15.0 * out[0], y + 15.0 * out[1])]).buffer((PIN_D + 0.1) / 2, 16, cap_style=2)
            s = s - B.solid_of(union_all([Point(x, y).buffer((PIN_D + 0.1) / 2, 32), slot]), COLUMN_TOP - 0.05, COLUMN_TOP + PIN_PROUD + 0.5)
            hw[f'bolt_pin_{B.NAMES[i]}_{n}'] = cyl(PIN_D / 2, PIN_L, (x, y, COLUMN_TOP + PIN_PROUD - PIN_L / 2))
    sec = cone.intersection(sbox(band[0], -FIN_FLANGE, band[2], FIN_FLANGE))                    # round 15: the wedge at the scan plane
    sec = affinity.translate(affinity.rotate(sec, bearing, origin=(0, 0)), *lc)
    return s, hw, sec, feet, long_


def build_supports():
    parts, hw, sections, screws, long_ = {}, {}, [], [], []
    for name, kind, side in FINS:
        s, h, sec, feet, lb = build_fin(kind, side)
        parts[name] = s
        hw.update(h)
        sections.append((name, sec))
        screws += feet
        long_ += lb
    return parts, hw, sections, screws, long_


LAP_SCREW_L = 16.0                         # round 13 (was 20: its tip ran 0.9 mm out of the member's underside, past the pilot's end)


def build(fp):
    BR = d435_cradle()
    sk, g = skeleton(fp, BR)
    regions, laps = split(sk)
    layers = lap_layers(regions, laps)
    parts = {k: prism(lo, Z0, ZL) + prism(hi, ZL, Z1) for k, (lo, hi) in layers.items()}
    # pod pockets: walls placed per pod, clipped to the members under them and given to the part each piece stands on
    fw, fhw = frame_walls()
    kw = keel_walls()
    hw = {}
    tops = {k: hi for k, (_, hi) in layers.items()}                     # where each part owns the top surface (laps: the upper half's part)
    walls_out = FF.placed(slide_path()['walls'])                        # round 12: off every pod's way out
    for i in range(4):
        for nm, w in (('frame', fw), ('keel', kw)):
            pw = B.place3d(w, i) - prism(walls_out, Z1 - 2, Z1 + 32)
            plan = B.proj(pw, True, tol=0.05)
            for k, reg in tops.items():
                if reg.is_empty or plan.intersection(reg).area < 1.0:
                    continue
                piece = pw & prism(reg, Z1 - 1, Z1 + 30)
                drop = thin_of(plan.intersection(reg))                  # round 14: no wall slivers where a split or the slide path cuts a wall
                if not drop.is_empty and piece is not None:
                    piece = piece - prism(drop.buffer(0.3, 8), Z1 - 2, Z1 + 32)
                if piece is not None and piece.solids() and piece.volume > 1.0:
                    parts[k] = parts[k] + piece
    holes = dict(pod=[], keel=[], pedestal=[], bracket=[])
    pod_len = []
    for i in range(4):                                                  # round 13: the column bolt comes up from below with the foot screws
        for p, Ls in [(((L.FI_X[0] + L.FI_X[1]) / 2, y), L.FOOT_SCREW_L) for y in L.FOOT_SCREW_Y] + [(p, L.FRAME_BOLT_L) for p in L.FRAME_BOLTS]:
            holes['pod'].append(tuple(np.asarray(B.place2d(Point(*p), i).coords[0])))
            pod_len.append(Ls)
        for p in L.KEEL_SCREWS:
            holes['keel'].append(tuple(np.asarray(B.place2d(Point(*p), i).coords[0])))
    holes['pedestal'] = [tuple(np.array([LIDAR_X, 0.0]) + U(a) * PED_SCREW_R) for a in (45, 135, 225, 315)]
    holes['bracket'] = BR['deck_holes']
    sup, shw, sections, fin_screws, long_bolts = build_supports()
    bump, bhw, tab_cuts = build_bumpers(g['rb'], g['rp'], sk)
    holes['fin'] = fin_screws
    holes['fin_long'] = long_bolts
    holes['bumper'] = [xy for _, xy in tab_cuts]
    lapped = lap_screws(laps, [tuple(p) for k in holes for p in holes[k]])
    owner = lambda x, y: next((k for k, reg in regions.items() if reg.buffer(0.5).contains(Point(x, y))), None)   # noqa: E731
    for kind in ('pedestal', 'bracket'):
        for x, y in holes[kind]:
            k = owner(x, y)
            if k:
                parts[k] = parts[k] - cyl(M3_PILOT / 2, 16.0, (x, y, Z1 - 8.0))
    for x, y in holes['fin_long']:                                      # round 15: the front fins' long bolts into vertical pilots in the node
        k = owner(x, y)
        parts[k] = parts[k] - cyl(M3_PILOT / 2, LONG_BOLT_PILOT + 0.1, (x, y, Z1 - LONG_BOLT_PILOT / 2 + 0.05))
    frame = [k for k in parts if k.startswith(('hub', 'edge', 'node'))]
    for kind, Ls in (('pod', pod_len), ('keel', [L.KEEL_SCREW_L] * len(holes['keel']))):   # up from under the members (laps: both halves)
        for n, (x, y) in enumerate(holes[kind]):
            hole = cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) + cyl(FF.CB_D / 2, BELOW_CB + 0.1, (x, y, Z0 + BELOW_CB / 2 - 0.05))
            for k in frame:
                if W.near(parts[k], hole, 0.0) and (parts[k] & hole).volume > 0.01:
                    parts[k] = parts[k] - hole
            hw[f'bolt_{kind}_{n}'] = pan_screw((x, y, Z0 + BELOW_CB), (0, 0, 1), Ls[n])
    for n, ((x, y), lo, hi) in enumerate(lapped):
        parts[hi] = parts[hi] - cyl(M3_CLR / 2, Z1 - ZL + 0.2, (x, y, (Z1 + ZL) / 2)) - cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z1 - FF.CB_DEPTH / 2 + 0.05))
        parts[lo] = parts[lo] - cyl(M3_PILOT / 2, ZL - Z0 - 1.0, (x, y, (ZL + Z0 + 1.0) / 2))
        hw[f'bolt_lap_{n}'] = pan_screw((x, y, Z1 - FF.CB_DEPTH), (0, 0, -1), LAP_SCREW_L)
    mounts = FF.mount_angles()
    for cut, (x, y) in tab_cuts:                                        # round 13: the bumper tabs' recesses under the arcs, vertical pilots
        k = owner(x, y)
        tip = Z0 + FF.CB_DEPTH + TAB_SCREW_L + 0.8
        parts[k] = parts[k] - cut - cyl(M3_PILOT / 2, tip - (Z0 + TAB_T), (x, y, (tip + Z0 + TAB_T) / 2))
    for n, (x, y) in enumerate(holes['pedestal']):
        hw[f'bolt_pedestal_{n}'] = pan_screw((x, y, Z1 + 4.0), (0, 0, -1), 14.0)
    for n, a in enumerate((45, 135, 225, 315)):                         # round 12: the lidar's 4 x M2.5 x 8 (v10's) up through the top plate
        x, y = np.array([LIDAR_X, 0.0]) + U(a) * FF.LIDAR_HOLES_R      # into its base (thread depth VERIFY); driven on the bench before
        hw[f'bolt_lidar_{n}'] = H.solid(H.pan((x, y, FF.LIDAR_BASE_Z - 5.0), (0, 0, 1), 8.0, 'bolt_lidar', size=2.5))   # the pedestal goes on
    hw.update(BR['hardware'])
    parts['pedestal'] = build_pedestal()
    parts['bracket_d435'] = BR['bracket']
    parts.update(bump)
    hw.update(bhw)
    for n, (x, y) in enumerate(fin_screws):                                             # up through the member (laps: both halves) into the fin's foot
        hole = cyl(M3_CLR / 2, Z1 - Z0 + 0.2, (x, y, (Z0 + Z1) / 2)) + cyl(FF.CB_D / 2, FF.CB_DEPTH + 0.1, (x, y, Z0 + FF.CB_DEPTH / 2 - 0.05))
        for k in frame:
            if W.near(parts[k], hole, 0.0) and (parts[k] & hole).volume > 0.01:
                parts[k] = parts[k] - hole
        hw[f'bolt_fin_{n}'] = pan_screw((x, y, Z0 + FF.CB_DEPTH), (0, 0, 1), FIN_SCREW_L)
    parts.update(sup)
    hw.update(shw)
    holes['fin'] = fin_screws
    env = dict(env_d435=BR['camera'], env_d435_plug=BR['plug'],
               env_lidar=cyl(FF.LIDAR['d'] / 2, FF.LIDAR['h'], (LIDAR_X, 0, FF.LIDAR_BASE_Z + FF.LIDAR['h'] / 2)),
               env_arm_riser=box(62.0, 205.0, -56.0, 56.0, FLOOR2_UNDER + 5.0, 218.5), **d435_fov())     # layout F: on the 5 mm floor (not drawn)
    g.update(sk=sk, regions=regions, laps=laps, layers=layers, holes=holes, lapped=lapped, mounts=mounts, sections=sections, cradle_edges_rounded=BR['rounded'])
    return parts, hw, env, g


# ---- checks ----------------------------------------------------------------------------------------------------------------------------------
def check(parts, hw, env, g, fp):
    rep = dict(layout=dict(D=B.D_WHEEL, slide=B.SLIDE, robot_diameter=round(2 * (g['rb'] + BUMPER_T), 1)),
               frame=dict(member='solid square section, 22 mm deep (print with few walls, low infill)', ring_band=[RING_IN, RING_OUT], cross_w=CROSS_W))
    pcs = {}
    for k, s in parts.items():
        gp = B.proj(s, False)
        fits = []
        for ang in range(0, 90, 2):
            x0, y0, x1, y1 = affinity.rotate(gp, ang, origin=(0, 0)).bounds
            fits.append((max(x1 - x0, y1 - y0), sorted((x1 - x0, y1 - y0))))
        e = min(fits)[1]
        pcs[k] = dict(solids=len(s.solids()), valid=bool(s.is_valid), volume_cm3=round(s.volume / 1000, 1),
                      footprint_on_bed=[round(e[1], 1), round(e[0], 1)], height=round(s.bounding_box().size.Z, 1), bed_fit=bool(e[1] <= FF.BED))
    rep['printed_parts'] = pcs
    rep['all_fit_256_bed'] = all(v['bed_fit'] for v in pcs.values())
    # round 13: every pair of chassis parts, and each against the pods' static parts (v12 shipped with the hub inside all four edges)
    sp_, _ = L.build_static()
    pod_static = {}
    for i in range(4):
        for k, nm in (('frame_inner', 'pod_frame_inner'), ('keel', 'pod_keel_outer')):
            pod_static[f'{B.NAMES[i]}/{nm}'] = B.place3d(sp_[k], i)
    keys, ov = list(parts), {}
    for i, a in enumerate(keys):
        for c in keys[i + 1:] + list(pod_static):
            x = parts.get(c, pod_static.get(c))
            if W.near(parts[a], x, 0.0):
                v = W.overlap(parts[a], x)
                if v > 0.01:
                    ov[f'{a} / {c}'] = round(v, 2)
    rep['part_overlaps_mm3'] = ov
    rep['laps'] = dict(n=len(g['laps']), dovetail=TAIL, socket_clearance_mm=DOVETAIL_CLR, screws=len(g['lapped']), zones_meet=False)
    # round 14: thin pieces (narrower than MIN_FEATURE, corners sharper than ~75 deg) in each frame part's layers and its pocket walls
    thin = {}
    for k, (lo, hi) in g['layers'].items():
        sec = B.proj(parts[k], True, Z1 + 1.0, Z1 + 12.0, tol=0.05)
        for n, gg in (('lower', lo), ('upper', hi), ('walls', sec)):
            a = thin_of(gg).area if gg is not None else 0.0
            if a > 0.0:
                thin[f'{k} / {n}'] = round(a, 1)
    rep['thin_mm2'] = dict(min_feature_mm=MIN_FEATURE, found=thin)
    rep['d435_cradle'] = dict(camera_centre=list(FF.D435_C), edges_rounded=g['cradle_edges_rounded'], usb=USB_PLUG)
    frame = [k for k in parts if k.startswith(('hub', 'edge', 'node'))]
    rep['frame_volume_cm3'] = round(sum(parts[k].volume for k in frame) / 1000, 1)
    bands = {k: FF.placed(v) for k, v in fp['bands'].items()}
    cl = dict(frame_to_moving_z40_56=g['sk'].distance(bands['below']), frame_to_moving_z56_62=g['sk'].distance(bands['plate']),
              bumper_to_moving=Point(0, 0).buffer(g['rb'] + BUMPER_T, 720).difference(Point(0, 0).buffer(g['rb'], 720)).distance(bands['bumper']))
    # the pocket walls (z 62..76) against the pods' moving parts, in 3D, every pose (they sit beside the static frames)
    parts_pod, _ = L.build_static()
    unit = P.wheel_unit()
    walls = {}
    for i in range(4):
        walls[f'frame{i}'] = B.place3d(frame_walls()[0], i)
        walls[f'keel{i}'] = B.place3d(keel_walls(), i)
    near_items = dict(walls=None, camera=env['env_d435'], plug=env['env_d435_plug'], bracket=parts['bracket_d435'], lidar=env['env_lidar'],
                      pedestal=parts['pedestal'], supports=None)
    sups = {k: v for k, v in parts.items() if k.startswith('support')}
    static_pod = ('pod_frame', 'pod_keel', 'pod_sleeve', 'bolt_pivot', 'dowel', 'washer')
    dmin = {k: 1e9 for k in near_items}
    for a in L.POSES.values():
        sh, _ = L.posed(parts_pod, unit, a)
        for i in range(4):
            for k, s in sh.items():
                if k.startswith(static_pod):
                    continue
                plunger = k.startswith(('spring_plunger', 'die_spring'))     # they sit beside the column the side fins stand on
                ps = B.place3d(s, i)
                for nm, it in near_items.items():
                    if plunger and nm != 'supports':
                        continue
                    items = list(walls.values()) if nm == 'walls' else (list(sups.values()) if nm == 'supports' else [it])
                    for x in items:
                        if W.near(ps, x, 12):
                            dmin[nm] = min(dmin[nm], W.min_dist(ps, x))
    cl.update({f'{k}_to_moving': v for k, v in dmin.items()})
    # the fins against everything static: the pods' frames (the side fins sit on the inner frames' column tops), the chassis, lidar, cradle
    sh, _ = L.posed(parts_pod, unit, 0.0)
    pod_static = {f'{B.NAMES[i]}/{k}': B.place3d(s, i) for i in range(4) for k, s in sh.items() if k.startswith(static_pod)}
    others = {k: v for k, v in parts.items() if not k.startswith('support')}
    others.update(lidar=env['env_lidar'], d435=env['env_d435'], d435_plug=env['env_d435_plug'])
    ov, gap = {}, {}
    for nm, f in sups.items():
        for k, x in {**pod_static, **others}.items():
            if not W.near(f, x, 6):
                continue
            d = W.min_dist(f, x)
            gap[f'{nm} / {k}'] = round(d, 2)
            if d < 1e-3:
                v = f & x
                ov[f'{nm} / {k}'] = round(v.volume, 3) if v is not None else 0.0
    rep['supports_static'] = dict(overlap_mm3=ov, gaps_mm=gap,
                                  note='0 overlap everywhere; the side fins sit 0.02 above the column top (bearing there), the feet stand on the members')
    rep['clearance_to_moving_mm'] = {k: round(float(v), 2) for k, v in cl.items()}
    # round 15: nothing of ours in the D435's fields of view (its own body and plug aside), the front fins' distance to them
    fov = {}
    for k in [k for k in env if k.startswith('env_d435_fov')]:
        hits = {}
        for nm, x in {**parts, 'lidar': env['env_lidar']}.items():
            if W.near(env[k], x, 0.0):
                v = W.overlap(env[k], x)
                if v > 0.01:
                    hits[nm] = round(v, 2)
        fov[k] = dict(overlaps_mm3=hits, front_fins_mm={nm: round(W.min_dist(env[k], parts[nm]), 1) for nm, kind, _ in FINS if kind == 'front'})
    rep['d435_fov'] = dict(fov, note=f'frustums {FOV_L:g} mm long from each lens (depth: left imager; lens positions VERIFY)')
    rep['lidar_to_d435_cradle_mm'] = round(W.min_dist(env['env_lidar'], parts['bracket_d435']), 2)
    rep['lidar_to_camera_mm'] = round(W.min_dist(env['env_lidar'], env['env_d435']), 2)
    rep['pedestal_to_cradle_mm'] = round(W.min_dist(parts['pedestal'], parts['bracket_d435']), 2)
    land = {k: f"{sum(g['sk'].buffer(-1.5).contains(Point(*p)) for p in g['holes'][k])}/{len(g['holes'][k])}" for k in g['holes']}
    land['laps'] = f"{len(g['lapped'])} screws in {sum(1 for z, *_ in g['laps'] if not z.is_empty)} laps"
    rep['fasteners_landing'] = land
    # round 13: hole audit over the chassis' own screws (records), into the chassis parts and the pods' static parts
    recs = [r for r in (H.from_solid(s, k) for k, s in hw.items() if k.startswith('bolt_') and not k.startswith('bolt_pin')) if r]
    up_pod = {f'{B.NAMES[i]}/{nm}': np.array([B.frame(i)[1][0], B.frame(i)[1][1], 0.0]) for i in range(4) for nm in ('pod_frame_inner', 'pod_keel_outer')}
    import print_beds as PB

    def up_of(n):
        if n in up_pod:                                                 # both print on their side (pod x up)
            return up_pod[n]
        bd = PB.bed(n)
        return H.up_axis(bd[0]) if bd else None
    au = H.audit(recs, dict(parts, **pod_static), up_of)
    rep['hole_audit'] = dict(screws=len(recs), partial_holes=au['partial'], threads_across_layers=au['across_layers'],
                             soft_pilots_across_layers=au['soft_across_layers'],
                             heads_in_parts=au['head'], no_hole=au['no_hole'],
                             short_of_6mm={k: v for k, v in au['thread'].items() if 0 < sum(v.values()) < 5.9 and not k.startswith(('bolt_d435', 'bolt_lidar'))},
                             note='partial: a hole that is not a full circle in a part (straddles a split or runs off an edge); across layers: a '
                                  'thread-forming pilot whose axis is > 45 deg off the part\'s print-up axis. The D435 / lidar screws thread into the devices')
    rep['per_pod'] = dict(inner_frame=f'3 x M3 x {L.FOOT_SCREW_L:g} + the column bolt M3 x {L.FRAME_BOLT_L:g}, up from under the member into hex nuts in the frame; '
                                      'pocket walls on its centre-side face and front end (0.2 clearance, 14 tall)',
                          keel=f'2 x M3 x {L.KEEL_SCREW_L:g} up from under the member into Ø{L.M3_SOFT_PILOT} pilots through the keel; pocket walls behind it and at its inboard end (clipped off the pod\'s way out)',
                          pivot_block='bears on the ring',
                          supports='front pods: the side fin bears on the column top, located by 2 dowels in its 2 pilots (slots in the fin open toward the '
                                   'pod\'s way out); rear pods: pilots unused',
                          removal='screws out from below, bumper quarter off, unplug the motor; the pod slides out radially (its -x) under the second floor')
    foot = Polygon([(66.6, 18.0), (78.6, 18.0), (78.6, 92.0), (66.6, 92.0)])
    rep['inner_frame_foot_supported'] = [round(B.place2d(foot, i).intersection(g['sk']).area / foot.area, 2) for i in range(4)]
    keel = keel_foot()
    rep['keel_foot_supported'] = [round(B.place2d(keel, i).intersection(g['sk']).area / keel.area, 2) for i in range(4)]
    lc = np.array([LIDAR_X, 0.0])
    sh_ = []
    for nm, sec in g['sections']:
        c = np.array(sec.centroid.coords[0])
        a0 = math.degrees(math.atan2(c[1] - lc[1], c[0] - lc[0]))
        angs = [math.degrees(math.atan2(y - lc[1], x - lc[0])) for x, y in sec.exterior.coords]
        w = max(abs(((v - a0) + 180) % 360 - 180) for v in angs) * 2
        sh_.append(dict(support=nm, nearest=round(Point(*lc).distance(sec), 1), bearing=round(a0, 1), shadow_deg=round(w, 1)))
    rep['lidar'] = dict(at=[LIDAR_X, 0], scan_z=FF.LIDAR['plane'], supports=sh_, blind_total_deg=round(sum(s['shadow_deg'] for s in sh_), 1),
                        note=f"only the fins' webs cross z {SUPPORT_PLANE_BAND[0]:g}..{SUPPORT_PLANE_BAND[1]:g}, each a wedge along a ray filling its own "
                             "shadow (round 15); plus the arm while it reaches down in front")
    rep['supports'] = dict(
        spine=dict(bearing=SPINE['bearing'], length=SPINE['top'][1] - SPINE['top'][0], nose_x=round(float(fin_xy('spine', SPINE['top'][0])[0]), 1),
                   feet="hub's front cross arm + the cross by the hub + back node, 2 x M3 x 30 up through each", arch_apex_z=SPINE['apex']),
        side=dict(bearing=[SIDE['bearing'], -SIDE['bearing']], length=SIDE['top'][1] - SIDE['top'][0],
                  feet="front pod's column top (bears, 2 dowels) + the ring by the side stem + the tail leg on the side node (1 x M3 x 30 up "
                       "through each)", nose_xy=[round(float(v), 1) for v in fin_xy('side', SIDE['top'][0])], arm_riser_rear_corner=[62.0, 56.0]),
        front=dict(bearing=[FRONT['bearing'], -FRONT['bearing']], length=FRONT['top'][1] - FRONT['top'][0],
                   foot_L=[[round(float(v), 1) for v in fin_xy('front', u)] for u in FRONT['foot']],
                   fixing=f'1 x M3 x 30 up into the foot; 1 x M3 x {LONG_BOLT_L:g} from the floor top down through the fin into a pilot in the node'),
        floor=f'M3 x 12 down through the 5 mm floor into Ø{M3_PILOT} x {FLOOR_PILOT_DEPTH:g} pilots in the flanges '
              f'({sum(len(FIN_DIMS[k]["floor"]) for _, k, _ in FINS)})',
        section=f'scan band z {SUPPORT_PLANE_BAND[0]:g}..{SUPPORT_PLANE_BAND[1]:g}: a wedge as wide as its own lidar shadow ({FIN_WEB:g} at its near end, '
                f'front fins {FRONT["web"]:g}), flange {FIN_FLANGE:g} under the floor, legs flaring toward the deck',
        printing='upside down on the top flange: arches, coves and leg flares all lean inward, no supports')
    rep['section_I_mm4'] = dict(plate_neck_19x6=round(19 * 6 ** 3 / 12), plate_neck_25x6=round(25 * 6 ** 3 / 12),
                                ring_21x22_solid=round(21 / math.sqrt(2) * 22 ** 3 / 12 * math.sqrt(2)), cross_16x22_solid=round(16 * 22 ** 3 / 12),
                                note='printed with 3-4 walls and low infill a solid square section is a box section: close to these numbers, stiffer than the U-channel')
    return rep


# ---- export / render ----------------------------------------------------------------------------------------------------------------------------
def export(parts, env):
    (OUT / 'parts').mkdir(parents=True, exist_ok=True)
    for f in (OUT / 'parts').glob('*.step'):
        f.unlink()
    for k, s in parts.items():
        write_part(s, colour(k), OUT / 'parts' / f'{k}.step', k)
    items = {k: (s, colour(k)) for k, s in parts.items()}
    items.update({k: (s, colour(k)) for k, s in env.items()})
    pod = FF.pod_items()
    for i in range(4):
        for k, s in pod.items():
            items[f'{B.NAMES[i]}/{k}'] = (B.place3d(s, i), L.colour(k))
    write_step(items, OUT / 'chassis_assembly.step', 'chassis')


def render(parts, hw, env, g, fp, rep=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MP
    for f in VAL.glob('*.png'):
        f.unlink()

    def fill(ax, geom, **kw):
        for q in getattr(geom, 'geoms', [geom]):
            if q.is_empty or q.geom_type != 'Polygon':
                continue
            ax.add_patch(MP(np.asarray(q.exterior.coords), closed=True, **kw))
            for h in q.interiors:
                ax.add_patch(MP(np.asarray(h.coords), closed=True, facecolor='white', edgecolor=kw.get('edgecolor', 'k'), lw=0.4))

    fig, axs = plt.subplots(1, 2, figsize=(22, 11))
    lc = np.array([LIDAR_X, 0.0])
    for ax, with_pods in zip(axs, (False, True)):
        rb = g['rb']
        ax.add_patch(Circle((0, 0), rb + BUMPER_T, facecolor=(0.95, 0.6, 0.3), edgecolor='none'))
        ax.add_patch(Circle((0, 0), rb, facecolor='white', edgecolor='none'))
        if with_pods:
            for i in range(4):
                fill(ax, B.place2d(fp['moving'], i), facecolor=(0.95, 0.75, 0.15, 0.35), edgecolor=(0.6, 0.45, 0), lw=0.4)
        for k, reg in g['regions'].items():
            fill(ax, reg, facecolor=colour(k), edgecolor='k', lw=0.6)
        for z, *_ in g['laps']:
            fill(ax, z, facecolor='none', edgecolor='r', lw=0.8, hatch='////')
        for i in range(4):
            fill(ax, B.proj(B.place3d(frame_walls()[0], i), False), facecolor=(0.3, 0.3, 0.35), edgecolor='k', lw=0.3)
            fill(ax, B.proj(B.place3d(keel_walls(), i), False), facecolor=(0.3, 0.3, 0.35), edgecolor='k', lw=0.3)
        if with_pods:
            for i in range(4):
                fill(ax, B.place2d(fp['static'], i), facecolor=(0.25, 0.68, 0.74, 0.75), edgecolor=(0.1, 0.35, 0.4), lw=0.4)
                A = B.frame(i)[0]
                ax.text(*(A * 1.13), B.NAMES[i], fontsize=11, weight='bold', ha='center')
        fill(ax, B.proj(parts['bracket_d435'], False), facecolor=FF.COLOURS['bracket'], edgecolor='k')
        for k in [k for k in env if k.startswith('env_d435_fov')]:     # round 15: the D435's fields of view (plan)
            fill(ax, B.proj(env[k], False).intersection(Point(0, 0).buffer(240)), facecolor=(*colour(k), 0.15), edgecolor=colour(k), lw=0.6)
        ax.add_patch(Circle(tuple(lc), FF.LIDAR['d'] / 2, facecolor=(0.1, 0.1, 0.1, 0.25), edgecolor='k', lw=1.0))
        for nm, sec in g['sections']:
            fill(ax, sec, facecolor='k', edgecolor='k')
            c = sec.centroid.coords[0]
            angs = [math.atan2(y, x - LIDAR_X) for x, y in sec.exterior.coords]
            a0 = math.atan2(c[1], c[0] - LIDAR_X)
            dev = [((v - a0) + math.pi) % (2 * math.pi) - math.pi for v in angs]
            for a in (a0 + min(dev), a0 + max(dev)):
                ax.plot([LIDAR_X, LIDAR_X + 380 * math.cos(a)], [0, 380 * math.sin(a)], color='0.55', lw=0.4)
        for k, s in hw.items():
            bb = s.bounding_box()
            ax.plot((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, 'm.', ms=4)
        if not with_pods:
            for k, reg in g['regions'].items():
                c = reg.representative_point()
                ax.text(c.x, c.y, k.replace('_', '\n'), fontsize=8, ha='center', va='center', color='0.15')
        ax.set_xlim(-240, 240)
        ax.set_ylim(-240, 240)
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        blind = rep['lidar']['blind_total_deg'] if rep else ''
        ax.set_title(('chassis (front = +x): hub (ring + cross), 4 nodes; red hatch: half-laps; dark: pod pocket walls; blue / green: D435 FOV;\n'
                      f'lidar at x {LIDAR_X:g} (circle); black: the second-floor fins at the scan plane, wedges along rays (blind {blind} deg)')
                     if not with_pods else 'with the pods: teal = static pod parts in their pockets, yellow = moving (swept)', fontsize=10)
    fig.tight_layout()
    fig.savefig(VAL / 'plan.png', dpi=85)
    plt.close(fig)
    fig, axs = plt.subplots(4, 1, figsize=(13, 17))
    for ax, (kind, title) in zip(axs, (('spine', 'spine (bearing 180, on the centreline): feet on the hub\'s front arm, the cross by the hub and the back node'),
                                        ('side', f'side fins (bearing +-{SIDE["bearing"]:g}): nose under the arm riser corner, seat on the front pod column top, '
                                                 'foot on the ring by the side stem, tail leg on the side node'),
                                        ('front', f'front fins (bearing +-{FRONT["bearing"]:g}): foot on node_front\'s arc beside the D435, corbel in over it; '
                                                  f'the long M3 x {LONG_BOLT_L:g} (dashed)'),
                                        (None, 'cross-sections: spine | side (legs shifted off the rear pod) | front; the scan band is a wedge in plan (not shown)'))):
        if kind is None:
            for n, (k, sd) in enumerate((('spine', 1), ('side', 1), ('front', 1))):
                fill(ax, affinity.translate(fin_cross_section(k, sd), 40 * n, 0), facecolor=COLOURS['support'], edgecolor='k')
            ax.set_xlim(-20, 100)
        else:
            fill(ax, fin_profile(kind), facecolor=COLOURS['support'], edgecolor='k')
            S = FIN_DIMS[kind]
            if 'bolt' in S:
                ax.plot([S['bolt']] * 2, [Z1 - LONG_BOLT_PILOT, FLOOR2_TOP], 'm--', lw=1.5)
            for u in S['screws']:
                ax.plot([u, u], [Z0, Z1 + FIN_PILOT_DEPTH], 'm-', lw=2)
            for u in S['floor']:
                ax.plot([u, u], [FLOOR2_UNDER + 5, FLOOR2_UNDER - FLOOR_PILOT_DEPTH], 'm-', lw=1.5)
            if kind == 'side':
                ax.add_patch(plt.Rectangle((SIDE['column'][0] - 1, COLUMN_TOP - 25), SIDE['column'][1] - SIDE['column'][0] + 2, 25, facecolor=(0.25, 0.68, 0.74)))
                ax.text(sum(SIDE['column']) / 2, COLUMN_TOP - 14, 'column top', ha='center', fontsize=8)
            ax.set_xlim(S['top'][0] - 15, S['top'][1] + 15)
        ax.axhspan(*SUPPORT_PLANE_BAND, color=(0.2, 0.7, 0.2, 0.15))
        ax.axhline(FF.LIDAR['plane'], color='g', lw=0.6)
        ax.axhline(Z1, color='0.3', lw=0.8)
        ax.axhline(FLOOR2_UNDER, color='0.3', lw=0.8, ls='--')
        ax.set_aspect('equal')
        ax.grid(True, lw=0.3)
        ax.set_title(title + ('' if kind is None else '; u = distance from the lidar axis; green: scan band (web only), magenta: screws'), fontsize=9)
    fig.tight_layout()
    fig.savefig(VAL / 'fin_profiles.png', dpi=85)
    plt.close(fig)
    items = dict(parts)
    items.update({k: v for k, v in env.items() if k != 'env_arm_riser' and not k.startswith('env_d435_fov')})
    items.update(hw)
    L.iso(items, VAL / 'iso_chassis.png', 215, 35, 'chassis parts (pods hidden)', colour, tol=0.6)
    L.iso({k: v for k, v in items.items() if not k.startswith(('bumper', 'bolt_bumper'))}, VAL / 'iso_chassis_below.png', 200, -35,
          'chassis from below (bumper hidden): solid 22 mm members, half-laps', colour, tol=0.6)
    full = dict(items)
    full['env_arm_riser'] = env['env_arm_riser']
    pod = {k: v for k, v in FF.pod_items().items() if not k.startswith(('roller_core', 'die_spring'))}
    for i in range(4):
        for k, s in pod.items():
            full[f'{B.NAMES[i]}/{k}'] = B.place3d(s, i)
    L.iso(full, VAL / 'iso_with_pods.png', 225, 28, 'chassis with the pods, lidar forward, second-floor fins (floor not drawn; dark box: the arm riser on it)',
          colour, tol=0.8)
    side = {k: v for k, v in full.items() if not k.startswith(('bumper', 'bolt_bumper'))}
    L.iso(side, VAL / 'iso_supports.png', 150, 10, 'second-floor fins from the rear left, bumper hidden (floor not drawn; dark box: the arm riser on it)',
          colour, tol=0.8)
    pocket = {k: v for k, v in full.items() if k.startswith(('hub', 'node_front', 'node_left', 'FL/pod_frame', 'FL/pod_keel', 'bolt_side_FL', 'bolt_pod', 'bolt_keel'))}
    L.iso(pocket, VAL / 'iso_pod_pocket_FL.png', 250, 30, 'FL pod static parts in their pockets (moving parts hidden)', colour, tol=0.4)
    front = {k: v for k, v in side.items() if not k.startswith(('RL/', 'RR/'))}                 # round 15: the front fins, the D435's FOV
    front.update({k: v for k, v in env.items() if k.startswith('env_d435_fov')})
    L.iso(front, VAL / 'iso_front_fins_fov.png', 30, 25, 'front fins beside the D435, its depth (blue) and RGB (green) fields of view (500 mm)',
          colour, tol=0.8)


def main(stage):
    VAL.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)
    fp = FF.pod_geometry()
    parts, hw, env, g = build(fp)
    for k, s in parts.items():
        print(k, len(s.solids()), s.is_valid, round(s.volume / 1000, 1), flush=True)
    rep = None
    if stage in ('check', 'all'):
        rep = check(parts, hw, env, g, fp)
        (VAL / 'checks.json').write_text(json.dumps(rep, indent=1), encoding='utf-8')
        print(json.dumps({k: v for k, v in rep.items() if k != 'printed_parts'}, indent=1), flush=True)
    if stage in ('export', 'all'):
        export(parts, env)
        print('exported', OUT, flush=True)
    if stage in ('render', 'all'):
        render(parts, hw, env, g, fp, rep)
        print('rendered', VAL, flush=True)


if __name__ == '__main__':
    main(sys.argv[1] if sys.argv[1:] else 'build')

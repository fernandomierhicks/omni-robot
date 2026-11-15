"""Dimensions of the cad/v11 compact wheel + motor, mm.

Pod frame: x = wheel axis (motor output flange toward -x, cup stub toward +x); y, z radial. Roller frames: local z = roller axis (tangent),
local y = radial outward, local x = pod x. Everything is nominal CAD; nothing has been printed.
"""
import math

# ---- wheel envelope (same as the v10 / v9 wheel and the sim baseline: Ø140, roller axes at r 52) ---------------------------
R_WHEEL = 70.0                     # wheel outer radius; the roller tread profile is this circle about the wheel axis
R_AXIS = 52.0                      # roller axis radius
D_ROW = 17.25                      # row planes at x = -D_ROW (row 1) and +D_ROW (row 2); rows are 30 deg apart in azimuth
N_PER_ROW = 6

# ---- rollers (PLA core + TPU tread, flush flat ends) -----------------------------------------------------------------------
HT = 19.5                          # half length of core and tread (2.35 deg overlap between neighbouring rollers)
TREAD_T = 2.5                      # TPU tread thickness
FILLET_TREAD = 2.0                 # TPU end-edge fillet
BORE_D = 3.4                       # roller bore on the M3 axle (was 3.6: too much slop)
JOURNAL, JOURNAL_R = 0.75, 4.0     # thrust journal at each end: length, radius
KEY_W, KEY_DEPTH = 2.4, 1.5        # three tread keys in core keyways open at both ends (the tread slides off): width, depth into the core
CORE_R = 12.7                      # straight core / tread bore (round 6): the barrel interface needed the TPU stretched 2.8 mm over the core's
                                   # middle, so a printed tread could not be taken off; the tread is now 2.5 mm thick at the ends, 5.3 in the middle
CORE_CHAMFER = 0.5                 # lead-in on the core ends and the tread bore ends
DETENT_DEPTH, DETENT_RIB = 0.5, 0.4   # round 7: the tread snaps into place: 45 deg V groove round the core's middle, a matching V rib 0.4 high
                                   # inside the tread bore (0.1 clear at its tip). Fitting stretches the bore 0.4 mm in radius (3 %), by hand
HUB_EDGE_R = 0.8                   # round 7: round on the spoke webs' x-end edges (each spoke filleted before the union) and the drum's left end

# ---- hub ----------------------------------------------------------------------------------------------------------------------
MOTOR_SHIFT_X = 27.5              # option 1 (Nov 2): motor moved outboard (+x) so its XT30 plug and leads sit outside the hub with >= 4 mm to anything rotating
X_LEFT, X_RIGHT = -39.5, 29.0      # hub end faces; the right end is the drum only: no right bearing, the hub rides on the motor output bearing + the left 6804
DRUM_IN, DRUM_OUT = 30.0, 33.0     # drum bore and outside radius (the front of the motor can lives in the bore)
WALL_BACK = -26.5                  # left end wall (r ACCESS_R .. DRUM_IN) ends here at the access bore and runs out on a 45 deg cone to the drum
FLANGE_BACK = -23.0 + MOTOR_SHIFT_X        # flange plate clamped to the motor output face by three M2.5 screws
FLANGE_FRONT = FLANGE_BACK - 3.5           # a 45 deg cone joins its front to the drum (printable cavity roof)
MOTOR_ROT = 143.0                  # motor turned about the axis from its STEP pose: XT30 at az ~160 (back, slightly up: away from the pivot and shocks)
FLANGE_R = 13.5
FLANGE_ANGLES = tuple(a + MOTOR_ROT for a in (30, 150, 270))   # follow the motor's M2.5 output holes
CART_ANGLES = (30, 150, 270)       # cartridge screws
M25_CLEAR = 2.7
PLATE_T = 3.0                      # cartridge plate, flush with the left end face
X_PLATE_FLOOR = X_LEFT + PLATE_T
ACCESS_R = 18.0                    # left access bore (cartridge boss, driver access to the flange screws)
CART_R = 26.0                      # cartridge screws (M3 x 12 flat head into Ø2.8 pilots)
R_BRG, BRG_FIT = 16.0, 0.05        # 6804 outside radius and seat allowance

THRUST_GAP = 0.3                   # journal face to block face
ZF = HT + JOURNAL + THRUST_GAP     # block thrust faces (+-ZF about the roller centre); the flat cut runs across the whole roller end
BAND_C = 0.95                      # radial clearance around the tread
BLEND_R = 3.0                      # blend between the thrust face and the tread clearance: the fillet at the thin spoke neck
BLOCK_R = 8.0                      # axle block radius
WEB_OUT = 12.0                     # spoke web reaches this far outboard of its row plane (x; row 2 stops at the drum end X_RIGHT)
WEB_GAP = 6.0                      # inboard the webs run to x = +-WEB_GAP / 2 (the other row's rollers reach x +-1.7 with clearance)
                                   # round 6: the roller cut is the roller's revolved envelope plus its radial drop-in channel, not a
                                   # prism through x, so the web outside the roller end stays and wraps it (v10 had 24 mm deep spokes)
WEB_REACH = 12.0                   # web foot half width on the drum, either side of the block centre (local z)
CHANNEL_CHAMFER = 11.0             # v13 Nov 10 (Fernando: supports deep in the channel between the spoke rows were hard to pull out): a 45 deg
                                   # collar round the drum, from r DRUM_OUT at x WEB_GAP/2 - this to r DRUM_OUT + this at x WEB_GAP/2 (row 2's inboard
                                   # web faces). Printed cartridge end down, row 2's web roots then grow out of the drum instead of hanging over the
                                   # channel. Its cone stays (D_ROW + R_AXIS - DRUM_OUT + WEB_GAP/2 - this) / sqrt 2 - (R_WHEEL + BAND_C - R_AXIS)
                                   # = 1.0 mm off row 1's roller clearance envelope (2 mm off the tread); 0 = no collar (v11/v12 hub)

ENGAGE = 6.5                       # M3 x 50 axle thread engagement in the pilot block
SEAT = 50.0 - ZF - ENGAGE          # axle head underside (local z); the shank runs 50 mm toward -z
TUNNEL_D = 8.0                     # round 6-7 head pocket + driver tunnel (double teardrop); now only the teardrop helper's default size
HEAD_POCKET_D, HEAD_DEPTH = 7.0, 3.0   # round 8: round counterbore for the M3 pan head (Ø6.5 x 2.5), Ø3.4 shank hole to the thrust face,
FLARE_L = 5.0                      # then a 45 deg cone opening outward for FLARE_L (to r 8.5, past the block), then Ø7 on for the head and driver.
                                   # Longer cuts into the neighbouring roller's head seat (the two axles in a spoke cross 4 mm past their heads):
                                   # 5: 99 % of its seat left, 7: 61 %, 9: 30 %
AXLE_HEAD_D = 6.4                  # axle head envelope for the insertion graze check
B_OUT = SEAT + HEAD_DEPTH          # head-side block outer face
PILOT_D = 2.9                      # roller axle thread pilot (printed across the layers; every other M3 pilot is 2.8)
PILOT_DEPTH = ENGAGE + 0.8
A_OUT = ZF + PILOT_DEPTH + 1.2     # pilot-side block outer face
CLR_D = 3.4

# ---- motor and harness (the pod's fork bolts straight to the motor's rear face; no motor mount) -------------------------------
# components/ak40-10/AK40-10 with connector.STEP is the round-2 pod-frame motor, moved; this puts it back, then MOTOR_SHIFT_X outboard,
# then turns it MOTOR_ROT about the axis
MOTOR_STEP_SHIFT = (34.462 + MOTOR_SHIFT_X, 41.0002, -49.0161)
MOTOR_R = 26.5                     # can radius
MOTOR_X = (-23.0 + MOTOR_SHIFT_X, 17.2 + MOTOR_SHIFT_X)   # output face, rear face
REAR_HOLES_AZ = tuple(a + MOTOR_ROT for a in (-120, -30, 60, 150))   # motor rear face: 4 holes x 4.3 deep at r 23.5. The maker's STEP draws them Ø2.8 (read as M3
                                   # until Nov 10); Fernando, with the motor in hand, screws M2.5 into them: tapped M2.5 here, drawn at the thread's minor Ø like the output holes
REAR_HOLES_R, REAR_HOLE_DEPTH, REAR_HOLE_D = 23.5, 4.3, 2.05
LEAD_D = 3.0                       # ASSUMED power lead OD (VERIFY)
LEAD_STRAIGHT = 0.5                # straight lead beyond the pin ends before the bend
LEAD_BEND_R = LEAD_D               # sharp 90 deg bend (centreline radius = 1 x OD) toward +x
LEAD_RUN_TO = MOTOR_X[1] + 12.0    # leads modelled to here (x) after the bend, inside the plate cutout (route on to the robot not drawn)
HARNESS_CLEAR = 4.0                # wanted clearance all round the plug and leads

# ---- robot wiring harness (harness.py, harness_route.py; Nov 11, Fernando: slots behind the fins, zip ties straight through the PLA) ---------
# Floor-2 cable slots: a lead that crosses the lidar's scan plane (z 163) where the lidar sees it is a blind angle. The spine and front fins run
# to the rim, so the only open shadow is past each side fin's tail (u 238 from the lidar's axis, along the fin's ray at bearing +-136.7): one
# stadium slot each, centred on the ray. The side fin's shadow is 3.7 deg wide (web 5 at u 78.1), half width 8.0 mm at u 245, so a slot of
# half width SLOT_R 6.5 stays inside it with 1.5 mm to spare and a bundle up to Ø8 in it is hidden (bundles lie one behind another along the ray)
HARNESS_SLOT_U = (245.0, 272.0)    # slot centreline from / to (u along the fin's ray); its round ends reach u 238.5 (the fin's last floor screw at
                                   # u 232: 3.2 mm of floor between its head and the slot) and 278.5 (r 208.1; the rim lip starts at r 212)
HARNESS_SLOT_R = 6.5               # slot half width (the shadow's half width at u 238.5 is 7.7)
# zip-tie anchors: two slots through a printed plate either side of the cable; the tie goes down one, under the plate's bridge, up the other
# and round the cable (no clip, no insert). Sized for 3.6 mm wide x 1.2 mm ties (VERIFY on the ties bought); fits bundles up to ~Ø10
TIE_SLOT_L, TIE_SLOT_W = 4.6, 2.0  # each slot: along the cable, across it
TIE_PITCH = 13.0                   # slot centre to centre across the cable (an 11 mm PLA bridge under the cable)
TIE_KEEP = 2.0                     # each tie pair's footprint this far from any hole, boss, lip, rib, flange or device underneath / on top
# Nov 11 (Fernando): besides the tie pairs, M3 pilots (Ø2.8, the floor's boss under each: upper.BOSS_*) on a square grid, to attach things later.
# The grid is the tie grid's (30 mm, on multiples of 30) shifted half a pitch, so each hole sits in the middle of four tie points
GRID_PITCH, GRID_OFFSET = 30.0, 15.0
GRID_KEEP = 2.0                    # each hole's boss (Ø upper.BOSS_D) this far from anything under the floor (lip, rib, flange, boss, lead) or any hole
                                   # or edge in the plate; nothing on top within GRID_TOP_R of the hole, GRID_TOP_H above the floor
GRID_TOP_R, GRID_TOP_H = 5.0, 8.0
# the three potted DC-DC modules (Fernando's picks, Nov 11; harness.CONVERTER_PARTS): body x, y, z as drawn (leads extra). ASSUMED (VERIFY):
# the 19 V size is a comparable 24-19 V 10 A unit's, the 12 V and 5 V the common potted 8 A / 5 A class
CONV_19V = (74.0, 74.0, 32.0)      # B09KY6H4FZ 24 -> 19 V 10 A buck (22-40 V in: bench-test its dropout at 20-22 V before trusting it)
CONV_12V = (64.0, 60.0, 22.0)      # B0F3HQ6ND8 15-90 -> 12 V 8 A buck (the arm)
CONV_5V = (60.0, 66.0, 22.0)       # B0CM37DX4Q 8-40 -> 5 V 5 A buck


# ---- motor-to-hub assembly jig (motor_jig.py; Nov 10, Fernando). Bench tooling, not part of the robot; drawn in the pod frame at the assembled position ------
JIG_CLR = 0.15                     # radial clearance on each centring fit (can in the sleeve, sleeve in the drum bore, stem in the access bore); first try
JIG_BASE_T, JIG_BASE_R, JIG_BORE_R = 5.0, 55.0, 20.0   # cradle's base disc under the can's rear face, and the opening in its middle
JIG_SLEEVE_TOP, JIG_CONE_X = 10.0, 33.0   # sleeve's top (x; 19 mm inside the drum, 4.5 short of the can's front) and where its 45 deg foot starts (4 past the drum's end)
JIG_SLOT_CLR = 1.0                 # each side of the XT30 + leads in the sleeve's open slot
JIG_CAP_T, JIG_CAP_R = 5.0, 36.0   # guide cap's plate on the hub's left end, past the drum so it can be lifted
JIG_STEM_GAP = 0.5                 # stem's end to the hub flange
JIG_GUIDE_D, JIG_PROBE_D, JIG_HEAD_RELIEF = 5.6, 2.2, 3.2   # bores for the M2.5 pan head (Ø5) and driver; the probe bore (1.75 mm filament), widened at its end over a screw head
JIG_PEG_D, JIG_PEG_L = 2.4, 5.3    # pegs into the cartridge's Ø2.8 pilots: they clock the cap to the flange holes

# ---- single-pod bench stand (pod_jig.py; Nov 11, Fernando). Bench tooling, not part of the robot; drawn in the pod frame. Its seat is the chassis' own
# (chassis.frame_outline / keel_outline / keel_foot / BRACE, pocket walls, screw seats), so the pod's six screws are the robot's ----------------------
POD_JIG_WHEEL_LIFT = 7.0           # bench to the wheel's bottom at droop (the unloaded pod's pose): the wheel spins free. Set to -DROOP to stand the pod on its
                                   # wheel at nominal ride height instead (it then rocks between wheel and feet unless loaded)
POD_JIG_CLR = 3.0                  # plan clearance to everything that moves, droop..bump (as the chassis: base.CLR_MOVE)
POD_JIG_RING_W, POD_JIG_RING_T = 12.0, 6.0   # flat ring on the bench round the wheel: width, height. The wheel and motor sit ahead of the seat, so it is what
                                   # keeps the stand from tipping forward
POD_JIG_CORNER_R = 10.0            # the ring's outer corners in plan
POD_JIG_TUNNEL_D = 7.2             # counterbore from the underside up to each screw head (the chassis' counterbore, FF.CB_D; takes a pan or socket head)
POD_JIG_SCREW_L = dict(foot=70.0, column=100.0, keel=80.0)   # Nov 11 (Fernando: long screws are fine on the jig): each screw's tip lands where the robot's
                                   # does (1.5 past its nut / 16.9 into the keel), so its head seats near the bench (8.9 / 13.9 / 8.9 mm up) instead of 51 mm up
                                   # a tunnel. M3 pan heads this long are rare: socket caps (Ø5.5 x 3) fit the same counterbores
POD_JIG_EDGE_C = 0.6               # 45 deg chamfer on the bench edges (elephant's foot) and the tunnels' mouths
POD_JIG_SHELL = 0.0                # Nov 12: > 0 hollows the legs (bench to the slab) into closed pockets with walls this thick, the base plate as their floor
                                   # and the 22 mm slab as their roof. Tried at 4: 4 pockets, 38 cm3 of 298, roofs <= 9.2 mm wide, every check clean; but at the
                                   # ~15 % infill the jig is meant for, that space is only ~6 cm3 of infill and the pockets add ~25 cm3 of perimeter walls,
                                   # so it prints heavier. Worth it only for a solid / high-infill print. 0 = solid (the slicer's infill does the hollowing)
POD_JIG_HOLE_WALL = 4.0            # solid wall kept round each screw's counterbore inside a pocket


# ---- print-fit calibration coupons (coupon.py; Nov 11 review). Bench prints, not part of the robot: each series brackets a "first try" value of the CAD ----
COUPON_CLR = (0.10, 0.15, 0.20, 0.30)          # per-face clearance, dovetail sockets and the fork tenon's socket (robot: DOVETAIL_CLR / TENON_CLR 0.1)
COUPON_DT = dict(ring=(2.5, 3.4, 5.0), cross=(4.0, 6.5, 7.0))   # dovetail tail: neck and flare half widths, length (chassis.TAIL); tab height = the half-lap
COUPON_DT_H = 11.0                             # tail height (chassis half-lap: members z 40..62)
COUPON_TENON = (9.0, 24.5, 7.0, 0.5)           # the fork bar's tenon: across, along, deep, lead-in at the socket's mouth (pod_lean.tenon)
COUPON_SEAT = dict(b627=(21.80, 21.90, 22.00, 22.10), b6804=(31.90, 32.00, 32.10, 32.20))   # seat diameters; robot: 627 22.00 (B627_OD), 6804 32.10 (R_BRG + BRG_FIT)
COUPON_SEAT_DEPTH, COUPON_SEAT_PLATE = 7.0, 9.0    # both bearings are 7 wide; the plate leaves 2 mm under the seat, relieved so only the outer race bears
COUPON_RELIEF = dict(b627=18.0, b6804=26.0)    # relief bore under each seat (SEAT_RELIEF_D for the 627)
COUPON_PILOT = (2.7, 2.8, 2.9, 3.0, 3.1)       # M3 thread-forming pilots; robot: 2.8 (M3_PILOT), 2.9 (M3_SOFT_PILOT), 3.0 (TIE_PILOT_D)
COUPON_SLOT = (0.2, 0.3, 0.4, 0.5)             # per-face clearance of a slide-in slot for a 12 mm part (the pod frame in its pocket: robot POCKET_CLR 0.2)
COUPON_SLOT_PEG, COUPON_SLOT_LEN = 12.0, 40.0   # the loose peg's width (FRAME_T, the pod frame's thickness) and the slots' length
COUPON_CAN = (0.05, 0.10, 0.15, 0.20)          # radial clearance of the can pocket, can Ø 2 x MOTOR_R (robot: MOTOR_POCKET_CLR 0.1)
COUPON_GAPS = (0.3, 0.4, 0.5, 0.6, 0.8)        # open slits (robot: roller thrust gap THRUST_GAP 0.3, hub-to-tread 0.95)


def rt(z):
    """tread outer radius about the roller axis at local z"""
    return math.sqrt(R_WHEEL ** 2 - z * z) - R_AXIS


def rc(z):
    """core outer radius"""
    return rt(z) - TREAD_T


# ---- suspension pod (pod.py) ------------------------------------------------------------------------------------------------
# Pod frame = wheel frame at nominal ride height (axle at the origin, floor at z = -R_WHEEL). The robot plate (designed later) is flat with its top
# at PLATE_TOP_Z, as v10's deck (top 3 mm above the axle): the hinge sits at plate height. Two side frames stand on the plate beside a cutout, each
# with a keel down through a notch to the shaft, and are held by vertical M3 screws from the top; the shaft ties them together.
PLATE_TOP_Z = 3.0
PIVOT_Y, PIVOT_Z = 80.6, 0.0       # hinge level with the axle, 80.6 mm arm (design.md 2, sim POD_PIVOT): no torque jacking
DROOP, BUMP = 3.0, 15.0            # axle travel below / above nominal (sim pod_droop, pod_travel - droop)
# pivot (round 7): no shaft. Each 627 (owned) presses into a seat in a fork arm's outer face. Each frame's keel carries a printed Ø7 pin in the
# 627's inner race on a Ø10 boss that bears on the inner race (spacer across the 1 mm frame gap), and one M3 x 16 pan head from the keel's
# outer face threads through the keel, boss and pin: a steel core for the printed pin, held in the static frame. The bosses locate the fork
# axially (0.1 mm each side).
B627_OD, B627_W, B627_BORE = 22.0, 7.0, 7.0
BRG_RECESS = 0.1                   # 627 face under the fork arm's outer face
BRG_PATH_CLEAR = 0.5               # Nov 9 (Fernando): the outer 627 slides into its seat along its axis from the arm's inner face, so the bar is cut away
                                   # around its path, a trough this much larger than the bearing's radius (the seat itself stays the press fit)
BRG_PATH_LEN = 10.0                # Nov 9 (Fernando): ... and only this far (along x) past the bearing's exposed face; the full-length trough was too deep
SEAT_RELIEF_D, SEAT_RELIEF_T = 18.0, 1.0   # beyond each seat, so the seat bottom bears on the outer race only
PIVOT_BOSS_D = 10.0                # frame boss onto the inner race only (627 inner ring about Ø10.5-11)
PIVOT_HEAD_D, PIVOT_HEAD_RECESS = 7.0, 2.6   # counterbore for the M3 pan head in the keel's outer face (flush: the keel sits in the plate notch)
PIVOT_SCREW_L = 16.0               # M3 pan head: all of it threaded into the keel (9.4), boss (1) and pin (5.6 of 6.9)
LOBE_R = 15.0                      # fork pivot lobe radius
M3_PILOT, M3_CLR = 2.8, 3.4
M3_SOFT_PILOT = 2.9                # round 14 (Fernando): a gentle thread-forming pilot, through the whole hole, allowed across print layers
DOVETAIL_CLR = 0.1                 # round 14 (Fernando): every dovetail socket this much bigger than its tail on each face (first try)
MIN_FEATURE = 4.0                  # round 14 (Fernando): no structural part narrower than this in plan (no thin slivers to match faces)
# springs (pod round 4, Nov 3): the yellow 8 x 25 mm 3D-printer bed-levelling springs (Ender 3 / CR-10 upgrade, sold in 10-12 packs). They
# copy the JIS extra-light die spring for an Ø8 hole / Ø4 rod x 25 free (MW Components DJ-8X25Y: 6.1 N/mm, 10 mm long-life = 40 %,
# 12.5 mm max = 50 %). The Amazon packs give no rate: 6.1 N/mm is ASSUMED until a spring is measured (VERIFY). One spring alone would reach
# 46 % at bump, so each side has TWO side by side, each on an M4 button-head bolt used as a plunger: its dome presses on the fork pad, its
# shank slides in the frame's guide hole, an M4 nyloc on top keeps it captive (it never bottoms: the fork stops first); an M4 washer seats
# the spring. No damper. Replaced the ISO 10243 Ø10 x 38 die spring (McMaster 1804N112) of rounds 2-3.
SPRING_HOLE_D, SPRING_ROD_D, SPRING_L0 = 8.0, 4.0, 25.0
SPRING_OD, SPRING_ID = 7.7, 4.3    # modelled envelope (VERIFY on the part)
SPRINGS = {'yellow 8x25': (6.1, 10.0)}   # N/mm, long-life working deflection mm (max 12.5)
SPRING_DESIGN = 'yellow 8x25'      # the nominal geometry is for this spring at DESIGN_MASS; other masses: printed shims (spring table)
SPRINGS_PER_SIDE, SPRING_PITCH = 2, 10.6   # two plungers per side, this far apart in y (2 mm wall between the Ø8.6 bores)
DESIGN_MASS = 10.0                 # kg robot (all on the 4 wheels: conservative)
SPRING_ARM = 26.0                  # centre of the spring pair this far from the pivot toward the axle (springs at 31.3 / 20.7): wheel rate 2.6 N/mm
BOLT_HEAD_D, BOLT_HEAD_K = 7.6, 2.2    # ISO 7380 M4 button head
SHANK_R, GUIDE_R = 2.0, 2.2        # M4 shank; its guide hole in the tower cap
WASHER_OD, WASHER_T = 9.0, 0.8     # M4 washer (ISO 7089)
NUT_T, NUT_AF, GUIDE_LEN, NUT_GAP = 5.0, 7.0, 8.0, 1.5   # M4 nyloc (DIN 985)
PAD_Z = 24.0                       # top of the fork pads under the plungers (both sides)
PAD_HALF = 13.0                    # pad half length (y): each dome slides about 6 mm along it over the travel, 5.3 mm either side of the centre
TOWER_CLEAR = 10.0                 # tower underside above the pad top (bump travel + rotation; the far plunger lifts 6.4 mm)
# fork (one printed part: two arms + a beam behind the wheel)
OUTER_ARM_X = (-60.5, -43.5)       # 4 mm from the cartridge face (x -39.5)
INNER_ARM_X = (MOTOR_X[1], MOTOR_X[1] + 13.0)   # clamped to the motor's rear face by 4 x M2.5 x 14 pan heads in Ø5.6 counterbores
OUTER_BOSS_R, INNER_PLATE_R = 17.0, 29.5
LEAD_NOTCH_R = 28.0                # inner plate cut back to this radius where the leads pass
REAR_CLR_D, REAR_CBORE_D = 2.9, 5.6   # Nov 10 (Fernando: long M2.5s, heads closer to the plate's outer face): M2.5 clearance; counterbore for the Ø5 x 2 pan head
REAR_SCREW_L, REAR_ENGAGE = 14.0, 3.8   # head seat 2.8 under the plate's outer face (head top 0.8 under it; was 4.8 deep with M3 x 12). 14 is the longest that fits:
                                   # 15 would leave the head proud or bottom in the 4.3 deep hole
MOTOR_POCKET_D, MOTOR_POCKET_CLR = 2.0, 0.1   # Nov 10 (Fernando): shallow tight-fit pocket for the can in the fork's motor face, so the stator drops onto the hole pattern.
                                   # The inner half's motor face moves this far toward the wheel (the motor does not move); the pocket's floor is the old face. 0.5 x 45 deg
                                   # lead-in at its mouth (and against elephant's foot); the can's rear edge has its own 0.5 chamfer, so 1.0 of straight wall grips. Prints motor
                                   # face down: support only inside the pocket. 0.1 radial is a first try for a tight fit (VERIFY on a print); 0 = no pocket
WHEEL_CLEAR_R = 74.0               # nothing of the fork between the arms inside this radius about the axle; the beam is the pivot lobe
                                   # (r LOBE_R) across, cut to this radius: no tail behind the pivot since the stops moved to the springs (round 5)
# axle stub (printed, bolted into the outer arm from outside; carries the left 6804's inner race)
STUB_D, STUB_SHOULDER_D, STUB_FLANGE_D, STUB_FLANGE_T = 20.0, 23.0, 34.0, 3.0
STUB_SHOULDER_GAP = 0.3            # to the inner race
STUB_SCREW_R = 13.5                # 3 x M3 x 12 into the arm
# side frames (two printed parts, each printed lying on its flat outer face)
FRAME_GAP, FRAME_T = 1.0, 12.0     # fork arm to frame; frame thickness (x) = foot width
FOOT_T = 8.0                       # foot on the plate top
FOOT_Y = (20.0, 104.0)
KEEL_Y = (41.6, 92.0)              # keel below the plate, from the spring column's front to 11.4 mm behind the pivot: needs a notch in the plate
# hard stops at the springs (round 5): bump = two rails under each tower, either side of the spring washers, that land flat on the fork pad
# at bump; droop = a hook from the keel under the fork arm, below the springs. Their faces are the pad / arm faces at bump / droop.
STOP_HALF = 8.0                    # droop hook half length (y), centred on the spring pair
HOOK_T = 6.0                       # droop hook thickness under the arm
LEDGE_W = 7.0                      # droop hook reach inward under the arm (x; 6 mm under it past the 1 mm gap)
RAIL_CLEAR = 1.0                   # bump rails to the spring washers (x)
# frame strength (pod round 3, Nov 3): the spring pushes the tower up and the shaft pulls the keel down, so the frame bends between them;
# a full-height web joins the column to the keel (no notch at the column root) and a gusset braces the column back onto the keel
TOWER_HALF = 13.0                  # spring column and tower half width (y): 3.4 mm wall past the two Ø8.6 spring bores
TOWER_REACH = 8.5                  # tower inner face this far past the spring axes (x): 4.2 mm wall
GUSSET_TOP_Z, GUSSET_END_Y = 50.0, 90.0   # gusset from the column's back face at this height down to the foot top at this y (over the shaft)
FILLET_COL, FILLET_GUSSET = 2.0, 5.0   # concave fillets: column front / foot (clear of the y 35 screw head), gusset to column and foot
MOUNT_Y = (25.0, 35.0, 98.0)       # vertical M3 through each foot into the plate (clear of the spring column, the gusset and the keel notch)
PLATE_T_ASSUMED = 6.0              # for the cutout check only

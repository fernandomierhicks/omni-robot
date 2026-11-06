# cad/v12: the whole robot (layout F, round 13), clean SolidWorks assembly

Started Nov 4 2026 as a clean copy of cad/v11's round-12 robot: only the scripts that generate and check it, and a freshly built SolidWorks
viewing assembly. The design history, earlier rounds, the pod-only v11 assembly and the old exports stay in `cad/v11/` (its README has the
round-by-round record to round 12); round 13 (below) was done here. Status: **proposal**, nominal and unprinted; nothing is load-tested or
analysed for strength.

| What | Where |
| --- | --- |
| **SolidWorks robot** (open this: the only assembly in `solidworks/`) | `solidworks/V12_robot.SLDASM` |
| Sub-assemblies (`solidworks/assemblies/`) | `V12_frame`, `V12_mounts`, `V12_bumper`, `V12_reference`, `V12_upper`, `V12_electronics`, `V12_arm`, `V12_fasteners`, `V12_pod` / `V12_pod_m` (`V12_pod_static`, `V12_unsprung` > `V12_wheel` > `V12_roller`, `V12_motor`) |
| Parts | `solidworks/parts/V12_*.SLDPRT` (imported STEP bodies, not native features; the D435 and RPLIDAR are the vendor SolidWorks parts) |
| Part STEPs + manifest (inputs to the assembly) | `solidworks/step_parts/`, `solidworks/manifest.json` (`solidworks/_keys/`: the build's component names) |
| Checks | `validation/` (`chassis/`, `pod_lean/`, `upper/` incl. `pod_service.json` and `arm_sweep.json`, `solidworks/motion_test.json`, `robot/` views) |
| Inputs outside this folder | `../v10 human editable/` (SO-101, riser / adapter, electronics STEPs, arm meshes, SolidWorks helpers), `../../components/ak40-10/`, `../other/Realsense to bucket/` (D435), `Roomba robot/CAD/v2/RPLIDAR.SLDPRT` |

## Regenerate

```
python -B tools/sw_robot_parts.py                                   (omni-robot-sim venv: part STEPs + manifest)
C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py import
C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py roller wheel motor unsprung static
C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py pod
C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py group_frame      (then group_mounts, group_bumper, group_reference,
                                                                     group_upper, group_electronics, group_arm, group_fasteners:
                                                                     one per call, SolidWorks dies building several big ones)
C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py robot test interference views
```

`sw_robot_parts.py` leaves a part's STEP (and its mtime) alone when it comes out the same, and prints which parts changed; `import` then
converts only those (it re-imports a part whose SLDPRT is older than its STEP, and carries on after a crash).

Checks (omni-robot-sim venv, from `tools/`): `chassis.py check`, `pod_lean.py check`, `upper.py check`, `pod_service.py`, `arm_sweep.py`.
The chassis and upper checks include every pair of parts and the hole audit (`hardware.audit`: half holes, pilots across print layers,
buried heads, screws through solid); `sw_robot.py interference` writes `validation/solidworks/interference.json`.
`chassis.py`, `pod_lean.py` and `upper.py` also take `export` / `render` (printed-part STEPs to `step_export/`, views to `validation/`).

## Tools

| Script | Makes |
| --- | --- |
| `dims.py`, `wheel_motor.py`, `pod.py` | dimensions; wheel, rollers and the AK40-10 model; the pod geometry the lean pod builds on |
| `pod_lean.py` | the suspension pod (inner frame, keel, fork halves, sleeve, springs / plungers) and its poses |
| `base.py`, `first_floor.py`, `chassis.py` | pod placement and footprints; the first-floor chassis (hub, nodes, fins, bumper, lidar / D435 mounts, the D435's FOV), clipped to each pod's slide-out path |
| `layout_study.py`, `screen_study.py`, `upper.py` | plug / lead envelopes; the SO-101's meshes and kinematics; the second floor, arm mount, electronics, bin deck, face |
| `hardware.py`, `print_beds.py`, `step_colour.py` | fastener records (magenta) and the hole audit; print orientation per printed part; coloured STEP writing |
| `sw_robot_parts.py`, `sw_robot.py` | part STEPs + manifest; the SolidWorks assemblies and the suspension motion test |
| `pod_service.py`, `arm_sweep.py` | can one pod come out (driver access to its screws, the slide out); the arm's planned moves against the robot |
| `robot_step.py` | one STEP of the whole robot's printed parts for review (`step_export/robot_assembly.step`) |

## Round 13 (Nov 4, Fernando's review of v12)

| Item | Was | Now |
| --- | --- | --- |
| Hub / ring edges | the hub overlapped all four edges by 3.6 cm³ each: its laps to the nodes (a ±20° sector) reached into the ring, where the edge-node laps also are, and both gave their lower half to different parts | splits square to the members (hub ends at \|x\|, \|y\| 88 on the cross; edges at \|x ∓ y\| 53), no two lap zones may meet (asserted) |
| Why it was missed | `chassis.py check` compared the frame with the pods only, never its parts with each other; `upper.py` compared upper parts with each other and with the chassis; the SolidWorks stage ran the motion test only | `chassis.py check` tests every pair of chassis parts (and the pods' static parts); `sw_robot.py interference` runs SolidWorks' interference detection on the whole robot |
| Half-circle holes | three pod foot screws straddled the hub / node split; the spine's floor screws sat on the floor's rear seam; the pedestal's ran along its tube wall; the D435 screws notched the cradle's cheeks (heads inside them) | `hardware.audit()` in every check: each hole's wall sliced every mm along each screw, in our printed parts; none left in the chassis |
| Threads across layers | pod foot / keel screws into vertical pilots in side-printed parts; bumper screws radial into flat-printed nodes; the floor's seam screws horizontal; the 12 V buck's into the riser's wall | the pods' screws go into hex nuts in slots (the pocket walls cover the slots); the bumper hangs on tabs at its foot (printed on the bed) with one M3 × 10 up from below into a vertical pilot each; the floor's seams use vertical screws (below); the buck's screws go through the riser's wall into nuts. Left: the face crown's two M3 into the face frame (it prints on its back; see Open) |
| Floor joints | flat butt seams, horizontal M3 through two lips | dovetail tabs in the plate on every seam (they go together from above); side and front seams shiplapped (the front quadrant's 8 mm lip under the other's edge, flush M3 × 10 down into it); on the rear seam each spine floor screw sits in a dovetail tab, so its hole is in one quadrant |
| Chassis joints | plain half-laps | dovetail half-laps: each half's end is a dovetail in the other part's socket; parts still assemble vertically |
| Hub | cross 16 wide, disc r 20 | cross 24, disc r 34, every inside corner of the frame R8 in plan (frame 1151 cm³) |
| Keel pocket walls | boxes, 1 mm proud of the pad under them, sharp | plan outlines flush with the pad (one face), outer corner R6, free ends R2.4, top edges R1.5 |
| Column bolt | M3 × 60 from the column top into the foot (pod's own) | M3 × 60 up from under the chassis through the member, foot and column into a nut 30 mm up: it comes out with the pod's other screws |
| Pod screws from below | 5 | 6 (3 foot, column, 2 keel); the third foot screw moved y 88 → 89.5 (its hole was 0.66 mm off the member's edge) and the foot runs to y 94 |
| SO-101 base | two L-clamps + 4 × M2.5 into 2.4 mm dimples | its four teardrop holes (Ø5 bosses with a 90° countersink): 4 × M4 × 25 countersunk into Ø3.4 pilots through the adapter |
| D435 | box cradle, cheeks over the screw heads, no fillets, right-angle USB plug turning up | base, back plate and three webs (outer webs flush with the base), wings chamfered for printing, cut against the camera's real model; straight USB-C plug out of its −y end as on the bench, the lead bending up. **Camera raised 12 mm (z 111.5 → 123.5)**: at 111.5 only an 8 mm plug clears the FR wheel at bump; now 4.47 mm with a 21 mm plug. The sim's `ARM_D435` is still 111.5 (gap) |
| Servo bus adapter | on the floor (two spacers) | on a printed bracket on the riser's back face (2 × M3 × 16 through the riser's back wall into nuts inside it); the board on 5 mm spacers, 4 × M3 × 14 into pilots along the bracket's print axis; its USB-C lead turns up. The arm comes out as one module (riser, adapter, SO-101, buck, bus board). Not on the SO-101's WaveShare plate: the base motor holder sits right behind its four Ø5 holes (insert holes; no room for nuts) |
| IMU | none drawn | BNO085 board (the Roomba project's model, `reference/imu_bno085.step`) at (32, −40), about 100 mm from the Teensy carrier, on a printed holder with two standoffs (vertical pilots); its 1 × 8 header lead is an envelope (VERIFY) |
| Hex drives | flat heads and the M4 plungers had none | every pan and flat head has a hex socket |
| Fin floor screws | their 7 mm pilots ran from the 16 mm flange into the 5 mm web (1.1 mm wall) | the web-only band ends at z 170 (was 172; scan plane 163), so the flange starts at 172.5 and holds the whole pilot |
| Pod slide-out | (found here) the outer fork arm hangs 0.07 mm below the members' top at droop and was left out of the slide path, so it would scrape the side node's rim arc | moving parts count in the slide path up to 0.5 mm over the members' top |
| Others | | lidar pedestal top r 40 → 36.5 (2.0 mm from the FL rollers at bump); pedestal screws outside its tube; front deck posts (8, ±92) → (10, ±84) (their floor screws' heads sat in the side fin's flange with no boss); e-stop / switch pod screws off their tubes; mic screws M3 × 8; lap screws M3 × 20 → 16 (tips ran out of the member); four stray v10 M3 × 10 (floating inside the computer) removed; the stale arm-riser envelope dropped from the reference group |

**Round 13 results (nominal CAD; nothing printed):**

- **Chassis:** no part overlaps, no half holes, no pilots across layers, every fastener lands, ≥ 3.0 mm to moving parts, every part fits the
  bed.
- **Upper:** no overlaps (with the chassis too), no screw short of 6 mm, the battery slides out; flagged: the face crown's two screws (Open).
- **Pods:** each comes out with its 6 screws reachable from below and a clear slide (`pod_service.json`).
- **Arm:** its planned moves stay ≥ 12.3 mm from everything (`arm_sweep.json`).
- **SolidWorks:** motion test passes (695 components). Interference detection finds no part of ours and no fastener out of place. It still
  reports rows inside the SO-101 vendor model at stow, and 0.8–2.9 mm³ between the motor model's XT30 pins and leads.

## Round 14 (Nov 4, Fernando's review of round 13)

| Item | Was | Now |
| --- | --- | --- |
| Thin material | fillet cusps where a split line cut a fillet (hub at the ring), needle tips on the rim arcs, the ring dovetails' socket walls 2.9 mm, slivers on the edges along the laps and on pocket walls cut by a split or the slide path | **no frame part narrower than 4 mm in plan (`dims.MIN_FEATURE`), no corner sharper than ~75°**: `chassis.thin_of` / `no_thin` on the skeleton, per part (a cusp goes to the neighbour it belongs with: `merge_thin`), per layer after the laps and on the wall pieces; ring dovetails 3 / 4.5 / 6 → 2.5 / 3.4 / 5 (neck / flare half width, length); the check reports what is left (`thin_mm2`) |
| Dovetail fit | chassis laps: tail and socket the same size; floor tabs 0.15 | every socket 0.1 mm bigger than its tail on each face (`dims.DOVETAIL_CLR`, first try): chassis laps per layer (`lap_layers`), floor tabs |
| Keel screws | 2 × M3 × 25 into hex nuts in a slot | 2 × M3 × 35 into a Ø2.9 pilot through the whole keel (16.9 mm engaged; Fernando: gentle enough not to delaminate across the layers). `hardware.audit` lists such pilots as `soft_across_layers` (allowed), not `across_layers` |
| Gap under the inner frame | the ring was cut away under each inner frame's pivot end (pod y 76–86, the whole foot width; foot 82 % supported) | closed: the frame's pivot-block round (r 9 about the pivot) dipped 1 mm under its foot, and the slide-out clip took it for part of the pod's way out. Clipped at the foot; the ring (edge into node) now runs on under the whole foot |
| D435 | 30° down at z 123.5 (12 mm over the sim, for a straight 21 mm USB-C plug) | **7.5° down at the sim's pose (190, 0, 111.5)**, `robot_export.D435_TILT`; the cradle follows the tilt. An L-shaped USB-C plug (Fernando), its head ≤ 8 mm off the camera (VERIFY), the lead straight up: 4.2 mm from the FR rollers at bump. No straight plug fits at this height: the FR wheel at bump reaches z 116 beside the camera's end |

**Round 14 results (nominal CAD; nothing printed):** chassis: no part overlaps, nothing thinner than 4 mm left (`thin_mm2` empty), no half
holes, no threads across layers but the keel's Ø2.9 pilots (`soft_pilots_across_layers`), every fastener lands, inner frames' feet 94 %
supported (82 %), every part fits the bed, the D435 plug 4.2 mm from the FR rollers at bump. Upper: no overlaps, no screw short of 6 mm (the
face crown and vendor mic-board items as round 13). Pod service: all four come out (6 screws reachable from below, clear slide). Arm sweep:
≥ 12.3 mm from everything (gripper to node_front, as round 13), none near the D435 cradle; 33 moves, the four grasps at 0.36 m reach found
no IK (the arm and riser did not change this round; not compared with round 13's file, which was overwritten). SolidWorks (rebuilt Nov
4): motion test passes both hands; interference detection finds no part of ours and no fastener out of place, only the round 13 rows inside
the SO-101 vendor model at stow and the motor model's XT30 pins / leads. (A first rebuild showed 0.4-0.5 mm³ between the hub and edge_FR /
the side nodes, also in `chassis.py check`: where parts butt outside a lap, each outline's simplification pushed one into the other; butt
joints are now 0.1 mm apart as well.) Review STEP of the whole robot (printed parts only): `step_export/robot_assembly.step`
(`tools/robot_step.py`).

## Round 15 (Nov 4–5, Fernando's review of round 14)

| Item | Was | Now |
| --- | --- | --- |
| Hub and ring edges | hub (cross + disc) and four ring edges, nine frame parts, 12 laps; the hub never touched an edge (3.4 mm apart, both lapped to the nodes) | **one hub: ring + cross + disc** (233 × 232 mm turned 45°, fits the bed); each node starts square across its stem (`NODE_SPLIT`: front x 120 between the pedestal's screw rows, back 130, sides 126), one dovetail half-lap each (6 lap screws; the side fins' foot screws also pass through the side laps). Five frame parts |
| Spine | u 140..336 (x −20..−216), 2 feet | **on forward past the centre to x 70** (8 mm under the arm riser's rear edge, 12 mm from the lidar), a third foot on the hub's front cross arm (x 25..50), two arches. Its new floor screw (x 40) sits in a front-seam dovetail tab; the front seam's lip screw at x 50 is gone. None under the battery tray |
| Front fins | none | **two fins on node_front's arc**, bearing ±36° from the lidar, beside the D435 and behind its lens plane: ≥ 15 mm outside the RGB and ≥ 21 mm outside the depth FOV; 6.3 mm from the camera, 4.4 mm from the USB lead (they reach in over it only above z 134). Each: M3 × 30 up into its foot, and one **M3 × 130** pan head from the floor's top down through the floor and fin into a 10.5 mm pilot in the node (clamps all three). Their flanges carry the riser extension's front corners |
| Scan-band web | 5 mm web along each ray | **a wedge in plan** filling its own lidar shadow (5 mm at its nearest point, front fins 6 mm), capped at the 16 mm flange: the material is where the lidar is already blind |
| Side fins | u 78..225, the tail a cove hanging at z 168 | on to u 238 with a **second leg (a ~9 mm column) at u 226..238** on the side node, both legs 4 mm off the rear pod (was 2.5). Beyond u 248 the rear pod's keel fills the shadow up to z 86, and the power stack's corner screw stands at u 243.5 |
| Arm base | v10's riser x 62..130; the adapter (10 mm) and the SO-101's base (x 117.6..204.6) cantilevered 75 mm | **riser on under the whole adapter** (`upper.RISER_EXT`): an open-top box (4 mm side, front and centre walls on a 4 mm foot plate) to x 205, 2 × M3 × 12 down through its plate into floor bosses (x 188, ±30, clear of the lidar) |
| D435 FOV | not drawn | `env_d435_fov_depth` (87 × 58°, from the left imager) and `env_d435_fov_rgb` (69 × 42°), 500 mm long, see-through in SolidWorks; lens positions VERIFY |
| Lidar blind | 9.4° (spine 2.0, sides 3.7 each) | **20.9°**: spine 5.3, sides 3.7 each, front fins 4.1 each |

**Round 15 results (nominal CAD; nothing printed):** chassis: no part overlaps, nothing thinner than 4 mm, no half holes, no threads
across layers (the keel's Ø2.9 pilots as before), every fastener lands (incl. both long bolts), ≥ 3.39 mm from the fins to moving parts,
nothing of ours in either FOV frustum, every part fits the bed. Upper: no overlaps (with the chassis too), no screw short of 6 mm, the
battery slides out; the face crown and mic-board items as round 13. Pod service: all four come out (6 screws reachable from below, clear
slide). Arm sweep: 33 moves ≥ 12.3 mm from everything (gripper to node_front), the four 0.36 m grasps still without IK, as round 14 (the
FOV envelopes are views, not obstacles). SolidWorks: rebuilt Nov 5 (`import`, `group_frame`, `group_mounts`, `group_reference`,
`group_upper`, `group_fasteners`, `robot`: 685 components, no mate errors); **the motion test, interference detection and views were not
run this round**, so `validation/solidworks/` is still round 14. The old `parts/V12_edge_*.SLDPRT` are deleted.

**D435 on the riser (Nov 5, proposal): see `../v13/README.md`.** The sim's portrait riser mount was checked against this round's robot
there: portrait fits nowhere (clear of the arm it reaches below the lidar's plane); a landscape D435 was drawn low in a pocket in the riser's
front, through a notch in the floor. This folder is unchanged.

**Why there is a riser:** the sim baseline (`robot_export.ARM`, Nov 1) puts the SO-101's base at 218.5 mm, 37 mm over the 181.5 floor.
The floor-reach problem was the opposite one: the drawn 50 mm riser (base at 231.5) gave no fingers-down floor reach, and lowering it to
37 mm restored 0.26–0.35 m (45 mm: 0.30–0.32, 40: 0.27–0.34). The Oct 25 study found mounts on the plate itself clear only far back
(x ≤ 0.05, no bin drop); the riser lets the arm stand forward and still drop into the rear bin. Lower than 37 mm has not been studied in
layout F: it would likely extend floor reach but changes the bin drop and the shoulder's sweep over the D435.

## The robot in short

- **Pods:** four bolt-on suspension pods, each held by 6 M3 driven up from under the chassis (4 into hex nuts in the inner frame, 2 into
  Ø2.9 pilots in the keel). To remove one: robot
  on a stand, its bumper quarter off (its screws from below, slide it out), the 6 screws out from below, slide it straight out ~100 mm, unplug
  its XT30, finish the slide. The second floor stays on. The motor lead needs ~120 mm of service loop at its grommet (not drawn).
- **Chassis (round 15):** one hub (ring + cross + disc) and four nodes; five second-floor fins in vertical planes through the lidar's axis
  (spine, two side fins, two front fins beside the D435), their scan-band webs wedges filling their own shadows (20.9° blind).
- **Upper robot:** second floor in four interlocking quadrants on five fins; SO-101 on v10's riser (extended under the whole adapter, round 15) and adapter at the front, with its 12 V
  buck and servo bus board on it; an IMU near the centre; computer, battery, power board + Teensy carrier at the rear under a bin deck; bin on
  the deck; 5 in face behind the bin with mics and speaker on its crown; e-stop and switch at the rear corners. The battery slides out to the
  left. The face cannot go in front of the arm (the SO-101's shoulder stands over the front edge).
- **Fasteners (round 15):** every screw and nut is modelled (magenta library parts, each with its hex socket): 2 × M3 × 130, 64 × M3 × 50,
  8 × M3 × 60, 2 × M3 × 45, 10 × M3 × 35, 12 × M3 × 30, 12 × M3 × 25, 4 × M3 × 20, 12 × M3 × 16, 10 × M3 × 14, 49 × M3 × 12, 48 × M3 × 10,
  6 × M3 × 8 (pan); 12 × M3 × 12 (flat, wheel cartridges); 4 × M4 × 25 (flat, SO-101 base); 12 × M2.5 × 5, 4 × M2.5 × 8; 20 M3 hex nuts; 4 dowels
  Ø3 × 10; plus 16 spring plungers (M4 button heads), 4 pivot dowels Ø7 × 20, 4 M4 washers.
- **Print beds:** every printed part has a `PRINT_BED (±X/Y/Z up)` plane at the face that goes on the bed (`print_beds.py`; the reasoning
  per part is in cad/v11's README).
- **Open / VERIFY:** strength (fins, riser, keel, deck posts, face legs); M2.5 thread depth in the lidar; the rear deck posts' floor screws
  are reachable only with the floor off; charge port vs battery drawer; face location; cables and harness not modelled; the mirrored pods'
  motor is the whole motor mirrored, not the real opposite-hand plug. Round 13:
  - **Face crown screws:** they still thread vertically into the face frame, which prints on its back (layer plane). A crown flange screwing
    along x would need support whichever way the crown prints; the load is light (crown, mics, speaker).
  - **D435 USB-C plug (round 14):** the camera is at the sim's pose; the cable must have an L-shaped plug whose head stands ≤ 8 mm off the
    camera with its lead going up (9 mm leaves 3.3 mm to the FR rollers at bump). Also check which end of the D435 has its port.
  - **Dovetail fit (round 14):** 0.1 mm per face is a first try; print a test joint and adjust `dims.DOVETAIL_CLR`.
  - **Vendor models:** the SO-101 at its stow pose interferes with itself in the vendor meshes (wrist follower in the rotation-pitch part and in
    the base motor holder, about 2 cm³ each; SolidWorks interference); the stow pose needs checking against the arm's own parts. The mic
    board's holes sit about 0.4 mm off its screws (vendor model). The bus board's hole pattern and the IMU's header lead are assumed.
  - **Nuts in slots:** the pods' hex nuts go in before a pod is fitted; check that they stay put while the screws are started from below.
  - **Round 15:** the M3 × 130 pan heads (front fins) are an unusual length: VERIFY a source, else M3 threaded rod with a nut on the floor.
    The D435's lens positions in its FOV envelopes are assumed (17.5 / 32.5 mm left of centre). Not routed: the lidar's lead (out of the
    pedestal's back, under the spine's first arch) and the D435's lead (up beside the right front fin to the floor). The front fins' webs
    round the long bolt are 2.05 mm each side. The 75 mm riser extension adds a load path through the floor above the lidar (no fin there):
    no strength check.

# cad/v11: compact wheel + motor and its suspension pod

Started Nov 2 2026. A self-contained wheel pod (wheel, AK40-10, sprung fork, two side frames) that drops into a cutout in the flat robot plate, which is designed later.
It is built in build123d, reviewed as STEP, and assembled in SolidWorks from those STEPs. The SolidWorks parts are imported bodies, not native editable features.
Everything is nominal CAD; nothing has been printed.

| What | Where |
| --- | --- |
| **SolidWorks robot, viewing (layout F: chassis, 4 lean pods, second floor, arm, electronics, bin, face, every fastener; drag any fork)** | `solidworks/robot/V11R_robot.SLDASM`; sub-assemblies `V11R_frame`, `V11R_mounts`, `V11R_bumper`, `V11R_reference`, `V11R_upper`, `V11R_electronics`, `V11R_arm`, `V11R_fasteners`, `V11R_pod` / `V11R_pod_m` (`V11R_pod_static`, `V11R_unsprung` > `V11R_wheel`, `V11R_motor`); see SolidWorks below |
| **SolidWorks pod (drag the fork or the wheel)** | `solidworks/V11_pod.SLDASM`; sub-assemblies `V11_chassis`, `V11_fork`, `V11_wheel` (12 x `V11_roller`), `V11_motor`; parts in `solidworks/parts/` |
| Pod STEP (nominal pose; coloured per face; springs shown; no fasteners or bearings) | `step_export/pod/pod_assembly.step`; printed parts in `step_export/pod/parts/` (2 side frames, fork, axle stub) |
| Wheel + motor STEP | `step_export/wheel_motor/wheel_motor_assembly.step`; printed parts in `step_export/wheel_motor/parts/` (hub, cartridge, roller core, roller tread) |
| Checks and views | `validation/pod/` (`checks.json`: droop / nominal / bump, spring table, plate cutout; `section_*.png`; `iso_*.png`), `validation/wheel_motor/`, `solidworks/motion_test.json` |
| Build | `tools/dims.py` (every dimension), `wheel_motor.py`, `pod.py`, `sw_parts.py` (omni-robot-sim venv, `python -B <script> all`), then `sw_build.py` (cadenv, see below) |
| Inputs | `reference/` (bolt and bearing STEPs used by the checks), `../../components/ak40-10/AK40-10 with connector.STEP` (the maker's motor with its XT30(2+2) plug; only the plug is used) |

## Robot base and layout (proposals, Nov 3; not adopted)
STEP-only exploration. Round 7 below, its STEPs and the SolidWorks files are unchanged.
- **Lean pod, round 8 proposal** (`tools/pod_lean.py` → `step_export/pod_lean/`, `validation/pod_lean/`):
  - Nothing static stands outboard of the wheel, so pod pairs can sit close.
  - All four springs are on the inner arm, at the same lever arms (same rate and travel).
  - The inner pivot is a Ø7 steel dowel, since it now carries about 2.7× the wheel load.
  - A small outer keel inside the outer arm holds the outer pivot (M3 + printed flanged sleeve in the 627).
  - The fork's base bar runs under the pivot instead of the Ø30 beam. The outer arm is 11 mm.
  - The plate is lowered to 8 mm below the axle (62 above the floor), so both static parts stand on it without notches.
  - `check`: no interference at droop / nominal / bump.
- **Lean pod round 9 (Nov 3, Fernando's review of the SolidWorks robot; `pod_lean.py`, STEPs regenerated, SolidWorks robot NOT rebuilt):**
  - **Fork in two printed halves**, joined on the motor-face plane (x 44.7). Each prints flat on a big face with no support:
    - `pod_fork_outer` (72.6 cm³, 108 × 111 × 47): outer arm, the axle stub as part of it (no separate stub, flange or its 3 screws), the whole base bar and a gabled tenon. Prints on the arm's outer face, bar upright.
    - `pod_fork_inner` (78.5 cm³, 21 × 124 × 59): motor plate, spring pads, inner 627 seat, the bar's last 21 mm and the socket. Prints on the motor face; the socket's roof is 45° (no bridge).
  - **Tie bolts:** 4 × M3 × 50 pan heads on two lines along the bar, at (y, z) (78.5, −16.5) and (82, −24.5), one from each end per line, each with 10 mm in a Ø2.8 pilot.
    - The inner ones (heads 10 mm deep in the inner half, tips at x 5.5) clamp the halves and squeeze the bar from the joint to x 15.5.
    - The outer ones (heads 3.5 mm deep in the arm's outer face, tips at x −2) squeeze the arm root and bar to x −12. The bar prints upright, so its layers lie across it; the bolts pre-compress them.
    - Counterbores are ≥ 2.2 mm off the 627 seats.
  - **Base bar:** 14 × 21.5 (y 73–87, z −29.5 to −8; round 8: 12.5 × 18). Past the keel it ramps up at 45° to the pivot height (z 0) and stays there to the joint.
  - **Outer keel:** prints on its flat outer face. An M4 washer (Ø9 × 0.8) on the 627's inner race replaces its printed boss.
    - A gusset on the foot makes it an L in plan, against side load at the pivot, with an R3 cove in the L's corner.
    - A Ø10 boss round the pivot pilot on the wheel side (≥ 10 mm from the rollers radially). The pivot screw is now M3 × 16 (7.1 mm engagement; was M3 × 14).
    - Teardrop foot-screw holes; R0.8 on 15 of 29 edges (OCC refused the rest: mostly tangent seams).
    - 4.25 cm³ (was 3.3).
  - **Rounding:** R2 on the arm outlines (0.8 chamfer on the bed faces), R2.5 on the bar's long edges, R1.5 at the stub root, R1 at the motor-plate root. The bar-to-outer-arm root would not fillet.
  - **`check`:** no interference at droop / nominal / bump; fork halves meet with 0 overlap.
    - Overhang (> 45°) as printed: inner half 1.2 mm², keel 0, outer half 79 mm² (the two outer counterbore ceilings: small annular bridges round Ø3.4 holes).
    - The inner frame's 600 mm² is unchanged from round 8.
  - **Chassis (read-only check against the exported layout-F STEPs, scratch script; chassis not regenerated):** no overlap. Fork to the side nodes is 2.52 mm at bump, round 8 2.44: both under the chassis's 3 mm rule, which its own check reports as 3.0.
- **Lean pod round 10 (Nov 3, Fernando's review of round 9; STEPs regenerated, chassis and SolidWorks robot rebuilt for it):**
  - **Outer arm** 3 mm closer to the wheel: x −52.5 to −41.5, 2 mm from the hub's left face (was 5). The pod is 3 mm narrower outboard.
    The keel moves with it; it stands outside the wheel's radius (keel to rotating 9.6 mm).
  - **Tie bolts:** each threads into a Ø2.9 pilot its whole length; the inner ones pass the inner half in Ø3.4 so they still clamp it to the bar.
    - Staggered head seats (outer, inner): line 1 at −49 / 55.5 (threaded −49..1 and 5.5..44.7), line 2 at −42 / 62 (−42..8 and 12..44.7).
    - Some bolt is threaded in at every x along the bar except the 3.5 mm under the outer heads.
  - **Motor plate:** a 6.7 mm plate (M3 × 8) was tried and put back to 13 mm (M3 × 12; inner half 78.6 cm³), because the inner half carries all four springs.
  - **Keel:** round 9's foot screw at x −36 was blind under the gusset.
    - The screws are now at x −27 and −17, heads on the foot top, in round counterbores down through the gusset (`KEEL_SCREWS`; the chassis takes its holes from it).
    - The holes are plain round, with no teardrop points (Fernando). As printed on its outer face, their roofs are about 58 mm² of small bridges.
    - 3.8 cm³.
  - **Inner frame:** R1 on 36 of 48 edges, excluding the print face and the plate face.
  - **`check`:** no interference at droop / nominal / bump; the halves meet with 0 overlap; fork to harness 3.8 mm.
  - **Chassis regenerated** (`pod_fp.pkl` deleted and rebuilt, `chassis.py all`): keel pockets and screw holes follow round 10.
    The robot is Ø450.3 (was 456.5), all fasteners land, and the supports overlap nothing.
    The frame is 2.86 mm from moving parts at plate level (z 56–62), at the same place near the rim on all four pods, under the 3 mm rule. The cutout's 0.2 mm simplification (`base.footprints`) may account for it; not fixed.
    The 3D fork-to-side-node gap at bump (2.52 in round 9) was not rechecked.
  - **SolidWorks robot rebuilt** with the two fork halves (unsprung) and the pivot washer (static); motion test passes (below).
- **Round 11 (Nov 4, Fernando's review of the SolidWorks robot; pod, chassis, upper robot and SolidWorks all regenerated):**
  - **Outer keel bulkier** (`pod_lean.py`): side plate 9.2 thick (was 6.2) and 16 tall behind the pivot (was 7); the gusset is now a full block from
    the side plate to the foot's end, as tall as the side plate. 7.45 cm³ (was 3.8). Its two M3 now run from flush counterbores on the block's top
    through the whole keel into the chassis (M3 × 30: 21.1 in the keel, 8.9 in the chassis). Pivot screw M3 × 20 (11.1 engagement) into a longer boss.
    Keel to rotating 8.2 mm, keel to fork 0.8 (the washer gap, as before).
  - **Inner frame through-bolt:** one M3 × 70 from a flush counterbore on the spring column's top (x 72.6, y 54.6, between the two support pilots)
    down through the column and foot into an 8 mm pilot in the ring, so the whole frame is clamped to the chassis.
  - **`pod_lean.py check`:** no interference at droop / nominal / bump; overhang as printed: keel 147 mm², inner frame 773 mm² (the new holes'
    round roofs).
  - **Chassis brace** (`chassis.py` `BRACE`): one member per pod from the keel's pocket on the side node back to the stem by the ring, parallel to the
    pivot axis behind the fork's base bar (clipped 3 mm off everything that moves). Each side node is now triangulated (stem, rim arc, brace).
    Keel pocket walls now run up to the keel's top (24 mm), the keel pad comes from `KEEL_FOOT`, and the ring takes the four through-bolts.
    `check`: everything lands (pod 8/8, keel 8/8, frame bolts 4/4, fins 6/6, bumper 16/16, 18 lap screws); robot Ø450.3; frame 2.86 mm from moving parts at
    plate level (unchanged, under the 3 mm rule); every part fits the bed (side nodes now 46 tall).
  - **Lidar pedestal 1.3 mm lower:** the vendor RPLIDAR model puts its scan plane 31.3 mm over its base (`first_floor.LIDAR`, was 30 assumed),
    so the scan is at z 163 as in the sim; 3.8 mm from the lidar's top to the second floor.
- **Round 12 (Nov 4, fixes from the review below; pod, chassis, upper robot and SolidWorks regenerated): a pod comes out with the robot assembled.**
  - **Pod screws from below** (`pod_lean.py`, `chassis.py`): the inner frame's foot takes three M3 (`FOOT_SCREW_Y` 23 / 34.5 / 88) and the keel's
    foot two (`KEEL_SCREWS`, foot lengthened to y 103), all driven up from under the chassis members (heads in 3.9 mm counterbores) into pilots in
    the pod. The three horizontal side screws are gone. The column through-bolt (M3 × 60) now stops in the frame's own foot (pilot −7..2): it
    clamps the column and foot, not the pod to the chassis. Landing: pod 12/12, keel 8/8.
  - **Slide-out path:** `chassis.slide_path()` sweeps each pod's plan outward along its −x for 260 mm (+1 mm) in two bands (members z 40–62,
    walls 62–92), and the chassis is clipped to it; the bumper mounts at 25° moved (`MOUNTS_Q0` 8 / 17 / 72 / 84°) to keep their landing; the
    front pods' side-fin dowels sit in slots 15 mm long toward the way out, so the dowels leave with the pod.
  - **Procedure:** robot on a stand (wheels at droop) or on its side; take off the pod's bumper quarter; undo the 5 screws from below; slide the
    pod straight out ~100 mm, unplug its XT30 (96 mm in from the bumper line), finish the slide. The motor lead needs ~120 mm of service loop
    at its grommet (not drawn).
  - **`pod_service.py`** (`validation/upper/pod_service.json`): per pod, a Ø6 driver along every screw from its head for 300 mm hits nothing
    (20/20); each pod's 60–62 parts slide 250 mm out against the other 424–428 parts of the robot with no contact. Continuous, not stepped: surface
    samples (~1 / mm²) cast along the path into the other part's triangles both ways, counting only crossings into a solid (sliding on the members
    is not a contact). Controls: bumper left on, the fork hits it at 3.2 mm; slid inward, the frame and keel hit the chassis at 1.0 mm.
  - **Upper fixes:** battery drawer, rear ribs, SO-101 base clamps and screws, riser screws, lidar screws, stacked power / MCU boards, mic and
    speaker fixings (below). Arm sweep rerun with the clamps and crown as keep-outs: 61 legs, nearest 12.3 mm, no contact.
- **Upper robot (`tools/upper.py` → `step_export/upper/`, `validation/upper/`; Nov 4, proposal):** layout F's second floor and everything on it.
  - **Second floor:** 5 mm plate, r 218, top z 181.5, in four quadrants (218 × 218, bed fits). Under it 6 × 6 lips round the rim and along the two
    side seams (x = 0), bolted on the bench by 8 horizontal M3 × 14; the rear seam lies on the spine's flange, whose five floor screws straddle it;
    the front seam is tied by the arm riser and adapter. 13 M3 × 12 down into the fins' flanges. Bosses under it give device screws ≥ 6 mm, except
    over the lidar. Three grommet holes for the motor leads / CAN and the USB leads.
  - **Arm:** cad/v10's reconciled riser and adapter (drawn for the same 181.5 floor and the sim's arm height) and its 12 V buck; the adapter's four
    screws lengthened so they clamp the riser's whole height (front two M3 × 35 into its lowest 10 mm, rear two M3 × 45 on into floor bosses).
    The SO-101's official parts placed by omni_arm's kinematics at the stow pose.
  - **Electronics block (layout F):** computer (BMAX B8A Pro envelope) in four corner clips, battery (OVONIC 6S 5200) in a tray with two strap slots,
    servo adapter by the arm. Power board and Teensy carrier stacked on one hole pattern (round 12: the carrier, our own board, is laid out on the
    power board's four corner holes): 6 mm spacer, power board, 40 mm printed post, carrier, one M3 × 60 per corner into a floor boss (9 mm).
    Labelled; plugs and leads from `layout_study.py` (the two rear USB-A plugs side by side: stacked they clashed).
  - **Battery out to the left (round 12):** the tray is open at both ends; open the strap, unplug the XT90 (reached under the deck from the right),
    slide the pack out +y under the deck. `check` sweeps it 180 mm: nothing in the way (the front deck posts moved 3 mm forward for it, the deck's
    front edge to x 16). Charging in place would also need the balance lead brought out: not drawn.
  - **Bin deck:** a 6 mm plate on four posts over the block (3 mm over the computer, which it holds down); the v10 bin on it. The arm reaches it
    (drop: tool at (−0.10, 0, 0.396) m, fingers 45° back).
  - **Rear floor ribs (round 12):** 10 × 6 ribs under the rear quadrants (as deep as the lips, above the lidar's band): from the spine's flange to each
    rear deck post, on to the rim, and from the post to the side fin's tail. The rear grommets moved to x −105, off the ribs.
  - **Face:** the 5 in panel on a printed frame behind the bin, facing forward (panel z 322..399). Its crown (now x −204..−147) carries the USB mic
    array on four Ø7 × 3 standoffs (M3 × 10 through its Ø3.2 holes, 8+ mm of thread) and the speaker in a cup under a grille cover (2 × M3 × 10 into
    Ø8 bosses). E-stop and power switch in printed pods on the deck's rear corners (the computer fills the space under the deck).
  - **SO-101 base (round 12):** its official model has no screw holes where v9 assumed four M3 (x 128 / 192, y ±45: just outside its walls). Front:
    two printed L-clamps, heel on the adapter, toe over the base's side wall (z 239.6), M3 × 30 into new pilots at y ±48.5. The rear half lies under
    the shoulder's sweep (r 37.7 round the pan axis at x 178.8, down to z 240.7), so no clamp there: 4 × M2.5 × 14 up through the adapter into the
    base's own Ø2.4 underside holes at (177, ±47) and (190.7, ±16.5), driven from the open side under the adapter (their depth VERIFY).
  - **Riser floor screws:** v10's two front ones (105, ±25) stood over the lidar with 5 mm of thread; moved to (97, ±40.5) in the riser's foot plate,
    over bosses, M3 × 12 (8 mm).
  - **Lidar:** v10's 4 × M2.5 × 8 up through the pedestal's top plate into its base, driven on the bench before the pedestal goes on (thread depth VERIFY).
  - **`check`:** every printed part one valid solid on the bed; no overlap anywhere in the upper robot or with the chassis, fins or pods. Thread
    engagement per screw (plastic between the pilot and clearance radii): all ≥ 6 mm; the M2.5s into the SO-101 base are listed apart (vendor holes).
- **Screen in front of the arm? No** (`tools/screen_study.py`, `validation/screen/`): the SO-101's own shoulder bracket and motor stand over x 205..217
  at z 241 at pan 0, in every pose (high, ready, during picks). So anything in front of the arm must stay below ~z 235, 50 mm over the floor; the 5 in
  panel is 77 tall and would be hit by 21 mm. Leaning it back far enough to clear the arm puts it into the adapter and the arm base.
  The planner's capsules alone had said it fits (15 mm): they miss the shoulder bracket (design.md 4.4's warning). Fits there: a strip display or
  two round eye displays up to ~45 mm tall.
- **Arm sweep** (`tools/arm_sweep.py`, `validation/upper/arm_sweep.json`): the official meshes through 61 planned tidy moves (stow, high, ready,
  floor grasps x 0.26..0.36 / y −0.12..0.12 both yaws, drop into the raised bin and back) against every static surface: no contact; nearest 12.3 mm (round 12)
  (gripper over the front node at the nearest grasp). The four corner targets at x 0.36, y ±0.12 have no IK. Sampling about one point per 3 mm² and
  every third planned step, so ±3 mm.
- **Print beds** (`tools/print_beds.py`; in SolidWorks each printed part has a `PRINT_BED (±X/Y/Z up)` plane at its bed face):

  | Part | Up | Supports | Why |
  | --- | --- | --- | --- |
  | inner frame | −x (outer face down) | hole roofs | spring and pivot loads in the layer planes; the M3 × 60 column bolt clamps it along z |
  | outer keel | +x (outer face down) | hole roofs | pivot and side loads bend the side plate in the layer planes; two M3 × 30 from below hold its foot |
  | fork outer / inner | +x | none | arm bending in the layers; the bar prints upright, so the tie bolts pre-compress its layers |
  | sleeve | +x (flange down) | none | |
  | wheel hub | +x (cartridge end down) | under the spokes' flat faces | radial loads in the layers |
  | cartridge | +x (plate down) | none | |
  | roller core / tread | +z (on end) | none | radial axle load in the layers |
  | chassis hub, edges, nodes | +z | none | plan and bending loads along the members; pocket walls bend across layers (they locate, screws hold) |
  | pedestal, D435 cradle | +z | none | compression |
  | bumper quarters | +z (on edge) | none | impacts bend the 3 mm wall across layers: PETG, drop-test it |
  | fins | −z (flange down) | none | compression down the legs; never lift the robot by the second floor (it would pull the legs across their layers) |
  | second floor quadrants | −z (top down) | none | lips and bosses grow up from the plate |
  | arm riser / adapter (v10) | +z / −z | none | the adapter's long screws clamp the riser's height |
  | clips, tray, spacers, posts, deck, e-stop / switch housings, face crown, speaker cover | +z | strap-slot bridges; the 22 mm holes' roofs | |
  | face frame | +x (back plate down) | the bezel lip's bridge | legs, rails and back in the layer planes |
  | SO-101 clamps | +x (on a side) | none | the L lies in the layer plane, so the clamping bend is in-plane |
  | SO-101 parts | as TheRobotStudio's print-ready STLs | | no plane added |
- **Fasteners:** every screw, dowel and nut is a record (`tools/hardware.py`), placed in SolidWorks as instances of magenta library parts
  (`fst_*`). Per robot (round 12): 64 × M3 × 50, 8 × M3 × 60, 2 × M3 × 45, 2 × M3 × 35, 16 × M3 × 30, 12 × M3 × 25, 22 × M3 × 20, 4 × M3 × 16,
  14 × M3 × 14, 43 × M3 × 12, 30 × M3 × 10, 2 × M3 × 8 (pan); 16 × M3 × 10 and 12 × M3 × 12 flat; 12 × M2.5 × 5 (hub to rotor, 2 mm thread in our
  rotor model: VERIFY); 4 × M2.5 × 14 (SO-101 base, VERIFY) and 4 × M2.5 × 8 (lidar); 4 dowels Ø3 × 10; plus the 16 spring plungers
  (M4 × 40 + washer + nyloc), 4 Ø7 × 20 pivot dowels and 4 M4 washers.
- **Review (Nov 4) and what round 12 left open.** The review found: a pod could not come out without lifting the second floor (all its screws
  under the floor, the front pods carrying the side fin's seat); the battery under the bin deck; the floor's rear corners hanging ~150 mm on the
  rim lip; no screws for the SO-101 base, the lidar, the mics or the speaker. Round 12 fixed all four (above). Still open:
  - **Strength (no analysis or print):** fins, riser box, keel block and the bin deck's four 14 × 14 × 58 posts (one screw at each end); the face
    frame on two 14 × 7 legs; pocket walls, bumper and fins load across layers as in the print-bed table. Never lift the robot by the second floor.
  - **Service details not drawn:** the motor leads' service loop (~120 mm each), a charge port (charging in place needs the balance lead brought
    out; today the pack slides out to charge), cables and the harness (only plugs and short leads are modelled).
  - **VERIFY:** M2.5 engagement in the SO-101 base's own holes (1.9–2.0 mm in our model of them) and in the lidar's base; the rear deck posts'
    floor screws are still driven from below the floor (reachable with the floor off only).
  - **Collisions:** none found statically (pods, chassis, upper robot, the arm at stow), in the arm's planned moves, or along any pod's way out;
    the fork-to-side-node gap at bump and the 2.86 mm plate-level clearance stay as above.
- **Base** (`tools/base.py` → `step_export/base/`, `validation/base/`):
  - The four pods use the sim's hands, with each wheel slid along its own rolling line (kinematics unchanged).
  - A printed plate in four petals, a spider under the centre and a slim bumper.
  - The study gives Ø440 at D 146. The current pod would need about Ø490.
  - Its full `check` was not completed.
- **Layout study** (`tools/layout_study.py`, `validation/layout/LAYOUT.md`):
  - The deck between the pods cannot take the electronics at any reasonable size.
  - Battery, computer (modelled plugs and leads) and power go in one block on the second floor's rear.
  - Proposed F: sim hands, D 154, slide −20, Ø462, lidar on a centre pedestal. Fernando: go with F (Nov 3).
- **Chassis for F** (`tools/chassis.py` → `step_export/chassis/`, `validation/chassis/`), replacing the plate below:
  - **Why:** the first floor is a frame, not a plate. The plate tied pods through 19–25 × 6 mm necks.
  - **Members:** solid square sections, 22 mm deep (z 40–62), printed as box sections with 3–4 walls and low infill (1,126 cm³ solid; mass set by the slicer).
  - **Ring:** on |x| + |y| = 100–121, under the pods' inner frames.
  - **Cross:** a cross on its diagonals triangulates it.
  - **Nodes:** four nodes run out to the rim.
  - **Assembly:** nine printed parts (hub, 4 edges, 4 nodes) joined by 12 half-laps with 18 M3 × 20.
  - **Pods are held, not just screwed:**
    - The inner frame stands in a pocket (0.2 mm clearance). Its 14 mm walls hug its centre-side face and front end.
    - It is fixed by 3 M3 × 25 up from under the member into its foot (round 12; round 11: 2 from above plus 3 horizontal through the wall); its
      pivot block bears on the ring.
    - The keel stands in a walled pocket on the side node with 2 M3 × 30 up from under the member.
    - Members and walls are clipped 1 mm off the path every pod part sweeps going out radially (round 12), so a pod slides out under the second floor.
  - **Lidar:** A2M12 on a pedestal on the front node at x 120 (sim x 140 puts its top plate into the D435 cradle), scan z 163. It clears the pods by 6.5 mm and the cradle by 12.7.
  - **Second-floor supports: three printed fins** (`validation/chassis/fin_profiles.png`, `iso_supports.png`):
    - **Fin plane:** each fin stands in a vertical plane through the lidar's axis. At the scan plane the lidar sees only its 5 mm web edge-on, however long the fin is.
    - **Using the length:** a 16 mm flange runs the whole length under the floor. Two legs, an elliptical arch between them, coves out to the tips. Legs flare toward the deck.
    - **Spine** (centreline, behind the lidar, 196 long): feet on the cross by the hub and on the back node, 2 × M3 × 30 up through each member. It sits under the battery and computer.
    - **Side fins** (bearing ±136.7, 147 long): they lie on the ray through the front pod's two column-top pilots, which also passes under the arm riser's rear corner (62, ±56).
      - Nose under that corner (the arm's front support).
      - Seat on the column top, located by two Ø3 × 10 dowels pressed into its support pilots.
      - Foot on the ring by the side stem, 1 × M3 × 30 up through it. That leg is shifted 2.5 mm off the rear pod's fork.
    - **Lidar:** nothing stands in the lidar's front half. Blind 9.4° in total (was 13.6° with the blade envelopes): spine 2.0° straight back, side fins 3.7° each.
    - **Rear pods:** their column pilots are unused.
    - **Floor:** screws down into 2.8 × 7 pilots in the flanges (13).
    - **Printing:** upside down on the flange, no supports.
    - **Not checked:** the floor itself and the riser box are not designed, and there is no strength analysis.
    - The arm's support relies on the riser being a stiff box on the floor, cantilevering about 80 mm from the fin noses to the arm base at x 140.
    - A front pair of Ø5 rods at the front arc would cost about 3° each in the forward sector. Not drawn.
  - **Bumper:** impact only. 3 mm PETG quarters with countersunk flat-head M3 from outside; robot Ø450.3 with round-10 pods (456.5 with round 8).
  - **`check`:** one valid solid each, all fit the 256 bed, 3 mm from everything that moves (pocket walls 3.2, pedestal 2.5; round 10: frame 2.86 at plate level), every screw lands in a member.
    - The fins are 3.21 mm from moving parts, plungers included, and overlap nothing static.
    - The side fins sit 0.02 mm above the column top (modelled bearing contact).
  - **Not done:** strength analysis or print.
- **First floor for F, plate version (superseded by the chassis)** (`tools/first_floor.py` → `step_export/first_floor/`, `validation/first_floor/`):
  - **Wheels:** all four on one circle (r 155.3) at ±37.6°, not 45°. Each sits on its 45° rolling line, so the kinematics are the X4's.
  - **Deck:** four PETG petals, 6 mm, top z 62, R 224.7, each about 140–150 cm³. They split on the diagonals; each split jogs between a pod frame's two foot screws, so every pod ties two petals.
  - **Spider:** an 8 mm X under the centre ties the petals and carries the lidar pedestal (A2M12 scan at z 163; its hole pattern is a placeholder).
  - **Pods:** each attaches with 4 × M3 × 14 straight down into the plate (6 mm engagement) and lifts out from above.
  - **D435 cradle:** at the sim's nose pose, with a right-angle USB-C plug. A straight one reaches the front wheel.
  - **Second-floor posts:** eight radial-blade posts (5 × 25 mm), feet only so far. Two sit under the arm in front of the lidar, four at the sides, two at the back. The lidar is blind about 17° in total.
  - **Bumper:** four PETG quarters (6 mm wall, z 40–90, dipped to z 52 across the nose so the D435 sees the floor from x 267). Each quarter has 4 radial M3 × 12 screws from outside into rim bosses under the plate, plus lips on the plate top. To remove: 4 screws, lift, pull out.
  - **`check`:** every part is one valid solid that fits the 256 bed.
  - **Clearances to moving parts:** plate 2.8, bumper 3.0, D435 cradle 9.0 / camera 10.9 / plug 5.4.

## The pod
- **Mounting to the robot plate:** the plate is flat with its top 3 mm above the axle (73 mm above the floor), as v10's deck. The pivot is at plate height.
  The pod drops into a cutout and its two side frames stand on the plate top beside it. Each frame takes **3 vertical M3 screws from the top**: x -67.5 / +64.7, y 25, 35 and 98 in the pod frame.
  The screws go into Ø2.8 pilots in a printed plate (≥ 8 mm deep), or through the plate into nuts. The pod comes out from above without turning the robot over.
  The plate needs (assuming 6 mm thick):
  - the moving parts' cutout x -63.5..57.7, y -70..95.6 (`checks.json` plate; add about 2 mm all round);
  - a notch under each frame's keel at x -73.5..-61.5 and 58.7..70.7, y 41.6..92.

  The pod is 174 mm long (wheel front to frame back, y -70..104; 192 before round 5).

  Side holes are not used: a thin plate edge is a weak place for screws.
- **Hinge level with the axle** (design.md 2), 80.6 mm from the axle at axle height, **with no shaft**: one M3 x 16 per side (round 7).
  - Each fork arm carries a 627, pressed into a seat in its outer face (0.1 mm under it); a Ø18 x 1 relief behind the seat keeps the seat bottom off the inner race.
  - Each frame's keel has a printed **pivot pin** (Ø7 x 6.9, in the 627's inner race) on a Ø10 x 1 boss that bears on the inner race and spaces the fork 1 mm off the frame.
    The bosses locate the fork axially (0.1 mm each side).
  - An **M3 x 16 pan head** from the keel's outer face (head counterbored flush, Ø7 x 2.6) threads all its length through the keel, boss and pin (Ø2.8 pilot; tip 1.3 mm short of the pin end).
    It is the printed pin's steel core and is held in the static frame. The pin's layers run across it (the frame prints on its outer face), so the screw takes the bending.
  - Assembly: press the 627s into the fork, set it between the frames with the pins in the bearings, then a screw into each pin (before the pod goes into the plate).
  - Load: about 26 N per pin at 10 kg, 4.6 mm from the keel face to the bearing centre; not tested.
  Axle travel is -3 / +15 mm (12.9 deg), as in the sim baseline.
- **Fork** (one printed part, 118 x 125 x 59 mm, 167 cm3; the beam is solid now that no shaft runs through it):
  - the outer arm takes a bolt-in **axle stub** into the wheel's left 6804;
  - the inner arm is bolted straight to the **motor's rear face** (4 x M3 x 12 in counterbores; rear holes Ø2.8 x 4.3 in the maker's STEP, taken as M3, VERIFY);
  - a beam behind the wheel joins the arms: the Ø30 pivot lobe across, cut 4 mm clear of the wheel; nothing behind the pivot (the tail went with the old stops);
  - a flat pad on each arm (26 mm long) sits under its two spring plungers.
- **Side frames** (two printed parts, 54 and 56 cm3, 84 mm long, each 12 mm thick with a flat outer face: print lying on it). Each has:
  - a foot on the plate;
  - a spring column and a tower over the fork arm (top 54 mm above the plate; 26 mm long, two Ø8.6 spring bores 10.6 apart, walls 2 mm between them, 3.4 mm outside and 4.2 mm toward the wheel);
  - a keel down through the plate notch carrying the pivot pin;
  - a full-height web from the column to the keel and a gusset from the column (z 50) down to the foot top (y 90), with R5 fillets and R2 at the column foot.
    The spring pushes the tower up and the pivot pulls the keel down, so the frame bends between them; before round 3 only the 8 mm foot joined them there;
  - the **hard stops, at the springs**: two **bump rails** under the tower, either side of the spring washers (1 mm clear), whose sloped undersides land flat on the fork pad at bump;
    a **droop hook** from the keel 6 mm under the fork arm (y 46.6..62.6), whose top is the arm's underside at droop. The inner arm is deeper (motor plate), so the inner keel reaches z -28.9 (outer -23.2).
- **Springs, no damper:** **two yellow 8 x 25 mm 3D-printer bed springs per side** (Ender 3 / CR-10 bed-levelling upgrade, sold in 10-12 packs; 4 per pod, 16 per robot).
  They copy the JIS extra-light die spring for an Ø8 hole / Ø4 rod x 25 free (MW Components DJ-8X25Y: 6.1 N/mm, 10 mm long-life deflection = 40 %, 12.5 mm max).
  **The Amazon packs give no rate: 6.1 N/mm is assumed (VERIFY: stand one on a kitchen scale, press it 5 mm, rate ≈ grams x 0.00981 / 5).**
  - Each spring sits on an **M4 x 40 button-head bolt used as a plunger**. Its dome presses on the fork pad; the pair sits 31.3 and 20.7 mm from the pivot.
    The shank slides in a Ø4.4 guide in the tower cap, an M4 washer seats the spring, and an M4 nyloc on top keeps it captive.
  - At 10 kg each spring carries 19 N (3.1 mm compression). Bump: 9.5 mm (far spring) / 7.5 mm (near), under the 10 mm long-life limit. Droop: still 2.0 / 2.4 mm compressed (12 / 14.5 N).
    Plunger lift -1.15 / +6.43 (far) and -0.75 / +4.43 (near). Wheel rate 2.6 N/mm; the sim wants 0.8-3.
  - The geometry is set for 10 kg. Another mass or measured rate needs a printed shim of `shim_mm` (`checks.json` spring table) under each spring to keep the ride height.
    Above about 12 kg the far spring passes 10 mm at bump (12.5 kg: 10.3; 15 kg: 11.1; under the 12.5 mm max up to about 18 kg).
  - One spring per side would reach 11.6 mm (46 %) at bump wherever it sits on the fork: less force from a longer lever means more travel.
  - Rounds 2-3 used one ISO 10243 Ø10 x 38 die spring per side (green, McMaster 1804N112, 6.7 N/mm) on an M5 plunger: an exact fit, not as easy to buy.
  - Torsion springs were ruled out: off-the-shelf ones reach about 0.3 N·m, against the 2-4 N·m needed.
- **Contact and clapping:** every spring is still compressed at full droop, and a plunger (about 4 g) is pushed by at least 12 N, so it follows the fork and never leaves the pad.
  The nyloc stays 0.35 mm above the tower at droop, so it never holds a plunger off the pad. What can clap is the fork on the hard printed stops (the pad on the bump rails, the arm on the droop hooks) when the wheel drops off a step or hits a big bump.
- **Checks** (`pod.py check`, droop / nominal / bump): no interference anywhere.
  - Clearances: frames to anything rotating ≥ 7.4 mm (the towers' inner wall, at bump); frames to the motor leads ≥ 40 mm; fork to anything rotating 4.0 mm; fork to the leads 3.8 mm (they move together); axle stub 1.5 mm radially inside the cartridge lip.
  - The stops touch the fork exactly at droop and bump (0.7 mm clear at nominal). They act 23-26 mm from the pivot, so 0.2 mm of print error moves the axle's end of travel by about 0.6 mm
    (the old tail ledges, 11-25 mm behind the pivot: about ±1 mm).
  - Each plunger dome stays on its pad: it slides up to 5.9 mm, leaving 1.8 mm of pad at bump. The far plunger's washer clears the tower underside by 0.6 mm at bump.
- **Hardware per pod:**
  - pivot: 2 x 627 (in the fork), 2 x M3 x 16 pan head (into the frames' printed pins; no shaft);
  - fork and stub: 4 x M3 x 12 pan (fork to motor), 3 x M3 x 12 pan (stub);
  - springs: 4 x yellow 8 x 25 bed spring, 4 x M4 x 40 button head (ISO 7380), 4 x M4 washer, 4 x M4 nyloc;
  - mounting: 6 x M3 into the plate (length = foot 8 + engagement).
- **Printing:** each printed part is one solid. Frames print on their outer face. The fork is still heavy and blocky; lightening it is a later pass.

## The wheel + motor
Ø140 wheel (rollers on r 52, tread profile R70: same as the v10/v9 wheel and the sim). One printed hub (drum r 30..33, 2 x 6 spokes with R2.5 root fillets, axle blocks), x -39.5 .. +29, 234 cm3 solid.
- **Rollers** drop in radially after printing; M3 x 50 axles slide in along their own axis into Ø2.9 pilots (no nuts, 6.5 mm engagement). Ø3.4 bore, 0.75 mm thrust journals, flat ends, R2 TPU end fillets,
  half length 19.5 (2.35 deg overlap), rows at ±17.25 mm. Each spoke's flat thrust face runs across the whole roller end (1.05 mm from the end face) and blends R3 into the 0.95 mm tread clearance.
- **Roller core and tread (round 6):** a straight Ø25.4 PLA core and a TPU tread with a straight bore, keyed by three 2.4 x 1.5 keys open at both ends, 0.5 mm lead-in chamfers.
  The tread slides on and off along the axis (the old barrel-shaped core needed the TPU stretched 2.8 mm in radius over its middle, so a printed tread could not be removed).
  The tread is 2.5 mm thick at the ends and 5.3 mm in the middle (more TPU at the contact).
  **Round 7:** it snaps into place. A 45° V groove, 0.5 deep, runs round the core's middle, and a matching 0.4 mm rib inside the tread bore sits in it (0.1 clear at the tip).
  Fitting stretches the bore 0.4 mm in radius (3 %) over half the length, which is meant to work by hand; the rib's 45° flanks let it pull off by hand. Both print upright without support (`validation/wheel_motor/section_roller_detent*.png`).
  Untested: how hard it is to push on or pull off, and whether 0.4 is enough to stop walking under side load.
- **Hub round 8 (Nov 3, Fernando's sketch): head pockets.**
  - The round-6/7 double-teardrop tunnel ("heart" section) left cantilevered slivers of web where it passed through the spoke.
  - Each pocket is now a round Ø3.4 shank hole, a round Ø7 × 3 head counterbore, then a 45° cone opening outward along the axle for 5 mm (to r 8.5, past the block), then Ø7 on for the head and driver.
  - **Cone length is set by the neighbouring roller.** The two axles sharing a spoke cross 4 mm past their heads, so a longer cone cuts into the other roller's head seat: 5 mm leaves 99 % of it, 7 mm 61 %, 9 mm 30 %.
  - What is left over 45° near the pockets is round-hole roofs: the Ø3.4 shank, the Ø7 counterbore, and the Ø7 head path where it passes the neighbour's block (about 90 mm² per spoke).
  - The axle head (Ø6.4) still passes along every axle.
- **Hub round 7 (Nov 3):** the Ø3.4 shank hole was made a teardrop (replaced by round 8).
  - R0.8 on the spoke webs' x-end edges (each spoke filleted before it joins the drum; OCC refuses them on the finished hub) and on the drum's left end.
  - Still overhanging, unchanged: each row's web inboard face at x ±3 (the other row is 30° away, so nothing is under it) and the flange roof. About 8,100–11,600 mm² in total depending on which end is down, so the hub still needs some support there.
- **Spokes (round 6):** each row's webs run from x ±3 to x ±29.25 (row 2 stops at the drum end, x 29), 26 mm deep instead of 16. The cut for each roller is its revolved clearance envelope plus
  the radial drop-in channel, not a prism through x, so the web beside the roller end stays and wraps it.
  At the thin neck (r 42..44) the section across the spoke is about 129-143 mm2, against about 30 mm2 before (1.9 x 15.9); the 1.9 mm tangential width at the row plane is unchanged.
- **Axle head pockets (round 6; round 7 also the shank hole):** each head pocket and its driver tunnel is Ø8 (was 6.4) with 45 deg points toward ±x (a double teardrop), so it prints without support with either hub end down
  (`validation/wheel_motor/section_head_pocket.png`). The insertion check still uses a Ø6.4 head: it grazes the neighbouring tread by up to 9 mm3, as before.
- **Bearings:** the hub rides on the motor's output bearing and the left 6804. The 6804 presses into the **cartridge** from its open side; the cartridge has no floor, so there is no trapped cavity and it prints plate-down without support.
  The 6804 floats axially; the motor bearing locates the wheel.
- **AK40-10** sits outboard (output face x 4.5, rear 44.7), turned so its radially mating XT30 points back and slightly up (az ~160 deg), away from the pivot and springs.
  The plug and leads sit outside the hub, 4.7 mm from anything rotating.
- **Hub flange:** the flange plate (x 1..4.5) joins the drum by a 45 deg cone, and the left end wall runs out on a second 45 deg cone, so the hub cavity needs no internal support printed either end down.
- **Own motor model:** the maker's STEP is surfaces only (B-spline faces, no solid). `wheel_motor.build_motor` is a solid stator + rotor with the measured interfaces: can Ø53, output disk Ø37 with
  3 x M2.5 at r 13.5 and dowel holes, rear 4 x Ø2.8 at r 23.5, front 6 x Ø2.6, and a socket pocket shaped by the real plug. Only the plug and its pins come from the maker's file.

## SolidWorks robot (viewing assembly, Nov 3; complete robot Nov 4)
The whole proposed robot (chassis.py's layout F + four round-12 lean pods + upper.py; rebuilt Nov 4 for round 12, 660 components), for looking at it in SolidWorks. Imported STEP bodies, not native features, except the two vendor parts. Every printed part has a `PRINT_BED` plane.
`python -B tools/sw_robot_parts.py` (omni-robot-sim) writes `solidworks/robot/step_parts/` + `manifest.json`; then
`C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_robot.py import`, `... roller wheel motor unsprung static`, `... pod groups robot test` (each stage first closes the robot documents; SolidWorks threw internal errors without that).
  Nov 4: SolidWorks died (RPC server unavailable) building several big groups in one call; build them one per call: `group_frame`, `group_mounts`, `group_bumper`, `group_reference`, `group_upper`, `group_electronics`, `group_arm`, `group_fasteners`, then `robot test`.
- **Tree:** `V11R_robot` > `V11R_frame` (hub, 4 edges, 4 nodes), `V11R_mounts` (lidar pedestal, D435 cradle, 3 fins), `V11R_bumper` (4 quarters),
  `V11R_reference` (vendor D435 and RPLIDAR parts; envelope for the D435's plug), `V11R_upper` (second floor, riser + adapter, holders, posts, bin deck, face frame, pods for the e-stop / switch),
  `V11R_electronics` (computer, battery, power board, Teensy carrier, servo adapter with plugs and labels, 12 V buck, bin, display, mic array, speaker, e-stop, switch), `V11R_arm` (SO-101 at stow, fixed),
  `V11R_fasteners` (159 screws / dowels outside the pods, the pod and keel screws from below among them), 4 pods. Each pod: `V11R_pod_static` (inner frame, keel, sleeve, dowel, pivot washer, 4 springs, pivot screw, column through-bolt),
  `V11R_unsprung` (fork outer half with its stub, fork inner half, 2 x 627, 4 tie bolts, 4 motor screws, `V11R_wheel` with 12 x `V11R_roller`, 12 axles, 3 cartridge + 3 rotor screws, `V11R_motor`), 4 plungers.
- **Vendor parts** (`sw_robot.VENDOR`; `import` copies them to `parts/V11R_d435.SLDPRT`, `V11R_rplidar.SLDPRT` when the source is newer; poses in `manifest.json` `vendor`, from `sw_robot_parts.vendor_poses`):
  - D435 from `cad/other/Realsense to bucket/IntelRealsense_D435_Multibody.SLDPRT` (8 bodies). Its back face (2 x M3, 45 apart, as the cradle's holes) is on the cradle's back plate, centred on the old envelope, tilted 30° down.
    The model has no USB-C port, so which end faces the plug envelope (camera -y, robot right) is still VERIFY.
  - RPLIDAR from `Roomba robot/CAD/v2/RPLIDAR.SLDPRT` (Ø76 x 41, the A2M12 envelope's size), base on the pedestal top, its `Laser_link` x forward.
    Its `Laser_link` is 31.3 mm above the base; since Nov 4 `first_floor.LIDAR` uses that, so the scan plane is at z 163 as in the sim (VERIFY on the unit).
- **Mates only in the suspension:** the unsprung mass hinges on the inner dowel (concentric + Right Planes); each plunger is concentric in its guide and tangent to the fork pad;
  `Suspension_travel` limits the `inner_a1` plunger lift (axle -3 .. +15). Everything else is fixed where build123d put it (wheel and rotor don't turn). Pods are flexible in the robot, so a fork can be dragged there.
- **Hands:** FL and RR use `V11R_pod_m`, built from mirrored STEPs (`*_m`), as chassis.py's STEP. Its motor is the whole motor mirrored, not the real opposite-hand plug position (open item below).
- **Orientation:** the top assembly turns the robot so robot z is SolidWorks Y (upright in the standard views) and its front faces the Front view; the sub-assemblies stay in their build123d frames.
- **Test** (`sw_robot.py test` → `solidworks/robot/motion_test.json`, passes): both hands land on axle z -3.0 / 0 / +15.0 at droop / nominal / bump with every plunger at its build123d lift (0.01 mm), no mate errors;
  a drag past each end stays inside the travel; the robot opens with four flexible pods at their poses, at nominal (298 components). Views: `validation/robot/solidworks_robot_*.png`.
- Not included: cables and the harness. It inherits the proposals' status: nothing here is adopted geometry or checked beyond chassis.py / pod_lean.py / upper.py / arm_sweep.py.

## SolidWorks
`C:/Users/ferna/cadenv/Scripts/python.exe tools/sw_build.py import roller wheel motor fork chassis pod test` (after `sw_parts.py`; a few stages per call, SolidWorks leaks memory).
It uses cad/v10's helpers (`subasm`, `mate_api`, `sw_native`) unchanged.
- **Mates:**
  - fork: Right Plane on the outer frame's, and its 627 bores concentric with the outer frame's pivot pin (it hinges);
  - motor: stator locked to the fork, hub locked to the rotor (the rotor turns in `V11_motor`);
  - rollers: each concentric on its hub hole with a 0.3 mm thrust gap (turns);
  - plungers (four: `outer_a/b`, `inner_a/b`, a nearer the axle): each concentric with its tower guide, with its dome **tangent** to the fork pad, so it rides up and down as the fork moves;
  - `Suspension_travel`: a distance limit between the `outer_a` plunger and the outer frame's foot top, for plunger lift -1.15 .. +6.43 mm = axle -3 .. +15.
  The wheel and motor are flexible sub-assemblies. The springs are shown at their nominal length (they don't compress in SolidWorks).
- **Motion test** (`sw_build.py test`, `solidworks/motion_test.json`, passes):
  - With the limit suppressed, the fork set to droop / nominal / bump lands on axle z -3.0 / 0 / +15.0 with every plunger at its build123d lift (to 0.01 mm), with no mate errors.
  - With the limit on, a GUI-style drag (`IDragOperator`) moves the fork and all four plungers and stays inside the travel.
- Saves can fail with error 1 while Dropbox is still uploading the previous save; `sw_build.py` retries them.
- An angle limit between fork and frame planes left the fork fully defined, which is why the limit sits on the plunger. API `Transform2` cannot move a part held by a limit mate; a GUI drag can.
- Component names are SolidWorks' file names (`V11_motor-1` ...); the scripts map them by file. STEP colours are written per face, in sRGB (`tools/step_colour.py`).

## Open items
- **Motor output bearing load:** it now carries part of the wheel load; its rating is unknown (VERIFY with CubeMars). Rear-hole thread (M3?) VERIFY.
- **Spoke neck:** still 1.9 mm tangential at the row plane (r 42), now flanked by the deeper web (about 129 mm2 across the spoke at r 43-44). Further options if a printed hub cracks there:
  journals 0.5 + gap 0.25: 2.5 mm; rollers 1 mm shorter (half length 18.5, overlap 0.65 deg): 3.7 mm; both: 4.1 mm.
- **Round 6 not reprinted yet:** SolidWorks parts (`V11_wheel_hub`, `V11_roller_core`, `V11_roller_tread`) are not rebuilt from the new STEPs (`sw_parts.py`, then `sw_build.py import roller wheel ...`).
  The tread's fit on the straight core and whether the TPU walks axially under side load are untested. `hub_overlaps_mm3` now reports nothing because OCC fails the boolean against the threaded
  bolt STEP; a Ø3 rod in each pilot overlaps 2.9 mm3, so the pilots are intact.
- **Springs:** measure the bed springs' rate and OD/ID (modelled 7.7 / 4.3; VERIFY) and set `SPRINGS` / the shim. The button heads slide on the PETG pad (a steel wear plate could be added).
- **Damping and end stops:** no damper (design.md: damping matters little in the sim), so a bump rings at about 5 Hz with only roller and friction losses. The fork hits hard printed stops.
  Cheapest next step if the prototype clatters or bounces: TPU pads on the bump rails and droop hooks (re-cut the stop faces by the pad thickness), then an O-ring rubbing on each plunger shank for friction. A hydraulic damper is not planned.
- **Mirror-hand pod** (opposite diagonals, design.md 2) is not generated. It needs mirrored frames, fork and hub (the hub's flange holes) and the motor turned to put the plug at az ~20 deg.
- **Frames without a shaft:** off the robot the pod holds together only through the fork; on it the plate ties the frames (6 screws). If the plate flexes, a printed cross tie between the frames' feet is the fix.
- Lead route from the plug into the robot harness (modelled to x 56.7, inside the cutout); the robot plate itself.
- Spokes print with external supports under their flat x faces (now x ±3 and ±29.25), and the roller pockets in the flanks overhang up to about 49 deg at the inner flank edge; axle bolt heads graze the neighbouring roller's TPU tread on insertion (upper bound 9 mm3 per tunnel).

## History
- Wheel round 6 (Nov 3, after Fernando printed the hub and the v10 rollers): head pockets and driver tunnels Ø8 double teardrops (supports in them were hard to remove; bigger driver);
  spokes 26 mm deep wrapping the roller ends (neck section about 4x); straight roller core so the TPU tread can come off.
- Pod round 7 (Nov 3): the pivot is a printed pin on each frame with an M3 x 16 core threaded into the frame; the 627s are back in the fork; no sleeves, the keels are back to round 5 (pod 174 mm).
- Pod round 6 (Nov 3): the Ø7 x 144 shaft and its M3 cross pin are replaced by one M3 x 20 per side through a printed sleeve in a 627 in each frame (superseded by round 7).
  Since then the frames are tied only by the robot plate (and through the fork).
- Pod round 5 (Nov 3): hard stops moved from the fork's tail to the springs (bump rails under the towers, droop hooks under the arms); the tail behind the pivot is gone,
  the keels and feet end 11.4 mm behind the shaft. Pod 18 mm shorter, fork 11 mm shorter and 23 cm3 lighter. The 80.6 mm arm stays (set by the wheel: shaft and beam pass behind it).
- Pod round 4 (Nov 3): yellow 8 x 25 3D-printer bed springs, two per side on M4 plungers, replace the Ø10 x 38 die springs; spring pair centred 26 mm from the pivot; towers 12 mm lower and 26 mm long.
  SolidWorks rebuilt (four plungers), motion test passes. The rate is assumed until measured.
- Pod round 3 (Nov 3): the notch between the spring column and the keel (the frame's weak point) is filled by a web and gusset; towers 2 mm wider each way, fillets on the profile's
  inside corners. SolidWorks rebuilt, motion test passes; `validation/pod/solidworks_pod_outer.png`. No strength analysis or print yet.
- Pod round 2 (Nov 3): the RC shocks were too soft, so ISO 10243 die springs on button-head plungers replace them (torsion springs ruled out). The roof chassis is replaced by two side frames on the plate, with the pivot at plate height as v10.
  The cartridge has no floor (no trapped cavity). The plunger carries the SolidWorks travel limit.
- Round 5 / pod round 1 (Nov 3): motor mount dropped (the fork bolts to the motor), motor turned to az 160, second hub chamfer, own solid motor, face colours, shock pod, SolidWorks assemblies.
- Round 4 (Nov 2, option 1): the XT30 plug mates radially (pins r 31.3, leads r 35.9), so the motor moved 27.5 mm outboard; the hub ends at x 29; the right 6804 went. Rule since: `cad/solidworks.md` section 16.
- Round 3 (Nov 2): real plug and leads, thrust cut across the whole roller end, roller bore 3.6 -> 3.4, one script + `dims.py`.
- Round 1-2: `cad/v10 human editable/` (`step_export/wheel_motor_v2`, `tools/human/70_wheel_motor_v2.py`, ASSEMBLIES.md).

# cad/v14: the whole robot, clean, with the bearing roller wheel (Nov 13 2026)

Started Nov 13 2026 (Fernando) as a clean copy of cad/v13: only the scripts that generate and check the robot, its reference files, and a
freshly built SolidWorks viewing assembly. **One design change from v13: the bearing roller wheel is the baseline** (every roller on two 687s
round a clamped sleeve, small round petals, the tread with tip relief; "The wheel" below). Everything else is v13 as of Nov 13 (layout F,
lean pods, the D435 on the arm riser, the harness and power stack, the filament colours). The history (rounds, studies, the D435 placement
scan, the stress models, the motor and pod jigs, the print-fit coupons, the wheel prototypes) stays in `cad/v13/` and its README.

Status: **proposal**, nominal CAD; Fernando printed and assembled rollers on Nov 13 (the M3 x 50 barely threaded and the 687s were hard to get flush: fixed in CAD that evening, not yet reprinted); nothing is load-tested. Physical tests still open are listed under
"Open / VERIFY".

## Nov 13 review (Fernando): printed-part rules and pod fixes

Four general rules, now in `cad/solidworks.md` section 18 (exceptions go to Fernando for approval): **no captive nuts** (a nut sits on an outer
face in a 5.5 AF x 2.5 hex recess, `hardware.hex_recess`); **every hole through its part** (`hardware.make_through`: the build scripts drill
each screw's blind hole on out of its part where ≤ 20 mm of plastic is left, and list the rest); **flat-head countersinks 0.5 deeper**;
**no dowels or special shafts** (an M3 in printed PLA).

| Where | Change |
| --- | --- |
| Wheel | pockets Ø14.05 and sleeve Ø7.15 in the bearings (`coupon_687` cells 2 / 4); a thrust land on each hub face, 0.2 off the 687's static inner race (keeps the roller centred, max 0.2 walk); axle and cartridge pilots through; cartridge countersinks 0.5 deeper ("The wheel") |
| Pod: fork halves | tenon 0.05 a face (was 0.1: too loose); an engraved "up" arrow on each half's top face as printed (outer arm's wheel side at y 45; motor plate's outer face at the axle), 9 x 5 x 0.4 |
| Pod: inner pivot | the Ø7 x 20 steel dowel is gone: a **printed Ø7 pin on the inner frame** (one part; it prints upright on top) in the inner 627, with an **M3 x 16 from the frame's outer face threaded along its axis** as its core (16 mm of thread, tip 1.0 inside the pin) |
| Pod: chassis screws | no hex nuts in slots: the three foot screws (M3 x 25, 6.2 mm thread) and the column bolt (M3 x 60, 14.2 mm) thread into **Ø2.9 pilots**, as the keel's (round 14; evening: the foot screws Ø2.8, below); every hole through the frame; the side fin's support pilots Ø2.9 through the column |
| Pod: 6804 retainer | new printed `pod_retainer` (1.5 cm³): a spigot centres it in the 6804's bore, its Ø23 flange bears on the inner race's inner face, an **M3 x 25 from the outer arm's outer face along the axle** (counterbore, through the stub) threads 6.2 mm into it. The bearing cannot slide in, and the stub's shoulder (gap now 0, was 0.3) clamps the inner race: the fork bears on the bearing. Nearest turning part 4.55 (the cartridge). Pressed into the bearing from inside the hub before the cartridge goes on; the screw is driven on the bench |
| Pod: motor leads | a **zip tie** (drawn 3.6 x 1.2 strap + head, `tie_lead`) holds both leads to the inner fork half, which they move with: through a tangential 4.6 x 2.0 tunnel just inside the motor plate's rim (2.0 bridge) at x 52 and round both leads. ≥ 15.75 mm from anything that turns, 44 from the static frame, in all three poses; 0.1 off the leads as drawn. The old lead notch is gone (the leads pass 2.7 outside the rim) |
| Chassis | the side fin's two Ø3 steel dowels per front pod are **M3 x 8 pan heads** screwed into the column top, their heads the locating studs in the fin's slot (6.9 wide, open toward the pod's way out); 28 blind pilots drilled through |
| Upper floor | nuts in hex recesses on faces: the 12 V buck's two (riser wall, inside face), the servo bracket's two (riser back wall, inside face), the D435 cradle's two (each ear's inner face; M3 x 14, was x 12, was a drop-in slot); deck posts one pilot through each; 6 blind pilots drilled through |
| Coupon | `coupon_627.py`: fit coupon for the loose 627 fork seats ("627 fit coupon") |

**Nov 13 evening (Fernando):**

| Where | Change |
| --- | --- |
| Pod: inner frame pilots | the short threads in **Ø2.8** (were Ø2.9): the three foot screws (M3 x 25, 6.9 mm of thread by the audit) and the side fin's two M3 x 8 studs; the column bolt (14.2 mm) keeps Ø2.9 (long thread, section 18). The hole audit now lists the three foot screws as "across layers" (Ø2.8 in the frame's layer plane): Fernando's choice |
| Pod: optional foot nuts | a **5.5 AF x 2.5 hex recess in a Ø9.8 x 2.5 pad on the foot's top** round the y 23 and y 34.5 foot screws (corner along the print axis), for a nut when more grip is wanted (then an M3 x 30 from below). The pad keeps the foot's full 8 mm under the nut, so without a nut the M3 x 25 still has 6.2 mm of Ø2.8 thread (a recess cut into the foot left 5.5, under the 6 mm rule: the first try, fixed); pad walls 2.15 at the flats, 1.73 at the corners. None at y 89.5: the pivot block stands over it (a recess there would cut the block's back open); that screw already has 6.9 mm of thread and the block above it has room for a longer screw. Not in the BOM (nuts optional). |
| Pod: inner frame pivot | **reinforced round the bearing**: the pivot block is R 13.4 about the pin (was R 9), its back flush with the foot's end, and the gusset runs tangent to it from the column's back face, so column, gusset, block and foot are one solid (12 mm over the pin's pilot up to z 13.4, the gusset over the pin to z 29; was z 9 / 11.8). Frame 45.0 → 48.2 cm³ (with the nut pads); its outline seen from above is unchanged (the pod still slides out the same way) |
| Pod: keel pivot screw | the keel's **pin boss 13.5 mm long** (was 3.5), the pivot screw **M3 x 30** (was x 20) in a **Ø2.9** pilot (now a long thread), 18.8 mm threaded in the keel by the audit (21.1 from the keel's face; was 11.1), tip 1.6 inside the boss end. The fork bar's cut under the keel (top at z -10) now runs to x -16.5 before the 45 deg ramp (was -26.5): fork outer 68.6 → 67.2 cm³. **Not a cantilever (Fernando):** behind the pivot the keel is solid out to the boss's end (the side plate's profile run on along x, z -7..16), so the boss is one piece with the block that carries the two M3 x 35 mounting screws: the screw that holds the bearing is stiff against them. Keel 11.9 → 14.3 cm³, overhang unchanged (111 mm²); keel to fork 0.8 (the side plate's old minimum, unchanged), to anything turning 5.61 in every pose (was 8.19) |
| Snug M3 holes (new rule, `cad/solidworks.md` 18) | **Ø3.2 where an M3 sits snug in a printed part it does not thread into** (`dims.M3_SNUG`): the fork outer's axle stub (the retainer's screw is its steel core; was Ø3.4), the outer pivot sleeve on its M3 x 30 (was Ø3.4), the 6804 retainer's spigot and flange. The retainer's screw now threads only into its boss: **M3 x 30** (was x 25), boss 9.4 long (was 4.5), 7.9 mm of Ø2.8 thread (was 6.2), tip 1.5 inside the boss's end. The hole audit's M3 thread probe ends at r 1.58 (was 1.68), so a snug hole does not count as thread |
| Pod: pivot bearings clamped (Fernando) | The fork was held to its 627s one way only by the outer races' friction: both static parts sit on the +x side of their bearings, and both seats stopped only +x. Now each screw clamps its 627's inner race to the static part through **printed retainer caps** (no washers: a printed washer would be weak), and the fork is held both ways. **Outer:** the seat opens on the arm's **outer** face (bearing 1.5 in), a 45 deg lip on the keel side (Ø22 → Ø18) bears on the outer race. The **M3 x 30** bears on the **outer retainer cap** (`pod_pivot_cap`: Ø10 x 2 flange on the inner race, a Ø7 spigot 3 into its bore) and runs through the **sleeve** (`pod_sleeve`: a Ø7 spigot 3 into the bore from the other side, a **Ø10 collar** through the lip's hole onto the keel), both Ø3.2 snug, into the keel (Ø2.9, ~18 mm). Fork pulled away from the keel: lip → outer race → balls → inner race → cap → screw → keel. Head and cap stand 3.0 proud of the arm's outer face. The Nov 9 trough in the bar's top and the Ø14 head hole are gone. **Inner (Fernando, third try): the 627 moved into the frame.** Its outer race sits in a seat in the frame's fork-side face (the seat floor prints facing up and bears on the outer race past a Ø18 relief) and stays still; its inner race turns with the fork. The fork's inner half carries a printed **Ø7 pin** in the race and a **Ø10 boss** on its motor-side face (its own 627 seat is gone: solid there now); an **M3 x 20 from the frame's outer face**, through a Ø12 bore, bears on a printed **Ø10 x 2 cap** (`pod_pin_cap`) on the race's other face and threads ~18 mm into the pin (Ø2.9 pilot, through to the split face): it presses the fork onto the bearing. Fork pushed +x: boss → inner race → balls → outer race → seat floor. The M3 x 20 is a **flat head countersunk in the cap** (Ø10 x 2.4), reached through the frame's outer face (head 0.6 under it); cap and head turn with the fork in the Ø12 bore. **The frame is clamped too (Fernando):** a printed **clamp ring** (`pod_brg_ring`: an upper arc, r 9.5..12.5, 2.5 thick over the outer race, two 4 mm tabs) sits flush in a recess in the frame's fork-side face and its **2 x M3 x 8** pan heads (top and front-top, the only places with solid frame behind them) clamp the outer race onto the seat floor: frame + bearing clamped by the ring, fork + inner race by the pivot screw. Fit the ring at the bench before the fork's inner half goes on (the fork's Ø10 boss passes its Ø19 hole). The 627 sits 2.5 deeper in the frame for it. The 627 reaches z -11, **3 mm under the frame's foot**: the chassis clips round it along the pod's way out (as round the fork's lobe). The y 89.5 foot screw moved to **y 65** (it would have run into the seat), sharing the side fin's stud pilot. (Tries one and two kept the 627 in the fork and clamped its inner race to the frame: a screw head inside the fork, then a keyed cap and nut on the frame's pin; neither could be tightened.) Checks: `nov13.pivot_clamps (evening)` |
| Coupon | `coupon_627.py` made compact (below): 18.0 cm³, was 35 |
| STEP hardware (new rule, `cad/solidworks.md` 15) | **assembly STEPs show every bearing and fastener, magenta**, as real models: `hardware.step_model` turns a plain screw into its library part (hex socket) and a plain bearing envelope into `hardware.bearing_model` (rings + shields recessed 0.25); real models (the wheel's `reference/bolt_M3_*.step`) pass through. Done for `step_export/pod_lean/pod_lean_assembly.step` (116 solids: 26 bearings, 23 screws incl. the 6 mounting screws from below, 4 plungers, the washer). `parts/` stays printed parts only. Not yet: `wheel_motor_assembly.step`, `robot_assembly.step` (printed parts only, as before) |

**Exceptions for Fernando to approve** (left as they are):
1. The fins' foot pilots (12 x M3 x 30 up from under the chassis into the fins' legs) and the floor's screws down into the fins' flanges
   (15): through would bore a fin's whole height (31 mm to more than 80 mm of plastic). The fins print upside down, so the foot pilots open on
   top as printed and the flange pilots on the bed side: neither needs support.
2. The face crown's two M3 into the face frame's rails: through would run the rail's height; small round holes, no support (and, as before,
   they lie across the frame's layers).
3. The SO-101 base's four countersinks are the official part's (vendor geometry): not deepened.
4. The spring plungers' M4 nylocs sit on the tower tops and move with the plungers: not captive, kept.

The hole audit flags the servo bracket's two holes as "partial" (0.15 of a ring): that is the new hex recess round them, by design. Found while
checking, not changed: the inner tie bolt on line 2 runs 1.0 mm from the tenon's faces (the audit's "partial", 0.71); each axle's head path grazes
its neighbour roller (below).

## Pod_reimagined_1 / 2 / 3 (Nov 13-14, Fernando: blue-sky STEP proposals, not adopted)

Fernando's brief: redo how the suspension mounts on the fixed structure, with the bearing's outer race held both ways in its housing and an
M3 + PLA shaft clamping the inner race; springs on both sides; smooth, deliberate surfaces and joins; symmetric pivots; 2 mm from the fork to
the TPU; compact, or a compact / medium / relaxed set. Three proposals, each in its own folder with `build.py`, `README.md`, `checks.json`,
print STEPs (`parts/`), an assembly STEP (bearings and fasteners magenta) and renders. Side by side: `Pod_reimagined_compare.png`.
The chassis floor and the pod jig are not adapted (Fernando: they follow the chosen pod). Nominal CAD, hand statics, nothing printed.

**Shared** (`tools/pod_rx.py`):
- The arms are one outline each (an exact convex hull of circles, plus waists and pad ramps, every corner rounded), 12 thick.
- Every pivot is built the same way. The 627's outer race sits in a seat from the arm's **wheel face**, on the seat floor at the outer side,
  under a printed ring with 3 × M3 × 10 flat heads, flush. A static boss and a printed cap, with Ø7.2 spigots into the bore, are clamped onto
  the inner race by one M3 along the axis. Printed outer face down, no housing surface needs support.
- A pivot located from both sides is axially closed twice (the outer race in the arm, the inner race on the static part). So each boss stops
  0.6 short of its race, and a printed shim (`rx_shim_*`, Ø10 / Ø7.4) fills the gap. It is picked at assembly from 0.4 / 0.6 / 0.8 so the fork
  turns free with the screws tight (Nov 14).
- The outer arm hugs the wheel: wheel face at x -37.4, 2 mm off the turning parts, with a 45° dish to x -41.5 round the hub's cartridge. Its
  outer face is 3.1 mm further in than pod_lean's.
- The wheel, motor, 6804 stub and retainer, the pivot (80.6 mm, level with the axle), the travel (-3 / +15), the springs and the wheel rate
  (2.54 N/mm) are cad/v14's.
- The checks use the wheel's **swept** envelope (a revolved profile of everything that turns), not the rollers' pose.

| | 1 COMPACT | 2 MEDIUM | 3 RELAXED | pod_lean today |
| --- | --- | --- | --- | --- |
| Static structure | one block between the arms (both pivots, springs, stops) | block (pivots) + two C-frames over the arms | two outboard side frames + a tie | inner frame + outer keel |
| Fork | closed frame (bar + bridge) | open U (bar) | open U (bar) | open U (bar) |
| Springs | behind the pivot, up into the bridge, 2 + 2 | over each arm (pod.py), 2 + 2 | over each arm, 2 + 2 | 4 on the motor side |
| Width (x, swept) | -50.4 .. 55.7 (106) | -50.4 .. 55.7 (106) | -62.4 .. 67.7 (130) | -52.5 .. 78.6 (131) |
| Back (y) / top (z) | 117 (125 swept) / 38.9 | 120 / 56.9 + nuts | 114 / 56.9 + nuts | 103 / 56.9 + nuts |
| Pivot load at bump (each) | 126 N | 64 N | 64 N | ~2.7 × the wheel load on the inner pivot |
| Longest spring load path | short (block) | long (C-frame cantilever 48 mm) | shortest (tower on its plate) | |
| Printed parts (plus 2 shims each) | 8, 281 cm³ | 10, 308 cm³ | 10, 261 cm³ | 9, ~222 cm³ |
| Interference, three poses; hole audit | none; clean | none; clean | none; clean | none; one partial |

**Pod_reimagined_4** (Nov 14, Fernando: proposal 3 + inboard support):
- Proposal 3's side frames, thinned to 8 mm, plus a **beam between the arms**, so each 627's inner race is held on both sides. One
  M3 × 30 runs from the frame through the race into the beam, and the threaded caps are gone.
- The droop stop is the fork bar's back face on a lug under the beam. The droop screws and their arc slots are gone.
- `rx_deck` stands in for the floor and replaces the tie.
- 122 mm wide, 9 parts plus 2 shims, about 299 cm³ without the deck. No interference, clean audit (`Pod_reimagined_4/README.md`).
- **Its robot-level impact study is `cad/v15`:** each pod on a hollow-triangle chassis node with the bumper on its curved side. Robot
  Ø450 → Ø481.

| What | Where |
| --- | --- |
| **SolidWorks robot** (open this: the only assembly in `solidworks/`) | `solidworks/V14_robot.SLDASM` |
| Sub-assemblies (`solidworks/assemblies/`) | `V14_frame`, `V14_mounts`, `V14_bumper`, `V14_reference`, `V14_upper`, `V14_electronics`, `V14_arm`, `V14_fasteners`, `V14_harness`, `V14_pod` / `V14_pod_m` (`V14_pod_static`, `V14_unsprung` > `V14_wheel` > `V14_roller`, `V14_motor`) |
| Parts | `solidworks/parts/V14_*.SLDPRT` (imported STEP bodies, not native features; the RPLIDAR is the vendor SolidWorks part) |
| Part STEPs + manifest (the assembly's inputs) | `solidworks/step_parts/`, `solidworks/manifest.json` (`solidworks/_keys/`: the build's component names) |
| Print STEPs (printed parts only) | `step_export/` (`wheel_motor/`, `pod_lean/`, `chassis/`, `upper/`, `robot_assembly.step`) |
| Checks | `validation/` |
| Scripts | `tools/` (below) |
| Inputs | `reference/` only (below) |

## The wheel (baselined Nov 13, Fernando)

The cad/v13 prototype `wheel_quiet.py` round 5 (687 cartridges, round-3 hub) with `wheel_quiet_2.py`'s tread, now built by `wheel_motor.py`
into the pod and robot like the old wheel. Why: the sim found the clicks of the printed v13 wheel are roller play on the axles, radial and
axial (`simulation/v11 sil/studies/results/wheel_noise_2026-11-11.md`), and the thump of a leaned wheel is the row handover
(`wheel_wobble_2026-11-12.md`). The arm robot's sim already rolls this roller (`robot_export.ARM`, catalog `Printed-140-2x6-687-v13`).

- **Roller cartridge** (`roller_core`, `roller_sleeve`, `roller_tread`, two 687ZZ, 7 x 14 x 5): the PLA core has a 687 pocket in each end
  (**Ø14.05**, Nov 13: `coupon_687` cell 2, printed by Fernando) and a Ø10.6 bore between them; a stepped printed sleeve (**Ø7.15 through the
  bearings**, Nov 13: cell 4, stepping down on a 45 deg cone to Ø7.0 just outside each bearing for the click seats; Ø8.6 middle on the inner
  races, Ø3.2 bore on the M3). Both are friction fits; Fernando, Nov 13: they hold, no snap features.
  **Bearing stops (Nov 13 evening, Fernando: hard to get both bearings flush or a little inside):** the core prints upright, B end (+z) on
  top, so pocket B's floor faces up and prints true, while pocket A's floor is a 1.7 mm ring printed over air that sags. Bearing A used to
  stop on that floor (0.2 inside) and B was placed by A plus the sleeve's shoulder with 0.1 of margin, so every print error pushed a bearing
  proud. Now **B stops on pocket B's floor and A, pressed in last, on the sleeve's shoulder** (pocket A runs 0.5 past A, so A never meets
  its floor), **both 0.2 inside the core ends** (0.3 was tried: the thrust lands then stood inside the core's end plane and the core hit them in the radial drop-in; the lands are now 0.25 off the bearings, 0.05 outside the core end, so a roller walks at most 0.25); the shoulder is 28.6 long (was 28.7). The two bearings hold the roller between the shoulders
  from the inside: no axial play beyond the 687s' own.
- **Sleeve in the hub:** the sleeve sits on its seat floors (Nov 13 evening, Fernando: the M3 x 50 barely threaded; it was 0.1 too long at each
  end and the spokes had to spring 0.2 apart; now 41.8 long, was 42.3, with a 0.2 lead-in). Each thrust face has a 0.5 deep click seat for its
  end: a Ø7.1 round about the axle that closes to a 6.9 neck just past the sleeve's centre, then opens at 45 deg into a 7.3 channel out to the
  block's edge (the drop-in path), 0.1 shallower than the round. The cartridge presses into its slot (riding 0.1 high in the neck), clicks into
  the round, and the M3 x 50 clamps the sleeve: the screw only holds a stationary sleeve. **6.9 mm of M3 thread in the Ø2.8 pilot** (was 6.0):
  the core end to thrust face is 0.9 (was 1.05) and the wall under the head seat 1.3 (was 1.9; the head presses it straight onto the sleeve's
  end, so it only takes compression). The TPU tread is unchanged.
- **Thrust lands (Nov 13, Fernando: a thrust washer / journal to keep the roller centred):** on each thrust face a ring Ø7.3..8.6 round the click
  seat stands up to 0.25 mm from its 687's face (0.85 tall each side; evening: was 0.2, the face now 0.05 outside the core's end plane), so it
  meets only the static inner race; the shields (bore ~Ø9.2, VERIFY) stay 0.39 off it and the outer race 1.1 off the face. A bearing or the
  roller can then walk at most 0.25 out, and only onto a part that does not turn. The ring is cut open on the drop-in side (the 7.3 channel), so the cartridge still presses in
  radially. The ring is 0.65 wide (two 0.4 mm lines).
- **Through holes (Nov 13, a general rule):** each axle's Ø2.8 pilot runs on out of its block's round end (was a 2.95 floor), the three
  cartridge pilots on through the end wall's cone into the cavity, and the cartridge's countersinks are 0.5 deeper (a Ø5.5 x 0.5 bore over
  the cone; the M3 x 12 flat heads sit 0.5 lower).
- **Petals:** the axle blocks are R6 (v13: R8), seen along the axle a round with 45 deg tangent flanks down to the spoke's faces and R2.5 into
  them: smaller, round, symmetric, printed without support. The spoke necks below r 54, the drum, collar, flange, cartridge seat and roller
  cuts are v13's.
- **Tread tip relief:** the last 7 mm of each tread end leave the R70 circle on a tangent arc, 0.2 under it at the end (0.14 at the upright
  row handover), so the incoming roller takes up the load over a few millimetres. Bore, keys and the snap rib are v13's.
- **Assembly of a roller (Nov 13 evening: B first):** press bearing B into pocket B (the end that printed on top) down to its floor; push the
  sleeve in from the A end, through B to its shoulder; press bearing A over the sleeve's other end until it stops on the shoulder (both end
  0.2 inside the core); slide on the tread; press the cartridge into its slot until it clicks; fit the M3 x 50. Print the core B end up. 24 687s per wheel, 96 per robot.

Checks (`wheel_motor.py check`, `validation/wheel_motor/checks.json`; pictures `section_roller_cartridge.png`, `section_click_seat.png`,
`tread_profile.png`, `section_axis_plane_z0.png`, `section_row2_plane.png`):

| Check | Result |
| --- | --- |
| Printed parts | one valid solid each: hub 209.6 cm³ (evening; was 208.6) (v13 hub 244.3), cartridge 7.7, roller core 15.1, sleeve 1.9, tread 16.3 |
| Hub against everything | no overlap (evening: the sleeves sit on their seat floors, 0 mm³; were pressed 5.3 mm³ in, 0.1 per end); the 15 screws 0 |
| Turning parts to the hub | 687 outer race 1.1 mm, its shields (from r 4.6) 0.39, core 0.9, tread 0.9 (evening; were 1.15 / 0.36 / 1.05 / 0.95) (four rollers, both rows, both axle directions) |
| Thrust lands (Nov 13) | 0.25 from each 687's face (evening; was 0.2), 0.85 tall, on its inner race only; Ø7.3..8.6, 0.65 wide |
| Roller cartridge | core / sleeve / tread: 0 overlap; sleeve in the 687s 16.7 mm³: the Ø7.15 press fit on their 7.0 bores, by design; pockets Ø14.05 (0.025 clear in CAD; the coupon's fit); sleeve middle 1.0 from the core; evening: bearing B on its pocket floor, A 0.5 off pocket A's floor, both 0.2 inside the core ends, the shoulder 28.6, the sleeve 41.8 (was 42.3); sleeve wall 1.9 at the seats; core wall from pocket to key slot 4.17 |
| Click seat | Ø7.1 x 0.5, neck 6.9 (0.05 a side on the Ø7 sleeve), 0.1 step; **6.9 mm of M3 x 50 thread in the pilot, 1.3 under the head seat** (evening; was 6.0 / 1.9); the head counterbore 3.75 deep (the block's outer face and head path unchanged) |
| Radial drop-in | the core, tread and 687s clear the hub at 0 / 10 / 20 / 30 mm along the slot; only the sleeve's designed pinch at its seat |
| Tread relief | CAD sections equal the formula (z 0 / 12.5 / 15.3); 0.14 under the R70 circle at the upright handover (z 18.12), 0.2 by the end |
| Block walls | 2.5 round the Ø7 head counterbore (1.82 at its rim in the wheel plane), 4.6 round the pilot; the pilot open past the block (through); the cartridge pilots open into the cavity |
| Printing (cartridge end down) | 9,456 mm² under 45 deg (6,043 on the spokes; evening, was 9,370 / 5,950; Nov 13 +320 for the lands' undersides and the pilot exits): the flat x faces over the open roller pockets, the flange inside the access bore and the cartridge counterbore roof; none in the channel between the rows |
| Neighbours, motor, plug | treads 1.24 apart; nothing in the motor; XT30 + leads 4.68 mm from anything that turns (want 4); each axle's head path (Ø6.35 along the axle past its seat) grazes the neighbouring roller in its spoke: its tread 7.9 mm³ and its axle's head 1.2 mm³ (as in v13's record, 9.0 / 1.2; earlier versions of this README said "clear", which was wrong): drive the two axles of a spoke in turn, VERIFY with a driver |

## The robot in short (unchanged from cad/v13)

- **Layout F:** Ø450 robot on four lean pods (`pod_lean.py`: fork hinged level with the axle, two bed springs per side on plunger bolts,
  droop 3 / bump 15 mm), the AK40-10 bolted to the fork, its XT30(2+2) radial and outside the hub. Frame chassis (hub + four nodes on
  dovetail laps, lidar pedestal, five fins in the lidar's own shadow), bumper quarters, pods slide out from below (`chassis.py`).
- **Second floor** (`upper.py`): four quadrants, the arm riser + adapter with the SO-101 (official parts, stow pose from the robot's
  `omni_arm` kinematics), the D435 in a pocket in the riser's front (30 deg down, landscape, centre (190.8, 0, 193.0); `d435_riser.py`),
  the electronics (computer, battery tray, power stack with the three converter modules, distribution board and Teensy carrier), bin deck,
  face frame with the 5 in display, mic array and speaker, e-stop and switch; zip-tie pairs and a 30 mm grid of M3 pilots in the floor.
- **Harness** (`harness.py`, `harness_route.py`; a proposal, nothing wired): every lead routed on both floors, the leads that rise to the
  second floor only through the slots in the side fins' shadows (0 deg of lidar blind angle added). Connectors, lead ODs and port positions
  on our own boards are ASSUMED. The inventory, power budget and the converter picks are in cad/v13's README "Wiring harness", unchanged.
- Design rules this CAD follows: `cad/solidworks.md` sections 15-18 and `cad/AGENTS.md`.

## Filament colours (baselined Nov 13, Fernando)

`tools/palette.py` holds the four colours and the part → filament map; every robot colour function asks it first. Bought parts keep
realistic colours, fasteners magenta, bearings / springs / plugs / leads / field envelopes their reading colours.

| Filament (Bambu swatch hex) | Parts |
| --- | --- |
| **Jade White** (`#ffffff`) | second floor (`floor2_*`), arm riser and adapter (`riser_*`), D435 cradle, bin deck, face frame, crown and speaker cover, chassis (hub, nodes, pedestal, supports), the bin if printed |
| **Dark Gray** (`#545454`) | the whole SO-101's printed parts, pods (`pod_fork_*`, `pod_frame_inner`, `pod_keel_outer`, `pod_sleeve`), **roller cores and roller sleeves**, the brackets and posts on the floor (computer clips, battery tray, power box and shelf, posts, servo-adapter bracket, IMU bracket), e-stop and switch housings |
| **Orange** (`#ff6a13`) | bumper quarters, wheel hubs, wheel cartridges |
| black TPU | roller treads |

The arm stays neutral on purpose: `omni_arm/color_vision.py` finds objects by saturated hue. Open: the real shades on a print, and whether the
white chassis shows dirt through the bumper gap.

## Checks

All re-run on v14 on Nov 13 (omni-robot-sim venv, from `tools/`); outputs in `validation/`. Compared with cad/v13's records, only the wheel
changed the geometry, and every check outside the wheel came out as v13's or within route / tessellation noise, except the arm sweep (below).
**After the Nov 13 review** (section above) `wheel_motor`, `pod_lean`, `coupon_627`, `chassis`, `upper`, `pod_service`, `screw_access` and
`robot_step` were run again; `harness_route` / `harness` and `arm_sweep` were not: the arm, the leads' ends and the routes did not move (the
new lead tie lies within the first 20 mm of each motor lead, which the harness check leaves out as flexing with its fork).

| Check | Result |
| --- | --- |
| `pod_lean.py check` (droop / nominal / bump) | no interference in any pose; every gap as v13 (inner frame to anything turning 7.57-8.24, keel 8.19); the inner fork half to the nearest tread 7.48; fork to the XT30 and leads 2.72 (Nov 13: was 3.78 at the old lead notch; both move with the fork). Six printed parts (Nov 13: + the retainer), one valid solid each. Nov 13 (`nov13` in checks.json): pin in the 627 0 overlap, M3 x 16 tip 1.0 inside the pin; retainer 0 overlap with the 6804, nearest turning part 4.55, 6.2 mm of thread; lead tie ≥ 15.75 from anything that turns, 0 overlap with fork and leads; both arrows engraved; **no blind holes, no nuts** in the pod; hole audit: 13 screws, threads 6.2-35.5, the four pilots and the keel's two Ø2.9 across the layers (allowed), one "partial": tie line 2's inner bolt 1.0 from the tenon's faces (v13 geometry). **Evening** (`nov13.evening`): no interference in any pose; keel to anything turning **5.61** (the longer pin boss, joined to the block; was 8.19), every other gap unchanged; frame 48.2 cm³ (overhang 1,095 mm², was 1,013), keel 14.3, fork outer 67.2; no blind holes; foot screws 6.9 mm each of Ø2.8 thread (the two under the nut pads too), now "across layers"; keel pivot screw 18.8 mm |
| `chassis.py check` | no part overlaps, nothing under 4 mm, all fit the 256 bed, every fastener lands (pod 16/16, keel 8/8, pedestal 4/4, fins 14/14, bumper 16/16, laps 6), lidar blind 20.9 deg. Nov 13: 28 blind pilots drilled through, 12 left (the fins' foot pilots: exception 1); hole audit clean (the side fin's M3 studs in Ø2.9 pilots across the inner frame's layers: allowed). Clearance to moving parts: bumper 3.0, frame 3.1 at z 56-62 and **2.99 at z 40-56** (v13 3.00: the frame is cut at exactly 3 mm round the pods' swept sections, and the new hub and tread outlines tessellate 0.01 mm differently; read it as the 3 mm cut, not a loss) |
| `upper.py check` | as v13 except the Nov 13 holes: no overlaps (also with the chassis), battery slides out, no screw short of 6 mm, hole audit as v13 (the face crown's two screws and the mic board's vendor holes) plus the servo bracket's two hex recesses read as "partial" (by design); 6 blind pilots drilled through, the face crown's two left (exception 2); D435: 13.3 mm from the arm (SO-101 base), cradle 11.0, plug and lead 17.8; 15.1 mm over the scan plane (0 deg added); inside the bumper; nothing of the robot in either field; nearest floor 0.31 m |
| `harness_route.py`, `harness.py check` | the same routes as v13 (lengths within 2 mm); no lead centreline in any part, no plug overlaps, nothing in the D435's fields; nearest static 0.47 (M_RR in its slot, tied), then 1.38 (MIC to the deck); pods' moving parts **≥ 5.23 mm** (M_RR to the RR treads at bump); arm sweep ≥ 16.5; **0 deg** of lidar blind angle added; drawn bends still under the allowed radius in places (as v13: open) |
| `arm_sweep.py` (official meshes, 33 planned moves) | **≥ 7.5 mm**: the gripper to the front bumper quarter at the 0.26 m floor grasp. v13's record (12.2, Nov 5) predates v13's Nov 11 bumper change (full height across the nose, `NOSE_DIP = False`) and was not re-run then; it is a v13 change, now measured. Over the 3 mm rule |
| `pod_service.py` | (Nov 13, after the review) each pod comes out alone: its 6 chassis screws reachable from below, then it slides 250 mm out along its way with its bumper quarter off and no contact; the front pods carry the side fin's two M3 studs out in their column (the fin's slots open that way); the XT30 sits 96.4 in from the bumper (`validation/pod_service_nov13.log`) |
| `screw_access.py` | (Nov 13, after the review) 164 screws, 66 with a driver blocked in the finished robot (v13: 162 / 62): the same screws as v13 (only renumbered; the riser buck's two are gone), plus the four new M3 studs on the front pods' columns, which the side fin and the floor cover: they go in on the bench before the fin and come out with the pod. Blocked means "drive it before that part goes on": the assembly order, not a defect (`validation/upper/screw_access.json`) |
| `robot_step.py` | `step_export/robot_assembly.step`: 229 printed parts (v13 177 + the 48 roller sleeves + Nov 13 the 4 pods' 6804 retainers) |

## The SolidWorks assembly

**Nov 13 review:** `sw_robot_parts.py` was re-run (246 part STEPs, 34 changed: the pod's fork halves, inner frame and new
retainer and lead tie, the hub, the chassis nodes and fins, the floor quadrants, the riser and cradle), but the SolidWorks stages (`import`, the
pod / group stages, `robot`, `interference`) were **not**: the `.SLDPRT` / `.SLDASM` files still show the geometry from before the review,
and the pod's static sub-assembly still lists the removed dowel until `sw_robot.py static pod` is run again.

SW_SECTION

## Tools

| Script | Makes |
| --- | --- |
| `dims.py` | dimensions of the wheel, motor, pod and harness |
| `wheel_motor.py` | the hub, cartridge, 687 roller cartridges and treads, the AK40-10 model with its plug and leads; the wheel's checks |
| `pod.py`, `pod_lean.py` | the pod geometry the lean pod builds on; the suspension pod (inner frame, keel, fork halves, sleeve, springs / plungers) and its poses |
| `base.py`, `first_floor.py`, `chassis.py` | pod placement and footprints; the first-floor chassis (hub, nodes, fins, bumper, lidar mount), clipped to each pod's slide-out path |
| `devices.py`, `arm_pose.py`, `d435_riser.py`, `upper.py` | electronics envelopes with real-size plugs; the SO-101's parts posed by `omni_arm`; the D435 mount and its check; the second floor, arm mount, electronics, bin deck, face |
| `harness.py`, `harness_route.py` | the harness inventory, power budget, drawn leads and plugs, tie and grid placement; the lead routes |
| `hardware.py`, `print_beds.py`, `step_colour.py`, `palette.py` | fastener records (magenta) and the hole audit; print orientation per printed part; coloured STEP writing; filament colours |
| `pod_service.py`, `screw_access.py`, `arm_sweep.py` | can one pod come out; can a driver reach every screw (assembly order); the arm's planned moves against the robot |
| `robot_step.py` | one STEP of the whole robot's printed parts (`step_export/robot_assembly.step`) |
| `coupon_687.py`, `coupon_627.py` | bench prints, not robot: the 687 (roller) and 627 (fork pivot) fit coupons (below) |
| `pod_rx.py`, `pod_rx_summary.py`, `pod_rx_compare.py` | what the `Pod_reimagined_*` proposals share (arms, pivot housing, swept wheel envelope, checks, outputs); their side-by-side numbers; the comparison picture |
| `sw_robot_parts.py`, `sw_robot.py` (+ `subasm.py`, `mate_api.py`, `sw_native.py`) | part STEPs + manifest; the SolidWorks assemblies, motion test, interference, paths, views |

Not carried over from cad/v13 (all still there): `coupon.py`, `motor_jig.py`, `pod_jig.py` (bench prints and tooling), `fe_*.py`,
`hub_sector.py`, `pod_fe_prep.py` (stress models), `imx662_fit.py`, `layout_study.py`, `screen_study.py`, `cable_route.py` (studies; v14 keeps
only the device inventory as `devices.py` and the arm pose helpers as `arm_pose.py`), `wheel_quiet.py` / `wheel_quiet_2.py` (now
`wheel_motor.py`), the D435 placement scan (`d435_riser.py scan | candidates`) and the D435 mount's review STEP.

## 687 fit coupon (Nov 13, Fernando)

`tools/coupon_687.py` (dims `C687_*`): one **40 x 40 x 12 mm** block (18.8 over the pins, 15 cm³), four numbered cells, each with a core pocket
opening on the bed side and a sleeve stub on top. Both get tighter from cell to cell (0.05 mm each step); cell 1 is the roller as drawn:

| Cell | Pocket Ø (bearing OD 14) | Sleeve pin Ø (bearing bore 7) |
| --- | --- | --- |
| 1 (as drawn) | 14.10 | 7.00 |
| 2 | 14.05 | 7.05 |
| 3 | 14.00 | 7.10 |
| 4 | 13.95 | 7.15 |

The pocket is like the core's: 4.0 mm of PLA round it (the core has 4.15 from pocket to key slot), no lead-in, opening on the bed as the
core's lower pocket does when it prints upright. It is 5.6 deep under a 45 deg cone roof, so no support is needed. Set a 687 on a flat
bench and press the block down over it: it stops flush with the face. The stub is the sleeve's end: a Ø8.6 x 0.8 shoulder on the inner
race, the pin with the sleeve's 0.2 lead-in, and the Ø3.2 bore (the same 1.9 mm wall). Two Ø1.8 holes per pocket reach the bearing's outer
ring from the top, and they sit wholly inside the pocket, so the wall is never nicked: push the bearing out with a 1.5 mm hex key. Print it
in the roller core's filament and settings, flat on its face. Choose the tightest pocket that takes the bearing without cracking and the
tightest pin that presses on and holds, then put those sizes into `dims.BRG_POCKET_FIT` and the sleeve's Ø7 ends. Print STEP:
`step_export/coupon_687/coupon_687.step`; checks and pictures are in `validation/coupon_687/`. It is nominal CAD and has not been printed.
The click seat's neck is still tested only by cad/v13's `fit_coupon_687`.

**Result (Nov 13, Fernando printed it):** the best pocket is **cell 2 (Ø14.05)**, the best pin **cell 4 (Ø7.15)**. Both are now in the roller
(`dims.BRG_POCKET_FIT` 0.025, `dims.SLEEVE_BRG_D` 7.15). Cell 4 was the tightest pin offered, so a still tighter pin was not tried.

## 627 fit coupon (Nov 13, Fernando: the fork's 627 seats are too loose)

`tools/coupon_627.py` (dims `C627_*`): the 687 coupon's layout for the 627 (7 x 22 x 7) in the fork's pivot seats, drawn Ø22.00 and printed too
loose. **Compact (Nov 13 evening, Fernando):** a **51.4 x 51.4 mm** block, **18.0 cm³** (the first version this afternoon was a solid
52.9 x 52.9 x 18.2 block, 35 cm³; not printed), 26.6 mm over the pins. A **7.6 mm base plate** holds the four numbered pockets, open on the bed
side (as the fork's seats print, axis vertical), with **2.5 mm of PLA** between pockets and to the faces (what the fork's lobes leave round their
627s). Over each pocket stands only a **cone shell**: the 45 deg roof inside and a 45 deg cone outside, about 3 mm thick, closing on the Ø10
shoulder 0.6 over the roof's apex (no support anywhere). On top, the pin for the inner race: a Ø10 x 0.8 shoulder, the pin 8 long, with a Ø2.8
bore through the whole block, as the inner pivot's printed pin with its M3 core. Two Ø1.6 push-out holes per pocket come down through the cone
onto the outer ring, inside the tightest pocket. The cell numbers are engraved in the side faces next to each cell (1 2 on the +y face, 3 4 on
the -y face). Pictures: `validation/coupon_627/coupon_627_section.png`, `coupon_627_iso.png`.

| Cell | Pocket Ø (bearing OD 22) | Pin Ø (bearing bore 7) |
| --- | --- | --- |
| 1 | 21.95 | 7.05 |
| 2 | 21.90 | 7.10 |
| 3 | 21.85 | 7.15 |
| 4 | 21.80 | 7.20 |

Print it in the fork's filament and settings, flat on its face. Press a 627 into each pocket from a flat bench and onto each pin. Choose the
tightest pocket that takes the bearing without cracking and the pin that needs a firm press and holds, then put them into the fork's two 627
seats (`pod_lean`: today `B627_OD` 22.0 exactly), the inner pivot pin (`pod_lean.PIN_D`, 7.0) and the outer pivot's sleeve (`SLEEVE_D`, 6.96).
Print STEP: `step_export/coupon_627/coupon_627.step`; checks and the section in `validation/coupon_627/`.

**Result (Nov 13 evening, Fernando printed it):** the best pocket is **cell 1 (Ø21.95)**, the best pin **cell 4 (Ø7.20)**. Both are in the pod now
(`dims.B627_SEAT_D`, `dims.B627_PIN_D`): the outer arm's and the frame's 627 seats, the fork's pin in the frame's 627, and the outer retainer cap's
and sleeve's spigots. Cell 4 was the largest pin offered, so a still larger pin was not tried. The pod check lists these designed press fits apart
(`press_fits_627_mm3`).

## Files this version reads

Everything is inside `cad/v14` (`cad/solidworks.md`, top): `reference/` is cad/v13's, unchanged (where each file came from: cad/v13 README
"Files this version reads"). **One import that is not CAD:** the arm's poses come from the robot's own kinematics, `ros/src/omni_arm`
(`arm_pose.py` puts it on the path). Check after any change: `grep -rnE "v1[0-35-9]/|v1[0-9] human|components/|Roomba|C:/" tools/*.py` shows
only comments and interpreter paths, and `sw_robot.py paths` fails if any component file is outside this folder.

Caches copied from cad/v13 (arm only, independent of the wheel): `validation/d435/arm_motion.json`, `arm_cloud.npz`,
`validation/harness/arm_occ_p3.npz`. Delete them to recompute (`d435_riser.py motion | cloud`, `harness_route.py`).

## Regenerate

From `tools/`, omni-robot-sim venv (`C:/Users/ferna/.venvs/omni-robot-sim/Scripts/python.exe`):

```
python -B wheel_motor.py all        python -B pod_lean.py all       python -B chassis.py all       python -B upper.py all
python -B harness_route.py          python -B harness.py all        (harness.py ties | grid re-place the floor's tie pairs / M3 grid:
                                                                     paste into upper.TIES_FLOOR / upper.GRID_PILOTS)
python -B pod_service.py            python -B arm_sweep.py          python -B screw_access.py (~25 min)     python -B robot_step.py
python -B coupon_687.py all          python -B coupon_627.py all       (bench prints)
```

The SolidWorks assembly (`C:/Users/ferna/cadenv/Scripts/python.exe`; one stage group per process, SolidWorks leaks memory; it closes only
files under `solidworks/`; `solidworks/run_stages.sh <stage> ...` runs them one process each and stops at the first failure):

```
python -B sw_robot_parts.py                            (omni-robot-sim venv: part STEPs + solidworks/manifest.json; it keeps unchanged STEPs)
sw_robot.py import                                     (STEP -> V14_*.SLDPRT, only parts whose STEP is newer; import in batches if memory runs short)
sw_robot.py roller wheel motor unsprung static
sw_robot.py pod
sw_robot.py group_frame | group_mounts | group_bumper | group_reference | group_upper | group_electronics | group_arm | group_fasteners | group_harness
sw_robot.py robot
sw_robot.py test | paths | interference | views | views_close
```

## Open / VERIFY

- **Wheel, Nov 13 evening:** whether the shorter sleeve (on its seat floors) still clicks in firmly and the M3 x 50 now bites (6.9 mm of
  thread); the 1.3 mm wall under the head seat under the screw's clamp; both bearings 0.2 inside on a printed core printed B end up.
- **Wheel (rollers first printed Nov 13; hub not yet with this geometry):** the 687 inner-race land (taken as ≥ Ø8.6) and its shields' bore (taken as Ø9.2: the thrust lands are Ø8.6); the
  pocket (Ø14.05) and sleeve (Ø7.15) fits are `coupon_687`'s (printed Nov 13), not yet a printed roller; whether the 0.65 wide lands print
  cleanly; the click seat's neck (6.9): cad/v13's `fit_coupon_687`; the sleeve bore on the M3 (Ø3.2: drill if it binds); the
  screw torque that clamps the sleeve without crushing the wall under the head seat; whether a 0.4 mm nozzle holds the 0.14-0.2 mm tread
  relief; the roller-to-roller radius spread of a printed set; the wheel's camber on the pod. The hub's stress model (cad/v13, Nov 11) was of
  the R8 spokes; the R6 blocks keep the necks below r 54 (the load path it found) but were not re-modelled.
- **Pod (Nov 13 changes, none printed):** the 627 seat and pin fits (print `coupon_627`); the tenon at 0.05 a face (was 0.1, too loose); the
  inner pivot's printed Ø7 pin with its M3 x 16 core under the pivot load; the four Ø2.9 pilots that replaced the pod's nuts (as the keel's);
  the 6804 retainer (6804 inner ring taken as ~Ø23.6 at its face: the flange is Ø23) and the clamp on the inner race; the lead tie (drawn
  round the leads where they are drawn: tightened it pulls them onto the plate's rim, not modelled; ties assumed 3.6 x 1.2). Evening: the Ø2.8
  foot pilots lie in the frame's layer plane (the wedging risk round 13 moved away from; watch for split layers at the foot), the optional nut
  recesses, the keel's M3 x 30 in its 13.5 mm boss; the reinforced frame has no stress model.
- **Carried from cad/v13:** the print-fit "first tries" (dovetails 0.1, 627 / 6804 seats, pilots, can pocket 0.1; cad/v13 `coupon.py`), the
  outer fork's Ø3.0 tie-bolt pilots, the D435's two rear M3 and its USB-C plug / lead envelope, the 0.9 / 1.2 mm static gaps round the camera,
  the adapter's span over the camera pocket (no strength check), the harness's connectors, lead ODs and port positions, its tight drawn bends
  round the computer, the motor leads' 100 mm service loops (not drawn), the AK40-10's CAN termination, the 19 V converter's dropout, and
  the FL / RR pods being mirror images (not a real opposite-hand motor plug).
- Not modelled: cables' flex beyond the three pod poses, ties, strength of anything but the cad/v13 hub and bumper models, printing.

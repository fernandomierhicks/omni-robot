# Printable free-turning AK40 mounting dummy (two printed parts)

This is a hand-operated fit/motion fixture. It has no windings, magnets,
gearbox or electrical drive, and does not reproduce motor torque, backlash or
mass. Only the mounting interfaces follow the v9 AK40: Ø53 body, 40.2 mm
contact-face length, three output holes on PCD27 and four rear holes on PCD47.
Everything inside is free to differ, so it is as simple as it can be: a
**stator** and a **rotor** held together by **one bearing** and **one screw**.
Native parts are in `parts/human/`; check the main README for build status.
The earlier four-part version (case, rear cover, rotor, retaining washer, two
bearings, five screws) is kept in `superseded/dummy_motor_4part/`.

## Parts and hardware

| Item | Qty | Notes |
|---|---:|---|
| `dummy_motor_stator` | 1 | Printed PETG / PLA. Ø53 body, front mounting holes on PCD47.5, rear holes on PCD47; hollow, open to the rear; plug flat. |
| `dummy_motor_rotor` | 1 | Printed PETG / PLA. Ø37 output flange with the three PCD27 holes and the Ø15 recess, short Ø6.9 shaft. |
| 687ZZ bearing, 7 × 14 × 5 mm | 1 | From Fernando's listed stock. Holds the rotor to the stator. |
| M3 × 8 pan-head screw | 1 | Threads into the rotor's Ø2.8 pilot. 7.2 mm nominal engagement. |
| M3 large washer, Ø9 × 0.8 (DIN 9021) | 1 | **Not the ordinary M3 washer.** The ordinary one is Ø7, the same as the bearing bore, so it could not retain the rotor. |
| M2.5 × 10 screws | 3 + 4 | Three attach the wheel flange to the rotor (6.5 mm engagement), four attach the cradle to the stator's rear (6 mm engagement). They enter printed pilots; confirm that they form threads without splitting. The real motor uses the shorter M2.5 × 6 output and × 8 rear screws. |

## Matches the AK40 drawing

Overall length 40.2 mm from the rear face to the rotor's Ø37 output face, which
protrudes 1 mm ahead of the Ø53 body. Mounting holes: 3 × M2.5 on Ø27 in the
rotor face, 3 × M2.5 on Ø47.5 in the stator's front face, 4 × M2.5 on Ø47 in
the rear face. The Ø15 recess (1.1 mm deep) is in the output face, and a flat
7.8 mm long and 18.7 mm wide, 27.0 to 34.8 mm behind the output face, marks
where the power plug cable leaves, 45° from a rear hole. The flat is for
reference only; there is no connector. The drawing's hex screw heads in both
end faces are not modelled. The printed pilots are deeper than the real
M2.5 × 3 / × 5 threads.

## How it fits together

- The stator's front has a Ø37.5 recess for the rotor flange (0.25 mm radial
  clearance), then a Ø14 pocket 5 mm deep for the bearing, with a 2 mm lip
  behind it that bears on the outer race. A recess around the pocket (Ø21 to
  Ø33, 3 mm deep) lets the rotor's three screw bosses sweep a full revolution,
  with 0.2 mm axial and 0.25 mm radial nominal clearance.
- Behind the lip the stator is hollow (Ø40 to the rear face). That gives
  access to the screw head with a hex driver. Only a 6.5 mm wall ring remains
  around the rear holes, which are blind M2.5 pilots 8 mm deep on PCD47.
- The rotor's shaft passes through the bearing from the front. Its shoulder
  touches only the inner race. The shaft ends 0.3 mm beyond the bearing's rear
  face, so the washer clamps the printed shaft end rather than squeezing the
  bearing. That leaves 0.3 mm of axial end float. The shaft is Ø6.9 in the
  bearing's Ø7 bore (0.1 mm diametral clearance).
- Retention: pushing the rotor into the stator is stopped by the stator's lip.
  Pulling it out is stopped by the washer on the inner race and the
  bearing's press fit in the pocket. If the bearing walks forward, add a drop
  of CA glue to its outside. The load path is not designed for wheel pull-out.

All fits are nominal CAD values, not measured printer results.

## Print and assemble

Print the stator with its front (output-side) face on the bed and the open
rear upward. Print the rotor with its output face on the bed, shaft upward.
The recess around the bearing pocket bridges over 6 mm; the bridge is the
only unsupported area. Do not scale a whole part to adjust a bearing fit:
edit the seat or journal dimension in SolidWorks.

1. Clean the bearing pocket and test the shaft in a loose bearing. Remove
   elephant foot; the shaft must turn without forcing it.
2. Press the 687 into the stator's front pocket, pressing only on the outer
   race, until it is flush with the front face.
3. Slide the rotor's shaft through the bearing from the front.
4. From the rear, through the stator's hollow, fit the large washer and the
   M3 × 8 screw into the shaft end. Tighten until the washer seats on the
   shaft end. Verify free spin and a little end float; do not preload the
   bearing to remove that float.
5. Fit the dummy to the cradle with the four rear M2.5 × 10 screws, then
   attach hub A to the output flange with three M2.5 × 10 screws. Install the
   wheel's removable outer bearing cartridge after the flange screws, as in
   the v9 wheel kit.

The wheel's own two 6804 bearings carry its support loads. A single 687 holds
the rotor, so this dummy is for turning the assembled wheel by hand and
checking fits, not for carrying it. A successful CAD collision check does not
establish printed bearing fit or load capacity.

## Checks done in CAD (Nov 1)

- Both parts: one valid solid, every sketch fully constrained, every dimension
  linked to a named variable, no dead equations. Volumes match the hand
  calculation (stator 43 755.1 mm³, rotor 4 942.8 mm³).
- The mounting holes land on exactly the same coordinates as the four-part
  version (PCD27 ×3 on the rotor, PCD47 ×4 on the stator), and both follow the
  master layout (`rear_radius`, `rear_phase`, `output_radius`, `output_phase`).
- Assembly `V10H_dummy_motor.SLDASM`: spin and end float (0 to 0.3 mm) work, and
  SolidWorks interference detection finds nothing at 0, 0.15 and 0.3 mm float
  and two spin angles (`validation/human/dummy_interference.json`).

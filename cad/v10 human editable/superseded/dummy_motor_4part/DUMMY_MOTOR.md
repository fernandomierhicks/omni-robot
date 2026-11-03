# Printable free-turning AK40 mounting dummy

This is a hand-operated fit/motion fixture. It has no windings, magnets,
gearbox or electrical drive, and does not reproduce motor torque, backlash or
mass. Its output and rear mounting interfaces follow the v9 AK40 envelope:
Ø53 body, 40.2 mm contact-face length, output holes on PCD27 and rear holes on
PCD47. Native parts are in `parts/human/`; check the main README for build status.

## Parts and hardware

- One case, one rear cover, one rotor and one retaining washer, printed in
  PLA or PETG. Printed shaft nominal Ø6.9; the bearings have a Ø7 bore.
- Two **687ZZ, 7 × 14 × 5 mm**, from Fernando's listed stock.
- Four M3 × 12 flat-head screws join the rear cover to the case. Heads are
  countersunk flush so they clear the motor cradle's rear bridge.
- One M3 × 8 pan-head screw joins the retaining washer to the rotor's blind
  Ø2.8 pilot; nominal thread engagement 6.8 mm after the washer.
- Three **M2.5 × 10** screws attach the wheel flange to the dummy's output,
  with 6.5 mm nominal printed engagement. Four **M2.5 × 10** screws attach
  the cradle to the dummy's rear, with 6 mm engagement. These enter printed
  pilots; confirm that they form threads without splitting. The real motor
  uses the original shorter M2.5 × 6 output and ×8 rear screws; the longer
  screws are specific to this printed dummy.

The front bearing has a nominal 2.31 mm axial backing shoulder. The rear
bearing has a 1.5 mm shoulder. The rotor has nominal 0.3 mm axial end float;
the retaining washer clamps to the shaft end rather than squeezing the two
bearings together. The journals clear the bearing bores by 0.10 mm on
diameter. All fits are nominal CAD values, not measured printer results.
The output has three deeper screw bosses. A complete annular recess in the
case clears them through a full revolution, with 0.25 mm radial and 0.2 mm
axial nominal clearance. The front bearing retains a 3.5 mm radial wall.
The four cover screws have reinforced Ø6 × 11 mm case bosses: 1.6 mm of
material around the Ø2.8 pilots and a 2 mm nominal blind-hole floor.

## Print and assemble

Print the rotor with its broad output face on the bed, shaft upward. Print
the case with its output face on the bed, open rear upward. Print the rear
cover with its exterior flat face on the bed, bearing carrier upward. Print
the washer flat. Inspect the bearing-seat shoulders and add local support
where your slicer requires it. Do not scale a whole part to adjust a bearing
fit: edit the seat/journal dimension in SolidWorks.

1. Clean the bearing seats and test the shaft in a loose bearing. Remove
   elephant foot; the shaft must turn without forcing it.
2. Press one 687 into the case's front seat and one into the rear cover's
   carrier, pressing only on the outer race.
3. Slide the rotor through the front bearing. Offer the rear cover and its
   bearing onto the shaft, leaving the cover loose.
4. Through the rear cover's central opening, install the retaining washer
   with the M3 × 8 screw. It seats on the printed shaft end. Verify free spin
   and a little end float; do not preload the bearings to remove that float.
5. Install the four flush M3 flat-head cover screws. Check spin again.
6. Fit the dummy to the cradle, then attach hub A to the output flange.
   Install the wheel's removable outer bearing cartridge after the flange
   screws, as in the v9 wheel kit.

The wheel's own two 6804 bearings carry its support loads. This plastic
dummy shaft is for turning the assembled wheel by hand. A successful CAD
collision check does not establish printed bearing fit or load capacity.


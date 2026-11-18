# Battery: HRB 6S 3300 mAh LiPo (baseline since Nov 16, 2026)

Fernando baselined this pack on Nov 16, 2026. It replaces the OVONIC Roam 6S1P 5200 mAh.

## Pack data

| Item | Value | Status |
| --- | --- | --- |
| Make / model | HRB, 6S1P 3300 mAh LiPo | Fernando, Nov 16 |
| Mass | 558 g | Fernando, Nov 16 |
| Size | 43 × 46 × 136 mm | Fernando, Nov 16 |
| Cells | 6S: 25.2 V full, 22.2 V nominal; the robot stops at 21 V (3.5 V/cell, provisional) | standard LiPo values |
| Energy | 73.3 Wh nominal (6 × 3.7 V × 3.3 Ah) | calculated from the label rating, not measured |
| C rating, internal resistance | unknown | **VERIFY** from the label; bench-measure sag |
| Discharge connector, lead gauge and length | unknown; the CAD assumes an XT90 and 10 AWG | **VERIFY** |
| Balance lead | JST XH 7-pin assumed (standard for 6S) | **VERIFY** |

## Against the robot's loads (estimates)

The loads are from the power budget in `cad/v15/validation/harness/power.json` (`harness.py power`, est).

| Load | Pack current | Pack C-rate |
| --- | --- | --- |
| Typical, 123 W | 5.5 A at 22.2 V | 1.7 C |
| Traction-limited burst with arm and computer at peak, 419 W | 20.0 A at 21 V | 6.1 C |
| Ceiling: every motor at its limit at once, 884 W | 42.1 A at 21 V | 12.8 C |

- **Runtime:** about 29 min typical on 80 % of 73.3 Wh (est). The OVONIC gave about 45 min on 115 Wh.
- **Sag:** with a third less capacity, a burst pulls the pack lower, closer to the 21 V stop and to the 19 V buck's 22 V input floor (design.md 4.7). Measure the sag at 20 A before trusting the stop threshold.
- **Shipping:** at 73 Wh the pack is under the 100 Wh airline limit; the 115 Wh OVONIC was over it.

## Compared with the previous pack

| | HRB 6S 3300 (baseline) | OVONIC Roam 6S 5200 (previous) |
| --- | --- | --- |
| Size | 136 × 46 × 43 mm | 157 × 47 × 46 mm |
| Mass | 558 g | 706 g |
| Energy | 73.3 Wh | 115.4 Wh |

## Where it is modelled

- **`cad/v15` (the working CAD, not formally adopted):** the pack hangs under the deck's centre (`tools/chassis_mid.py` `BATTERY`, `tools/devices.py`). Its 43 mm side is vertical, so it occupies z 37–80. Its centre stays at x 11.5, so the two straps keep their slots in the deck. The XT90 pair stays where it was drawn behind the pack.
- **`cad/v14` (last adopted robot CAD):** still draws the OVONIC envelope in the second floor's tray. Not changed.
- **v11 sim:** still `battery="6S1P-OVONIC-5200"` (`params.py`, `catalog.py`). The HRB is not yet in the catalog. Switching the sim's pack changes `ARM`'s mass and needs regenerated exports.

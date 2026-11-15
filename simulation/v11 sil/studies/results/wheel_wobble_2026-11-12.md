# Wheel wobble with the rollers locked (Nov 12 2026)

Fernando rolled the printed v13 wheel by hand on a hollow hardwood floor with every roller locked. It still made the floor vibrate, and it seemed to rock from three rollers down to one or two. This is the low-frequency thump, not the roller clicks of [wheel_noise_2026-11-11.md](wheel_noise_2026-11-11.md).

Everything here is simulation only: `studies/wheel_wobble.py` (`geom`, `dyn`, `roll`, `tune`, `refine`, `mj`; JSON in `results/wheel_wobble/`, logs `wheel_wobble_*.log`). The proposal CAD is `cad/v13/tools/wheel_quiet_2.py`. Nothing is adopted.

## 1. What makes it

**Geometry.** The model is exact for the drawn rollers: each roller is a solid of revolution with cad/v13's profile, an R70 arc about the wheel axis with an R2 end fillet.

- **Upright, the wheel is round.** The hub height varies by 0.5 µm as it turns.
- **Leaned by β, it is not.** The two rows sit at x ±17.25 mm, so a lean lifts one row's axles and drops the other's by 17.25 sin β. Under a lean the bottom roller turns about its own axis, so the roller's shape does not matter.
- **The step.** Every 30° the hub climbs onto, or drops off, the other row:

  | Lean | Step |
  | --- | --- |
  | 0.1° | 0.06 mm |
  | 0.25° | 0.15 mm |
  | 0.5° | 0.30 mm |
  | 1° | 0.60 mm |

  This is 34.5 sin β for every variant with the same row spacing. Roller overlap, end fillet, relief and tread hardness change only how fast the climb happens, not its height.
- **Held only at the hub** (a hand), nothing stops the lean. The wheel falls onto one roller of one row plus the ends of the two rollers of the other row: the tripod Fernando felt. Then it rocks across at every handover: **±3.9° of lean and 1.3 mm of hub bob, 12 times a turn** (8 Hz at 0.3 m/s). A hollow floor responds to that.

**Dynamics** (rollers locked, Hertzian TPU contact as in wheel_noise.py, a quarter of the robot on the pod spring, the lean held by the fork's roll stiffness, est 300 N·m/rad; floor-force ripple 5–500 Hz in dB):

- **Hand case** (a soft press at the hub, 0.5 N·m/rad of roll hold): 4.0 N rms of floor-force ripple, 8.5° of lean pp, 1.5 mm of hub bob. It is all rocking.
- **On the pod:** 0.2° of lean pp and 0.12 mm of hub bob at 0.3 m/s; 0.25 mm and 8.3 N rms at 1 m/s.
  - Fork roll stiffness from 300 to 3,000 N·m/rad changes it by ≤0.2 dB. Down at 30 N·m/rad it rocks 2° (+6 dB).
  - So on the robot the rocking is not what remains.
- **What remains on the pod is the handover's stiffness change**, which is like a gear's mesh stiffness.
  - One contact sinks ~0.22 mm under the quarter load; two contacts share it and sink ~0.14 mm. So the hub bobs ~0.08 mm at every handover, upright or not.
  - At 1 m/s the handovers come at 27 Hz, and their harmonics drive the hub on its tread (~70 Hz), which is where the 8 N rms comes from.
- **Camber the fork holds** (build error, a floor slope across the wheel): 0.5° adds a 0.3 mm step per handover. Hub bob goes 0.25 → 0.53 mm and the floor ripple rises 1.5 dB at 1 m/s.

## 2. Does the sim show it?

- **The robot sim does not.** `robot_model.py` with `wheel_model="rollers"`, the arm robot driving straight at 0.3 / 1 m/s (`wheel_wobble.py mj`):
  - Wheel-load ripple is 1.1–3.1 N rms against 0.08 N for the round "sphere" wheel, so the rollers do show a little.
  - But its hubs cannot lean on their pods (the camber is rigid), and its rollers are longer than the CAD's: half-length 22.3 mm (`coverage` 0.62), a 7° overlap against 2.35°. So it cannot show the rocking, and it shows less handover than the drawn wheel.
- **`wheel_noise.py` did not either.** It has the lean freedom, but even its hand case used the fork's 300 N·m/rad, and it scored only 0.4–10 kHz, A-weighted: the clicks. This thump is at 8–30 Hz plus harmonics.

## 3. What helps (dB against v13; floor-force ripple 5–500 Hz; pod case unless noted)

| Change | 0.3 m/s | 0.6 m/s | 1 m/s | 1 m/s, 0.5° camber | Hand 0.3 m/s | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| **Tip relief 0.2 mm / last 7 mm (R44.5 tangent arc)** | **−2.8** | **−4.8** | **−9.6** | **−4.7** | −0.7 | Hub bob 249 → 95 µm at 1 m/s. Light load −9.8, 18 kg robot −4.4, roller radius error 0.1 mm −1.9 / −6.6 |
| Relief 0.3 / 8 | −0.4 | −1.8 | −6.4 | −6.6 | | Deeper: better on a lean, worse at light load (−2.3) |
| TPU 85A tread | −0.8 | −0.5 | −1.6 | −3.2 | −3.3 | With the 0.2 / 7 relief: −3.1 / −4.4 / −6.7 / −6.8 / −3.8 (the relief was tuned for 95A's sink) |
| Rows ±14 (smaller rollers) | −0.1 | | −0.4 | −1.0 | −2.6 | Step 25 % lower |
| Rows ±10 | −0.1 | | −0.9 | −2.3 | −6.6 | Step 42 % lower; rollers about half the size |
| Longer rollers, half-length 22.5 (no relief) | +0.2 | | −3.2 | −1.4 | −1.8 | Does not fit: neighbouring M3 axles meet 9.5 mm past the thrust faces |
| 2 × 5 rollers, 4–8° overlap, relief 0.3 / 8 | −3.0 | −5.3 | −10.2 | −9.2 | −1.4 | Hub redrawn; no better than relief on 2 × 6 |
| Sharp roller ends (no fillet) | +0.5 | | −2.4 | −0.6 | −1.3 | |

**Reading:**

- **The relief is the robot's lever.** The incoming roller takes up the load over a few millimetres instead of in a step, as tip relief does on gear teeth. Its drop at the upright handover (0.14 mm, CAD) is below the tread's own sink under load.
  - Deeper or longer reliefs win in some cases but lose at light load or walking pace. 0.2 mm over 7 mm is the best all-round choice.
  - It is only a TPU tread change: wheel_quiet's 687 cartridges and hub stay.
- **Nothing on the wheel stops the free rocking of a hand-rolled wheel** short of putting both rows in one plane, i.e. a single-plane design with alternating large and small rollers (not drawn). Every double-row omni wheel rocks like this when rolled loose.
  - The robot's fork prevents it. So **test by hand with the hub held level**: a rod through the bore held at both ends, or the pod on its jig pushed along the floor. A wheel tipping freely on its hub is not what the robot does.
- **On the robot, keep the wheel upright**: fork, motor-face and pivot alignment within ~0.2° of the chassis, and no roll play in the pivot or the 6804/motor bearing stack. Every 0.1° of camber is a 0.06 mm step at every handover.

**Limits:** the contact stiffness and damping are estimates (Hertz on a TPU half-space; the real tread is 2.5–5.3 mm thick on PLA and its end contacts are truncated). There are no structural or floor modes. The 1 m/s numbers sit near the hub-on-tread resonance, so they are sensitive. Only differences between variants mean anything.

# Shopping BOM — arm robot

Based on [design.md](../design.md) and current [v14 CAD](../cad/v14/README.md), November 13, 2026. Quantities are for one complete four-wheel SO-101 robot, before subtracting anything already owned. This is a planning shopping list: CAD, power-board design and several product choices remain open. No prices or stock checked.

## Main components

| Qty | Buy | Notes |
| --- | --- | --- |
| 4 | CubeMars AK40-10 V3.0 KV170 geared motors | Integrated drivers; no separate wheel ESCs. Buy/test one pod before completing the set. |
| 1 | SO-101 follower arm kit | Six servos, servo cables and hardware; choose the follower kit, not a leader/follower pair. Check kit contents before buying duplicate parts. |
| 1 | SO-101 USB servo-bus adapter | Required unless included in arm kit; compatible with its servos and 12 V supply. |
| 1 | Intel RealSense D435 | Arm riser camera; include a suitable USB 3 cable. |
| 1 | SLAMTEC RPLIDAR A2M12 plus USB adapter/cable | Baseline; existing A2M8 bench unit is a different model. |
| 1 | Teensy 4.1 | Robot MCU; include headers and USB data cable. |
| 1 | BNO085 IMU breakout | SPI-capable, compatible with Teensy 3.3 V logic. |
| 1 | Robot computer with RAM/storage and Wi-Fi | Choice OPEN: leading candidates BMAX B8A Pro 8745HS and GMKtec M5 Ultra 7730U; $500 Getorli 8745HS and $700 UM880 Plus / SER9 H255 alternatives in the refreshed trade. See [compute trade](computer/compute_purchase_trade_2026-11-02.md). Verify DC input before buying power hardware. |
| 1 | OVONIC Roam 6S1P 5200 mAh LiPo | Baseline pack; verify actual dimensions and connector. |
| 1 | 6S-compatible balance charger and matching charge leads | Supporting equipment; model not selected. |

## Wheels, suspension and printed structure

| Qty | Buy | Notes |
| --- | --- | --- |
| 96 | 687 bearings, 7 × 14 × 5 mm | 4 wheels × 12 rollers × 2 bearings. Add spares as desired. |
| 4 | 6804 bearings, 20 × 32 × 7 mm | One outer wheel support per pod. |
| 8 | 627 bearings, 7 × 22 × 7 mm | Two fork pivots per pod. |
| 16 | Yellow printer-bed springs, OD 8 / ID 4 × 25 mm | Four per pod; actual spring rate/load capacity VERIFY. |
| 16 sets | M4 button-head plunger bolts, washers and nyloc nuts | Bolt length must match current pod CAD; confirm before ordering. |
| 48 | M3 × 50 roller axle screws | One per roller; head style per wheel CAD. |
| 1 lot | Remaining M3 assembly screws and any specified washers/nuts | Mixed lengths and pan/flat heads: extract final quantities from v14 hardware records before ordering. Includes chassis, pods, arm, mounts and electronics; do not treat an assortment as a verified final count. No inserts or steel dowels in the baseline. |
| As needed | Bambu PLA Basic Jade White | Chassis, second floor, riser, bin deck and face. Spool quantity TBD from final slicing. |
| As needed | Bambu PLA Basic Dark Gray | Arm printed parts, pods, roller cores/sleeves and floor brackets. |
| As needed | Bambu PLA Basic Orange | Bumper and wheel hubs. |
| As needed | Black TPU | 48 printed roller treads; grade and spool quantity to confirm. |
| 1 lot | Gripper finger-pad material | Material/thickness TBD. Bin, riser, wheels and chassis are printed parts, not separate purchased assemblies. |

## Power, safety and wiring

These are required functions; the carrier/distribution board and exact parts are still proposed. Confirm ratings and connectors against the final electrical design.

| Qty | Buy | Notes |
| --- | --- | --- |
| 1 | 19 V 10 A DC converter | Proposed 22–40 V input buck may drop out before the 21 V pack stop; verify or select suitable buck-boost. Match computer input. |
| 1 | 12 V 8 A DC converter | Arm rail; e-stop cuts its input. |
| 1 | 5 V 5 A DC converter | MCU, sensors, face/audio rail. |
| 1 set | Main disconnect/anti-spark switch and DC-rated e-stop contactor circuit | E-stop removes drive and arm power while leaving computer powered. Exact devices TBD. |
| 1 | Latching emergency-stop button | Include sense wiring to MCU; final circuit TBD. |
| 1 + spares | 30 A main fuse and holder | Baseline design value. |
| 4 + spares | 7.5 A motor fuses and holders | One per motor. |
| 1 lot | Converter branch fuses/holders | Ratings to finalize with wiring and converter selection. |
| 1 set | Power distribution board and Teensy carrier | Custom/proposed: boards, connectors and assembly parts, including BUS_EN switching. No finalized orderable kit yet. |
| At least 1 | 3.3 V-compatible CAN transceiver | CAN1 for four motors; second transceiver only if reserved payload CAN is populated. |
| 2 | 120-ohm CAN termination resistors | Both ends of proposed out/return daisy chain on carrier; disable motor termination as required. |
| 1 set | Battery voltage divider, rail sensing and battery current sensor | MCU analog-compatible; component values/rating TBD. |
| 1 each | Buzzer, status LED and associated passives | MCU status/warnings. |
| 1 lot | 18 AWG motor/power wire, twisted CAN pairs and lighter signal wire | Lengths from final harness; preserve motor service loops. |
| 4 sets | Motor mating plugs/leads | CubeMars XT30(2+2) power/CAN plug; verify actual mating connector. |
| 1 lot | Battery connectors, XT30 connectors, signal plugs, terminals and heat-shrink | Exact types/counts TBD; include arm and converter DC connections. |
| 1 lot | Zip ties | Harness fastens through printed slots. |
| 1 set | USB data cables and display cable | D435 USB 3, Teensy, lidar adapter, arm adapter; lengths/plug clearance to verify. |

## Interface and unresolved extras

| Qty | Buy / decide | Status |
| --- | --- | --- |
| 1 | 5-inch face display | Model, interface, power and mounting TBD; CAD assumes HDMI/5 V. |
| 1 | Microphone or USB microphone array | Model TBD. |
| 1 each | Speaker and compatible amplifier | Model TBD; CAD harness assumes 4-ohm 3 W speaker and PAM8302-class amplifier. |
| 1 set | RC transmitter and compatible ELRS/CRSF receiver | Manual override; models TBD. |
| 1 | Gamepad | Console control; model TBD, reuse existing if suitable. |
| TBD | Wrist camera | Design/simulation mentions one but physical selection is unresolved. D435 is the only adopted arm camera; do not order an extra camera until this is reconciled. |
| Optional, 1 | Forward IMX662 USB camera | Proposal only for tags/people; not adopted. |
| If needed, 1 | USB hub | Four baseline USB devices fill four ports; extra cameras/microphone may require more. Verify power and bandwidth. |
| 1 set | Printed AprilTags and calibration cube | Calibration supplies; use official tag images. |

Catch hardware (stereo cameras, flywheels, net, feed motors, tilt servo and beams) is excluded because that payload is paused. Bench-only UNO R4 hardware is not part of the robot BOM. Check existing motors, Teensy, D435, Pi and other bench equipment against this list before ordering; ownership is not established here.

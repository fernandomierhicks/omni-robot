# pcb/v1: Teensy 4.1 carrier (schematic draft, for review)

A two-layer breakout for the robot's Teensy 4.1: every header pin to a JST XH connector (2–6 pins), the CAN
transceivers as plug-in modules, and only the few parts a bare breakout can't do without. **Status (Nov 17): rev C
schematic for review; board placement study, not routed.** Every part is through-hole (Fernando solders by hand). Target fab:
OSH Park 2-layer (6 mil trace/space, 13 mil drill, 7 mil annular ring, 1.6 mm, ENIG, $5 per square inch for three).
How the board sits in the robot: [robot_block_diagram.html](robot_block_diagram.html) (published as an artifact).

| File | What |
| --- | --- |
| `tools/gen_sch.py` | **The source.** Generates the schematic, the project's symbol/footprint libraries and `nets.json` (the intended connectivity). Edit this, not the sheet, until the schematic is approved; after that KiCad is the source |
| `tools/check.py` | KiCad ERC; KiCad's netlist against `nets.json`; every Teensy pin against `firmware/lib/omni_fw/board_teensy41.h` (and the firmware changes this board needs); the review PDF |
| `omni_carrier.kicad_sch` / `.kicad_pro` | Open in KiCad 10 |
| `omni_carrier.kicad_sym`, `omni_carrier.pretty/` | Teensy 4.1 (outer 48 pins) and the SN65HVD230 module; the Teensy's 2 × 1x24 socket footprint, the Ø12 buzzer's (slotted pin 2) and J5's (1x4 socket + the ELRS receiver's outline and model) |
| `tools/elrs_model.py` → `models/elrs_receiver_on_J5.step` | The ELRS receiver on its male header, with its U.FL plug and T antenna, in J5's 3D frame (sizes from the vendor's pictures: VERIFY) |
| `out/omni_carrier_schematic.pdf` | The sheet for review (A2) |
| `tools/gen_pcb.py` | Places the footprints on `omni_carrier.kicad_pcb` (placement study; runs in KiCad's Python) |
| `robot_block_diagram.html` | The whole robot's power and signal block diagram |

```powershell
cd pcb/v1
& $env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe tools/gen_sch.py
& $env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe tools/check.py    # ERC 0, netlist = intent, firmware pins
```

## The robot around it (Fernando, Nov 17)

- **One main switch, which is also the e-stop**, right before the 30 A fuse: it cuts the whole robot, the computer
  and the Teensy included. There is no separate e-stop circuit, no contactor and no power board: the converters
  (19 V, 12 V, 5 V) are discrete DC/DC modules, and the four AK40-10s take the pack directly with no fuses of their
  own (they limit their own current).
- **One common ground** in the robot; CAN carries no ground wire.
- **USB:** the BMAX B8A Pro's four A and one C ports are all taken without a hub (D435, Teensy, servo adapter, lidar
  adapter, mic), and the GMKtec M5 Ultra has fewer. Proposed: a powered USB 2 hub on the 5 V rail for the lidar adapter,
  the mic and the servo adapter (plus spares); the D435 (USB 3) and the Teensy keep direct ports.

## Design (rev C)

- **Power: the Teensy runs from USB only.** +5V on the connectors is the Teensy's VIN (= VUSB, trace intact); +3V3 is
  its regulator (~250 mA for the BNO085, the CAN modules ≤ 20 mA each, the 3V3 LED and the spares).
- **Through-hole only:** standing 1/4 W resistors (2.54 mm pitch), disc and radial capacitors, a 2N7000 (TO-92), a
  1N4148 (DO-35), a 3 mm LED; the Teensy (2 × 1x24) and the CAN modules (1x6) in 0.1 in sockets.
- **CAN1 (J6: H, L):** the AK40-10s are daisy-chained in the harness. The SN65HVD230 module (U2, `components/can`;
  pins 3V3, GND, CTX, CRX, CANH, CANL) carries its own 120 Ω, terminating this end; **the far end of the chain needs
  the second 120 Ω** (harness or last motor). **CAN3 (J10: H, L):** reserved payload bus, U3 fitted when used.
- **IMU, the BNO085 board in `components/imu`:** J4 = VCC, GND; J3 = SCL, SDA, AD0, CS, INT, RST in the board's own
  order, so the cable runs straight. In SPI mode SCL is SCK (13), SDA is MISO (12), AD0 is MOSI (11) (BNO08x pin
  functions; VERIFY on the board). PS0/PS1 are tied high on the board; BOOTN is not wired.
- **Pack voltage (J1: pack +, GND):** 100k/10k on A17 (= the firmware's `V_BUS_DIVIDER` 11.0, safe to ~36 V), after
  the main switch and fuse. It stays analog, so the 21 V low-battery stop never depends on I2C.
- **Buzzer (BZ1, on the board, Nov 18):** a 12085 passive magnetic buzzer (Ø12 × 8.5 mm, 42 Ω, 5 V;
  `components/buzzer`) soldered straight to the carrier, no lead. 2N7000 low-side switch with a 1N4148 flyback to
  +5V; firmware `tone()` drives it (a passive buzzer needs the square wave). About 120 mA peak from USB 5 V, ~60 mA
  average while sounding. Footprint `omni_carrier:Buzzer_D12_P6.5-7.6_Slot`: pin 1 = + (square pad, marked +), pin 2
  in a plated slot that takes either common pin pitch (6.5 or 7.6 mm). **VERIFY the pitch and + mark on the
  delivered buzzers.** The 2N7000 at 3.3 V gate is near its threshold; if the buzzer is quiet, swap Q1 for a
  logic-level TO-92 FET.
- **RGB status LED (J15: R, GND, G, B, Nov 18):** common cathode, driven straight from pins 28 (R), 29 (G), 5 (B)
  through the board's 330 / 100 / 100 Ω (~4 mA each at 3.3 V; VERIFY brightness on the LED chosen). J15 is in the
  LED's own leg order (R, cathode = the longest leg, G, B), so the LED can solder straight into J15's holes instead of
  the XH (2.50 vs 2.54 mm: 0.12 mm over the four legs) or sit on a lead. VERIFY the leg order on the LED bought.
- **NeoPixel strip (J20: +5V, DIN, GND):** driven straight from pin 35 (Serial8 TX, so WS2812Serial can update it
  without blocking the 1 kHz loop) through 330 Ω; Fernando has run strips on 3.3 V data. +5V here is USB, so short
  strips only (~10 pixels at modest brightness); a longer strip takes 5 V from the 5 V converter at the strip and only
  DIN + GND from here.
- **ELRS receiver (J5, on the board, Nov 18, Fernando):** the HPXGRC ELRS 2.4 GHz receiver
  (`components/radio receiver`: 20 × 11 mm, 0.8 g, four pads T R V G on one short edge, a U.FL and a 40 mm T
  antenna) plugs straight into the carrier, no lead: a straight male 1x4 header soldered into its pads (short pins
  up through the pads, the plastic under it) goes into **J5, a 1x4 2.54 mm female socket**. The receiver lies flat
  along the top edge, components up, 11 mm over the board (socket 8.5 + the header's plastic 2.5), its U.FL toward
  −x; nothing is placed under it (its outline is in J5's courtyard). Read from the G end its pads are G, V, R, T =
  J5's 1-4 (GND, 5 V, its RX ← Teensy TX2 (8), its TX → Teensy RX2 (7)). Footprint
  `omni_carrier:ELRS_Rx_Socket_1x04_P2.54mm` carries KiCad's socket model and the receiver's
  (`models/elrs_receiver_on_J5.step`, from `tools/elrs_model.py`). **VERIFY before ordering, with calipers: the pads'
  pitch (the vendor's pictures read 2.3-2.45 mm; a 2.54 header needs 2.54) and that the pads are plated through.**
  If they are not, the fallback is the receiver's own pre-soldered leads crimped into a 1x4 Dupont housing on the
  same socket. The antenna's 40 mm coax stands up from the U.FL in the CAD; route it upright, away from the computer
  and the converters.
- **Power monitors on I2C (pins 18/19):** J12 to the INA3221, J21 to the INA226, same bus; the modules' own pull-ups
  serve it. Both sit with the converters, on cables, so no rail current crosses the carrier.
- **Spares (J11, J16–J19):** every remaining header pin. Each spare signal has a 10k position (DNI by default) to its
  connector's bias net; a 3-pad solder jumper per connector sets the bias: pads 1-2 = pull-up to 3V3, 2-3 =
  pull-down. Fit only the resistors a signal needs (open-drain, floating inputs).

## Power monitoring

| Monitor | Measures | Shunt | Full scale |
| --- | --- | --- | --- |
| **INA226** (rated to 36 V), J21, set to 0x41 | **Robot total current**, high side after the fuse | external 50 A / 75 mV (1.5 mΩ) | ~55 A |
| INA3221 CH1, J12 | **Arm 12 V** (1 A typical, 6.7 A peak est) | 10 mΩ 2512 (replace R100) | 16.4 A |
| INA3221 CH2 | **Computer 19 V** (1.6 A typical, 3.4 A peak) | 20 mΩ 2512 | 8.2 A |
| INA3221 CH3 | 5 V rail (display, audio, USB hub) | 50 mΩ 2512 | 3.3 A |

**Why the total isn't on the INA3221:** its inputs are rated 0–26 V absolute maximum and a full 6S pack is 25.2 V,
so regen or a charger spike can exceed it, and its module's traces aren't for 20 A bursts. The INA226 (GY-INA226
class modules, 36 V) on an external shunt takes the pack current; the pack voltage stays on the divider.

## Connectors

| Ref | Pins | 1 | 2 | 3 | 4 | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| J1 pack sense | XH2 | pack + | GND | | | | |
| J3 BNO085 data | XH6 | SCL = SCK (13) | SDA = MISO (12) | AD0 = MOSI (11) | CS (10) | INT (9) | RST (6) |
| J4 BNO085 power | XH2 | VCC (3V3) | GND | | | | |
| J5 ELRS (receiver plugged in) | 1x4 2.54 socket | GND (rx G) | 5 V (rx V) | TX2 (8) → rx R | RX2 (7) ← rx T | | |
| J6 CAN1 | XH2 | CAN H | CAN L | | | | |
| J10 CAN3 | XH2 | CAN H | CAN L | | | | |
| J11 spare | XH6 | GND | 3V3 | 0 RX1 | 1 TX1 | 27 | 40 (A16) |
| J12 INA3221 | XH4 | 3V3 (VS) | GND | SDA (18) | SCL (19) | | |
| J13 camera / sync | XH4 | GND | CAM_TRIG (24) | SYNC_OUT (25) | CAM_STROBE (26) | | |
| J15 RGB LED | XH4 | R (28) | GND | G (29) | B (5) | | |
| J16 spare | XH6 | GND | 3V3 | 14 TX3 | 15 RX3 | 16 RX4 | 17 TX4 |
| J17 spare | XH6 | GND | 3V3 | 2 | 3 | 20 TX5 | 21 RX5 |
| J18 spare | XH6 | GND | 32 | 33 | 36 | 37 | 38 (A14) |
| J19 spare | XH4 | 3V3 | GND | 39 (A15) | 34 | | |
| J20 NeoPixel | XH3 | 5 V | DIN (35) | GND | | | |
| J21 INA226 | XH4 | 3V3 (VS) | GND | SDA (18) | SCL (19) | | |

## Board: placement study (Nov 17; routing study below)

`tools/gen_pcb.py` (KiCad's own Python: `"C:\Program Files\KiCad\10.0\bin\python.exe" tools/gen_pcb.py`, after
`tools/check.py`) builds `omni_carrier.kicad_pcb` from the netlist, with each footprint linked to its symbol (so
KiCad's Update PCB from Schematic still works) and OSH Park's 2-layer rules. Renders: `out/placement_top.png`,
`out/placement_iso.png`. KiCad DRC: no courtyard overlaps; the open items are the unrouted connections and
reference-text placement.

- **Size: 102 × 62 mm** (9.8 sq in, about $49 for three at OSH Park; 7 mm wider than the earlier 95 × 62 for the
  corner holes, Nov 18). The CAD's carrier envelope (100 × 65 mm) adapts to it; it sits
  on top of the power stack (`cad/v15/tools/upper.py` BLOCK `mcu`, centred at robot (37, −143.5), z 252.5).
- **Edges follow the cables in `cad/v15`:** board x = robot +x, board y down = robot −y.
  - **Left (robot −x, toward the computer):** the Teensy's USB end.
  - **Top (robot +y, toward the IMU at (32, −40) and the robot's centre):** J4, J3 IMU; J20 NeoPixel; J15 RGB (their
    resistors under them); then J5 at the right end (pin 1 at board (80.2, 2.5)) with the ELRS receiver lying over
    the strip x 61.1-82.0, y 0.5-12.1 above the Teensy's end. BZ1 (the buzzer) in the top-right corner below it
    (courtyards 0.8 mm apart); its Q1, R12, D1 between C6 and J19.
  - **Bottom (robot −y, toward the motor leads' fin slot and the converters below):** J6 CAN1, J10 CAN3, J1 pack
    sense, J12 INA3221, J21 INA226, J13 camera/sync.
  - **Right (robot +x):** spares J11, J16, J17, each with its bias column, 7 mm in from the edge (the corner holes);
    J18 on the left under the Teensy, J19 inside. The top and bottom rows start 7 mm in from the left corners.
- **Mounting (Fernando, Nov 18): four M3 holes at the corners**, 3.5 mm in from both edges: board (3.5, 3.5),
  (98.5, 3.5), (3.5, 58.5), (98.5, 58.5), 95 × 55 mm apart; each corner kept clear 7 mm for the hole and an M3 pan
  head (Ø6.5). They replace the earlier holes inside the board. The CAD adapts to them (its tray / posts are not
  drawn yet; M3 into Ø2.8 pilots).
- **Height (VERIFY against the CAD's 25 mm envelope):** the Teensy on 8.5 mm sockets stands about 12 mm; each CAN
  module standing in its 1x6 socket about 27 mm; an XH plug with its wires about 20 mm; the ELRS receiver about
  14.5 mm (its U.FL plug), its antenna about 52 mm as drawn (upright).
- **STEP:** `"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe" pcb export step -f -o out/omni_carrier.step
  omni_carrier.kicad_pcb` (includes the receiver); `cad/v15` reads a copy in `reference/electronics/omni_carrier_v1.step`.
- **Fab rules in the project file:** `gen_pcb.py` writes OSH Park's limits (0.152 mm track / space, 0.33 mm drill, 0.178 mm
  ring, 0.381 mm copper to edge) and the net classes (signals 0.2 mm, power +5V / +3V3 / GND 0.5 mm, 0.2 mm clearance,
  vias 0.8 / 0.4 mm) into `omni_carrier.kicad_pro`; KiCad keeps a board's rules there, not in the board file.
  Q1 is on `TO-92_Inline_Wide` (2.54 mm pitch; spread the 2N7000's legs): the 1.27 mm footprint's copper ring is 0.15 mm.

## Preliminary routing (Nov 19): two layers work

`tools/route.py` (KiCad's Python, after `gen_pcb.py`) autoroutes the placement with Freerouting 2.5.0 and writes
**`omni_carrier_routed.kicad_pcb`** (a study; the placement board stays the source until the layout is approved).
Result: **every connection routed on two layers**, about 3.7 m of track and 24 vias, GND poured on both sides; KiCad
DRC against the OSH Park rules: 0 unconnected, no clearance, width or edge errors (left: silkscreen text overlaps from the
placement, and 5 GND pads whose pour thermal has one spoke instead of two: they are also on GND tracks). Renders:
`out/route/routed_top.png`, `routed_bottom.png`; DRC `out/route/drc_routed.json`.

- The router only finishes this board with its automatic neckdown on: it squeezes tracks to 0.12-0.19 mm between
  pads (with it off it stalls at 34-37 unrouted, at 0.25 or 0.2 mm tracks). `route.py` widens every track under
  0.16 mm back to 0.16 after import; the router kept 0.2 mm around them, so they keep >= 0.18 mm to their neighbours.
- An autorouted layout is a feasibility answer, not the board to order: hand-tidy or reroute it once the placement and
  the CAD heights are settled (long detours round the Teensy, a few vias that a hand route would not need).
- Tools (not in the repo; download and verify the checksums): `freerouting-2.5.0.jar` (github.com/freerouting/freerouting
  releases; the v2.5.0 native `freerouting-cli.exe` crashes on Windows) on a Java 25 runtime (Temurin 25 JRE zip;
  the jar is class version 69). `FREEROUTING_JAR=<jar> JAVA=<jre>\bin\java.exe "C:\Program Files\KiCad\10.0\bin\python.exe"
  tools/route.py`. Do not run it with the project open in KiCad: it clears KiCad's project lock file.

## Firmware to change for this board (`tools/check.py` lists them)

`board_teensy41.h`: the e-stop input (pin 35) goes, as the main switch now removes all power (left as it is, pin 35
would read 1 = stop), and 35 becomes the NeoPixel data; pin 5 (was BNO_WAKE) → LED_B, 29 (was BNO_PS1) → LED_G,
28 LED → LED_R; 27 (BNO_BOOTN), 34, 39 (I_BUS) and 40 (V_12) become spares. `main.cpp`: an INA3221 + INA226 reader on
Wire; RGB and NeoPixel status; stop driving the BNO085's WAKE/PS1/BOOTN.

## Open for the review

- The main switch: it must carry the whole robot (20 A bursts, 30 A fuse) and the converters' inrush. Few latching
  e-stop buttons are rated for that much DC, so pick a rated battery-disconnect switch or let the button drive a
  high-current anti-spark switch. It also drops the computer without a shutdown.
- The INA226's address: set it to 0x41 (A0 to VS); both modules default to 0x40.
- The CAD changes to fit it (blue sky, not drawn; block diagram page, "Blue sky: power down, logic in the middle"):
  the power parts move to the first floor's centre over the battery (the drive leads stay down there); the carrier
  sits in the open middle of floor 2 between the arm and the bin, plugs up; the computer moves under the bin; one
  slot beside the spine fin carries everything that climbs. Board size: as small as the layout allows (Fernando).
- The height of the CAN modules standing in their sockets (see Board).

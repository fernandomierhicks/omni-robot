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
| `omni_carrier.kicad_sym`, `omni_carrier.pretty/` | Teensy 4.1 (outer 48 pins) and the SN65HVD230 module; the Teensy's 2 × 1x24 socket footprint |
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
- **Buzzer (J14):** 2N7000 low-side switch with a 1N4148 flyback to +5V; any 5 V buzzer, firmware `tone()`.
- **RGB status LED (J15: R, G, B, GND):** common cathode, driven straight from pins 28 (R), 29 (G), 5 (B) through
  330 / 100 / 100 Ω (~4 mA each at 3.3 V; VERIFY brightness on the LED chosen).
- **NeoPixel strip (J20: +5V, DIN, GND):** driven straight from pin 35 (Serial8 TX, so WS2812Serial can update it
  without blocking the 1 kHz loop) through 330 Ω; Fernando has run strips on 3.3 V data. +5V here is USB, so short
  strips only (~10 pixels at modest brightness); a longer strip takes 5 V from the 5 V converter at the strip and only
  DIN + GND from here.
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
| J5 ELRS | XH4 | 5 V | GND | RX2 (7) ← rx TX | TX2 (8) | | |
| J6 CAN1 | XH2 | CAN H | CAN L | | | | |
| J10 CAN3 | XH2 | CAN H | CAN L | | | | |
| J11 spare | XH6 | GND | 3V3 | 0 RX1 | 1 TX1 | 27 | 40 (A16) |
| J12 INA3221 | XH4 | 3V3 (VS) | GND | SDA (18) | SCL (19) | | |
| J13 camera / sync | XH4 | GND | CAM_TRIG (24) | SYNC_OUT (25) | CAM_STROBE (26) | | |
| J14 buzzer | XH2 | 5 V | switched − | | | | |
| J15 RGB LED | XH4 | R (28) | G (29) | B (5) | GND | | |
| J16 spare | XH6 | GND | 3V3 | 14 TX3 | 15 RX3 | 16 RX4 | 17 TX4 |
| J17 spare | XH6 | GND | 3V3 | 2 | 3 | 20 TX5 | 21 RX5 |
| J18 spare | XH6 | GND | 32 | 33 | 36 | 37 | 38 (A14) |
| J19 spare | XH4 | 3V3 | GND | 39 (A15) | 34 | | |
| J20 NeoPixel | XH3 | 5 V | DIN (35) | GND | | | |
| J21 INA226 | XH4 | 3V3 (VS) | GND | SDA (18) | SCL (19) | | |

## Board: placement study (Nov 17; no tracks yet)

`tools/gen_pcb.py` (KiCad's own Python: `"C:\Program Files\KiCad\10.0\bin\python.exe" tools/gen_pcb.py`, after
`tools/check.py`) builds `omni_carrier.kicad_pcb` from the netlist, with each footprint linked to its symbol (so
KiCad's Update PCB from Schematic still works) and OSH Park's 2-layer rules. Renders: `out/placement_top.png`,
`out/placement_iso.png`. KiCad DRC: no courtyard overlaps; the open items are the unrouted connections and
reference-text placement.

- **Size: 95 × 62 mm** (9.1 sq in, about $46 for three at OSH Park), inside the CAD's 100 × 65 mm carrier envelope
  on top of the power stack (`cad/v15/tools/upper.py` BLOCK `mcu`, centred at robot (37, −143.5), z 252.5).
- **Edges follow the cables in `cad/v15`:** board x = robot +x, board y down = robot −y.
  - **Left (robot −x, toward the computer):** the Teensy's USB end.
  - **Top (robot +y, toward the IMU at (32, −40) and the robot's centre):** J4, J3 IMU; J5 ELRS; J20 NeoPixel;
    J15 RGB; J14 buzzer.
  - **Bottom (robot −y, toward the motor leads' fin slot and the converters below):** J6 CAN1, J10 CAN3, J1 pack
    sense, J12 INA3221, J21 INA226, J13 camera/sync.
  - **Right (robot +x):** spares J11, J16, J17, each with its bias column; J18 on the left under the Teensy, J19
    inside.
- **Mounting:** four M3 holes inside the board, at board (24, 50.5), (49, 36.5), (74.5, 26.5), (64, 39.5). The CAD's
  stack screws are 154 × 54.5 mm apart, wider than the board, so the proposal is a printed tray on the stack's four
  posts (clamped by the existing M3 × 80 screws) with bosses for these holes (M3 into Ø2.8 pilots). This is a
  `cad/v15` change, not drawn.
- **Height (VERIFY against the CAD's 25 mm envelope):** the Teensy on 8.5 mm sockets stands about 12 mm; each CAN
  module standing in its 1x6 socket about 27 mm; an XH plug with its wires about 20 mm.

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

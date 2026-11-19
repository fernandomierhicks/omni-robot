"""Generate the omni-robot Teensy 4.1 carrier v1 schematic (KiCad 10) and its project files.

    python tools/gen_sch.py          (from pcb/v1; any Python 3.9+, no packages)

Writes omni_carrier.kicad_sch, omni_carrier.kicad_pro (only if missing), omni_carrier.kicad_sym (Teensy 4.1 and the
SN65HVD230 module), omni_carrier.pretty/Teensy41_Socket.kicad_mod, sym-lib-table, fp-lib-table and nets.json (the
intended connectivity, checked against KiCad's own netlist by tools/check.py).

The schematic connects by net labels at every pin (plus a few drawn wires inside blocks), so every net is named.
Pin assignments follow firmware/lib/omni_fw/board_teensy41.h; connectors follow cad/v13 README "Wiring harness".
"""
from __future__ import annotations

import json
import os
import re
import uuid as _uuid

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
KICAD = os.environ.get("KICAD_SHARE", r"C:\Program Files\KiCad\10.0\share\kicad")
PROJECT = "omni_carrier"
ROOT_UUID = "6f1e2c4a-0b7d-4c55-9a51-1c0de0ca7701"
_ns = _uuid.UUID("2b0f6a3e-5d1c-4b8e-9f00-0a1b2c3d4e5f")
_count = [0]


def uid() -> str:
    """deterministic UUIDs, so a regeneration diffs cleanly"""
    _count[0] += 1
    return str(_uuid.uuid5(_ns, f"{_count[0]}"))


# ---------------------------------------------------------------------------------------------- S-expression helpers
def balanced(text: str, start: int) -> str:
    """the balanced (...) block starting at text[start] == '(' (strings respected)"""
    depth, i, in_str = 0, start, False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == "\\":
                i += 1
            elif c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return text[start:i + 1]
        i += 1
    raise ValueError("unbalanced")


_lib_cache: dict[str, str] = {}


def stock_symbol(lib_id: str) -> str:
    lib, name = lib_id.split(":")
    if lib not in _lib_cache:
        with open(os.path.join(KICAD, "symbols", f"{lib}.kicad_sym"), encoding="utf-8") as f:
            _lib_cache[lib] = f.read()
    text = _lib_cache[lib]
    m = re.search(r'\n\t\(symbol "%s"\n' % re.escape(name), text)
    if not m:
        raise KeyError(lib_id)
    block = balanced(text, m.start() + 2)
    if "(extends " in block:
        raise ValueError(f"{lib_id} extends another symbol: pick its base")
    return block.replace(f'(symbol "{name}"', f'(symbol "{lib_id}"', 1)


def pins_of(block: str) -> dict[str, tuple[float, float, int]]:
    """pin number -> (x, y, rot) of its connection point, in symbol coordinates (y up)"""
    out = {}
    for m in re.finditer(r"\(pin \w+ \w+", block):
        p = balanced(block, m.start())
        at = re.search(r"\(at ([-\d.]+) ([-\d.]+) (\d+)\)", p)
        num = re.search(r'\(number "([^"]*)"', p)
        out[num.group(1)] = (float(at.group(1)), float(at.group(2)), int(at.group(3)))
    return out


def pin_units(block: str) -> dict[str, int]:
    """pin number -> unit (0 = common to all units)"""
    out = {}
    for m in re.finditer(r'\(symbol "[^"]*_(\d+)_(\d+)"', block):
        sub = balanced(block, m.start())
        for n in re.findall(r'\(number "([^"]*)"', sub):
            out[n] = int(m.group(1))
    return out


def esc(s: str) -> str:
    return s.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")


# ---------------------------------------------------------------------------------------------- custom symbols
def font(size=1.27):
    return f"(effects (font (size {size} {size})))"


def prop(name, value, x, y, hide=False, size=1.27, justify=None):
    j = f" (justify {justify})" if justify else ""
    h = " (hide yes)" if hide else ""
    return (f'(property "{name}" "{esc(value)}" (at {x:g} {y:g} 0){h} '
            f"(effects (font (size {size} {size})){j}))")


def pin_sexpr(kind, x, y, rot, length, name, number):
    return (f'(pin {kind} line (at {x:g} {y:g} {rot}) (length {length:g}) '
            f'(name "{esc(name)}" {font()}) (number "{number}" {font()}))')


# Teensy 4.1 outer rows, top view, USB at the top (PJRC pinout card). (pad, name shown, pin type)
TEENSY_LEFT = (["GND", "0/RX1", "1/TX1", "2", "3", "4", "5", "6", "7/RX2", "8/TX2", "9", "10/CS", "11/MOSI",
                "12/MISO", "3V3", "24", "25", "26", "27", "28", "29", "30/CRX3", "31/CTX3", "32"])
TEENSY_RIGHT = (["VIN", "GND", "3V3", "23/CRX1", "22/CTX1", "21/RX5", "20/TX5", "19/SCL", "18/SDA", "17/TX4",
                 "16/RX4", "15/RX3", "14/TX3", "13/SCK", "GND", "41/A17", "40/A16", "39/A15", "38/A14", "37",
                 "36", "35", "34", "33"])


def teensy_pin_type(pad: int, name: str) -> str:
    if name == "VIN":
        return "power_out"          # USB 5 V comes out here (VUSB-VIN trace intact): the board's +5V
    if name == "3V3":
        return "power_out" if pad == 15 else "passive"   # one driver per net for ERC
    if name == "GND":
        return "passive"
    return "bidirectional"


def teensy_symbol(lib_name: str) -> str:
    n = "Teensy4.1"
    pins = []
    for i, name in enumerate(TEENSY_LEFT):
        pins.append(pin_sexpr(teensy_pin_type(i + 1, name), -17.78, 29.21 - 2.54 * i, 0, 5.08, name, i + 1))
    for i, name in enumerate(TEENSY_RIGHT):
        pins.append(pin_sexpr(teensy_pin_type(25 + i, name), 17.78, 29.21 - 2.54 * i, 180, 5.08, name, 25 + i))
    return (f'(symbol "{lib_name}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
            f'  {prop("Reference", "U", 0, 33.02)}\n  {prop("Value", "Teensy 4.1", 0, -33.02)}\n'
            f'  {prop("Footprint", "omni_carrier:Teensy41_Socket", 0, -35.56, hide=True)}\n'
            f'  {prop("Datasheet", "https://www.pjrc.com/store/teensy41.html", 0, -38.1, hide=True)}\n'
            f'  {prop("Description", "PJRC Teensy 4.1 on two 1x24 0.1 in sockets (outer rows only)", 0, -40.64, hide=True)}\n'
            f'  (symbol "{n}_0_1" (rectangle (start -12.7 31.75) (end 12.7 -31.75) (stroke (width 0.254) (type default)) (fill (type background))))\n'
            f'  (symbol "{n}_1_1"\n    ' + "\n    ".join(pins) + ")\n  (embedded_fonts no))")


CAN_PINS = [("3V3", "power_in"), ("GND", "power_in"), ("CTX", "input"), ("CRX", "output"),
            ("CANH", "bidirectional"), ("CANL", "bidirectional")]


def can_module_symbol(lib_name: str) -> str:
    n = "SN65HVD230_Module"
    pins = [pin_sexpr(kind, -10.16, 6.35 - 2.54 * i, 0, 5.08, name, i + 1) for i, (name, kind) in enumerate(CAN_PINS)]
    return (f'(symbol "{lib_name}" (pin_names (offset 1.016)) (exclude_from_sim no) (in_bom yes) (on_board yes)\n'
            f'  {prop("Reference", "U", 0, 10.16)}\n  {prop("Value", "SN65HVD230 module", 0, -10.16)}\n'
            f'  {prop("Footprint", "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical", 0, -12.7, hide=True)}\n'
            f'  {prop("Datasheet", "https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf", 0, -15.24, hide=True)}\n'
            f'  {prop("Description", "VP230 CAN transceiver breakout (3V3 GND CTX CRX CANH CANL, 120 ohm R2 fitted) in a 1x6 0.1 in socket", 0, -17.78, hide=True)}\n'
            f'  (symbol "{n}_0_1" (rectangle (start -5.08 8.89) (end 5.08 -8.89) (stroke (width 0.254) (type default)) (fill (type background))))\n'
            f'  (symbol "{n}_1_1"\n    ' + "\n    ".join(pins) + ")\n  (embedded_fonts no))")


CUSTOM = {f"{PROJECT}:Teensy4.1": teensy_symbol, f"{PROJECT}:SN65HVD230_Module": can_module_symbol}


def symbol_def(lib_id: str) -> str:
    return CUSTOM[lib_id](lib_id) if lib_id in CUSTOM else stock_symbol(lib_id)


# ---------------------------------------------------------------------------------------------- the schematic model
class Sheet:
    def __init__(self):
        self.libs: dict[str, str] = {}
        self.pins: dict[str, dict] = {}
        self.units: dict[str, dict] = {}
        self.items: list[str] = []
        self.parts: dict[str, tuple[str, float, float]] = {}
        self.nets: dict[str, set] = {}        # intended: net -> {(ref, pin)}
        self.pwr_n = 0

    def lib(self, lib_id):
        if lib_id not in self.libs:
            self.libs[lib_id] = symbol_def(lib_id)
            self.pins[lib_id] = pins_of(self.libs[lib_id])
            self.units[lib_id] = pin_units(self.libs[lib_id])
        return self.pins[lib_id]

    # a placed symbol
    def part(self, ref, lib_id, x, y, value, footprint="", dnp=False, in_bom=True, datasheet="", fields=None,
             ref_at=(2.54, -2.54), val_at=(2.54, 2.54), unit=1):
        """a placed symbol; unit > 1 is addressed as "REF:unit" in pos / connect"""
        pins = [p for p in self.lib(lib_id) if self.units[lib_id].get(p, 1) in (0, unit)]
        self.parts[ref if unit == 1 else f"{ref}:{unit}"] = (lib_id, x, y)
        hidden_ref = ref.startswith("#")
        props = [prop("Reference", ref, x + ref_at[0], y + ref_at[1] if not hidden_ref else y + 5, hide=hidden_ref, justify="left"),
                 prop("Value", value, x + val_at[0], y + val_at[1] if not hidden_ref else y + 3.5, justify="left"),
                 prop("Footprint", footprint, x, y, hide=True),
                 prop("Datasheet", datasheet, x, y, hide=True)]
        for k, v in (fields or {}).items():
            props.append(prop(k, v, x, y, hide=True))
        pin_uuids = " ".join(f'(pin "{p}" (uuid "{uid()}"))' for p in pins)
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {x:g} {y:g} 0) (unit {unit}) (exclude_from_sim no) '
            f'(in_bom {"yes" if in_bom else "no"}) (on_board yes) (dnp {"yes" if dnp else "no"}) (uuid "{uid()}")\n  '
            + "\n  ".join(props) + f"\n  {pin_uuids}\n"
            f'  (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (reference "{ref}") (unit {unit})))))')

    def no_connect(self, key, pin):
        x, y, _ = self.pos(key, pin)
        self.items.append(f'(no_connect (at {x:g} {y:g}) (uuid "{uid()}"))')

    def pos(self, ref, pin):
        lib_id, x, y = self.parts[ref]
        px, py, rot = self.pins[lib_id][pin]
        return x + px, y - py, rot

    @staticmethod
    def outward(rot):
        return {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}[rot]

    def wire(self, a, b):
        self.items.append(f'(wire (pts (xy {a[0]:g} {a[1]:g}) (xy {b[0]:g} {b[1]:g})) '
                          f'(stroke (width 0) (type default)) (uuid "{uid()}"))')

    def junction(self, p):
        self.items.append(f'(junction (at {p[0]:g} {p[1]:g}) (diameter 0) (color 0 0 0 0) (uuid "{uid()}"))')

    def label(self, net, p, direction):
        ang = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[direction]
        just = "left bottom" if ang in (0, 90) else "right bottom"
        self.items.append(f'(label "{esc(net)}" (at {p[0]:g} {p[1]:g} {ang}) '
                          f'(effects (font (size 1.27 1.27)) (justify {just})) (uuid "{uid()}"))')

    def power(self, net, p, direction=(0, 1)):
        """a power symbol pointing along `direction` (sch coords); its net name kept horizontal"""
        lib_id = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V"}[net]
        gnd = net == "GND"
        # KiCad: GND 0 down, 90 right, 180 up, 270 left; supplies 0 up, 90 left, 180 down, 270 right
        rot = {(0, 1): 0, (1, 0): 90, (0, -1): 180, (-1, 0): 270}[direction] if gnd else \
              {(0, -1): 0, (-1, 0): 90, (0, 1): 180, (1, 0): 270}[direction]
        self.pwr_n += 1
        ref = f"#PWR{self.pwr_n:03d}"
        self.parts[ref] = (lib_id, p[0], p[1])
        self.lib(lib_id)
        d = 6.35 if direction[0] else 5.08
        vx, vy = p[0] + direction[0] * d, p[1] + direction[1] * d + (1.0 if direction[1] > 0 else 0) - (1.27 if direction[0] else 0)
        fa = rot if rot in (90, 270) else 0
        self.items.append(
            f'(symbol (lib_id "{lib_id}") (at {p[0]:g} {p[1]:g} {rot}) (unit 1) (exclude_from_sim no) (in_bom yes) '
            f'(on_board yes) (dnp no) (uuid "{uid()}")\n'
            f'  (property "Reference" "{ref}" (at {p[0]:g} {p[1]:g} 0) (hide yes) {font()})\n'
            f'  (property "Value" "{net}" (at {vx:g} {vy:g} {fa}) {font()})\n'
            f'  (property "Footprint" "" (at {p[0]:g} {p[1]:g} 0) (hide yes) {font()})\n'
            f'  (property "Datasheet" "" (at {p[0]:g} {p[1]:g} 0) (hide yes) {font()})\n'
            f'  (pin "1" (uuid "{uid()}"))\n'
            f'  (instances (project "{PROJECT}" (path "/{ROOT_UUID}" (reference "{ref}") (unit 1)))))')

    def note(self, text, x, y, size=1.27, bold=False):
        b = " (bold yes)" if bold else ""
        self.items.append(f'(text "{esc(text)}" (exclude_from_sim no) (at {x:g} {y:g} 0) '
                          f'(effects (font (size {size} {size}){b}) (justify left top)) (uuid "{uid()}"))')

    def connect(self, ref, pin, net, stub=2.54):
        """a short stub out of the pin and a label (or a power symbol) on its end"""
        x, y, rot = self.pos(ref, pin)
        dx, dy = self.outward(rot)
        end = (round(x + dx * stub, 2), round(y + dy * stub, 2))
        self.wire((x, y), end)
        if net in ("GND", "+3V3", "+5V"):
            self.power(net, end, (dx, dy))
        else:
            self.label(net, end, (dx, dy))
        self.nets.setdefault(net, set()).add((ref.split(":")[0], pin))

    def same_net(self, net, *refpins):
        """record pins already joined by drawn wires/coincident ends"""
        for rp in refpins:
            self.nets.setdefault(net, set()).add(rp)

    def write(self, path, title, rev, date, comments):
        lib = "\n".join(self.libs.values())
        cm = "\n".join(f'    (comment {i + 1} "{esc(c)}")' for i, c in enumerate(comments))
        text = (f'(kicad_sch (version 20250610) (generator "omni_gen_sch") (generator_version "10.0")\n'
                f'  (uuid "{ROOT_UUID}") (paper "A2")\n'
                f'  (title_block (title "{esc(title)}") (date "{date}") (rev "{rev}") (company "omni-robot")\n{cm})\n'
                f"  (lib_symbols\n{lib}\n  )\n"
                + "\n".join(self.items)
                + '\n  (sheet_instances (path "/" (page "1")))\n  (embedded_fonts no)\n)\n')
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)


# ---------------------------------------------------------------------------------------------- footprints
# Fernando, Nov 17: every part he solders is through-hole. Resistors stand upright (2.54 mm pitch) to save area.
R_TH = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical"
C_TH = "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm"
CP_TH = "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm"
LED_TH = "LED_THT:LED_D3.0mm"
D_TH = "Diode_THT:D_DO-35_SOD27_P2.54mm_Vertical_AnodeUp"
TO92 = "Package_TO_SOT_THT:TO-92_Inline_Wide"     # 2N7000: 1 S, 2 G, 3 D; legs spread to 2.54 (the 1.27 pitch footprint's ring is 0.15 mm, under OSH Park's 0.178)
SOCK6 = "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical"
SJ3 = "Jumper:SolderJumper-3_P1.3mm_Open_RoundedPad1.0x1.5mm"
MH = "MountingHole:MountingHole_3.2mm_M3"


def xh(n):
    return f"Connector_JST:JST_XH_B{n}B-XH-A_1x{n:02d}_P2.50mm_Vertical"


# ---------------------------------------------------------------------------------------------- the circuit
# Teensy pad -> net. Firmware pins (board_teensy41.h) except where this board changes them (FIRMWARE_TO_CHANGE)
TEENSY_NETS = {
    "GND": "GND", "3V3": "+3V3", "VIN": "+5V",
    "0/RX1": "SP_D0", "1/TX1": "SP_D1", "2": "SP_D2", "3": "SP_D3", "4": "BUZZER", "5": "LED_B",
    "6": "IMU_RST", "7/RX2": "RC_RX", "8/TX2": "RC_TX", "9": "IMU_INT", "10/CS": "IMU_CS", "11/MOSI": "IMU_MOSI",
    "12/MISO": "IMU_MISO", "13/SCK": "IMU_SCK", "14/TX3": "SP_D14", "15/RX3": "SP_D15", "16/RX4": "SP_D16",
    "17/TX4": "SP_D17", "18/SDA": "I2C_SDA", "19/SCL": "I2C_SCL", "20/TX5": "SP_D20", "21/RX5": "SP_D21",
    "22/CTX1": "CAN1_TX", "23/CRX1": "CAN1_RX", "24": "CAM_TRIG", "25": "SYNC_OUT", "26": "CAM_STROBE",
    "27": "SP_D27", "28": "LED_R", "29": "LED_G", "30/CRX3": "CAN3_RX", "31/CTX3": "CAN3_TX", "32": "SP_D32",
    "33": "SP_D33", "34": "SP_D34", "35": "NEO_3V3", "36": "SP_D36", "37": "SP_D37", "38/A14": "SP_D38",
    "39/A15": "SP_A15", "40/A16": "SP_A16", "41/A17": "VBUS_ADC",
}
# what the firmware's pin map must change to match this board (tools/check.py reports these)
FIRMWARE_TO_CHANGE = {
    "ESTOP": "pin 35 is the NeoPixel data (Serial8 TX, for WS2812Serial). The e-stop is now the main power button, "
             "so the firmware must stop reading an e-stop input (pin 35 would read 1 = stop)",
    "BNO_WAKE": "pin 5 is LED_B (the BNO085 breakout ties PS0/PS1 high)",
    "BNO_PS1": "pin 29 is LED_G",
    "BNO_BOOTN": "pin 27 is a spare (BOOTN not wired)",
    "V_12": "pin 40 is a spare: the rails are measured by the INA3221 (I2C)",
    "I_BUS": "pin 39 is a spare: currents are measured by the INA3221 / INA226 (I2C)",
    "LED": "pin 28 is now LED_R of the RGB status LED (with 29 G, 5 B), driven directly",
}


def conn(s: Sheet, ref, x, y, nets, title, note=None, footprint=None, value=None):
    """a JST XH connector (or the given footprint) whose title starts at y; returns the y below its block"""
    n = len(nets)
    lib_id = f"Connector_Generic:Conn_01x{n:02d}"
    pin1_y = s.lib(lib_id)["1"][1]
    ysym = y + 7.62 + pin1_y
    s.part(ref, lib_id, x, ysym, value or f"XH{n}", footprint or xh(n))
    for i, net in enumerate(nets):
        s.connect(ref, str(i + 1), net)
    s.note(title, x - 33.02, y + 2.54, size=1.524, bold=True)
    yb = y + 7.62 + (n - 1) * 2.54 + 2.54
    if note:
        s.note(note, x - 33.02, yb + 3.81)
        yb += 3.81 + 2.54 * note.count("\n")
    return yb + 10.16


_bias_r = [30]                                      # bias resistors are R30, R31, ...


def spare(s: Sheet, ref, x, y, nets, title, note=None):
    """a spare connector; each SP_ signal gets a 10k (DNI) to a bias net that a 3-pad jumper sets to 3V3 or GND"""
    yb = conn(s, ref, x, y, nets, title, note)
    sig = [n for n in nets if n.startswith("SP_")]
    bias = f"BIAS_{ref}"
    for k, net in enumerate(sig):
        rr = f"R{_bias_r[0]}"
        _bias_r[0] += 1
        r(s, rr, x + 20.32 + k * 10.16, y + 20.32, "10k DNI", net, bias, dnp=True)
    jx = x + 20.32 + len(sig) * 10.16 + 7.62
    s.part(f"JP{ref[1:]}", "Jumper:SolderJumper_3_Open", jx, y + 15.24, "pull: 1-2 up / 2-3 down", SJ3,
           in_bom=False, ref_at=(-5.08, -5.08), val_at=(-5.08, 8.89))
    s.connect(f"JP{ref[1:]}", "1", "+3V3")
    s.connect(f"JP{ref[1:]}", "3", "GND")
    s.connect(f"JP{ref[1:]}", "2", bias)
    return max(yb, y + 45.72)


def r(s, ref, x, y, value, top, bottom, dnp=False):
    s.part(ref, "Device:R", x, y, value, R_TH, dnp=dnp)
    s.connect(ref, "1", top)
    s.connect(ref, "2", bottom)


def c(s, ref, x, y, value, top, bottom, polarized=False):
    s.part(ref, "Device:C_Polarized" if polarized else "Device:C", x, y, value, CP_TH if polarized else C_TH)
    s.connect(ref, "1", top)
    s.connect(ref, "2", bottom)


def node_out(s, net, a, b):
    """a wire from a to b ending in a label (direction a -> b)"""
    s.wire(a, b)
    d = (1 if b[0] > a[0] else -1 if b[0] < a[0] else 0, 1 if b[1] > a[1] else -1 if b[1] < a[1] else 0)
    s.label(net, b, d)


def low_side(s, q, rpd, x, y, gate, drain, value="2N7000"):
    """2N7000 low-side switch, gate straight from the pin with a 100k pull-down (off at reset)"""
    s.part(q, "Transistor_FET:Q_NMOS_SGD", x, y, value, TO92, ref_at=(5.08, 2.54), val_at=(5.08, 5.08))
    s.connect(q, "2", gate)
    s.connect(q, "1", "GND")
    s.connect(q, "3", drain)
    r(s, rpd, x - 20.32, y + 2.54, "100k", gate, "GND")


def build() -> Sheet:
    _bias_r[0] = 30
    s = Sheet()
    s.note("omni-robot Teensy 4.1 carrier v1: schematic for review (not for fabrication)", 25.4, 22.86, size=3.0, bold=True)
    s.note("Rev C draft (Nov 17): all through-hole; Teensy powered from USB only (+5V = Teensy VIN = VUSB, +3V3 = its regulator, ~250 mA).\n"
           "Pins: firmware/lib/omni_fw/board_teensy41.h, changed where noted.  Connectors: JST XH 2.5 mm, pin 1 first.  One common ground in the robot.",
           25.4, 30.48, size=1.524)

    # ===== column A: the Teensy, supply, holes, two spare connectors
    s.note("U1: Teensy 4.1 (on two 1x24 sockets)", 60.96, 50.8, size=1.778, bold=True)
    s.part("U1", f"{PROJECT}:Teensy4.1", 96.52, 93.98, "Teensy 4.1", f"{PROJECT}:Teensy41_Socket",
           datasheet="https://www.pjrc.com/store/teensy41.html", ref_at=(-12.7, -33.02), val_at=(-12.7, 34.29))
    for i, name in enumerate(TEENSY_LEFT + TEENSY_RIGHT):
        s.connect("U1", str(i + 1), TEENSY_NETS[name])
    y = 152.4
    s.note("Supply: bulk caps, 3V3 LED", 50.8, y, size=1.778, bold=True)
    c(s, "C5", 55.88, y + 15.24, "10u 16V", "+5V", "GND", polarized=True)
    c(s, "C6", 68.58, y + 15.24, "10u 16V", "+3V3", "GND", polarized=True)
    s.part("#FLG01", "power:PWR_FLAG", 81.28, y + 22.86, "PWR_FLAG")
    s.connect("#FLG01", "1", "GND")
    r(s, "R15", 96.52, y + 12.7, "1k", "+3V3", "PWR_LED_A")
    s.part("D3", "Device:LED", 116.84, y + 25.4, "green 3 mm: 3V3 on", LED_TH)
    s.connect("D3", "2", "PWR_LED_A")
    s.connect("D3", "1", "GND")
    y = 200.66
    s.note("H1-H4: M3 mounting holes", 50.8, y, size=1.778, bold=True)
    for i in range(4):
        s.part(f"H{i + 1}", "Mechanical:MountingHole", 55.88 + i * 15.24, y + 10.16, "M3", MH)
    y = 228.6
    s.note("Spares: every unused header pin [catch-robot use].\n"
           "Each SP_ pin has a 10k (DNI) to its connector's bias;\n"
           "the 3-pad jumper sets the bias: 1-2 = pull-up, 2-3 = pull-down.", 50.8, y, size=1.524)
    y = spare(s, "J18", 83.82, y + 12.7, ["GND", "SP_D32", "SP_D33", "SP_D36", "SP_D37", "SP_D38"], "J18: GPIO",
              note="32, 33 [beams]; 36 [IR]; 37 [bumper]; 38 A14")
    spare(s, "J19", 83.82, y, ["+3V3", "GND", "SP_A15", "SP_D34"], "J19: analog + GPIO", note="39 = A15, 34")

    # ===== column B: CAN buses, buzzer, RGB LED, NeoPixel
    X = 210.82
    s.note("CAN1: the four AK40-10s, 1 Mbit/s", X - 45.72, 50.8, size=1.778, bold=True)
    s.note("Daisy-chained in the harness. U2's own 120R terminates this end;\n"
           "the far end of the chain needs the second 120R (harness / last motor).", X - 45.72, 57.15)
    s.part("U2", f"{PROJECT}:SN65HVD230_Module", X - 2.54, 78.74, "SN65HVD230 module", SOCK6)
    for i, net in enumerate(["+3V3", "GND", "CAN1_TX", "CAN1_RX", "CAN1_H", "CAN1_L"]):
        s.connect("U2", str(i + 1), net)
    y = conn(s, "J6", X, 91.44, ["CAN1_H", "CAN1_L"], "J6: CAN1 to the motor chain (1 H, 2 L)")
    s.note("CAN3: payload bus (reserved; fit U3 when used)", X - 45.72, y, size=1.778, bold=True)
    s.part("U3", f"{PROJECT}:SN65HVD230_Module", X - 2.54, y + 17.78, "SN65HVD230 module (optional)", SOCK6)
    for i, net in enumerate(["+3V3", "GND", "CAN3_TX", "CAN3_RX", "CAN3_H", "CAN3_L"]):
        s.connect("U3", str(i + 1), net)
    y = conn(s, "J10", X, y + 30.48, ["CAN3_H", "CAN3_L"], "J10: CAN3 (1 H, 2 L)")

    # Fernando, Nov 18: the buzzer solders on the board (12085 passive magnetic, Ø12 x 8.5 mm, 42 R, 5 V)
    s.note("Buzzer on the board (firmware tone() 2.7 kHz)", X - 45.72, y, size=1.778, bold=True)
    s.note("BZ1: 12085 passive magnetic buzzer, 42 R, 5 V (~120 mA peak)\n"
           "pin 1 = + (square pad), pin 2 slot takes 6.5 or 7.6 mm pitch", X - 45.72, y + 5.08)
    s.part("BZ1", "Device:Buzzer", X - 20.32, y + 17.78, "12085 passive 42R", f"{PROJECT}:Buzzer_D12_P6.5-7.6_Slot")
    s.connect("BZ1", "1", "+5V")
    s.connect("BZ1", "2", "BUZ_N")
    s.part("D1", "Device:D", X + 15.24, y + 22.86, "1N4148", D_TH)
    s.connect("D1", "1", "+5V")
    s.connect("D1", "2", "BUZ_N")
    low_side(s, "Q1", "R12", X + 5.08, y + 38.1, "BUZZER", "BUZ_N")
    y += 58.42
    s.note("RGB status LED (on a lead or soldered in J15): common cathode, driven straight from the Teensy", X - 45.72, y, size=1.778, bold=True)
    # Fernando, Nov 18: J15 in the LED's own leg order (common-cathode 5 mm RGB: R, cathode (longest leg), G, B), so the LED can
    # also solder straight into J15's holes (XH 2.50 mm vs the legs' 2.54: 0.12 mm over the four)
    conn(s, "J15", X - 20.32, y + 5.08, ["LED_R_OUT", "GND", "LED_G_OUT", "LED_B_OUT"],
         "J15: RGB LED (1 R, 2 GND, 3 G, 4 B)",
         note="the LED's leg order (2 = common cathode, the longest leg); it can solder in directly\n"
              "R 28, G 29, B 5 (PWM); ~4 mA each at 3.3 V")
    for k, (col, rv) in enumerate((("R", "330R"), ("G", "100R"), ("B", "100R"))):
        r(s, f"R{20 + k}", X + 20.32 + k * 12.7, y + 22.86, rv, f"LED_{col}", f"LED_{col}_OUT")
    y += 45.72
    s.note("NeoPixel strip (WS2812): pin 35 = Serial8 TX (non-blocking WS2812Serial)", X - 45.72, y, size=1.778, bold=True)
    s.note("Driven straight from the Teensy (3.3 V data; Fernando has run strips this way).\n"
           "+5V here is USB: short strips only (~10 pixels); a longer strip takes 5 V from the\n"
           "5 V converter at the strip, DIN + GND from here.", X - 45.72, y + 6.35)
    yy = y + 30.48
    r(s, "R26", X - 20.32, yy + 7.62, "330R", "NEO_3V3", "NEO_DIN")
    conn(s, "J20", X + 12.7, yy - 2.54, ["+5V", "NEO_DIN", "GND"], "J20: NeoPixel (1 +5V, 2 DIN, 3 GND)")

    # ===== column C: pack voltage, I2C power monitors, spares
    X = 340.36
    y = conn(s, "J1", X, 48.26, ["VBUS_IN", "GND"], "J1: pack voltage sense",
             note="1 pack + (after the main switch and fuse), 2 GND")
    s.note("Pack voltage (pin 41 = A17; firmware V_BUS_DIVIDER 11.0)", X - 33.02, y, size=1.778, bold=True)
    x0, y0 = X - 25.4, y + 20.32
    s.part("R1", "Device:R", x0, y0, "100k", R_TH)
    s.connect("R1", "1", "VBUS_IN")
    s.part("R2", "Device:R", x0, y0 + 7.62, "10k", R_TH)
    s.connect("R2", "2", "GND")
    s.part("C1", "Device:C", x0 + 10.16, y0 + 7.62, "100n", C_TH)
    s.connect("C1", "2", "GND")
    j, cp = (x0, y0 + 3.81), (x0 + 10.16, y0 + 3.81)
    s.wire(j, cp)
    s.junction(j)
    s.junction(cp)
    node_out(s, "VBUS_ADC", cp, (x0 + 17.78, y0 + 3.81))
    s.same_net("VBUS_ADC", ("R1", "2"), ("R2", "1"), ("C1", "1"))
    s.note("0-36 V in -> 0-3.3 V; analog, so the 21 V stop needs no I2C", x0 - 5.08, y0 + 22.86)
    y += 48.26
    s.note("Power monitors on I2C (Wire, 18 / 19), in the power stack on cables", X - 33.02, y, size=1.778, bold=True)
    y = conn(s, "J12", X, y + 5.08, ["+3V3", "GND", "I2C_SDA", "I2C_SCL"], "J12: INA3221 (arm 12 V, 19 V, 5 V)",
             note="1 VS 3V3, 2 GND, 3 SDA, 4 SCL; address 0x40")
    y = conn(s, "J21", X, y, ["+3V3", "GND", "I2C_SDA", "I2C_SCL"], "J21: INA226 (robot total current)",
             note="same bus; set its address to 0x41 (both modules default to 0x40)\n"
                  "external 50 A / 75 mV shunt; the modules' pull-ups serve the bus")
    y = spare(s, "J11", X, y, ["GND", "+3V3", "SP_D0", "SP_D1", "SP_D27", "SP_A16"], "J11: Serial1, GPIO",
              note="0 RX1, 1 TX1, 27, 40 = A16")
    spare(s, "J16", X, y, ["GND", "+3V3", "SP_D14", "SP_D15", "SP_D16", "SP_D17"], "J16: Serial3, Serial4",
          note="14 TX3, 15 RX3; 16 RX4, 17 TX4")

    # ===== column D: IMU, RC, time, spare
    X = 485.14
    y = conn(s, "J4", X, 48.26, ["+3V3", "GND"], "J4: BNO085 power (1 VCC, 2 GND)")
    y = conn(s, "J3", X, y, ["IMU_SCK", "IMU_MISO", "IMU_MOSI", "IMU_CS", "IMU_INT", "IMU_RST"], "J3: BNO085 SPI",
             note="board order: SCL=SCK, SDA=MISO, AD0=MOSI, CS, INT, RST\nPS0/PS1 tied high on the board (SPI); BOOTN not wired")
    # Fernando, Nov 18: the receiver (components/radio receiver: HPXGRC ELRS 2.4 GHz, 20 x 11 mm, pads T R V G) lives
    # on the carrier: a male pin header soldered into its four pads plugs into J5, a 1x4 2.54 mm female socket; it lies
    # flat over the board (tools/elrs_model.py). Its row read from the G end is G, V, R, T = J5's 1..4. VERIFY the pads'
    # pitch is 2.54 (the vendor's pictures read 2.3-2.45) and that they are through-holes before ordering.
    y = conn(s, "J5", X, y, ["GND", "+5V", "RC_TX", "RC_RX"], "J5: ELRS receiver (CRSF), plugged in",
             note="1x4 2.54 mm female socket; the receiver on a male header, pads G V R T = 1 2 3 4\n"
                  "3 its RX = Teensy TX2 (8), 4 its TX = Teensy RX2 (7); VERIFY its pad pitch = 2.54",
             footprint=f"{PROJECT}:ELRS_Rx_Socket_1x04_P2.54mm", value="ELRS socket 1x4")
    y = conn(s, "J13", X, y, ["GND", "CAM_TRIG", "SYNC_OUT", "CAM_STROBE"], "J13: camera trigger / time sync (reserved)")
    spare(s, "J17", X, y, ["GND", "+3V3", "SP_D2", "SP_D3", "SP_D20", "SP_D21"], "J17: PWM, Serial5",
          note="2, 3 [DShot]; 20 TX5, 21 RX5")
    return s


# ---------------------------------------------------------------------------------------------- project files
def teensy_footprint() -> str:
    pads = []
    for i in range(48):
        col, row = divmod(i, 24)
        x, y = col * 15.24, row * 2.54
        shape = "rect" if i == 0 else "oval"
        pads.append(f'  (pad "{i + 1}" thru_hole {shape} (at {x:g} {y:g}) (size 1.7 1.7) (drill 1) '
                    f'(layers "*.Cu" "*.Mask") (remove_unused_layers no) (uuid "{uid()}"))')
    x0, y0, x1, y1 = -1.27, -1.27, 16.51, 59.69          # Teensy 4.1: 17.78 x 60.96 board (0.7 x 2.4 in)
    silk = (f'  (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.12) (type solid)) (fill no) (layer "F.SilkS") (uuid "{uid()}"))\n'
            f'  (fp_rect (start 4.39 -2.5) (end 10.85 2.6) (stroke (width 0.12) (type solid)) (fill no) (layer "F.SilkS") (uuid "{uid()}"))\n'
            f'  (fp_text user "USB" (at 7.62 4.2 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (fp_rect (start {x0 - 0.25} {y0 - 1.5}) (end {x1 + 0.25} {y1 + 0.25}) (stroke (width 0.05) (type solid)) (fill no) (layer "F.CrtYd") (uuid "{uid()}"))\n'
            f'  (fp_rect (start {x0} {y0}) (end {x1} {y1}) (stroke (width 0.1) (type solid)) (fill no) (layer "F.Fab") (uuid "{uid()}"))')
    return (f'(footprint "Teensy41_Socket" (version 20241229) (generator "omni_gen_sch") (layer "F.Cu")\n'
            f'  (descr "PJRC Teensy 4.1 in two 1x24 2.54 mm female sockets, rows 15.24 mm apart (outer pins only). '
            f'Pads 1-24 left column top to bottom (USB end first), 25-48 right column")\n'
            f'  (tags "Teensy 4.1 socket")\n'
            f'  (property "Reference" "U1" (at 7.62 -4.5 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (property "Value" "Teensy41_Socket" (at 7.62 62 0) (layer "F.Fab") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f"  (attr through_hole)\n{silk}\n" + "\n".join(pads) + "\n  (embedded_fonts no)\n)\n")


def buzzer_footprint() -> str:
    """the 12 mm magnetic buzzer (12085 class, Ø12 x 8.5, 42 R passive), soldered on the board. Pin pitch varies by
    maker (6.5 or 7.6 mm), so pad 2 is a plated slot that takes either: pin 1 (+, square) at 0, pin 2 anywhere from
    6.5 to 7.6 mm. Body circle drawn for the middle of that range."""
    cx = 3.5                                              # body centre: 3.25 (6.5 pitch) .. 3.8 (7.6 pitch)
    def circ(r, layer, w):
        return (f'  (fp_circle (center {cx} 0) (end {cx + r} 0) (stroke (width {w}) (type solid)) (fill no) '
                f'(layer "{layer}") (uuid "{uid()}"))')
    return (f'(footprint "Buzzer_D12_P6.5-7.6_Slot" (version 20241229) (generator "omni_gen_sch") (layer "F.Cu")\n'
            f'  (descr "12 mm magnetic buzzer (12085 / 12095 class), through-hole; pin 2 in a plated slot for '
            f'6.5 or 7.6 mm pitch. Pin 1 = +")\n'
            f'  (tags "buzzer magnetic 12mm")\n'
            f'  (property "Reference" "BZ1" (at {cx} -7.3 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (property "Value" "Buzzer_D12" (at {cx} 7.4 0) (layer "F.Fab") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (attr through_hole)\n'
            + circ(6.15, "F.SilkS", 0.12) + "\n" + circ(6.0, "F.Fab", 0.1) + "\n" + circ(6.65, "F.CrtYd", 0.05) + "\n"
            + f'  (fp_text user "+" (at -0.2 -2.0 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1.2 1.2) (thickness 0.18))))\n'
            f'  (pad "1" thru_hole rect (at 0 0) (size 2 2) (drill 1) (layers "*.Cu" "*.Mask") '
            f'(remove_unused_layers no) (uuid "{uid()}"))\n'
            f'  (pad "2" thru_hole oval (at 7.05 0) (size 2.9 1.9) (drill oval 2.0 0.9) (layers "*.Cu" "*.Mask") '
            f'(remove_unused_layers no) (uuid "{uid()}"))\n'
            f'  (embedded_fonts no)\n)\n')


ELRS_FP = "ELRS_Rx_Socket_1x04_P2.54mm"
RX_BOX = (-18.8, 1.2, -1.69, 9.31)       # the receiver board over the footprint (x0, x1, y0, y1; tools/elrs_model.py, footprint y down)


def elrs_footprint() -> str:
    """J5: a 1x4 2.54 mm female socket (KiCad's PinSocket_1x04_P2.54mm_Vertical pads and outline) carrying the ELRS
    receiver on its male header: the receiver lies flat 11 mm over the board toward -x (its outline on silk, fab and
    one courtyard round socket + receiver, so nothing gets placed under it), and its 3D model next to the socket's."""
    x0, x1, y0, y1 = RX_BOX
    g = 0.25

    def line(a, b, layer, w):
        return (f'  (fp_line (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke (width {w}) (type solid)) '
                f'(layer "{layer}") (uuid "{uid()}"))')

    def rect(a, b, layer, w):
        return (f'  (fp_rect (start {a[0]} {a[1]}) (end {b[0]} {b[1]}) (stroke (width {w}) (type solid)) (fill no) '
                f'(layer "{layer}") (uuid "{uid()}"))')

    def model(path):
        return f'  (model "{path}" (offset (xyz 0 0 0)) (scale (xyz 1 1 1)) (rotate (xyz 0 0 0)))'

    silk = [line((-1.33, 1.27), (-1.33, 8.95), "F.SilkS", 0.12), line((-1.33, 1.27), (1.33, 1.27), "F.SilkS", 0.12),
            line((-1.33, 8.95), (1.33, 8.95), "F.SilkS", 0.12), line((0, -1.33), (1.33, -1.33), "F.SilkS", 0.12),
            line((1.33, -1.33), (1.33, 0), "F.SilkS", 0.12), line((1.33, 1.27), (1.33, 8.95), "F.SilkS", 0.12),
            rect((x0, y0), (x1, y1), "F.SilkS", 0.12), rect((x0, y0), (x1, y1), "F.Fab", 0.1),
            line((-1.27, -1.27), (0.635, -1.27), "F.Fab", 0.1), line((-1.27, 8.89), (-1.27, -1.27), "F.Fab", 0.1),
            line((0.635, -1.27), (1.27, -0.635), "F.Fab", 0.1), line((1.27, -0.635), (1.27, 8.89), "F.Fab", 0.1),
            line((1.27, 8.89), (-1.27, 8.89), "F.Fab", 0.1),
            rect((x0 - g, min(y0 - g, -1.77)), (max(x1 + g, 1.77), max(y1 + g, 9.39)), "F.CrtYd", 0.05)]
    pads = [f'  (pad "{k + 1}" thru_hole {"rect" if k == 0 else "circle"} (at 0 {k * 2.54:g}) (size 1.7 1.7) (drill 1) '
            f'(layers "*.Cu" "*.Mask") (remove_unused_layers no) (uuid "{uid()}"))' for k in range(4)]
    return (f'(footprint "{ELRS_FP}" (version 20241229) (generator "omni_gen_sch") (layer "F.Cu")\n'
            f'  (descr "1x04 2.54 mm female socket with the ELRS receiver (20 x 11 mm, HPXGRC) plugged in on a male header, '
            f'lying flat 11 mm over the board toward -x. Pads G V R T = 1 2 3 4")\n'
            f'  (tags "ELRS receiver socket 1x04 2.54mm")\n'
            f'  (property "Reference" "J5" (at 0 -2.77 0) (layer "F.SilkS") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (property "Value" "{ELRS_FP}" (at {(x0 + x1) / 2:g} {y1 + 1.2:g} 0) (layer "F.Fab") (uuid "{uid()}") (effects (font (size 1 1) (thickness 0.15))))\n'
            f'  (attr through_hole)\n'
            + "\n".join(silk) + "\n"
            + f'  (fp_text user "ELRS rx, on header" (at {(x0 + x1) / 2 - 1.5:g} {(y0 + y1) / 2:g} 0) (layer "F.SilkS") (uuid "{uid()}") '
              f'(effects (font (size 0.8 0.8) (thickness 0.12))))\n'
            + "\n".join(pads) + "\n  (embedded_fonts no)\n"
            + model("${KICAD10_3DMODEL_DIR}/Connector_PinSocket_2.54mm.3dshapes/PinSocket_1x04_P2.54mm_Vertical.step") + "\n"
            + model("${KIPRJMOD}/models/elrs_receiver_on_J5.step") + "\n)\n")


def write_project_files():
    lib = (f"(kicad_symbol_lib (version 20251024) (generator \"omni_gen_sch\") (generator_version \"10.0\")\n"
           + teensy_symbol("Teensy4.1") + "\n" + can_module_symbol("SN65HVD230_Module") + "\n)\n")
    with open(os.path.join(HERE, f"{PROJECT}.kicad_sym"), "w", encoding="utf-8", newline="\n") as f:
        f.write(lib)
    os.makedirs(os.path.join(HERE, f"{PROJECT}.pretty"), exist_ok=True)
    with open(os.path.join(HERE, f"{PROJECT}.pretty", "Teensy41_Socket.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(teensy_footprint())
    with open(os.path.join(HERE, f"{PROJECT}.pretty", "Buzzer_D12_P6.5-7.6_Slot.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(buzzer_footprint())
    with open(os.path.join(HERE, f"{PROJECT}.pretty", f"{ELRS_FP}.kicad_mod"), "w", encoding="utf-8", newline="\n") as f:
        f.write(elrs_footprint())
    with open(os.path.join(HERE, "sym-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write(f'(sym_lib_table\n  (version 7)\n  (lib (name "{PROJECT}")(type "KiCad")(uri "${{KIPRJMOD}}/{PROJECT}.kicad_sym")(options "")(descr "omni carrier parts"))\n)\n')
    with open(os.path.join(HERE, "fp-lib-table"), "w", encoding="utf-8", newline="\n") as f:
        f.write(f'(fp_lib_table\n  (version 7)\n  (lib (name "{PROJECT}")(type "KiCad")(uri "${{KIPRJMOD}}/{PROJECT}.pretty")(options "")(descr "omni carrier footprints"))\n)\n')
    pro = os.path.join(HERE, f"{PROJECT}.kicad_pro")
    if not os.path.exists(pro):
        with open(pro, "w", encoding="utf-8", newline="\n") as f:
            json.dump({"meta": {"filename": f"{PROJECT}.kicad_pro", "version": 3},
                       "sheets": [[ROOT_UUID, "Root"]]}, f, indent=2)


def main():
    s = build()
    s.write(os.path.join(HERE, f"{PROJECT}.kicad_sch"), "omni-robot Teensy 4.1 carrier", "v1 draft", "2026-11-17",
            ["Rev C draft: through-hole, for review", "Generated by tools/gen_sch.py: edit the script, not the sheet",
             "Pins: firmware/lib/omni_fw/board_teensy41.h", "OSH Park 2-layer target"])
    write_project_files()
    nets = {k: sorted(f"{r}.{p}" for r, p in v if not r.startswith("#")) for k, v in s.nets.items()}
    with open(os.path.join(HERE, "nets.json"), "w", encoding="utf-8", newline="\n") as f:
        json.dump(nets, f, indent=1, sort_keys=True)
    print(f"{len(s.parts)} symbols, {len(nets)} nets -> {PROJECT}.kicad_sch")


if __name__ == "__main__":
    main()

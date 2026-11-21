"""Check the schematic: KiCad ERC, and KiCad's own netlist against the intended connectivity (nets.json).

    python tools/check.py            (from pcb/v1, after tools/gen_sch.py)

Also checks every Teensy pin against firmware/lib/omni_fw/board_teensy41.h (the pin -> function the firmware uses),
and exports out/omni_carrier_schematic.pdf for review.
"""
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(HERE))
CLI = os.environ.get("KICAD_CLI", r"C:\Program Files\KiCad\10.0\bin\kicad-cli.exe")
SCH = os.path.join(HERE, "omni_carrier.kicad_sch")
OUT = os.path.join(HERE, "out")

# firmware pin name -> the net this board must give it
FW_TO_NET = {
    "CAN1_TX": "CAN1_TX", "CAN1_RX": "CAN1_RX", "CAN3_TX": "CAN3_TX", "CAN3_RX": "CAN3_RX",
    "BNO_MOSI": "IMU_MOSI", "BNO_MISO": "IMU_MISO", "BNO_SCK": "IMU_SCK", "BNO_CS": "IMU_CS", "BNO_INT": "IMU_INT",
    "BNO_RST": "IMU_RST",
    "CAM_TRIG": "CAM_TRIG", "SYNC_OUT": "SYNC_OUT", "CAM_STROBE": "CAM_STROBE", "RC_RX": "RC_RX", "RC_TX": "RC_TX",
    "ROBOCLAW_RX": "SP_D15", "ROBOCLAW_TX": "SP_D14", "SERVO_BUS": "SP_D17", "DSHOT_L": "SP_D2", "DSHOT_R": "SP_D3",
    "IR_EMIT": "SP_D36", "BEAM_CATCH": "SP_D33", "BEAM_FEED_EXIT": "SP_D32",
    "BUMPER_FRONT": "SP_D37", "BUMPER_REAR": "SP_D38", "BUZZER": "BUZZER",
}
# pins this board deliberately changes (gen_sch.FIRMWARE_TO_CHANGE): reported, not failed


def run(*args):
    r = subprocess.run([CLI, *args], capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"kicad-cli {' '.join(args)} failed: {r.stdout}{r.stderr}")
    return r.stdout


def main():
    os.makedirs(OUT, exist_ok=True)
    fails = []
    # 1. ERC
    rpt = os.path.join(OUT, "erc.rpt")
    run("sch", "erc", "--severity-all", "--format", "report", "-o", rpt, SCH)
    erc = open(rpt, encoding="utf-8").read()
    m = re.search(r"ERC messages: (\d+)\s+Errors (\d+)\s+Warnings (\d+)", erc)
    print(f"ERC: {m.group(0)}")
    if m.group(1) != "0":
        fails.append("ERC violations (out/erc.rpt)")
    # 2. KiCad's netlist vs the intended nets
    net_xml = os.path.join(OUT, "omni_carrier.net.xml")
    run("sch", "export", "netlist", "--format", "kicadxml", "-o", net_xml, SCH)
    root = ET.parse(net_xml).getroot()
    kicad = {}
    for net in root.iter("net"):
        name = net.get("name").lstrip("/")
        nodes = sorted(f"{n.get('ref')}.{n.get('pin')}" for n in net.iter("node") if not n.get("ref").startswith("#"))
        if nodes:
            kicad[name] = nodes
    want = json.load(open(os.path.join(HERE, "nets.json"), encoding="utf-8"))
    by_members = {tuple(v): k for k, v in kicad.items()}
    for name, members in want.items():
        got = kicad.get(name)
        if got != members:
            fails.append(f"net {name}: want {members}, KiCad has {got} (same members under {by_members.get(tuple(members))})")
    extra = set(kicad) - set(want)
    for name in sorted(extra):
        fails.append(f"KiCad net not intended: {name} {kicad[name]}")
    singles = [k for k, v in kicad.items() if len(v) < 2]
    print(f"netlist: {len(kicad)} nets, {sum(map(len, kicad.values()))} pin connections; intended {len(want)}")
    if singles:
        fails.append(f"nets with one pin: {singles}")
    # 3. the Teensy pins against the firmware's pin map
    hdr = open(os.path.join(REPO, "firmware", "lib", "omni_fw", "board_teensy41.h"), encoding="utf-8").read()
    pins = {}
    for line in re.findall(r"constexpr uint8_t ([^;]+);", hdr.split("namespace pin {")[1].split("}")[0]):
        for name, num in re.findall(r"(\w+) = (\d+)", line):
            pins[name] = int(num)
    teensy_pad = {}
    for node_net, members in kicad.items():
        for mbr in members:
            ref, pad = mbr.split(".")
            if ref == "U1":
                teensy_pad[int(pad)] = node_net
    sys.path.insert(0, os.path.join(HERE, "tools"))
    import gen_sch
    names = gen_sch.TEENSY_LEFT + gen_sch.TEENSY_RIGHT
    pin_net = {}
    for pad, nm in enumerate(names, 1):
        num = nm.split("/")[0]
        if num.isdigit():
            pin_net[int(num)] = teensy_pad.get(pad)
    for fw, net in FW_TO_NET.items():
        if pin_net.get(pins[fw]) != net:
            fails.append(f"firmware {fw} = pin {pins[fw]}: board net {pin_net.get(pins[fw])}, expected {net}")
    print(f"firmware pin map: {len(FW_TO_NET)} signals checked; Teensy pins on the board: {len(pin_net)} of 42")
    covered = set(FW_TO_NET) | set(gen_sch.FIRMWARE_TO_CHANGE)
    for fw in pins:
        if fw not in covered:
            fails.append(f"firmware pin {fw} is neither checked nor listed in FIRMWARE_TO_CHANGE")
    print("firmware to change for this board:")
    for fw, why in gen_sch.FIRMWARE_TO_CHANGE.items():
        print(f"  {fw} (pin {pins[fw]}): {why}")
    if sorted(pin_net) != list(range(42)) or any(v is None for v in pin_net.values()):
        fails.append("not every Teensy header pin 0-41 is connected")
    # 4. PDF for review
    pdf = os.path.join(OUT, "omni_carrier_schematic.pdf")
    run("sch", "export", "pdf", "-o", pdf, SCH)
    print(f"PDF: {os.path.relpath(pdf, HERE)}")
    if fails:
        print("FAIL:\n  " + "\n  ".join(fails))
        sys.exit(1)
    print("all checks pass")


if __name__ == "__main__":
    main()

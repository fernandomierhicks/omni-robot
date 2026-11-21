"""One review STEP of v15's electrics and harness in the robot (Nov 18 2026): step_export/electrical/electrical_review.step.

    python -B electrical_step.py        (omni-robot-sim interpreter; after harness_route.py, which writes validation/harness/routes.json)

What it holds (each body coloured): the first floor (deck pieces, fins, pedestal, the battery and its clips), the pods at nominal, the
second floor (its quadrants, the bin deck, posts, face frame), the computer, every electrical.py block and cradle, the carrier (pcb/v1's
coloured STEP, Nov 19: with the Teensy 4.1, the two CAN modules, the buzzer and the ELRS receiver on it, in its own face colours), the IMU, the lidar, and the harness: every routed lead
(harness.build: tubes along the routes, bends at their radii) and every plug. The SO-101 is left out (its sweep is in the checks).
Purchased parts are envelopes unless a model is named; nothing here is bought, printed or wired.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import electrical as EL                                                # noqa: E402
import harness as HN                                                   # noqa: E402
import upper as U                                                      # noqa: E402
import wheel_motor as W                                                # noqa: E402
import world as WD                                                     # noqa: E402
from step_colour import faces_in_order, read_coloured, write_step      # noqa: E402

OUT = W.ROOT / 'step_export' / 'electrical'
VAL = W.ROOT / 'validation' / 'electrical'
C = dict(power=(0.95, 0.55, 0.10), board=(0.36, 0.18, 0.56), signal=(0.15, 0.15, 0.15), usb=(0.10, 0.35, 0.80), dev=(0.55, 0.57, 0.62),
         cradle=(0.92, 0.92, 0.90), context=(0.80, 0.80, 0.82), fin=(0.33, 0.33, 0.36), plug=(0.95, 0.85, 0.20), comp=(0.25, 0.30, 0.55))
POWER_LEADS = ('pair12', 'pair18', 'pair20', 'motor', 'bat')
USB_LEADS = ('usb2', 'usb3', 'usbc', 'hdmi')
BOARD = (None, ())                                                     # (shape, face colours) of the carrier, read in main()


def colour(k, lead=None):
    if k.startswith('lead_'):
        return C['power'] if lead in POWER_LEADS else C['usb'] if lead in USB_LEADS else C['signal']
    if k.startswith('plug_'):
        return C['plug']
    if k.startswith(('bracket_', 'tray_')):
        return C['cradle']
    if k == 'carrier_pcb':                                             # Nov 19: pcb/v1's own face colours (carrier_coloured)
        return faces_in_order(BOARD[1])
    if k.startswith('support'):
        return C['fin']
    if k == 'computer':
        return C['comp']
    if k in EL.BLOCKS or k.startswith(('imu', 'lidar')):
        return C['dev']
    return C['context']


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    VAL.mkdir(parents=True, exist_ok=True)
    printed, bought, vendor, hw, _ = U.build()
    items = {}
    base = WD.base_items()
    for k, s in base.items():
        if k.startswith(('deck_', 'support', 'pedestal', 'bracket_battery', 'battery', 'strap', 'env_lidar')):
            items[k] = s
    for k, s in WD.pods(0.0).items():
        if k.split('/')[1].startswith(('pod_fork', 'rx_', 'wheel_hub', 'motor', 'roller_tread')):
            items[f'pod_{k.replace("/", "_")}'] = s
    for k, s in printed.items():
        if k.startswith(('floor', 'deck_bin', 'post_deck', 'housing_face', 'bracket_', 'tray_', 'housing_switch', 'riser_arm')):
            items[k] = s
    for k, s in bought.items():
        if k == 'computer' or k in EL.BLOCKS or k.startswith(('plug_switch_main', 'plug_d435', 'd435')):
            items[k] = s
    global BOARD
    path, T = vendor.pop('carrier_pcb')
    BOARD = read_coloured(path)
    items['carrier_pcb'] = BOARD[0].moved(U.mat_loc(T))
    items.update(U.load_vendor({k: v for k, v in vendor.items() if k in ('imu_bno085', 'display',
                                                                       'switch_main')}))
    hitems, info, _ = HN.build()
    runs = json.loads((HN.VAL / 'routes.json').read_text())['runs']
    lead_of = {f'lead_{k}': r['lead'] for k, r in runs.items() if r.get('reachable')}
    items.update(hitems)
    write_step({k: (s, colour(k, lead_of.get(k))) for k, s in items.items()}, OUT / 'electrical_review.step', 'electrical_review')
    (VAL / 'electrical_step.json').write_text(json.dumps(dict(bodies=len(items), leads=len(lead_of), blocks=EL.report()), indent=1), encoding='utf-8')
    print('wrote', OUT / 'electrical_review.step', len(items), 'bodies,', len(lead_of), 'leads')


if __name__ == '__main__':
    main()

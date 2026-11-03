"""Nov 2 2026 level fork pivot: move the pod's hardware in reference/assembly_manifest.json (the pod builders read it).

    python 19_level_pivot_manifest.py          (any Python; refuses to run twice)

pivot (68.2, 43) -> (80.6, 0); travel limits from the -3 / +15 mm stops on the new arc; the shaft, its spacers and 627s, the
shocks and their pins, the cross bar and its screws, the pivot side's guard foot screws (y 94.5, the inner one at x 30) move.
The shaft pin becomes an M3 x 16 seated on the inner ear's ledge (z 14), threading above the shaft (was an M3 x 25 from z 57).
"""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / 'reference' / 'assembly_manifest.json'
PIVOT = (80.6, 0.0)
TRAVEL = (-3, 15)
# key -> new (x, y, z) of the part's origin
MOVE = {'Pivot_shaft': (3, 80.6, 0), 'Inner_pivot_spacer': (44, 80.6, 0), 'Outer_pivot_spacer': (-36.5, 80.6, 0),
        'bearing_627_1': (48, 80.6, 0), 'bearing_627_2': (-42.5, 80.6, 0),
        'Fork_bar': (5, 78, -23), 'bolt_M3_18_PAN_13': (-43.5, 83, -17), 'bolt_M3_18_PAN_14': (-43.5, 83, -29),
        'bolt_M3_18_PAN_15': (51, 83, -17), 'bolt_M3_18_PAN_16': (51, 83, -29),
        'washer_M3_1': (66, 37, 11), 'bolt_M3_14_PAN_1': (66.5, 37, 11), 'bolt_M3_16_PAN_1': (-57.5, 37, 11),
        'bolt_M3_20_PAN_1': (60, 37, 59.2), 'nut_M3_nyloc_1': (75.5, 37, 59.2), 'bolt_M3_16_PAN_2': (-51.5, 37, 59.2),
        'Inner_shock_case': (63, 37, 11), 'Inner_shock_rod': (63, 37, 59.2), 'Outer_shock_case': (-54.5, 37, 11), 'Outer_shock_rod': (-54.5, 37, 59.2),
        'Guard_foot_bolt_1': (-14, 94.5, -0.1), 'Guard_foot_bolt_2': (16, 94.5, -0.1), 'Guard_foot_bolt_5': (30, 94.5, -0.1), 'Guard_foot_bolt_6': (-31.5, 94.5, -0.1)}
PIN_OLD, PIN_NEW, PIN_AT = 'bolt_M3_25_PAN_5', ('bolt_M3_16_PAN_3', 'bolt_M3_16_PAN'), (40.25, 80.6, 14)


def main():
    man = json.loads(PATH.read_text())
    if man['pivot'] == list(PIVOT):
        raise SystemExit('manifest already has the level pivot')
    rows = {r['key']: r for r in man['parts']}
    for key, xyz in MOVE.items():
        for i in range(3):
            rows[key]['matrix'][i][3] = float(xyz[i])
    pin = rows[PIN_OLD]
    pin['key'], pin['part'] = PIN_NEW
    for i in range(3):
        pin['matrix'][i][3] = float(PIN_AT[i])
    rho = math.hypot(*PIVOT)
    man['pivot'] = list(PIVOT)
    man['limits_deg'] = [90 + math.degrees(math.asin((b - PIVOT[1]) / rho) + math.atan2(PIVOT[1], PIVOT[0])) for b in TRAVEL]
    PATH.write_text(json.dumps(man, indent=1))
    print('pivot', man['pivot'], 'limits', man['limits_deg'], 'moved', len(MOVE) + 1)


if __name__ == '__main__':
    main()

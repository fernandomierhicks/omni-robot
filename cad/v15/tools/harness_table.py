"""The harness as a table, from validation/harness/routes.json: every lead, from where to where, through where, how long (Nov 18).

    python -B harness_table.py        (omni-robot-sim interpreter; after harness_route.py) -> validation/harness/harness_table.md

"Through": the floors the lead runs on (first floor = the deck top, under the lidar's band; second floor = on the plate) and, where it
changes floor, the slot it climbs (R / L: the second floor's cable slots in the side fins' lidar shadows; every crossing is pinned there).
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import harness as HN                                                   # noqa: E402

Z_SPLIT = 170.0                                                        # under: the first floor (deck top to the floor plate, in the lidar layer); over: the second


def describe(run):
    zs = [p[2] for p in run['waypoints']]
    lo, hi = min(zs) < Z_SPLIT, max(zs) > Z_SPLIT
    slot = run.get('scan_band', {}).get('slot')
    if lo and hi:
        return f'floor 1 -> slot {slot} (in the side fin\'s shadow) -> floor 2' if slot else 'floor 1 -> floor 2 (NOT in a slot)'
    return 'floor 1' if lo else 'floor 2'


def main():
    runs = json.loads((HN.VAL / 'routes.json').read_text())['runs']
    P = HN.ports()
    rows, cross = [], {'R': [], 'L': []}
    for c in HN.CONNECTIONS:
        r = runs.get(c['id']) if c.get('route') else None
        pa, pb = P.get(c['a']), P.get(c['b'])
        frm = f"{c['a']} ({pa['dev']})" if pa else c['a']
        to = f"{c['b']} ({pb['dev']})" if pb else (c['b'] or '')
        if r and r.get('reachable'):
            thru = describe(r)
            ln = r['length_mm']
            slot = r.get('scan_band', {}).get('slot')
            if slot:
                cross[slot].append(c['id'])
        else:
            thru = 'not routed: ' + (c.get('survive') or c.get('conn') or '') if not c.get('route') else 'NO ROUTE'
            ln = ''
        lead = HN.LEADS[c['lead']][2] if c.get('lead') else ''
        rows.append(f"| {c['id']} | {frm} | {to} | {thru} | {ln} | {c['sig']} | {lead} |")
    out = ['| Lead | From | To | Through | mm | Carries | Cable |', '| --- | --- | --- | --- | --- | --- | --- |'] + rows
    out += ['', f"Lidar-plane crossings: {len(cross['R']) + len(cross['L'])}, all in the side fins' shadows: slot R {', '.join(cross['R'])}; "
                f"slot L {', '.join(cross['L'])}."]
    (HN.VAL / 'harness_table.md').write_text('\n'.join(out) + '\n', encoding='utf-8')
    print('\n'.join(out))


if __name__ == '__main__':
    main()

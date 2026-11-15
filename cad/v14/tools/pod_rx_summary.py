"""Side-by-side summary of cad/v14/Pod_reimagined_1..3 checks.json (Nov 13 2026).   python -B pod_rx_summary.py"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
rows = []
for i in (1, 2, 3):
    f = ROOT / f'Pod_reimagined_{i}' / 'checks.json'
    if not f.exists():
        print(f'Pod_reimagined_{i}: no checks.json')
        continue
    r = json.loads(f.read_text(encoding='utf-8'))
    st = [k for k in r['nominal']['gaps_mm'] if k.endswith('_to_fork') and k.startswith('rx_')]
    sw = [k for k in r['nominal']['gaps_mm'] if k.endswith('_to_wheel_envelope') and k.startswith('rx_')]
    sh = [k for k in r['nominal']['gaps_mm'] if k.endswith('_to_harness') and k.startswith('rx_')]
    pp = r['printed_parts']
    env = r['pod_envelope_swept']
    rows.append(dict(
        name=f'Pod_reimagined_{i}',
        interference={p: r[p]['interference_mm3'] for p in ('droop', 'nominal', 'bump')},
        static_to_fork={p: min(r[p]['gaps_mm'][k] for k in st) for p in ('droop', 'nominal', 'bump')},
        static_to_wheel={p: min(r[p]['gaps_mm'][k] for k in sw) for p in ('droop', 'nominal', 'bump')},
        static_to_harness=min(r[p]['gaps_mm'][k] for k in sh for p in ('droop', 'nominal', 'bump')),
        extra={p: {k: v for k, v in r[p]['gaps_mm'].items() if not k.startswith('rx_')} for p in ('droop', 'nominal', 'bump')},
        fork_to_wheel=r['fork_to_wheel_envelope_mm'], fork_to_harness=r['fork_to_harness_mm'],
        springs=r['springs'], envelope=env,
        printed={k: (v['volume_cm3'], v['overhang_mm2'], v['bed']['height'], v['bed']['footprint'], v['solids'], v['valid']) for k, v in pp.items()},
        printed_total_cm3=round(sum(v['volume_cm3'] for v in pp.values()), 1),
        n_printed=len(pp), n_screws=len(r['hole_audit']['thread']),
        audit={k: r['hole_audit'][k] for k in ('partial', 'across_layers', 'head', 'no_hole')}, blind=list(r['blind_holes']),
        loads={k: r['hand_loads'][k] for k in ('static', 'bump', 'side_hit')}))
print(json.dumps(rows, indent=1))

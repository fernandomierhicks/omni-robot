"""Spatial clearance using the actual exported design-intent wheel parts."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import wheel as W

host=b.Compound([b.import_step(ROOT/f'validation/human/wheel_hub_{k}.step') for k in 'ABC']+[b.import_step(ROOT/'validation/human/wheel_cartridge.step')])
core=b.import_step(ROOT/'validation/human/roller_core.step')
body=core & b.Box(100,100,2*W.HL)
tread=b.import_step(ROOT/'validation/human/roller_tread.step')
rows=[]
for row in (0,1):
    for shift in (-W.END_FLOAT/2,0,W.END_FLOAT/2):
        for j,loc in enumerate(W.roller_locs(row)):
            at=loc*b.Pos(0,0,shift)
            distances={name:shape.moved(at).distance_to(host) for name,shape in [('core_body',body),('TPU',tread)]}
            rows.append({'row':row+1,'roller':j+1,'shift_mm':shift,'clearance_mm':distances,'pass':min(distances.values())>=W.MIN_BODY_CLEARANCE-1e-6})
report={'required_clearance_mm':W.MIN_BODY_CLEARANCE,'samples':rows,'pass':all(r['pass'] for r in rows)}
(ROOT/'validation/human/wheel_clearance.json').write_text(json.dumps(report,indent=2))
print('True-arc native wheel clearance',min(min(r['clearance_mm'].values()) for r in rows),'PASS' if report['pass'] else 'FAIL',flush=True)
if not report['pass']:raise SystemExit(1)

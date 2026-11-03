"""Validate the exported native hubs against the roller clearance contract."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import wheel as W

def main():
    missing=[k for k in 'ABC' if not (ROOT/f'validation/V10_wheel_hub_{k}.step').exists()]
    if missing:raise RuntimeError('Native hubs not finished: '+str(missing))
    hubs={k:b.import_step(ROOT/f'validation/V10_wheel_hub_{k}.step') for k in 'ABC'}
    hubs['cartridge']=b.import_step(ROOT/'validation/V10_wheel_cartridge.step')
    host=b.Compound(list(hubs.values()));rows=[]
    body=W._revolve(W._profile(core=True));tread=W._revolve(W._profile(tread=True))
    for row in (0,1):
        for shift in (-W.END_FLOAT/2,0,W.END_FLOAT/2):
            for j,loc in enumerate(W.roller_locs(row)):
                at=loc*b.Pos(0,0,shift)
                values={name:shape.moved(at).distance_to(host) for name,shape in [('body',body),('TPU',tread)]}
                rows.append({'row':row+1,'roller':j+1,'shift_mm':shift,'clearance_mm':values,'pass':min(values.values())>=W.MIN_BODY_CLEARANCE-1e-6})
    data={'required_clearance_mm':W.MIN_BODY_CLEARANCE,'samples':rows,'pass':all(r['pass'] for r in rows)}
    (ROOT/'validation/native_wheel_clearance.json').write_text(json.dumps(data,indent=2))
    print('Native wheel clearance',min(min(r['clearance_mm'].values()) for r in rows),'PASS' if data['pass'] else 'FAIL')
    if not data['pass']:raise SystemExit(1)
if __name__=='__main__':main()

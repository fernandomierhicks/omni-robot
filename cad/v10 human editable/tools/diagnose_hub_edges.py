import faulthandler
faulthandler.dump_traceback_later(25,repeat=True)
from sw_native import *
import numpy as np
print('connecting',flush=True)
sw=connect();p=ROOT/'validation/INCOMPLETE_wheel_hub_A.SLDPRT'
print('opening',flush=True)
raw,_,_=sw.OpenDoc6(str(p),1,1,'',0,0);m=wrap(raw,sld.IModelDoc2)
d=json.loads((ROOT/'validation/RESUME_wheel_hub_A.json').read_text());n=next(n for n in reversed(d['nodes']) if n['op']=='fillet')
print('reading edges',flush=True)
all_edges=[e for b in bodies(m) for e in edges(b)]
print('edges',len(all_edges),flush=True)
curves=[(e,curve(e)) for e in all_edges]
lines=[]
for e,cv in curves:
    if cv.IsLine():
        lp=cv.LineParams;lines.append((e,np.array(lp[:3]),np.array(lp[3:])))
print('lines',len(lines),flush=True)
for sig in n['edges']:
    pts=np.array(sig['points'])*.001
    scores=[]
    for e,origin,direction in lines:
        delta=pts-origin
        score=max(np.linalg.norm(delta-np.outer(delta@direction,direction),axis=1))
        scores.append((score,e))
    score,e=min(scores,key=lambda v:v[0]);cv=wrap(e.GetCurve(),sld.ICurve)
    print('sig',sig['points'][1],'miss_mm',score*1000,'closest',[list(np.array(cv.GetClosestPointOn(*p)[:3])*1000) for p in pts],flush=True)
print('volume',sum(b.GetMassProperties(1)[3]*1e9 for b in bodies(m)))
print(m.Extension.SaveAs3(str(ROOT/'validation/hub_A_repaired_prefillet.step'),0,3,None,None,0,0))

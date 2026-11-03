from swhelpers import *
from pathlib import Path
import json
ROOT=Path(__file__).resolve().parents[2]
sw=connect();path=ROOT/'V10_hubA_human.SLDPRT'
raw,e,w=sw.OpenDoc6(str(path),1,3,'',0,0)
if raw is None:raise RuntimeError('Example open failed '+str((e,w)))
m=wrap(raw,sld.IModelDoc2);eq=wrap(m.GetEquationMgr(),sld.IEquationMgr)
rows=[]
for f in features(m):
    row={'name':f.Name,'type':f.GetTypeName2(),'errors':f.GetErrorCode2()[0],'dimensions':feature_own_dims(f)}
    sub=wrap(f.GetFirstSubFeature(),sld.IFeature);sketches=[]
    while sub:
        if sub.GetTypeName2()=='ProfileFeature':sketches.append({'name':sub.Name,'status':wrap(sub.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus()})
        sub=wrap(sub.GetNextSubFeature(),sld.IFeature)
    if sketches:row['sketches']=sketches
    rows.append(row)
report={'path':str(path),'title':m.GetTitle(),'volume_mm3':volume_mm3(m),'bodies':[b.Check2() for b in bodies(m)],
        'equations':[eq.Equation(i) for i in range(eq.GetCount())],'features':rows}
(ROOT/'validation/human_example_inspection.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))

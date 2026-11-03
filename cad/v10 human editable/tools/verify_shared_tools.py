"""Verify every linked native tool retained its exact nominal part placement."""
from sw_native import *
from update_native import open_part
import numpy as np

def position(m,item):
    part=wrap(m,sld.IPartDoc);f=wrap(part.FeatureByName(item['translation']),sld.IFeature)
    fd=wrap(f.GetDefinition(),sld.IMoveCopyBodyFeatureData)
    t=np.array([fd.TransformX,fd.TransformY,fd.TransformZ])*1000
    if item['kind']=='circle':
        sk=wrap(wrap(part.FeatureByName(item['sketch']),sld.IFeature).GetSpecificFeature2(),sld.ISketch)
        arc=wrap(sk.GetSketchSegments()[0],sld.ISketchArc);p=wrap(arc.GetCenterPoint2(),sld.ISketchPoint)
        return t+np.array(item['R'])@np.array([p.X*1000,p.Y*1000,0])
    frame=wrap(part.FeatureByName('FRAME_'+item['translation']),sld.IFeature)
    fd=wrap(frame.GetDefinition(),sld.IMoveCopyBodyFeatureData)
    return t+np.array([fd.TransformX,fd.TransformY,fd.TransformZ])*1000

if __name__=='__main__':
    sw=connect();items=json.loads((ROOT/'reference/shared_hole_candidates.json').read_text())['candidates'];rows=[]
    for name in sorted({r['part'] for r in items}):
        m=open_part(sw,name)
        for item in (r for r in items if r['part']==name):
            actual=position(m,item);expected=np.array(item['t']);error=float(max(abs(actual-expected)))
            rows.append({'part':name,'feature':item['feature'],'pattern':item['pattern'],'max_position_error_mm':error,'pass':error<1e-6})
        print(name,'max placement error',max(r['max_position_error_mm'] for r in rows if r['part']==name),flush=True)
        sw.CloseDoc(m.GetTitle())
    (ROOT/'validation/shared_tool_placement.json').write_text(json.dumps({'tools':rows,'pass':all(r['pass'] for r in rows)},indent=2))
    if any(not r['pass'] for r in rows):raise SystemExit(1)

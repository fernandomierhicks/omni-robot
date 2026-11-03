from sw_native import *
from update_native import open_part
sw=connect();m=open_part(sw,'bench_plate');master=open_part(sw,'master_layout');sw.ActivateDoc3(m.GetTitle(),False,0,0)
p=wrap(m,sld.IPartDoc);name='MASTER_Cradle_to_fork-LINK_Master_layout';f=wrap(p.FeatureByName(name),sld.IFeature)
print('before',wrap(f.GetSpecificFeature2(),sld.ISketch).GetConstrainedStatus(),flush=True)
print('rollback',m.FeatureManager.EditRollback(4,'LINK_Master_layout'),flush=True)
m.ClearSelection2(True);f.Select2(False,0);m.SketchManager.InsertSketch(True)
if Path(wrap(sw.ActiveDoc,sld.IModelDoc2).GetPathName())!=ROOT/'parts/V10_bench_plate.SLDPRT':raise RuntimeError('Unexpected document switch')
sk=wrap(m.SketchManager.ActiveSketch,sld.ISketch);print('active',sk is not None,flush=True)
if sk:
    segs=sk.GetSketchSegments();arc=wrap(segs[-1],sld.ISketchArc);pt=wrap(arc.GetCenterPoint2(),sld.ISketchPoint)
    print('anchor',pt.X,pt.Y,'segments',len(segs),flush=True)
    m.ClearSelection2(True);pt.Select4(False,None);m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
    m.SketchAddConstraints('sgCOINCIDENT');print('after',sk.GetConstrainedStatus(),flush=True)
    m.SketchManager.InsertSketch(True)
m.FeatureManager.EditRollback(1,'');rebuild(m)
def report():
    sk=wrap(f.GetSpecificFeature2(),sld.ISketch);pt=wrap(wrap(sk.GetSketchSegments()[1],sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint)
    print('status/point',sk.GetConstrainedStatus(),pt.X*1000,pt.Y*1000,flush=True)
report();d=wrap(master.Parameter('D4@MASTER_Cradle_to_fork'),sld.IDimension);v=d.SystemValue
try:
    d.SetSystemValue3(v+.001,1,None);rebuild(master);master.Save3(1,0,0);rebuild(m);report()
finally:
    d.SetSystemValue3(v,1,None);rebuild(master);master.Save3(1,0,0);rebuild(m);report()
sw.CloseDoc(m.GetTitle())

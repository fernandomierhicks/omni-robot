from sw_native import *
sw=connect();p=NativePart(sw,'derived_probe');p.extrude({'kind':'circle','radius':3},0,5,'Base')
m=p.model;part=wrap(m,sld.IPartDoc)
f=wrap(part.InsertPart3(str(ROOT/'parts/V10_master_layout.SLDPRT'),64|1024,''),sld.IFeature)
print('DERIVED',f)
if f:
    f.Name='LINK_Master_layout'
    print('reorder',m.Extension.ReorderFeature(f.Name,'Base_Sketch',2))
    def walk(g,sub=False,depth=0):
        while g:
            print(' '*depth,g.Name,g.GetTypeName2())
            walk(wrap(g.GetFirstSubFeature(),sld.IFeature),True,depth+1)
            g=wrap(g.GetNextSubFeature() if sub else g.GetNextFeature(),sld.IFeature)
    walk(wrap(m.FirstFeature(),sld.IFeature))
    master=wrap(part.FeatureByName('MASTER_Cradle_to_fork-LINK_Master_layout'),sld.IFeature)
    ms=wrap(master.GetSpecificFeature2(),sld.ISketch)
    ref=wrap(ms.GetSketchSegments()[1],sld.ISketchSegment)
    base=wrap(part.FeatureByName('Base_Sketch'),sld.IFeature);m.ClearSelection2(True);base.Select2(False,0);m.SketchManager.InsertSketch(True)
    sk=wrap(m.SketchManager.ActiveSketch,sld.ISketch);rm=wrap(sk.RelationManager,sld.ISketchRelationManager)
    for r in rm.GetRelations(0) or []:
        r=wrap(r,sld.ISketchRelation);print('relation',r.GetRelationType())
        if r.GetRelationType()==9:print('delete',rm.DeleteRelation(r))
    own=wrap(sk.GetSketchSegments()[0],sld.ISketchSegment)
    m.ClearSelection2(True);own.Select4(False,None);print('select reference',ref.Select4(True,None));m.SketchAddConstraints('sgCONCENTRIC')
    print('status',sk.GetConstrainedStatus())
    center=wrap(wrap(own,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint);print('CENTER',center.X,center.Y)
    m.SketchManager.InsertSketch(True);m.EditRebuild3();print('VOLUME',bodies(m)[0].GetMassProperties(1)[3]*1e9)

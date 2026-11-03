from sw_native import *
sw=connect();p=NativePart(sw,'polygon_probe');m=p.model;sm=m.SketchManager
m.Extension.SelectByID2('Front Plane','PLANE',0,0,0,False,0,None,0);sm.InsertSketch(True)
sm.AddToDB=True;sm.CreateCornerRectangle(-.01,-.01,0,.01,.01,0);sm.AddToDB=False
m.ClearSelection2(True);print(m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,False,0,None,0))
sel=wrap(m.SelectionManager,sld.ISelectionMgr);ori=sel.GetSelectedObject6(1,-1);print('origin',ori)
m.ClearSelection2(True);print('define',sm.FullyDefineSketch(True,True,1023,True,1,ori,1,ori,1,1))
sk=wrap(sm.ActiveSketch,sld.ISketch);print('status',sk.GetConstrainedStatus());sm.InsertSketch(True)
m.ClearSelection2(True);last(m).Select2(False,0);sm.InsertSketch(True);sk=wrap(sm.ActiveSketch,sld.ISketch)
points=[wrap(x,sld.ISketchPoint) for x in sk.GetSketchPoints2()];print('points',[(q.X,q.Y) for q in points])
for q in points:
    if abs(q.X)+abs(q.Y)<1e-8: continue
    for xy in (0,1):
        m.ClearSelection2(True);q.Select4(False,None);m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
        dd=(m.AddHorizontalDimension2 if xy==0 else m.AddVerticalDimension2)(q.X+.003,q.Y+.003,0)
        print('dim',xy,dd)
print('explicit status',sk.GetConstrainedStatus());sm.InsertSketch(True)

from sw_native import *
sw=connect(); p=NativePart(sw,'constraints_probe');m=p.model;sm=m.SketchManager
m.Extension.SelectByID2('Front Plane','PLANE',0,0,0,False,0,None,0); sm.InsertSketch(True)
sm.AddToDB=True; seg=sm.CreateCircleByRadius(0,0,0,.004);sm.AddToDB=False
sk=wrap(sm.ActiveSketch,sld.ISketch)
print('initial',sk.GetConstrainedStatus())
arc=wrap(seg,sld.ISketchArc); pt=wrap(arc.GetCenterPoint2(),sld.ISketchPoint)
pt.Select4(False,None);print('origin select',m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0));m.SketchAddConstraints('sgCOINCIDENT')
print('coincident alone',sk.GetConstrainedStatus())
m.ClearSelection2(True);seg.Select4(False,None);dd=m.AddDiameterDimension2(.006,.006,0)
print('dim',dd,sk.GetConstrainedStatus())
print('auto',sm.FullyDefineSketch(True,True,1023,True,1,None,1,None,1,1))
m.EditRebuild3();print('after',sk.GetConstrainedStatus())
sm.InsertSketch(True)

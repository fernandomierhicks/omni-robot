from sw_native import *
sw=connect();p=NativePart(sw,'zero_dim_probe');m=p.model;sm=m.SketchManager
m.Extension.SelectByID2('Front Plane','PLANE',0,0,0,False,0,None,0);sm.InsertSketch(True)
sm.AddToDB=True;q=sm.CreatePoint(0,.01,0);sm.AddToDB=False
q.Select4(False,None);m.Extension.SelectByID2('Point1@Origin','EXTSKETCHPOINT',0,0,0,True,0,None,0)
dd=m.AddHorizontalDimension2(.005,.005,0);print('ZERO DIM',dd)
if dd:
    d=wrap(wrap(dd,sld.IDisplayDimension).GetDimension2(0),sld.IDimension);print(d.FullName,d.SystemValue,d.DrivenState);print('EDIT',d.SetSystemValue3(.002,1,None));print(q.X,q.Y)
sm.InsertSketch(True)

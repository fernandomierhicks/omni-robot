from sw_native import *
from update_native import open_part
import numpy as np
sw=connect();m=open_part(sw,'dummy_motor_rear_cap');p=wrap(m,sld.IPartDoc);fm=m.FeatureManager
name='0014_cone_Translate';print('rollback',fm.EditRollback(4,name),flush=True)
f=wrap(p.FeatureByName(name),sld.IFeature);fd=wrap(f.GetDefinition(),sld.IMoveCopyBodyFeatureData)
print('access',fd.AccessSelections(m,None),flush=True)
fd.TransformX+=1;fd.TransformY+=1
print('modify',f.ModifyDefinition(fd,m,None),flush=True);m.EditRebuild3()
far=[b for b in bodies(m) if b.GetMassProperties(1)[0]>.5 and b.GetMassProperties(1)[1]>.5]
print('far',len(far),flush=True)
m.ClearSelection2(True);sd=wrap(wrap(m.SelectionManager,sld.ISelectionMgr).CreateSelectData(),sld.ISelectData);sd.Mark=1
for i,b in enumerate(far):b.Select2(i>0,sd)
g=fm.InsertMoveCopyBody2(-1,-1,0,0,0,0,0,0,0,0,False,1)
print('compensate',g.Name if g else None,flush=True)
if g:g.Name='Coordinate_frame_compensation'
print('rollforward',fm.EditRollback(1,''),flush=True);m.ForceRebuild3(False)
print('solidchecks',[(b.Check2(),b.GetMassProperties(1)[3]*1e9) for b in bodies(m)],flush=True)
sw.CloseDoc(m.GetTitle())

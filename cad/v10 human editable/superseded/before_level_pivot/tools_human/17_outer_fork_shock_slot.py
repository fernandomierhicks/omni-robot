"""Outer fork: a slot in the cap's outer face for the shock case. The case is 11 mm wide and runs up from the eye, so the 9 mm eye relief alone
left a sliver of the case inside the cap rim (0.004 mm3 nominal, 0.6 mm3 at full bump)."""
from common import *
sw=connect()
try:
    key='outer_fork_6804';m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0);sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False;wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    E=DesignEqs(m);delete_from(m,'Shock Case Slot Sketch')
    E.var('shock_y',36,'mm','Lower shock eye Y; replaced by the shared layout value when the part is linked')
    E.var('shock_z',10,'mm','Lower shock eye Z; replaced by the shared layout value when the part is linked')
    E.var('shock_slot_half_width',5.8,'mm','Half width of the shock-case slot: the case is 11 mm wide plus 0.6 mm clearance')
    E.var('shock_slot_length',12,'mm','Slot length from the shock eye along the shock axis (toward +Z)')
    E.var('shock_slot_depth',3,'mm','Slot depth into the cap, same as the eye relief')
    vs=[(-10,30.2,'shock_z','shock_y - shock_slot_half_width'),(-22,30.2,'shock_z + shock_slot_length','shock_y - shock_slot_half_width'),
        (-22,41.8,'shock_z + shock_slot_length','shock_y + shock_slot_half_width'),(-10,41.8,'shock_z','shock_y + shock_slot_half_width')]
    s=orthogonal_profile(m,E,'Cap Exterior Plane','Shock Case Slot Sketch',vs,axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);before=volume_mm3(m)
    f=cut_extrude(m,3,reverse=True,name='Shock Case Slot');link_feature(E,f,['shock_slot_depth'])
    E.solve();m.ForceRebuild3(False)
    removed=before-volume_mm3(m);print('removed',round(removed,1),'mm3 (upper bound',round(11.6*12*3,1),')')
    if not 0<removed<=11.6*12*3+1:raise RuntimeError('Slot volume is wrong: check the cut direction')
    finish(sw,m,E,key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

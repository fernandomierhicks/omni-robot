"""Outer fork: a slot in the cap's outer face for the shock case. The case is 11 mm wide and runs up from the eye, so the 9 mm eye relief alone
left a sliver of the case inside the cap rim (0.004 mm3 nominal, 0.6 mm3 at full bump). Since the Nov 2 level pivot the case leans up to
14.7 deg toward the axle (fork frame, full bump; 2.6 deg the other way at droop): the slot reaches 8.5 mm toward the axle and 6 mm deep
(the case spans x -60..-49), staying outside the head-ring groove (r 30)."""
from common import *
sw=connect()
try:
    key='outer_fork_6804';m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0);sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False;wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    E=DesignEqs(m);delete_from(m,'Shock Case Slot Sketch')
    E.var('shock_y',37,'mm','Lower shock eye Y; replaced by the shared layout value when the part is linked')
    E.var('shock_z',11,'mm','Lower shock eye Z; replaced by the shared layout value when the part is linked')
    E.var('shock_slot_half_width',5.8,'mm','Slot half width away from the axle: the case is 11 mm wide plus 0.6 mm clearance')
    E.var('shock_slot_inner_reach',8.5,'mm','Slot reach toward the axle: the case leans 14.7 deg that way at full bump')
    E.var('shock_slot_length',12,'mm','Slot length from the shock eye along the shock axis (toward +Z)')
    E.var('shock_slot_depth',6,'mm','Slot depth into the cap: past the case inner edge (x -49) by 0.5 mm')
    # eye at (y 37, z 11) since the Nov 2 level pivot
    vs=[(-11,28.5,'shock_z','shock_y - shock_slot_inner_reach'),(-23,28.5,'shock_z + shock_slot_length','shock_y - shock_slot_inner_reach'),
        (-23,42.8,'shock_z + shock_slot_length','shock_y + shock_slot_half_width'),(-11,42.8,'shock_z','shock_y + shock_slot_half_width')]
    s=orthogonal_profile(m,E,'Cap Exterior Plane','Shock Case Slot Sketch',vs,axis=None)
    m.ClearSelection2(True);s.feat.Select2(False,0);before=volume_mm3(m)
    f=cut_extrude(m,6,reverse=True,name='Shock Case Slot');link_feature(E,f,['shock_slot_depth'])
    E.solve();m.ForceRebuild3(False)
    removed=before-volume_mm3(m);print('removed',round(removed,1),'mm3 (upper bound',round(14.3*12*6,1),')')
    if not 0<removed<=14.3*12*6+1:raise RuntimeError('Slot volume is wrong: check the cut direction')
    finish(sw,m,E,key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

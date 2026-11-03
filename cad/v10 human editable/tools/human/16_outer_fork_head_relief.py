"""Outer fork: a ring groove in the cap's inner face for the cartridge screw heads. The three M3 x 12 heads turn with the wheel,
so the clearance must be a full ring (the heads overlapped the cap by ~60 mm3 each)."""
from common import *
sw=connect()
try:
    key='outer_fork_6804';m=open_owned(sw,key);sw.ActivateDoc3(m.GetTitle(),False,0,0);sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False)
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False;wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    E=DesignEqs(m);delete_from(m,'Cap Inner Face Plane')
    E.var('head_ring_outer_diameter',60,'mm','Outer diameter of the screw-head ring groove: head circle (r 26) plus 3.25 head radius plus 0.75 clearance')
    E.var('head_ring_inner_diameter',40,'mm','Inner diameter of the groove; inside the 45 mm cavity, so no material is removed there and the journal is untouched')
    E.var('head_ring_depth',3,'mm','Groove depth from the cap inner face; the heads are 2.5 mm tall and sit 0.5 mm behind that face')
    ref_plane(m,E,'Right Plane',-44.5,'Cap Inner Face Plane','cap_inner_face')
    s=Sketch(m,'Cap Inner Face Plane','Head Ring Sketch')
    for d,expr,at in ((60,'head_ring_outer_diameter',(0,32)),(40,'head_ring_inner_diameter',(0,22))):
        ci=s.circle(0,0,d/2);s.rel('sgCOINCIDENT',ci.GetCenterPoint2(),origin=True);s.ddim(ci,expr,at)
    s.finish(E)
    if s.status()!=3:raise RuntimeError('Ring sketch status '+str(s.status()))
    m.ClearSelection2(True);s.feat.Select2(False,0);before=volume_mm3(m)
    f=cut_extrude(m,3,name='Cartridge Head Ring Groove');link_feature(E,f,['head_ring_depth'])
    E.solve();m.ForceRebuild3(False)
    removed=before-volume_mm3(m);print('removed',round(removed,1),'(ring r20..30 x 3 mm is',round(math.pi*(30**2-20**2)*3,1),'; the part inside r22.5 is already cavity)')
    if not 0<removed<=math.pi*(30**2-22.5**2)*3+1:raise RuntimeError('Groove volume is wrong: check the cut direction')
    finish(sw,m,E,key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

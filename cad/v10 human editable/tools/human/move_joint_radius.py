"""Permanently change the master's joint_radius and rebuild the three hubs that follow it."""
from common import *
from importlib import import_module
T=import_module('test_master_updates');L=import_module('13_link_layout')
NEW=float(sys.argv[1]) if sys.argv[1:] else 52.0
sw=connect()
try:
    master=open_owned(sw,'master_layout')
    T.set_master(master,'joint_radius',NEW,'mm')
    report={'joint_radius':NEW,'parts':{}}
    for key,names in {'wheel_hub_A':['Outer Joint Hole Sketch'],'wheel_hub_B':['Outer Joint Hole Sketch','Inner Joint Hole Sketch'],'wheel_hub_C':['Inner Joint Hole Sketch']}.items():
        m=open_owned(sw,key)
        em=wrap(m.GetEquationMgr(),sld.IEquationMgr);existing={em.Equation(i).split('=',1)[0].strip().strip('"') for i in range(em.GetCount())}
        for name,rhs in L.aliases(key).items():
            if name in existing:L.replace_equation(em,name,rhs)
        T.rebuild(m);r=audit_report(m)
        if not r['pass']:raise RuntimeError(key+' audit failed '+str({k:r[k] for k in ('feature_errors','dead_equations','unlinked_dimensions','body_checks')}))
        coords={n:T.circle_centres(m,n) for n in names}
        for n,v in coords.items():
            if max(abs(math.hypot(x,y)-NEW) for x,y in v)>1e-4:raise RuntimeError('Joint sketch did not follow in '+key)
        r=finish(sw,m,DesignEqs(m),key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
        report['parts'][key]={'volume_mm3':r['volume_mm3'],'sketch_radii_ok':True};print('moved',key,r['volume_mm3'],flush=True)
    (REPORTS/'joint_radius_move.json').write_text(json.dumps(report,indent=2))
    master.FeatureManager.EnableFeatureTree=True;master.FeatureManager.EnableFeatureTreeWindow=True;wrap(master.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
    close_owned(sw,master,save=True)
finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

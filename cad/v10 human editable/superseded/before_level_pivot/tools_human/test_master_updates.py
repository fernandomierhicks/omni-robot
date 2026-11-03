"""Edit the actual master, verify native tool/body positions, restore and save."""
from common import *
from importlib import import_module
L=import_module('13_link_layout')

def rebuild(m):
    connect().ActivateDoc3(m.GetTitle(),False,0,0)
    m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False
    wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    for _ in range(2):
        m.ForceRebuild3(False);wrap(m.GetEquationMgr(),sld.IEquationMgr).EvaluateAll();m.ForceRebuild3(False)

def set_master(m,name,value,unit='mm'):
    connect().ActivateDoc3(m.GetTitle(),False,0,0)
    em=wrap(m.GetEquationMgr(),sld.IEquationMgr)
    for i in range(em.GetCount()):
        old=em.Equation(i)
        if old.split('=',1)[0].strip().strip('"')==name:
            comment=old.split("'",1)[1] if "'" in old else 'Shared design control'
            em.SetEquation(i,'"'+name+'" = '+str(value)+unit+" '"+comment);rebuild(m)
            ok,e,w=m.Save3(1,0,0)
            if not ok or e:raise RuntimeError('Cannot save master update '+str((e,w)))
            return
    raise RuntimeError('Missing master variable '+name)

def circle_centres(m,name):
    sk=wrap(feat_by_name(m,name).GetSpecificFeature2(),sld.ISketch)
    return [(wrap(wrap(z,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint).X*1000,wrap(wrap(z,sld.ISketchArc).GetCenterPoint2(),sld.ISketchPoint).Y*1000) for z in sk.GetSketchSegments() if wrap(z,sld.ISketchSegment).GetType()==1]

def check_radius_edges(m,radius,pcd_radius):
    es=edges_where(bodies(m)[0],lambda g:g[0]=='circle' and abs(g[3]-radius)<1e-4 and abs(abs(g[2][0])-1)<1e-5)
    values=[math.hypot(edge_geom(e)[1][1],edge_geom(e)[1][2]) for e in es]
    if not values or max(abs(v-pcd_radius) for v in values)>1e-4:raise RuntimeError('Actual native cut axes lag the master: '+str(values))
    return values

def run(sw):
    master=open_owned(sw,'master_layout');report_path=REPORTS/'master_update_tests.json'
    report=json.loads(report_path.read_text()) if report_path.exists() else {'tests':[]}
    report['peak_open_documents']=len(sw.GetDocuments() or [])
    cases=[('cradle_radius',6.5,6.4,{'motor_cup':['Fork Pilot Sketch'],'inner_fork_6804':['Cradle Clearance Sketch','Cradle Counterbore Sketch']}),
           ('rear_radius',23.5,23.4,{'dummy_motor_stator':['Rear Mount Pilot Sketch'],'motor_cup':['Rear Mount Sketch']}),
           ('joint_radius',38,37.9,{'wheel_hub_A':['Outer Joint Hole Sketch'],'wheel_hub_B':['Outer Joint Hole Sketch','Inner Joint Hole Sketch'],'wheel_hub_C':['Inner Joint Hole Sketch']}),
           ('outer_ear_foot_x',31.5,31.6,{'bench_plate':['Pod Foot Clearance Sketch','Foot Counterbore Sketch'],'wheel_guard':['Deck Foot Pilot Sketch']}),
           ('roller_axis_radius',52,52.1,{'wheel_hub_A':['Outer Axle Hole Sketch'],'wheel_hub_B':['Outer Axle Hole Sketch','Inner Axle Hole Sketch'],'wheel_hub_C':['Inner Axle Hole Sketch'],'roller_core':[],'roller_tread':[]}),
           ('inner_roller_phase',30,30.1,{'wheel_hub_B':['Inner Axle Hole Sketch'],'wheel_hub_C':['Inner Axle Hole Sketch']})]
    if sys.argv[1:]:cases=[c for c in cases if c[0] in sys.argv[1:]]
    for control,initial,test,targets in cases:
        row={'control':control,'nominal':initial,'test':test,'parts':{}}
        for key,names in targets.items():
            m=open_owned(sw,key);report['peak_open_documents']=max(report['peak_open_documents'],len(sw.GetDocuments() or []))
            em=wrap(m.GetEquationMgr(),sld.IEquationMgr);existing={em.Equation(i).split('=',1)[0].strip().strip('"') for i in range(em.GetCount())}
            for name,rhs in L.aliases(key).items():
                if name in existing:L.replace_equation(em,name,rhs)
            rebuild(m);before=volume_mm3(m)
            try:
                set_master(master,control,test,'deg' if control.endswith('_phase') else 'mm')
                rebuild(m);r=audit_report(m)
                if not r['pass']:raise RuntimeError(key+' rebuild failed during master test')
                coords={name:circle_centres(m,name) for name in names}
                for values in coords.values():
                    if control in ('cradle_radius','rear_radius','joint_radius'):
                        if max(abs(math.hypot(x,y)-test) for x,y in values)>1e-4:raise RuntimeError('Sketch did not follow '+key)
                    elif control=='roller_axis_radius' or control.endswith('_phase'):
                        if max(abs(y-(52 if control.endswith('_phase') else test)) for x,y in values)>1e-4:raise RuntimeError('Roller axis did not follow '+key)
                    elif not any(abs(x+test)<1e-4 for x,y in values):raise RuntimeError('Ear mounting hole did not follow '+key)
                actual_edges=check_radius_edges(m,1.4 if key=='motor_cup' else 1.6,test) if control=='cradle_radius' else None
                if control.endswith('_phase'):
                    L.bind_sketch(m,'Inner Axle Hole Sketch','Inner Roller Axles')
                    sk=wrap(feat_by_name(m,'Inner Axle Hole Sketch').GetSpecificFeature2(),sld.ISketch);mat=wrap(sk.ModelToSketchTransform,sld.IMathTransform).ArrayData
                    if abs(math.degrees(math.atan2(-mat[5],mat[4]))-test)>1e-4:raise RuntimeError('Roller plane angle did not follow the master')
                if key in ('roller_core','roller_tread') and not volume_mm3(m)<before-.01:raise RuntimeError('Barrel surface did not follow roller-axis change')
                row['parts'][key]={'circles':coords,'actual_cut_axis_radii_mm':actual_edges,'volume_mm3':volume_mm3(m),'pass':True}
            finally:
                set_master(master,control,initial,'deg' if control.endswith('_phase') else 'mm')
                rebuild(m)
                delta=volume_mm3(m)-before
                if abs(delta)>max(.1,before*2e-6):raise RuntimeError('Nominal geometry was not restored '+str((key,delta)))
                finish(sw,m,DesignEqs(m),key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
            print('Verified and closed',key,flush=True)
        row['restored']=True;report['tests']=[r for r in report['tests'] if r['control']!=control]+[row];report_path.write_text(json.dumps(report,indent=2))
        print('Master edit restored:',control,'PASS',flush=True)
    report['pass']=len(report['tests'])==6 and all(r.get('restored') and all(p['pass'] for p in r['parts'].values()) for r in report['tests']);report_path.write_text(json.dumps(report,indent=2))
    master.FeatureManager.EnableFeatureTree=True;master.FeatureManager.EnableFeatureTreeWindow=True;wrap(master.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
    close_owned(sw,master,save=True)

if __name__=='__main__':
    sw=connect()
    try:run(sw)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True)

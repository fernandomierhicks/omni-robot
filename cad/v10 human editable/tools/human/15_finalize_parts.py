"""Small-batch native cleanup: never preload the part library."""
from common import *
from importlib import import_module
L=import_module('13_link_layout')

def change_literal(m,name,value,comment):
    em=wrap(m.GetEquationMgr(),sld.IEquationMgr)
    L.replace_equation(em,name,str(value)+'mm',comment)

def prune_unused_globals(m):
    em=wrap(m.GetEquationMgr(),sld.IEquationMgr);removed=[]
    while True:
        texts=[em.Equation(i) for i in range(em.GetCount())]
        used=set(n for t in texts for n in re.findall(r'"([^"]+)"',t.split('=',1)[1].split("'",1)[0]))
        unused=[(i,t.split('=',1)[0].strip().strip('"')) for i,t in enumerate(texts) if '@' not in t.split('=',1)[0] and t.split('=',1)[0].strip().strip('"') not in used]
        if not unused:break
        for i,name in reversed(unused):em.Delete(i);removed.append(name)
    return removed

# Light, distinct colours so parts read clearly in SolidWorks; fasteners stay magenta (Fernando's rule).
PALETTE={'wheel_hub_A':(.45,.75,1.0),'wheel_hub_B':(.55,.90,.55),'wheel_hub_C':(1.0,.80,.40),'wheel_cartridge':(.40,.90,.85),
         'roller_core':(.97,.95,.70),'roller_tread':(.84,.84,.90),'inner_fork_6804':(.50,.72,.98),'outer_fork_6804':(.75,.62,.98),
         'motor_cup':(.98,.75,.50),'wheel_guard':(.60,.88,.60),'bench_plate':(.88,.84,.76),'pod_mount':(.82,.92,.50),
         'fork_bar':(.78,.80,.85),'pivot_shaft':(.78,.80,.85),'shock_case':(.98,.62,.62),'shock_rod':(.82,.84,.88),
         'pivot_shim':(.96,.86,.45),'pivot_outer_spacer':(.96,.86,.45),'dummy_motor_stator':(1.0,.68,.30),'dummy_motor_rotor':(.35,.85,.95)}

def colour(key):
    if key in PALETTE:return PALETTE[key]
    if key.startswith(('bolt_','nut_','washer_')):return (.95,.15,.65)
    if key.startswith('bearing_'):return (.82,.84,.88)
    return (.60,.75,.90)

def style(m,key):
    # R G B, ambient, diffuse, specular, shininess, transparency, emission: bright ambient and full diffuse keep colours light
    m.MaterialPropertyValues=num_arr(list(colour(key))+[.55,1.0,.25,.3,0,0])

def run(sw,keys):
    import gc
    results=[]
    for key in keys:
        master=open_owned(sw,'master_layout')
        m=open_owned(sw,key);sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,False);sw.CommandInProgress=True
        m.FeatureManager.EnableFeatureTree=False;m.FeatureManager.EnableFeatureTreeWindow=False;wrap(m.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
        E=DesignEqs(m)
        if key=='motor_cup':change_literal(m,'channel_floor',7.5,'Harness floor leaves 1.4 mm to the nominal M3 major thread')
        if key=='outer_fork_6804':
            change_literal(m,'wheel_lobe_radius',35,'Wheel lobe retains a 1.5 mm rim outside the cartridge sweep')
            change_literal(m,'cap_radius',35,'Cap outside radius; avoids the former 0.5 mm annular rim')
        if key=='dummy_motor_case' and feat_by_name(m,'Rotor Axial Datum Plane') is None:
            E.var('rotor_float_datum_distance',10,'mm','Offset datum makes outward rotor end-float unambiguous')
            ref_plane(m,E,'Front Plane',-10,'Rotor Axial Datum Plane','rotor_float_datum_distance')
        style(m,key);removed=prune_unused_globals(m)
        r=finish(sw,m,E,key,OUT/('V10H_'+key+'.SLDPRT'));close_owned(sw,m,save=True)
        results.append({'part':key,'removed_unused_controls':removed,'pass':r['pass'],'open_documents_after_close':len(sw.GetDocuments() or [])})
        (REPORTS/'finalize_parts.json').write_text(json.dumps(results,indent=2))
        close_owned(sw,master,save=True);m=E=master=None;gc.collect()

if __name__=='__main__':
    sw=connect();keys=sys.argv[1:] or sorted(k for k in json.loads(REGISTRY.read_text()) if k!='master_layout')
    try:run(sw,keys)
    finally:sw.SetUserPreferenceToggle(c.swInputDimValOnCreate,True);sw.CommandInProgress=False

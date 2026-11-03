"""Native mated assemblies. No fixed-position STEP assembly is substituted."""
import sys
from pathlib import Path
TASK_ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(TASK_ROOT/'tools'))
from sw_native import *
import numpy as np
import mate_api as M
from scipy.spatial.transform import Rotation

def transform(x=0,y=0,z=0,rx=0,ry=0,rz=0):
    a=np.eye(4);a[:3,:3]=Rotation.from_euler('XYZ',[rx,ry,rz],degrees=True).as_matrix();a[:3,3]=[x,y,z];return a

class Assembly:
    def __init__(self,sw,name):
        self.sw=sw;self.name='V10H_'+name;self.path=ROOT/(self.name+'.SLDASM');self.parts={}
        self.progress=ROOT/'validation/human'/(self.name+'_progress.json')
        if self.path.exists() and self.progress.exists():
            d,e,w=sw.OpenDoc6(str(self.path),2,1,'',0,0)
            if d is None:raise RuntimeError('Cannot resume assembly')
            self.model=wrap(d,sld.IModelDoc2);self.asm=wrap(self.model,sld.IAssemblyDoc)
            existing=M.comps(self.model)
            self.parts={key:existing[cn] for key,cn in json.loads(self.progress.read_text()).items()}
            sw.ActivateDoc3(self.model.GetTitle(),False,0,0)
            self.quiet()
            print('RESUMING assembly',self.name,len(self.parts),'components',flush=True)
            return
        template=sw.GetUserPreferenceStringValue(c.swDefaultTemplateAssembly)
        self.model=wrap(sw.NewDocument(template,0,0,0),sld.IModelDoc2)
        self.model.SetTitle2(self.name);self.asm=wrap(self.model,sld.IAssemblyDoc)
        self.save(check=False)
        self.quiet()
    def quiet(self):
        self.model.FeatureManager.EnableFeatureTree=False
        self.model.FeatureManager.EnableFeatureTreeWindow=False
        wrap(self.model.ActiveView,sld.IModelView).EnableGraphicsUpdate=False
    def add(self,key,part,T=None,fixed=False):
        if key in self.parts:return self.parts[key].Name2
        path=ROOT/'parts/human'/('V10H_'+part+'.SLDPRT')
        if not path.exists():raise RuntimeError('Missing native part '+str(path))
        d,e,w=self.sw.OpenDoc6(str(path),1,1,'',0,0)
        if d is None:raise RuntimeError('Cannot open native part '+str(path))
        self.sw.ActivateDoc3(self.model.GetTitle(),False,0,0)
        cp=wrap(self.asm.AddComponent5(str(path),0,'',False,'',0,0,0),sld.IComponent2)
        if cp is None:raise RuntimeError('Cannot insert '+part)
        cp.Name2=key;self.parts[key]=cp
        self.model.ClearSelection2(True);cp.Select4(False,None,False)
        self.asm.UnfixComponent()
        T=np.eye(4) if T is None else T
        M.set_placement(self.sw,cp,T[:3,:])
        if fixed:
            self.model.ClearSelection2(True);cp.Select4(False,None,False);self.asm.FixComponent()
        self.model.ClearSelection2(True)
        return cp.Name2
    def mater(self):return M.Mater(self.sw,self.model,prefix='POD_')
    def plane(self,key,plane):return f'{plane}@{self.parts[key].Name2}@{self.name}'
    def save(self,check=True):
        m=self.model;m.EditRebuild3()
        if check:
            bad=M.mate_errors(m)
            if bad:raise RuntimeError('Mate errors '+str(bad))
            for cp in M.comps(m).values():
                if not str(Path(cp.GetPathName())).lower().startswith(str(ROOT).lower()):raise RuntimeError('External reference '+cp.GetPathName())
        ok,e,w=m.Extension.SaveAs3(str(self.path),0,1,None,None,0,0)
        if not ok or e:raise RuntimeError('Assembly save '+str((e,w)))
        self.progress.write_text(json.dumps({key:cp.Name2 for key,cp in self.parts.items()},indent=2))
        if check:
            data={'assembly':str(self.path),'mates':[s.Name for s in M.mates_of(m)],'mate_errors':M.mate_errors(m),'components':{n:{'path':cp.GetPathName(),'fixed':cp.IsFixed(),'constrained_status':cp.GetConstrainedStatus(),'transform':placement(cp).tolist()} for n,cp in M.comps(m).items()}}
            (ROOT/'validation/human'/(self.name+'.json')).write_text(json.dumps(data,indent=2))
        return self.path
    def view(self):
        self.model.FeatureManager.EnableFeatureTree=True
        self.model.FeatureManager.EnableFeatureTreeWindow=True
        wrap(self.model.ActiveView,sld.IModelView).EnableGraphicsUpdate=True
        self.model.ShowNamedView2('*Isometric',7);self.model.ViewZoomtofit2();self.model.GraphicsRedraw2()
    def concentric(self,m,name,a,axis,point,ra,bb,rb):
        return m.mate(name,'concentric',m.cyl(a,axis,point,ra),m.cyl(bb,axis,point,rb))
    def limit(self,m,name,first,second,kind,value,lo,hi,flip=False):
        existing=wrap(self.asm.FeatureByName('POD_'+name),sld.IFeature)
        if existing is not None:return existing
        poses=M.placements(self.model)
        for direction in (flip,not flip):
            before=set(f.Name for f in M.mates_of(self.model));m._select([first,second])
            mate,error=self.asm.AddMate5(6 if kind=='angle' else 5,c.swMateAlignCLOSEST,direction,
                value*MM if kind=='distance' else 0,hi*MM if kind=='distance' else 0,lo*MM if kind=='distance' else 0,0,0,
                math.radians(value) if kind=='angle' else 0,math.radians(hi) if kind=='angle' else 0,math.radians(lo) if kind=='angle' else 0,False,False,0)
            self.model.ClearSelection2(True);self.model.EditRebuild3();new=[f for f in M.mates_of(self.model) if f.Name not in before]
            if mate is not None and error==1 and len(new)==1 and not new[0].GetErrorCode2()[0] and not M.compare(poses,M.placements(self.model))[0]:
                new[0].Name='POD_'+name;return new[0]
            for f in new:self.model.ClearSelection2(True);f.Select2(False,0);self.model.Extension.DeleteSelection2(0)
            self.model.EditRebuild3()
            if m._restore(poses):raise RuntimeError('Cannot restore rejected limit '+name)
        raise RuntimeError('Limit mate failed or moved components: '+name)

def pod(sw):
    manifest=json.loads((ROOT/'reference/assembly_manifest.json').read_text());records=manifest['parts'];bykey={r['key']:r for r in records}
    a=Assembly(sw,'wheel_pod')
    a.add('Master_coordinates','master_layout',fixed=True);a.parts['Master_coordinates'].Visible=0
    for i,r in enumerate(records):
        a.add(r['key'],r['part'],np.array(r['matrix']),fixed=r['role']=='fixed')
        if i%20==0:
            a.save(check=False);print('Inserted components',i+1,'/',len(records),flush=True)
    a.save(check=False)
    m=a.mater()
    # The movable structural groups are defined by real hinge mates. Rigid
    # children use lock mates, retaining their deliberate relative transforms.
    fast=lambda r:r['part'].startswith(('bolt_','nut_')) or r['part']=='washer_M3'
    for r in records:
        if r['parent'] and not fast(r) and not r['part'].startswith('bearing_') and r['key'] not in ('Pivot_shaft','Inner_pivot_spacer','Outer_pivot_spacer'):
            m.lock(r['key'],a.parts[r['key']].Name2,a.parts[r['parent']].Name2)
    a.save(check=False)
    inner=a.parts['Inner_fork'].Name2;guard=a.parts['Guard'].Name2;shaft=a.parts['Pivot_shaft'].Name2;hub=a.parts['Hub_B'].Name2
    m.mate('Shaft_axial_location','distance',a.plane('Pivot_shaft','Right Plane'),a.plane('Guard','Right Plane'),dist=3)
    m.mate('Shaft_in_guard','concentric',m.cyl(shaft,[1,0,0],[0,*manifest['pivot']],3.5),m.cyl(guard,[1,0,0],[0,*manifest['pivot']],3.55),lock=True)
    for key in ('Inner_pivot_spacer','Outer_pivot_spacer'):
        x=bykey[key]['matrix'][0][3]
        m.mate(key+'_axial','distance',a.plane(key,'Front Plane'),a.plane('Guard','Right Plane'),dist=abs(x))
        m.mate(key+'_axis','concentric',m.cyl(a.parts[key].Name2,[1,0,0],[0,*manifest['pivot']],3.6),m.cyl(shaft,[1,0,0],[0,*manifest['pivot']],3.5),lock=True)
    m.mate('Fork_axial_location','coincident',a.plane('Inner_fork','Right Plane'),a.plane('Guard','Right Plane'))
    a.concentric(m,'Suspension_pivot',inner,[1,0,0],[0,*manifest['pivot']],11,shaft,3.5)
    a.limit(m,'Suspension_travel',a.plane('Inner_fork','Top Plane'),a.plane('Guard','Front Plane'),'angle',90,*manifest['limits_deg'])
    m.mate('Wheel_axial_location','coincident',a.plane('Hub_B','Right Plane'),a.plane('Inner_fork','Right Plane'))
    a.concentric(m,'Wheel_bearings',hub,[1,0,0],[0,0,0],30,inner,10)
    for r in records:
        if r['part'].startswith('bearing_'):mate_bearing(a,m,r,records)
    a.save(check=False)
    # Telescoping shock mockups: each eye pivots, the rod slides in its case.
    for prefix,lower,upper in [('Inner','Inner_fork','Tower'),('Outer','Outer_fork','Guard')]:
        ck=prefix+'_shock_case';rk=prefix+'_shock_rod';case=a.parts[ck].Name2;rod=a.parts[rk].Name2
        T=np.array(bykey[ck]['matrix']);H=np.array(bykey[rk]['matrix']);lo=T[:3,3];hi=H[:3,3];axis=T[:3,2]
        m.mate(prefix+'_lower_eye','concentric',m.cyl(case,[1,0,0],lo,1.6),m.cyl(a.parts[lower].Name2,[1,0,0],lo,1.4))
        m.mate(prefix+'_shock_lateral','distance',a.plane(ck,'Right Plane'),a.plane('Inner_fork','Right Plane'),dist=abs(lo[0]))
        m.mate(prefix+'_upper_eye','concentric',m.cyl(rod,[1,0,0],hi,1.6),m.cyl(a.parts[upper].Name2,[1,0,0],hi,1.7 if prefix=='Inner' else 1.4))
        m.mate(prefix+'_shock_slider','concentric',m.cyl(case,axis,lo,2.2),m.cyl(rod,axis,lo,2))
        # Eye positions and the concentric slider determine shock length.
        # The fork's angle limits keep it inside 40–50 mm; a second limit
        # distance overdefines this closed linkage in SolidWorks.
    a.save(check=False)
    # Stock fasteners are seated and concentric, so a master-hole edit also
    # moves the hardware. Rotational locking is appropriate for bolted joints.
    hardware=[r for r in records if fast(r)]
    for i,r in enumerate(hardware):
        mate_fastener(a,m,r,records)
        if i%5==0:a.save(check=False)
    # A roller turns on its axle and retains the specified 0.6 mm axial float.
    for row in (1,2):
        for j in range(1,7):
            key=f'Roller_{row}_{j}';bolt=f'bolt_M3_50_PAN_{(row-1)*6+j}'
            T=np.array(bykey[key]['matrix']);axis=T[:3,2];pt=T[:3,3]
            a.concentric(m,key+'_axle',a.parts[key].Name2,axis,pt,1.8,a.parts[bolt].Name2,1.2)
            a.limit(m,key+'_float',a.plane(key,'Front Plane'),a.plane(bolt,'Front Plane'),'distance',23.5,23.2,23.8)
    a.view();a.save();print('ASSEMBLY SAVED',a.path,flush=True);return a

def mate_fastener(a,m,r,records):
    cp=a.parts[r['key']];name=cp.Name2;T=np.array(r['matrix']);axis=T[:3,2];point=T[:3,3]
    nut=r['part'].startswith('nut_');washer=r['part'].startswith('washer_M3');flat=r['part'].endswith('_FLAT')
    radius=1.4 if nut else 1.6 if washer else 1.0 if 'M2p5' in r['part'] else 1.2
    # Candidate structural hosts only; using another free screw as a host
    # would leave an unintended moving pair.
    hosts=[]
    for host in records:
        if host['key']==r['key'] or host['part'].startswith(('bolt_','nut_','bearing_','roller_')):continue
        hn=a.parts[host['key']].Name2
        for f in m.faces(hn):
            if f['kind']!='cyl' or abs(abs(f['axis']@axis)-1)>1e-6 or f['r']<.95 or f['r']>3.65:continue
            delta=f['pt']-point
            if np.linalg.norm(delta-(delta@axis)*axis)>.015:continue
            hosts.append((host['part'].startswith('shock_'),abs(f['r']-radius),-f['area'],host,f))
    if not hosts:raise RuntimeError('No concentric host for '+r['key'])
    hosts.sort(key=lambda x:x[:3]);host,face=hosts[0][3:]
    own=m.cyl(name,axis,point,radius)
    fastnormal=axis if flat else -axis
    own_plane=m.plane(name,fastnormal,float(fastnormal@point))
    seats=[]
    for host in records:
        if host['key']==r['key'] or host['part'].startswith(('bolt_','nut_','bearing_','roller_')):continue
        for f in m.faces(a.parts[host['key']].Name2):
            if f['kind']!='plane' or f['n']@axis<.99999:continue
            gap=float(axis@point)-f['off']
            if abs(gap)>.06:continue
            box=f['aabb']
            if np.any(point<box[0]-.1) or np.any(point>box[1]+.1):continue
            seats.append((abs(gap),host['key']!=r.get('parent'),-f['area'],f,gap))
    if not seats:raise RuntimeError('No seating face for '+r['key'])
    seats.sort(key=lambda x:x[:3]);seat,gap=seats[0][3:]
    m.mate(r['key']+'_seat','distance' if abs(gap)>.001 else 'coincident',own_plane,seat,dist=abs(gap))
    m.mate(r['key']+'_axis','concentric',own,face,lock=True)

def mate_bearing(a,m,r,records):
    spec={'bearing_627':(3.5,11,7),'bearing_6804':(10,16,7),'bearing_687':(3.5,7,5)}[r['part']]
    radius,outer,width=spec;T=np.array(r['matrix']);axis=T[:3,2];pt=T[:3,3];name=a.parts[r['key']].Name2
    options=[]
    for h in records:
        if h['part'].startswith(('bolt_','nut_','washer_','bearing_','roller_','shock_')):continue
        for f in m.faces(a.parts[h['key']].Name2):
            if f['kind']!='cyl' or abs(f['r']-outer)>.01 or abs(abs(f['axis']@axis)-1)>1e-6:continue
            delta=f['pt']-pt
            if np.linalg.norm(delta-(delta@axis)*axis)>.015:continue
            options.append((abs((f['ctr']-pt)@axis),h,f))
    if not options:raise RuntimeError('No bearing housing '+r['key'])
    options.sort(key=lambda x:x[0]);_,host,hole=options[0]
    seats=[]
    for sign in (-1,1):
        end=pt+axis*sign*width/2;normal=axis*sign
        own=m.plane(name,normal,float(normal@end))
        for f in m.faces(a.parts[host['key']].Name2):
            if f['kind']=='plane' and f['n']@(-normal)>.99999 and abs(f['off']-float((-normal)@end))<.02:
                seats.append((abs(f['off']-float((-normal)@end)),own,f))
    if not seats:raise RuntimeError('No bearing shoulder '+r['key'])
    seats.sort(key=lambda x:x[0]);gap,own,seat=seats[0]
    m.mate(r['key']+'_shoulder','distance' if gap>.001 else 'coincident',own,seat,dist=gap)
    m.mate(r['key']+'_housing','concentric',m.cyl(name,axis,pt,outer),hole,lock=True)

def dummy(sw):
    """Two printed parts (stator, rotor), one 687 bearing, one M3 x 8 screw and one large M3 washer."""
    a=Assembly(sw,'dummy_motor')
    a.add('Stator','dummy_motor_stator',fixed=True)
    a.add('Bearing_687','bearing_687',transform(z=-7.2))
    a.add('Rotor','dummy_motor_rotor')
    T_washer=transform(z=-10.8);T_screw=transform(z=-10.8,rx=180)
    a.add('Washer','washer_M3_large',T_washer)
    a.add('Screw','bolt_M3_8_PAN',T_screw)
    records=[{'key':key,'part':part,'parent':parent,'matrix':T.tolist()} for key,part,parent,T in
             [('Stator','dummy_motor_stator',None,np.eye(4)),('Rotor','dummy_motor_rotor',None,np.eye(4)),
              ('Washer','washer_M3_large','Rotor',T_washer),('Screw','bolt_M3_8_PAN','Rotor',T_screw)]]
    a.save(check=False)
    m=a.mater()
    m.lock('Motor_Bearing_687',a.parts['Bearing_687'].Name2,a.parts['Stator'].Name2)
    a.limit(m,'Rotor_end_float',a.plane('Rotor','Front Plane'),a.plane('Stator','Rotor Axial Datum Plane'),'distance',10,10,10.3,flip=True)
    a.concentric(m,'Rotor_bearing',a.parts['Rotor'].Name2,[0,0,1],[0,0,0],3.45,a.parts['Bearing_687'].Name2,3.5)
    m.lock('Washer_to_rotor',a.parts['Washer'].Name2,a.parts['Rotor'].Name2)
    mate_fastener(a,m,records[3],records)
    a.view();a.save();print(a.path,flush=True)
    return a

if __name__=='__main__':
    sw=connect()
    if len(sys.argv)>1 and sys.argv[1]=='dummy':dummy(sw)
    if len(sys.argv)>1 and sys.argv[1]=='pod':pod(sw)
    elif len(sys.argv)>1 and sys.argv[1]=='pod':pod(sw)



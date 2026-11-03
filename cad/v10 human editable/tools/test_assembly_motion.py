"""Drive temporary mates, read actual transforms, then restore free motion."""
from sw_native import *
import numpy as np
import mate_api as M

def angle_driver(sw,m,first,second,value,name):
    asm=wrap(m,sld.IAssemblyDoc);mater=M.Mater(sw,m)
    before=set(f.Name for f in M.mates_of(m));mater._select([first,second])
    mate,error=asm.AddMate5(c.swMateANGLE,c.swMateAlignCLOSEST,False,0,0,0,0,0,math.radians(value),0,0,False,False,0)
    m.ClearSelection2(True);m.EditRebuild3()
    new=[f for f in M.mates_of(m) if f.Name not in before]
    if mate is None or error!=1 or len(new)!=1:raise RuntimeError('Cannot create motion driver '+name)
    f=new[0];f.Name=name
    md=wrap(f.GetSpecificFeature2(),sld.IMate2)
    dim=wrap(wrap(md.DisplayDimension,sld.IDisplayDimension).GetDimension2(0),sld.IDimension)
    return f,dim

def remove(m,f):
    m.ClearSelection2(True);f.Select2(False,0)
    if not m.Extension.DeleteSelection2(0):raise RuntimeError('Cannot remove temporary motion driver')
    m.EditRebuild3()

def main():
    sw=connect();path=ROOT/'V10_wheel_pod.SLDASM'
    d,e,w=sw.OpenDoc6(str(path),2,1,'',0,0);m=wrap(d,sld.IModelDoc2)
    sw.ActivateDoc3(m.GetTitle(),False,0,0)
    progress=json.loads((ROOT/'validation/V10_wheel_pod_progress.json').read_text());cs=M.comps(m)
    cp=lambda key:cs[progress[key]]
    plane=lambda key,name:f'{name}@{cp(key).Name2}@V10_wheel_pod'
    report={'suspension':[],'wheel_spin':[],'roller_spin':[]}
    nominal=placement(cp('Inner_fork'));py,pz=json.loads((ROOT/'reference/assembly_manifest.json').read_text())['pivot']
    f,dim=angle_driver(sw,m,plane('Inner_fork','Top Plane'),plane('Guard','Front Plane'),90,'VERIFY_Suspension')
    try:
        for bump in (-3,0,7.5,15,0):
            a=math.asin((bump-pz)/math.hypot(py,pz))+math.atan2(pz,py)
            dim.SetSystemValue3(math.pi/2+a,1,None);m.EditRebuild3()
            actual=placement(cp('Inner_fork'));errors=M.mate_errors(m)
            shock={prefix:float(np.linalg.norm(placement(cp(prefix+'_shock_rod'))[:,3]-placement(cp(prefix+'_shock_case'))[:,3])) for prefix in ('Inner','Outer')}
            row={'bump_mm':bump,'actual_axle_z_mm':float(actual[2,3]),'shock_lengths_mm':shock,'mate_errors':errors}
            row['pass']=abs(actual[2,3]-bump)<.002 and not errors and all(39.999<v<50.001 for v in shock.values())
            report['suspension'].append(row);print(row,flush=True)
            if not row['pass']:raise RuntimeError('Suspension motion check failed')
    finally:
        dim.SetSystemValue3(math.pi/2,1,None);m.EditRebuild3();remove(m,f)
    for key,relative,section in [('Hub_B','Inner_fork','wheel_spin'),('Roller_1_1','bolt_M3_50_PAN_1','roller_spin')]:
        # A single temporary angle mate drives the remaining rotational DOF.
        # Roller local X is perpendicular to its tangent axis; the axle's
        # corresponding plane starts aligned, so use an offset reference.
        first=plane(key,'Top Plane')
        second=plane(relative,'Front Plane' if key=='Hub_B' else 'Right Plane')
        before=placement(cp(key))
        f,dim=angle_driver(sw,m,first,second,90,'VERIFY_'+section)
        try:
            for degrees in (105,125,90):
                dim.SetSystemValue3(math.radians(degrees),1,None);m.EditRebuild3()
                actual=placement(cp(key));errors=M.mate_errors(m)
                row={'command_deg':degrees,'rotation_change':float(np.max(abs(actual[:,:3]-before[:,:3]))),'mate_errors':errors}
                row['pass']=not errors and (degrees==90 or row['rotation_change']>.01)
                report[section].append(row);print(key,row,flush=True)
                if not row['pass']:raise RuntimeError('Rotation motion check failed')
        finally:
            dim.SetSystemValue3(math.pi/2,1,None);m.EditRebuild3();remove(m,f)
    report['pass']=all(r['pass'] for rows in report.values() if isinstance(rows,list) for r in rows)
    if any(f.Name.startswith('VERIFY_') for f in M.mates_of(m)):raise RuntimeError('Temporary driver left in assembly')
    ok,e,w=m.Save3(1,0,0)
    if not ok or e:raise RuntimeError('Cannot save restored free assembly')
    (ROOT/'validation/assembly_motion.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()

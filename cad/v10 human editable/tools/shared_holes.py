"""Shared fastener coordinates and exact part-to-pod reference frames."""
import sys,json,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'v9'))
import pod_design as V
import pod as P,wheel as W,dims as D

def matrix(loc):
    t=loc.wrapped.Transformation()
    return np.array([[t.Value(i,j) for j in range(1,5)] for i in range(1,4)]+[[0,0,0,1]])

def plan():
    patterns={};lines=[]
    def add(name,coords,u,v,axis,radii,parts,source_pattern=None,source_indices=None):
        pattern=source_pattern or name
        if source_pattern is None:patterns[pattern]={'points':coords}
        for j,pt in enumerate(coords):
            idx=source_indices[j] if source_indices else j
            lines.append({'name':name+'_'+str(j+1),'pattern':pattern,'index':idx,'point':(np.array(u)*pt[0]+np.array(v)*pt[1]).tolist(),'u':u,'v':v,'axis':axis,'radii':radii,'parts':parts})
    ux,vx,dx=[0,0,-1],[0,1,0],[1,0,0]
    add('Cradle_to_fork',[[-z,y] for y,z in P.CUP_SCREWS],ux,vx,dx,[1.4,1.6,3.6],['motor_cup','inner_fork_6804'])
    for row in (0,1):
        add('Hub_joint_row_'+str(row+1),[[-z,y] for y,z in W._joint_pts(row)],ux,vx,dx,[1.4,1.7,3.6,7,9],['wheel_hub_A','wheel_hub_B','wheel_hub_C'])
    add('Bearing_cartridge',[[-D.polar(26,a)[1],D.polar(26,a)[0]] for a in (30,150,270)],ux,vx,dx,[1.4,1.7,4.5],['wheel_cartridge','wheel_hub_A'])
    add('Motor_output',[[-D.polar(D.AK_OUT_PCD/2,a)[1],D.polar(D.AK_OUT_PCD/2,a)[0]] for a in D.AK_OUT_ANGLES],ux,vx,dx,[1.05,D.CLR_M25/2,2.75],['wheel_hub_A','dummy_motor_rotor'])
    add('Motor_rear',[[-D.polar(D.AK_REAR_PCD/2,a)[1],D.polar(D.AK_REAR_PCD/2,a)[0]] for a in D.AK_REAR_ANGLES],ux,vx,dx,[1.05,D.CLR_M25/2,3,3.15],['motor_cup','dummy_motor_rear_cap','dummy_motor_case'])
    add('Motor_cover',[[-D.polar(23.3,a)[0],D.polar(23.3,a)[1]] for a in (45,135,225,315)],ux,vx,dx,[1.25,1.4,1.7,2.8,3],['dummy_motor_case','dummy_motor_rear_cap'])
    add('Guard_feet',[[x,s*(D.HOUSE_Y+4)] for x,s in P.guard_feet(False)],[1,0,0],[0,1,0],[0,0,1],[1.4,1.7,1.9,3.6],['wheel_guard','bench_plate'])
    add('Tower_feet',[list(xy) for xy in P.MOUNT_SCREWS],[1,0,0],[0,1,0],[0,0,1],[1.4,1.7,1.9,3.6],['pod_mount','bench_plate'])
    add('Fork_bar',[[x,z] for x in P.FORK_SCREW_X for z in D.FORK_SCREW_Z],[1,0,0],[0,0,1],[0,1,0],[1.4,1.7,1.9],['inner_fork_6804','outer_fork_6804','fork_bar'])
    patterns['Roller_axles']={'points':[[D.ROW_X[0],D.ROLLER_AXIS_R],[D.ROW_X[1],D.ROLLER_AXIS_R]]}
    for row in (0,1):
        for j in range(W.N):
            a=math.radians(W.PITCH*j+W.PITCH/2*row)
            add(f'Roller_{row+1}_{j+1}',[[D.ROW_X[row],D.ROLLER_AXIS_R]],[1,0,0],[0,math.cos(a),math.sin(a)],[0,-math.sin(a),math.cos(a)],[1.7,3.6,3.34863156,6,18.8,19.3],['wheel_hub_A','wheel_hub_B','wheel_hub_C'],source_pattern='Roller_axles',source_indices=[row])
    add('Pivot',[[-D.PIVOT_YZ[1],D.PIVOT_YZ[0]]],ux,vx,dx,[3.55,4,8,11,11.5],['inner_fork_6804','outer_fork_6804','wheel_guard'])
    add('Shaft_pin',[[D.PIN_X,D.PIVOT_YZ[0]]],[1,0,0],[0,1,0],[0,0,1],[1.4,1.7],['wheel_guard','pivot_shaft'])
    for label,point,parts,radii in [('Inner_lower',D.SHOCK_IN[0],['inner_fork_6804'],[1.4,6.5,7.5]),
                                  ('Inner_upper',D.SHOCK_IN[1],['pod_mount'],[1.7,4.5,6.5]),
                                  ('Outer_lower',D.SHOCK_OUT[0],['outer_fork_6804'],[1.4,6.5,7.5]),
                                  ('Outer_upper',D.SHOCK_OUT[1],['wheel_guard'],[1.4,4.5])]:
        add('Shock_'+label,[[-point[1],point[0]]],ux,vx,dx,radii,parts)
    frames={name:np.eye(4).tolist() for name in V.PARTS}
    frames.update({name:np.eye(4).tolist() for name in ('wheel_hub_A','wheel_hub_B','wheel_hub_C','wheel_cartridge')})
    for name in ('dummy_motor_case','dummy_motor_rear_cap','dummy_motor_rotor','dummy_motor_retainer'):frames[name]=matrix(P.MOTOR_LOC).tolist()
    frames['fork_bar']=matrix(P.fork_bar_loc()).tolist();frames['pivot_shaft']=matrix(P.shaft_loc()).tolist()
    return {'patterns':patterns,'lines':lines,'frames':frames}

if __name__=='__main__':
    p=plan();(ROOT/'reference/shared_holes.json').write_text(json.dumps(p,indent=2));print(len(p['patterns']),'coordinate patterns;',len(p['lines']),'shared hole axes')

"""Export precise source placements for the native assembly, in mm."""
import sys,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent/'v9'))
import build123d as b
import pod_design as V
import pod as P,wheel as W,hw,dims as D

def matrix(loc):
    t=loc.wrapped.Transformation()
    return [[t.Value(i,j) for j in range(1,5)] for i in range(1,4)]+[[0,0,0,1]]

def main():
    rows=[]
    def add(key,part,loc=b.Location(),parent=None,role='rigid'):
        rows.append({'key':key,'part':part,'matrix':matrix(loc),'parent':parent,'role':role})
    add('Bench','bench_plate',role='fixed')
    add('Guard','wheel_guard',parent='Bench')
    add('Tower','pod_mount',parent='Bench')
    add('Pivot_shaft','pivot_shaft',P.shaft_loc(),'Guard')
    add('Inner_pivot_spacer','pivot_shim',b.Pos(D.X_EAR_IN[1],*D.PIVOT_YZ)*hw.rot_to('x'),'Guard')
    add('Outer_pivot_spacer','pivot_outer_spacer',b.Pos(D.X_EAR_OUT[0],*D.PIVOT_YZ)*hw.rot_to('-x'),'Guard')
    add('Inner_fork','inner_fork_6804',role='fork')
    add('Outer_fork','outer_fork_6804',parent='Inner_fork')
    add('Motor_cup','motor_cup',parent='Inner_fork')
    add('Fork_bar','fork_bar',P.fork_bar_loc(),'Inner_fork')
    add('Hub_B','wheel_hub_B',role='wheel')
    for key,part in [('Hub_A','wheel_hub_A'),('Hub_C','wheel_hub_C'),('Cartridge','wheel_cartridge')]:add(key,part,parent='Hub_B')
    for key,part,parent,loc in [('Motor_stator','dummy_motor_stator','Inner_fork',P.MOTOR_LOC),
                                 ('Motor_rotor','dummy_motor_rotor','Hub_B',P.MOTOR_LOC)]:add(key,part,loc,parent)
    add('Motor_bearing','bearing_687',P.MOTOR_LOC*b.Pos(0,0,-7.2),'Inner_fork')
    add('Motor_washer','washer_M3_large',P.MOTOR_LOC*b.Pos(0,0,-10.8),'Hub_B')
    # Wheel hardware and fork hardware, recorded from the same source recipes.
    counters={}
    def hardware(items,parent):
        for it in items:
            key=it.key
            if key.startswith(('M3 x','M2.5 x')):
                size,_,length,kind=key.split()
                # The v9 20 mm wheel joint screws would extend 0.4 mm into
                # the blind-hole floor. An 18 mm screw leaves 1.6 mm spare
                # depth and retains 8.9 mm engagement in the centre hub.
                if parent=='Hub_B' and size=='M3' and length=='20':length='18'
                if size=='M2.5' and length in ('6','8'):length='10'
                part=f'bolt_{size.replace(".","p")}_{length}_{kind}'
            elif key=='M3 hex nut':part='nut_M3_hex'
            elif key=='M3 nyloc nut':part='nut_M3_nyloc'
            elif key=='M3 nylon-insert locknut':part='nut_M3_nyloc'
            elif key=='M3 washer':part='washer_M3'
            elif key=='bearing 6804':part='bearing_6804'
            elif key=='bearing 608-2RS':part='bearing_627'
            else:continue
            counters[part]=counters.get(part,0)+1
            add(f'{part}_{counters[part]}',part,it.loc,parent)
    hardware(W.hardware().items,'Hub_B')
    hardware(P.hardware_unsprung().items,'Inner_fork')
    hardware(P.hardware_sprung().items,'Guard')
    for k,(x,s) in enumerate(P.guard_feet(False)):
        add(f'Guard_foot_bolt_{k+1}','bolt_M3_10_PAN',b.Pos(x,s*(D.HOUSE_Y+4),-.1)*b.Rot(180,0,0),'Bench')
    for k,(x,y) in enumerate(P.MOUNT_SCREWS):
        add(f'Tower_foot_bolt_{k+1}','bolt_M3_12_PAN',b.Pos(x,y,-.1)*b.Rot(180,0,0),'Bench')
    add('Motor_screw','bolt_M3_8_PAN',P.MOTOR_LOC*b.Pos(0,0,-10.8)*b.Rot(180,0,0),'Hub_B')
    for row in (0,1):
        for j,loc in enumerate(W.roller_locs(row)):
            core=f'Roller_{row+1}_{j+1}'
            add(core,'roller_core',loc,role='roller')
            add(core+'_TPU','roller_tread',loc,core)
    for name,(lo,hi),xs in [('Inner',D.SHOCK_IN,D.SHOCK_X),('Outer',D.SHOCK_OUT,D.SHOCK_X_OUT)]:
        ang=-math.degrees(math.atan2(hi[0]-lo[0],hi[1]-lo[1]));mid=sum(xs)/2
        add(name+'_shock_case','shock_case',b.Pos(mid,*lo)*b.Rot(ang,0,0),role='shock_case')
        add(name+'_shock_rod','shock_rod',b.Pos(mid,*hi)*b.Rot(ang,0,0),role='shock_rod')
    data={'parts':rows,'pivot':list(D.PIVOT_YZ),'travel_mm':[-3,15],
          'limits_deg':[90+math.degrees(math.asin((v-D.PIVOT_YZ[1])/math.hypot(*D.PIVOT_YZ))+math.atan2(D.PIVOT_YZ[1],D.PIVOT_YZ[0])) for v in (-3,15)]}
    (ROOT/'reference/assembly_manifest.json').write_text(json.dumps(data,indent=2))
    print('Native assembly manifest:',len(rows),'components',len({r['part'] for r in rows}),'unique parts',data['limits_deg'])
if __name__=='__main__':main()

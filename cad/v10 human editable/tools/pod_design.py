"""v10 integration of the frozen v9 6804 wheel and owned bearing inventory.

All dimensions in mm. The wheel's hubs, cartridge, cores and treads are unchanged.
The stationary support interfaces and test motor are new v10 parts.
"""
from pathlib import Path
import sys, inspect, math
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'v9'))
import build123d as b
import pod as P, dims as D, hw

# 627ZZ has the old 608 seat envelope, with a 7 mm shaft bore.
D.BRG_608=(7.,22.,7.)
P.CUP_SCREWS=[(6.5*math.cos(math.radians(a)),6.5*math.sin(math.radians(a))) for a in (0,90,180,270)]

def revised_function(original,replacements):
    source=inspect.getsource(original)
    source=source.replace('@lru_cache(maxsize=None)\n','')
    for old,new in replacements:
        if old not in source:raise RuntimeError('v9 source changed: '+old)
        source=source.replace(old,new)
    ns=dict(P.__dict__);exec(source,ns)
    return ns[original.__name__]

motor_cup=revised_function(P.motor_cup,[
    ('x_cyl(20.0, rear1 - 0.5, D.X_CUP_IN)','x_cyl(10.0, rear1 - 0.5, D.X_CUP_IN)'),
    ('x_cyl(8.0, rear0 - 0.1, D.X_CUP_IN + 1)','x_cyl(3.0, rear0 - 0.1, D.X_CUP_IN + 1)')])

inner_fork=revised_function(P.arm,[
    ('D.BRG_61808[0] / 2','10.0'),
    ('x_cyl(23.0, D.X_BRG[1], xa0 + 0.5)','x_cyl(11.5, D.X_BRG[1], xa0 + 0.5)'),
    ('x_cyl(8.0, D.X_BRG[0] - 1, xa1 + 1)','x_cyl(3.0, D.X_BRG[0] - 1, xa1 + 1)'),
    ('p -= Box(4.0, 9.0, 12.0, align=(Align.MIN, Align.CENTER, Align.MIN)).moved(Pos(D.X_BRG[0] - 0.1, 0, 6.0))','p -= x_cyl(3.0,D.X_BRG[0]-1,xa1+1)')])

def outer_fork():
    """Outboard cap carries the 6804 INNER race, outside the rotating cartridge.

    Ring ends 0.5 before the cartridge flange; the web keeps its v9 pivot.
    """
    w0,w1=D.X_OPLATE;py,pz=D.PIVOT_YZ;sy,sz=D.SHOCK_OUT[0]
    bz0,bz1=D.FORK_BAR['z'];ly0,ly1=D.FORK_LUG_Y
    p=P.yz_plate(P.hull([(0,0,34),(py,pz,P.PIV_R),(sy,sz,6.5)]+
                [(y,z,.01) for y in (ly0-8,ly1) for z in (bz0,bz1)]),w0,w1)
    p-=b.Box(30,60,60,align=(b.Align.CENTER,b.Align.MIN,b.Align.MAX)).moved(b.Pos((w0+w1)/2,P.OUT_CLEAR[0],P.OUT_CLEAR[1]))
    p+=P.yz_plate(P.hull([(py,pz,P.PIV_R),(py-8,pz-4,P.PIV_R)]),w0,w1)
    # Clearance for ALL of the cartridge's rotating flange, not just its barrel.
    p-=P.x_cyl(33.5,-44.5,w1+1)
    p-=P.x_cyl(22.5,-51.5,-44.49)
    p+=P.x_cyl(34,-54.5,-44.5)-P.x_cyl(22.5,-51.5,-44.4)
    p+=P.x_cyl(10,-52,-43.5)+P.x_cyl(11.5,-52,-50.5)
    p-=P.x_cyl(4,-55,-43)
    # Clearance around the lower shock eye.
    p-=P.yz_cyl(4.5,sy,sz,-55,-51.5)
    p-=hw.edge_chamfer(10,.5).moved(b.Pos(-43.5,0,0)*hw.rot_to('x'))
    p-=P._608_seat(*D.X_PIV_OUT,w1)+P.yz_cyl(8,py,pz,w0-1,D.X_PIV_OUT[0]+.1)
    p+=P.yz_cyl(6.5,sy,sz,D.SHOCK_X_OUT[1]+.5,w0+.5)
    p-=P.yz_cyl(D.TAP_M3/2,sy,sz,D.SHOCK_X_OUT[1]+.49,w1-2)
    p+=hw.cove_ring(6.5,1).moved(b.Pos(w0,sy,sz)*hw.rot_to('-x'))
    for z in D.FORK_SCREW_Z:p-=hw.tap(9,top=ly1,entry='top').moved(b.Pos(P.FORK_SCREW_X[0],0,z)*hw.rot_to('y'))
    p.label='outer_fork_6804';return p

def pivot_shaft():return P.shaft()
def pivot_shim():return b.Cylinder(5.5,.5,align=P.CB)-b.Cylinder(3.6,2)
def pivot_outer_spacer():return b.Cylinder(5.5,2.5,align=P.CB)-b.Cylinder(3.6,5)

# Printable AK40 dummy. Native bearing envelopes are separate components.
# Axis +Z, output contact z=0, rear face z=-40.2, as the real motor envelope.
def dummy_case():
    p=b.Cylinder(26.5,33,align=P.CB).moved(b.Pos(0,0,-36.7))
    p-=b.Cylinder(21.5,25.7,align=P.CB).moved(b.Pos(0,0,-36.71))
    # Front 687 seat, pressed from the output side, outer-race shoulder at -9.
    p-=b.Cylinder(7,5.01,align=P.CB).moved(b.Pos(0,0,-8.7))
    # A swept annular recess clears the output's deeper printed screw bosses
    # at every rotor angle, retaining 3.5 mm around the front bearing seat.
    recess=b.Cylinder(16.5,3.1,align=P.CB).moved(b.Pos(0,0,-6.7))-b.Cylinder(10.5,3.3,align=P.CB).moved(b.Pos(0,0,-6.8))
    p-=recess
    p-=b.Cylinder(4.5,34,align=P.CB).moved(b.Pos(0,0,-37))
    # Rear cover's M3 screws; distinct from the real motor mount holes.
    for angle in (45,135,225,315):
        x,y=D.polar(23.3,angle)
        p+=b.Cylinder(3,11,align=P.CB).moved(b.Pos(x,y,-36.7))
        p-=b.Cylinder(1.4,9,align=P.CB).moved(b.Pos(x,y,-36.71))
    for angle in D.AK_REAR_ANGLES:
        x,y=D.polar(D.AK_REAR_PCD/2,angle)
        p-=b.Cylinder(3.15,5.7,align=P.CB).moved(b.Pos(x,y,-36.71))
    p.label='dummy_motor_case';return p

def dummy_rear_cap():
    p=b.Cylinder(26.5,3.5,align=P.CB).moved(b.Pos(0,0,-40.2))
    # Spigot and rear bearing carrier, leaving room for the rotor retainer.
    p+=b.Cylinder(10.5,6.5,align=P.CB).moved(b.Pos(0,0,-36.7))
    p-=b.Cylinder(7,5.1,align=P.CB).moved(b.Pos(0,0,-35.2))
    p-=b.Cylinder(5,5.1,align=P.CB).moved(b.Pos(0,0,-40.3))
    for angle in (45,135,225,315):
        x,y=D.polar(23.3,angle)
        p-=b.Cylinder(1.7,4,align=P.CB).moved(b.Pos(x,y,-40.3))
        p-=b.Cone(2.8,1.25,1.65,align=P.CB).moved(b.Pos(x,y,-40.2))
    # AK40 rear 4 x M2.5 on PCD47. Printed thread pilots, not metal threads.
    for angle in D.AK_REAR_ANGLES:
        x,y=D.polar(D.AK_REAR_PCD/2,angle)
        p+=b.Cylinder(3.0,5.5,align=P.CB).moved(b.Pos(x,y,-36.7))
        p-=b.Cylinder(1.05,9.1,align=P.CB).moved(b.Pos(x,y,-40.21))
    p.label='dummy_motor_rear_cap';return p

def dummy_rotor():
    p=b.Cylinder(18.5,3.5,align=P.CB).moved(b.Pos(0,0,-3.5))
    p+=b.Cylinder(3.45,32.0,align=P.CB).moved(b.Pos(0,0,-35.5))
    # The front inner race meets the flange via a narrow shoulder, not the seal.
    p+=b.Cylinder(4.5,.2,align=P.CB).moved(b.Pos(0,0,-3.7))
    p-=b.Cylinder(1.4,9,align=P.CB).moved(b.Pos(0,0,-35.51))
    for angle in D.AK_OUT_ANGLES:
        x,y=D.polar(D.AK_OUT_PCD/2,angle)
        p+=b.Cylinder(2.75,3,align=P.CB).moved(b.Pos(y,x,-6.5))
        p-=b.Cylinder(1.05,6.7,align=P.CB).moved(b.Pos(y,x,-6.6))
    p.label='dummy_motor_rotor';return p

def dummy_retainer():
    return b.Cylinder(4.5,1.2,align=P.CB).moved(b.Pos(0,0,-36.7))-b.Cylinder(1.7,2,align=P.CB).moved(b.Pos(0,0,-37))

def bearing_627():return hw.bearing_shape(7,22,7)
def bearing_6804():return hw.bearing_shape(20,32,7)
def bearing_687():return hw.bearing_shape(7,14,5)

def bench_plate():
    # U frame, open outboard. The guard's curved front panel crosses the deck
    # height; a rectangular closed ring would intersect it.
    p=b.Box(188,196,6).moved(b.Pos(24,0,0))-b.Box(142,166,8).moved(b.Pos(-9,0,0))
    panel_clear=(b.Cylinder(D.DECK_R+.5,8)-b.Cylinder(D.DECK_R-5.5,10)).moved(b.Pos(D.L_WHEEL,0,0))
    panel_clear &= b.Box(500,181,8)
    p-=panel_clear
    for x,y in [(x,s*(D.HOUSE_Y+4)) for x,s in P.guard_feet(False)]+P.MOUNT_SCREWS:
        p-=b.Cylinder(1.7,8).moved(b.Pos(x,y,0))
        p-=b.Cylinder(3.6,2.9,align=P.CB).moved(b.Pos(x,y,-3))
    return p

def shock_case():
    p=b.Cylinder(5.5,28.5,align=P.CB).moved(b.Pos(0,0,7))
    p-=b.Cylinder(2.2,24.5,align=P.CB).moved(b.Pos(0,0,12))
    p+=b.Cylinder(2,6,align=P.CB).moved(b.Pos(0,0,3))
    p+=b.Cylinder(4,6,rotation=(0,90,0))
    p-=b.Cylinder(1.6,8,rotation=(0,90,0))
    return p

def shock_rod():
    p=b.Cylinder(2,24,align=P.CB).moved(b.Pos(0,0,-27))
    p+=b.Cylinder(4,6,rotation=(0,90,0))
    p-=b.Cylinder(1.6,8,rotation=(0,90,0))
    return p

PARTS={'motor_cup':motor_cup,'inner_fork_6804':inner_fork,'outer_fork_6804':outer_fork,
       'pivot_shaft':pivot_shaft,'pivot_shim':pivot_shim,'pod_mount':P.pod_mount,'wheel_guard':P.wheel_guard,
       'dummy_motor_case':dummy_case,'dummy_motor_rear_cap':dummy_rear_cap,
       'dummy_motor_rotor':dummy_rotor,'dummy_motor_retainer':dummy_retainer,
       'bearing_627':bearing_627,'bearing_6804':bearing_6804,'bearing_687':bearing_687}
PARTS.update(bench_plate=bench_plate,shock_case=shock_case,shock_rod=shock_rod)
PARTS['pivot_outer_spacer']=pivot_outer_spacer

"""Standalone wheel contracts; no robot assemblies or simulation runs."""
import math
import pytest
from build123d import Compound, Pos, Rot
import hw
import wheel as W
from OCP.BRepAdaptor import BRepAdaptor_Surface


@pytest.mark.parametrize('factory',[W.roller_core,W.roller_tread,W.bearing_cartridge]+[lambda k=k:W.hub(k) for k in 'ABC'])
def test_connected_prints(factory):
    p=factory()
    assert p.is_valid and len(p.solids())==1


def test_owned_bearings_and_removable_motor_access():
    assert W.BRG==(20,32,7)
    assert sum(i.key=='bearing 6804' for i in W.hardware().items)==2
    assert any(p[0]=='wheel_cartridge' for p in W.printed())


def test_nut_is_captive_in_hex_pocket():
    host=Compound([W.hub(k) for k in 'ABC'])
    loc=W.roller_locs(0)[0]*Pos(0,0,23.5)
    nut=hw._nut_shape('hex')
    assert (nut.moved(loc)&host).volume<.05
    assert (nut.moved(loc*Rot(0,0,30))&host).volume>.1


@pytest.mark.parametrize('name,row,side',[('A',0,-1),('B',0,1),('B',1,-1),('C',1,1)])
def test_cheek_fillets_add_material_at_both_web_roots(name,row,side):
    hub=W.hub(name)
    # Points just outside the old sharp-ended pocket were empty before the
    # root blend. Check actual added PLA at both roots of every cheek half.
    for loc in W.roller_locs(row):
        for sign in (-1,1):
            point=(loc*Pos(side*11.5,-11.76,sign*20.8)).position
            assert hub.is_inside(point)
    radii=[BRepAdaptor_Surface(f.wrapped).Torus().MinorRadius()
           for f in hub.faces() if f.geom_type.name=='TORUS']
    assert sum(abs(r-W.WEB_ROOT_FILLET)<1e-5 for r in radii)==(24 if name=='B' else 12)


@pytest.mark.parametrize('name,row,side',[('A',0,-1),('B',0,1),('B',1,-1),('C',1,1)])
def test_triangular_support_has_no_end_relief_slot_underneath(name,row,side):
    hub=W.hub(name)
    # Sample through the support's depth, on both roller ends. These points
    # were cut away by the old barrel-width relief beyond the core's cone.
    for loc in W.roller_locs(row):
        for sign in (-1,1):
            for depth in (1,5,10):
                assert hub.is_inside((loc*Pos(side*depth,-13,sign*20.8)).position)


@pytest.mark.parametrize('sign',[-1,1])
@pytest.mark.parametrize('row',[0,1])
def test_rigid_journal_stops_axial_motion_before_tpu(sign,row):
    host=Compound([W.hub(k) for k in 'ABC'])
    loc=W.roller_locs(row)[0]
    core=W.roller_core()
    assert (core.moved(loc*Pos(0,0,sign*W.END_FLOAT/2))&host).volume<.05
    assert (core.moved(loc*Pos(0,0,sign*(W.END_FLOAT/2+.1)))&host).volume>.1
    assert (W.roller_tread().moved(loc*Pos(0,0,sign*W.END_FLOAT/2))&host).volume<.05


@pytest.mark.parametrize('row',[0,1])
@pytest.mark.parametrize('shift',[-W.END_FLOAT/2,0,W.END_FLOAT/2])
def test_every_roller_body_clears_finished_hubs_at_full_end_travel(row,shift):
    host=Compound([W.hub(k) for k in 'ABC']+[W.bearing_cartridge()])
    # Axisymmetric envelopes cover every roller angle, including the material
    # between core keyways. Exclude only the intended journal/thrust contact.
    body=W._revolve(W._profile(core=True))
    tread=W._revolve(W._profile(tread=True))
    for loc in W.roller_locs(row):
        at=loc*Pos(0,0,shift)
        assert body.moved(at).distance_to(host)>=W.MIN_BODY_CLEARANCE-1e-6
        assert tread.moved(at).distance_to(host)>=W.MIN_BODY_CLEARANCE-1e-6


def test_round_tread_still_covers_contact_transition():
    assert W.HL-W.CONE>W.R*math.sin(math.pi/(2*W.N))


def test_stock_axle_hardware():
    items=W.hardware().items
    assert sum(i.key=='M3 x 50 PAN' for i in items)==12
    assert sum(i.key=='M3 hex nut' for i in items)==12

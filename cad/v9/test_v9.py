"""Mechanical contracts for v9's first physical wheel and the compact base."""
import math

import pytest
from build123d import Plane

import base
import dims as D
import pod as P
import vendor
import wheel as W


@pytest.mark.parametrize("which", "ABC")
def test_hub_is_one_valid_print_and_has_nonhanded_motor_pattern(which):
    shape = W.hub(which)
    assert shape.is_valid and len(shape.solids()) == 1
    # The same wheel must mate to the motor on both handed forks. An odd-row
    # mirror rotation inherited from v8 would misalign the output bolt pattern.
    assert (shape - shape.mirror(Plane.XZ)).volume < 0.05


def test_roller_rows_clear_each_other_with_print_tolerance():
    env = W._roller_env(0.4)
    first = env.moved(W.roller_locs(0)[0])
    others = W.roller_locs(0)[1:] + W.roller_locs(1)
    assert min(first.distance_to(env.moved(loc)) for loc in others) > 0.25


@pytest.mark.parametrize("factory", [W.roller_core, W.roller_tread, P.motor_cup, P.arm, P.outer_plate])
def test_parts_remain_connected(factory):
    shape = factory()
    assert shape.is_valid and len(shape.solids()) == 1


def test_output_and_rear_screws_cannot_bottom_in_maker_blind_holes():
    output = 6 - (D.X_DRIVE_PLATE[1] - D.X_DRIVE_PLATE[0])
    rear = 8 - (D.X_REAR_BRIDGE[1] - D.X_REAR_BRIDGE[0])
    assert 2.0 <= output <= 2.5  # maker's 3 mm holes, 0.5 mm minimum reserve
    assert 2.0 <= rear <= 4.0    # maker's 5 mm holes, 1 mm reserve


@pytest.mark.parametrize("bump", [-3, 0, 3, 6, 9, 12, 15])
def test_actual_axle_height_and_shock_stroke(bump):
    assert P.arm_motion(bump).position.Z == pytest.approx(bump, abs=1e-8)
    for eyes in (D.SHOCK_IN, D.SHOCK_OUT):
        assert 40.0 <= P._shock_len(*eyes, bump) <= 50.0


def test_lidar_and_payload_clear_the_larger_wheel_at_full_bump():
    assert D.LIDAR_SCAN_Z - (2 * D.WHEEL_R + D.POD_TRAVEL) >= 8
    assert D.PLATE_BOT - (D.LIDAR_SCAN_Z + D.A2M12["h"] - D.A2M12["scan_z"]) >= 2


def test_motor_inserts_without_scraping_its_cradle():
    motor = vendor.ak40_10().moved(P.MOTOR_LOC)
    from build123d import Pos
    for x in range(0, -66, -4):
        assert (motor.moved(Pos(x, 0, 0)) & P.motor_cup()).volume < 0.05


def test_inner_bearing_has_a_printable_retaining_shoulder():
    # The 52 mm race seats at x=37; the shoulder begins 2.5 mm earlier.
    from build123d import Cylinder, Pos
    probe = Cylinder(25.8, 2.0, rotation=(0, 90, 0)).moved(Pos(D.X_BRG[0] - 1.3, 0, 0))
    probe -= Cylinder(25.0, 4.0, rotation=(0, 90, 0)).moved(Pos(D.X_BRG[0] - 1.3, 0, 0))
    assert (probe & W.hub("C")).volume / probe.volume > 0.99


def test_rear_bumper_routes_round_mast_as_one_connected_print():
    bumper = base.bumper("lobe_rear")
    assert bumper.is_valid and len(bumper.solids()) == 1
    assert 2 * (D.DECK_R + D.BUMPER_T) < 462


def test_round_barrel_covers_a_complete_staggered_roller_pitch():
    # Beyond this angle the coned ends must not form the wheel's contact surface.
    half_contact_angle = math.pi / (2 * D.ROLLERS_PER_ROW)
    assert D.ROLLER_HALF - D.ROLLER_CONE > D.WHEEL_R * math.sin(half_contact_angle)


def test_real_motor_connectors_keep_the_same_unmirrored_cradle():
    import assembly
    robot = assembly.build(0, 0, "pods")
    assert robot.printed["motor_cup"].qty == 4
    assert "motor_cup_M" not in robot.printed


def test_stock_axles_replace_cut_pins():
    items = W.hardware().items
    assert sum(i.key == "M3 x 50 PAN" for i in items) == 12
    assert sum(i.key == "M3 hex nut" for i in items) == 12
    assert not any(i.key.startswith("pin 3") for i in items)
    assert W.roller_core().bounding_box().size.Z == pytest.approx(39.0)


def test_axle_engages_whole_standard_nut_without_clamping_roller():
    items = W.hardware().items
    axle = next(i for i in items if i.key == "M3 x 50 PAN")
    nut = next(i for i in items if i.key == "M3 hex nut")
    frame = W.roller_locs(0)[0].inverse()
    screw_end = (frame * axle.loc).position.Z + 50
    nut_end = (frame * nut.loc).position.Z + nut.shape.bounding_box().max.Z
    assert screw_end - nut_end >= 0.5

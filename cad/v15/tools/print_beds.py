"""cad/v11: how each printed part is meant to go on the bed (Nov 4 2026). One entry per part (key prefix): the axis that points UP while
printing, in the part's own STEP frame ('+x' = the part's +x up, its lowest x face on the bed), and why. Layers are planes normal to that
axis: they are strongest in their plane and weakest across (layer adhesion), so the main loads should lie in the layer planes.
sw_robot.py puts a reference plane PRINT_BED in each printed SolidWorks part at its bed face; the same table is README's print list.
"""
BEDS = [  # (key prefix, up axis, supports, why)
    # pod (pod frame: x = wheel axis)
    ('pod_frame_inner', '-x', 'none (round hole roofs only; the pivot pin stands up on top)', 'on its flat outer face: the springs push the tower up and the pivot pulls the keel down, both in the layer planes; Nov 13: the inner pivot pin prints with it (upright, on top) with an M3 x 16 threaded along its axis as its steel core (no dowel), and the four chassis screws thread into Ø2.9 pilots (no captive nuts; round 13-14 had hex nuts in slots)'),
    ('pod_keel_outer', '+x', 'none (hole roofs)', 'on its flat outer face: the pivot load (mostly z) and the lateral load bend the side plate in the layer planes; two M3 x 35 from under the chassis into Ø2.9 pilots through the whole keel (round 14, Fernando: a 2.9 pilot is gentle enough not to split the layers it lies in; round 13 had hex nuts in a slot)'),
    ('pod_fork_outer', '+x', 'only the two tie-bolt head seats in the bed face (the pilots run straight through)', 'on the arm\'s outer face: the arm\'s bending is in the layer planes; the base bar prints upright (layers across it), so its tie bolts pre-compress those layers'),
    ('pod_fork_inner', '-x', "only inside the 2 mm motor pocket (Ø53 roof); the tenon socket's roof is a 9.2 mm bridge; the zip-tie tunnel is round (push its support out)", 'v15: on its outer face (x 54.7), as pod_rx.motor_plate: motor torque and the spring pads load it in the layer planes'),
    # v15 mid-deck pod (pod_mid.py)
    ('rx_hanger', '+z', 'none (the pivot boss and holes: round roofs)', 'upright on its underside (Nov 14): the plan taper (14 mm under the tower, 8 behind the pivot) prints free; the long bolts and screws run along the print axis'),
    ('rx_beam', '-z', "none (the bosses' round undersides)", "top face down, the droop lug standing up: the lug bolt runs along the print axis and keeps the lug's layers in compression"),
    ('rx_tower_outer', '+x', "none (the spring bores' round roofs)", 'on its outer face: the spring force runs down the tower in the layer planes'),
    ('rx_tower_inner', '-x', "none (the spring bores' round roofs)", 'as rx_tower_outer, mirrored'),
    ('rx_ring_outer', '+x', 'none', 'flat, countersinks up'),
    ('rx_ring_inner', '-x', 'none', 'flat, countersinks up'),
    ('rx_shim', '+x', 'none', 'flat (0.4 / 0.6 / 0.8 mm)'),
    ('deck_FL', '+z', 'none (counterbore and hole roofs)', 'v15 mid deck: flat; the pods hang under it and the towers stand on it, both loads in compression across the layers, bending in their planes'),
    ('deck_FR', '+z', 'none (counterbore and hole roofs)', 'as deck_FL'),
    ('deck_RL', '+z', 'none (counterbore and hole roofs)', 'as deck_FL'),
    ('deck_RR', '+z', 'none (counterbore and hole roofs)', 'as deck_FL'),
    ('pod_sleeve', '-x', 'none', 'on the end of its collar (Nov 13 evening: the collar replaced the flange)'),
    ('pod_retainer', '+x', 'none (the flange overhangs the spigot by 1.6 mm)', 'Nov 13: the 6804 retainer, on its spigot: the clamp of its screw loads it along its axis'),
    ('pod_pivot_cap', '+x', 'none', 'Nov 13 evening: the outer pivot retainer cap, on its flange: the screw clamp loads it along its axis'),
    ('pod_pin_cap', '+x', 'none', 'Nov 13 evening: the inner pivot cap, countersink up: the flat head loads it along its axis'),
    ('pod_brg_ring', '+x', 'only the two counterbore roofs (Ø7.2 over Ø3.4)', 'Nov 13 evening: the frame 627 clamp ring, on its flat face: the screws clamp it along its axis'),
    # wheel (pod frame / roller frame)
    ('wheel_hub', '+x', 'under the spokes\' flat x faces over the open roller pockets and the flange (README); none in the channel between the rows (collar)', 'cartridge end down: wheel load and roller axles load it radially, in the layer planes'),
    ('wheel_cartridge', '+x', 'none', 'plate down (no floor, no trapped cavity)'),
    ('roller_core', '+z', 'none', 'upright on its end: the bearing load is radial, in the layer planes; the 45 deg V groove prints without support; the pocket floors are open rings'),
    ('roller_sleeve', '+z', 'none', 'upright on its end: the axle load is radial, in the layer planes; the 0.8 mm shoulder steps are short ledges'),
    ('roller_tread', '+z', 'none', 'upright on its end (TPU)'),
    # chassis (robot frame)
    ('hub', '+z', 'none', 'members flat: plan loads and bending in their length lie in the layers; pocket walls bend across layers (screws carry the pods, the walls only locate)'),
    ('edge', '+z', 'none', 'as hub'),
    ('node', '+z', 'none', 'as hub; the keel pocket walls (24 tall) bend across the layers: located by them, held by the keel\'s two M3 x 35 into its Ø2.9 pilots; vertical pilots (bumper tabs, laps) along the layers\' normal'),
    ('pedestal', '+z', 'none', 'on its flange: the lidar presses down along the tube'),
    ('bracket_d435_riser', '+z', 'none (back plate 30 deg from vertical; Nov 13: the nuts in hex recesses on the inner faces of the ears, corner up)', 'v13: on its feet (the riser\'s foot plate): the camera hangs on the back plate and the side screws pull the ears toward the walls, both in the layer planes; the camera\'s M3 holes are clearance holes'),
    ('bracket_d435', '+z', 'none (back plate 7.5 deg from vertical, wings chamfered 45 deg under)', 'on its base: the webs carry the camera down onto the node in the layer planes'),
    ('bumper', '-z', 'none', 'v15 Nov 14: upside down, its top edge and the tabs (under the deck rim) on the bed, the gussets 45 deg (a 3 mm arc can only print upright): impacts bend the wall across the layers between its tabs: PETG, VERIFY by a drop test'),
    ('support', '-z', 'none', 'upside down on the flange: the floor\'s weight is compression down the legs; lifting the robot by the second floor would pull the legs across the layers: lift it by the chassis'),
    # upper (robot frame)
    ('floor2', '-z', 'none', 'top face down: the lips and bosses on its underside grow up from the plate'),
    ('riser_arm', '+z', 'none', 'on its foot (v10 part): the arm\'s moment pulls one side up across the layers; the adapter screws reach into its lowest 10 mm (see README)'),
    ('riser_adapter', '-z', 'none (the underside recess opens upward)', 'top face down (v10 part)'),
    ('bracket_computer', '+z', 'none', 'on its tab'),
    ('bracket_imu', '+z', 'none', 'on its base: the two standoffs grow up from it, their pilots vertical'),
    ('spacer_servo_adapter', '+x', 'none', 'standing on its end (a short tube along robot x)'),
    ('bracket_servo_adapter', '-x', 'none', 'flat on its riser face: its board pilots and mounting holes run along the print axis'),
    ('bracket_battery', '+z', 'bridges over the strap slots (22 mm)', 'on its floor'),
    ('post', '+z', 'none', 'standing: compression'),
    ('deck_bin', '+z', 'none (hole roofs)', 'flat'),
    ('housing_face_crown', '+z', 'none', 'on its plate: the speaker cup, bosses and mic standoffs grow up from it'),
    ('housing_face', '+x', 'none (the panel slot\'s roof is the 3 mm bezel lip, bridged)', 'on its back plate: legs, rails and back in the layer planes, so the panel\'s weight and a knock bend it in-plane'),
    ('housing_speaker_cover', '+z', 'none', 'flat (grille plate)'),
    ('housing_estop', '+z', 'none (the 22 mm hole\'s roof is a bridge)', 'on its base plate: a press on the button is compression down the tube'),
    ('housing_switch', '+z', 'none (the 22 mm hole\'s roof is a bridge)', 'as housing_estop'),
    ('bin', '+z', 'none', 'open side up (v10 part)'),
]


def bed(key):
    """(up axis, supports, why) for a part key (mirrored '_m' parts the same: the mirror is about y), or None for a bought part"""
    k = key.removesuffix('_m')
    for pre, up, sup, why in BEDS:
        if k.startswith(pre):
            return up, sup, why
    return None

"""cad/v11: how each printed part is meant to go on the bed (Nov 4 2026). One entry per part (key prefix): the axis that points UP while
printing, in the part's own STEP frame ('+x' = the part's +x up, its lowest x face on the bed), and why. Layers are planes normal to that
axis: they are strongest in their plane and weakest across (layer adhesion), so the main loads should lie in the layer planes.
sw_robot.py puts a reference plane PRINT_BED in each printed SolidWorks part at its bed face; the same table is README's print list.
"""
BEDS = [  # (key prefix, up axis, supports, why)
    # pod (pod frame: x = wheel axis)
    ('pod_frame_inner', '-x', 'none (round hole roofs only)', 'on its flat outer face: the springs push the tower up and the pivot pulls the keel down, both in the layer planes; the M3 x 60 column bolt clamps the column to the foot along z too'),
    ('pod_keel_outer', '+x', 'none (hole roofs)', 'on its flat outer face: the pivot load (mostly z) and the lateral load bend the side plate in the layer planes; two M3 x 30 from under the chassis hold its foot'),
    ('pod_fork_outer', '+x', 'none', 'on the arm\'s outer face: the arm\'s bending is in the layer planes; the base bar prints upright (layers across it), so its tie bolts pre-compress those layers'),
    ('pod_fork_inner', '+x', 'none (45 deg socket roof)', 'on the motor face: motor torque and the spring pads load it in the layer planes'),
    ('pod_sleeve', '+x', 'none', 'on its flange'),
    # wheel (pod frame / roller frame)
    ('wheel_hub', '+x', 'under the spokes\' flat x faces (README)', 'cartridge end down: wheel load and roller axles load it radially, in the layer planes'),
    ('wheel_cartridge', '+x', 'none', 'plate down (no floor, no trapped cavity)'),
    ('roller_core', '+z', 'none', 'upright on its end: the axle load is radial, in the layer planes; the 45 deg V groove prints without support'),
    ('roller_tread', '+z', 'none', 'upright on its end (TPU)'),
    # chassis (robot frame)
    ('hub', '+z', 'none', 'members flat: plan loads and bending in their length lie in the layers; pocket walls bend across layers (screws carry the pods, the walls only locate)'),
    ('edge', '+z', 'none', 'as hub'),
    ('node', '+z', 'none', 'as hub; the keel pocket walls (24 tall) bend across the layers: located by them, held by the keel\'s two M3 x 30'),
    ('pedestal', '+z', 'none', 'on its flange: the lidar presses down along the tube'),
    ('bracket_d435', '+z', 'none (back plate 30 deg from vertical)', 'on its base'),
    ('bumper', '+z', 'none', 'standing on its lower edge (a 3 mm arc can only print upright): impacts bend the wall across the layers between its screws: PETG, VERIFY by a drop test'),
    ('support', '-z', 'none', 'upside down on the flange: the floor\'s weight is compression down the legs; lifting the robot by the second floor would pull the legs across the layers: lift it by the chassis'),
    # upper (robot frame)
    ('floor2', '-z', 'none', 'top face down: the lips and bosses on its underside grow up from the plate'),
    ('riser_arm', '+z', 'none', 'on its foot (v10 part): the arm\'s moment pulls one side up across the layers; the adapter screws reach into its lowest 10 mm (see README)'),
    ('riser_adapter', '-z', 'none (the underside recess opens upward)', 'top face down (v10 part)'),
    ('bracket_computer', '+z', 'none', 'on its tab'),
    ('bracket_battery', '+z', 'bridges over the strap slots (22 mm)', 'on its floor'),
    ('post', '+z', 'none', 'standing: compression'),
    ('deck_bin', '+z', 'none (hole roofs)', 'flat'),
    ('housing_face_crown', '+z', 'none', 'on its plate: the speaker cup, bosses and mic standoffs grow up from it'),
    ('housing_face', '+x', 'none (the panel slot\'s roof is the 3 mm bezel lip, bridged)', 'on its back plate: legs, rails and back in the layer planes, so the panel\'s weight and a knock bend it in-plane'),
    ('housing_speaker_cover', '+z', 'none', 'flat (grille plate)'),
    ('housing_estop', '+z', 'none (the 22 mm hole\'s roof is a bridge)', 'on its base plate: a press on the button is compression down the tube'),
    ('housing_switch', '+z', 'none (the 22 mm hole\'s roof is a bridge)', 'as housing_estop'),
    ('clamp_so101', '+x', 'none', 'on its side: the L (heel and toe) lies in the layer plane, so the clamping bend is in-plane'),
    ('bin', '+z', 'none', 'open side up (v10 part)'),
]


def bed(key):
    """(up axis, supports, why) for a part key (mirrored '_m' parts the same: the mirror is about y), or None for a bought part"""
    k = key.removesuffix('_m')
    for pre, up, sup, why in BEDS:
        if k.startswith(pre):
            return up, sup, why
    return None

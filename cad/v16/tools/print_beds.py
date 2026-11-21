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
    ('rx_hanger_outer', '+x', 'none (the up-screws\' holes and counterbores: round roofs; the alignment boss a 1.5 mm ledge)', 'Nov 15 (Fernando: holes perpendicular to the bed): on its outer face, so the pivot boss, its Ø7.2 spigot and the pivot screw\'s snug hole stand vertical (round and true); the pivot pull and the long bolts\' clamp load the plate in its layer planes'),
    ('rx_hanger_inner', '-x', 'none (as rx_hanger_outer)', 'as rx_hanger_outer, on its outer face'),
    ('rx_beam', '+x', "none (the screws' holes and counterbores: round roofs; the alignment boss a 1.5 mm ledge)", 'Nov 15: standing on its outer end, so the pivot line (Ø2.9, both pivot screws) and the inner boss\'s spigot stand vertical; the outer boss is the separate rx_pin and the droop lug the separate rx_lug, so the end is flat and nothing hangs'),
    ('rx_pin', '-x', 'none (the Ø7.2 spigot stands on the Ø10 flange)', 'Nov 15: the beam\'s outer pivot boss, on its flange: the spigot vertical; the pivot screw clamps it along its axis'),
    ('rx_lug', '+x', 'none (the lug bolt\'s hole and counterbore: round roofs)', 'Nov 15: the droop lug, on its outer face: the fork bar\'s push bends it in the layer planes (it used to bend across them)'),
    ('rx_tower_outer', '-z', 'under the two 12 mm stub tabs (their tops, the nut faces, hang ~21 mm over the bed); the bores, guides and counterbore open from the bed', 'Nov 15 (Fernando: holes perpendicular to the bed): top down, so the spring bores, plunger guides and every screw hole stand vertical; the overhang\'s underside and the bump rails face up; the spring force compresses the tower along the print axis'),
    ('rx_tower_inner', '-z', 'as rx_tower_outer (under the stub tabs)', 'as rx_tower_outer'),
    ('coupon_dovetail', '+z', 'none', 'Nov 15 evening: flat, as the deck pieces (their laps both start at the bed)'),
    ('deck_key', '+z', 'none', 'Nov 15 evening: the four-arm key at the deck seams\' crossing, flat: the pull across a seam is in the layer plane'),
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
    ('wheel_cartridge_cap', '+x', 'none', 'Nov 15: the flat cap, either face down'),
    ('wheel_cartridge', '+x', 'none (the lip: a 1.55 mm ring overhang at the top)', 'Nov 15: the cup, flange down: the 6804 seat\'s axis vertical (round and true)'),
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
    ('support', '-z', 'none (the arches and the generated lower edge open upward)', 'v15 (fins.py): upside down on the flange: the floor\'s weight is compression down the feet; the feet\'s M3 pilots run along the print axis; lifting the robot by the second floor would pull the feet across the layers: lift it by the deck'),
    ('bracket_battery_clip', '-z', 'none', 'v15: on its tab (the face against the deck\'s underside), the two walls standing up: the pack bears on the walls sideways, in the layer planes; its M3 runs along the print axis'),
    # upper (robot frame)
    ('floor2', '-z', 'none', 'top face down: the lips and bosses on its underside grow up from the plate'),
    ('riser_arm', '+z', 'none', 'on its foot (v10 part): the arm\'s moment pulls one side up across the layers; the adapter screws reach into its lowest 10 mm (see README)'),
    ('riser_adapter', '-z', 'none (the underside recess opens upward)', 'top face down (v10 part)'),
    ('bracket_computer', '+z', 'none', 'on its tab'),
    ('bracket_power_box', '+z', 'none (open top; the ears sit on the bed)', 'Nov 16: the power stack\'s tray, on its floor: the modules\' weight and the corner screws\' clamp press across the layers, the walls stand in the layer planes; the ear holes run along the print axis'),
    ('bracket_power_shelf', '+z', 'none', 'Nov 16: flat plate, as printed it lies on the box\'s posts: its corner holes run along the print axis'),
    ('bracket_conv_', '+z', 'none (the wall notches under the ears are open at the top)', 'Nov 18: a converter cradle, on its plate: the module\'s weight and the screws\' clamp press across the layers; the screw holes run along the print axis'),
    ('bracket_anti_spark', '+z', 'none', 'Nov 18: a cradle, on its plate (as bracket_conv_)'),
    ('bracket_xt30_block', '+z', 'none', 'Nov 18: a cradle, on its plate (as bracket_conv_)'),
    ('bracket_ina3221', '+z', 'none', 'Nov 18: a cradle, on its plate (as bracket_conv_)'),
    ('bracket_usb_hub', '+z', 'none', 'Nov 18: a cradle, on its plate (as bracket_conv_)'),
    ('tray_carrier', '+z', 'none', 'Nov 18: the carrier\'s tray, on its plate: the four standoffs grow up from it, their pilots vertical'),
    ('bracket_imu', '+z', 'none', 'on its base: the two standoffs grow up from it, their pilots vertical'),
    ('spacer_servo_adapter', '+x', 'none', 'standing on its end (a short tube along robot x)'),
    ('bracket_servo_adapter', '-x', 'none', 'flat on its riser face: its board pilots and mounting holes run along the print axis'),
    ('bracket_battery', '+z', 'bridges over the strap slots (22 mm)', 'v14 tray (v15: the clips above)'),
    ('post', '+z', 'none', 'standing: compression'),
    ('deck_bin', '+z', 'none (hole roofs)', 'flat'),
    ('housing_face_crown', '+z', 'none', 'on its plate: the speaker cup, bosses and mic standoffs grow up from it'),
    ('housing_face', '+x', 'none (the panel slot\'s roof is the 3 mm bezel lip, bridged)', 'on its back plate: legs, rails and back in the layer planes, so the panel\'s weight and a knock bend it in-plane'),
    ('housing_speaker_cover', '+z', 'none', 'flat (grille plate)'),
    ('housing_estop', '+z', 'none (the 22 mm hole\'s roof is a bridge)', 'on its base plate: a press on the button is compression down the tube'),
    ('housing_switch', '+x','none (the base plate and webs grow up from the panel; the Ø27 recess opens upward)', 'Nov 18: the main '
     'switch\'s housing on floor 2, on its panel\'s outer face: the Ø20.2 hole runs along the print axis (round, for the snap fit); a press '
     'on the rocker bends the panel in its layer planes, the base\'s root across them (small load; the webs carry it)'),
    ('bin', '+z', 'none', 'open side up (v10 part)'),
]


BED_MM = 256.0         # the printer's bed, square (Bambu, 256 x 256; Fernando, Nov 16)
BED_MARGIN_MM = 2.0    # kept free on each side (skirt, purge line, placement): a part must fit BED_MM - 2 * BED_MARGIN_MM


def bed_fit(shape, up):
    """does the printed part fit the bed, printed with `up` up ('+z', '-x'... or a vector)? The footprint is the real solid's (tessellated,
    every feature: tabs, lugs, flanges), not a plan outline. Returns its height, its footprint as exported (the part's own axes, the way it
    lands in the slicer), and the footprint at the best rotation about the up axis (the slicer can turn it), with that angle.
    fits: the best rotation inside BED_MM - 2 * BED_MARGIN_MM; fits_as_exported: the same without turning it (cad/solidworks.md 18)"""
    import numpy as np
    from shapely.geometry import MultiPoint
    from shapely import affinity
    if isinstance(up, str):
        u = np.zeros(3)
        u['xyz'.index(up[1])] = -1.0 if up[0] == '-' else 1.0
    else:
        u = np.asarray(up, float) / np.linalg.norm(up)
    ax = int(np.argmax(np.abs(u)))
    if abs(abs(u[ax]) - 1.0) < 1e-9:                                   # an axis up: the other two axes, in order, as the slicer shows them
        e1, e2 = (np.eye(3)[i] for i in range(3) if i != ax)
    else:
        e1 = np.cross(u, np.eye(3)[(ax + 1) % 3]); e1 /= np.linalg.norm(e1); e2 = np.cross(u, e1)
    v, _ = shape.tessellate(0.05, 0.2)
    P = np.array([[q.X, q.Y, q.Z] for q in v])
    h = P @ u
    xy = np.stack([P @ e1, P @ e2], 1)
    hull = MultiPoint([tuple(p) for p in xy]).convex_hull
    best = None
    for a2 in range(0, 360):                                           # 0.5 deg steps over 180 deg
        b = affinity.rotate(hull, a2 / 2.0, origin=(0, 0)).bounds
        w, d = b[2] - b[0], b[3] - b[1]
        if best is None or max(w, d) < best[0]:
            best = (max(w, d), min(w, d), a2 / 2.0)
    ex = xy.max(0) - xy.min(0)
    lim = BED_MM - 2 * BED_MARGIN_MM
    return dict(height=round(float(h.max() - h.min()), 1), as_exported=[round(float(ex[0]), 1), round(float(ex[1]), 1)],
                best=[round(best[0], 1), round(best[1], 1)], rotate_deg=best[2], limit=lim,
                fits=bool(best[0] <= lim), fits_as_exported=bool(max(ex) <= lim), spare_mm=round(lim - best[0], 1))


def bed_failures(report):
    """the parts of a {name: {..., 'bed': bed_fit(...)}} report that do not fit the bed, as printable lines"""
    return [f"{k}: {v['bed']['best'][0]:.1f} mm at its best rotation > {v['bed']['limit']:.0f}" for k, v in report.items()
            if isinstance(v.get('bed'), dict) and not v['bed']['fits']]


def bed(key):
    """(up axis, supports, why) for a part key (mirrored '_m' parts the same: the mirror is about y), or None for a bought part"""
    k = key.removesuffix('_m')
    for pre, up, sup, why in BEDS:
        if k.startswith(pre):
            return up, sup, why
    return None

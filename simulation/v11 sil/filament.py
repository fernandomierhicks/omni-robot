"""The arm robot's filament colours (Nov 13, Fernando baselined; cad/v13 README "Filament colours", cad/v13/tools/palette.py): Bambu PLA
Basic Jade White #ffffff, Dark Gray #545454, Orange #ff6a13 (hex from Bambu's swatch) and black TPU treads, as MuJoCo materials.

SimParams.colours "auto" gives the arm robot (payload "arm") these; the catch robot, and colours="legacy" (the pre-Nov 13 renders: a yellow
SO-101, an orange bin, riser and adapter, dark decks), keep the earlier materials, which reproduce earlier results. The materials are new
names; the shared ones (petg_dark, petg_orange, so101_print, ...) are unchanged.

Where the sim's coarse arm robot takes each (robot_model.py, arm_so101.py):
    white   the deck and its top (the chassis / first floor), the payload plate (second floor), the lidar pedestal, the bin, the arm's
            riser and adapter (and the nose bracket of arm_mount_z layouts)
    gray    the printed SO-101 (its servos stay black), the pods (fork plates, the guard's ears, wheel guards), the battery tray, the
            e-stop's housing (its red button stays)
    orange  the wheel hubs and their mid-plane disc, the bumper ring and skirt
    tpu     the rollers (one mesh: the tread is what shows)
Bought parts (servos, motors, cameras, lidar, display, electronics, battery) keep their materials.
"""


def _hex(h: str) -> str:
    h = h.lstrip("#")
    return " ".join(f"{int(h[i:i + 2], 16) / 255:.4g}" for i in (0, 2, 4))


MATERIALS_XML = (f'<material name="fil_white" rgba="{_hex("#ffffff")} 1" specular="0.2" shininess="0.3"/>'
                 f'<material name="fil_gray" rgba="{_hex("#545454")} 1" specular="0.2" shininess="0.3"/>'
                 f'<material name="fil_orange" rgba="{_hex("#ff6a13")} 1" specular="0.25" shininess="0.35"/>'
                 '<material name="fil_tpu" rgba="0.06 0.06 0.065 1" specular="0.1"/>')


def on(p) -> bool:
    """the arm robot in the Nov 13 filament colours"""
    return p.payload == "arm" and getattr(p, "colours", "auto") == "auto"


def mat(p, legacy: str, filament: str) -> str:
    """the material name: the filament's ('white', 'gray', 'orange', 'tpu') on the arm robot, else the earlier one"""
    return f"fil_{filament}" if on(p) else legacy

"""The arm robot's filament colours (Nov 13, Fernando: baselined; cad/v14 README "Filament colours" is the source of truth): three Bambu
PLA Basic colours plus black TPU, hex sampled from Bambu's swatch, written to STEP as sRGB (step_colour.py).

    filament(name) -> 'white' | 'gray' | 'orange' | 'tpu' | None     (None: not a printed part of the robot: keep the caller's colour)
    colour(name)   -> its (r, g, b) or None

Names are the generating scripts' part keys; a group prefix ('chassis/', 'FL/'), leading underscores (unplaced copies) and the mirrored
hand's '_m' suffix do not matter (first matching prefix wins). Bought parts (servos, AK40-10s, D435, display, electronics, battery, the
red e-stop button), fasteners (magenta) and mechanical items (bearings, springs, leads, plugs, envelopes) return None.
"""


def _hex(h):
    h = h.lstrip('#')
    return tuple(round(int(h[i:i + 2], 16) / 255.0, 4) for i in (0, 2, 4))


WHITE = _hex('#ffffff')                   # Jade White
DARK_GRAY = _hex('#545454')               # Dark Gray
ORANGE = _hex('#ff6a13')                  # Orange
TPU_BLACK = _hex('#1a1a1a')               # black TPU (no swatch hex: a near black)
FILAMENTS = dict(white=WHITE, gray=DARK_GRAY, orange=ORANGE, tpu=TPU_BLACK)

# (prefix, filament): first match wins, so the exceptions come before the general prefixes
RULES = (
    ('arm_STS3215', None),                                             # the SO-101's servos are bought (black)
    ('arm_', 'gray'),                                                  # every printed SO-101 part, base to jaws
    ('bracket_d435', 'white'),                                         # the D435 cradle (riser pocket; the nose cradle likewise)
    ('floor2', 'white'), ('riser_', 'white'), ('deck_bin', 'white'),
    ('housing_face', 'white'), ('housing_speaker_cover', 'white'),      # face frame, crown, speaker cover
    ('hub', 'white'), ('node', 'white'), ('edge', 'white'), ('pedestal', 'white'), ('support', 'white'),   # the chassis
    ('plate', 'white'), ('spider', 'white'),                           # earlier layouts' chassis (base.py, first_floor.py)
    ('bin', 'white'),                                                  # the bin (printed)
    ('pod_fork', 'gray'), ('pod_frame', 'gray'), ('pod_keel', 'gray'), ('pod_sleeve', 'gray'), ('pod_chassis', 'gray'), ('pod_retainer', 'gray'), ('pod_pivot_cap', 'gray'), ('pod_pin_cap', 'gray'), ('pod_brg_ring', 'gray'),
    ('roller_core', 'gray'), ('roller_sleeve', 'gray'),
    ('bracket', 'gray'), ('post_', 'gray'), ('spacer_', 'gray'),       # brackets, clips, posts and spacers on the floor
    ('housing_estop', 'gray'), ('housing_switch', 'gray'),
    ('bumper', 'orange'), ('wheel_hub', 'orange'), ('wheel_cartridge', 'orange'),
    ('roller_tread', 'tpu'),
)


def _stem(name):
    return name.split('/')[-1].lstrip('_')


def filament(name):
    n = _stem(name)
    for k, f in RULES:
        if n.startswith(k):
            return f
    return None


def colour(name):
    f = filament(name)
    return FILAMENTS[f] if f else None

"""The robot's filament colours: the BASELINE (Fernando, Nov 16 2026, on cad/v15; the Nov 13 three-colour scheme before it). Hex from Bambu's swatch
where there is one, written to STEP as sRGB (step_colour.py). cad/v15 README "Filament colours" has the part lists by colour.

    filament(name) -> 'white' | 'gray' | 'orange' | 'black' | 'tpu' | None     (None: not a printed part of the robot: keep the caller's colour)
    colour(name)   -> its (r, g, b) or None

    Jade White   the deck (four pieces + key), second floor, bin deck, the SO-101's printed parts, face frame / crown / speaker cover, brackets,
                 posts, spacers, e-stop and switch housings, battery clips, roller cores and sleeves
    Dark Gray    the suspension (fork halves, hangers, beam + pin + lug, towers, rings, 6804 retainer) and the five fins
    Orange       bumper, bin, lidar pedestal, arm riser with the D435 cradle
    Black        wheel hubs, wheel cartridges (cup + cap)
    TPU gray     roller treads (Gray #8e9089: Fernando's TPU, a little darker than Silver #a6a9aa)

Names are the generating scripts' part keys; a group prefix ('chassis/', 'FL/'), leading underscores (unplaced copies) and the mirrored
hand's '_m' suffix do not matter (first matching prefix wins). Bought parts (servos, AK40-10s, D435, display, electronics, battery, the
red e-stop button), fasteners (magenta) and mechanical items (bearings, springs, leads, plugs, envelopes) return None.
"""


def _hex(h):
    h = h.lstrip('#')
    return tuple(round(int(h[i:i + 2], 16) / 255.0, 4) for i in (0, 2, 4))


WHITE = _hex('#ffffff')                   # Jade White
DARK_GRAY = _hex('#545454')               # Dark Gray (the PLA gray)
ORANGE = _hex('#ff6a13')                  # Orange
BLACK = _hex('#151515')                   # Black PLA as it prints: the swatch's #000000 shows no shading in SolidWorks or a render
TPU_GRAY = _hex('#8e9089')                # Nov 16 (Fernando): the TPU treads, "a little darker" than Silver #a6a9aa (Nov 14: light gray #c4c4c4, an estimate)
FILAMENTS = dict(white=WHITE, gray=DARK_GRAY, orange=ORANGE, black=BLACK, tpu=TPU_GRAY)

# (prefix, filament): first match wins, so the exceptions come before the general prefixes
RULES = (
    ('arm_STS3215', None),                                             # the SO-101's servos are bought (black)
    ('arm_', 'white'),                                                 # every printed SO-101 part, base to jaws (the colour vision wants it neutral)
    ('bracket_d435', 'orange'),                                        # the D435 cradle (in the riser's pocket): with the riser
    ('floor2', 'white'), ('riser_', 'orange'), ('deck', 'white'),      # second floor; arm riser and adapter; deck_FL / FR / RL / RR / key and deck_bin
    ('housing_face', 'white'), ('housing_speaker_cover', 'white'),     # face frame, crown, speaker cover
    ('bin', 'orange'),                                                 # the bin
    ('bumper', 'orange'), ('pedestal', 'orange'),                      # the base's orange: the four bumper quarters and the lidar pedestal
    ('support', 'gray'),                                               # the five fins (support_front_L / R, support_side_L / R, support_spine)
    ('rx_deck', 'white'),                                              # the pod study's stand-in for the deck
    ('pod_', 'gray'), ('rx_', 'gray'),                                 # the suspension: fork halves, retainer; hangers, beam, pin, lug, towers, rings
    ('wheel_hub', 'black'), ('wheel_cartridge', 'black'),              # the wheel's hub and its bearing cartridge (cup + cap)
    ('roller_core', 'white'), ('roller_sleeve', 'white'),              # under the TPU
    ('roller_tread', 'tpu'),
    ('bracket', 'white'), ('post_', 'white'), ('spacer_', 'white'),    # brackets, clips, posts and spacers on the floors
    ('housing_estop', 'white'), ('housing_switch', 'white'),
)


def _stem(name):
    return name.split('/')[-1].lstrip('_')


def filament(name):
    n = _stem(name)
    if 'STS3215' in n:                                                 # the SO-101's servos are bought, whatever number the arm's file gives them
        return None
    for k, f in RULES:
        if n.startswith(k):
            return f
    return None


def colour(name):
    f = filament(name)
    return FILAMENTS[f] if f else None

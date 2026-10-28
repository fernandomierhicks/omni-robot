"""Colours for the v8 CAD: one table for the renders, the viewer and the STEP export. Fasteners are magenta (Fernando,
Oct 26: every screw, nut, washer and shim stands out); labels are white."""

from __future__ import annotations

COLORS = {   # class: RGBA
    "petg_dark": (0.2, 0.21, 0.24, 1), "petg_grey": (0.5, 0.52, 0.55, 1), "petg_orange": (0.98, 0.45, 0.08, 1),
    "petg_clear": (0.72, 0.82, 0.92, 0.85),     # the chute halves (at 0.55 their screws looked loose)
    "tpu_black": (0.07, 0.07, 0.08, 1), "tpu_orange": (0.95, 0.42, 0.06, 1),
    "foam": (0.35, 0.55, 0.85, 1), "net": (0.95, 0.95, 0.95, 0.35), "ball": (0.82, 0.95, 0.12, 1),
    "motor": (0.72, 0.74, 0.77, 1), "motor_black": (0.12, 0.12, 0.13, 1), "servo": (0.15, 0.2, 0.6, 1),
    "lidar": (0.1, 0.1, 0.11, 1), "jetson": (0.08, 0.35, 0.18, 1), "battery": (0.12, 0.3, 0.7, 1),
    "estop": (0.85, 0.08, 0.06, 1), "camera": (0.05, 0.05, 0.05, 1), "aluminium": (0.78, 0.8, 0.83, 1),
    "steel": (0.85, 0.86, 0.88, 1), "shock": (0.85, 0.2, 0.1, 1), "carbon": (0.06, 0.06, 0.06, 1),
    "fastener": (1.0, 0.0, 1.0, 1), "pcb": (0.05, 0.32, 0.17, 1), "label": (0.97, 0.97, 0.97, 1),
    "so101": (0.93, 0.93, 0.9, 1),
}

COLOR_CLASS = [   # (match on item name / key prefix, class)
    ("label ", "label"), ("fork bar", "aluminium"), ("mast post", "aluminium"), ("crown", "petg_orange"),
    ("mast_foot", "petg_dark"), ("payload_ports", "petg_orange"), ("switch_riser", "petg_dark"),
    ("on/off switch", "steel"), ("XT30", "battery"), ("Micro-Fit", "motor_black"), ("USB-C", "motor_black"), ("SO-101 STS3215", "motor_black"), ("SO-101", "so101"),
    ("bumper", "tpu_orange"), ("roller_tread", "tpu_black"), ("flywheel_tyre", "tpu_black"), ("feed_wheel_tyre", "tpu_black"),
    ("wheel_hub", "petg_orange"), ("roller_core", "petg_grey"), ("flywheel_hub", "petg_orange"), ("feed_wheel_hub", "petg_orange"),
    ("payload_plate", "petg_grey"), ("standoff rod", "aluminium"), ("standoff", "petg_orange"), ("deck", "petg_dark"), ("chute_half", "petg_clear"),
    ("collar", "petg_orange"), ("rim_arc", "petg_orange"), ("fw_mount", "petg_grey"), ("pad_", "foam"), ("funnel_net", "net"),
    ("ball", "ball"), ("face_", "petg_dark"), ("audio_", "petg_grey"), ("5 in display", "camera"),
    ("polycarbonate", "petg_clear"), ("cheek", "petg_orange"), ("cam_holder", "petg_grey"),
    ("micro speaker", "motor_black"),
    ("Damiao", "motor"), ("Pololu 25D", "motor"), ("T-Motor", "motor_black"), ("Feetech", "servo"), ("RPLIDAR", "lidar"),
    ("Jetson", "jetson"), ("battery", "battery"), ("OV9782", "camera"), ("camera bar", "aluminium"),
    ("Pololu #", "aluminium"), ("bearing", "steel"), ("608", "steel"), ("shaft", "steel"), ("pin", "steel"),
    ("RC shock", "shock"), ("carbon", "carbon"), ("M3", "fastener"), ("M2", "fastener"), ("M5", "fastener"), ("shim", "fastener"),
]


def color_class(it) -> str:
    for k, c in COLOR_CLASS:
        if it.name.startswith(k) or it.key.startswith(k):
            return c
    if it.kind == "fastener":
        return "fastener"
    if it.kind == "purchased":
        return "pcb"
    return "petg_dark"


def rgba(it) -> tuple:
    return COLORS[color_class(it)]

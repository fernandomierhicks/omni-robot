"""v5 simulation parameters: the v3b compact robot with its firmware limits removed.

v5 (see README.md):
  - limits "physics" (default): no speed, acceleration, jerk or yaw caps beyond what
    the hardware and the floor allow. The wheel-speed ceiling follows the live bus
    voltage (motor back-EMF), acceleration runs at the measured slip-free traction
    limit in each direction, the yaw rate is motor-limited. limits "firmware" restores
    v3b's caps (1.4 m/s, 85% traction table capped at 4.5 m/s^2, 40 m/s^3, 4 rad/s);
  - n_balls: extra balls in the scene, so the funnel can be loaded as a hopper.

v3 changes from v2:
  - N omni wheels (default 4 in an X), on a smaller base, each with the
    vertical compliance of its rubber rollers (so four wheels share load the way
    real ones do) and an optional rocker beam;
  - the whole catch module (funnel, pocket, gate, J-bend, flywheels) pitches on
    one tilt joint: forward to meet the incoming ball, back to set the launch
    elevation. That allows a shallower funnel;
  - stereo cameras on the fixed base instead of the funnel rim.

Round 2 (v3b, the defaults):
  - deck_mode "axle": the deck sits in the plane of the wheel axles, notched
    for the wheels and motors, with a flat battery pack under it; everything
    above drops ~63 mm ("high" = round 1, deck above the wheels);
  - feed "kicker": no J-bend. The caught ball rests in a corner at the bottom of
    the pocket; the gate servo becomes a paddle that kicks it up a short ramp
    into the flywheels ("j_bend" = round 1, trapdoor + gravity drop).

Feed-dependent defaults (launch angle, tilt range, pivot, frame mass) are filled
in when they are left as None, so to switch feed build a fresh SimParams(feed=...)
rather than using with_() on an existing one.

Change a component by swapping its catalog key, e.g. SimParams(wheel="Nexus-127").
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

from catalog import BALLS, BATTERIES, DRIVE_MOTORS, ESCS, FLYWHEEL_MOTORS, OMNI_WHEELS

LAYOUTS = {
    "x4": (45.0, 135.0, 225.0, 315.0),     # X drive, launcher between the two front wheels
    "kiwi3": (60.0, 180.0, 300.0),         # v2 layout
}

# Direction-aware acceleration limit for the v3b base (X4 + rocker, deck in the axle plane, mu 0.95), body frame,
# 0 = forward, mirrored left/right: 85% of the slip-free acceleration measured by studies/wheel_count.py (the worse of
# each left/right pair), capped at 4.5 m/s^2. The tip-over limit is 9.9 m/s^2 in the worst direction.
ACCEL_TABLE_V3B = ((0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0),
                   (4.5, 3.24, 3.0, 3.9, 4.3, 3.72, 4.5))

# v5 "physics" limit: 100% of the slip-free acceleration measured by studies/envelope.py on the v3b base (mu 0.95),
# body frame, mirrored left/right (the worse of each pair), no cap. Only ~10% above the v3b firmware table (which
# was 85% of v3's measurement); v3's "9.9 m/s^2 forward" was its test's ceiling (a 1 m/s ramp at 40 m/s^3 can't
# exceed 6.3), not the robot's: forward is 5.0.
ACCEL_TABLE_PHYS = ((0.0, 30.0, 60.0, 90.0, 120.0, 150.0, 180.0),
                    (5.04, 3.58, 3.36, 4.01, 4.52, 4.01, 4.91))

# Base limits per profile. "firmware" = v3b's caps; "physics" = only what the hardware and floor impose
# (speed: back-EMF at the live bus voltage; accel: traction; jerk and yaw acceleration: tuned for the
# fastest settled moves in studies/envelope.py, not for comfort or safety).
LIMIT_PROFILES = {
    "firmware": dict(base_max_speed=1.4, base_max_accel=2.5, accel_table=ACCEL_TABLE_V3B, base_max_jerk=40.0,
                     base_max_yaw_rate=4.0, base_max_yaw_accel=15.0, brake_margin=0.8, wheel_speed_frac=0.95,
                     live_wheel_limit=False),
    # jerk 0 = none: fastest (0.3 m in 0.64 s vs 0.68 at 400 m/s^3, 0.93 s firmware) with no overshoot;
    # yaw accel 100: 180 deg in 0.73 s, beyond it the motors' speed (~16 rad/s) is the limit
    "physics": dict(base_max_speed=math.inf, base_max_accel=3.36, accel_table=ACCEL_TABLE_PHYS, base_max_jerk=0.0,
                    base_max_yaw_rate=math.inf, base_max_yaw_accel=100.0, brake_margin=0.9, wheel_speed_frac=0.92,
                    live_wheel_limit=True),
}

FEED_DEFAULTS = {
    "j_bend": dict(launch_angle_deg=40.0, tilt_range_deg=(-20.0, 35.0), tilt_pivot=(0.01, 0.16), m_launcher_frame=0.42),
    "kicker": dict(launch_angle_deg=30.0, tilt_range_deg=(-30.0, 25.0), tilt_pivot=(-0.01, 0.15), m_launcher_frame=0.32),
}


@dataclass
class SimParams:
    # --- fitted components (catalog keys) ----------------------------------
    drive_motor: str = "AK40-10"
    wheel: str = "goBILDA-96"
    flywheel_motor: str = "F90-2806.5-1300"
    esc: str = "Lucid-8S-4in1"
    battery: str = "7S1P-P50B"
    ball: str = "foam-pickleball"

    # --- wheels --------------------------------------------------------------
    layout: str = "x4"
    wheel_model: str = "sphere"        # "sphere" (fast, default) or "rollers" (explicit rollers)
    wheel_mu: float | None = None      # None = catalog value for the chosen wheel
    roller_friction_torque: float = 0.004   # N m Coulomb roller-bearing drag (est)
    roller_damping: float = 2e-4       # N m s/rad roller viscous drag (est)
    roller_stiffness: float = 30e3     # N/m vertical compliance of a loaded 50A roller (est)
    roller_damping_ratio: float = 0.3  # of the base bouncing on its wheels (est)
    wheel_z_offset: tuple = ()         # m per wheel, + = hub sits lower (floor dip / build error elsewhere)
    rocker_deg: float = 3.0            # 0 = rigid; else the two front wheels share a rocker with +/- this travel.
                                       # Rigid X4 loses half its diagonal traction with 1 mm of floor/build error
    allocation: str = "pinv"           # "pinv" (plain speed loops) or "load_aware" (4+ wheels: null-space torque)

    # --- base geometry -----------------------------------------------------
    wheel_base_radius: float = 0.15    # base center to wheel center (v2: 0.18)
    deck_mode: str = "axle"            # "axle": deck in the wheel-axle plane, notched; "high": above the wheels (round 1)
    deck_radius: float = 0.185         # v2: 0.225
    deck_thickness: float = 0.006
    deck_clearance: float = 0.012      # "high" only: gap from wheel top to deck underside
    deck_cut_radius: float = 0.075     # "axle": the wheel+motor slots start this far out along each wheel's axle
    deck_slot_half_width: float = 0.055   # "axle": slot half width (wheel 96 mm + clearance)
    plate_gap: float = 0.065           # deck top to payload-plate underside (lidar + Jetson layer)
    plate_radius: float = 0.12
    # printed-part masses scaled from v2's CAD (cad/out/parts.json) to the smaller deck / 4 pods
    m_deck: float | None = None        # None: 0.40 for "high" (full disc), 0.26 for "axle" (notched, ~half the area)
    m_pod: float = 0.125               # per wheel: pod + adapter
    m_battery_tray: float = 0.03
    m_plate: float = 0.18
    m_jetson: float = 0.30
    m_electronics: float = 0.25        # MCU, power board, relay, DC-DCs
    m_lidar: float = 0.11
    m_bumper: float = 0.17
    m_misc: float = 0.33               # wiring, fasteners

    # --- base limits -----------------------------------------------------------
    limits: str = "physics"            # "physics" (v5: hardware and floor only) or "firmware" (v3b caps)
    # None = the profile's value (LIMIT_PROFILES); set any of them to override
    base_max_accel: float | None = None     # flat limit when there is no table
    accel_table: tuple | str | None = "auto"   # direction-aware limit; "auto" = the profile's table on the base it was
                                       # measured for (X4, rocker, axle deck), else None = base_max_accel everywhere
    base_max_jerk: float | None = None      # m/s^3; 0 = no jerk limit (v2 behavior)
    base_max_speed: float | None = None     # inf = only the motors' back-EMF limits it
    base_max_yaw_rate: float | None = None
    base_max_yaw_accel: float | None = None
    brake_margin: float | None = None       # stops are planned at this fraction of the worst-direction limit
    wheel_speed_frac: float | None = None   # wheel-speed ceiling as a fraction of no-load speed
    live_wheel_limit: bool | None = None    # True: no-load speed at the live bus voltage; False: at 24 V (v3)
    drive_kp: float = 2.0
    drive_ki: float = 40.0

    # --- catch module ---------------------------------------------------------
    funnel_rim_radius: float = 0.165
    funnel_throat_radius: float = 0.042
    funnel_wall_angle_deg: float = 55.0     # from the funnel axis (v2: 35): 86 mm deep instead of 176
    funnel_panels: int = 20
    tube_half_width: float = 0.039
    gate_pad_restitution: float = 0.05 # low-resilience foam on the gate, e.g. 10 mm slow-recovery PU (est; v2: 3 mm EVA,
                                       # 0.15). With the funnel tilted into the ball's path a centered ball reaches the
                                       # gate at ~6 m/s; at 0.15 a few bounced out (fallback: tilt gain 0.3)
    funnel_mu: float = 1.0             # ball on fabric net (est)
    funnel_roll: float = 0.012         # m, rolling resistance of the ball on the soft net (est)
    pocket_depth: float = 0.085        # throat to gate
    throat_flaps: bool = False         # one-way TPU flaps at the throat (rejected: too slow to close before the
                                       # rebound, and they jam the ball; the tilt "dump" does the job instead)
    flap_stiffness: float = 0.001      # N m/rad return spring: nearly constant torque, below the ball's weight torque
    flap_preload: float = 1.5          # rad of spring wind-up holding the flaps on their closed stop
    flap_up_deg: float = 20.0          # closed flaps point inward and this far up (a shallow cone, like a fish trap)
    feed: str = "kicker"               # "kicker" (paddle pushes the ball into the flywheels) or "j_bend" (round 1)
    drop_gap: float = 0.030            # j_bend: gate to J-center height (v2: 0.057): 118 mm drop, feeds up to 60 deg
    j_radius: float = 0.050
    # kicker: a plunger face perpendicular to the inclined floor, pushed along it by the old gate servo
    # through a horn + link. (A paddle hinged at the corner was tried first: it wedge-jams the ball against
    # the floor, because the wedge half-angle drops below the friction angle as it swings.)
    kick_stroke: float = 0.05          # m of plunger travel (the flywheels take the ball after ~44 mm)
    kick_speed: float = 0.4            # m/s plunger speed (fast servo on a ~25 mm horn) (est)
    kick_force: float = 40.0           # N plunger force limit (25 kg cm servo on a 25 mm horn is ~90 N)
    plunger_height: float = 0.07       # plunger face height, perpendicular to the floor
    kicker_nip_distance: float = 0.05  # kicker: resting ball center to the flywheel nip, along the launch line. The
                                       # plunger must still be pressing when the wheels take the ball: at 0.07 a
                                       # 0.4 m/s plunger lets go first and exit speed scatters 3.6%; at 0.05, 0.0%
    pocket_clearance: float = 0.012    # kicker: resting ball top to throat
    m_funnel: float = 0.16             # collar, rim, struts, net
    m_launcher_frame: float | None = None   # None: 0.42 j_bend (channel halves with J, flap, servo tower); 0.32 kicker
    m_tilt_frame: float = 0.12         # two pivot yokes + bearings on the payload plate
    m_cameras: float = 0.08

    # --- tilt joint -------------------------------------------------------------
    tilt: bool = True                  # False: module fixed at 0 (v2-style, for comparison)
    tilt_range_deg: tuple | None = None     # - = funnel tips back (higher launch), + = toward the thrower.
                                       # None: (-20, 35) j_bend; (-30, 25) kicker (its launch line sits lower in the module)
    tilt_pivot: tuple | None = None    # (x, z) of the pivot in the module frame (z from the module base). None: the
                                       # per-feed spot with the lowest rim height that clears the plate at all tilts
    tilt_clearance: float = 0.006      # module to payload plate over the whole tilt range
    tilt_torque: float = 3.0           # N m (35 kg cm class servo on the pivot, e.g. DS3235 (est))
    tilt_speed: float = 5.0            # rad/s (0.2 s / 60 deg)
    m_tilt_servo: float = 0.08

    # --- launcher -----------------------------------------------------------
    launch_angle_deg: float | None = None   # in the module frame; world elevation = this - tilt. None: 40 j_bend;
                                       # 30 kicker (shallower than the 35 deg funnel wall, so the ball passes under it)
    flywheel_diameter: float = 0.070
    flywheel_width: float = 0.028
    flywheel_inertia: float = 8.0e-5
    flywheel_mass: float = 0.12
    flywheel_mu: float = 1.1           # TPU tire on foam (est)
    squeeze: float = 0.010
    squeeze_force: float = 15.0        # N per wheel at full squeeze (est)
    nip_distance: float = 0.032
    gate_open_angle: float = 1.66
    gate_torque: float = 2.4
    gate_speed: float = 8.0

    # --- cameras ----------------------------------------------------------------
    camera_mount: str = "plate_front"  # "plate_front" (fixed, low, ~40 cm of FFC to the Jetson), "side_posts"
                                       # (fixed, beside the funnel, ~70 cm), "rim" (v2: moves with the tilt)
    camera_baseline: float = 0.19      # m between the stereo pair (plate_front, rim)
    camera_tilt_deg: float = 30.0
    camera_fovy_deg: float = 70.0

    # --- hopper -----------------------------------------------------------------
    n_balls: int = 0                   # extra balls in the scene ("ball_1".. "ball_n"), to load the funnel as a hopper
    hopper: str = "none"               # multi-ball feed: "none" (the bare funnel: two balls arch over the throat),
                                       # "tube" (a single-file magazine tube dropped into the funnel for hopper mode),
                                       # "sweeper" (a wire arm on the net, turned about the funnel axis from the rim)
    hopper_tube_height: float = 0.33   # m above the throat: 6 balls fully inside (at 0.30 the 6th stuck out 2 cm and hard turns threw it out)
    sweeper_speed: float = 6.0         # rad/s when sweeping (est: small gearmotor on the rim ring)
    sweeper_torque: float = 0.3        # N m at the arm (est)
    m_hopper: float = 0.10             # tube or sweeper ring + arm + motor
    pocket_detent: float = 0.0         # m: height of a retaining ridge in front of the resting ball (0 = none, v3b).
                                       # The pocket is open toward the nip; in hopper mode the flywheels run, and hard
                                       # turns and braking rolled the waiting ball into them

    # --- environment ----------------------------------------------------------
    area_size: tuple = (5.0, 4.0)
    wall_height: float = 2.0
    thrower_xy: tuple = (-2.0, 0.0)
    home_xy: tuple = (1.0, 0.0)
    floor_mu: float = 0.9
    timestep: float = 0.0005
    air_density: float = 1.204
    air_viscosity: float = 1.8e-5
    electronics_power: float = 22.0

    # ------------------------------------------------------------------------
    def __post_init__(self):
        fd = FEED_DEFAULTS[self.feed]
        for k, v in fd.items():
            if getattr(self, k) is None:
                setattr(self, k, v)
        if self.m_deck is None:
            self.m_deck = 0.26 if self.deck_mode == "axle" else 0.40
        prof = LIMIT_PROFILES[self.limits]
        for k, v in prof.items():
            if k != "accel_table" and getattr(self, k) is None:
                setattr(self, k, v)
        if self.accel_table == "auto":
            v3b_base = (self.layout == "x4" and self.deck_mode == "axle" and self.rocker_deg > 0
                        and self.wheel_base_radius == 0.15 and self.wheel_mu is None)
            self.accel_table = prof["accel_table"] if v3b_base else None

    @property
    def gate_open_target(self) -> float:
        """Gate joint position commanded when the gate 'opens' (j_bend: trapdoor angle; kicker: plunger stroke)."""
        return self.kick_stroke if self.feed == "kicker" else -self.gate_open_angle

    @property
    def gate_slew(self) -> float:
        return self.kick_speed if self.feed == "kicker" else self.gate_speed

    @property
    def wheel_angles_deg(self) -> tuple:
        return LAYOUTS[self.layout]

    @property
    def n_wheels(self) -> int:
        return len(self.wheel_angles_deg)

    @property
    def motor(self):
        return DRIVE_MOTORS[self.drive_motor]

    @property
    def omni(self):
        return OMNI_WHEELS[self.wheel]

    @property
    def fly_motor(self):
        return FLYWHEEL_MOTORS[self.flywheel_motor]

    @property
    def esc_spec(self):
        return ESCS[self.esc]

    @property
    def pack(self):
        return BATTERIES[self.battery]

    @property
    def ball_spec(self):
        return BALLS[self.ball]

    @property
    def mu(self) -> float:
        return self.omni.mu if self.wheel_mu is None else self.wheel_mu

    @property
    def wheel_radius(self) -> float:
        return self.omni.radius

    @property
    def deck_height(self) -> float:
        """Deck underside."""
        if self.deck_mode == "axle":
            return self.wheel_radius - self.deck_thickness / 2
        return 2 * self.wheel_radius + self.deck_clearance

    @property
    def plate_height(self) -> float:
        return self.deck_height + 0.006 + self.plate_gap + 0.006

    @property
    def funnel_depth(self) -> float:
        a = math.radians(self.funnel_wall_angle_deg)
        return (self.funnel_rim_radius - self.funnel_throat_radius) / math.tan(a)

    @property
    def tilt_limits(self) -> tuple:
        lo, hi = self.tilt_range_deg
        return (math.radians(lo), math.radians(hi)) if self.tilt else (0.0, 0.0)

    @property
    def home_yaw(self) -> float:
        return math.atan2(self.thrower_xy[1] - self.home_xy[1], self.thrower_xy[0] - self.home_xy[0])

    def with_(self, **kw) -> "SimParams":
        return replace(self, **kw)

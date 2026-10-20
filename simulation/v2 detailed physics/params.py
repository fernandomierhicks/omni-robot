"""Simulation parameters: which catalog parts are fitted, plus geometry and limits.

Change a component by swapping its catalog key, e.g. SimParams(wheel="Nexus-127").
Geometry that depends on the parts (deck height from the wheel size, motor
position from the motor length) is derived, so a swap keeps the robot consistent.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

from catalog import BALLS, BATTERIES, DRIVE_MOTORS, ESCS, FLYWHEEL_MOTORS, OMNI_WHEELS


@dataclass
class SimParams:
    # --- fitted components (catalog keys) ----------------------------------
    drive_motor: str = "AK40-10"
    wheel: str = "goBILDA-96"
    flywheel_motor: str = "F90-2806.5-1300"
    esc: str = "Lucid-8S-4in1"
    battery: str = "7S1P-P50B"
    ball: str = "foam-pickleball"

    # --- wheel model -------------------------------------------------------
    wheel_model: str = "sphere"        # "sphere" (fast, default) or "rollers" (explicit rollers)
    wheel_mu: float | None = None      # None = catalog value for the chosen wheel
    roller_friction_torque: float = 0.004   # N m Coulomb roller-bearing drag (est)
    roller_damping: float = 2e-4       # N m s/rad roller viscous drag (est)

    # --- base geometry -----------------------------------------------------
    wheel_base_radius: float = 0.18    # base center to wheel center
    wheel_angles_deg: tuple = (60.0, 180.0, 300.0)
    deck_radius: float = 0.225
    deck_clearance: float = 0.012      # gap from wheel top to deck underside
    plate_gap: float = 0.065           # deck top to payload-plate underside (lidar + Jetson layer)
    # masses of printed parts and electronics (kg); printed parts in PETG/PLA
    # printed-part masses from cad/out/parts.json (PETG/PLA/TPU, infill + walls)
    m_deck: float = 0.58               # center plate + 6 sectors
    m_lower_frame: float = 0.38        # 3 wheel pods, adapters, battery tray
    m_plate: float = 0.18              # payload plate
    m_jetson: float = 0.30
    m_electronics: float = 0.25        # MCU, power board, relay, DC-DCs
    m_lidar: float = 0.11
    m_bumper: float = 0.20
    m_misc: float = 0.30               # wiring, fasteners

    # --- base limits (firmware) --------------------------------------------
    base_max_accel: float = 2.5        # just above the worst-direction slip limit (~2.4 at mu 0.95)
    base_max_speed: float = 1.4
    base_max_yaw_rate: float = 4.0
    drive_kp: float = 2.0              # driver speed-loop P gain (N m per rad/s at the output)
    drive_ki: float = 40.0

    # --- catch module -------------------------------------------------------
    funnel_rim_height: float = 0.60
    funnel_rim_radius: float = 0.165
    funnel_throat_radius: float = 0.042
    funnel_wall_angle_deg: float = 35.0
    funnel_panels: int = 20
    tube_half_width: float = 0.039     # pocket / drop tube / J inner half width: 2 mm clearance so
                                       # the falling ball meets the J nearly tangentially
    gate_pad_restitution: float = 0.15 # EVA foam pad on the gate (est)
    funnel_mu: float = 1.0             # ball on fabric net (est)
    funnel_roll: float = 0.012         # m, rolling resistance of the ball on the soft net (est)
    pocket_depth: float = 0.085        # throat to gate: the held ball sits fully in the tube
    j_radius: float = 0.050            # J-bend centerline radius
    m_funnel: float = 0.19             # collar, 4 rim arcs, 6 carbon struts, net
    m_launcher_frame: float = 0.51     # launcher base, channel halves, flap, servo tower
    m_cameras: float = 0.08

    # --- launcher -----------------------------------------------------------
    launch_angle_deg: float = 52.0
    flywheel_diameter: float = 0.070
    flywheel_width: float = 0.028
    flywheel_inertia: float = 8.0e-5   # per wheel: PETG hub + TPU tire + 8 x M4 ring + rotor (CAD)
    flywheel_mass: float = 0.12
    flywheel_mu: float = 1.1           # TPU tire on foam (est)
    squeeze: float = 0.010             # ball diameter minus wheel gap
    squeeze_force: float = 15.0        # N per wheel at full squeeze (foam ball compression, est)
    nip_distance: float = 0.032        # J exit to wheel centerline, along the launch line
    # gate: trapdoor hinged along x at the -y wall, driven directly by a standard servo
    gate_open_angle: float = 1.66      # rad (95 deg, flap hangs flush in the wall window)
    gate_torque: float = 2.4           # N m (25 kg cm servo, e.g. DS3225MG)
    gate_speed: float = 8.0            # rad/s (0.13 s / 60 deg)

    # --- environment ----------------------------------------------------------
    area_size: tuple = (5.0, 4.0)
    wall_height: float = 2.0
    thrower_xy: tuple = (-2.0, 0.0)
    home_xy: tuple = (1.0, 0.0)
    floor_mu: float = 0.9              # floor's own friction; contacts use the max of the pair
    timestep: float = 0.0005           # 1 ms gives erratic flywheel shots above ~4000 rpm
    air_density: float = 1.204
    air_viscosity: float = 1.8e-5
    electronics_power: float = 22.0    # W: Jetson + lidar + cameras + MCU

    # ------------------------------------------------------------------------
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
        """Deck underside sits just above the wheel tops."""
        return 2 * self.wheel_radius + self.deck_clearance

    @property
    def plate_height(self) -> float:
        """Payload-plate top surface (catch-module origin)."""
        return self.deck_height + 0.006 + self.plate_gap + 0.006

    @property
    def funnel_depth(self) -> float:
        a = math.radians(self.funnel_wall_angle_deg)
        return (self.funnel_rim_radius - self.funnel_throat_radius) / math.tan(a)

    @property
    def home_yaw(self) -> float:
        return math.atan2(self.thrower_xy[1] - self.home_xy[1], self.thrower_xy[0] - self.home_xy[0])

    def with_(self, **kw) -> "SimParams":
        return replace(self, **kw)

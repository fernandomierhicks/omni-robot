"""Component catalog: datasheet values for the parts the simulation and trade study use.

Values marked (est) are engineering estimates, not datasheet figures; replace them
with measured data once parts are on the bench (the motor models take these
dataclasses directly, so a characterized motor is just a new entry here).
Prices are approximate street prices in USD seen Oct 2026.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Drive actuators (geared BLDC with integrated FOC driver)

@dataclass(frozen=True)
class Thermal:
    """Two-node winding/case model (same form as the wheeled-leg robot sim)."""
    c_winding: float      # J/K
    c_case: float         # J/K
    r_wc: float           # K/W winding -> case
    r_ca: float           # K/W case -> ambient
    t_max: float          # C, driver starts derating 10 C below this


@dataclass(frozen=True)
class DriveMotor:
    name: str
    part: str
    price: float
    mass: float               # kg
    diameter: float           # m
    length: float             # m
    kv: float                 # rpm/V, motor side
    kt: float                 # N m/A, motor side (torque per q-axis amp)
    gear: float
    r_ll: float               # ohm, phase to phase
    i_rated: float            # A
    i_peak: float             # A, driver limit
    v_rated: float            # V
    noload_rpm: float         # output rpm at v_rated (sets the back-EMF constant)
    rotor_inertia: float      # kg m^2, motor side
    backdrive_torque: float   # N m at the output (gearbox Coulomb friction)
    gear_eff: float = 0.90    # (est) planetary efficiency
    thermal: Thermal = field(default_factory=lambda: Thermal(8.0, 100.0, 1.5, 10.0, 100.0))
    notes: str = ""

    @property
    def ke(self) -> float:
        """Back-EMF constant, V s/rad motor side, from the datasheet no-load speed."""
        return self.v_rated / (self.noload_rpm * self.gear * 2 * math.pi / 60)

    @property
    def r_eq(self) -> float:
        """DC-equivalent resistance for FOC: copper loss = r_eq * iq^2."""
        return 0.75 * self.r_ll

    @property
    def peak_torque(self) -> float:
        return self.kt * self.gear * self.gear_eff * self.i_peak

    @property
    def rated_torque(self) -> float:
        return self.kt * self.gear * self.gear_eff * self.i_rated

    @property
    def reflected_inertia(self) -> float:
        return self.rotor_inertia * self.gear ** 2


DRIVE_MOTORS = {
    "AK40-10": DriveMotor(
        "CubeMars AK40-10 V3.0 KV170", "AK40-10 V3.0 KV170 (with driver)", 135.90, 0.190, 0.053, 0.0402,
        kv=170, kt=0.056, gear=10, r_ll=0.978, i_rated=2.7, i_peak=7.3, v_rated=24, noload_rpm=435,
        rotor_inertia=9.735e-6, backdrive_torque=0.06,
        notes="Baseline. Datasheet values from cubemars.com (Oct 2026); thermal (est)."),
    "AK45-10": DriveMotor(
        "CubeMars AK45-10 KV75", "AK45-10 KV75", 150.0, 0.260, 0.053, 0.044,
        kv=75, kt=0.127, gear=10, r_ll=2.19, i_rated=2.4, i_peak=6.3, v_rated=24, noload_rpm=180,
        rotor_inertia=1.2e-5, backdrive_torque=0.1,
        thermal=Thermal(10.0, 200.0, 1.0, 6.0, 100.0),
        notes="R and thermal from the wheeled-leg robot sim; currents/inertia (est). Too slow at 24 V."),
    "AK60-6": DriveMotor(
        "CubeMars AK60-6 V3.0 KV80", "AK60-6 V3.0 KV80", 360.0, 0.380, 0.0768, 0.040,
        kv=80, kt=0.119, gear=6, r_ll=0.595, i_rated=4.2, i_peak=12.6, v_rated=24, noload_rpm=320,
        rotor_inertia=3.0e-5, backdrive_torque=0.1,
        thermal=Thermal(15.0, 200.0, 1.0, 6.0, 100.0),
        notes="Datasheet speed/R/torque; inertia (est)."),
    "GIM6010-8": DriveMotor(
        "SteadyWin GIM6010-8 (GDS68 driver)", "GIM6010-8", 105.0, 0.388, 0.068, 0.045,
        kv=140, kt=0.068, gear=8, r_ll=0.88, i_rated=9.2, i_peak=20.0, v_rated=24, noload_rpm=420,
        rotor_inertia=2.5e-5, backdrive_torque=0.12,
        thermal=Thermal(15.0, 220.0, 1.0, 6.0, 100.0),
        notes="Listings disagree on KV and R (0.42-0.44 ohm phase); inertia (est)."),
    "DM-J4310": DriveMotor(
        "Damiao DM-J4310-2EC V1.1", "DM-J4310-2EC V1.1", 125.0, 0.300, 0.056, 0.046,
        kv=95, kt=0.12, gear=10, r_ll=1.2, i_rated=2.5, i_peak=7.5, v_rated=24, noload_rpm=200,
        rotor_inertia=1.0e-5, backdrive_torque=0.1,
        notes="Datasheet torque/speed; R, KV, inertia (est). Too slow at 24 V."),
}


# ---------------------------------------------------------------------------
# Omni wheels

@dataclass(frozen=True)
class OmniWheel:
    name: str
    part: str
    price: float
    diameter: float      # m
    width: float         # m
    mass: float          # kg
    rollers_per_row: int
    roller_diameter: float
    mu: float            # roller-floor friction on a smooth hard floor (est)
    bore: str
    notes: str = ""

    @property
    def radius(self) -> float:
        return self.diameter / 2


OMNI_WHEELS = {
    "goBILDA-96": OmniWheel(
        "goBILDA 96mm Omni Wheel (14mm bore, 50A)", "3624-0014-0096", 20.0, 0.096, 0.038, 0.119,
        8, 0.016, 0.95, "14 mm round, 16/32 mm patterns",
        notes="Plastic hub, soft 50A rubber rollers: grippy and light."),
    "Nexus-100": OmniWheel(
        "Nexus 100mm double aluminum omni (bearing rollers)", "14054", 35.0, 0.100, 0.029, 0.335,
        9, 0.019, 0.75, "Nexus hub",
        notes="Aluminum plates, 18 rollers, 30 kg rating."),
    "Nexus-127": OmniWheel(
        "Nexus 127mm double aluminum omni (bearing rollers)", "14073", 45.0, 0.127, 0.029, 0.480,
        11, 0.019, 0.75, "Nexus hub",
        notes="Design-doc size class (125 mm). 22 rollers, 30 kg rating."),
}


# ---------------------------------------------------------------------------
# Launcher flywheel motors (drone outrunners) and ESC

@dataclass(frozen=True)
class FlywheelMotor:
    name: str
    part: str
    price: float
    mass: float
    kv: float             # rpm/V
    r_m: float            # ohm, phase to phase
    i0: float             # A no-load current at 10 V
    i_max: float          # A, motor rating (short bursts)
    rotor_inertia: float  # kg m^2 (bell + magnets)
    stator: str
    mount: str
    notes: str = ""

    @property
    def kt(self) -> float:
        return 60 / (2 * math.pi * self.kv)


FLYWHEEL_MOTORS = {
    "F90-2806.5-1300": FlywheelMotor(
        "T-Motor F90 2806.5 1300KV", "F90 2806.5 KV1300", 30.0, 0.0466, 1300, 0.076, 0.95, 45.0, 4.5e-6,
        "28x6.5 mm", "4x M3 on 19x19 mm", notes="Datasheet KV/R/I0/mass; rotor inertia (est)."),
    "XING-2806.5-1300": FlywheelMotor(
        "iFlight XING 2806.5 1300KV", "XING 2806.5 1300KV", 26.0, 0.0495, 1300, 0.0485, 1.0, 48.0, 4.5e-6,
        "28x6.5 mm", "4x M3 on 19x19 mm", notes="Datasheet R/mass; I0 and inertia (est)."),
    "2207-1750": FlywheelMotor(
        "Generic 2207 1750KV (5-inch 6S class)", "e.g. T-Motor F60 Pro V 1750KV", 25.0, 0.034, 1750, 0.065, 1.2, 40.0, 2.5e-6,
        "22x7 mm", "4x M3 on 16x16 mm", notes="Class-typical values (est)."),
    "3115-900": FlywheelMotor(
        "Generic 3115 900KV (cinelifter class)", "e.g. BrotherHobby Tornado T5 3115 900KV", 40.0, 0.080, 900, 0.050, 1.0, 50.0, 1.2e-5,
        "31x15 mm", "4x M3 on 19x19 mm", notes="Class-typical values (est)."),
}


@dataclass(frozen=True)
class Esc:
    name: str
    part: str
    price: float
    current_limit: float     # A per channel, set in AM32 config
    v_max: float
    telemetry_period: float  # s between bidirectional-DShot eRPM frames
    notes: str = ""


ESCS = {
    "Lucid-8S-4in1": Esc("TBS Lucid 8S 60A 4-in-1 (AM32)", "TBS Lucid 8S 4in1 60A", 90.0, 25.0, 36.0, 0.0005,
                         notes="3-8S, bidirectional DShot + telemetry; 2 of 4 channels used."),
}


# ---------------------------------------------------------------------------
# Battery

@dataclass(frozen=True)
class Cell:
    name: str
    capacity: float          # Ah
    r0: float                # ohm DC at 50% SoC, 25 C
    r1: float                # ohm polarization
    tau1: float              # s polarization time constant
    i_max: float             # A continuous
    mass: float
    ocv_soc: tuple = (0.0, 0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0)
    ocv_v: tuple = (3.00, 3.30, 3.45, 3.56, 3.63, 3.69, 3.75, 3.83, 3.92, 4.00, 4.08, 4.18)


@dataclass(frozen=True)
class BatteryPack:
    name: str
    cell: Cell
    series: int
    parallel: int
    r_extra: float           # ohm: BMS FETs, fuse, wiring, connectors (est)
    price: float
    i_bms: float             # A BMS overcurrent trip

    @property
    def capacity(self) -> float:
        return self.cell.capacity * self.parallel

    @property
    def r0(self) -> float:
        return self.cell.r0 * self.series / self.parallel + self.r_extra

    @property
    def mass(self) -> float:
        return self.cell.mass * self.series * self.parallel * 1.25   # + BMS, wrap, wiring


P50B = Cell("Molicel INR21700-P50B", 5.0, 0.016, 0.008, 15.0, 60.0, 0.070)
P45B = Cell("Molicel INR21700-P45B", 4.5, 0.015, 0.008, 15.0, 45.0, 0.070)

BATTERIES = {
    "7S1P-P50B": BatteryPack("7S1P Molicel P50B, 5 Ah, ~126 Wh", P50B, 7, 1, 0.025, 120.0, 40.0),
    "7S1P-P45B": BatteryPack("7S1P Molicel P45B, 4.5 Ah, ~113 Wh", P45B, 7, 1, 0.025, 110.0, 40.0),
    "7S2P-P45B": BatteryPack("7S2P Molicel P45B, 9 Ah, ~227 Wh", P45B, 7, 2, 0.020, 190.0, 60.0),
}


# ---------------------------------------------------------------------------
# Ball

@dataclass(frozen=True)
class Ball:
    name: str
    diameter: float
    mass: float
    drag_cd: float
    stiffness: float      # N/m, small-compression foam stiffness (est)
    restitution: float    # bounce on a hard floor (est)
    mu: float

    @property
    def radius(self) -> float:
        return self.diameter / 2


BALLS = {
    "foam-pickleball": Ball("Pickleball-size foam ball (74 mm)", 0.074, 0.022, 0.50, 1500.0, 0.45, 0.9),
}

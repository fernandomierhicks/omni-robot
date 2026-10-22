"""Component catalog: datasheet values for the parts the simulation and trade study use.

Values marked (est) are engineering estimates, not datasheet figures; replace them
with measured data once parts are on the bench (the motor models take these
dataclasses directly, so a characterized motor is just a new entry here).
Prices are approximate street prices in USD seen Oct 2026. The shared actuator database (omni-robot/database,
design.md 24) holds every part with its sources and provenance; entries marked "exported from database/actuator_db.py"
came from it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace


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
    # v6 round 3 motor study (studies/motor_study.py): cheaper drive candidates
    "M2006": DriveMotor(
        "DJI RoboMaster M2006 P36 + C610 ESC", "M2006 P36 + C610", 55.0 + 39.0, 0.107, 0.0244, 0.0648,
        kv=750, kt=0.0115, gear=36, r_ll=1.17, i_rated=3.0, i_peak=10.0, v_rated=24, noload_rpm=500,
        rotor_inertia=1.2e-6, backdrive_torque=0.15, gear_eff=0.65,
        thermal=Thermal(3.0, 40.0, 3.0, 12.0, 100.0),
        notes="RoboMaster: 500 rpm no-load, 1 N m continuous (416 rpm there), 36:1, 90 g + 17 g ESC, C610 10 A "
              "continuous over CAN, $55 + $39 at the DJI store (Oct 20 2026, both out of stock; was $41). kt, R from a third-party listing "
              "(stall 7 N m / 27.3 A, est); inertia, drag, thermal (est). VERIFY the C610's input voltage: "
              "RoboMaster packs are 6S, a full 7S pack is 29.4 V."),
    # design.md 24: a reseller page (robotdoo, Oct 2026) gives the M2006 P36 a 0.18 N m/A output torque constant
    # (gear losses included), so 10 A on the C610 is 1.8 N m, not the 2.7 N m above. Same motor, the cautious kt.
    "M2006-lowkt": DriveMotor(
        "DJI RoboMaster M2006 P36 + C610 ESC (reseller kt 0.18 N m/A)", "M2006 P36 + C610", 55.0 + 39.0, 0.107, 0.0244,
        0.0648, kv=750, kt=0.18 / (36 * 0.65), gear=36, r_ll=1.17, i_rated=3.0, i_peak=10.0, v_rated=24, noload_rpm=500,
        rotor_inertia=1.2e-6, backdrive_torque=0.15, gear_eff=0.65,
        thermal=Thermal(3.0, 40.0, 3.0, 12.0, 100.0),
        notes="As M2006, with the reseller's output torque constant (0.18 N m/A incl. gear losses): 1.8 N m at the "
              "C610's 10 A. DJI store Oct 2026: motor $55, C610 $39, both out of stock. Exported from "
              "database/actuator_db.py (dji-m2006-p36)."),
    # design.md 24 sweep: Damiao planetary BLDC motors with a built-in FOC driver (CAN: MIT / speed / position modes).
    # Reseller data only (Foxtech, aifitlab, Oct 2026). kt ~ ke from the no-load speed (est), gear efficiency 0.9 (est).
    "DM-2325": DriveMotor(
        "Damiao DM-2325 28 mm planetary BLDC with driver, 1:25", "DM-2325", 86.0, 0.172, 0.028, 0.070,
        kv=560 * 25 / 24, kt=24 / (560 * 25 * 2 * math.pi / 60), gear=25,
        r_ll=2 * 0.2915, i_rated=3.6, i_peak=13.5, v_rated=24, noload_rpm=560, rotor_inertia=3.0e-6,
        backdrive_torque=0.08, gear_eff=0.9, thermal=Thermal(3.0, 40.0, 3.0, 12.0, 100.0),
        notes="Reseller spec table: 24 V (driver 24-48 V), 1:25, ~560 rpm no-load, ~380 rpm nominal, phase R 0.2915 ohm, "
              "L 75 uH, 172 g, $86. Torque conflicts between listings (1.35 rated / 5 peak vs 5 / 2): this entry takes "
              "1.35 / 5 N m (i_rated, i_peak from kt est). Length, inertia, drag, thermal (est, as the M2006). Exported "
              "from database/actuator_db.py (damiao-dm-2325)."),
    "DM-2325-low": DriveMotor(
        "Damiao DM-2325 (cautious: 2 N m peak)", "DM-2325", 86.0, 0.172, 0.028, 0.070,
        kv=560 * 25 / 24, kt=24 / (560 * 25 * 2 * math.pi / 60), gear=25, r_ll=2 * 0.2915, i_rated=3.6, i_peak=5.4,
        v_rated=24, noload_rpm=560, rotor_inertia=3.0e-6, backdrive_torque=0.08, gear_eff=0.9,
        thermal=Thermal(3.0, 40.0, 3.0, 12.0, 100.0),
        notes="As DM-2325 with the other listing's 2 N m peak (i_peak 5.4 A at the estimated kt)."),
    "DM-J3507": DriveMotor(
        "Damiao DM-J3507-2EC V1.1, 7:1, dual encoder, MIT driver", "DM-J3507-2EC", 92.0, 0.150, 0.040, 0.040,
        kv=460 * 7 / 24, kt=24 / (460 * 7 * 2 * math.pi / 60), gear=7, r_ll=1.0, i_rated=1.6, i_peak=6.0, v_rated=24,
        noload_rpm=460, rotor_inertia=1.0e-5, backdrive_torque=0.05, gear_eff=0.9, thermal=Thermal(5.0, 60.0, 2.0, 12.0, 100.0),
        notes="Damiao manual (aifitlab wiki): 24 V version 460 rpm no-load, 150 rpm rated, 0.8 N m rated / 3 N m peak, "
              "7:1, 14-bit dual encoders, CAN + MIT; $89-95 (Foxtech). Currents set from those torques with kt from ke; R, "
              "size, inertia, thermal (est). Exported from database/actuator_db.py (damiao-dm-j3507-2ec)."),
    "DM-S3519": DriveMotor(
        "Damiao DM-S3519 42 mm planetary BLDC with driver, 1:19.2", "DM-S3519-1EC", 102.0, 0.396, 0.042, 0.0915,
        kv=435 * 19.2 / 24, kt=24 / (435 * 19.2 * 2 * math.pi / 60), gear=19.2, r_ll=0.4, i_rated=7.4, i_peak=16.4,
        v_rated=24, noload_rpm=435, rotor_inertia=1.5e-5, backdrive_torque=0.12, gear_eff=0.9,
        thermal=Thermal(10.0, 150.0, 1.5, 6.0, 100.0),
        notes="Reseller listings: 24 V (driver 15-52 V), 1:19.2, 435 rpm no-load, 3.5 N m rated at 395 rpm, 7.8 N m peak, "
              "396 g, $102 (an M3508-class motor with its own driver). R, inertia, drag, thermal (est). Exported from "
              "database/actuator_db.py (damiao-dm-s3519)."),
    "Pololu-37D-19": DriveMotor(
        "Pololu 19:1 Metal Gearmotor 37Dx68L 24V, 64 CPR encoder + H-bridge", "Pololu #4691", 60.95 + 12.0, 0.215,
        0.037, 0.068, kv=410, kt=0.0223, gear=18.75, r_ll=10.67, i_rated=1.0, i_peak=3.0, v_rated=24,
        noload_rpm=530, rotor_inertia=1.5e-6, backdrive_torque=0.08, gear_eff=0.765,
        thermal=Thermal(15.0, 120.0, 1.5, 8.0, 100.0),
        notes="Pololu: 530 rpm / 0.1 A at 24 V, stall 9.5 kg cm / 3 A (extrapolated), brushed, 1200 counts/rev. "
              "R = 8 ohm (r_eq), kt from the stall point; mass, inertia, drag, thermal (est). Driver e.g. a DRV8874 "
              "carrier ($12) per motor."),
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
    rows: int = 2              # v7: staggered rows of rollers (2 = double, 3 = triple)
    coverage: float = 0.62     # v7: each roller spans this fraction of its row pitch (rows overlap to roll smoothly)
    kind: str = "omni"         # v7: "omni" (rollers tangent to the rim) or "mecanum" (rollers at 45 deg)
    printed: bool = False      # v7: our own printed design (PETG hub, TPU-tread rollers on steel pins)

    @property
    def radius(self) -> float:
        return self.diameter / 2

    @property
    def n_rollers(self) -> int:
        return self.rows * self.rollers_per_row

    def edge_radius(self, beta_deg: float) -> float:
        """Radius of the wheel's tread where it meets a square edge whose normal is beta_deg from the rolling
        direction (0 = rolling straight at it, 90 = moving along the axle, the roller's own radius): the normal
        radius of curvature of the wheel's torus-like envelope in that direction (1/R cos^2 + 1/r sin^2)^-1."""
        b = math.radians(beta_deg)
        return 1.0 / (math.cos(b) ** 2 / self.radius + math.sin(b) ** 2 / (self.roller_diameter / 2))


def printed_omni(diameter: float, rollers_per_row: int, roller_diameter: float, rows: int = 2, width: float | None = None,
                 mu: float = 0.9) -> OmniWheel:
    """v7: a printed omni wheel. Mass (est) from its parts: two or three PETG hub plates, TPU-tread rollers on PETG
    cores spinning on 3 mm steel pins. Scaled so the 96 mm / 2 x 8 / 16 mm version weighs ~125 g, the goBILDA's 119 g."""
    R, r = diameter / 2, roller_diameter / 2
    half = R * math.sin(math.pi * 0.62 / rollers_per_row)
    n = rows * rollers_per_row
    width = width or rows * (roller_diameter + 0.003) + 0.002
    roller = n * (1.24e3 * math.pi * r * r * 2 * half * 0.35 + 7.8e3 * math.pi * 0.0015 ** 2 * 2 * half)   # ~55% solid
    plates = (rows + 1) * 1.27e3 * math.pi * (R - r) ** 2 * 0.003 * 0.45 + 0.012       # webbed plates + hub boss
    return OmniWheel(f"Printed {diameter * 1000:.0f} mm omni, {rows} x {rollers_per_row} rollers of "
                     f"{roller_diameter * 1000:.0f} mm", "printed (PETG + TPU 95A tread, 3 mm pins)",
                     6.0 + 0.4 * n, diameter, width, roller + plates, rollers_per_row, roller_diameter, mu,
                     "printed hub on the motor's output flange", rows=rows, printed=True,
                     notes="v7 (est): mass from the part volumes, price = filament + pins; mu for TPU on a hard "
                           "floor (est, VERIFY). Bed: fits a 256 mm bed up to ~250 mm.")


OMNI_WHEELS = {
    "goBILDA-96": OmniWheel(
        "goBILDA 96mm Omni Wheel (14mm bore, 50A)", "3624-0014-0096", 21.99, 0.096, 0.038, 0.119,
        8, 0.016, 0.95, "14 mm round, 16/32 mm patterns",
        notes="Plastic hub, two rows of soft 50A rubber rollers: grippy and light. 119 g, $21.99 (gobilda.com, "
              "Oct 2026). Roller count and diameter from the drawing (VERIFY)."),
    "Nexus-100": OmniWheel(
        "Nexus 100mm double aluminum omni (bearing rollers)", "14054", 35.0, 0.100, 0.029, 0.335,
        9, 0.019, 0.75, "Nexus hub",
        notes="Aluminum plates, 18 rollers, 30 kg rating."),
    "Nexus-127": OmniWheel(
        "Nexus 127mm double aluminum omni (bearing rollers)", "14073", 45.0, 0.127, 0.029, 0.480,
        11, 0.019, 0.75, "Nexus hub",
        notes="Design-doc size class (125 mm). 22 rollers, 30 kg rating."),
    # v7 (design.md 25): bigger wheels for home floors. Sources: maker / reseller pages, Oct 2026
    "goBILDA-120": OmniWheel(
        "goBILDA 3604 Series 120mm Omni Wheel (14mm bore, 70A)", "3604-0014-0120", 11.99, 0.120, 0.040, 0.139,
        10, 0.019, 0.85, "14 mm round",
        notes="139 g, 70A rubber rollers, $11.99, but DISCONTINUED (servocity.com). Width, roller count and "
              "diameter (est)."),
    "AM-DuraOmni-4": OmniWheel(
        "AndyMark 4 in. DuraOmni (TPU over nylon rollers)", "am-3047", 29.0, 0.1016, 0.0508, 0.268,
        4, 0.026, 0.85, "1.125 in bearing bore, 6 x #6 on a 1.875 in circle",
        notes="0.59 lb, 8 rollers (80A TPU tread, nylon core, 1/8 in steel pins), 2 in wide, 120 lb, $29 "
              "(andymark.com). Split into 2 rows of 4 and the roller diameter (est): VERIFY on the drawing."),
    "AM-DuraOmni-6": OmniWheel(
        "AndyMark 6 in. DuraOmni (TPU over nylon rollers)", "am-3154", 42.0, 0.1524, 0.0508, 0.449,
        5, 0.034, 0.85, "1.125 in bearing bore, 6 x #6 on a 1.875 in circle",
        notes="0.99 lb, 10 rollers (80A, RobotShop says 77A), 2 in wide, 120 lb, $42 (andymark.com). Split into "
              "2 rows of 5 and the roller diameter (est): VERIFY on the layout PDF."),
    "Rotacaster-125-R2": OmniWheel(
        "Rotacaster 125mm double (R2), 85A polyurethane rollers", "R2-1258-85", 60.0, 0.125, 0.043, 0.315,
        8, 0.024, 0.8, "keyed or plain bore, 22 mm bearing seats",
        notes="0.315 kg, 43 mm wide, 85A or 95A PU rollers, 57 kg lateral / 115 kg static (rotacaster.com.au). "
              "Price, rollers per row and roller diameter (est)."),
    "Rotacaster-150-R2": OmniWheel(
        "Rotacaster 150mm double, 85A polyurethane rollers", "150mm double 85A", 75.0, 0.150, 0.050, 0.510,
        8, 0.030, 0.8, "keyed or plain bore",
        notes="0.49-0.515 kg (74/85/95A), 70-125 kg (rotacaster.com.au). Price, width, rollers and roller "
              "diameter (est)."),
    "Nexus-152": OmniWheel(
        "Nexus 152mm double plastic omni (brass-bearing rollers)", "14085 (RB-Nex-10)", 79.11, 0.152, 0.029, 0.630,
        14, 0.019, 0.75, "Nexus hub",
        notes="630 g, 28 rollers, 29 mm wide, 40 kg, $79.11 (robotshop.com); 19 mm rollers on the aluminum "
              "version (est here)."),
    "goBILDA-140-mecanum": OmniWheel(
        "goBILDA 140mm Mecanum (70A, bearing-supported rollers)", "3213-3606-0003 (set of 4)", 75.0, 0.140, 0.037,
        0.383, 16, 0.024, 0.85, "16/32 mm patterns",
        rows=1, coverage=1.0, kind="mecanum",
        notes="383 g per wheel, 16 rollers, 37 mm wide, $299.99 a set of 4 (gobilda.com). Roller diameter (est)."),
    # printed designs (v7): big rollers for a big wheel
    "Printed-127-2x7-28": printed_omni(0.127, 7, 0.028),
    "Printed-127-2x6-32": printed_omni(0.127, 6, 0.032),
    "Printed-152-2x7-36": printed_omni(0.152, 7, 0.036),
    "Printed-152-3x6-36": printed_omni(0.152, 6, 0.036, rows=3),
    "Printed-127-2x5-36": printed_omni(0.127, 5, 0.036),
    "Printed-140-2x6-36": printed_omni(0.140, 6, 0.036),
    "Printed-152-2x6-40": printed_omni(0.152, 6, 0.040),
}
# v8 swerve (design.md 26): plain tired wheels for steered modules (kind "tire": one cylinder, no rollers). Printed
# PETG hub + a cast or printed TPU / urethane tread (est; mu on a hard floor est, VERIFY)
OMNI_WHEELS.update({
    "Printed-100-tire": OmniWheel("Printed 100 mm wheel, 28 mm TPU tread", "printed (PETG hub, TPU 95A tread)", 5.0,
                                  0.100, 0.028, 0.085, 1, 0.0, 1.0, "printed hub on the drive motor's output",
                                  rows=1, coverage=1.0, kind="tire", printed=True,
                                  notes="v8 (est): mass from the part volumes; mu 1.0 for TPU on wood (est, VERIFY)."),
    "Printed-127-tire": OmniWheel("Printed 127 mm wheel, 30 mm TPU tread", "printed (PETG hub, TPU 95A tread)", 7.0,
                                  0.127, 0.030, 0.125, 1, 0.0, 1.0, "printed hub on the drive motor's output",
                                  rows=1, coverage=1.0, kind="tire", printed=True,
                                  notes="v8 (est): as Printed-100-tire."),
    "Printed-76-tire": OmniWheel("Printed 76 mm wheel, 25 mm TPU tread", "printed (PETG hub, TPU 95A tread)", 4.0,
                                 0.076, 0.025, 0.055, 1, 0.0, 1.0, "printed hub on the drive motor's output",
                                 rows=1, coverage=1.0, kind="tire", printed=True,
                                 notes="v8 (est): a small wheel, to see how far down the size can go."),
})


# ---------------------------------------------------------------------------
# v8 swerve: steering actuators (one per module). From the shared actuator database (database/servos.csv, Oct 2026)

@dataclass(frozen=True)
class SteerServo:
    name: str
    part: str
    price: float
    mass: float               # kg
    stall_torque: float       # N m at its rated voltage
    speed: float              # rad/s no-load
    notes: str = ""


STEER_SERVOS = {
    "ST3215-HS": SteerServo("Waveshare ST3215-HS 20 kg cm high-speed serial bus servo, 12-bit absolute", "ST3215-HS",
                            21.99, 0.068, 1.96, 11.1,
                            notes="12 V, 0.094 s/60 deg, 2.4 A stall, 68 g, TTL bus, position + load + speed feedback "
                                  "(waveshare.com via database/servos.csv)."),
    "ST3215": SteerServo("Waveshare ST3215 30 kg cm serial bus servo (= Feetech STS3215)", "ST3215", 21.99, 0.055,
                         2.94, 4.7, notes="12 V, 0.222 s/60 deg; mass (est)."),
    "ST3025": SteerServo("Waveshare ST3025 40 kg cm brushless serial bus servo", "ST3025", 79.99, 0.124, 3.92, 9.0,
                         notes="12 V, 0.117 s/60 deg, brushless, current + temperature feedback."),
    "STS3250": SteerServo("Feetech STS3250 50 kg cm serial bus servo", "STS3250-C002", 73.99, 0.075, 4.90, 7.9,
                          notes="12 V, 0.133 s/60 deg; mass (est)."),
}

# diagnostic only (design.md 25.5): the v7 wheel at the goBILDA 96's 119 g
OMNI_WHEELS["Printed-127-2x5-36-light"] = replace(OMNI_WHEELS["Printed-127-2x5-36"], mass=0.119)


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
        "iFlight XING 2806.5 1300KV", "XING 2806.5 1300KV", 40.99, 0.0495, 1300, 0.0485, 1.0, 48.0, 4.5e-6,
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
# Feed-wheel motors (v6 chute: two driven feed wheels hold the ball and pass it to the flywheels)

@dataclass(frozen=True)
class FeedMotor:
    """A small gearmotor (brushed DC or BLDC) and its driver, from datasheet points at the output shaft.

    A DC motor is fixed by five datasheet numbers: rated voltage, no-load speed and current, stall torque and stall
    current. From them (output side): R = V / i_stall, kt = stall torque / (i_stall - i_noload) (gear losses
    included), ke = (V - i_noload R) / w_noload, and the gearbox + bearing drag kt i_noload. A FOC-driven BLDC
    (gimbal motor) is the same model with gear 1. The driver sets the supply rail and caps the phase current.
    kind: "dc" (brushed + H-bridge), "bldc" (gimbal + FOC), "worm" (self-locking: holds any back-driving torque),
    "ideal" (v6 rounds 1-2: a stiff velocity loop with a 0.3 N m cap, holding like a worm).
    """
    name: str
    part: str
    price: float              # motor + its share of the driver, USD
    mass: float               # kg, motor + encoder
    kind: str
    v_rated: float            # V of the datasheet points
    noload_rpm: float         # output rpm at v_rated
    i_noload: float           # A
    stall_torque: float       # N m at the output, at v_rated (often an extrapolation)
    stall_current: float      # A at v_rated
    gear: float
    rotor_inertia: float      # kg m^2, motor side (est unless stated)
    driver: str
    v_supply: float           # V: the rail the driver runs from
    i_limit: float            # A: the driver's current limit (set in firmware / by its sense resistor)
    notes: str = ""

    @property
    def r(self) -> float:
        return self.v_rated / self.stall_current

    @property
    def kt(self) -> float:
        return self.stall_torque / (self.stall_current - self.i_noload)

    @property
    def w_noload(self) -> float:
        return self.noload_rpm * 2 * math.pi / 60

    @property
    def ke(self) -> float:
        return (self.v_rated - self.i_noload * self.r) / self.w_noload

    @property
    def friction(self) -> float:
        """N m of drag at the output (gearbox, brushes, bearings): the no-load current's torque."""
        return self.kt * self.i_noload

    @property
    def reflected_inertia(self) -> float:
        return self.rotor_inertia * self.gear ** 2

    def torque_at(self, w_out: float, v: float | None = None) -> float:
        """Most motoring torque at this output speed on the driver's rail (current and back-EMF limits)."""
        v = self.v_supply if v is None else v
        return self.kt * max(0.0, min(self.i_limit, (v - self.ke * abs(w_out)) / self.r))


FEED_MOTORS = {
    "ideal": FeedMotor(
        "Ideal feed motor (v6 rounds 1-2)", "-", 0.0, 0.012, "ideal", 12.0, 2000.0, 0.0, 0.3, 1.0, 1.0, 2e-5,
        "-", 12.0, 1.0, notes="What v6 assumed: a stiff velocity loop capped at 0.3 N m that holds like a worm."),
    "N20-worm-380": FeedMotor(
        "N20 worm gearmotor, ~380 rpm (est)", "generic JGY-370/N20 worm", 12.0 + 6.0, 0.030, "worm", 12.0, 380.0, 0.05,
        0.30, 0.6, 60.0, 1.0e-8, "DRV8874 carrier (shared)", 12.0, 0.6,
        notes="v6 round 1's assumption (est). Self-locking, but 380 rpm only moves a 40 mm wheel at 0.8 m/s; "
              "the chute needs 1.6 m/s (~764 rpm)."),
    "Pololu-HPCB-30": FeedMotor(
        "Pololu 30:1 Micro Metal Gearmotor HPCB 12V, 12 CPR encoder", "Pololu #5210", 27.0 + 11.94, 0.0105, "dc",
        12.0, 1100.0, 0.08, 0.038, 0.75, 29.86, 2.5e-8, "Pololu DRV8874 carrier #4035 on the 12 V rail", 12.0, 0.75,
        notes="Datasheet: 1100 rpm / 80 mA at 12 V, stall 0.39 kg cm / 0.75 A (extrapolated), 9.5 g + encoder. "
              "Rotor inertia (est). Price of the encoder version (est)."),
    "Pololu-25D-9.7": FeedMotor(
        "Pololu 9.7:1 Metal Gearmotor 25Dx63L HP 12V, 48 CPR encoder", "Pololu #4842", 56.95 + 11.94, 0.095, "dc",
        12.0, 1000.0, 0.30, 0.382, 5.0, 9.68, 6.0e-7, "Pololu DRV8874 carrier #4035 on the 12 V rail", 12.0, 2.1,
        notes="Datasheet: 1000 rpm / 300 mA at 12 V, stall 3.9 kg cm / 5.0 A (extrapolated), 464.6 counts/rev. "
              "Mass and rotor inertia (est). Driver limit = the DRV8874's 2.1 A continuous rating."),
    "Pololu-25D-RoboClaw": FeedMotor(
        "Pololu 9.7:1 25Dx63L HP 12V, 48 CPR encoder, on a RoboClaw 2x7A", "Pololu #4842 + RoboClaw 2x7A (half)",
        56.95 + 40.0, 0.095, "dc", 12.0, 1000.0, 0.30, 0.382, 5.0, 9.68, 6.0e-7, "RoboClaw 2x7A (onboard PID, 12 V rail)",
        12.0, 4.0,
        notes="As Pololu-25D-9.7, driven by a RoboClaw 2x7A (7.5 A continuous per channel, encoder inputs, its own "
              "speed/position PID): current limit set to 4 A (est setting). Half a $79.95 board per motor."),
    "GM2804": FeedMotor(
        "iFlight iPower GM2804 gimbal motor (100T) + AS5048A encoder, FOC", "iFlight GM2804 w/ AS5048A", 48.35 + 15.0,
        0.051, "bldc", 10.0, 1690.0, 0.09, 0.053, 1.11, 1.0, 5.0e-6, "SimpleFOC Mini v1 (DRV8313) on the 12 V rail",
        12.0, 1.3,
        notes="iFlight: 1602-1771 rpm / 0.09 A no-load at 10 V, 9 ohm (+/-5%), 42 g (51 g with encoder). kt from "
              "ke (est); stall at 10 V = 10 / 9 ohm. Direct drive: the TPU tire goes on the 35 mm bell. Driver "
              "rated 8-24 V, so it needs the 12 V rail, not the 7S pack. Bell inertia (est)."),
    "GM3506": FeedMotor(
        "iFlight iPower GM3506 gimbal motor + AS5048A encoder, FOC", "iFlight GM3506 w/ AS5048A", 53.63 + 15.0,
        0.072, "bldc", 16.0, 2262.0, 0.17, 0.171, 2.857, 1.0, 1.2e-5, "SimpleFOC Mini v1 (DRV8313) on the 12 V rail",
        12.0, 2.1,
        notes="iFlight: 2149-2375 rpm / 0.17 A no-load at 16 V, 5.6 ohm (+/-5%), 72 g with encoder, 40 mm bell; "
              "load point 1 A at 12 V for 0.06-0.10 N m. kt from ke (est), stall at 16 V = 16 / 5.6 ohm. On the 12 V "
              "rail the phase current tops out at ~2.1 A, inside the DRV8313's 2.5 A. Bell inertia (est)."),
    "LK-MF4005": FeedMotor(
        "LK-TECH (LKMTECH) MF4005-V2 25T, integrated FOC driver + 18-bit encoder, CAN", "LKMTECH MF4005-V2 (CAN)",
        100.0, 0.105, "bldc", 12.0, 2390.0, 0.15, 0.223, 4.8, 1.0, 8.0e-6, "integrated (CAN: torque, speed, position)",
        12.0, 2.5,
        notes="Reseller listing: 12 V, max 2390 rpm, rated 0.07 N m at 1600 rpm, 18-bit magnetic encoder, CAN or "
              "RS485, torque/speed/position modes, ~$100. kt ~ ke from the max speed, R from the rated point, the "
              "driver's current limit (2.5 A ~ 0.12 N m), mass and inertia (est): no maker datasheet found."),
    # design.md 24: the cheaper integrated LK motor ($75). Reseller data only: 12 V, 1090 rpm no-load, rated 0.07 N m
    # at 450 rpm and 0.65 A, max 0.1 N m. Five points derived from those (est): kt ~ 0.07 / 0.65, stall on the
    # line through the no-load and rated points (0.119 N m at 1.1 A), the 0.1 N m max as the driver's limit.
    "LK-MS4005": FeedMotor(
        "LK-TECH (LKMTECH) MS4005-V3, integrated FOC driver + encoder, CAN", "LKMTECH MS4005-V3 (CAN)",
        75.0, 0.09, "bldc", 12.0, 1090.0, 0.08, 0.119, 1.18, 1.0, 6.0e-6, "integrated (CAN: torque, speed, position)",
        12.0, 1.0,
        notes="Reseller listings (roboticscenter.ai, aifitlab, Oct 2026): 12 V, 1090 rpm no-load, rated 0.07 N m at "
              "450 rpm / 0.65 A, max 0.1 N m, 15- or 18-bit encoder, CAN or RS485. No-load current, stall point, "
              "limit, mass, inertia (est). Exported from database/actuator_db.py (lk-ms4005-v3)."),
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
    # actuator choice (design.md 24): a 6S pack keeps the DJI C610 (rated 24 V) and 6S ESCs inside their ratings
    "6S1P-P50B": BatteryPack("6S1P Molicel P50B, 5 Ah, ~108 Wh", P50B, 6, 1, 0.025, 105.0, 40.0),
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
    restitution: float    # bounce on a hard floor (est); the elastic contact model uses it at every speed
    mu: float
    e_slow: float = 0.55  # viscoelastic model: bounce on a hard floor at 1 m/s (a ~5 cm drop) (est, VERIFY drop test)
    e_fast: float = 0.38  # ... and at 7 m/s (a caught throw); foam loses more the harder it is hit (est)

    @property
    def radius(self) -> float:
        return self.diameter / 2


BALLS = {
    "foam-pickleball": Ball("Pickleball-size foam ball (74 mm)", 0.074, 0.022, 0.50, 1500.0, 0.45, 0.9),
}

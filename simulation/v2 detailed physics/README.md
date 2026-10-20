# v2: detailed physics

Builds on v1 with:

- an electromechanical model of each part,
- a physical launcher and catch path that match the CAD in `../../cad`,
- a foam pickleball,
- a nicer-looking scene,
- the component trade studies.

The design baseline, expected performance and trade-study conclusions are in [../../design.md](../../design.md).

Videos: [catch_demo.mp4](catch_demo.mp4) (45 s: toss → catch → gate → J-bend → flywheels → return throw, with a robot close-up and live battery/flywheel/motor telemetry) and [drive_demo.mp4](drive_demo.mp4) (25 s of omni moves).

## Run it

```powershell
$py = "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe"
& $py run.py                          # catch demo in the MuJoCo viewer
& $py run.py --drive                  # omni-drive demo
& $py run.py --headless -n 30         # 30 tosses, prints catch rate, return-throw accuracy, battery, motor temp
& $py run.py --record catch_demo.mp4 -s 45
& $py run.py --wheel-model rollers    # explicit double-row rollers instead of the fast sphere wheels
& $py launcher_bench.py               # re-fire the launcher at 1000-5000 rpm, rewrite launcher_cal.json
& $py -m pytest -q                    # 7 checks, ~5 s
& $py studies\wheel_slip.py           # and drive_components.py, drive_reach.py, launcher_components.py, battery.py
```

It runs at about 4× real time with a 0.5 ms step. At 1 ms, flywheel shots above ~4000 rpm come out erratic.

## Files

| File | What it is |
| --- | --- |
| `catalog.py` | Datasheet values for every candidate part (drive motors, omni wheels, drone motors, ESC, cells, ball); (est) marks estimates. A characterized motor is just a new entry |
| `params.py` | Which catalog parts are fitted, plus geometry, masses (from CAD) and limits. `SimParams(wheel="Nexus-127")` swaps a part and the robot rebuilds itself |
| `models.py` | `DriveActuator` (FOC speed loop, current and back-EMF limits, gearbox, thermal), `FlywheelDrive` (drone motor + AM32 ESC + MCU rpm loop), `GateServo`, `Battery` (OCV, R0, RC branch, BMS trip) |
| `robot_model.py` | MJCF builder: base, both omni-wheel models, funnel, catch pocket, trapdoor gate, J-bend, flywheels, scene and graphics |
| `robot.py` | The robot between firmware and physics: sensors in, wheel/flywheel/gate commands out, electrical bookkeeping each step |
| `catch_game.py` | Toss → predict → stop under → catch → aim → spin up → gate → launch → return |
| `launcher_bench.py`, `launcher_cal.json` | Launcher calibration: flywheel speed → ball exit speed and elevation |
| `kiwi_control.py`, `ball_flight.py` | Kinematics and acceleration-limited base controller; drag-aware landing prediction and throw solvers |
| `assets.py` | Procedural textures (maple floor, net) and meshes (funnel cone, barrel rollers) |
| `studies/` | Trade studies; results land in `studies/results/` |

## Modeling notes worth knowing

- **Drive motors.** MuJoCo applies the driver's proportional speed term implicitly, so it's stable at any gain. Python adds the integral term and updates the force range every step from the current and back-EMF limits at the present battery voltage. Torques actually applied go back through the model to give phase current, bus current, copper loss, winding temperature and battery sag.
- **Foam squeeze.** MuJoCo's soft contacts scale stiffness with the constraint's effective mass, which is unbounded for a ball pinched between two wheels; the first attempt produced 250 N pinch forces and stalled flywheels. Here each flywheel sits on a sprung slide (~3 kN/m per wheel, about 15 N at full squeeze) and the ball-wheel contact is stiff, so friction is proper Coulomb.
- **Friction-critical contacts** (wheels-floor, ball-flywheel, ball-funnel, ball-gate) are explicit `<pair>`s with stiff `solreffriction`; MuJoCo's default soft friction behaves like a weak damper.
- **Funnel net.** Modeled as grippy (μ 1.0) with rolling resistance, otherwise off-center balls swirl around the cone and climb out.
- **Explicit roller model.** The rollers' bearing drag is scaled by (roller radius / wheel radius)² so both wheel models see the same sideways drag.
- **Perception** is still ground truth after a 0.25 s delay at 20 Hz. Detection noise, latency jitter and dropped frames are the next things to add, and they will start to separate the drive options.

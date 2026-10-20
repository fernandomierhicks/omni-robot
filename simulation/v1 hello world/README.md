# v1 hello world: kiwi play-catch robot in MuJoCo

A first, deliberately simple MuJoCo model of the play-catch robot. It covers the
kiwi base, the catch module and a 5 × 4 m test area, plus a toss → catch → throw-back
loop that runs in the viewer or headless. There is no detailed CAD yet.

Video: [catch_demo.mp4](catch_demo.mp4) (20 s, side camera).

## Run it

The sim uses its own Python environment, kept outside Dropbox so it doesn't sync:

```powershell
# one-time setup (already done on this PC)
& "$env:USERPROFILE\.platformio\python3\python.exe" -m venv "$env:USERPROFILE\.venvs\omni-robot-sim"
& "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe" -m pip install -r ..\requirements.txt

$py = "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe"
& $py hello_world.py                    # catch demo in the MuJoCo viewer
& $py hello_world.py --drive            # omni-drive demo: forward, strafe, turn-while-moving, spin, circle
& $py hello_world.py --headless -n 30   # 30 tosses, no graphics, prints the catch rate
& $py hello_world.py --record catch_demo.mp4 -s 20
& $py -m pytest -q                      # 10 headless checks, ~6 s
```

In the VS Code Python extension, select `~\.venvs\omni-robot-sim\Scripts\python.exe` as the interpreter.
Viewer tips: press `3` to show the hidden wheel contact spheres, and `Tab` / `Shift+Tab` to show the side panels.

## Files

| File | What it is |
| --- | --- |
| `kiwi_model.py` | `SimParams` (all dimensions, masses, limits) and the MJCF generator. Run it to write `scene.xml` for MuJoCo's stand-alone `simulate` app |
| `kiwi_control.py` | Kiwi kinematics (twist ↔ wheel speeds) and an acceleration-limited go-to-pose controller |
| `ball_flight.py` | Ball flight with drag (same law MuJoCo uses), landing prediction, toss and launch solvers |
| `hello_world.py` | Catch game state machine, drive demo, and the viewer / headless / video runners |
| `test_hello_world.py` | Mass/CG range, standing still, kinematics, drag-model agreement, funnel + break-beam, catch rate, drive demo |

## What is modeled

- **Base:** 3 × 125 mm omni wheels at 60° / 180° / 300°, 18 cm from the center, under a 45 cm deck. Battery and motors are low and the lidar sits under the payload plate. The robot weighs 6.6 kg with the CG at 17.5 cm.
- **Motors:** AK40-10 as a velocity servo, limited to 4.1 N·m peak torque and 38.7 rad/s (370 rpm), which gives a 2.4 m/s top speed. v2 replaces this with a motor and battery model.
- **Omni wheels:** a driven hub (spins visibly) plus a contact sphere on a ball joint. A stiff damper couples the sphere to the hub about the axle only, so the wheel drives in its rolling direction and rolls freely along its axle. Friction is Coulomb (μ = 0.8, elliptic cone).
- **Catch module:** a 33 cm funnel at 0.6 m (20 panels, 35° from vertical) feeds a square chamber. A rangefinder across the chamber acts as the break-beam, and two stereo cameras sit on the rim (`stereo_left` / `stereo_right`, renderable).
- **Ball:** pickleball-sized foam ball (74 mm, 25 g) with MuJoCo's ellipsoid drag model (Cd 0.5). Drag is large for a ball this light: about 25% of its weight at 6 m/s.
- **Launcher:** modeled by its output, a 52° launch with 2% speed and 0.7° angle scatter, as the design doc specifies. v2 models the flywheels.
- **Perception:** ground truth after a 0.25 s delay, refreshed at 20 Hz. Noise, latency jitter and dropped frames come later.

## Findings so far

1. **Traction, not torque, limits acceleration.** Accelerating between the two front wheels is pushed by those two wheels alone, each carrying 0.58·m·a, while load transfer unloads them. With μ = 0.8 and this CG the wheels start to slip at about 3.1 m/s², below the doc's 4–5 m/s² target. The base is capped at 3.0 m/s² here; v2 quantifies this properly.
2. **Kiwi tip-over is worse than the doc's formula suggests.** The support polygon is the wheel triangle, whose inradius is half the wheel circle radius (9 cm here, not the 22.5 cm half-width). Tip-over toward a triangle edge starts near 5 m/s², and the doc's "45 cm wide, CG at 40 cm" example would tip at about 2.2–2.5 m/s² (wheels 18–20 cm out), not 5.5.
3. **With perfect perception the stop-and-catch loop works:** 30/30 catches inside a 0.37 m catch zone, with a median funnel-center error of 0.5 cm.

## Modeling notes

- The contact sphere carries 0.15 kg of the 0.25 kg wheel mass. A lighter sphere makes the 2 ms step drift in yaw (tested down to 20 g). The heavier sphere costs about 0.06 kg of sideways rolling inertia per wheel, about 1% of robot mass.
- MuJoCo sensors on `objtype="body"` report the inertial (CoM) frame. Use `xbody` for the body frame.
- The robot reads only sensors (`base_pos`, `base_quat`, `ball_pos`, `catch_beam`, wheel speeds) and writes wheel speed commands. Placing the ball for tosses and launches is done by the "world", not the robot.

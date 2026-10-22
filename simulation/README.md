# Simulation

| Version | What it adds |
| --- | --- |
| [v1 hello world](v1%20hello%20world/README.md) | Kiwi base, catch module, toss → catch → throw-back loop; velocity-servo motors, launcher modeled by its output |
| [v2 detailed physics](v2%20detailed%20physics/README.md) | AK40-10 motor + driver model, battery sag, omni-wheel traction/slip (plus an explicit-roller check), physical drone-motor flywheel launcher, trapdoor gate and J-bend fed from the funnel, foam pickleball, nicer graphics, component trade studies |
| [v3 compact](v3%20compact/README.md) | Proposed compact robot: 4-wheel X base with compliant wheels and a rocker, catch/launch module on one tilt joint (aligns the funnel with the ball, sets the launch elevation), shallower funnel, base-mounted cameras; studies of wheel count, tip-over, funnel tilt and camera placement |
| [v5 unlimited](v5%20unlimited/README.md) | The v3b robot with every firmware limit removed (only motors, traction, tip-over and the BMS left), a per-pixel camera/vision model (blob detector, stereo + known-size ranging, image-space filter) with a sensitivity sweep, and a multi-ball hopper mode (magazine tube, shooting at targets, on the move). There is no v4 |
| [v6 shallow throw](v6%20shallow%20throw/README.md) | Shallow throws from 6-7 m: roller feed (driven feed wheels hold the ball and pass it to the flywheels, one per trigger), padded pocket, catch tilt to 35 deg, honest stereo track start; multi-ball options including a tennis-machine indexer wheel |
| [v10 ros bridge](v10%20ros%20bridge/README.md) | v9 (v7 home-floor base, sensors, estimator) as a ROS 2 Jazzy robot in WSL2: MuJoCo publishes `/clock`, `/scan` (RPLIDAR A2M12), `/imu` (BNO085), `/odom`, TF and takes `/cmd_vel`; robot_localization, slam_toolbox, Nav2, RViz (WSLg), Foxglove and a flight stick run against it. Its ROS side runs in WSL (`ros/omni.sh`; design.md 28) |
| [v11 sil](v11%20sil/README.md) | v10 with the robot's own code in the loop: the Teensy firmware's C++ core ticked every 1 ms (software in the loop, behind raw CAN frames and sensor values), and the Jetson's catch code (pixel detector, filter, behaviour, modes, MCU link) in one process or as ROS nodes. The default sim for `ros/omni.sh` (design.md 30) |

The design baseline (with sim-predicted performance and the recommended parts) is [../design.md](../design.md); the printable CAD that matches v2 is in [../cad](../cad/README.md).

## Python environment

The only Python on this PC is PlatformIO's bundled interpreter, so the sim has its own venv outside Dropbox (so it doesn't sync):

```powershell
& "$env:USERPROFILE\.platformio\python3\python.exe" -m venv "$env:USERPROFILE\.venvs\omni-robot-sim"
& "$env:USERPROFILE\.venvs\omni-robot-sim\Scripts\python.exe" -m pip install -r requirements.txt build123d
```

The setup is already done on this PC (MuJoCo 3.16, build123d 0.13). In VS Code, select `~\.venvs\omni-robot-sim\Scripts\python.exe` as the interpreter.

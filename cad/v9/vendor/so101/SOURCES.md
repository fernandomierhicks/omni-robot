# SO-101 STEP parts

TheRobotStudio's SO-ARM100 repository (github.com/TheRobotStudio/SO-ARM100) at commit 5f6d2b876a53a4872e405b991dd925556c9e38a4 (the commit `simulation/v11 sil/arm_so101.py` pins for the sim's meshes), downloaded Oct 27 2026. Licence: Apache-2.0 (`LICENSE` here, from the same commit). Unmodified.

Each file is the follower arm's printed part (or the Feetech STS3215 servo) in the same frame as the sim's STL of it (checked to 0.1 mm), so `cad/v8/build.py` places them with the sim's poses (`simulation/v11 sil/studies/layout_trade.py arm-poses`). The STS3215 STEP has its horn; the sim's wrist servo mesh has none (there's no STEP without it). To change a part: edit its STEP here (or in SolidWorks), rebuild (`python build.py` or `python step_export.py --kit arm`); for the sim, export the same part as STL in metres to `simulation/v11 sil/assets/so101/`.

| File | Source path | sha256 (16) |
| --- | --- | --- |
| Base_SO101.step | [STEP/SO101/Base_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Base_SO101.step) | 95209ec6a3ca06b1 |
| Base_motor_holder_SO101.step | [STEP/SO101/Base_motor_holder_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Base_motor_holder_SO101.step) | f13462eea875727a |
| Motor_holder_SO101_Base.step | [STEP/SO101/Motor_holder_SO101_Base.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Motor_holder_SO101_Base.step) | fb2ef4603324ddea |
| Motor_holder_SO101_Wrist.step | [STEP/SO101/Motor_holder_SO101_Wrist.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Motor_holder_SO101_Wrist.step) | e58d86502535a5a4 |
| Rotation_Pitch_SO101.step | [STEP/SO101/Rotation_Pitch_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Rotation_Pitch_SO101.step) | a392681efb05bf40 |
| Upper_arm_SO101.step | [STEP/SO101/Upper_arm_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Upper_arm_SO101.step) | efa19a6dd2ccb459 |
| Under_arm_SO101.step | [STEP/SO101/Under_arm_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Under_arm_SO101.step) | 677a1a57efeddc43 |
| Wrist_Roll_Pitch_SO101.step | [STEP/SO101/Wrist_Roll_Pitch_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Wrist_Roll_Pitch_SO101.step) | 39c5e78647d471ab |
| Wrist_Roll_Follower_SO101.step | [STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step) | b7f9d59f7991f425 |
| Moving_Jaw_SO101.step | [STEP/SO101/Follower_Specific/Moving_Jaw_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/Follower_Specific/Moving_Jaw_SO101.step) | dff53ace072470c9 |
| WaveShare_Mounting_Plate_SO101.step | [STEP/SO101/WaveShare_Mounting_Plate_SO101.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO101/WaveShare_Mounting_Plate_SO101.step) | 16510f5aedf04186 |
| STS3215_03a.step | [STEP/SO100/STS3215_03a.step](https://github.com/TheRobotStudio/SO-ARM100/blob/5f6d2b876a53a4872e405b991dd925556c9e38a4/STEP/SO100/STS3215_03a.step) | cacd717c2f22bec8 |

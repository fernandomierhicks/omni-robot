0. Sensor update first, in v10 (and back-port to v9 if small). Fernando chose the RPLIDAR A2M12 lidar and
   the BNO085 IMU (design.md section 7).
   - Update sensors.py: A2M12 = 16 kHz samples, 10 Hz typical (5-15 Hz), 0.225 deg at 10 Hz, 0.2-12 m,
     accuracy 1% of range to 3 m, 2% at 3-5 m, 2.5% beyond, ±1.5 deg scan flatness, UART 256 kbaud
     (Slamtec datasheet).
   - BNO085: on-chip fusion, game rotation vector up to 400 Hz, raw gyro up to 1 kHz. Get noise, bias
     and drift from CEVA's BNO08x datasheet; mark anything not found as (est).
   - Check the A2M12's real height and scan-plane position against the deck and payload plate (its
     housing is taller than the C1's), and the 0.2 m minimum range against the robot's own standoffs.
   - Rerun studies/localization.py and report what changed.


code


-GUI

-





   One Jetson caveat (VERIFY): the Orin's JetPack 6 is Ubuntu 22.04, and Jazzy needs 24.04. So Jazzy either runs in Docker on the Jetson, as section 5 already says, or waits for a JetPack 7 release for Orin, if one exists by then.



CAD:
add rx receiver

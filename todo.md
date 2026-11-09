

---HARDWAREW
-CubeMars AK40-10 driver: written (Nov 7, servo mode, firmware/lib/omni_fw/ak_motor.h), the arm build compiles and the
 v11 SIL emulates the motor. On the bench (console Test > Bring-up "AK40-10 setup" and "speed scale"): servo mode, ids
 1-4, send_can_status 500 Hz, timeout 100 ms + brake current in the CubeMars tool; common ground; 14 pole pairs;
 the status position's wrap; the error codes' clearing; the 60000 ERPM limit. Check against the AK40-10 V3.0 manual.
-Computer + global shutter camera? super deep trade study. 


-How can we give claude full control of robot? talking and understanding speech ( highes level commander, "go to kitchen and pickp up the red ball") ai_architecutre.md

CAD:
- Add bumper switches (front and rear) to the CAD. Wire them normally closed (a broken wire reads as pressed), then set
  HAS_BUMPERS = true for the arm robot in the firmware params (robot_export.py) and add a bumper latch: after a hit the
  base stays stopped until the bumper is released AND a fresh command arrives (today a released bumper resumes the old
  command at once).




Before a per-camera model can be fitted into the sim, the bench still needs a distance sweep (1–6 m), an angle sweep (0–70°), a run with the camera turning, and an Arducam calibration. The camera server on the Pi is running again, so the Cameras tab works.


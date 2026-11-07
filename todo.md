

---HARDWAREW
-CubeMars AK40-10 CAN driver for the firmware (the adopted drive motor). The firmware only speaks Damiao (dm_motor.h),
 so the arm robot's hardware build stops with an #error until this exists. Write it from CubeMars's published CAN
 protocol behind the same motor interface, mark it VERIFY, and make the v11 sim emulate the AK motor so SIL runs the
 code that will be flashed.
-Computer + global shutter camera? super deep trade study. 


-How can we give claude full control of robot? talking and understanding speech ( highes level commander, "go to kitchen and pickp up the red ball") ai_architecutre.md

CAD:
- Add bumper switches (front and rear) to the CAD. Wire them normally closed (a broken wire reads as pressed), then set
  HAS_BUMPERS = true for the arm robot in the firmware params (robot_export.py) and add a bumper latch: after a hit the
  base stays stopped until the bumper is released AND a fresh command arrives (today a released bumper resumes the old
  command at once).




Before a per-camera model can be fitted into the sim, the bench still needs a distance sweep (1–6 m), an angle sweep (0–70°), a run with the camera turning, and an Arducam calibration. The camera server on the Pi is running again, so the Cameras tab works.


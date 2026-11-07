# The console on the Jetson: page and launcher at boot

The supervisor (`omni_console/supervisor.py`) is the robot's front door. systemd starts it at boot; it serves the
operator page on port 8080 and starts the robot's software (`config/robot_steps.json`, step `robot`) at once. From the
PC or the iPad, open `http://omni.local:8080` and pick a mode: there is no launcher to run. Its System tab restarts the
software, shows its log, and shuts the Jetson down.

Not tried on a Jetson yet (no hardware): the steps below are the plan. VERIFY each one on the robot.

1. A user `omni` with the workspace in `/home/omni/omni_ws`, built with `colcon build --packages-skip omni_sim`.
2. The unit:

   ```bash
   sudo cp systemd/omni-console.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable --now omni-console
   journalctl -u omni-console -f        # the supervisor's own log; each step's is in ~/.omni/logs/steps/
   ```

3. The name `omni.local`: set the hostname to `omni` (`sudo hostnamectl set-hostname omni`); Ubuntu's avahi-daemon
   announces it, and iPad Safari and Windows both resolve `.local` names.
4. Shutdown and reboot from the page: the supervisor runs `sudo -n systemctl poweroff | reboot`. Allow exactly those:

   ```bash
   echo 'omni ALL=(root) NOPASSWD: /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot' | sudo tee /etc/sudoers.d/omni-power
   sudo chmod 440 /etc/sudoers.d/omni-power
   ```

5. Optional PIN for the System tab and power (kid-proofing, not security): `Environment=OMNI_CONSOLE_PIN=1234` in the
   unit.
6. JetPack 6 is Ubuntu 22.04 and the stack is Jazzy (24.04): if Jazzy runs in Docker, the unit starts the container and
   the supervisor runs inside it (with `--network host`, so 8080 and rosbridge's 9090 are the Jetson's ports).
7. The secure page, `https://omni.local:8443` (`OMNI_CONSOLE_HTTPS` in the unit; `openssl` makes a self-signed
   certificate in `~/.omni/tls` on first start). Browsers let a page use the microphone only when it is secure, so the
   Call page's push-to-talk works there; rosbridge and the camera streams go through the same port. Each phone or
   tablet warns about the certificate once: accept it. The robot's own microphone (Listen on the Call page) is
   `arecord` on the default ALSA capture device (`OMNI_MIC_CMD` to change it; VERIFY the device on the robot).

No browser runs on the Jetson: drawing the page costs CPU and memory the robot needs, so the page always runs on the
viewing device. For a screen on the robot, mount an old phone or tablet showing the page.

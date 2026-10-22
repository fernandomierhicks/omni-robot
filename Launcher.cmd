@echo off
rem omni-robot launcher: double-click. Buttons for the sims, the ROS 2 stack in WSL, Foxglove and the joystick.
start "" C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe -NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File "%~dp0launcher.ps1"

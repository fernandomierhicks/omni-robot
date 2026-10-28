"""The robot's face and voice on Windows (the sim): an 800 x 480 window (the 5 in panel) with sound, fed by the face
node in WSL; it listens for voice commands on the PC's microphone (Windows' recognizer) while the console's Face tab
has voice commands on.

The face node starts it by itself (ros/omni.sh tells it where this is); the Launcher's "Robot face" button opens it
too, and "Face demo" plays a scripted session with no robot running. Keys: M mutes, L switches the microphone off and
on, Esc / Q closes.

  python face_view.py [--demo] [--scale 1.25] [--mute] [--listen sapi|none] [--port 9881]
"""

import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "omni_face"))
runpy.run_module("omni_face.display", run_name="__main__")

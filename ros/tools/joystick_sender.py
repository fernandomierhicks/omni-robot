"""Windows side of the joystick relay (design.md 28): read the flight stick with pygame and stream it to joy_relay in WSL.

  & $py "ros\\joystick_sender.py"            (Windows venv; start it after `omni.sh map`, `nav` or `sim`)
  & $py "ros\\joystick_sender.py" --list     show the controllers it sees and which it ignores

3D mice (3Dconnexion, SpaceMouse, KMJ) also show up as joysticks; they are skipped. The first remaining controller
is used (--name picks one by part of its name). Axes are sent in ROS joy_node's convention: pygame reports forward /
left / twist-left as negative, ROS as positive, so all axes are negated. WSL2 forwards localhost TCP to its own
ports, so --host localhost reaches the relay.
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

IGNORE = ("KMJ", "3Dconnexion", "SpaceMouse")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=9870)
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--name", help="use the controller whose name contains this")
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()
    import os
    os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")     # keep reading without a focused window
    import pygame
    pygame.init()
    pygame.joystick.init()
    sticks, ignored = [], []
    for k in range(pygame.joystick.get_count()):
        js = pygame.joystick.Joystick(k)
        js.init()
        name = js.get_name()
        if any(s in name for s in IGNORE):
            ignored.append(name)
            continue
        if a.name and a.name.lower() not in name.lower():
            continue
        sticks.append(js)
    for n in ignored:
        print("ignoring", n, "(a 3D mouse's virtual joystick)")
    for js in sticks:
        print(f"controller: {js.get_name()} ({js.get_numaxes()} axes, {js.get_numbuttons()} buttons)")
    if a.list:
        return
    if not sticks:
        sys.exit("no joystick found (other than 3D mice)")
    js = sticks[0]
    while True:
        try:
            s = socket.create_connection((a.host, a.port), timeout=2.0)
        except OSError:
            print(f"waiting for joy_relay on {a.host}:{a.port} ...")
            time.sleep(2.0)
            continue
        print(f"connected; streaming {js.get_name()} at {a.rate:.0f} Hz (Ctrl-C to stop)")
        s.sendall((json.dumps(dict(name=js.get_name(), ignored=ignored)) + "\n").encode())
        try:
            while True:
                pygame.event.pump()
                axes = [-js.get_axis(i) for i in range(js.get_numaxes())]
                buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
                s.sendall((json.dumps(dict(axes=axes, buttons=buttons)) + "\n").encode())
                time.sleep(1.0 / a.rate)
        except OSError:
            print("relay gone; reconnecting")
        except KeyboardInterrupt:
            return
        finally:
            s.close()


if __name__ == "__main__":
    main()

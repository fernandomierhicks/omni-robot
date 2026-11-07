"""Kids' programs, map zones and rooms, frames (tf), part of missions.Console. ROS-free."""

from __future__ import annotations

import math

from omni_console.console_defs import (Reply, _name, _program_check)  # noqa: F401


class EditMixin:
    def _program(self, verb: str, a: dict) -> Reply:
        from omni_console.program import ProgramRunner
        if verb == "program_save":
            name = _name(a, "A program")
            self.programs[name] = _program_check(a["blocks"])
            self.programs_changed = True
            return Reply(True, f"Saved program '{name}'", self.schema())
        if verb == "program_delete":
            name = str(a["name"])
            if self.programs.pop(name, None) is None:
                return Reply(False, f"No program called '{name}'")
            self.programs_changed = True
            return Reply(True, f"Deleted program '{name}'", self.schema())
        if verb != "program_run":
            return Reply(False, f"Unknown command '{verb}'")
        name = str(a.get("name", "")).strip() or "unsaved"
        blocks = a.get("blocks", self.programs.get(name))
        if blocks is None:
            return Reply(False, f"No program called '{name}'")
        runner = ProgramRunner(name, _program_check(blocks), self.values["program.speed"],
                               math.radians(self.values["program.turn"]))
        if not runner.steps:
            return Reply(False, "The program is empty: add some blocks")
        if any(s[0] == "goto" for s in runner.steps):
            missing = [s[1] for s in runner.steps if s[0] == "goto" and s[1] not in self.places]
            if missing:
                return Reply(False, f"No saved place called '{missing[0]}' on this map")
            bad = self._need_nav()
            if bad:
                return bad
        r = self._enter("PROGRAM", f"Program '{name}': starting", "info")
        self.runner = runner
        return Reply(True, f"Running '{name}' ({len(runner.steps)} steps)", None, r.effects)

    def _zones(self, verb: str, a: dict) -> Reply:
        from omni_console.zones import check_edge, check_zone
        if self.map_name in (None,) and self.slam.get("mode") is None:
            return Reply(False, "No map: no-go zones and edges are drawn on a map")
        if verb == "zone_save":
            name = _name(a, "A no-go zone")
            self.zones[name] = check_zone(a["points"])
            msg = f"No-go zone '{name}' saved: Nav2 plans around it"
        elif verb == "edge_save":
            name = _name(a, "An edge")
            self.edges[name] = check_edge(a["points"], a.get("height_mm", 10.0), a.get("speed", 1.0),
                                          a.get("kind", "rug"))
            msg = f"Edge '{name}' saved: {self.edges[name]['speed']:g} m/s within 35 cm of it"
        else:
            book = self.zones if verb == "zone_delete" else self.edges
            name = str(a["name"])
            if book.pop(name, None) is None:
                return Reply(False, f"Nothing called '{name}'")
            msg = f"Deleted '{name}'"
        self.places_changed = self.zones_changed = True
        return Reply(True, msg, self.schema(), (("masks",),))

    def _rooms(self, verb: str, a: dict) -> Reply:
        """A room's outline on the map (map frame), by name: labels for "drive to the kitchen"; not a no-go zone."""
        from omni_console.zones import check_zone
        if verb == "room_save":
            name = _name(a, "A room")
            self.rooms[name] = check_zone(a["points"])
            msg = f"Room '{name}' saved ({len(self.rooms[name])} corners)"
        else:
            name = str(a["name"])
            if self.rooms.pop(name, None) is None:
                return Reply(False, f"No room called '{name}'")
            msg = f"Deleted room '{name}'"
        self.places_changed = True
        return Reply(True, msg, self.schema())

    def _tf(self, verb: str, a: dict) -> Reply:
        """Read-only: a frame's planar pose in another (x, y m, yaw deg), or the frames the console knows. The console
        keeps the newest transform of each frame in 2D (TfLite): enough to place things on the map."""
        tf = self.tf
        if tf is None:
            return self._err("no_robot", "No transforms yet")
        if verb == "frames":
            return Reply(True, "", dict(frames={c: e[0] for c, e in sorted(tf.tf.items())}))
        parent, child = str(a.get("parent", "map")), str(a["child"])
        p = tf.lookup(parent, child)
        if p is None:
            return self._err("not_found", f"No transform from '{parent}' to '{child}' (the frames: verb frames; "
                                          f"the child must lie under the parent in the tree)")
        return Reply(True, "", dict(parent=parent, child=child, x=round(p[0], 4), y=round(p[1], 4),
                                    yaw_deg=round(math.degrees(math.remainder(p[2], 2 * math.pi)), 2),
                                    stamp=round(p[3], 3), planar=True))

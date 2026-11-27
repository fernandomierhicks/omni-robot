"""Definitions shared by the console core's modules (missions.py and its mixins): the missions, verbs, limits,
settings, Reply and the small helpers. ROS-free."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

VERSION = "0.4.0"                 # the console protocol: the page warns when it differs from the robot's
MISSIONS = ("IDLE", "MANUAL", "GOTO", "EXPLORE", "PATROL", "CATCH", "THROWER", "FOLLOW", "PROGRAM", "CALIBRATE",
            "TIDY", "FETCH", "MESSAGE", "TOWER", "DANCE", "GOALIE", "MOTORS")
MODE_OF = {"IDLE": "IDLE", "MANUAL": "TELEOP", "GOTO": "TELEOP", "EXPLORE": "TELEOP", "PATROL": "TELEOP",
           "CATCH": "CATCH", "THROWER": "THROWER", "FOLLOW": "TELEOP", "PROGRAM": "TELEOP",
           "CALIBRATE": "TELEOP", "TIDY": "PICK", "FETCH": "PICK", "MESSAGE": "TELEOP", "TOWER": "PICK",
           "DANCE": "TELEOP", "GOALIE": "TELEOP", "MOTORS": "WHEELS"}   # mission -> mode_manager mode (FETCH switches)
N_WHEELS = 4                       # the drive motors, in the firmware's order (fw_params DM_IDS: CAN ids 1-4)
WHEEL_TEST_MAX = 37.2              # rad/s: the motor's own limit, the firmware's motor-test cap (firmware.h WHEEL_TEST_MAX
                                   # = fw_params_arm.h MAX_WHEEL_SPEED: 0.95 x the arm robot's AK40-10 no-load speed at
                                   # 24 V, measured Nov 13: 355 rpm; it was 44.9, its ERPM limit, from the maker's chart)
WHEEL_TEST_ACCEL = 20.0            # rad/s^2: the firmware's default motor-test ramp (firmware.h WHEEL_TEST_ACCEL) ...
WHEEL_TEST_ACCEL_MAX = 2000.0      # ... and the most it takes (protocol 9, CMD a_brake: WHEEL_TEST_ACCEL_MAX)
WHEEL_PROFILE_MAX_S = 60.0         # s: the longest motor-test profile (motors_run)
ARM_POSES = ("stow", "ready", "high", "drop", "look")   # look: the wrist camera ahead, 24 deg up (omni_arm pick.py)
ARM_JOG_MAX = 30.0                 # deg: one jog of one joint (omni_arm's motion.JOG_MAX)
ARM_STALE = 2.0                    # s without /arm/state: the arm node isn't running
NAV_MISSIONS = ("GOTO", "EXPLORE", "PATROL")
JAM_MOVE = 0.5          # m: getting this far from where it was is headway (Nav2's backup recovery moves 0.3 m)
EXPLORE_NEAR = 0.3      # m from a frontier goal for EXPLORE_NEAR_S (robot time): there (the lidar sees all round).
EXPLORE_NEAR_S = 8.0    # Nav2 wants 0.10 m and in a small room MPPI hovered 0.17 m short in the goal's inflation for
                        # 100 s (Nov 25, hm9, bathroom 1). Not at once: a new map's first goals are 0.5-0.7 m off and
                        # cutting them short left slam_toolbox no travel to add a scan: "nothing left" at 2 m² (hm10)
BLOCK_AHEAD, BLOCK_R = 0.7, 0.45   # m: Explore's learned no-go spot, on the plan this far ahead, this big (missions
                        # _block_ahead). Nav2 applies its keepout filter after inflation, so the disc must close a 1 m
                        # corridor by itself (0.15 m left a strip the planner took, hm20); its edge stays 0.25 m from
                        # the robot's centre, off the robot's own cell
HOME_RETRIES, HOME_RETRY_S = 2, 10.0   # Explore's drive back: Nav2 giving up is tried again this often, after this
                        # long (a floor-camera mark may clear; Nov 25, hm12 gave up at once with the map done)
BACKOUT_CLEAR = 0.29    # m from walls (and floor marks) Nav2 can plan from: its robot_radius 0.24 + a cell. Nearer, a
                        # goal's failure is the robot's spot, not the goal's (missions _backout)
BACKOUT_MAX = 2.0       # m of its own trail Explore backs out along, at most
BACKOUT_SPEED = 0.12    # m/s backing out (the drive guard still shapes it)
BACKOUT_S = 30.0        # s (robot time): a back-out ends after this, there or not
BACKOUT_STALL = 0.5     # m: a stall on the way to a frontier backs out at least this far (missions _backout)
BACKOUT_MOVED = 0.2     # m: a back-out that moved the robot this far restarts the jam check's headway
TRAIL_STEP, TRAIL_N = 0.1, 60   # m between the trail's points, points kept (6 m)
# Nov 26 (world:=course): MPPI stalls in tight gaps (a 0.8 m gap leaves the robot's centre ~0.3 m: nearly every sampled
# trajectory clips the inscribed zone, so it asks ~0.02 m/s and sits; the home's 24 in door). After CREEP_AFTER s of no
# headway toward a goal the console follows Nav2's own plan (NavFn keeps it centred) CREEP_LEN m at CREEP_SPEED on the
# manual input, the guard still shaping it, then hands back to MPPI. Once per spot (headway resets it)
CREEP_AFTER = 10.0      # s (robot time) of no headway
CREEP_LEN = 1.5         # m of the plan, from its point nearest the robot
CREEP_SPEED = 0.12      # m/s
CREEP_S = 30.0          # s (robot time): a creep ends after this, there or not
FLOOR_STALE = 2.0       # s without the height map's state (2 Hz): it isn't running
FLOOR_GRACE = 3.0       # s the floor camera may be blind while Explore drives before it stops
FLOOR_WAIT = 45.0       # s Explore waits at the start for the map and the floor camera
DRIVE_MISSIONS = ("MANUAL", "FOLLOW", "PROGRAM", "CALIBRATE")   # the console drives: drive_output()
LAUNCHER_RPMS = (2500.0, 3000.0, 3500.0, 4000.0, 4500.0)        # the launcher calibration's shots (est: the range used)
MANUAL_TIMEOUT = 0.3               # s: the console's stick stops counting (dead man released, tab closed, Wi-Fi gone)
ZERO_TAIL = 0.3                    # s of zero twist after that, then silence (Nav2 / Foxglove / the joystick may drive)
MAP_WAIT = 6.0                     # s Explore waits for a fresh /map after a goal ends (slam_toolbox's interval is 2 s)
KID_SPEED = 0.8                    # m/s, the firmware's kid cap (shown, not enforced, here)
FLY_RPM_MAX = 6000.0               # the console's slider; the firmware caps by profile (kid lower)
TILT_RANGE = (-30.0, 40.0)         # deg, the module's travel
MAP_NAME = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
NEW_MAP = "_new"                   # places / routes made while mapping, before the map has a name
VERBS = ("ping", "schema", "stop", "mission", "goto", "cancel", "set", "place_save", "place_delete", "explore",
         "patrol", "route_save", "route_delete", "thrower", "fire", "slam_new", "slam_save", "slam_load",
         "initial_pose", "follow", "program_save", "program_delete", "program_run", "zone_save", "zone_delete",
         "edge_save", "edge_delete", "calib", "calib_start", "calib_measure", "calib_shot", "calib_fit",
         "calib_record", "calib_clear", "player_set", "stats", "health", "maint_done", "tidy", "arm", "say", "hear",
         "ai_lease", "move", "find", "pick", "task", "look", "room_save", "room_delete", "tf", "frames",
         "preflight", "tests", "test_record", "activity", "fetch", "message", "tower", "dance", "goalie", "clip",
         "ai_mission", "motors", "motors_run", "express")
CLIP_NAME = re.compile(r"^clip-[0-9]{10,16}\.wav$")   # the supervisor's /api/clip names
AI_VERBS = ("ai_lease", "move", "find", "pick", "task", "look", "tf", "frames", "ai_mission")   # the AI client's own
AI_MISSIONS = ("fetch", "message", "tower", "dance", "goalie")    # what ai_mission starts (console_fun.py), as a task
# what a person's command (the page, the voice) takes the robot back from the AI with
HUMAN_VERBS = ("stop", "mission", "goto", "cancel", "explore", "patrol", "thrower", "fire", "follow", "program_run",
               "calib_start", "tidy", "arm", "slam_new", "slam_load", "fetch", "message", "tower", "dance", "goalie",
               "motors", "motors_run")
HUMAN_HOLD = 10.0                  # s after a person takes over that the AI may not start motion
# the firmware's profile limits, fw_params_arm.h ADULT_LIMITS / KID_LIMITS: m/s, yaw rad/s, yaw rad/s^2
FW_LIMITS = {"adult": (2.0, 4.0, 15.0), "kid": (0.8, 2.0, 7.5)}
AI_TTL = (1.0, 10.0)               # s: the AI lease's time to live, its range (omni_mcp renews a 3 s lease every 1 s)
AI_TIMEOUT = {"goto": (120.0, 300.0), "pick": (180.0, 180.0), "arm_pose": (30.0, 60.0), "find": (15.0, 15.0),
              "mission": (600.0, 1800.0)}
AI_MOVE_MAX = 300.0                # s: the longest move (its default deadline: 1.5 x the estimate + 10 s)
FIND_SHAPES = ("square", "round", "long", "any")
FIND_STALE = 30.0                  # s: a find older than this can't be picked from (find again)
FAULTS_STOP_AI = ("ESTOP", "FAULT_STOP", "LOW_BATT", "LINK_LOST", "MOTOR_ERR", "MOTOR_SILENT")
NAME = re.compile(r"^[^\x00-\x1f]{1,40}$")
SAY_MAX = 200                      # characters the Say box sends at once
# the Say box's expressions: what the face shows while it says the line (omni_face moments: mood, antic)
EXPRESSIONS = {"auto": (None, None), "happy": ("happy", "smile"), "excited": ("surprised", "wheee"),
               "sad": ("sad", None), "worried": ("worried", None), "sleepy": ("sleepy", "yawn"),
               "love": ("happy", "love"), "silly": ("happy", "giggle"), "wink": (None, "wink"),
               "thinking": ("focused", "think")}
# the feelings the robot can be asked to feel (the say verb's "feel"; omni_face affect.EMOTIONS, kept equal by a test)
FEELINGS = ("neutral", "content", "happy", "excited", "proud", "affection", "relieved", "curious", "surprised",
            "startled", "focused", "effort", "worried", "frustrated", "sad", "tired", "bored")
# the antics the console can ask for (the Face tab's tricks; omni_face moments.ANTICS)
TRICKS = ("sing", "hum", "whistle", "dance", "giggle", "smile", "wink", "love", "yawn", "nap", "sneeze", "hiccup",
          "babble", "look", "dizzy", "sigh", "quip")


# -- features: what the robot offers, for the page's workspaces (the page draws what the robot lists) ----------------
# The page groups features into workspaces: home (the modes: driving by hand and every task), map, payload (the fitted
# payload's: arm or catch), see, test and robot (the menu). needs: caps that must be up (summary()["caps"]: nav, arm,
# catch, collision, sim); a feature that isn't available (a cap down, or a failing preflight check: checks.unavailable,
# the status's "avail") is shown greyed with the reason, or hidden if optional (the parked catch robot, the sim's
# tools). card: a Home card's text (a task anyone can start); start: the verb its card's button sends (None: it opens
# its page instead; open: it sends it and opens the page). moves: it drives the base or the arm by itself (the
# preflight's motion failures refuse it). base: it drives the base, by hand too (missing drive motors refuse it;
# base and moves: the base drives itself, which a silent IMU refuses).
# A feature the page has no page for gets a generic one: its card text, its settings (Setting.tabs) and its start.
# server: it works without the robot's software (the page server's own tools: the supervisor, camera_snap); every
# other page offers to start the software until it answers. The supervisor serves this list too (GET /api/features),
# so the page shows every page before the robot's software runs.
WORKSPACES = (("home", "Home"), ("map", "Map"), ("payload", "Payload"), ("see", "See"), ("test", "Test"),
              ("robot", "Robot"))


def _feature(id_, label, group, needs=(), optional=False, mission=None, card="", start=None, moves=False, hint="",
             server=False, base=False):
    return dict(id=id_, label=label, group=group, needs=list(needs), optional=optional, mission=mission, card=card,
                start=start, moves=moves, hint=hint, server=server, base=base)


FEATURES = (
    _feature("home", "Overview", "home", server=True),
    _feature("manual", "Drive", "home", mission="MANUAL", base=True,
             card="Drive it yourself: gamepad, keyboard or the pads on the screen.",
             start=dict(verb="mission", args=dict(name="MANUAL"), open=True)),
    _feature("tidy", "Tidy", "home", ("arm",), mission="TIDY", moves=True, base=True,
             card="Picks up small things from the floor and drops them in its bin.", start=dict(verb="tidy")),
    _feature("follow", "Follow", "home", mission="FOLLOW", moves=True, base=True,
             card="Follows the nearest person, or plays tag at kid speed.",
             start=dict(verb="follow", args=dict(mode="follow"))),
    _feature("fetch", "Fetch", "home", ("arm",), mission="FETCH", moves=True, base=True,
             card="\"Bring me the red block\": finds it, picks it up and brings it to you."),
    _feature("tower", "Block tower", "home", ("arm",), mission="TOWER", moves=True, base=True,
             card="Stacks the blocks it finds on the floor into a tower.", start=dict(verb="tower")),
    _feature("message", "Messenger", "home", ("nav",), mission="MESSAGE", moves=True, base=True,
             card="Goes room to room until it finds someone, then tells them your message."),
    _feature("dance", "Dance party", "home", mission="DANCE", moves=True, base=True,
             card="Slides, spins and shimmies on the beat, and sings along.", start=dict(verb="dance")),
    _feature("goalie", "Goalie", "home", mission="GOALIE", moves=True, base=True,
             card="Roll a ball at it: it slides sideways to block, and keeps score.", start=dict(verb="goalie")),
    _feature("program", "Program", "home", mission="PROGRAM", moves=True, base=True,
             card="Line up moves (forward, turn, spin, wait, go to a place) and press Run."),
    _feature("call", "Call", "home", card="Telepresence: its camera big, drive it and talk through the robot.",
             hint="Telepresence: the camera big, drive and talk through the robot"),
    _feature("goto", "Go To", "map", ("nav",), mission="GOTO", moves=True, base=True,
             card="Click on the map, or pick a saved place."),
    _feature("patrol", "Patrol", "map", ("nav",), mission="PATROL", moves=True, base=True,
             card="Drives a saved route, once or in a loop."),
    _feature("explore", "Explore", "map", ("nav",), mission="EXPLORE", moves=True, base=True,
             card="Maps the house by itself, room by room.", start=dict(verb="explore", args=dict(new_map=False))),
    _feature("slam", "SLAM", "map", hint="New map, save, load, where the robot is"),
    _feature("mapedit", "Map edit", "map", hint="Places, routes, rooms, no-go zones, edges"),
    _feature("arm", "Arm", "payload", ("arm",), hint="Joints, poses, gripper, wrist camera"),
    _feature("catch", "Catch", "payload", ("catch",), optional=True, mission="CATCH", moves=True, base=True,
             card="Throw the ball to the robot: it catches it and throws it back (parked catch robot).",
             start=dict(verb="mission", args=dict(name="CATCH"))),
    _feature("thrower", "Thrower", "payload", ("catch",), optional=True, mission="THROWER", moves=True,
             hint="Flywheels, tilt and one ball, with the base parked"),
    _feature("cameras", "Cameras", "see", server=True),
    _feature("vision", "Vision", "see"),
    _feature("preflight", "Preflight", "test", hint="Automatic checks, nothing moves"),
    _feature("bringup", "Bring-up", "test", hint="Guided checks on the real robot (design.md 8)"),
    _feature("calibrate", "Calibrate", "test", mission="CALIBRATE", moves=True, base=True),
    _feature("base", "Base", "test", hint="The MCU's 200 Hz samples"),
    # a bench test a person runs and judges (wheels up): not "moves", so only what makes it impossible refuses it
    # (checks.BENCH: no MCU, other firmware, the e-stop), not a fault stop, a low pack, the IMU or a missing motor
    _feature("motors", "Motors", "test", mission="MOTORS",
             hint="Drive each wheel motor by itself, with the wheels off the ground"),
    _feature("lidar", "Lidar", "test", hint="The lidar from its serial port: model, health, scan modes, live turns",
             server=True),
    _feature("imu", "IMU", "test", hint="The BNO085 as the robot sees it (/imu): a box that turns with it, live charts"),
    _feature("faults", "Faults", "test", ("sim",), optional=True, hint="Break the sim robot on purpose"),
    _feature("face", "Face", "robot"),
    _feature("health", "Health", "robot"),
    _feature("logbook", "Logbook", "robot", server=True),
    _feature("rc", "RC", "robot"),
    _feature("settings", "Settings", "robot", hint="Every console setting and the robot nodes' ROS parameters"),
    _feature("system", "System", "robot", server=True),
    _feature("version", "Version", "robot", server=True),
    _feature("ros", "ROS", "robot", server=True),
)
FEATURE = {f["id"]: f for f in FEATURES}
# the verbs that start a feature (the preflight's gate): verb -> feature; "mission" by its name
VERB_FEATURE = {"tidy": "tidy", "pick": "tidy", "goto": "goto", "explore": "explore", "patrol": "patrol",
                "follow": "follow", "program_run": "program", "move": "program", "calib_start": "calibrate",
                "thrower": "thrower", "fire": "thrower", "fetch": "fetch", "message": "message", "tower": "tower",
                "dance": "dance", "goalie": "goalie", "motors": "motors", "motors_run": "motors"}
# MANUAL is refused only by what stops driving by hand (missing drive motors, no MCU, the e-stop): a low pack or a
# fault stop leaves it, to drive to the charger (checks.FAULTS_BASE)
MISSION_FEATURE = {"CATCH": "catch", "THROWER": "thrower", "MOTORS": "motors", "MANUAL": "manual"}


@dataclass(frozen=True)
class Setting:
    key: str
    tabs: tuple
    label: str
    default: float | bool | str
    unit: str = ""
    lo: float | None = None
    hi: float | None = None
    step: float | None = None
    hint: str = ""
    options: tuple = ()              # a choice setting: its values

    def coerce(self, v):
        if self.options:
            v = str(v)
            if v not in self.options:
                raise ValueError(f"{self.label}: one of {', '.join(self.options)}")
            return v
        if isinstance(self.default, bool):
            if isinstance(v, str):
                v = v.strip().lower() in ("1", "true", "yes", "on")
            return bool(v)
        v = float(v)
        if not math.isfinite(v) or (self.lo is not None and v < self.lo) or (self.hi is not None and v > self.hi):
            raise ValueError(f"{self.label}: {self.lo:g} to {self.hi:g} {self.unit}".rstrip())
        return v

    def describe(self, value) -> dict:
        kind = "choice" if self.options else "bool" if isinstance(self.default, bool) else "float"
        d = dict(key=self.key, tabs=list(self.tabs), label=self.label, kind=kind, default=self.default, value=value,
                 unit=self.unit, hint=self.hint)
        if kind == "float":
            d.update(min=self.lo, max=self.hi, step=self.step)
        if kind == "choice":
            d.update(options=list(self.options))
        return d


SETTINGS = (
    Setting("manual.max_speed", ("manual",), "Top speed", 1.0, "m/s", 0.1, 2.0, 0.1,
            "Speed at full stick. The firmware caps it anyway: 2 m/s adult, 0.8 m/s kid."),
    Setting("manual.boost_speed", ("manual",), "Boost speed", 2.0, "m/s", 0.5, 2.0, 0.1,
            "Speed at full stick while boost is held (gamepad right trigger, the flight stick's trigger, Shift)."),
    Setting("manual.max_turn", ("manual",), "Turn rate", 120.0, "deg/s", 20.0, 200.0, 10.0,
            "Turn speed at full twist (right stick, stick twist, Q / E)."),
    Setting("manual.expo", ("manual",), "Stick softness", 0.3, "", 0.0, 1.0, 0.05,
            "0 = linear. Higher makes small stick moves gentler for fine positioning; full stick is still full speed."),
    Setting("manual.obstacle_stop", ("manual",), "Obstacle slow-down", True,
            hint="The lidar guard, for all driving (this stick, the RC, the flight stick, navigation): full speed, then "
                 "it brakes to stop ~10 cm short of what the lidar sees and lets you creep the rest; never limits "
                 "driving away. Needs only the lidar. Off: nothing stops the robot. Hold it off for a moment with the "
                 "RC's channel 6 or the flight stick's button 3."),
    Setting("motors.max_speed", ("motors",), "Top wheel speed", WHEEL_TEST_MAX, "rad/s", 0.5, WHEEL_TEST_MAX, 0.5,
            "Wheel speed at full stick (1 rad/s is about 10 rpm). The top is the firmware's cap, 0.95 of the motor's "
            "own no-load speed (the arm robot's AK40-10: 37.2 rad/s, 355 rpm; it tops out at ~38.5 on 24 V); lower "
            "it for gentler tests."),
    Setting("motors.expo", ("motors",), "Stick softness", 0.3, "", 0.0, 1.0, 0.05,
            "0 = linear. Higher makes small stick moves gentler; full stick is still the top speed."),
    Setting("motors.ramp", ("motors",), "Ramp", WHEEL_TEST_ACCEL, "rad/s²", 5.0, WHEEL_TEST_ACCEL_MAX, 5.0,
            "How fast the firmware changes each wheel's speed, for the stick and the profiles. 20 rad/s² is gentle "
            "(rest to full speed in ~2 s); 2000 is a step for a bare wheel (its motor's current limit sets the real "
            "rate). Letting go, STOP or leaving the test always ramps to rest at 20."),
    Setting("goto.speed", ("goto",), "Speed", 2.0, "m/s", 0.1, 2.0, 0.05,
            "Nav2's top speed between goals: 2 m/s is the firmware's limit (the kid profile caps it at 0.8). It slows "
            "down for the goal by itself."),
    Setting("goto.turn", ("goto",), "Turn rate", 90.0, "deg/s", 20.0, 105.0, 5.0,
            "Nav2's top turn rate, and how fast it turns to face the way it's going."),
    Setting("goto.facing", ("goto", "patrol", "explore"), "Driving", "forward",
            options=("forward", "agnostic", "spinning"),
            hint="forward: the face turns toward where it's going (it hands the turn back to Nav2 near the goal, to "
                 "end at the goal's heading). agnostic: Nav2 picks the heading (it often drives sideways). spinning: "
                 "it spins all the way there (Explore drives forward instead: spinning maps worse)."),
    Setting("goto.spin", ("goto", "patrol"), "Spin rate", 120.0, "deg/s", 45.0, 220.0, 5.0,
            "How fast it spins when driving is 'spinning' (the kid profile caps turning at 115 deg/s)."),
    Setting("goto.stuck_time", ("goto", "patrol", "explore"), "Stuck after", 40.0, "s", 10.0, 180.0, 5.0,
            "On the way to a goal, this long without getting 0.5 m from where it was means it's stuck: it gives up "
            "the goal, says so and drives home (the place 'home', else where the task started). Stuck on the way home "
            "too: it stops and waits for help."),
    Setting("goto.goal_time", ("goto", "patrol", "explore"), "Time per goal", 60.0, "s", 20.0, 600.0, 10.0,
            "A goal not reached after this long plus three times its drive time (along Nav2's route, else the "
            "straight line) is stuck too (the same as above)."),
    Setting("explore.radius", ("explore",), "How far", 15.0, "m", 2.0, 30.0, 0.5,
            "Openings farther than this from where exploring started are left alone."),   # (10 until Nov 25: the
            # home's kitchen and back hall are 11.3 m from the family room and were never explored, hm12)
    Setting("explore.max_time", ("explore",), "Time limit", 30.0, "min", 1.0, 60.0, 1.0,   # (10 until Nov 25: the
            # home's whole reachable floor took ~12 min of robot time, hm16)
            "Stops exploring after this long, mapped or not."),
    Setting("explore.min_frontier", ("explore",), "Smallest opening", 0.5, "m", 0.2, 2.0, 0.1,
            "Gaps into unknown space narrower than this are ignored (chair legs, clutter)."),
    Setting("explore.speed", ("explore",), "Speed", 0.5, "m/s", 0.1, 0.8, 0.05,
            "Slower maps better: the lidar has more scans of each wall."),
    Setting("explore.return_home", ("explore",), "Come back when done", True,
            hint="Drive back to where exploring started when nothing is left (or the time is up)."),
    Setting("patrol.mode", ("patrol",), "Order", "loop", options=("loop", "once", "back and forth"),
            hint="loop: last point back to the first. once: stop at the last point. back and forth: reverse at each "
                 "end."),
    Setting("patrol.laps", ("patrol",), "Laps", 0.0, "", 0.0, 100.0, 1.0, "How many times round; 0 = until stopped."),
    Setting("patrol.dwell", ("patrol",), "Wait at each point", 3.0, "s", 0.0, 300.0, 1.0,
            "Time the robot stays at every point before driving on."),
    Setting("patrol.speed", ("patrol",), "Speed", 0.8, "m/s", 0.1, 2.0, 0.05, "Nav2's top speed between points."),
    Setting("profile.kid", ("manual", "catch", "thrower"), "Kid profile", False,
            hint="The firmware's kid limits in every mode: 0.8 m/s, gentle accelerations, lobbed returns, lower "
                 "flywheel speed."),
    Setting("catch.returns", ("catch",), "Throw back", True,
            hint="Throw the ball back to the thrower after each catch. Off: the robot keeps the ball."),
    Setting("catch.return_to", ("catch",), "Throw it to", "the thrower", options=("the thrower", "the other player"),
            hint="The other player: the pass game. Two people stand apart (2 m or more) and take turns; the robot "
                 "throws each ball it catches to the one who didn't throw it, and turns to face them."),
    Setting("follow.distance", ("follow",), "Distance", 1.0, "m", 0.6, 2.5, 0.1,
            "How far behind the person the robot keeps."),
    Setting("follow.speed", ("follow",), "Top speed", 0.6, "m/s", 0.2, 1.2, 0.05,
            "Follow's top speed. Tag always chases at kid speed or less."),
    Setting("tag.speed", ("follow",), "Tag speed", 0.5, "m/s", 0.2, 0.8, 0.05,
            "How fast the robot chases in Tag (the kid profile caps everything at 0.8 m/s anyway)."),
    Setting("program.speed", ("program",), "Speed", 0.3, "m/s", 0.1, 0.6, 0.05,
            "How fast programs drive their moves."),
    Setting("program.turn", ("program",), "Turn rate", 60.0, "deg/s", 20.0, 120.0, 5.0,
            "How fast programs turn and spin."),
    Setting("calib.distance", ("calibrate",), "Straight-line distance", 2.0, "m", 1.0, 4.0, 0.5,
            "How far the odometry check drives (more is more precise; leave room in front of the robot)."),
    Setting("calib.turns", ("calibrate",), "Spin turns", 3.0, "", 1.0, 5.0, 1.0,
            "How many turns the heading check spins."),
    Setting("tidy.max_objects", ("tidy",), "How many", 20.0, "", 1.0, 50.0, 1.0,
            "Stops after picking up this many things (the bin holds about a dozen small toys)."),
    Setting("face.antics", ("face",), "Antics", "normal", options=("off", "quiet", "normal", "lively"),
            hint="How often the robot looks around, hums, sings, yawns, naps or says something dry when nothing "
                 "else is going on (quiet: every few minutes, normal: every two to five minutes, lively: often). "
                 "Never during a throw or a fault."),
    Setting("face.humor", ("face",), "Humor", 75.0, "%", 0.0, 100.0, 5.0,
            hint="How funny it tries to be: 0 is plain and businesslike, 100 is a wisecrack whenever it can. Also "
                 "by voice: \"Omni, humor sixty percent\". Faults are always said plainly."),
    Setting("face.preset", ("face",), "Voice", "cbt_vocoder",
            options=("cbt_vocoder", "cbt_voc1", "cbt_voc2", "cbt_voc3", "cbt_voc4", "cbt_voc5", "cbt_voc6", "cbt_voc7",
                     "cbt_voc8", "cbt_robot", "cbt_emotion", "cbt_metal", "cbt_dry", "dry_synth", "aiden_vocoder",
                     "aiden_synth", "ryan_vocoder", "ryan_metal", "ryan_digital", "ryan_synth_strong", "ryan_synth",
                     "ryan_synth_light", "michael_synth", "qwen_natural", "qwen_dry", "kokoro_dry", "cb_bright",
                     "pocket_alba", "qwen_bright", "qwen_small", "kokoro_bright", "droid", "deadpan", "bright",
                     "cute"),
            hint="Who it sounds like (Fernando's pick, Nov 23: cbt_vocoder, Chatterbox Turbo's designed dry voice with "
                 "a reference clip per feeling through a soft vocoder; cbt_voc1-8 the same voice, the same emotion, "
                 "steps more robotic, metallic and synthesized). The others, kept to choose from: cbt_robot / "
                 "cbt_emotion / cbt_metal the same voice as android, emotion only or light machine; cbt_dry the round-2 original; then his "
                 "other favourites (dry_synth, aiden_vocoder, ryan_vocoder / metal / digital) and every voice "
                 "auditioned. All are made on the main PC with each line's feeling (omni_face voice_server); "
                 "deadpan / bright / cute are the robot's own older Kokoro voices. If the PC's voice service doesn't "
                 "answer, it falls back to its own voice."),
    Setting("face.voice", ("face",), "Voice commands", "on", options=("off", "chat only", "on"),
            hint="Talk to the robot: its name first (\"Omni, follow me\"). Chat only: it answers and stops but "
                 "nothing else drives it. On: also follow, play catch, go to a saved place, explore. Off: the "
                 "microphones stay off."),
    # its feelings (omni_face affect.py, expression.py; Fernando, Nov 21: each channel tunable 0-10)
    Setting("emotion.on", ("face",), "Emotion", True,
            hint="The robot feels what happens to it (a pick that works, a grab that slips, praise, being pushed, a "
                 "flat battery) and shows it on its face, in its voice and sounds and, when they are free, with its "
                 "wheels and arm. Off: its moods and lines as before, nothing moves for a feeling."),
    Setting("emotion.speech", ("face",), "Talk", 5.0, "", 0.0, 10.0, 1.0,
            "How often it speaks up on its own: 0 only faults and answers to what you ask, 5 as designed, 10 chatty."),
    Setting("emotion.sounds", ("face",), "Chirps and sounds", 5.0, "", 0.0, 10.0, 1.0,
            "How often (and how loud) it chirps, says \"whoa!\" or plays its little sounds: 0 silent (fault alerts "
            "still sound), 5 as designed, 10 every chance it gets."),
    Setting("emotion.face", ("face",), "Face motion", 6.0, "", 0.0, 10.0, 1.0,
            "How much its eyes and mouth move and how far its expressions go: 0 calm (it still blinks and shows its "
            "mood faintly), 5 as designed, 10 cartoon."),
    Setting("emotion.base", ("face",), "Wheel gestures", 3.0, "", 0.0, 10.0, 1.0,
            "How often and how big its wheel gestures are (a happy wiggle when you praise it, a recoil when "
            "startled): only while it rests in Drive with nobody at the stick, never during a task. 0 never; at 10 "
            "the largest strays 6 cm and turns 14 deg, always back where it started."),
    Setting("emotion.arm", ("face",), "Arm gestures", 3.0, "", 0.0, 10.0, 1.0,
            "How often and how big its arm gestures are (a nod, a head tilt, a flinch): only while the arm is free "
            "and holds nothing, never during a pick. Every gesture is checked like any arm move and ends where it "
            "began. 0 never."),
    Setting("emotion.nav", ("face", "goto", "patrol", "explore", "follow"), "Mood sets driving pace", True,
            hint="On: a tired or sad robot drives up to 15 % slower on its own (Go To, Patrol, Explore, Follow); it "
                 "never drives faster than set. Off: navigation exactly as set, always."),
    Setting("emotion.sound_style", ("face",), "Reaction sounds", "mix", options=("mix", "voice", "chirps"),
            hint="What its reactions sound like: voice (\"whoa!\", \"hmm?\", \"ta-da!\" in its own voice), chirps "
                 "(beeps with the feeling's intonation, its robot language), or a mix."),
    Setting("tidy.radius", ("tidy",), "How far", 2.0, "m", 0.5, 5.0, 0.25,
            "Only things within this distance of where the tidy started. The tidy drives itself (not Nav2), slowly, "
            "and only through floor its camera has seen free; it checks the whole circle, then comes back."),
    Setting("fetch.to", ("fetch",), "Bring it to", "me", options=("me", "here"),
            hint="me: the nearest person it sees (it looks round for you if nobody is in view). here: back where the "
                 "fetch started."),
    Setting("tower.max_blocks", ("tower",), "How high", 4.0, "blocks", 2.0, 5.0, 1.0,
            "The tallest tower it builds (the arm reaches about 20 cm over the floor fingers-down)."),
    Setting("dance.routine", ("dance",), "Routine", "groove", options=("groove", "party", "robot"),
            hint="groove: slides, shimmies and a spin. party: circles and spins. robot: stiff square steps and poses."),
    Setting("dance.bpm", ("dance",), "Tempo", 110.0, "bpm", 60.0, 160.0, 5.0, "Beats per minute: match the music."),
    Setting("dance.speed", ("dance",), "Moves", 0.4, "m/s", 0.1, 0.6, 0.05,
            "How big and fast the moves are (it never strays more than 0.6 m from where it started)."),
    Setting("dance.seconds", ("dance",), "Length", 60.0, "s", 15.0, 300.0, 15.0, "How long the dance lasts."),
    Setting("goalie.width", ("goalie",), "Goal width", 1.2, "m", 0.6, 2.0, 0.1,
            "The goal it keeps, centred where it stood when the game started (mark the posts)."),
    Setting("goalie.speed", ("goalie",), "Dive speed", 1.2, "m/s", 0.3, 2.0, 0.1,
            "How fast it slides to block (the firmware's profile caps it: kid 0.8 m/s)."),
    Setting("ai.max_speed", ("ai",), "AI top speed", 0.5, "m/s", 0.1, 1.0, 0.05,
            "The fastest an AI client (Claude or Codex through omni_mcp) may drive the robot in its moves. The "
            "firmware's profile still caps it (kid 0.8 m/s)."),
    Setting("ai.max_turn", ("ai",), "AI turn rate", 180.0, "deg/s", 20.0, 229.0, 1.0,
            "The fastest an AI client may turn the robot ('as fast as you can'). The firmware caps turning at "
            "229 deg/s (adult) or 115 deg/s (kid)."),
)
SETTING = {s.key: s for s in SETTINGS}


@dataclass
class Reply:
    ok: bool
    message: str = ""
    data: dict | list | None = None
    effects: tuple = ()


def shape(u: float, expo: float) -> float:
    """Stick curve: linear at expo 0, cubic at 1; keeps sign and the end points."""
    return (1.0 - expo) * u + expo * u ** 3


def _points(raw) -> list:
    pts = [[float(p[0]), float(p[1]), float(p[2]) if len(p) > 2 else 0.0] for p in raw]
    if not all(math.isfinite(v) for p in pts for v in p):
        raise ValueError("not a number")
    return pts


def _program_check(blocks) -> list:
    from omni_console.program import check, flatten
    out = check(blocks)
    flatten(out)                         # too long once the repeats are unrolled: ValueError now, not at Run
    return out


_LAUNCH = None


def launch_limits() -> tuple:
    """(launch angle, the flattest launch fired), deg: the generated robot_v11.json (launch elevation = the launch angle
    minus the tilt). Below 24 deg the ball hits the face between the cameras (design.md 12.2); the firmware refuses
    those shots too (FIRE_TILT_MAX)."""
    global _LAUNCH
    if _LAUNCH is None:
        from omni_catch.robot_config import RobotConfig
        mo = RobotConfig().raw["module"]
        _LAUNCH = (float(mo["launch_angle_deg"]), float(mo.get("fire_min_elevation_deg", 25.0)))
    return _LAUNCH


def _name(a: dict, what: str) -> str:
    n = str(a.get("name", "")).strip()
    if not NAME.match(n):
        raise ValueError(f"{what} needs a name (up to 40 characters)")
    return n

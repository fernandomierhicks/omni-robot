// The drive guard (Nov 23): the lidar's last word on every twist the base is asked for, below SLAM and Nav2.
// mode_manager runs it on whoever drives (RC, the flight stick, the console, Nav2, the AI); ROS-free here.
//
// Points: every return of the last scan, plus each run of empty bins narrower than max_gap filled with virtual points
// at the nearer flank's range x cos(half the run): the blind wedges behind the robot's own posts (the lidar can't
// see a wall squarely behind it), a dropped return, glass. Wider runs are open space. The lidar sits inside the
// robot's outline, so every ray to the robot's way stays inside the way: an empty bin is the only way the way can
// hide something, and these fills cover it (a thing narrower than the wedge, in the wedge, stays unseen).
//
// The robot is a circle (radius). For the asked translation it finds the free distance: how far the circle travels
// before it touches a point (points inside the rim less self_margin are the robot itself; points behind the way never
// count, so moving away is never limited). A turn while translating sweeps the way's heading over the stopping time;
// the fan's shortest way counts. Then the speed is capped to what can still stop `buffer` short after `latency` at
// `decel`:
//     v = -a tau + sqrt((a tau)^2 + 2 a (free - buffer)),
// and never below touch_speed * free / buffer, so the robot creeps the last centimetres and can just touch.
// The turn rate is kept. Slide (Nov 24, the default): over the cap, the part of the translation along what it would
// hit is kept and only the part into it is cut, as little as the law allows: pushed diagonally at a wall, the robot
// glides along it; square on, or past a post's edge, it slows as before; a corner takes a second pass. The normal is
// the contact's (centre to point where the circle stops, averaged over the points within 1 cm of the first: a wall's
// own normal). Whatever it sends obeys the same law along its own way (with its own turn fan), and anything still over
// is capped along its way. slide false: the whole translation is scaled, the direction kept (before Nov 24).
#pragma once

#include <cstdint>
#include <vector>

namespace omni {

struct GuardConfig {
    double radius = 0.217;        // m: the robot's outline as a circle round base_link
    double buffer = 0.10;         // m: where a full-speed stop ends, short of the obstacle
    double decel = 2.5;           // m/s^2: the braking the speed law assumes (the firmware brakes harder)
    double latency = 0.10;        // s: from a lower limit to the wheels braking (tick, link, jerk ramp, motors)
    double touch_speed = 0.05;    // m/s: the creep at the buffer, falling to 0 at contact
    double self_margin = 0.01;    // m: points this far inside the rim are the robot's own structure
    double max_fan_deg = 90.0;    // deg: the most a turn sweeps the way's heading
    bool slide = true;            // cut only the part into what it would hit (false: scale it all)
};

struct GuardPoint {
    double x, y;                  // m
};

struct GuardResult {
    double vx = 0.0, vy = 0.0, wz = 0.0;   // the twist to send
    double asked = 0.0;                    // m/s: the asked translation's speed
    double limit = 1e9;                    // m/s: the cap for the asked way (1e9: none; a slide may beat it)
    double free = 1e9;                     // m: the asked way's free distance (1e9: past the look-ahead)
    bool limited = false;                  // the translation was cut (slowed or slid)
};

// A scan's points in its own frame (angles from angle_min by angle_inc, ranges outside [range_min, range_max] or not
// finite are empty), with each run of empty bins narrower than max_gap (rad) filled as above. full_turn: the bins
// wrap round (a 360 deg scan), so a run may cross the array's ends.
std::vector<GuardPoint> guard_scan_points(const std::vector<float>& ranges, double angle_min, double angle_inc,
                                          double range_min, double range_max, double max_gap, bool full_turn,
                                          int* n_filled = nullptr);

class DriveGuard {
public:
    GuardConfig cfg;

    void set_points(std::vector<GuardPoint> p) { pts_ = std::move(p); }   // base_link, now
    const std::vector<GuardPoint>& points() const { return pts_; }

    // m the circle can travel along the unit direction (ux, uy) before it touches a point (1e9: never); nx, ny: the
    // contact's unit normal, toward the obstacle (the direction itself when nothing is met)
    double free_distance(double ux, double uy, double* nx = nullptr, double* ny = nullptr) const;
    // the speed law: the most speed that still stops buffer short of `free` m
    double allowed_speed(double free) const;
    // the translation (vx, vy) with turn wz obeys the law along its way
    bool safe(double vx, double vy, double wz) const;
    // the asked body twist, capped
    GuardResult limit(double vx, double vy, double wz) const;

private:
    struct Way {
        double free = 1e9, nx = 0.0, ny = 0.0;   // the turn fan's shortest way and its contact normal
    };
    Way way(double vx, double vy, double wz) const;
    std::vector<GuardPoint> pts_;
};

}  // namespace omni

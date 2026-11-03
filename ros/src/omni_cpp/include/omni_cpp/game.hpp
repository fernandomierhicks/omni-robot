// The play-catch behavior (design.md 8, 9, 22, 30): C++ port of omni_catch/game.py's CatchBehavior, the same states
// (wait, flight, settle, aim, launch, return), decisions and timing; its notes say why each rule is there. `last`
// (the last throw's record for /catch/status) is built as the Python's dict, key for key. test/test_cores.py drives
// both through the same throws tick by tick and compares the commands, the states and the records.
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "omni_cpp/flight.hpp"
#include "omni_cpp/intercept.hpp"
#include "omni_cpp/json_value.hpp"
#include "omni_cpp/mode_mux.hpp"
#include "omni_cpp/robot_config.hpp"
#include "omni_cpp/tracker.hpp"

namespace omni {

struct GameParams {
    std::string catch_mode = "moving";
    double brake_with_ball = 3.0, plan_margin = 0.9, safety_margin = 0.20;
    bool gate = true;
    double person_radius = 0.30, coast_moving = 0.05, coast_after_catch = 0.2, moving_commit_sigma = 0.10,
           replan_period = 0.05;
    bool align_tilt = true;
    double tilt_gain = 0.6, max_catch_tilt_deg = 35.0, tilt_freeze = 0.15, dump_delay = 0.03;
    bool return_throws = true;
    double return_height = 1.2, return_elevation_deg = 50.0, min_return_elevation_deg = 30.0;
    std::optional<double> ceiling;
    double ceiling_margin = 0.25, fire_settle = 0.0, release_height = 1.0;
    double home[2] = {0.0, 0.0};
    double thrower[2] = {6.8, 0.0};
    double arena_center[2] = {3.5, 0.0};
    double arena_half[2] = {6.0, 3.0};
    bool learn_thrower = true, kid = false;
    std::string return_to = "thrower";      // or "other": the pass game
    double pass_gap = 1.5;
};

struct RobotView {
    double t = 0;
    double pose[3] = {0, 0, 0};             // x, y, yaw (odom)
    double vel_body[3] = {0, 0, 0};
    double v_cmd[2] = {0, 0};
    double tilt = 0, tilt_cmd = 0;
    double fly_rpm[2] = {0, 0};
    double fly_target_rpm = 0;
    bool fly_ready = false, ball_held = false;
    int fire_seq_done = 0;
    std::string mode = "DISARMED";
};

class CatchBehavior {
public:
    using Log = std::function<void(const std::string&)>;
    CatchBehavior(const RobotConfig& robot, const GameParams& params, Log log = nullptr);
    CatchCommand step(double t, const RobotView& rv, const TrackView* track);

    RobotConfig cfg;
    GameParams p;
    BallFlight flight;
    Log log;
    std::string state = "wait";
    double t_state = 0;
    double target[3];
    double home[3];
    double hand[3];
    double tilt_target = 0, fly_rpm = 0, range_bias = 0;
    uint16_t fire_seq = 0;
    int tosses = 0, catches = 0;
    Json last = Json::object();

private:
    void new_toss_state();
    void set(const std::string& s, double t);
    static double face(double x, double y, const double* toward);
    SafetyZone zone() const;
    void learn_hand(const V3& est_p, const V3& est_v, const double* pose);
    int which_hand(double x, double y);
    struct Plan {
        double tgt[3];
        double tilt;
        Crossing hit;
    };
    std::optional<Plan> catch_plan(const V3& ball_p, const V3& ball_v, const double v_robot[2]) const;
    V3 exit_world(const double* pose, double tilt) const;
    void plan_return(const double* pose, bool final, double tilt_meas);
    bool incoming(double t, const double* pose, const TrackView* track, V3& bp, V3& bv) const;
    void settle_done(double t, const RobotView& rv, const double* pose);
    void say(const char* fmt, ...);

    std::optional<double> t_beam, t_ready;
    std::vector<std::array<double, 2>> hands;
    int thrower_i = 0;
    // this toss
    double t_flight = 0, t_plan = 0;
    std::optional<V3> predicted;
    bool frozen = false;
    std::optional<double> t_c;
    double v_c[2] = {0, 0};
    double v_end = 0;
    bool moving_plan = false, gated = false;
    double land_sigma = 0, t_track_seen = 0;
    bool beam_seen = false;   // the catch beam saw the ball during this toss's settle
    Json throw_ = Json::object();
    double t_path = -1.0;
    double v_needed = 0, elev = 0;
    bool final_plan = false;
};

}  // namespace omni

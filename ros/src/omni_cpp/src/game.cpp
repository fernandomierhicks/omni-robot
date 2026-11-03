#include "omni_cpp/game.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>

namespace omni {

namespace {

constexpr double kNan = std::numeric_limits<double>::quiet_NaN();

double wrap(double a) { return py_mod(a + kPiC, 2 * kPiC) - kPiC; }
double rad(double d) { return d * (kPiC / 180.0); }       // math.radians
double deg(double r) { return r * (180.0 / kPiC); }       // math.degrees
double clip(double x, double lo, double hi) { return std::min(std::max(x, lo), hi); }
double norm2(double x, double y) { return std::sqrt(x * x + y * y); }    // np.linalg.norm of a 2-vector

Json rounded(std::initializer_list<double> v, int nd) {
    Json a = Json::array();
    for (double x : v) a.push(Json(py_round(x, nd)));
    return a;
}

}  // namespace

CatchBehavior::CatchBehavior(const RobotConfig& robot, const GameParams& params, Log lg)
    : cfg(robot), p(params), flight(robot.ball_mass, robot.ball_radius, robot.ball_cd, robot.air_density,
                                    robot.air_viscosity),
      log(lg ? lg : [](const std::string&) {}) {
    hand[0] = p.thrower[0], hand[1] = p.thrower[1], hand[2] = p.release_height;
    home[0] = p.home[0], home[1] = p.home[1], home[2] = face(p.home[0], p.home[1], hand);
    std::copy(home, home + 3, target);
    hands.push_back({hand[0], hand[1]});
    new_toss_state();
}

void CatchBehavior::say(const char* fmt, ...) {
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(b, sizeof b, fmt, ap);
    va_end(ap);
    log(b);
}

void CatchBehavior::new_toss_state() {
    t_flight = 0.0, t_plan = 0.0;
    predicted.reset();
    frozen = false;
    t_c.reset();
    v_c[0] = v_c[1] = 0.0;
    v_end = 0.0;
    moving_plan = false, gated = false;
    land_sigma = kNan;
    t_track_seen = 0.0;
    beam_seen = false;
    throw_ = Json::object();
    t_path = -1.0;
}

void CatchBehavior::set(const std::string& s, double t) {
    state = s, t_state = t;
    t_beam.reset();
}

double CatchBehavior::face(double x, double y, const double* toward) { return std::atan2(toward[1] - y, toward[0] - x); }

SafetyZone CatchBehavior::zone() const {
    SafetyZone z;
    if (!p.gate) {
        z.half_x = z.half_y = 1e3, z.margin = 0.0;
        return z;
    }
    if (p.return_to == "other")
        for (auto& h : hands) z.people.push_back({h[0], h[1], p.person_radius});
    else
        z.people.push_back({hand[0], hand[1], p.person_radius});
    z.half_x = p.arena_half[0], z.half_y = p.arena_half[1], z.margin = p.safety_margin;
    z.robot_radius = cfg.robot_radius, z.cx = p.arena_center[0], z.cy = p.arena_center[1];
    return z;
}

void CatchBehavior::learn_hand(const V3& est_p, const V3& est_v, const double* pose) {
    if (!p.learn_thrower) return;
    auto rp = flight.release_point(est_p, est_v, p.release_height, 0.004, 4.0);
    if (!rp) return;
    double dist = norm2(rp->first[0] - pose[0], rp->first[1] - pose[1]);
    if (!(1.0 < dist && dist < 12.0)) return;
    throw_.set("release", rounded({rp->first[0], rp->first[1], rp->first[2]}, 3));
    int i = which_hand(rp->first[0], rp->first[1]);
    hands[i] = {0.5 * hands[i][0] + 0.5 * rp->first[0], 0.5 * hands[i][1] + 0.5 * rp->first[1]};
    thrower_i = i;
    hand[0] = hands[i][0], hand[1] = hands[i][1];
}

int CatchBehavior::which_hand(double x, double y) {
    if (p.return_to != "other") return 0;
    int i = 0;
    double best = 0;
    for (size_t k = 0; k < hands.size(); ++k) {
        double d = norm2(x - hands[k][0], y - hands[k][1]);
        if (k == 0 || d < best) best = d, i = (int)k;
    }
    if (best > p.pass_gap && hands.size() < 2) {
        hands.push_back({x, y});
        return (int)hands.size() - 1;
    }
    return i;
}

std::optional<CatchBehavior::Plan> CatchBehavior::catch_plan(const V3& ball_p, const V3& ball_v,
                                                             const double v_robot[2]) const {
    double lo = cfg.tilt_lo, hi = cfg.tilt_hi;
    double tilt = tilt_target;
    std::optional<Crossing> hit;
    V3 v{};
    for (int k = 0; k < 3; ++k) {
        auto rc = cfg.rim_center(tilt);
        hit = flight.crossing_state(ball_p, ball_v, rc[1]);
        if (!hit) return std::nullopt;
        v = {hit->vel[0] - v_robot[0], hit->vel[1] - v_robot[1], hit->vel[2] - 0.0};
        double ang = std::atan2(std::hypot(v[0], v[1]), -v[2]);
        double hi_c = std::min(hi, rad(p.max_catch_tilt_deg));
        tilt = p.align_tilt ? clip(p.tilt_gain * ang, std::max(lo, 0.0), hi_c) : 0.0;
    }
    const V3& xy = hit->pos;
    double yaw = std::hypot(v[0], v[1]) > 0.3 ? std::atan2(-v[1], -v[0]) : face(xy[0], xy[1], hand);
    double x_rc = cfg.rim_center(tilt)[0];
    Plan pl;
    pl.tgt[0] = xy[0] - x_rc * std::cos(yaw), pl.tgt[1] = xy[1] - x_rc * std::sin(yaw), pl.tgt[2] = yaw;
    pl.tilt = tilt;
    pl.hit = *hit;
    return pl;
}

V3 CatchBehavior::exit_world(const double* pose, double tilt) const {
    auto e = cfg.exit_point(tilt);
    return {pose[0] + e[0] * std::cos(pose[2]), pose[1] + e[0] * std::sin(pose[2]), e[1]};
}

void CatchBehavior::plan_return(const double* pose, bool final, double tilt_meas) {
    double hx = std::cos(pose[2]), hy = std::sin(pose[2]);
    V3 tgt{hand[0] - range_bias * hx, hand[1] - range_bias * hy, p.return_height};
    auto solve = [&](double el, double& tilt, double& v, double& apex) {
        tilt = cfg.tilt_for(5.0, el), v = 5.0;
        V3 exit_pos{};
        for (int k = 0; k < 3; ++k) {
            exit_pos = exit_world(pose, final ? tilt_meas : tilt);
            v = flight.solve_launch_speed(exit_pos, hx, hy, el, tgt);
            tilt = cfg.tilt_for(v, el);
        }
        V3 vel{v * (std::cos(el) * hx), v * (std::cos(el) * hy), v * std::sin(el)};
        apex = flight.apex(exit_pos, vel);
    };
    double el = rad(p.return_elevation_deg), tilt, v, apex;
    solve(el, tilt, v, apex);
    if (p.ceiling && !final) {
        double top = *p.ceiling - p.ceiling_margin;
        double el_deg = p.return_elevation_deg;
        while (apex > top && el_deg > p.min_return_elevation_deg) {
            el_deg -= 1.0;
            el = rad(el_deg);
            solve(el, tilt, v, apex);
        }
    }
    tilt = clip(tilt, cfg.tilt_lo, cfg.tilt_hi);
    if (final) {                                    // the tilt is set now: take the elevation it really gives
        tilt = tilt_target;
        for (int k = 0; k < 2; ++k) {
            el = cfg.elevation_at(v, tilt);
            v = flight.solve_launch_speed(exit_world(pose, tilt_meas), hx, hy, el, tgt);
        }
    }
    v_needed = v, elev = el;
    tilt_target = tilt;
    fly_rpm = cfg.speed_for(v, tilt) * 60 / (2 * kPiC);
}

bool CatchBehavior::incoming(double t, const double* pose, const TrackView* track, V3& bp, V3& bv) const {
    if (!track || !track->ready) return false;
    track->estimate(t, bp, bv);
    return norm(bv) > 2.0 && bv[0] * (pose[0] - bp[0]) + bv[1] * (pose[1] - bp[1]) > 0 && bp[2] > 0.3;
}

void CatchBehavior::settle_done(double t, const RobotView& rv, const double* pose) {
    bool caught = rv.ball_held || beam_seen;   // a ball the beam saw counts (and goes back), even still bouncing
    catches += caught;
    const Json& th = throw_;
    Json L = Json::object();
    L.set("toss", Json(tosses));
    L.set("caught", Json(caught));
    L.set("moving", Json(moving_plan));
    L.set("gated", Json(gated));
    L.set("land_sigma", Json(land_sigma));
    L.set("v_end", Json(v_end));
    L.set("predicted", predicted ? Json::floats({(*predicted)[0], (*predicted)[1], (*predicted)[2]}) : Json());
    L.set("speed", Json(py_round(th.has("speed") ? th["speed"].num() : kNan, 3)));
    L.set("apex", Json(py_round(th.has("apex") ? th["apex"].num() : kNan, 3)));
    L.set("release", th.get("release"));
    L.set("path", th.has("path") ? th["path"] : Json::array());
    L.set("thrower", Json(thrower_i));
    L.set("t0", th.get("t0"));
    L.set("state0", th.get("state0"));
    L.set("statef", th.get("statef"));
    L.set("tf", th.get("tf"));
    L.set("flight_time", t_c ? Json(py_round(*t_c - t_flight, 3)) : Json());
    L.set("robot", rounded({pose[0], pose[1], pose[2]}, 3));
    last = L;
    say("toss %d: %s (%d/%d)", tosses, caught ? "CAUGHT" : "missed", catches, tosses);
    if (p.return_to == "other" && hands.size() == 2) {
        hand[0] = hands[1 - thrower_i][0], hand[1] = hands[1 - thrower_i][1];   // the pass game: the other player
    }
    if (caught && p.return_throws) {
        set("aim", t);
        target[0] = pose[0], target[1] = pose[1], target[2] = face(pose[0], pose[1], hand);
        plan_return(target, false, rv.tilt);
        final_plan = false;
        t_ready.reset();
    } else {
        set("wait", t);
    }
}

CatchCommand CatchBehavior::step(double t, const RobotView& rv, const TrackView* track) {
    const double* pose = rv.pose;
    CatchCommand cmd;
    if (track) t_track_seen = t;

    V3 bp{}, bv{};
    bool inc = incoming(t, pose, track, bp, bv);
    if (state == "return" && inc) set("wait", t);                    // a throw while going home: catch it from here
    if (state == "settle" && inc && t - t_state > 0.3) settle_done(t, rv, pose);   // the next throw is coming
    if (state == "wait") {
        std::copy(home, home + 3, target);
        target[2] = face(home[0], home[1], hand);
        tilt_target = 0.0, fly_rpm = 0.0;
        if (inc) {
            new_toss_state();
            set("flight", t);
            t_flight = t;
            ++tosses;
            throw_ = Json::object();
            throw_.set("speed", Json(norm(bv)));
            throw_.set("apex", Json(flight.apex(bp, bv)));
            throw_.set("release", Json());
            throw_.set("path", Json::array());
            throw_.set("t0", Json(t));
            throw_.set("state0", rounded({bp[0], bp[1], bp[2], bv[0], bv[1], bv[2]}, 4));
            auto rp = flight.release_point(bp, bv, p.release_height);
            if (rp) {
                double d = norm2(rp->first[0] - pose[0], rp->first[1] - pose[1]);
                if (1.0 < d && d < 12.0) throw_.set("release", rounded({rp->first[0], rp->first[1], rp->first[2]}, 3));
            }
            say("toss %d: tracking, ball %.1f m/s, %.1f m out", tosses, norm(bv),
                norm2(pose[0] - bp[0], pose[1] - bp[1]));
        }
    }

    if (state == "flight") {
        if (track && t - t_path >= 0.05 && throw_["path"].size() < 80) {
            t_path = t;                                            // the throw's path, for the console's replay
            V3 pt, vt;
            track->estimate(t, pt, vt);
            Json row = Json::array();
            row.push(Json(py_round(t - throw_["t0"].num(), 3)));
            for (double c : pt) row.push(Json(py_round(c, 3)));
            Json path = throw_["path"];
            path.push(row);
            throw_.set("path", path);
        }
        if (track && track->ready && t >= t_plan && !frozen) {
            t_plan = t + p.replan_period;
            V3 est_p, est_v;
            track->estimate(t, est_p, est_v);
            auto plan = catch_plan(est_p, est_v, v_c);
            if (plan && p.catch_mode == "moving") {
                double t_go = plan->hit.t;
                auto cr = track->crossing(t, plan->hit.pos[2]);
                bool confident = cr && std::sqrt(eig2_max(cr->cov[0], cr->cov[2], cr->cov[3])) < p.moving_commit_sigma;
                double dx = plan->tgt[0] - pose[0], dy = plan->tgt[1] - pose[1];
                double nd = norm2(dx, dy);
                double ux = nd > 1e-6 ? dx / nd : 1.0, uy = nd > 1e-6 ? dy / nd : 0.0;
                double v_top = cfg.top_speed(ux, uy, pose[2], p.kid);
                double a_plan = cfg.plan_accel(p.kid);
                double p0[2] = {pose[0], pose[1]}, v0[2] = {rv.v_cmd[0], rv.v_cmd[1]},
                       pc[2] = {plan->tgt[0], plan->tgt[1]};
                auto ch = choose_intercept(p0, v0, pc, t_go, cfg.brake_margin * a_plan, p.plan_margin * v_top, zone(),
                                           p.coast_moving, std::min(p.brake_with_ball, a_plan), confident);
                v_end = ch.v_f;
                v_c[0] = ch.v_f * ch.dx, v_c[1] = ch.v_f * ch.dy;
                gated = gated || ch.gated;
                plan = catch_plan(est_p, est_v, v_c);
            }
            if (plan) {
                const V3& xy = plan->hit.pos;
                double t_go = plan->hit.t, tilt = plan->tilt;
                if (!predicted)
                    say("  predicted landing %.2f m from home, %.2f s to go -> tilt %.0f deg",
                        norm2(xy[0] - home[0], xy[1] - home[1]), t_go, deg(tilt));
                predicted = xy;
                std::copy(plan->tgt, plan->tgt + 3, target);
                t_c = t + t_go;
                tilt_target = tilt;
                if (t_go < p.tilt_freeze) {
                    frozen = true;
                    // the best-informed state of the ball: a replay integrates it back to the release
                    throw_.set("statef", rounded({est_p[0], est_p[1], est_p[2], est_v[0], est_v[1], est_v[2]}, 4));
                    throw_.set("tf", Json(py_round(t - (throw_.has("t0") ? throw_["t0"].num() : t), 3)));
                    learn_hand(est_p, est_v, pose);
                    moving_plan = v_end > 0.3;
                    auto cr = track->crossing(t, cfg.rim_center(tilt)[1]);
                    if (cr) land_sigma = std::sqrt(eig2_max(cr->cov[0], cr->cov[2], cr->cov[3]));
                }
            }
        }
        bool arrived = t_c && t > *t_c + 0.3;
        bool lost = !predicted && t - t_track_seen > 1.0;
        if (rv.ball_held || arrived || lost || t - t_flight > 4.0) {
            if (lost && !rv.ball_held) say("  track lost before a plan");
            set("settle", t);
        }
    }

    if (state == "settle") {
        // the catch beam decides: a ball it has seen counts (wait up to 3 s for it to hold 0.3 s); without it, after
        // 0.8 s; with the base at rest
        double sp = std::hypot(rv.vel_body[0], rv.vel_body[1]);
        beam_seen = beam_seen || rv.ball_held;
        if (rv.ball_held) t_beam = (t_beam && *t_beam != 0.0) ? *t_beam : t;
        else t_beam.reset();
        bool sure = rv.ball_held && t - *t_beam >= 0.3;
        bool quiet = !beam_seen && t - t_state > 0.8;
        if (t - t_state > 0.5 && ((sp < 0.05 && (sure || quiet)) || t - t_state > 3.0))
            settle_done(t, rv, pose);
    }

    if (state == "aim") {
        bool aimed = std::fabs(wrap(target[2] - pose[2])) < rad(0.5) && std::fabs(rv.vel_body[2]) < 0.05 &&
                     std::fabs(rv.tilt - tilt_target) < rad(1.0) && std::fabs(rv.tilt_cmd - tilt_target) < 1e-3;
        if (aimed && !final_plan) {
            plan_return(pose, true, rv.tilt);
            final_plan = true;
        }
        double tg = fly_rpm;
        bool ready = rv.fly_ready && tg > 0 && std::fabs(rv.fly_rpm[0] - tg) < 0.02 * tg &&
                     std::fabs(rv.fly_rpm[1] - tg) < 0.02 * tg;
        if (ready) t_ready = (t_ready && *t_ready != 0.0) ? *t_ready : t;
        else t_ready.reset();
        bool still = std::hypot(rv.vel_body[0], rv.vel_body[1]) < 0.03;
        if ((aimed && final_plan && ready && t - *t_ready >= 0.1 && still) || t - t_state > 3.0) {
            fire_seq = (uint16_t)((fire_seq + 1) & 0xFFFF);
            last.set("return_speed", Json(v_needed));
            last.set("return_elev_deg", Json(deg(elev)));
            set("launch", t);
            say("  fire: %.2f m/s at %.1f deg, %.0f rpm", v_needed, deg(elev), fly_rpm);
        }
    }

    if (state == "launch" && t - t_state > 0.6) set("return", t);

    if (state == "return") {
        fly_rpm = 0.0, tilt_target = 0.0;
        std::copy(home, home + 3, target);
        target[2] = face(home[0], home[1], hand);
        if (t - t_state > 1.6) set("wait", t);
    }

    // the command
    std::copy(target, target + 3, cmd.target);
    cmd.tilt = tilt_target;
    cmd.fly_rpm = fly_rpm;
    cmd.fire_seq = fire_seq;
    if (state == "flight" || state == "settle") {
        cmd.v_end = (state == "flight" && predicted) ? v_end : 0.0;
        if (frozen && t_c) {
            cmd.t_brake = *t_c;
            bool moving = moving_plan || norm2(v_c[0], v_c[1]) > 0.05;
            cmd.coast_s = moving ? p.coast_moving : p.coast_after_catch;
            cmd.a_brake = p.brake_with_ball;
            cmd.t_tilt = *t_c + p.dump_delay;
            cmd.tilt_after = 0.0;
        }
    }
    return cmd;
}

}  // namespace omni

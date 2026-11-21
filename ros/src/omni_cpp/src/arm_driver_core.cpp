#include "omni_cpp/arm_driver_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "omni_cpp/json_value.hpp"

namespace omni {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ArmDriverCore::ArmDriverCore(const std::string& config_path, const std::string& calibration_path,
                             const std::string& load_path)
    : fk_(std::make_unique<ArmFk>(config_path)) {
    model = LoadModel::load(load_path, *fk_);
    for (int i = 0; i < 6; ++i) cal[i] = sts::nominal_calibration(fk_->servo_ids.at(i), fk_->lo[i], fk_->hi[i]);
    if (calibration_path.empty()) return;
    std::ifstream f(calibration_path);
    if (!f) throw std::runtime_error("calibration: can't read " + calibration_path);
    std::stringstream ss;
    ss << f.rdbuf();
    Json raw = Json::parse(ss.str());
    for (int i = 0; i < 6; ++i) {
        const Json& j = raw[fk_->joints.at(i)];
        sts::JointCal c;
        c.id = (int)j["id"].integer();
        c.drive_mode = j.has("drive_mode") ? (int)j["drive_mode"].integer() : 0;
        c.homing_offset = j.has("homing_offset") ? (int)j["homing_offset"].integer() : 0;
        c.range_min = (int)j["range_min"].integer();
        c.range_max = (int)j["range_max"].integer();
        cal[i] = c;
    }
}

std::vector<int> ArmDriverCore::connect(sts::Port& port, double timeout_s, sts::Bus::Clock clock) {
    auto bus = std::make_unique<sts::Bus>(port, timeout_s, clock);
    std::vector<int> miss;
    for (int id : fk_->servo_ids)
        if (!bus->ping(id)) miss.push_back(id);
    if (!miss.empty()) return miss;
    bus_ = std::move(bus);
    try {
        configure();
        enable(true);
        read();
    } catch (...) {
        bus_.reset();
        throw;
    }
    goal = q, have_goal = true, goal_prev_ = q;
    play_.reset();
    moving = false;
    detector.reset();
    return miss;
}

void ArmDriverCore::disconnect() {
    bus_.reset();
    play_.reset();
    moving = false;
    applied_arm_ = applied_grip_ = -1;
}

void ArmDriverCore::configure() {
    // the settings each servo should hold (EEPROM); written only where they differ, with the lock open for the write
    config_notes.clear();
    const ServoSettings& s = settings;
    for (int i = 0; i < 6; ++i) {
        int id = cal[i].id;
        bool grip = i == 5;
        std::vector<std::pair<std::string, int>> want = {
            {"p_gain", s.p_gain}, {"d_gain", s.d_gain}, {"i_gain", s.i_gain}, {"max_temp", s.max_temp},
            {"min_volt", s.min_volt}, {"max_volt", s.max_volt},
            {"protect_current", grip ? s.grip_protect_current : s.protect_current},
            {"overload_torque", grip ? s.grip_overload_torque : s.overload_torque},
            {"protect_torque", s.protect_torque}, {"protect_time", s.protect_time}};
        std::vector<std::pair<std::string, int>> diff;
        for (auto& [reg, v] : want) {
            int now = 0;
            if (!bus_->read(id, reg, now)) {
                config_notes.push_back("id " + std::to_string(id) + ": no answer reading " + reg);
                diff.clear();
                break;
            }
            if (now != v) diff.push_back({reg, v}), config_notes.push_back(
                "id " + std::to_string(id) + " " + reg + " " + std::to_string(now) + (s.write ? " -> " : " (want ") +
                std::to_string(v) + (s.write ? "" : ")"));
        }
        if (diff.empty() || !s.write) continue;
        bus_->write(id, "lock", 0);
        for (auto& [reg, v] : diff) bus_->write(id, reg, v);
        bus_->write(id, "lock", 1);
        for (auto& [reg, v] : diff) {
            int now = 0;
            if (!bus_->read(id, reg, now) || now != v)
                config_notes.push_back("id " + std::to_string(id) + " " + reg + ": didn't take");
        }
    }
}

void ArmDriverCore::enable(bool on) {
    for (int i = 0; i < 6; ++i) {
        int id = cal[i].id;
        if (on) {
            bus_->write(id, "torque_limit", i == 5 ? grip_torque : arm_torque);
            bus_->write(id, "acceleration", 0);   // the servo's own (no ramp): the motions ramp
        }
        bus_->write(id, "torque_enable", on ? 1 : 0);
    }
    torque = on;
    if (on) applied_arm_ = arm_torque, applied_grip_ = grip_torque, softened_ = {};
}

void ArmDriverCore::torque_limits(bool restore_soft) {
    // a changed limit (the live parameter), or (restore_soft: a new command) a joint the reflex softened: written now
    if (!torque) return;
    bool soft = false;
    for (bool b : softened_) soft = soft || b;
    if (applied_arm_ == arm_torque && applied_grip_ == grip_torque && !(restore_soft && soft)) return;
    for (int i = 0; i < 6; ++i) bus_->write(cal[i].id, "torque_limit", i == 5 ? grip_torque : arm_torque, false);
    applied_arm_ = arm_torque, applied_grip_ = grip_torque, softened_ = {};
}

void ArmDriverCore::read(double t) {
    auto st = bus_->sync_read_status(fk_->servo_ids);
    missing.clear(), errors.clear();
    for (int i = 0; i < 6; ++i) {
        auto it = st.find(cal[i].id);
        if (it == st.end()) {
            missing.push_back(cal[i].id);         // its joint keeps the last values read
            continue;
        }
        const sts::ServoStatus& r = it->second;
        if (!r.ok) errors.push_back(cal[i].id);   // an error reply (overload, heat, voltage)
        q_enc[i] = cal[i].to_rad(r.position);
        dq[i] = cal[i].sign() * r.speed * 2 * kPi / sts::TICKS;
        load[i] = cal[i].sign() * r.load;
        current[i] = cal[i].sign() * r.current;
        has_current = r.has_current && fk_->current_feedback;
        temp_c[i] = r.temp_c;
        status[i] = r.status;
        volts_j[i] = r.volts;
        volts = r.volts;
    }
    q = use_model_angles ? true_angles(model, *fk_, q_enc, gravity) : q_enc;
    // the load on each joint (design.md 4.5)
    ServoSample s;
    s.q = q, s.w = dq, s.duty = load, s.volts = volts_j, s.current = current, s.goal_vel = goal_vel,
    s.goal_acc = goal_acc;
    for (int i = 0; i < 6; ++i) s.temp[i] = temp_c[i];
    LoadModel m = model;
    m.use_current = model.use_current && has_current;
    est = estimate(m, *fk_, s, gravity);
    double dt = t_read_ >= 0 && t >= 0 ? t - t_read_ : 0.02;
    t_read_ = t;
    bool fresh = detector.update(model, est, dt);
    tool_force_n = tool_force(*fk_, q, est.ext, fk_->tool_point());
    bool still = torque && have_goal && !moving && !reflex_active;
    if (!still) t_still_ = -1;
    else if (t_still_ < 0) t_still_ = t;
    double still_for = still && t >= 0 ? t - t_still_ : 0.0;
    // the deflection cue: settled, a joint the encoders say has given
    for (int i = 0; i < N_ARM; ++i) {
        if (reflex_active) {                      // the joints that fired it stay strained while it holds
            detector.strained[i] = detector.strained[i] || deflected[i];
            continue;
        }
        double dev = q[i] - goal[i];
        bool over = deflect.on && guard && still && still_for >= deflect.settle && std::fabs(dev) > deflect.angle;
        def_count_[i] = over ? def_count_[i] + 1 : 0;
        bool d = def_count_[i] >= deflect.n;
        if (d && !deflected[i]) fresh = true, push_sign_[i] = dev > 0 ? 1.0 : -1.0;
        deflected[i] = d;
        detector.strained[i] = detector.strained[i] || d;
    }
    // the hold trim: settling, integrate what the encoders say is left (in the encoders' angles)
    if (trim_cfg.on && guard && still && still_for < deflect.settle) {   // unguarded: the servos raw
        for (int i = 0; i < N_ARM; ++i) {
            double e = goal[i] - q[i];
            bool pushed = std::fabs(detector.filt[i]) > 0.5 * model.j[i].thr || detector.strained[i];
            if (pushed || std::fabs(e) < trim_cfg.deadband) continue;
            trim[i] = std::min(std::max(trim[i] + trim_cfg.rate * dt * e, -trim_cfg.max), trim_cfg.max);
        }
    }
    bool armed = reflex.on && (reflex.nominal_ok || model.source != "nominal");
    if (fresh && guard && armed && torque && !reflex_active) fire_reflex();
}

void ArmDriverCore::fire_reflex() {
    // stop: hold where the arm is, and let the strained joints give with the force
    reflex_active = true;
    ++reflexes;
    reflex_why.clear();
    play_.reset(), moving = false;
    goal = q, have_goal = true;
    for (int i = 0; i < N_ARM; ++i) {
        if (!detector.strained[i]) continue;
        double s = deflected[i] ? push_sign_[i] : detector.filt[i] > 0 ? 1.0 : -1.0;
        goal[i] = std::min(std::max(goal[i] + s * reflex.backoff, fk_->lo[i]), fk_->hi[i]);
        char buf[96];
        std::snprintf(buf, sizeof buf, "%s%s %+.2f N m", reflex_why.empty() ? "" : ", ", fk_->joints[i].c_str(),
                      detector.filt[i]);
        reflex_why += buf;
        if (reflex.soft_torque < arm_torque) {
            bus_->write(cal[i].id, "torque_limit", reflex.soft_torque, false);
            softened_[i] = true;
        }
    }
    goal_vel = {}, goal_acc = {};
}

void ArmDriverCore::send(const Joints& g) {
    Joints c;
    std::vector<std::pair<int, int>> goals;
    for (int i = 0; i < 6; ++i) c[i] = std::min(std::max(g[i], fk_->lo[i]), fk_->hi[i]);   // never outside a range
    Joints e = use_model_angles ? encoder_goals(model, *fk_, c, gravity) : c;
    for (int i = 0; i < 6; ++i) goals.push_back({cal[i].id, cal[i].to_ticks(e[i] + (i < N_ARM ? trim[i] : 0.0))});
    goal = c;
    bus_->sync_write_goals(goals);
}

bool ArmDriverCore::command(int cmd, bool linear, const std::vector<double>& times, const std::vector<double>& wp,
                            std::string& why, bool guard_on) {
    if (!bus_) {
        why = "the arm isn't connected";
        return false;
    }
    if (cmd == RELAX) {
        play_.reset(), moving = false;
        enable(false);
        have_goal = false;
    } else if (cmd == HOLD) {
        play_.reset(), moving = false;
        if (!torque) enable(true);
        goal = q, have_goal = true;
    } else if (cmd == MOVE) {
        size_t n = times.size();
        if (n == 0 || wp.size() != 6 * n) {
            why = std::to_string(n) + " times, " + std::to_string(wp.size()) + " waypoint values";
            return false;
        }
        if (!torque) enable(true), goal = q, have_goal = true;
        std::vector<Joints> rows(n);
        for (size_t k = 0; k < n; ++k)
            for (int i = 0; i < 6; ++i) rows[k][i] = wp[6 * k + i];
        rows[0] = goal;                           // from where the arm is commanded now
        play_ = std::make_unique<ArmPlay>(linear ? ArmPlay::LINEAR : ArmPlay::QUINTIC, times, std::move(rows));
        moving = true;
    } else {
        why = "unknown command " + std::to_string(cmd);
        return false;
    }
    guard = guard_on;
    reflex_active = false;                        // a new command takes over from the reflex
    detector.reset();
    deflected = {}, def_count_ = {};
    torque_limits(true);
    return true;
}

void ArmDriverCore::drive(double t) {
    torque_limits(false);
    if (play_ && !play_->empty()) {
        goal = play_->sample(t);
        if (play_->done(t)) moving = false;
    }
    // the goal's velocity and acceleration (the load model's moving direction and the rotor's inertia)
    double dt = t_drive_ >= 0 ? t - t_drive_ : 0;
    if (dt > 1e-4) {
        for (int i = 0; i < 6; ++i) {
            double v = (goal[i] - goal_prev_[i]) / dt;
            goal_acc[i] = moving ? (v - goal_vel[i]) / dt : 0.0;
            goal_vel[i] = moving ? v : 0.0;
        }
    }
    goal_prev_ = goal;
    t_drive_ = t;
    if (have_goal && torque) send(goal);
}

bool ArmDriverCore::tool_out() const { return fk_->tool_out(q) || (have_goal && fk_->tool_out(goal)); }

}  // namespace omni

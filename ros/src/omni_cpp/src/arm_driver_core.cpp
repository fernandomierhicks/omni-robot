#include "omni_cpp/arm_driver_core.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "omni_cpp/json_value.hpp"

namespace omni {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

ArmDriverCore::ArmDriverCore(const std::string& config_path, const std::string& calibration_path)
    : fk_(std::make_unique<ArmFk>(config_path)) {
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
        enable(true);
        read();
    } catch (...) {
        bus_.reset();
        throw;
    }
    goal = q, have_goal = true;
    play_.reset();
    moving = false;
    return miss;
}

void ArmDriverCore::disconnect() {
    bus_.reset();
    play_.reset();
    moving = false;
}

void ArmDriverCore::enable(bool on) {
    for (int i = 0; i < 6; ++i) {
        int id = cal[i].id;
        if (on) {
            bus_->write(id, "torque_limit", i == 5 ? GRIP_TORQUE : ARM_TORQUE);
            bus_->write(id, "acceleration", 0);   // the servo's own (no ramp): the motions ramp
        }
        bus_->write(id, "torque_enable", on ? 1 : 0);
    }
    torque = on;
}

void ArmDriverCore::read() {
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
        q[i] = cal[i].to_rad(r.position);
        dq[i] = cal[i].sign() * r.speed * 2 * kPi / sts::TICKS;
        load[i] = cal[i].sign() * r.load;
        temp_c[i] = r.temp_c;
        volts = r.volts;
    }
}

void ArmDriverCore::send(const Joints& g) {
    Joints c;
    std::vector<std::pair<int, int>> goals;
    for (int i = 0; i < 6; ++i) {
        c[i] = std::min(std::max(g[i], fk_->lo[i]), fk_->hi[i]);   // never a goal outside a joint's range
        goals.push_back({cal[i].id, cal[i].to_ticks(c[i])});
    }
    goal = c;
    bus_->sync_write_goals(goals);
}

bool ArmDriverCore::command(int cmd, bool linear, const std::vector<double>& times, const std::vector<double>& wp,
                            std::string& why) {
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
    return true;
}

void ArmDriverCore::drive(double t) {
    if (play_ && !play_->empty()) {
        goal = play_->sample(t);
        if (play_->done(t)) moving = false;
    }
    if (have_goal && torque) send(goal);
}

bool ArmDriverCore::tool_out() const { return fk_->tool_out(q) || (have_goal && fk_->tool_out(goal)); }

}  // namespace omni

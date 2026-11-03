#include "omni_cpp/feetech.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <set>

namespace omni::sts {

constexpr double kPi = 3.14159265358979323846;

namespace {

const std::map<std::string, Reg>& table() {
    static const std::map<std::string, Reg> t = {
        {"id", {5, 1}},           {"baud", {6, 1}},          {"min_limit", {9, 2}},     {"max_limit", {11, 2}},
        {"homing_offset", {31, 2}}, {"mode", {33, 1}},       {"torque_enable", {40, 1}}, {"acceleration", {41, 1}},
        {"goal_position", {42, 2}}, {"goal_time", {44, 2}},  {"goal_speed", {46, 2}},   {"torque_limit", {48, 2}},
        {"lock", {55, 1}},        {"present_position", {56, 2}}, {"present_speed", {58, 2}}, {"present_load", {60, 2}},
        {"present_voltage", {62, 1}}, {"present_temperature", {63, 1}}, {"moving", {66, 1}},
        {"present_current", {69, 2}}};
    return t;
}

void u16(std::vector<uint8_t>& v, int x) {
    v.push_back((uint8_t)(x & 0xFF));
    v.push_back((uint8_t)((x >> 8) & 0xFF));
}

double steady_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

Reg reg(const std::string& name) { return table().at(name); }

uint8_t checksum(const uint8_t* body, size_t n) {
    unsigned s = 0;
    for (size_t i = 0; i < n; ++i) s += body[i];
    return (uint8_t)(~s & 0xFF);
}

static std::vector<uint8_t> framed(uint8_t sid, uint8_t code, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> p = {0xFF, 0xFF, sid, (uint8_t)(params.size() + 2), code};
    p.insert(p.end(), params.begin(), params.end());
    p.push_back(checksum(p.data() + 2, p.size() - 2));
    return p;
}

std::vector<uint8_t> packet(uint8_t sid, uint8_t instr, const std::vector<uint8_t>& params) {
    return framed(sid, instr, params);
}

std::vector<uint8_t> status_packet(uint8_t sid, uint8_t error, const std::vector<uint8_t>& params) {
    return framed(sid, error, params);
}

int to_sm(int v, int bit) { return (std::abs(v) & ((1 << bit) - 1)) | (v < 0 ? (1 << bit) : 0); }

int from_sm(int v, int bit) { return (v & (1 << bit)) ? -(v & ((1 << bit) - 1)) : (v & ((1 << bit) - 1)); }

std::vector<Packet> Parser::feed(const uint8_t* data, size_t n) {
    buf_.insert(buf_.end(), data, data + n);
    std::vector<Packet> out;
    for (;;) {
        size_t k = 0;
        while (k + 1 < buf_.size() && !(buf_[k] == 0xFF && buf_[k + 1] == 0xFF)) ++k;
        if (k + 1 >= buf_.size()) {                   // no FF FF: keep a trailing byte (it may be the first FF)
            if (buf_.size() > 1) buf_.erase(buf_.begin(), buf_.end() - 1);
            return out;
        }
        if (k) buf_.erase(buf_.begin(), buf_.begin() + (long)k);
        if (buf_.size() < 6) return out;
        uint8_t sid = buf_[2], len = buf_[3];
        if (sid == 0xFF) {                            // a third FF: resync on the next byte
            buf_.erase(buf_.begin());
            continue;
        }
        if (len < 2) {
            buf_.erase(buf_.begin(), buf_.begin() + 2);
            ++errors;
            continue;
        }
        if (buf_.size() < 4u + len) return out;
        uint8_t chk = buf_[3 + len];
        bool good = checksum(buf_.data() + 2, 2u + len - 1) == chk;
        Packet p{sid, len, buf_[4], std::vector<uint8_t>(buf_.begin() + 5, buf_.begin() + 4 + len - 1)};
        buf_.erase(buf_.begin(), buf_.begin() + 4 + len);
        if (!good) {
            ++errors;
            continue;
        }
        out.push_back(std::move(p));
    }
}

double JointCal::to_rad(int ticks) const { return sign() * (ticks - mid()) * 2 * kPi / TICKS; }

int JointCal::to_ticks(double rad) const {
    int t = (int)std::nearbyint(mid() + sign() * rad * TICKS / (2 * kPi));
    return std::min(std::max(t, range_min), range_max);
}

JointCal nominal_calibration(int id, double lo, double hi) {
    double half = std::max(std::abs(lo), std::abs(hi)) * TICKS / (2 * kPi);
    JointCal c;
    c.id = id;
    c.range_min = (int)std::nearbyint(2048 - half);
    c.range_max = (int)std::nearbyint(2048 + half);
    return c;
}

Bus::Bus(Port& port, double timeout_s, Clock clock)
    : port_(port), timeout_(timeout_s), clock_(clock ? std::move(clock) : Clock(steady_now)) {}

void Bus::send(const std::vector<uint8_t>& d) {
    port_.write(d.data(), d.size());
    ++n_tx;
}

std::map<int, Packet> Bus::replies(const std::vector<int>& ids) {
    std::set<int> want(ids.begin(), ids.end());
    std::map<int, Packet> got;
    double t_end = clock_() + timeout_;
    uint8_t buf[256];
    while (got.size() < want.size()) {
        double left = t_end - clock_();
        size_t n = port_.read(buf, sizeof buf, left > 0 ? (int)std::ceil(left * 1000) : 0);
        for (auto& p : parser.feed(buf, n))
            if (want.count(p.id)) {
                got[p.id] = std::move(p);
                ++n_rx;
            }
        if (got.size() < want.size() && clock_() > t_end) {
            ++n_timeouts;
            break;
        }
    }
    return got;
}

bool Bus::ping(int sid) {
    send(packet((uint8_t)sid, INST_PING));
    return replies({sid}).count(sid) > 0;
}

bool Bus::write(int sid, const std::string& reg_name, int value, bool reply) {
    Reg r = reg(reg_name);
    std::vector<uint8_t> params = {(uint8_t)r.addr};
    if (r.size == 1) params.push_back((uint8_t)(value & 0xFF));
    else u16(params, value);
    send(packet((uint8_t)sid, INST_WRITE, params));
    return reply ? replies({sid}).count(sid) > 0 : true;
}

void Bus::sync_write_goals(const std::vector<std::pair<int, int>>& goals) {
    std::vector<uint8_t> params = {(uint8_t)reg("goal_position").addr, 6};
    for (auto& [sid, pos] : goals) {
        params.push_back((uint8_t)sid);
        u16(params, pos);
        u16(params, 0);
        u16(params, 0);
    }
    send(packet(BROADCAST, INST_SYNC_WRITE, params));
}

std::map<int, ServoStatus> Bus::sync_read_status(const std::vector<int>& ids) {
    std::vector<uint8_t> params = {STATUS_ADDR, STATUS_LEN};
    for (int i : ids) params.push_back((uint8_t)i);
    send(packet(BROADCAST, INST_SYNC_READ, params));
    std::map<int, ServoStatus> out;
    for (auto& [sid, p] : replies(ids)) {
        const auto& q = p.params;
        if ((int)q.size() != STATUS_LEN) continue;
        ServoStatus s;
        s.position = q[0] | q[1] << 8;
        s.speed = from_sm(q[2] | q[3] << 8, 15);
        s.load = from_sm(q[4] | q[5] << 8, 10) * LOAD_UNIT;
        s.volts = q[6] * 0.1;
        s.temp_c = q[7];
        s.ok = p.code == 0;
        out[sid] = s;
    }
    return out;
}

}  // namespace omni::sts

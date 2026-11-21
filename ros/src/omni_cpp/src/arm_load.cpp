#include "omni_cpp/arm_load.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include "omni_cpp/json_value.hpp"

namespace omni {

namespace {
double sgn(double v) { return v > 0 ? 1.0 : v < 0 ? -1.0 : 0.0; }
}  // namespace

LoadModel LoadModel::nominal(const ArmFk& fk) {
    LoadModel m;
    m.links = fk.links;
    double stall = fk.stall_torque > 0 ? fk.stall_torque : 2.942, speed = fk.servo_speed > 0 ? fk.servo_speed : 4.7;
    for (auto& L : m.j) {
        L.A = stall / 12.0;                      // the datasheet's stall torque at full duty on 12 V
        L.E = stall / speed;                     // ... gone at the no-load speed
        L.F = L.Fs = 0.08, L.mu = L.mus = 0.6;  // unknown gearbox: a wide band (est)
        L.thr = 0.45, L.thr_move = 0.9;
    }
    return m;
}

LoadModel LoadModel::load(const std::string& path, const ArmFk& fk) {
    LoadModel m = nominal(fk);
    if (path.empty()) return m;
    std::ifstream f(path);
    if (!f) throw std::runtime_error("load model: can't read " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    Json raw = Json::parse(ss.str());
    m.source = raw.has("source") ? raw["source"].str() : path;
    m.use_current = raw.has("use_current") && raw["use_current"].boolean();
    if (raw.has("joints")) {
        const Json& js = raw["joints"];
        for (int i = 0; i < N_ARM; ++i) {
            const std::string& name = fk.joints.at(i);
            if (!js.has(name)) continue;
            const Json& j = js[name];
            JointLoad& L = m.j[i];
            auto take = [&](const char* k, double& v) {
                if (j.has(k)) v = j[k].num();
            };
            take("A", L.A), take("E", L.E), take("J", L.J), take("o", L.o), take("F", L.F), take("mu", L.mu);
            take("Fs", L.Fs), take("mus", L.mus);
            take("Ki", L.Ki), take("io", L.io), take("alpha", L.alpha), take("zero", L.zero);
            take("compliance", L.compliance), take("thr", L.thr), take("thr_move", L.thr_move);
        }
    }
    if (raw.has("links")) {
        m.links.clear();
        for (size_t i = 0; i < raw["links"].size(); ++i) {
            const Json& l = raw["links"][i];
            auto c = l["com"].nums();
            m.links.push_back({l["body"].str(), l["mass"].num(), {c.at(0), c.at(1), c.at(2)}});
        }
    }
    return m;
}

LoadEstimate estimate(const LoadModel& m, const ArmFk& fk, const ServoSample& s, const Vec3& g) {
    LoadEstimate e;
    e.grav = fk.gravity(s.q, g, m.links);
    e.inertia = fk.inertia_torque(s.q, s.goal_acc, m.links);
    for (int i = 0; i < N_ARM; ++i) {
        const JointLoad& L = m.j[i];
        double v = std::fabs(s.goal_vel[i]) > LOAD_V_EPS ? s.goal_vel[i] : std::fabs(s.w[i]) > LOAD_W_EPS ? s.w[i] : 0.0;
        double tm;
        if (m.use_current && L.Ki != 0)
            tm = L.Ki * (s.current[i] - L.io);
        else
            tm = L.A * s.duty[i] * s.volts[i] / (1.0 + L.alpha * (s.temp[i] - 25.0)) - L.E * v;
        tm -= L.J * s.goal_acc[i] + L.o;
        double c = -e.grav[i] - tm + e.inertia[i];   // the outside torque if nothing were friction
        e.motor[i] = tm;
        if (v != 0) {
            e.moving[i] = true;
            e.ext[i] = c + sgn(v) * (L.F + L.mu * std::fabs(e.grav[i]));
        } else {
            double fb = L.Fs + L.mus * std::fabs(e.grav[i]);
            e.band[i] = fb;
            e.ext[i] = sgn(c) * std::max(0.0, std::fabs(c) - fb);
        }
    }
    return e;
}

Vec3 tool_force(const ArmFk& fk, const Joints& q, const Joints& ext, const Vec3& p, double damping) {
    auto J = fk.jacobian(q, p);
    double A[3][3] = {}, b[3] = {};
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < N_ARM; ++k) A[r][c] += J[r * N_ARM + k] * J[c * N_ARM + k];
            if (r == c) A[r][c] += damping;
        }
        for (int k = 0; k < N_ARM; ++k) b[r] += J[r * N_ARM + k] * ext[k];
    }
    // Cramer's rule on the 3 x 3
    auto det = [](double M[3][3]) {
        return M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
    };
    double D = det(A);
    Vec3 f{};
    if (std::fabs(D) < 1e-15) return f;
    for (int c = 0; c < 3; ++c) {
        double M[3][3];
        for (int r = 0; r < 3; ++r)
            for (int k = 0; k < 3; ++k) M[r][k] = k == c ? b[r] : A[r][k];
        f[c] = det(M) / D;
    }
    return f;
}

Joints true_angles(const LoadModel& m, const ArmFk& fk, const Joints& q_enc, const Vec3& g) {
    Joints q = q_enc;
    for (int i = 0; i < N_ARM; ++i) q[i] += m.j[i].zero;
    bool sag = false;
    for (int i = 0; i < N_ARM; ++i) sag = sag || m.j[i].compliance != 0;
    if (!sag) return q;
    Joints base = q;
    for (int it = 0; it < 2; ++it) {             // the sag depends on the true pose: two rounds settle it
        Joints gr = fk.gravity(q, g, m.links);
        for (int i = 0; i < N_ARM; ++i) q[i] = base[i] + m.j[i].compliance * gr[i];
    }
    return q;
}

Joints encoder_goals(const LoadModel& m, const ArmFk& fk, const Joints& q_true, const Vec3& g) {
    Joints q = q_true;
    Joints gr = fk.gravity(q_true, g, m.links);
    for (int i = 0; i < N_ARM; ++i) q[i] = q_true[i] - m.j[i].zero - m.j[i].compliance * gr[i];
    return q;
}

void StrainDetector::reset() {
    filt = {}, count = {}, strained = {};
}

bool StrainDetector::update(const LoadModel& m, const LoadEstimate& e, double dt) {
    double a = dt > 0 ? dt / (tau + dt) : 1.0;
    bool fresh = false;
    for (int i = 0; i < N_ARM; ++i) {
        filt[i] += a * (e.ext[i] - filt[i]);
        double thr = e.moving[i] ? m.j[i].thr_move : m.j[i].thr;
        count[i] = std::fabs(filt[i]) > thr ? count[i] + 1 : 0;
        bool s = count[i] >= n;
        fresh = fresh || (s && !strained[i]);
        strained[i] = s;
    }
    return fresh;
}

}  // namespace omni

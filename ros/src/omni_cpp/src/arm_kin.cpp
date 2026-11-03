#include "omni_cpp/arm_kin.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace omni {

namespace {

Mat4 from_rt(const double R[9], const Vec3& p) {
    return {R[0], R[1], R[2], p[0], R[3], R[4], R[5], p[1], R[6], R[7], R[8], p[2], 0, 0, 0, 1};
}

Mat4 quat_mat(const std::vector<double>& q, const Vec3& p) {
    double w = q.at(0), x = q.at(1), y = q.at(2), z = q.at(3);
    double n = std::sqrt(w * w + x * x + y * y + z * z);
    w /= n, x /= n, y /= n, z /= n;
    double R[9] = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
                   2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                   2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
    return from_rt(R, p);
}

Mat4 axis_mat(const Vec3& a, double ang) {
    double x = a[0], y = a[1], z = a[2];
    double c = std::cos(ang), s = std::sin(ang), C = 1 - std::cos(ang);
    double R[9] = {c + x * x * C,     x * y * C - z * s, x * z * C + y * s,
                   y * x * C + z * s, c + y * y * C,     y * z * C - x * s,
                   z * x * C - y * s, z * y * C + x * s, c + z * z * C};
    return from_rt(R, {0, 0, 0});
}

Mat4 translation(const Vec3& p) {
    Mat4 T = mat_identity();
    T[3] = p[0], T[7] = p[1], T[11] = p[2];
    return T;
}

Vec3 vec3(const Json& j) {
    auto v = j.nums();
    return {v.at(0), v.at(1), v.at(2)};
}

}  // namespace

Mat4 mat_identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }

Mat4 mat_mul(const Mat4& a, const Mat4& b) {
    Mat4 c{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0;
            for (int k = 0; k < 4; ++k) s += a[i * 4 + k] * b[k * 4 + j];
            c[i * 4 + j] = s;
        }
    return c;
}

std::array<double, 4> quat_of(const Mat4& T) {
    auto R = [&](int i, int j) { return T[i * 4 + j]; };
    double t = R(0, 0) + R(1, 1) + R(2, 2);
    if (t > 0) {
        double s = std::sqrt(t + 1.0) * 2;
        return {(R(2, 1) - R(1, 2)) / s, (R(0, 2) - R(2, 0)) / s, (R(1, 0) - R(0, 1)) / s, 0.25 * s};
    }
    int i = 0;                                   // np.argmax(diag): the first of equal maxima
    if (R(1, 1) > R(i, i)) i = 1;
    if (R(2, 2) > R(i, i)) i = 2;
    int j = (i + 1) % 3, k = (i + 2) % 3;
    double s = std::sqrt(1.0 + R(i, i) - R(j, j) - R(k, k)) * 2;
    double q[4];
    q[i] = 0.25 * s;
    q[j] = (R(j, i) + R(i, j)) / s;
    q[k] = (R(k, i) + R(i, k)) / s;
    q[3] = (R(k, j) - R(j, k)) / s;
    return {q[0], q[1], q[2], q[3]};
}

ArmFk::ArmFk(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("arm config: can't read " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    Json raw = Json::parse(ss.str());
    const Json& a = raw["arm"];
    for (size_t i = 0; i < a["joints"].size(); ++i) joints.push_back(a["joints"][i].str());
    const Json& chain = a["chain"];
    for (size_t i = 0; i < chain.size(); ++i) {
        const Json& b = chain[i];
        Body body;
        body.name = b["body"].str();
        body.fixed = quat_mat(b["quat"].nums(), vec3(b["pos"]));
        const Json& j = b["joint"];
        if (!j.is_null()) {
            body.has_joint = true;
            body.axis = vec3(j["axis"]);
            body.pos = vec3(j["pos"]);
            auto r = j["range"].nums();
            lo.push_back(r.at(0)), hi.push_back(r.at(1));
        }
        bodies_.push_back(body);
    }
    auto jr = a["jaw"]["range"].nums();
    lo.push_back(jr.at(0)), hi.push_back(jr.at(1));
    tool_ = quat_mat(a["tool"]["quat"].nums(), vec3(a["tool"]["pos"]));
    jaw_ = quat_mat(a["jaw"]["quat"].nums(), vec3(a["jaw"]["pos"]));
    jaw_axis_ = vec3(a["jaw"]["axis"]);
    const Json& wc = a["wrist_cam"];
    cam_ = mat_mul(translation(vec3(wc["pos"])), quat_mat(wc["cam_quat"].nums(), vec3(wc["cam_pos"])));
    for (double v : a["servos"]["ids"].nums()) servo_ids.push_back((int)v);
    servo_speed = a["servos"]["speed"].num();
    out_radius = raw["arm_out"]["out_radius"].num();
    out_z = raw["arm_out"]["out_z"].num();
}

std::map<std::string, Mat4> ArmFk::fk(const Joints& q) const {
    std::map<std::string, Mat4> out;
    Mat4 T = mat_identity();
    int k = 0;
    for (const Body& b : bodies_) {
        T = mat_mul(T, b.fixed);
        if (b.has_joint) {
            T = mat_mul(mat_mul(mat_mul(T, translation(b.pos)), axis_mat(b.axis, q[k])),
                        translation({-b.pos[0], -b.pos[1], -b.pos[2]}));
            ++k;
        }
        out[b.name] = T;
    }
    const Mat4& g = out.at("arm_gripper");
    out["tool"] = mat_mul(g, tool_);
    out["arm_jaw"] = mat_mul(mat_mul(g, jaw_), axis_mat(jaw_axis_, q[5]));
    out["wrist_cam"] = mat_mul(g, cam_);
    return out;
}

Mat4 ArmFk::tool(const Joints& q) const {
    // the tool only: kinematics._chain's T @ tool
    Mat4 T = mat_identity();
    int k = 0;
    for (const Body& b : bodies_) {
        T = mat_mul(T, b.fixed);
        if (b.has_joint) {
            Mat4 J = axis_mat(b.axis, q[k]);
            if (b.pos[0] != 0 || b.pos[1] != 0 || b.pos[2] != 0) {
                for (int i = 0; i < 3; ++i)
                    J[i * 4 + 3] = b.pos[i] - (J[i * 4] * b.pos[0] + J[i * 4 + 1] * b.pos[1] + J[i * 4 + 2] * b.pos[2]);
            }
            T = mat_mul(T, J);
            ++k;
        }
    }
    return mat_mul(T, tool_);
}

bool ArmFk::tool_out(const Joints& q) const {
    Mat4 T = tool(q);
    return std::hypot(T[3], T[7]) > out_radius || T[11] > out_z;
}

}  // namespace omni

#include "base.h"

namespace fw {

static float approach_speed(float dist, float v_max, float a_brake, float gain) {
    float v = sqrtf(2.0f * a_brake * dist);
    float g = gain * dist;
    float out = v_max < v ? v_max : v;
    return out < g ? out : g;
}

void wheel_speeds(float vx, float vy, float wz, float* out) {
    for (int i = 0; i < prm::N_WHEELS; ++i) out[i] = prm::J[i][0] * vx + prm::J[i][1] * vy + prm::J[i][2] * wz;
}

void BaseController::reset() {
    v_cmd = Vec2();
    a_cmd = Vec2();
    w_cmd = 0.0f;
}

void BaseController::set_limits(const Limits& l, float speed_cap) {
    max_speed = (speed_cap > 0.0f && speed_cap < l.max_speed) ? speed_cap : l.max_speed;
    max_accel = l.max_accel;
    max_jerk = l.max_jerk;
    max_yaw_rate = l.max_yaw_rate;
    max_yaw_accel = l.max_yaw_accel;
    brake_margin = l.brake_margin;
    accel_scale = l.accel_scale;
}

float BaseController::accel_limit(Vec2 u_world, float yaw) const {
    if (prm::ACCEL_N == 0) return max_accel;
    Vec2 ub = world_to_body(u_world, yaw);
    float ang = fabsf(atan2f(ub.y, ub.x)) * 180.0f / PI_F;        // 0..180, left/right mirrored
    const float* d = prm::ACCEL_DIRS;
    const float* v = prm::ACCEL_VALS;
    float a = v[prm::ACCEL_N - 1];
    if (ang <= d[0]) {
        a = v[0];
    } else {
        for (int i = 0; i + 1 < prm::ACCEL_N; ++i) {
            if (ang <= d[i + 1]) {
                float f = (ang - d[i]) / (d[i + 1] - d[i]);
                a = v[i] + f * (v[i + 1] - v[i]);
                break;
            }
        }
    }
    return a * accel_scale;
}

float BaseController::worst_accel() const {
    if (prm::ACCEL_N == 0) return max_accel;
    float m = prm::ACCEL_VALS[0];
    for (int i = 1; i < prm::ACCEL_N; ++i) m = prm::ACCEL_VALS[i] < m ? prm::ACCEL_VALS[i] : m;
    return m * accel_scale;
}

float BaseController::top_speed(Vec2 u_world, float yaw) const {
    Vec2 ub = world_to_body(u_world, yaw);
    float n = ub.norm();
    if (n < 1e-9f) return max_speed;
    float k = 0.0f;
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        float s = fabsf(prm::J[i][0] * ub.x / n + prm::J[i][1] * ub.y / n);
        k = s > k ? s : k;
    }
    float v = prm::MAX_WHEEL_SPEED / k;
    return v < max_speed ? v : max_speed;
}

void BaseController::velocity_step(float dt, Vec2 v_des, float w_des, float yaw, float a_cap, float* out) {
    float sp = v_des.norm();
    if (sp > max_speed) v_des = v_des * (max_speed / sp);
    Vec2 dv = v_des - v_cmd;
    float n = dv.norm();
    float a_lim = n > 1e-9f ? accel_limit(dv * (1.0f / n), yaw) : max_accel;
    if (a_cap > 0.0f && a_cap < a_lim) a_lim = a_cap;
    if (max_jerk > 0.0f) {
        // fastest acceleration that can still ramp back to zero without overshooting
        float mag = sqrtf(2.0f * max_jerk * n);
        mag = a_lim < mag ? a_lim : mag;
        Vec2 a_des = n > 1e-9f ? dv * (mag / n) : Vec2();
        Vec2 da = a_des - a_cmd;
        float m = da.norm();
        if (m > max_jerk * dt) da = da * (max_jerk * dt / m);
        a_cmd = a_cmd + da;
        Vec2 step = a_cmd * dt;
        if (step.dot(dv) > n * n) step = dv;                     // don't step past the target
        v_cmd = v_cmd + step;
    } else {
        if (n > a_lim * dt) dv = dv * (a_lim * dt / n);
        a_cmd = dv * (1.0f / dt);
        v_cmd = v_cmd + dv;
    }
    float dw = clampf(w_des - w_cmd, -max_yaw_accel * dt, max_yaw_accel * dt);
    w_cmd += dw;
    Vec2 vb = world_to_body(v_cmd, yaw);
    wheel_speeds(vb.x, vb.y, w_cmd, out);
    float peak = 0.0f;
    for (int i = 0; i < prm::N_WHEELS; ++i) peak = fabsf(out[i]) > peak ? fabsf(out[i]) : peak;
    if (peak > prm::MAX_WHEEL_SPEED) {
        float k = prm::MAX_WHEEL_SPEED / peak;
        for (int i = 0; i < prm::N_WHEELS; ++i) out[i] *= k;
        v_cmd = v_cmd * k;
        w_cmd *= k;
    }
}

void BaseController::goto_step(float dt, const Pose& pose, const Pose& target, Vec2 v_ff, float v_end, float* out) {
    Vec2 e(target.x - pose.x, target.y - pose.y);
    float d = e.norm();
    // plan the stop with the worst-direction limit: the base often yaws while it brakes
    float a_brake = brake_margin * worst_accel();
    float d_eff = d;
    if (max_jerk > 0.0f) {
        // reversing from the current acceleration to braking takes (|a| + a_brake) / jerk, and the base keeps moving
        float t_rev = (a_cmd.norm() + a_brake) / max_jerk;
        d_eff = d - (v_cmd - v_ff).norm() * t_rev;
        if (d_eff < 0.0f) d_eff = 0.0f;
    }
    float v_top = d > 1e-9f ? 0.97f * top_speed(e * (1.0f / d), pose.yaw) : max_speed;
    float speed;
    if (v_end > 0.0f) {          // braking curve that ends at v_end instead of rest (a moving catch)
        speed = sqrtf(v_end * v_end + 2.0f * a_brake * d_eff);
        speed = v_top < speed ? v_top : speed;
        float g = v_end + 6.0f * d;
        speed = g < speed ? g : speed;
    } else {
        speed = approach_speed(d_eff, v_top, a_brake, 6.0f);
        float g = 6.0f * d;
        speed = g < speed ? g : speed;
    }
    Vec2 v_des = v_ff + (d > 1e-9f ? e * (speed / d) : Vec2());
    float n = v_des.norm();
    if (n > max_speed) v_des = v_des * (max_speed / n);
    float ey = wrap_angle(target.yaw - pose.yaw);
    float w_des = copysignf(approach_speed(fabsf(ey), max_yaw_rate, 0.8f * max_yaw_accel, 8.0f), ey);
    velocity_step(dt, v_des, w_des, pose.yaw, 0.0f, out);
}

// -- odometry ---------------------------------------------------------------------------------------------------

void Odometry::reset() {
    pose = Pose();
    vx = vy = wz = slip = 0.0f;
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        have_pos_[i] = false;
        w_[i] = 0.0f;
        last_dt_[i] = 0.0f;
    }
    imu_init_ = false;
    imu_ok = false;
}

void Odometry::on_wheels(float dt, const float* pos, const bool* valid, float p_max) {
    float span = 2.0f * p_max;
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        if (!valid[i]) {
            last_dt_[i] += dt;              // a missed reply: the next difference spans two ticks
            continue;
        }
        if (have_pos_[i]) {
            float dp = pos[i] - last_pos_[i];
            if (dp > p_max) dp -= span;      // the 16-bit position wraps at +/-p_max
            if (dp < -p_max) dp += span;
            float h = last_dt_[i] + dt;
            w_[i] = dp / h;
        }
        last_pos_[i] = pos[i];
        last_dt_[i] = 0.0f;
        have_pos_[i] = true;
    }
    float tw[3] = {0.0f, 0.0f, 0.0f};
    for (int r = 0; r < 3; ++r)
        for (int i = 0; i < prm::N_WHEELS; ++i) tw[r] += prm::J_PINV[r][i] * w_[i];
    vx = tw[0];
    vy = tw[1];
    if (!imu_ok) wz = tw[2];
    float s2 = 0.0f;
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        float e = prm::J[i][0] * tw[0] + prm::J[i][1] * tw[1] + prm::J[i][2] * tw[2] - w_[i];
        s2 += e * e;
    }
    slip = sqrtf(s2) * prm::WHEEL_RADIUS / sqrtf((float)prm::N_WHEELS);
}

void Odometry::on_imu(const float* q, const float* gyro) {
    float w = q[0], x = q[1], y = q[2], z = q[3];
    float yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
    float sp = 2.0f * (w * y - z * x);
    pitch = asinf(clampf(sp, -1.0f, 1.0f));
    roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    gyro_z = gyro[2];
    if (!imu_init_) {
        // the odom frame's heading is the heading at the first report (the game rotation vector's yaw is arbitrary)
        imu_yaw0_ = yaw - pose.yaw;
        imu_last_ = yaw;
        imu_unwrap_ = 0.0f;
        imu_init_ = true;
    }
    imu_unwrap_ += wrap_angle(yaw - imu_last_);
    imu_last_ = yaw;
    imu_yaw_ = wrap_angle(imu_last_ - imu_yaw0_);
    imu_ok = true;
    wz = gyro_z;
}

void Odometry::integrate(float dt) {
    if (imu_ok) {
        pose.yaw = imu_yaw_;
    } else {
        pose.yaw = wrap_angle(pose.yaw + wz * dt);
    }
    float c = cosf(pose.yaw), s = sinf(pose.yaw);
    pose.x += (c * vx - s * vy) * dt;
    pose.y += (s * vx + c * vy) * dt;
}

}  // namespace fw

#include "firmware.h"

#include <string.h>

namespace fw {

static constexpr uint64_t MS = 1000;

Limits Firmware::limits() const {
    const float* p = (have_cmd_ && cmd_.profile == PROFILE_KID) ? prm::KID_LIMITS : prm::ADULT_LIMITS;
    return Limits{p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]};
}

void Firmware::init() {
    *this = Firmware();
    mode_ = MODE_DISARMED;
    ctl_.reset();
    odom_.reset();
    ctl_.set_limits(limits(), 0.0f);
    for (auto& f : fly_) f.reset();
    feed_.reset();
}

// -- link -----------------------------------------------------------------------------------------------------------

void Firmware::rx(const uint8_t* data, size_t n) {
    uint8_t body[256];
    for (size_t i = 0; i < n; ++i) {
        uint8_t type;
        size_t len;
        if (!dec_.push(data[i], type, body, sizeof(body), len)) continue;
        if (type == MSG_CMD && len == sizeof(CmdBody)) {
            bool first = !have_cmd_;
            memcpy(&cmd_, body, sizeof(CmdBody));
            have_cmd_ = true;
            cmd_new_ = true;
            ++cmd_count_;
            if (first) {
                fire_seq_done_ = cmd_.fire_seq;      // a stale fire_seq at connect must not fire
                fire_init_ = true;
            }
        } else {
            ++dec_.errors;
        }
    }
}

void Firmware::push_tx(const uint8_t* d, size_t n) {
    for (size_t i = 0; i < n; ++i) {
        size_t next = (tx_head_ + 1) % sizeof(txbuf_);
        if (next == tx_tail_) return;            // full: drop the rest (the Jetson stopped reading)
        txbuf_[tx_head_] = d[i];
        tx_head_ = next;
    }
}

size_t Firmware::tx(uint8_t* out, size_t cap) {
    size_t n = 0;
    while (n < cap && tx_tail_ != tx_head_) {
        out[n++] = txbuf_[tx_tail_];
        tx_tail_ = (tx_tail_ + 1) % sizeof(txbuf_);
    }
    return n;
}

void Firmware::send_imu(const FwHwInputs& in) {
    ImuBody b{};
    b.t_mcu_us = t_us_;
    memcpy(b.quat, in.imu_quat, sizeof(b.quat));
    memcpy(b.gyro, in.imu_gyro, sizeof(b.gyro));
    memcpy(b.accel, in.imu_accel, sizeof(b.accel));
    uint8_t buf[128];
    size_t n = frame_encode(MSG_IMU, seq_imu_++, &b, sizeof(b), buf, sizeof(buf));
    push_tx(buf, n);
}

void Firmware::send_state(const FwHwInputs& in) {
    StateBody s{};
    s.t_mcu_us = t_us_;
    s.echo_t_host_ns = echo_host_;
    s.echo_rx_us = echo_rx_us_;
    s.mode = mode_;
    s.profile = have_cmd_ ? cmd_.profile : (uint8_t)PROFILE_ADULT;
    s.drive_mode = (have_cmd_ && cmd_.drive_mode == DRIVE_GOTO && phase_ != PH_TRACK) ? (uint8_t)3
                   : (uint8_t)(have_cmd_ ? cmd_.drive_mode : 0);
    s.feed_state = feed_.state;
    s.faults = faults_;
    for (int i = 0; i < prm::N_WHEELS && i < 4; ++i) s.dm_err[i] = fb_[i].err;
    s.x = odom_.pose.x;
    s.y = odom_.pose.y;
    s.yaw = odom_.pose.yaw;
    s.vx = odom_.vx;
    s.vy = odom_.vy;
    s.wz = odom_.wz;
    s.slip = odom_.slip;
    s.roll = odom_.roll;
    s.pitch = odom_.pitch;
    s.v_cmd_x = ctl_.v_cmd.x;
    s.v_cmd_y = ctl_.v_cmd.y;
    s.w_cmd = ctl_.w_cmd;
    for (int k = 0; k < 2; ++k) s.fly_rpm[k] = fly_[k].w_meas * 60.0f / (2.0f * PI_F);
    s.fly_target_rpm = fly_target_ * 60.0f / (2.0f * PI_F);
    s.tilt = tilt_meas_;
    s.tilt_cmd = tilt_cmd_;
    s.beams = (uint8_t)((in.beam_catch ? 1 : 0) | (in.beam_feed_exit ? 2 : 0));
    s.fly_ready = fly_ready_ ? 1 : 0;
    s.fire_seq_done = fire_seq_done_;
    s.fire_rejected = fire_rejected_;
    s.v_bus = last_v_bus_;
    s.cmd_count = cmd_count_;
    s.rx_errors = (uint16_t)dec_.errors;
    uint8_t buf[192];
    size_t n = frame_encode(MSG_STATE, seq_state_++, &s, sizeof(s), buf, sizeof(buf));
    push_tx(buf, n);
}

// -- drive motors ---------------------------------------------------------------------------------------------------

void Firmware::handle_can(const FwHwInputs& in, float dt) {
    static const DmRange rng{prm::DM_PMAX, prm::DM_VMAX, prm::DM_TMAX};
    bool valid[prm::N_WHEELS] = {};
    float pos[prm::N_WHEELS] = {};
    uint32_t n = in.n_can_rx < FW_CAN_MAX ? in.n_can_rx : FW_CAN_MAX;
    for (uint32_t k = 0; k < n; ++k) {
        const FwCanFrame& f = in.can_rx[k];
        if (f.id < prm::DM_MASTER_BASE || f.id >= prm::DM_MASTER_BASE + 16) continue;
        DmFeedback fb;
        if (!dm_decode_feedback(f, rng, fb)) continue;
        for (int i = 0; i < prm::N_WHEELS; ++i) {
            if (prm::DM_IDS[i] != fb.id) continue;
            fb.pos *= prm::DM_DIR[i];
            fb.vel *= prm::DM_DIR[i];
            fb.torque *= prm::DM_DIR[i];
            fb_[i] = fb;
            valid[i] = true;
            pos[i] = fb.pos;
            t_fb_us_[i] = t_us_;
        }
    }
    for (int i = 0; i < prm::N_WHEELS; ++i) fb_valid_[i] = valid[i];
    odom_.on_wheels(dt, pos, valid, prm::DM_PMAX);
}

void Firmware::drive_motors(bool enable, const float* wheel_cmd, FwHwOutputs& out) {
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        uint8_t id = prm::DM_IDS[i];
        uint8_t e = fb_[i].err;
        bool retry = t_us_ - t_enable_us_[i] > 50 * MS;
        if (out.n_can_tx + 2 > FW_CAN_MAX) break;
        if (enable && e != DM_ENABLED && retry) {
            if (e >= DM_OVERVOLT) out.can_tx[out.n_can_tx++] = dm_clear(id);     // clear a latched error first
            out.can_tx[out.n_can_tx++] = dm_enable(id);
            t_enable_us_[i] = t_us_;
        } else if (!enable && e == DM_ENABLED && retry) {
            out.can_tx[out.n_can_tx++] = dm_disable(id);
            t_enable_us_[i] = t_us_;
        } else {
            // the speed frame is also the poll: each one gets a feedback reply, so the wheels are read every tick
            // even when disarmed (a disabled motor ignores the setpoint)
            float w = enable ? wheel_cmd[i] : 0.0f;
            out.can_tx[out.n_can_tx++] = dm_speed_cmd(id, w * prm::DM_DIR[i]);
        }
        wheel_cmd_[i] = enable ? wheel_cmd[i] : 0.0f;
    }
}

// -- the base ---------------------------------------------------------------------------------------------------------

void Firmware::run_base(float dt, bool auto_ok, const FwHwInputs& in, float* wheel_cmd) {
    ctl_.set_limits(limits(), auto_ok ? cmd_.speed_cap : 0.0f);
    Pose pose = odom_.pose;
    if (!auto_ok || in.bumper) {
        // link lost, disarm requested or a bumper: a ramped stop, never a step to zero (design.md 3)
        phase_ = PH_TRACK;
        hold_yaw_set_ = false;
        ctl_.velocity_step(dt, Vec2(), 0.0f, pose.yaw, 0.0f, wheel_cmd);
        return;
    }
    switch (cmd_.drive_mode) {
        case DRIVE_VEL_BODY: {
            phase_ = PH_TRACK;
            brake_key_ = 0;
            Vec2 v = body_to_world(Vec2(cmd_.a, cmd_.b), pose.yaw);
            float w = cmd_.c;
            if ((cmd_.flags & CMD_HOLD_YAW) && fabsf(w) < 1e-3f) {
                if (!hold_yaw_set_) {
                    hold_yaw_ = pose.yaw;
                    hold_yaw_set_ = true;
                }
                w = clampf(6.0f * wrap_angle(hold_yaw_ - pose.yaw), -ctl_.max_yaw_rate, ctl_.max_yaw_rate);
            } else {
                hold_yaw_set_ = false;
            }
            ctl_.velocity_step(dt, v, w, pose.yaw, cmd_.a_cap, wheel_cmd);
            break;
        }
        case DRIVE_GOTO: {
            hold_yaw_set_ = false;
            if (cmd_.t_brake_us != brake_key_) {         // a new catch schedule (or none)
                brake_key_ = cmd_.t_brake_us;
                phase_ = PH_TRACK;
            }
            if (phase_ == PH_TRACK && brake_key_ && t_us_ >= brake_key_) phase_ = PH_COAST;
            if (phase_ == PH_COAST && t_us_ >= brake_key_ + (uint64_t)(cmd_.coast_s * 1e6f)) phase_ = PH_BRAKE;
            if (phase_ == PH_BRAKE && ctl_.v_cmd.norm() <= 0.02f) {
                phase_ = PH_HOLD;                        // at rest with the ball: hold wherever the base stopped
                hold_pose_ = pose;
            }
            switch (phase_) {
                case PH_TRACK: {
                    Pose tgt;
                    tgt.x = cmd_.a;
                    tgt.y = cmd_.b;
                    tgt.yaw = cmd_.c;
                    ctl_.goto_step(dt, pose, tgt, Vec2(), cmd_.v_end, wheel_cmd);
                    break;
                }
                case PH_COAST:      // hold the commanded velocity while the dump tips the ball into the pocket
                    ctl_.velocity_step(dt, ctl_.v_cmd, 0.0f, pose.yaw, 0.0f, wheel_cmd);
                    break;
                case PH_BRAKE:      // then brake gently with the ball aboard
                    ctl_.velocity_step(dt, Vec2(), 0.0f, pose.yaw, cmd_.a_brake, wheel_cmd);
                    break;
                case PH_HOLD:
                    ctl_.goto_step(dt, pose, hold_pose_, Vec2(), 0.0f, wheel_cmd);
                    break;
            }
            break;
        }
        case DRIVE_IDLE:
        default:
            phase_ = PH_TRACK;
            brake_key_ = 0;
            hold_yaw_set_ = false;
            ctl_.velocity_step(dt, Vec2(), 0.0f, pose.yaw, 0.0f, wheel_cmd);
            break;
    }
}

// -- the module -------------------------------------------------------------------------------------------------------

void Firmware::run_module(float dt, bool auto_ok, const FwHwInputs& in, FwHwOutputs& out) {
    float v_bus = in.v_bus > 5.0f ? in.v_bus : 5.0f;
    out.fly_brake = prm::FLY_BRAKE ? 1 : 0;
    out.tilt_enable = in.estop ? 0 : 1;
    if (!auto_ok) {
        fly_target_ = 0.0f;
        fly_ready_ = false;
        for (auto& f : fly_) f.step(dt, 0.0f, v_bus);
        feed_.reset();
        out.tilt_cmd = tilt_cmd_;                 // the servo holds where it is
        return;
    }
    float tilt = (cmd_.t_tilt_us && t_us_ >= cmd_.t_tilt_us) ? cmd_.tilt_after : cmd_.tilt;
    tilt_cmd_ = clampf(tilt, prm::TILT_MIN, prm::TILT_MAX);
    out.tilt_cmd = tilt_cmd_;
    Limits L = limits();
    float rpm = cmd_.fly_rpm < L.fly_rpm_max ? cmd_.fly_rpm : L.fly_rpm_max;
    fly_target_ = (rpm > 0.0f ? rpm : 0.0f) * 2.0f * PI_F / 60.0f;
    bool ready = fly_target_ > 0.0f;
    for (int k = 0; k < 2; ++k) {
        fly_[k].step(dt, fly_target_, v_bus);
        out.fly_throttle[k] = fly_[k].throttle;
        ready = ready && fabsf(fly_[k].w_meas - fly_target_) < prm::FLY_READY_TOL * fly_target_;
    }
    fly_ready_ = ready;
    if (fire_init_ && cmd_.fire_seq != fire_seq_done_) {
        fire_seq_done_ = cmd_.fire_seq;
        if (fly_ready_ && !in.bumper) {
            feed_.trigger();
        } else {
            ++fire_rejected_;                     // interlock: the Jetson sees the count and may ask again
        }
    }
    float f = feed_.step(dt, in.beam_feed_exit != 0, in.beam_catch != 0);
    out.feed_speed_cmd[0] = f;
    out.feed_speed_cmd[1] = f;
}

// -- the tick -----------------------------------------------------------------------------------------------------------

void Firmware::tick(const FwHwInputs& in, FwHwOutputs& out) {
    memset(&out, 0, sizeof(out));
    float dt = first_ ? prm::TICK_S : (float)(in.t_us - t_us_) * 1e-6f;
    if (dt <= 0.0f) dt = prm::TICK_S;
    if (dt > 0.05f) dt = 0.05f;
    first_ = false;
    t_us_ = in.t_us;
    if (cmd_new_) {
        cmd_new_ = false;
        t_cmd_us_ = t_us_;
        echo_rx_us_ = t_us_;
        echo_host_ = cmd_.t_host_ns;
    }

    handle_can(in, dt);
    if (in.imu_new) {
        odom_.on_imu(in.imu_quat, in.imu_gyro);
        t_imu_us_ = t_us_;
        send_imu(in);
    }
    odom_.integrate(dt);
    for (int k = 0; k < 2; ++k) {
        if (in.fly_new[k]) {
            fly_[k].on_telemetry(in.fly_erpm[k]);
            t_fly_telem_us_ = t_us_;
        }
    }
    if (in.tilt_new) tilt_meas_ = in.tilt_pos;
    if (in.feed_new) {
        feed_speed_meas_[0] = in.feed_speed[0];
        feed_speed_meas_[1] = in.feed_speed[1];
    }
    last_v_bus_ = in.v_bus;

    // faults (recomputed every tick; the Jetson logs the edges)
    bool link_ok = have_cmd_ && t_us_ - t_cmd_us_ < (uint64_t)(prm::CMD_TIMEOUT * 1e6f);
    faults_ = 0;
    if (!link_ok) faults_ |= F_LINK_LOST;
    if (in.estop) faults_ |= F_ESTOP;
    if (in.bumper) faults_ |= F_BUMPER;
    for (int i = 0; i < prm::N_WHEELS; ++i) {
        if (fb_[i].err >= DM_OVERVOLT) faults_ |= F_MOTOR_ERR;
        if (t_us_ - t_fb_us_[i] > (uint64_t)(prm::DM_FEEDBACK_TIMEOUT * 1e6f)) faults_ |= F_MOTOR_SILENT;
    }
    if (in.v_bus < prm::BATT_LOW) faults_ |= F_LOW_BATT;
    if (t_us_ - t_imu_us_ > 50 * MS) faults_ |= F_IMU_SILENT;
    if (fly_target_ > 0.0f && t_us_ - t_fly_telem_us_ > 20 * MS) faults_ |= F_FLY_TELEM;

    // mode
    bool manual = in.rc_ok && in.rc[4] > 0.5f;
    bool want_auto = link_ok && cmd_.req_mode == REQ_AUTO;
    if (in.estop) {
        if (mode_ != MODE_ESTOP) {
            ctl_.reset();
            feed_.reset();
            for (auto& f : fly_) f.reset();
        }
        mode_ = MODE_ESTOP;
    } else if (mode_ == MODE_ESTOP) {
        mode_ = MODE_DISARMED;                    // released: the Jetson must ask for AUTO again
        t_stopped_us_ = t_us_;
    } else if (manual) {
        mode_ = MODE_MANUAL;
    } else if (mode_ == MODE_MANUAL) {
        mode_ = MODE_DISARMED;
    } else if (mode_ == MODE_DISARMED || mode_ == MODE_BOOT) {
        // re-arm only on a fresh request: a CMD that arrived after the stop
        if (want_auto && t_cmd_us_ >= t_stopped_us_) {
            mode_ = MODE_AUTO;
            ctl_.reset();
        }
    } else if (mode_ == MODE_AUTO && !want_auto) {
        // ramp to a stop first; disarm once the base has been at rest for 0.5 s
        bool still = ctl_.v_cmd.norm() < 0.01f && fabsf(ctl_.w_cmd) < 0.01f;
        if (!still) {
            t_stopped_us_ = t_us_;
        } else if (t_us_ - t_stopped_us_ > 500 * MS) {
            mode_ = MODE_DISARMED;
            t_stopped_us_ = t_us_;
        }
    }

    float wheel_cmd[prm::N_WHEELS] = {};
    bool enable = false;
    if (mode_ == MODE_AUTO) {
        run_base(dt, want_auto, in, wheel_cmd);
        enable = true;
    } else if (mode_ == MODE_MANUAL) {
        Limits L = limits();
        ctl_.set_limits(L, prm::MANUAL_SPEED);
        Vec2 vb(in.rc[1] * prm::MANUAL_SPEED, -in.rc[0] * prm::MANUAL_SPEED);
        float w = -in.rc[3] * L.max_yaw_rate * 0.5f;
        float yaw = odom_.pose.yaw;
        ctl_.velocity_step(dt, in.bumper ? Vec2() : body_to_world(vb, yaw), in.bumper ? 0.0f : w, yaw, 0.0f,
                           wheel_cmd);
        enable = true;
    } else {
        ctl_.reset();
    }
    drive_motors(enable, wheel_cmd, out);
    run_module(dt, mode_ == MODE_AUTO && want_auto, in, out);
    out.buzzer = 0;
    out.led = mode_;

    if (t_us_ >= t_state_next_) {
        t_state_next_ = t_us_ + prm::STATE_PERIOD_US;
        send_state(in);
    }
    out.dbg_v_cmd[0] = ctl_.v_cmd.x;
    out.dbg_v_cmd[1] = ctl_.v_cmd.y;
    out.dbg_w_cmd = ctl_.w_cmd;
    for (int i = 0; i < prm::N_WHEELS && i < 4; ++i) out.dbg_wheel_cmd[i] = wheel_cmd_[i];
}

}  // namespace fw

// The catch / launch module: the flywheel rpm loops (design.md 9: closed on the MCU from bidirectional-DShot eRPM,
// a port of the sim's models.FlywheelDrive MCU half) and the chute feed's one-ball-per-trigger sequence (design.md
// 22.4, a port of robot.Robot.pre_step's feed logic).
#pragma once

#include "fw_params.h"
#include "fw_types.h"

namespace fw {

class FlywheelLoop {
public:
    float w_meas = 0.0f;        // rad/s, from the last eRPM frame
    float throttle = 0.0f;
    float integ = 0.0f;

    void reset() { w_meas = throttle = integ = 0.0f; }
    void on_telemetry(float erpm) { w_meas = erpm / (float)prm::FLY_POLE_PAIRS * 2.0f * PI_F / 60.0f; }
    void step(float dt, float w_target, float v_bus);
};

enum FeedState : uint8_t { FEED_IDLE = 0, FEED_SEAT = 1, FEED_RUN = 2, FEED_DONE = 3 };

class Feed {
public:
    FeedState state = FEED_IDLE;
    float t = 0.0f;             // s since this trigger started
    bool beam_seen = false;     // the ball blocked the exit beam (then it must clear)
    float clear_t = 0.0f;       // roller pocket: s since the catch beam cleared
    float cmd = 0.0f;           // rad/s commanded (slewed)

    void reset() {
        state = FEED_IDLE;
        t = clear_t = cmd = 0.0f;
        beam_seen = false;
    }
    void trigger() {             // a new fire request that passed the interlocks
        state = prm::FEED_SEAT_TIME > 0.0f ? FEED_SEAT : FEED_RUN;
        t = clear_t = 0.0f;
        beam_seen = false;
    }
    // returns the wheel speed command (rad/s; 0 = hold)
    float step(float dt, bool beam_exit, bool beam_catch);
};

}  // namespace fw

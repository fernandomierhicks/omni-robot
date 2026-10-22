#include "module.h"

namespace fw {

static float fly_friction(float w) {
    // bearing + iron loss torque, split Coulomb / viscous, matched to I0 at 10 V (models.FlywheelDrive.friction)
    float t0 = prm::FLY_KT * prm::FLY_I0;
    if (w == 0.0f) return 0.0f;
    return copysignf(0.4f * t0 + 0.6f * t0 * fabsf(w) / prm::FLY_W10V, w);
}

void FlywheelLoop::step(float dt, float w_target, float v_bus) {
    if (w_target <= 0.0f) {
        throttle = 0.0f;
        integ = 0.0f;
        return;
    }
    if (v_bus < 5.0f) v_bus = 5.0f;
    float err = w_target - w_meas;
    // feed-forward: the throttle that holds w_target against back-EMF and drag
    float ff = (prm::FLY_KE * w_target + prm::FLY_RM * fly_friction(w_target) / prm::FLY_KT) / v_bus;
    integ = clampf(integ + prm::FLY_KI * err * dt, -0.2f, 0.2f);
    throttle = clampf(ff + prm::FLY_KP * err + integ, 0.0f, 1.0f);
}

float Feed::step(float dt, bool beam_exit, bool beam_catch) {
    float target = 0.0f;
    switch (state) {
        case FEED_IDLE:
        case FEED_DONE:
            target = 0.0f;
            break;
        case FEED_SEAT:
            // run backward first, seating the ball on the back stop, so every shot starts from the same place
            t += dt;
            target = -0.5f * prm::FEED_SPEED;
            if (t > prm::FEED_SEAT_TIME) state = FEED_RUN;
            break;
        case FEED_RUN:
            t += dt;
            target = prm::FEED_SPEED;
            if (prm::FEED_CHUTE) {
                // chute: stop once the ball has passed the exit beam (blocked, then clear), so it leaves the feed
                // wheels at full speed and the next ball waits for its own trigger
                if (beam_exit) {
                    beam_seen = true;
                } else if (beam_seen || t > prm::FEED_TIME) {
                    state = FEED_DONE;
                    target = 0.0f;
                }
            } else {
                clear_t = beam_catch ? 0.0f : clear_t + dt;
                if (clear_t > prm::FEED_CLEAR_TRAVEL / prm::FEED_LINEAR_SPEED ||
                    t > prm::FEED_TIME + prm::FEED_SEAT_TIME) {
                    state = FEED_DONE;
                    target = 0.0f;
                }
            }
            break;
    }
    // slewed both ways; exactly 0 = hold where it stopped (the motor controller's position hold)
    float step = prm::FEED_SLEW * dt;
    float d = target - cmd;
    cmd = fabsf(d) <= step ? target : cmd + clampf(d, -step, step);
    return cmd;
}

}  // namespace fw

#include "dm_motor.h"

#include <string.h>

namespace fw {

uint32_t float_to_uint(float x, float lo, float hi, int bits) {
    float span = hi - lo;
    if (x < lo) x = lo;
    if (x > hi) x = hi;
    uint32_t top = (1u << bits) - 1u;
    return (uint32_t)((x - lo) * (float)top / span + 0.5f);
}

float uint_to_float(uint32_t v, float lo, float hi, int bits) {
    uint32_t top = (1u << bits) - 1u;
    return (float)v * (hi - lo) / (float)top + lo;
}

FwCanFrame dm_speed_cmd(uint8_t motor_id, float speed) {
    FwCanFrame f{};
    f.id = DM_SPEED_BASE + motor_id;
    f.len = 4;
    memcpy(f.data, &speed, 4);           // both targets are little-endian
    return f;
}

FwCanFrame dm_special(uint8_t motor_id, uint8_t code) {
    FwCanFrame f{};
    f.id = DM_SPEED_BASE + motor_id;
    f.len = 8;
    for (int i = 0; i < 7; ++i) f.data[i] = 0xFF;
    f.data[7] = code;
    return f;
}

bool dm_decode_feedback(const FwCanFrame& f, const DmRange& r, DmFeedback& out) {
    if (f.len != 8) return false;
    const uint8_t* d = f.data;
    out.id = d[0] & 0x0F;
    out.err = (d[0] >> 4) & 0x0F;
    uint32_t p = ((uint32_t)d[1] << 8) | d[2];
    uint32_t v = ((uint32_t)d[3] << 4) | (d[4] >> 4);
    uint32_t t = ((uint32_t)(d[4] & 0x0F) << 8) | d[5];
    out.pos = uint_to_float(p, -r.p_max, r.p_max, 16);
    out.vel = uint_to_float(v, -r.v_max, r.v_max, 12);
    out.torque = uint_to_float(t, -r.t_max, r.t_max, 12);
    out.t_mos = d[6];
    out.t_rotor = d[7];
    return true;
}

FwCanFrame dm_encode_feedback(uint32_t master_id, const DmFeedback& fb, const DmRange& r) {
    FwCanFrame f{};
    f.id = master_id;
    f.len = 8;
    uint32_t p = float_to_uint(fb.pos, -r.p_max, r.p_max, 16);
    uint32_t v = float_to_uint(fb.vel, -r.v_max, r.v_max, 12);
    uint32_t t = float_to_uint(fb.torque, -r.t_max, r.t_max, 12);
    f.data[0] = (uint8_t)((fb.id & 0x0F) | (fb.err << 4));
    f.data[1] = (uint8_t)(p >> 8);
    f.data[2] = (uint8_t)(p & 0xFF);
    f.data[3] = (uint8_t)(v >> 4);
    f.data[4] = (uint8_t)(((v & 0x0F) << 4) | (t >> 8));
    f.data[5] = (uint8_t)(t & 0xFF);
    f.data[6] = fb.t_mos;
    f.data[7] = fb.t_rotor;
    return f;
}

}  // namespace fw

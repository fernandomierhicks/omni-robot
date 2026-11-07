#include "ak_motor.h"

#include <math.h>

namespace fw {

static void put_be32(uint8_t* d, int32_t v) {
    uint32_t u = (uint32_t)v;
    d[0] = (uint8_t)(u >> 24);
    d[1] = (uint8_t)(u >> 16);
    d[2] = (uint8_t)(u >> 8);
    d[3] = (uint8_t)u;
}

static int32_t get_be32(const uint8_t* d) {
    return (int32_t)(((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) | ((uint32_t)d[2] << 8) | d[3]);
}

static int16_t get_be16(const uint8_t* d) { return (int16_t)(((uint16_t)d[0] << 8) | d[1]); }

static void put_be16(uint8_t* d, int16_t v) {
    d[0] = (uint8_t)((uint16_t)v >> 8);
    d[1] = (uint8_t)((uint16_t)v & 0xFF);
}

static int32_t sat_i32(float x) {
    if (!(x == x)) return 0;                                    // NaN: nothing
    if (x > 2147483000.0f) return 2147483000;
    if (x < -2147483000.0f) return -2147483000;
    return (int32_t)lrintf(x);
}

static int16_t sat_i16(float x) {
    if (!(x == x)) return 0;
    if (x > 32767.0f) return 32767;
    if (x < -32768.0f) return -32768;
    return (int16_t)lrintf(x);
}

static FwCanFrame cmd(uint8_t id, uint8_t mode, int32_t v) {
    FwCanFrame f{};
    f.id = ((uint32_t)mode << 8) | id;
    f.ext = 1;
    f.len = 4;
    put_be32(f.data, v);
    return f;
}

FwCanFrame ak_velocity_cmd(uint8_t id, float erpm) { return cmd(id, AK_VELOCITY, sat_i32(erpm)); }
FwCanFrame ak_current_cmd(uint8_t id, float amps) { return cmd(id, AK_CURRENT, sat_i32(amps * 1000.0f)); }
FwCanFrame ak_brake_cmd(uint8_t id, float amps) { return cmd(id, AK_BRAKE, sat_i32(fabsf(amps) * 1000.0f)); }

bool ak_decode_status(const FwCanFrame& f, AkStatus& out) {
    if (!f.ext || f.len != 8 || ((f.id >> 8) & 0xFF) != AK_STATUS || (f.id >> 16) != 0) return false;
    const uint8_t* d = f.data;
    out.id = (uint8_t)(f.id & 0xFF);
    out.pos_deg = 0.1f * get_be16(d);
    out.erpm = 10.0f * get_be16(d + 2);
    out.current = 0.01f * get_be16(d + 4);
    out.temp_c = (int8_t)d[6];
    out.err = d[7];
    return true;
}

FwCanFrame ak_encode_status(const AkStatus& s) {
    FwCanFrame f{};
    f.id = ((uint32_t)AK_STATUS << 8) | s.id;
    f.ext = 1;
    f.len = 8;
    put_be16(f.data, sat_i16(s.pos_deg * 10.0f));
    put_be16(f.data + 2, sat_i16(s.erpm * 0.1f));
    put_be16(f.data + 4, sat_i16(s.current * 100.0f));
    f.data[6] = (uint8_t)s.temp_c;
    f.data[7] = s.err;
    return f;
}

bool ak_decode_command(const FwCanFrame& f, uint8_t& id, uint8_t& mode, int32_t& value) {
    if (!f.ext || f.len < 4 || (f.id >> 16) != 0) return false;
    id = (uint8_t)(f.id & 0xFF);
    mode = (uint8_t)((f.id >> 8) & 0xFF);
    value = get_be32(f.data);
    return mode <= AK_POS_VEL;
}

}  // namespace fw

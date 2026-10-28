#include "sh2.h"

namespace fw {

size_t sh2_report_len(uint8_t id) {
    switch (id) {
        case 0x01: case 0x02: case 0x03: case 0x04: case 0x06: return 10;   // accel, gyro, mag, linear accel, gravity
        case 0x05: case 0x09: case 0x28: case 0x2A: return 14;              // rotation vectors with accuracy
        case 0x08: case 0x29: return 12;                                    // game rotation vectors
        case 0x07: case 0x0F: case 0x14: case 0x15: case 0x16: case 0x1E: return 16;   // uncalibrated, raw, activity
        case 0x0A: case 0x0B: case 0x18: return 8;
        case 0x11: return 12;
        case 0x10: case 0xFA: case 0xFB: return 5;
        case 0x0C: case 0x0D: case 0x0E: case 0x12: case 0x13: case 0x19: case 0x1A: case 0x1B: case 0x1C:
        case 0x1F: case 0x20: case 0x21: case 0x22: case 0x23: case 0x2B: return 6;
        default: return 0;
    }
}

static int16_t i16(const uint8_t* p) { return (int16_t)(p[0] | (p[1] << 8)); }
static int32_t i32(const uint8_t* p) {
    return (int32_t)((uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24);
}

size_t sh2_parse(const uint8_t* p, size_t n, uint64_t t_int_us, Sh2Report* out, size_t cap) {
    int64_t ref = 0;             // 100 us ticks relative to t_int
    size_t k = 0, i = 0;
    while (i < n && k < cap) {
        size_t len = sh2_report_len(p[i]);
        if (len == 0 || i + len > n) break;
        const uint8_t* r = p + i;
        if (r[0] == SH2_BASE_TS) {
            ref = -(int64_t)i32(r + 1);
        } else if (r[0] == SH2_REBASE) {
            ref += i32(r + 1);
        } else if (r[0] == SH2_ACCEL || r[0] == SH2_GYRO || r[0] == SH2_GAME_RV) {
            uint16_t delay = (uint16_t)(((r[2] & 0xFC) << 6) | r[3]);
            int64_t t = (int64_t)t_int_us + (ref + delay) * 100;
            Sh2Report& o = out[k++];
            o.id = r[0];
            o.seq = r[1];
            o.status = r[2] & 0x03;
            o.t_us = t > 0 ? (uint64_t)t : 0;
            if (r[0] == SH2_GAME_RV) {
                const float q = 1.0f / 16384.0f;
                o.v[0] = i16(r + 10) * q;        // real
                o.v[1] = i16(r + 4) * q;         // i
                o.v[2] = i16(r + 6) * q;         // j
                o.v[3] = i16(r + 8) * q;         // k
            } else {
                const float q = r[0] == SH2_GYRO ? 1.0f / 512.0f : 1.0f / 256.0f;
                for (int a = 0; a < 3; ++a) o.v[a] = i16(r + 4 + 2 * a) * q;
                o.v[3] = 0.0f;
            }
        }
        i += len;
    }
    return k;
}

size_t sh2_samples(const Sh2Report* r, size_t n, FwImuSample& last, FwImuSample* out, size_t cap) {
    size_t m = 0;
    for (size_t i = 0; i < n && m < cap; ++i) {
        if (r[i].id != SH2_GYRO) continue;
        FwImuSample s = last;
        s.t_us = r[i].t_us;
        for (int a = 0; a < 3; ++a) s.gyro[a] = r[i].v[a];
        // the accelerometer and game RV of this batch nearest in time to this gyro sample
        const Sh2Report* best[2] = {nullptr, nullptr};
        for (size_t j = 0; j < n; ++j) {
            int slot = r[j].id == SH2_ACCEL ? 0 : (r[j].id == SH2_GAME_RV ? 1 : -1);
            if (slot < 0) continue;
            uint64_t dj = r[j].t_us > s.t_us ? r[j].t_us - s.t_us : s.t_us - r[j].t_us;
            if (!best[slot]) {
                best[slot] = &r[j];
                continue;
            }
            uint64_t db = best[slot]->t_us > s.t_us ? best[slot]->t_us - s.t_us : s.t_us - best[slot]->t_us;
            if (dj < db) best[slot] = &r[j];
        }
        if (best[0])
            for (int a = 0; a < 3; ++a) s.accel[a] = best[0]->v[a];
        if (best[1])
            for (int a = 0; a < 4; ++a) s.quat[a] = best[1]->v[a];
        out[m++] = s;
        last = s;
    }
    return m;
}

}  // namespace fw

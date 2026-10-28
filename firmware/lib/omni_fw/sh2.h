// The BNO085's sensor reports, with each sample's own time (design.md 10). Board-free: the Teensy's SPI driver reads
// SHTP packets and calls this; the SIL tests it on packets built in Python.
//
// SH-2 (CEVA SH-2 Reference Manual 1000-3625, sections 6.5 and 7.2; CEVA's sh2.c computes the same): a channel-3
// payload (input reports) is a batch the hub queued since it last raised H_INTN:
//   0xFB  base timestamp reference: int32 base delta, 100 us ticks, relative to the host interrupt (H_INTN asserted)
//   0xFA  timestamp rebase: int32 delta, 100 us ticks, added to the base for the reports after it
//   each sensor report: id, sequence, status (bits 7:2 = delay[13:8]), delay[7:0], then its values
// A report's sample time = t_int + (-base + rebase + delay) * 100 us, where t_int is when H_INTN fell (the board's
// interrupt stamps it). Resolution 100 us; the hub's own sampling-to-report latency is inside the delay.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hw_io.h"

namespace fw {

enum Sh2Id : uint8_t {
    SH2_ACCEL = 0x01,            // m/s^2, Q8 (with gravity)
    SH2_GYRO = 0x02,             // calibrated, rad/s, Q9
    SH2_GAME_RV = 0x08,          // game rotation vector, Q14: i, j, k, real
    SH2_REBASE = 0xFA,
    SH2_BASE_TS = 0xFB,
};

struct Sh2Report {
    uint8_t id;
    uint8_t seq;
    uint8_t status;              // bits 1:0 accuracy
    uint64_t t_us;               // sample time, MCU clock
    float v[4];                  // accel / gyro: x y z; game RV: w x y z
};

// the length of a report by id (0 = unknown: the rest of the batch can't be walked)
size_t sh2_report_len(uint8_t id);

// one channel-3 payload (without the 4-byte SHTP header) -> reports with times; returns how many were written
size_t sh2_parse(const uint8_t* p, size_t n, uint64_t t_int_us, Sh2Report* out, size_t cap);

// reports -> IMU samples, one per gyro report, in time order; accelerometer and game RV come from the same batch
// (the nearest in time) or, if the batch had none, the last ones seen (kept in last)
size_t sh2_samples(const Sh2Report* r, size_t n, FwImuSample& last, FwImuSample* out, size_t cap);

}  // namespace fw

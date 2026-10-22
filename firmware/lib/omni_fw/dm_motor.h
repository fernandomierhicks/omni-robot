// Damiao DM-series motor over CAN (design.md 24: the DM-2325 drive), velocity mode.
//
// From Damiao's published protocol (DM-J4310 manual; the DM-2325 is the same driver family, VERIFY on the bench):
//   command   id 0x200 + motor id, 4 bytes: speed setpoint, float32 little-endian, rad/s at the output
//   enable    id 0x200 + motor id, FF FF FF FF FF FF FF FC   (disable ... FD, clear error ... FB)
//   feedback  id = the motor's master id, one reply per command:
//             D0 = id | err << 4, D1-2 position (16 bit), D3 + D4 high nibble velocity (12 bit),
//             D4 low nibble + D5 torque (12 bit), D6 MOS temperature, D7 rotor temperature (deg C)
//             each mapped linearly onto [-PMAX, PMAX], [-VMAX, VMAX], [-TMAX, TMAX], as set in the motor (VERIFY:
//             the ranges below are what fw_params.h sets; they must match the values flashed with Damiao's tool)
//   err       0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current, B MOS over-temperature,
//             C coil over-temperature, D communication lost, E overload
#pragma once

#include <stdint.h>

#include "hw_io.h"

namespace fw {

struct DmFeedback {
    uint8_t id = 0;
    uint8_t err = 0;
    float pos = 0.0f;       // rad, wraps at +/-PMAX
    float vel = 0.0f;       // rad/s
    float torque = 0.0f;    // N m
    uint8_t t_mos = 0, t_rotor = 0;
};

struct DmRange {
    float p_max, v_max, t_max;
};

enum DmErr : uint8_t {
    DM_DISABLED = 0x0, DM_ENABLED = 0x1, DM_OVERVOLT = 0x8, DM_UNDERVOLT = 0x9, DM_OVERCURRENT = 0xA,
    DM_MOS_HOT = 0xB, DM_COIL_HOT = 0xC, DM_COMM_LOST = 0xD, DM_OVERLOAD = 0xE,
};

constexpr uint32_t DM_SPEED_BASE = 0x200;

FwCanFrame dm_speed_cmd(uint8_t motor_id, float speed);
FwCanFrame dm_special(uint8_t motor_id, uint8_t code);         // 0xFC enable, 0xFD disable, 0xFB clear error
inline FwCanFrame dm_enable(uint8_t id) { return dm_special(id, 0xFC); }
inline FwCanFrame dm_disable(uint8_t id) { return dm_special(id, 0xFD); }
inline FwCanFrame dm_clear(uint8_t id) { return dm_special(id, 0xFB); }

bool dm_decode_feedback(const FwCanFrame& f, const DmRange& r, DmFeedback& out);
// the inverse (the sim's motor model replies with it; also a unit test of the codec)
FwCanFrame dm_encode_feedback(uint32_t master_id, const DmFeedback& fb, const DmRange& r);

uint32_t float_to_uint(float x, float lo, float hi, int bits);
float uint_to_float(uint32_t v, float lo, float hi, int bits);

}  // namespace fw

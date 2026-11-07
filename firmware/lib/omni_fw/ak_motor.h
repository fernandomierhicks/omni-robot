// CubeMars AK-series motor over CAN, SERVO mode (the arm robot's AK40-10 V3.0 KV170 drives), velocity loop.
//
// Why servo mode, not MIT mode (Nov 7; design.md 2.1):
//   - the wheels want a speed; servo mode's velocity loop runs inside the motor (its own speed PI), commanded in
//     electrical rpm, a unit fixed by the motor (pole pairs x gearbox), not by a per-model range table;
//   - MIT mode packs every field into a per-model range (P, V, T, Kp, Kd). Getting those constants wrong is silent:
//     the motor decodes with its own, we with ours, and everything scales with no error anywhere (the wheeled-leg
//     robot's AK45-10 hips ran 2.5x off for weeks: its HipMotors README). The AK40-10's MIT table couldn't be checked
//     here (the CubeMars manual's host is outside this environment's network);
//   - MIT's impedance law (Kp, Kd, torque feed-forward) is for legs and arms, not wheels.
//   The price: servo mode must be selected in the CubeMars tool (it persists in the motor), it answers only extended
//   frames, and its feedback is a periodic broadcast, not a reply per command (below).
//
// Protocol (CubeMars AK series driver manual, servo mode; layouts as the wheeled-leg robot's tested
// ServoMode_move_motor.py uses them on its AK45-10; the manual itself VERIFY against the AK40-10's V3.0 manual):
//   command   EXTENDED id = (mode << 8) | controller id, big-endian payload:
//               0 duty       int32  duty x 100000
//               1 current    int32  A x 1000           (q-axis current; 0 = free, the rotor coasts)
//               2 brake      int32  A x 1000           (current brake: holds against motion with this current)
//               3 velocity   int32  electrical rpm     (ERPM = output rpm x gear ratio x pole pairs)
//               4 position   int32  deg x 10000
//               5 set origin uint8  0 temporary, 1 permanent (writes the motor's flash: calibration only, never at
//                                   start: the wheeled-leg robot's "zeroing persists across power cycles")
//               6 pos + vel  int32 deg x 10000, int16 ERPM / 10, int16 ERPM/s / 10
//   status    EXTENDED id = (0x29 << 8) | controller id, 8 bytes big-endian, broadcast at the rate set in the tool
//             (send_can_status on: it is OFF at the factory, then no feedback at all; 500 Hz on the robot, VERIFY):
//               int16 position deg x 10, int16 speed ERPM / 10, int16 current A x 100, int8 temperature degC,
//               uint8 error (0 none, 1 motor over-temperature, 2 over-current, 3 over-voltage, 4 under-voltage,
//               5 encoder fault, 6 MOSFET over-temperature, 7 stall)
//   timeout   the motor's own (VESC app setting timeout_msec, 1000 ms at the factory, then timeout_brake_current,
//             0 A = coast): set it to ~100 ms and a few A of brake in the tool (bring-up check "link")
#pragma once

#include <stdint.h>

#include "hw_io.h"

namespace fw {

enum AkMode : uint8_t {
    AK_DUTY = 0, AK_CURRENT = 1, AK_BRAKE = 2, AK_VELOCITY = 3, AK_POSITION = 4, AK_ORIGIN = 5, AK_POS_VEL = 6,
};
constexpr uint8_t AK_STATUS = 0x29;

enum AkErr : uint8_t {
    AK_OK = 0, AK_MOTOR_HOT = 1, AK_OVERCURRENT = 2, AK_OVERVOLT = 3, AK_UNDERVOLT = 4, AK_ENCODER = 5,
    AK_MOS_HOT = 6, AK_STALL = 7,
};

struct AkStatus {
    uint8_t id = 0;
    float pos_deg = 0.0f;      // output shaft, deg (wraps at the int16's +/-3276.8 deg: AK_POS_WRAP_DEG, VERIFY)
    float erpm = 0.0f;         // electrical rpm
    float current = 0.0f;      // A
    int8_t temp_c = 0;
    uint8_t err = AK_OK;
};

constexpr float AK_POS_WRAP_DEG = 3276.8f;     // the status position's int16 x 0.1 deg range (half span)

FwCanFrame ak_velocity_cmd(uint8_t id, float erpm);
FwCanFrame ak_current_cmd(uint8_t id, float amps);
FwCanFrame ak_brake_cmd(uint8_t id, float amps);
bool ak_decode_status(const FwCanFrame& f, AkStatus& out);
FwCanFrame ak_encode_status(const AkStatus& s);           // the sim's motor and the tests
bool ak_decode_command(const FwCanFrame& f, uint8_t& id, uint8_t& mode, int32_t& value);   // the sim's motor

// output shaft rad/s <-> electrical rpm
inline float ak_erpm(float w, float gear, float pole_pairs) { return w * 9.5492966f * gear * pole_pairs; }
inline float ak_rad_s(float erpm, float gear, float pole_pairs) { return erpm / (9.5492966f * gear * pole_pairs); }

}  // namespace fw

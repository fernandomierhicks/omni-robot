// The Jetson <-> MCU link (design.md 5: a small custom protocol, not micro-ROS; design.md 30). USB serial on the
// Teensy (native USB, 480 Mbit/s), a TCP socket in the sim. Python mirror: ros/src/omni_mcu/omni_mcu/protocol.py
// (test_v11.py checks the two against each other through the SIL library).
//
// Frame:   COBS( payload | crc16 ) 0x00         crc16 = CRC-16/CCITT-FALSE over the payload, little-endian
// Payload: type u8 | seq u8 | body (packed, little-endian)
//
//   CMD   0x01  Jetson -> MCU, 100 Hz (it is also the heartbeat: none for CMD_TIMEOUT -> ramped stop)
//   STATE 0x81  MCU -> Jetson, 200 Hz: odometry, module, beams, mode, faults, time-sync echo
//   IMU   0x82  MCU -> Jetson, every BNO085 report (400 Hz), forwarded for robot_localization
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace fw {

enum MsgType : uint8_t { MSG_CMD = 0x01, MSG_STATE = 0x81, MSG_IMU = 0x82 };

enum ReqMode : uint8_t { REQ_DISARM = 0, REQ_AUTO = 1 };
enum Profile : uint8_t { PROFILE_ADULT = 0, PROFILE_KID = 1 };
enum DriveMode : uint8_t { DRIVE_IDLE = 0, DRIVE_VEL_BODY = 1, DRIVE_GOTO = 2 };
enum CmdFlags : uint8_t {
    CMD_HOLD_YAW = 1,            // VEL_BODY: with wz = 0, hold the heading (teleop)
};

#pragma pack(push, 1)
struct CmdBody {
    uint64_t t_host_ns;          // the Jetson's clock when sent (time sync)
    uint8_t req_mode;            // ReqMode
    uint8_t profile;             // Profile
    uint8_t drive_mode;          // DriveMode
    uint8_t flags;               // CmdFlags
    float a, b, c;               // VEL_BODY: vx, vy, wz (body). GOTO: x, y, yaw (odom frame)
    float v_end;                 // GOTO: pass through the target at this speed (moving catch); 0 = stop there
    float a_cap;                 // extra acceleration cap (m/s^2), 0 = none
    float speed_cap;             // m/s, 0 = the profile's
    uint64_t t_brake_us;         // GOTO: at this MCU time stop tracking the target: hold the commanded velocity for
    float coast_s;               //   coast_s, then brake at a_brake to rest and hold there (the catch: design.md 22)
    float a_brake;               //   (0 = no schedule)
    float tilt;                  // rad, module tilt target
    uint64_t t_tilt_us;          // at this MCU time switch the tilt to tilt_after (the dump after the catch); 0 = none
    float tilt_after;
    float fly_rpm;               // flywheel target, mechanical rpm (0 = off)
    uint16_t fire_seq;           // a new value fires one ball (if the interlocks pass)
};

struct StateBody {
    uint64_t t_mcu_us;           // when this state was sampled
    uint64_t echo_t_host_ns;     // the last CMD's t_host_ns ...
    uint64_t echo_rx_us;         // ... and the MCU time it arrived
    uint8_t mode;                // McuMode (firmware.h)
    uint8_t profile;
    uint8_t drive_mode;          // what the base is doing (DriveMode, or 3 = coasting / braking after a catch)
    uint8_t feed_state;          // FeedState
    uint16_t faults;             // Fault bits (firmware.h)
    uint8_t dm_err[4];           // each drive motor's last error code (0 disabled, 1 enabled, ...)
    float x, y, yaw;             // odometry pose, odom frame
    float vx, vy, wz;            // body twist (wheels; wz from the IMU when it is there)
    float slip;                  // m/s, wheel disagreement
    float roll, pitch;           // rad, from the BNO085
    float v_cmd_x, v_cmd_y, w_cmd;   // the controller's command, odom frame
    float fly_rpm[2];            // measured
    float fly_target_rpm;
    float tilt, tilt_cmd;        // rad: measured (servo feedback), commanded
    uint8_t beams;               // bit0 catch beam, bit1 feed-exit beam
    uint8_t fly_ready;
    uint16_t fire_seq_done;      // the last fire_seq acted on
    uint16_t fire_rejected;      // fire requests refused by the interlocks (count)
    float v_bus;
    uint16_t cmd_count;          // CMDs received (mod 2^16)
    uint16_t rx_errors;          // frames dropped (CRC, length)
};

struct ImuBody {
    uint64_t t_mcu_us;
    float quat[4];               // w x y z, game rotation vector
    float gyro[3];
    float accel[3];
};
#pragma pack(pop)

static_assert(sizeof(CmdBody) == 74, "CmdBody layout changed: update omni_mcu/protocol.py");
static_assert(sizeof(StateBody) == 116, "StateBody layout changed: update omni_mcu/protocol.py");
static_assert(sizeof(ImuBody) == 48, "ImuBody layout changed: update omni_mcu/protocol.py");

uint16_t crc16(const uint8_t* d, size_t n);
// payload (type, seq, body) -> a COBS frame ending in 0x00; returns its length (0 if out does not fit)
size_t frame_encode(uint8_t type, uint8_t seq, const void* body, size_t body_len, uint8_t* out, size_t out_cap);

// Streaming decoder: feed bytes, get whole payloads
class FrameDecoder {
public:
    // returns true when a frame completed: type, body (in body_out), its length
    bool push(uint8_t byte, uint8_t& type, uint8_t* body_out, size_t body_cap, size_t& body_len);
    uint32_t errors = 0;

private:
    uint8_t buf_[256];
    size_t n_ = 0;
    bool overflow_ = false;
};

}  // namespace fw

// omni-robot MCU, Teensy 4.1: the hardware layer around the firmware core (design.md 4, 30).
//
// Every 1 ms: read the peripherals into FwHwInputs, Firmware::tick(), drive the peripherals from FwHwOutputs. The
// robot computer's bytes come and go on the native USB serial port between ticks. The core (lib/omni_fw) is the code the sim
// runs in the loop; only this file is Teensy-specific.
//
// Drivers here, and how far each is done:
//   CAN1, DM-2325 x4          FlexCAN_T4, 1 Mbit/s: complete (the protocol is in the core: dm_motor.cpp)
//   USB serial, host link   complete (framing in the core: protocol.cpp)
//   RoboClaw, feed motors     packet serial, speed commands + speed reads: written, VERIFY on the bench
//   STS3250, tilt servo       Feetech half-duplex bus, goal position + position reads: written, VERIFY on the bench
//   break-beams, e-stop, bumpers, battery ADC, buzzer, LED       complete (thresholds and divider ratio VERIFY)
//   BNO085 (SH-2 over SPI)    the wheeled-leg robot's proven driver ported (Nov 17): CEVA's SH-2 through the vendored,
//                             patched Adafruit_BNO08x (lib/Adafruit_BNO08x), PS0/WAKE and PS1 strapped high on the
//                             module, so reset-as-wake handshakes; game RV + calibrated gyro + accelerometer at 100 Hz,
//                             each sample's time from the INT edge and the SH-2 delays; stalls and sensor resets
//                             recovered without a reboot. Its test_imu passed on this Teensy and module (Nov 17)
//   time references           written, VERIFY: SYNC_OUT and CAM_TRIG on one-shot PIT timers at the core's scheduled
//                             MCU times, the fired time read in the timer's interrupt; the left camera's STROBE on a pin
//                             interrupt (both edges). CAN replies stamped by FlexCAN's own timer (design.md 10)
//   bidirectional DShot       TODO: stub, no output or telemetry yet (the core's rpm loop is ready for it)
//   ELRS / CRSF               TODO: stub, rc_ok = 0 (MANUAL mode unreachable until written)
//   watchdog (WDOG1)          written, VERIFY on the bench: a loop that stops feeding it for 0.5 s resets the MCU,
//                             which boots DISARMED
//
// Build variants (platformio.ini): env arm (the default, fw_params_arm.h: CubeMars AK40-10 drives in servo mode,
// ak_motor.h), env arm_bench (the same on a bare bench Teensy: the e-stop input ignored, a fixed 24 V reported; never
// on a robot) and env catch (fw_params.h: Damiao DM-2325, dm_motor.h). The core picks the protocol (DRIVE_DAMIAO); this
// layer only carries each frame's id kind (CubeMars servo mode answers extended frames only). CubeMars bench notes from
// the wheeled-leg robot: CAN needs a common ground with the motor (its UART connector's GND), and servo mode, the
// status broadcast (send_can_status, off at the factory) and the command timeout are set in the CubeMars tool.
#include <Adafruit_BNO08x.h>                 // CEVA's SH-2 (its sh2.h, not lib/omni_fw's SIL parser of the same name)
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

#include "board_teensy41.h"
#include "firmware.h"

using namespace board;

static fw::Firmware g_fw;
static FwHwInputs g_in;
static FwHwOutputs g_out;
static FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// -- hardware constants (VERIFY at assembly) ----------------------------------------------------------------------
static constexpr float V_BUS_DIVIDER = 11.0f;          // 100k / 10k divider on A17
static constexpr float ADC_REF = 3.3f;
static constexpr float FEED_COUNTS_PER_REV = 464.64f;  // Pololu 25D 9.7:1, 48 CPR encoder at the output
static constexpr int FEED_DIR[2] = {1, -1};            // the two feed wheels counter-rotate
static constexpr uint8_t ROBOCLAW_ADDR = 0x80;
static constexpr uint8_t SERVO_ID = 1;
static constexpr int TILT_DIR = 1;                     // servo steps per +tilt
static constexpr int TILT_ZERO = 2048;                 // servo position at a level module

// -- watchdog ------------------------------------------------------------------------------------------------------
// WDOG1, 0.5 s: (WT + 1) x 0.5 s. A hung loop resets the MCU; after the reset it boots DISARMED and the drive motors'
// own CAN timeouts stop them (VERIFY both on the bench: the reset fires; the motors' comm-loss timeout is set).
static void watchdog_start() {
    CCM_CCGR3 |= CCM_CCGR3_WDOG1(CCM_CCGR_ON);
    WDOG1_WMCR = 0;                                                         // no power-down counter
    WDOG1_WCR = WDOG_WCR_WDA | WDOG_WCR_SRS | WDOG_WCR_WT(0) | WDOG_WCR_WDE;
}

static inline void watchdog_feed() {
    WDOG1_WSR = 0x5555;
    WDOG1_WSR = 0xAAAA;
}

// -- 64-bit microseconds ----------------------------------------------------------------------------------------
// micros64() runs in the loop only; an interrupt keeps micros() (32 bits, wraps every 71 min) and the loop extends it
static uint64_t micros64() {
    static uint32_t last = 0;
    static uint64_t hi = 0;
    uint32_t now = micros();
    if (now < last) hi += (1ull << 32);
    last = now;
    return hi | now;
}

static uint64_t extend_us(uint32_t t32, uint64_t now64) { return now64 - (uint32_t)((uint32_t)now64 - t32); }

// -- CAN reply stamps (design.md 10) ------------------------------------------------------------------------------
// FlexCAN copies its free-running 16-bit timer into each mailbox as a frame is received (CAN_message_t::timestamp).
// The timer counts CAN bit times (1 us at 1 Mbit/s) and wraps every 65.5 ms; read next to micros64() it turns each
// frame's stamp into MCU microseconds. VERIFY on the bench: the capture point (the i.MX RT1060 manual: the start of
// the identifier field) and that the timer runs at the bit rate. The core wants the end of the frame: + its length.
static constexpr uint32_t CAN1_BASE = 0x401D0000;               // FlexCAN1 (FlexCAN_T4's CAN1)
static constexpr uint32_t DM_REPLY_US = 124;                    // an 8-byte reply on the wire (est, with stuffing)
static constexpr uint32_t EXT_REPLY_US = 150;                   // ... an extended one (29-bit id: CubeMars, est)
static inline uint16_t can1_timer() { return (uint16_t)(*(volatile uint32_t*)(CAN1_BASE + 0x8)); }

// -- RoboClaw packet serial (Serial3) -----------------------------------------------------------------------------
static uint16_t crc16_rc(const uint8_t* d, size_t n) {          // RoboClaw: CRC-16/XMODEM (poly 0x1021, init 0)
    uint16_t crc = 0;
    for (size_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; ++b) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

struct RoboClaw {
    uint8_t rx[16];
    size_t n = 0;
    int pending = -1;                   // which motor's speed read is in flight
    uint32_t t_sent = 0;
    float speed[2] = {0, 0};
    bool fresh = false;

    void speed_cmd(int motor, float rad_s) {
        int32_t qpps = (int32_t)lroundf(rad_s * FEED_DIR[motor] * FEED_COUNTS_PER_REV / (2.0f * PI));
        uint8_t p[8] = {ROBOCLAW_ADDR, (uint8_t)(motor == 0 ? 35 : 36), (uint8_t)(qpps >> 24), (uint8_t)(qpps >> 16),
                        (uint8_t)(qpps >> 8), (uint8_t)qpps};
        uint16_t c = crc16_rc(p, 6);
        p[6] = c >> 8;
        p[7] = c & 0xFF;
        Serial3.write(p, 8);
    }
    void request_speed(int motor) {
        uint8_t p[2] = {ROBOCLAW_ADDR, (uint8_t)(motor == 0 ? 18 : 19)};
        Serial3.write(p, 2);
        pending = motor;
        n = 0;
        t_sent = millis();
    }
    // replies: 0xFF for each write; speed reads: 4 bytes speed, status, CRC (over the request and the reply)
    void poll() {
        while (Serial3.available()) {
            uint8_t b = Serial3.read();
            if (pending < 0) continue;                           // write acks
            if (n == 0 && b == 0xFF) continue;
            rx[n++] = b;
            if (n == 7) {
                uint8_t all[9] = {ROBOCLAW_ADDR, (uint8_t)(pending == 0 ? 18 : 19)};
                memcpy(all + 2, rx, 5);
                if (crc16_rc(all, 7) == (uint16_t)((rx[5] << 8) | rx[6])) {
                    int32_t q = (int32_t)((uint32_t)rx[0] << 24 | (uint32_t)rx[1] << 16 | (uint32_t)rx[2] << 8 | rx[3]);
                    if (rx[4]) q = -q;                           // status bit: direction
                    speed[pending] = (float)q * 2.0f * PI / FEED_COUNTS_PER_REV * FEED_DIR[pending];
                    fresh = true;
                }
                pending = -1;
                n = 0;
            }
        }
        if (pending >= 0 && millis() - t_sent > 5) pending = -1;  // no reply: try again later
    }
};
static RoboClaw g_rc;

// -- Feetech STS3250 bus (Serial4, half duplex) -----------------------------------------------------------------------
struct StsServo {
    uint8_t rx[16];
    size_t n = 0;
    bool waiting = false;
    uint32_t t_sent = 0;
    float pos = 0.0f;
    bool fresh = false;

    static uint8_t chk(const uint8_t* p, size_t n) {
        uint8_t s = 0;
        for (size_t i = 2; i < n; ++i) s += p[i];
        return (uint8_t)~s;
    }
    void goal(float tilt_rad) {
        int steps = TILT_ZERO + (int)lroundf(TILT_DIR * tilt_rad / (2.0f * PI) * 4096.0f);
        steps = constrain(steps, 0, 4095);
        uint8_t p[9] = {0xFF, 0xFF, SERVO_ID, 5, 0x03, 0x2A, (uint8_t)(steps & 0xFF), (uint8_t)(steps >> 8), 0};
        p[8] = chk(p, 8);
        Serial4.write(p, 9);
    }
    void torque(bool on) {
        uint8_t p[8] = {0xFF, 0xFF, SERVO_ID, 4, 0x03, 0x28, (uint8_t)(on ? 1 : 0), 0};
        p[7] = chk(p, 7);
        Serial4.write(p, 8);
    }
    void request_pos() {
        uint8_t p[8] = {0xFF, 0xFF, SERVO_ID, 4, 0x02, 0x38, 2, 0};
        p[7] = chk(p, 7);
        Serial4.write(p, 8);
        waiting = true;
        n = 0;
        t_sent = millis();
    }
    void poll() {
        while (Serial4.available()) {
            uint8_t b = Serial4.read();
            if (!waiting) continue;
            if (n < 2 && b != 0xFF) {
                n = 0;
                continue;
            }
            rx[n++] = b;
            if (n == 8) {                                         // FF FF ID LEN ERR P_lo P_hi CHK
                if (rx[2] == SERVO_ID && chk(rx, 7) == rx[7]) {
                    int steps = rx[5] | (rx[6] << 8);
                    pos = (float)(steps - TILT_ZERO) / 4096.0f * 2.0f * PI * TILT_DIR;
                    fresh = true;
                }
                waiting = false;
                n = 0;
            }
        }
        if (waiting && millis() - t_sent > 3) waiting = false;
    }
};
static StsServo g_servo;

// -- BNO085 over SPI: the wheeled-leg robot's driver (its lib/IMU), ported --------------------------------------------
// CEVA's SH-2 library through the vendored Adafruit_BNO08x (lib/Adafruit_BNO08x: bounded SPI handshakes, no hidden
// resets, a report FIFO; plus our patch that stamps each transfer at the INT edge). SPI0 mode 3 at 3 MHz, CS 10, INT 9,
// RST 6. PS0/WAKE and PS1 are strapped high on the module (SPI), so the host can't wake the hub before a write: like the
// wheeled-leg robot, every configuration is sent right after a reset (the boot wake window) or on an interrupt the
// sensor raised (it is awake then). The required reports (game rotation vector, calibrated gyro) go first; the
// accelerometer is staged on the next interrupt.
//   boot (setup, before the watchdog: begin_SPI's product-id exchange takes ~0.5 s): up to 2 attempts; if both fail the
//     IMU stays off (the core reports IMU_SILENT) until the MCU restarts
//   runtime: streams silent for STREAM_TIMEOUT_MS or a sensor reset -> GRACE (its own retry) -> a reset pulse (not
//     blocking: RESET_LOW) -> WAIT_RESET -> required reports -> CONFIG_AUX -> WAIT_DATA -> NOMINAL; a failed attempt
//     retries every RETRY_MS. While it recovers no samples go out, so the core's IMU_SILENT shows it (50 ms)
// Samples: one per gyro report, with the newest game RV and accelerometer, at the gyro's own sample time (SH-2: the INT
// edge less the report's delay). The game RV's yaw restarts at 0 after a sensor reset; the driver turns it back to the
// last yaw it sent so heading stays continuous (any turn during the gap is lost from the IMU's yaw).
struct Bno085 {
    static constexpr uint32_t RATE_HZ = 100;              // each report (heading and the EKF need no more)
    static constexpr uint32_t STREAM_TIMEOUT_MS = 100;
    static constexpr uint32_t GRACE_MS = 20;              // the BNO's own retry of an unserviced transfer is ~10 ms
    static constexpr uint32_t RESET_LOW_MS = 10;
    static constexpr uint32_t WAKE_TIMEOUT_MS = 500;      // reset notice / first interrupt / first data
    static constexpr uint32_t RETRY_MS = 1000;
    static constexpr uint8_t MAX_DRAIN = 16;
    enum class Phase : uint8_t { OFF, NOMINAL, GRACE, RESET_LOW, WAIT_RESET, CONFIG_AUX, WAIT_DATA, RETRY_DELAY };

    Adafruit_BNO08x dev{(int8_t)pin::BNO_RST};
    Phase phase = Phase::OFF;
    uint32_t t_phase = 0;
    uint32_t last_rv_ms = 0, last_gyro_ms = 0;
    bool have_quat = false;
    float quat[4] = {1, 0, 0, 0};                         // w x y z, yaw-corrected across sensor resets
    float accel[3] = {0, 0, 9.81f};
    float yaw_off = 0, last_out_yaw = 0;
    bool restitch = false, any_out = false;
    struct {
        uint32_t boot_attempts, sensor_resets, recovery_attempts, recoveries, stalls, failures, dropped, invalid;
    } diag{};

    bool int_low() const { return digitalReadFast(pin::BNO_INT) == LOW; }
    void enter(Phase p) { phase = p; t_phase = millis(); }

    bool enable(sh2_SensorId_t id, uint32_t hz) { return dev.enableReport(id, hz ? 1000000UL / hz : 0, true); }
    bool enable_required() {
        return enable(SH2_GAME_ROTATION_VECTOR, RATE_HZ) && enable(SH2_GYROSCOPE_CALIBRATED, RATE_HZ);
    }

    void clear_fresh() {
        last_rv_ms = last_gyro_ms = 0;
        have_quat = false;
    }

    // setup(): blocking, before the watchdog starts
    bool begin() {
        pinMode(pin::BNO_CS, OUTPUT);
        digitalWriteFast(pin::BNO_CS, HIGH);
        // PS0/WAKE + PS1 high at reset = SPI; BOOTN high = its application. Strapped on the bench module (then these
        // pins are unconnected or at the same level); routed here on the carrier (pcb/v1 J4)
        for (uint8_t p : {pin::BNO_WAKE, pin::BNO_PS1, pin::BNO_BOOTN}) {
            pinMode(p, OUTPUT);
            digitalWriteFast(p, HIGH);
        }
        for (int a = 0; a < 2; ++a) {
            diag.boot_attempts++;
            if (!dev.begin_SPI(pin::BNO_CS, pin::BNO_INT)) continue;
            // reset as wake: service one boot transfer, then configure before the hub can sleep again
            dev.hardwareReset();
            const uint32_t t0 = millis();
            while (!int_low() && millis() - t0 <= WAKE_TIMEOUT_MS) {}
            if (!int_low()) { sh2_close(); continue; }
            sh2_SensorValue_t ignored;
            (void)dev.getSensorEvent(&ignored);
            (void)dev.wasReset();
            dev.clearSensorEvents();
            clear_fresh();
            if (!enable_required()) { sh2_close(); continue; }
            enter(Phase::CONFIG_AUX);
            return true;
        }
        enter(Phase::OFF);
        return false;
    }

    void begin_reset() {
        diag.recovery_attempts++;
        clear_fresh();
        restitch = true;
        dev.clearSensorEvents();
        (void)dev.wasReset();
        pinMode(pin::BNO_RST, OUTPUT);
        digitalWriteFast(pin::BNO_RST, LOW);
        enter(Phase::RESET_LOW);
    }

    void fail() {
        diag.failures++;
        enter(Phase::RETRY_DELAY);
    }

    static float yaw_of(const float* q) {
        return atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
    }

    void on_rotation(float qi, float qj, float qk, float qr, uint32_t now) {
        if (!isfinite(qi) || !isfinite(qj) || !isfinite(qk) || !isfinite(qr)) { diag.invalid++; return; }
        const float n2 = qi * qi + qj * qj + qk * qk + qr * qr;
        if (!(n2 > 0.25f && n2 < 2.25f)) { diag.invalid++; return; }
        const float k = 1.0f / sqrtf(n2);
        const float q[4] = {qr * k, qi * k, qj * k, qk * k};
        if (restitch) {                                   // the first rotation after a reset: keep the heading
            if (any_out) yaw_off = last_out_yaw - yaw_of(q);
            restitch = false;
        }
        const float c = cosf(0.5f * yaw_off), s = sinf(0.5f * yaw_off);    // a turn about the world's z, first
        quat[0] = c * q[0] - s * q[3];
        quat[1] = c * q[1] - s * q[2];
        quat[2] = c * q[2] + s * q[1];
        quat[3] = c * q[3] + s * q[0];
        have_quat = true;
        last_rv_ms = now;
    }

    void handle(const sh2_SensorValue_t& v, FwImuSample* out, uint32_t cap, uint32_t& n, uint64_t now64, uint32_t now) {
        if (v.sensorId == SH2_GAME_ROTATION_VECTOR) {
            on_rotation(v.un.gameRotationVector.i, v.un.gameRotationVector.j, v.un.gameRotationVector.k,
                        v.un.gameRotationVector.real, now);
        } else if (v.sensorId == SH2_ACCELEROMETER) {
            const auto& a = v.un.accelerometer;
            if (!isfinite(a.x) || !isfinite(a.y) || !isfinite(a.z)) { diag.invalid++; return; }
            accel[0] = a.x, accel[1] = a.y, accel[2] = a.z;
        } else if (v.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            const auto& g = v.un.gyroscope;
            if (!isfinite(g.x) || !isfinite(g.y) || !isfinite(g.z)) { diag.invalid++; return; }
            last_gyro_ms = now;
            if (!have_quat) return;                       // the core takes its heading zero from the first sample
            if (n >= cap) { diag.dropped++; return; }
            FwImuSample& s = out[n++];
            s.t_us = extend_us((uint32_t)v.timestamp, now64);
            memcpy(s.quat, quat, sizeof(quat));
            s.gyro[0] = g.x, s.gyro[1] = g.y, s.gyro[2] = g.z;
            memcpy(s.accel, accel, sizeof(accel));
            last_out_yaw = yaw_of(quat);
            any_out = true;
        }
    }

    bool fresh(uint32_t now) const {
        return last_rv_ms && last_gyro_ms && now - last_rv_ms <= STREAM_TIMEOUT_MS &&
               now - last_gyro_ms <= STREAM_TIMEOUT_MS;
    }

    // every tick: -> the samples (each with its own time) since the last call
    uint32_t poll(FwImuSample* out, uint32_t cap, uint64_t now64) {
        const uint32_t now = millis();
        switch (phase) {
            case Phase::OFF: return 0;
            case Phase::RESET_LOW:
                if (now - t_phase >= RESET_LOW_MS) {
                    digitalWriteFast(pin::BNO_RST, HIGH);
                    enter(Phase::WAIT_RESET);
                }
                return 0;
            case Phase::WAIT_RESET:                       // the first transfer after our reset: configure at once
                if (int_low()) {
                    sh2_SensorValue_t ignored;
                    (void)dev.getSensorEvent(&ignored);
                    (void)dev.wasReset();
                    diag.sensor_resets++;
                    dev.clearSensorEvents();
                    clear_fresh();
                    if (enable_required()) enter(Phase::CONFIG_AUX);
                    else fail();
                } else if (now - t_phase > WAKE_TIMEOUT_MS) {
                    fail();
                }
                return 0;
            case Phase::CONFIG_AUX:                       // on an interrupt the sensor raised: it is awake
                if (int_low()) {
                    if (enable(SH2_ACCELEROMETER, RATE_HZ)) enter(Phase::WAIT_DATA);
                    else fail();
                } else if (now - t_phase > WAKE_TIMEOUT_MS) {
                    fail();
                }
                return 0;
            case Phase::RETRY_DELAY:
                if (now - t_phase >= RETRY_MS) begin_reset();
                return 0;
            default: break;                               // NOMINAL, GRACE, WAIT_DATA: read
        }
        uint32_t n = 0;
        sh2_SensorValue_t v;
        for (uint8_t i = 0; i < MAX_DRAIN; ++i) {         // drain first: a late tick finds what the sensor kept
            if (!dev.hasQueuedSensorEvent() && !int_low()) break;
            if (!dev.getSensorEvent(&v)) break;
            handle(v, out, cap, n, now64, now);
        }
        const uint32_t t = millis();
        if (dev.wasReset()) {                             // the sensor reset by itself: its reports are gone
            diag.sensor_resets++;
            clear_fresh();
            restitch = true;
            phase = Phase::GRACE;
            t_phase = t - GRACE_MS;                       // reconfigure through our own reset, next tick
            return n;
        }
        const bool ok = fresh(t);
        switch (phase) {
            case Phase::NOMINAL:
                if (!ok) { diag.stalls++; enter(Phase::GRACE); }
                break;
            case Phase::GRACE:
                if (ok) { diag.recoveries++; enter(Phase::NOMINAL); }
                else if (t - t_phase >= GRACE_MS) begin_reset();
                break;
            case Phase::WAIT_DATA:
                if (ok) { if (diag.recovery_attempts) diag.recoveries++; enter(Phase::NOMINAL); }
                else if (t - t_phase > WAKE_TIMEOUT_MS) fail();
                break;
            default: break;
        }
        return n;
    }
};
static Bno085 g_bno;

// -- time references (design.md 10) --------------------------------------------------------------------------------
// The core schedules each edge in MCU microseconds; a one-shot PIT timer fires it and its interrupt reads the time it
// actually rose, which is what the core reports (the TIME events). Pulse widths: SYNC_OUT 10 us, CAM_TRIG 20 us (VERIFY
// the OV9782's FSIN minimum).
struct EdgeTimer {
    IntervalTimer t;
    uint8_t pin = 0;
    uint32_t width_us = 10;
    uint64_t armed_for = 0;
    volatile uint32_t fired32 = 0;
    volatile bool fired = false;

    void fire() {
        digitalWriteFast(pin, HIGH);
        fired32 = micros();
        delayMicroseconds(width_us);
        digitalWriteFast(pin, LOW);
        fired = true;
        t.end();                                            // one shot
    }
    // re-arm when the schedule moved and the edge is due within the next tick and a bit
    void arm(uint64_t when, uint64_t now64, void (*isr)()) {
        if (!when || when == armed_for || when <= now64 || when - now64 > 1500) return;
        armed_for = when;
        t.begin(isr, (uint32_t)(when - now64));
        t.priority(16);                                     // above USB and the serial ports: the edge's jitter
    }
    uint64_t take(uint64_t now64) {
        if (!fired) return 0;
        fired = false;
        return extend_us(fired32, now64);
    }
};
static EdgeTimer g_sync, g_cam;
static void sync_isr() { g_sync.fire(); }
static void cam_isr() { g_cam.fire(); }

static volatile uint32_t g_strobe_rise32 = 0, g_strobe_width = 0;
static volatile bool g_strobe_new = false;
static void strobe_isr() {
    uint32_t now = micros();
    if (digitalReadFast(pin::CAM_STROBE)) {
        g_strobe_rise32 = now;
    } else {
        g_strobe_width = now - g_strobe_rise32;
        g_strobe_new = true;
    }
}

// -- stubs (TODO) -----------------------------------------------------------------------------------------------------
// Bidirectional DShot600 on the two ESC pins: throttle 0..1 -> 48..2047, 0 -> DShot 0; decode the eRPM telemetry
// (GCR, period in us as a 9-bit mantissa + 3-bit exponent) into fly_erpm, set fly_new.
static void dshot_write(const float*, bool) {}
static bool dshot_telemetry(FwHwInputs&) { return false; }
// CRSF from the ELRS receiver: channels 1-8 -> rc[] (-1..1), link state -> rc_ok (failsafe = 0).
static bool crsf_poll(FwHwInputs&) { return false; }

// -- setup / loop ---------------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(2000000);                       // native USB: the baud is ignored
    can1.begin();
    can1.setBaudRate(1000000);
    Serial3.begin(460800);                       // RoboClaw packet serial (VERIFY: set the same in Motion Studio)
    Serial4.begin(1000000, SERIAL_HALF_DUPLEX);  // STS bus
    Serial2.begin(420000);                       // CRSF
    pinMode(pin::BEAM_CATCH, INPUT);
    pinMode(pin::BEAM_FEED_EXIT, INPUT);
    pinMode(pin::ESTOP, INPUT_PULLUP);
    pinMode(pin::BUMPER_FRONT, INPUT_PULLUP);
    pinMode(pin::BUMPER_REAR, INPUT_PULLUP);
    pinMode(pin::BUZZER, OUTPUT);
    pinMode(pin::LED, OUTPUT);
    analogWriteFrequency(pin::IR_EMIT, 38000);   // the beams' receivers are 38 kHz demodulators
    analogWrite(pin::IR_EMIT, 128);
    analogReadResolution(12);
    g_bno.begin();
    g_sync.pin = pin::SYNC_OUT;
    g_cam.pin = pin::CAM_TRIG;
    g_cam.width_us = 20;
    pinMode(pin::SYNC_OUT, OUTPUT);
    pinMode(pin::CAM_TRIG, OUTPUT);
    pinMode(pin::CAM_STROBE, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin::CAM_STROBE), strobe_isr, CHANGE);
    g_fw.init();
    g_servo.torque(true);
    watchdog_start();                            // last: setup's own waits don't count
}

static void read_inputs(uint64_t t_us) {
    FwHwInputs& in = g_in;
    memset(&in, 0, sizeof(in));
    in.t_us = t_us;
    CAN_message_t msg;
    uint16_t timer_now = can1_timer();
    uint64_t now64 = micros64();
    while (in.n_can_rx < FW_CAN_MAX && can1.read(msg)) {
        FwCanFrame& f = in.can_rx[in.n_can_rx++];
        f.id = msg.id;
        f.ext = msg.flags.extended ? 1 : 0;
        f.len = msg.len;
        memcpy(f.data, msg.buf, 8);
        f.t_rx_us = now64 - (uint16_t)(timer_now - msg.timestamp) + (f.ext ? EXT_REPLY_US : DM_REPLY_US);   // its end
    }
    in.imu_n = g_bno.poll(in.imu, FW_IMU_MAX, now64);
    in.sync_fired_us = g_sync.take(now64);
    in.cam_fired_us = g_cam.take(now64);
    if (g_strobe_new) {
        noInterrupts();
        uint32_t rise = g_strobe_rise32, width = g_strobe_width;
        g_strobe_new = false;
        interrupts();
        in.strobe_rise_us = extend_us(rise, now64);
        in.strobe_width_us = width;
    }
    dshot_telemetry(in);
    g_rc.poll();
    if (g_rc.fresh) {
        in.feed_new = 1;
        in.feed_speed[0] = g_rc.speed[0];
        in.feed_speed[1] = g_rc.speed[1];
        g_rc.fresh = false;
    }
    g_servo.poll();
    if (g_servo.fresh) {
        in.tilt_new = 1;
        in.tilt_pos = g_servo.pos;
        g_servo.fresh = false;
    }
    in.beam_catch = digitalReadFast(pin::BEAM_CATCH) ? 1 : 0;      // the receiver idles low while it sees the IR
    in.beam_feed_exit = digitalReadFast(pin::BEAM_FEED_EXIT) ? 1 : 0;
#ifdef BENCH_NO_ESTOP
    in.estop = 0;                                // env arm_bench: no e-stop chain on the bench (never on a robot)
#else
    in.estop = digitalReadFast(pin::ESTOP) ? 1 : 0;                 // the chain's relay contact opens to 1
#endif
    // normally-closed switches to ground: closed (low) = clear; pressed or a broken wire (high, the pull-up) = hit.
    // The core ignores it without HAS_BUMPERS (the arm robot has none yet: todo.md)
    in.bumper = (digitalReadFast(pin::BUMPER_FRONT) || digitalReadFast(pin::BUMPER_REAR)) ? 1 : 0;
#ifdef BENCH_V_BUS
    in.v_bus = BENCH_V_BUS;                      // env arm_bench: no divider on A17, a fixed bench supply (not measured)
#else
    in.v_bus = analogRead(pin::V_BUS) * ADC_REF / 4095.0f * V_BUS_DIVIDER;
#endif
    in.rc_ok = crsf_poll(in) ? 1 : 0;
}

static void write_outputs(uint32_t tick) {
    const FwHwOutputs& out = g_out;
    uint64_t now64 = micros64();
    g_sync.arm(out.sync_next_us, now64, sync_isr);
    g_cam.arm(out.cam_next_us, now64, cam_isr);
    for (uint32_t i = 0; i < out.n_can_tx; ++i) {
        CAN_message_t msg;
        msg.id = out.can_tx[i].id;
        msg.flags.extended = out.can_tx[i].ext ? 1 : 0;
        msg.len = out.can_tx[i].len;
        memcpy(msg.buf, out.can_tx[i].data, 8);
        can1.write(msg);
    }
    dshot_write(out.fly_throttle, out.fly_brake != 0);
    // the slower buses at their own rates: the RoboClaw at 100 Hz (speeds) / 50 Hz (reads), the servo at 100 Hz
    switch (tick % 10) {
        case 0: g_rc.speed_cmd(0, out.feed_speed_cmd[0]); g_rc.speed_cmd(1, out.feed_speed_cmd[1]); break;
        case 3: g_rc.request_speed((tick / 10) % 2); break;
        case 5: g_servo.goal(out.tilt_cmd); break;
        case 7: if (tick % 20 == 7) g_servo.request_pos(); break;
        default: break;
    }
    if (out.buzzer) tone(pin::BUZZER, 2700, 50);
    digitalWriteFast(pin::LED, (out.led == fw::MODE_AUTO) ? ((tick / 250) & 1) : (out.led == fw::MODE_ESTOP));
}

void loop() {
    static uint64_t next = 0;
    static uint32_t tick = 0;
    static uint8_t buf[512];
    while (Serial.available()) {                 // the robot computer's bytes, as they come, stamped (the echo's t2)
        int n = Serial.readBytes((char*)buf, min((int)sizeof(buf), Serial.available()));
        g_fw.rx(buf, (size_t)n, micros64());
    }
    uint64_t now = micros64();
    if (now < next) return;
    next = (next == 0 ? now : next) + 1000;      // 1 kHz, no drift
    read_inputs(now);
    g_fw.tick(g_in, g_out);
    write_outputs(tick++);
    watchdog_feed();
    // take out only what USB has room for: the rest stays queued in the core (it used to be taken and dropped)
    int room = Serial.availableForWrite();
    size_t n = room > 0 ? g_fw.tx(buf, min((size_t)room, sizeof(buf))) : 0;
    if (n) Serial.write(buf, n);
}

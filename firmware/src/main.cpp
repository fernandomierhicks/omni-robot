// omni-robot MCU, Teensy 4.1: the hardware layer around the firmware core (design.md 4, 30).
//
// Every 1 ms: read the peripherals into FwHwInputs, Firmware::tick(), drive the peripherals from FwHwOutputs. The
// Jetson's bytes come and go on the native USB serial port between ticks. The core (lib/omni_fw) is the code the sim
// runs in the loop; only this file is Teensy-specific.
//
// Drivers here, and how far each is done:
//   CAN1, DM-2325 x4          FlexCAN_T4, 1 Mbit/s: complete (the protocol is in the core: dm_motor.cpp)
//   USB serial, Jetson link   complete (framing in the core: protocol.cpp)
//   RoboClaw, feed motors     packet serial, speed commands + speed reads: written, VERIFY on the bench
//   STS3250, tilt servo       Feetech half-duplex bus, goal position + position reads: written, VERIFY on the bench
//   break-beams, e-stop, bumpers, battery ADC, buzzer, LED       complete (thresholds and divider ratio VERIFY)
//   BNO085 (SH-2 over SPI)    written, VERIFY on the bench: SHTP reads on INT, the INT edge stamped in its interrupt,
//                             gyro / accel / game RV at 400 Hz, each sample's time from the SH-2 timestamps (sh2.cpp)
//   time references           written, VERIFY: SYNC_OUT and CAM_TRIG on one-shot PIT timers at the core's scheduled
//                             MCU times, the fired time read in the timer's interrupt; the left camera's STROBE on a pin
//                             interrupt (both edges). CAN replies stamped by FlexCAN's own timer (design.md 10)
//   bidirectional DShot       TODO: stub, no output or telemetry yet (the core's rpm loop is ready for it)
//   ELRS / CRSF               TODO: stub, rc_ok = 0 (MANUAL mode unreachable until written)
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>

#include "board_teensy41.h"
#include "firmware.h"
#include "sh2.h"

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

// -- BNO085 over SPI (SHTP / SH-2), written: VERIFY on the bench ----------------------------------------------------
// SPI mode 3, <= 3 MHz; PS0 (WAKE) and PS1 high at reset select SPI. H_INTN falls when the hub has data: its
// interrupt stamps that moment, which is the SH-2 timestamps' reference (sh2.h). Every packet: a 4-byte SHTP header
// (length with bit 15 = continuation, channel, sequence), then the payload. Channel 3 = sensor reports.
struct Bno085 {
    static constexpr uint32_t PERIOD_US = 2500;          // 400 Hz for gyro, accelerometer and game rotation vector
    volatile uint32_t t_int32 = 0;
    volatile bool pending = false;
    uint8_t seq_out[6] = {};
    uint8_t buf[512];
    FwImuSample last{};
    bool configured = false;
    uint32_t t_reset_ms = 0;

    void begin() {
        pinMode(pin::BNO_CS, OUTPUT);
        digitalWriteFast(pin::BNO_CS, HIGH);
        pinMode(pin::BNO_WAKE, OUTPUT);
        digitalWriteFast(pin::BNO_WAKE, HIGH);
        pinMode(pin::BNO_PS1, OUTPUT);
        digitalWriteFast(pin::BNO_PS1, HIGH);
        pinMode(pin::BNO_BOOTN, OUTPUT);
        digitalWriteFast(pin::BNO_BOOTN, HIGH);                 // run the application, not the bootloader
        pinMode(pin::BNO_INT, INPUT_PULLUP);
        pinMode(pin::BNO_RST, OUTPUT);
        digitalWriteFast(pin::BNO_RST, LOW);
        delay(2);
        digitalWriteFast(pin::BNO_RST, HIGH);                   // the straps are read now
        t_reset_ms = millis();
        SPI.begin();
        attachInterrupt(digitalPinToInterrupt(pin::BNO_INT), isr, FALLING);
    }
    static void isr();

    size_t transfer(const uint8_t* tx, size_t n_tx) {           // one SHTP packet each way; returns the rx length
        SPI.beginTransaction(SPISettings(3000000, MSBFIRST, SPI_MODE3));
        digitalWriteFast(pin::BNO_CS, LOW);
        uint8_t h[4];
        for (int i = 0; i < 4; ++i) h[i] = SPI.transfer(i < (int)n_tx ? tx[i] : 0);
        size_t len = (size_t)((h[0] | (h[1] << 8)) & 0x7FFF);
        size_t n = len > n_tx ? len : n_tx;
        if (n > sizeof(buf)) n = sizeof(buf);
        memcpy(buf, h, 4);
        for (size_t i = 4; i < n; ++i) buf[i] = SPI.transfer(i < n_tx ? tx[i] : 0);
        digitalWriteFast(pin::BNO_CS, HIGH);
        SPI.endTransaction();
        return len <= sizeof(buf) ? len : 0;
    }

    void set_feature(uint8_t report_id) {                       // SH-2 Set Feature (0xFD) on channel 2
        uint8_t p[21] = {21, 0, 2, seq_out[2]++, 0xFD, report_id, 0, 0, 0};
        uint32_t us = PERIOD_US;
        memcpy(p + 9, &us, 4);
        digitalWriteFast(pin::BNO_WAKE, LOW);                   // wake it to take a write (it answers on INT)
        uint32_t t0 = micros();
        while (!pending && micros() - t0 < 20000) {}
        digitalWriteFast(pin::BNO_WAKE, HIGH);
        transfer(p, sizeof(p));
        pending = false;
    }

    // -> samples (each with its own time) since the last call
    uint32_t poll(FwImuSample* out, uint32_t cap, uint64_t now64) {
        if (!configured) {
            if (millis() - t_reset_ms < 300) return 0;          // its boot (est; it announces itself on channel 0)
            set_feature(fw::SH2_GYRO);
            set_feature(fw::SH2_ACCEL);
            set_feature(fw::SH2_GAME_RV);
            configured = true;
        }
        if (!pending) return 0;
        noInterrupts();
        uint32_t t32 = t_int32;
        pending = false;
        interrupts();
        size_t len = transfer(nullptr, 0);
        if (len < 5 || buf[2] != 3) return 0;                   // only sensor reports (channel 3) carry samples
        fw::Sh2Report r[16];
        size_t nr = fw::sh2_parse(buf + 4, len - 4, extend_us(t32, now64), r, 16);
        return (uint32_t)fw::sh2_samples(r, nr, last, out, cap);
    }
};
static Bno085 g_bno;
void Bno085::isr() {
    g_bno.t_int32 = micros();
    g_bno.pending = true;
}

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
        f.len = msg.len;
        memcpy(f.data, msg.buf, 8);
        f.t_rx_us = now64 - (uint16_t)(timer_now - msg.timestamp) + DM_REPLY_US;     // the frame's end
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
    in.estop = digitalReadFast(pin::ESTOP) ? 1 : 0;                 // the chain's relay contact opens to 1
    in.bumper = (!digitalReadFast(pin::BUMPER_FRONT) || !digitalReadFast(pin::BUMPER_REAR)) ? 1 : 0;
    in.v_bus = analogRead(pin::V_BUS) * ADC_REF / 4095.0f * V_BUS_DIVIDER;
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
    while (Serial.available()) {                 // the Jetson's bytes, as they come, stamped (the echo's t2)
        int n = Serial.readBytes((char*)buf, min((int)sizeof(buf), Serial.available()));
        g_fw.rx(buf, (size_t)n, micros64());
    }
    uint64_t now = micros64();
    if (now < next) return;
    next = (next == 0 ? now : next) + 1000;      // 1 kHz, no drift
    read_inputs(now);
    g_fw.tick(g_in, g_out);
    write_outputs(tick++);
    size_t n = g_fw.tx(buf, sizeof(buf));
    if (n && Serial.availableForWrite() >= (int)n) Serial.write(buf, n);
}

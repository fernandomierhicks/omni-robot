// The robot's firmware core on an Arduino UNO R4 WiFi: a bench stand-in for the Teensy 4.1 layer (firmware/src/main.cpp)
// with ONE real CubeMars AK40-10 on CAN (README.md here). The core (lib/omni_fw, arm variant) is the robot's, unchanged:
// modes, watchdog on the link, ramped stops, the base controller, odometry, the servo-mode CAN driver, the USB protocol.
//
// What this layer does differently from the Teensy's, and why:
//   CAN       the R4's own controller (Arduino_CAN, CANTX D10 / CANRX D13, 1 Mbit/s) through a transceiver board, plus the
//             start-up kick the bench found (bench/ak40_uno_r4: CANRX stuck dominant after CAN.begin until D10 is
//             driven high once). Frames are stamped when the loop reads them (no hardware timestamp)
//   motors    commands go on the bus only for the motors heard there (is_real: any CAN id whose status broadcast came
//             in), and frames for the other ids never do (unacknowledged, a frame would hold the R4's single extended
//             TX mailbox and block the real motor's). By default the others are simply absent, as on a robot with
//             motors missing: STATE reports them DM_NO_REPLY, the console refuses driving and its motor test runs the
//             real one(s) (protocol 8). BENCH_VIRTUAL_WHEELS (env virtual) makes wheels 2-4
//             VIRTUAL instead: each answers its commands with a status broadcast (500 Hz, first-order speed response),
//             like the sim's AkMotor, so the whole drive chain runs as with four motors (the Nov 12 TELEOP run). The
//             console then believes four motors are there: never on a robot
//   v_bus     no divider on this board: a fixed BENCH_V_BUS (the bench supply, not measured)
//   link      the RA4M1's native USB CDC (platformio.ini drops -DNO_USB: the core moves the board's USB mux off the
//             ESP32-S3), like the Teensy's; the baud is ignored. Through the bridge's UART, with CMDs both ways, the
//             link corrupted frames and hung within seconds (2 Mbaud and 921600 alike). Read with TinyUSB's bulk
//             tud_cdc_read(), and the port tested with tud_cdc_connected(): Serial.readBytes() goes one read() per
//             byte (each checking the connection) and `Serial ?` runs tud_task(), the whole USB service (it already
//             runs in the USB interrupt), every call: a CMD's receive took 1.28 ms that way, 0.46 ms in bulk (Nov 13)
//   flashing  the 1200-baud touch: the core's own bootloader jump is off (NO_1200_BPS_RESET; it left the board silent
//             and unflashable). Instead: zero current to the motors, USB detached, the mux back to the ESP32-S3; the
//             bridge then appears (2341:1002) and flashes as usual (flash_pi.sh touches it again)
//   not here  BNO085 (IMU_SILENT is reported, it doesn't stop the base), RC, e-stop input (0), bumpers, flywheels, feed,
//             tilt, time references, the hardware watchdog, the LED (D13 is CANRX on this board)
//
// BENCH_PROFILE (platformio.ini env profile, with the virtual wheels; README "Late ticks"): each loop section's longest
// time over 0.5 s windows, reported as the VIRTUAL wheels' temperatures (the only way out is the robot's binary link): wheel 2 = receive,
// 3 = inputs + tick + outputs, 4 = transmit; in units of 20 us (127 = 2.54 ms or more). link_probe.py --profile
#include <Arduino.h>
#include <Arduino_CAN.h>

#include "firmware.h"
#include "tusb.h"

#ifdef BENCH_PROFILE
struct ProfSection {
    uint32_t cur = 0, shown = 0;                       // longest in this window, in the last one (us)
    void add(uint32_t us) { if (us > cur) cur = us; }
};
static ProfSection g_prof[3];                          // receive, tick, transmit -> virtual motors 0, 1, 2 (ids 2, 3, 4)
static uint64_t g_prof_t = 0;
static constexpr uint64_t PROF_WINDOW_US = 500000;
static void prof_roll(uint64_t now) {
    if (now - g_prof_t < PROF_WINDOW_US) return;
    g_prof_t = now;
    for (ProfSection& p : g_prof) { p.shown = p.cur; p.cur = 0; }
}
static int8_t prof_temp(int k) {
    uint32_t v = g_prof[k].shown / 20;
    return (int8_t)(v > 127 ? 127 : v);
}
#define PROF_T0(name) uint32_t name = micros()
#define PROF_ADD(k, t0) g_prof[k].add(micros() - (t0))
#else
#define PROF_T0(name)
#define PROF_ADD(k, t0)
#endif

static fw::Firmware g_fw;
static FwHwInputs g_in;
static FwHwOutputs g_out;

static constexpr uint32_t LINK_BAUD = 2000000;         // ignored by native USB
static constexpr size_t TX_CHUNK = 256;                // link bytes handed to USB per tick at most (see loop)
static constexpr int PIN_USB_MUX = 21;                 // P408: high = USB to the RA4M1 (variant.cpp), low = ESP32-S3
static constexpr float BENCH_V_BUS = 24.0f;           // V: above BATT_LOW + BATT_RESUME (22.2 V), so it can arm
static constexpr uint8_t REAL_IDS[] = {1};             // env virtual: the motor on the bus; the core's other ids are virtual
static constexpr float VIRT_TAU_S = 0.05f;             // virtual motors: speed response time constant (est)
static constexpr uint32_t VIRT_PERIOD_US = 2000;       // ... their status broadcast, 500 Hz like the real one
static bool g_heard[256];                              // the controller ids whose status broadcast came in on the bus

// A motor is real (its commands go on the bus) once its status broadcast has been heard, whatever its CAN id: the
// bench takes whichever wheel is plugged in (Nov 13: id 2, whose commands a fixed REAL_IDS = {1} dropped while its
// status still showed it answering). Ids never heard stay off the bus. Env virtual keeps the fixed REAL_IDS: there the
// other ids are the virtual motors
static bool is_real(uint8_t id) {
    for (uint8_t r : REAL_IDS)
        if (r == id) return true;
#ifdef BENCH_VIRTUAL_WHEELS
    return false;
#else
    return g_heard[id];
#endif
}

// -- 64-bit microseconds --------------------------------------------------------------------------------------------
static uint64_t micros64() {
    static uint32_t last = 0;
    static uint64_t hi = 0;
    uint32_t now = micros();
    if (now < last) hi += (1ull << 32);
    last = now;
    return hi | now;
}

// -- virtual motors -------------------------------------------------------------------------------------------------
struct VirtMotor {
    uint8_t id = 0;
    float target_erpm = 0.0f, erpm = 0.0f, pos_deg = 0.0f;
    bool coast = true;
    uint64_t t_next = 0;
};
static VirtMotor g_virt[fw::prm::N_WHEELS];
static int g_n_virt = 0;

static void virt_command(const FwCanFrame& f) {
    uint8_t id, mode;
    int32_t v;
    if (!fw::ak_decode_command(f, id, mode, v)) return;
    for (int k = 0; k < g_n_virt; ++k) {
        VirtMotor& m = g_virt[k];
        if (m.id != id) continue;
        if (mode == fw::AK_VELOCITY) {
            m.target_erpm = (float)v;
            m.coast = false;
        } else {                                       // current 0 / brake: it coasts down
            m.target_erpm = 0.0f;
            m.coast = true;
        }
    }
}

static void virt_step(float dt, uint64_t now, FwHwInputs& in) {
    for (int k = 0; k < g_n_virt; ++k) {
        VirtMotor& m = g_virt[k];
        float tau = m.coast ? 4.0f * VIRT_TAU_S : VIRT_TAU_S;
        m.erpm += (m.target_erpm - m.erpm) * (dt / (tau + dt));
        m.pos_deg += fw::ak_rad_s(m.erpm, fw::prm::AK_GEAR, fw::prm::AK_POLE_PAIRS) * dt * 57.29578f;
        if (m.pos_deg > fw::AK_POS_WRAP_DEG) m.pos_deg -= 2.0f * fw::AK_POS_WRAP_DEG;
        if (m.pos_deg < -fw::AK_POS_WRAP_DEG) m.pos_deg += 2.0f * fw::AK_POS_WRAP_DEG;
        if (now >= m.t_next && in.n_can_rx < FW_CAN_MAX) {
            m.t_next = now + VIRT_PERIOD_US;
            fw::AkStatus s;
            s.id = m.id;
            s.pos_deg = m.pos_deg;
            s.erpm = m.erpm;
            s.current = 0.0f;
#ifdef BENCH_PROFILE
            s.temp_c = k < 3 ? prof_temp(k) : 25;
#else
            s.temp_c = 25;
#endif
            s.err = fw::AK_OK;
            FwCanFrame& f = in.can_rx[in.n_can_rx++];
            f = fw::ak_encode_status(s);
            f.t_rx_us = now;
        }
    }
}

// -- CAN --------------------------------------------------------------------------------------------------------------
static uint32_t g_can_tx_fail = 0;

static void can_start() {
    CAN.begin(CanBitRate::BR_1000k);
    // the start-up kick (bench/ak40_uno_r4 README): drive CANTX high once, then hand it back to the CAN peripheral
    pinMode(PIN_CAN0_TX, OUTPUT);
    digitalWrite(PIN_CAN0_TX, HIGH);
    delay(5);
    R_IOPORT_PinCfg(&g_ioport_ctrl, g_pin_cfg[PIN_CAN0_TX].pin,
                    (uint32_t)IOPORT_CFG_PERIPHERAL_PIN | (uint32_t)IOPORT_PERIPHERAL_CAN);
    delay(5);
}

static void can_send(const FwCanFrame& f) {
    uint32_t id = f.ext ? (f.id | CanMsg::CAN_EFF_FLAG) : f.id;
    if (CAN.write(CanMsg(id, f.len, f.data)) < 0) ++g_can_tx_fail;
}

// -- setup / loop -------------------------------------------------------------------------------------------------------
void setup() {
    Serial.begin(LINK_BAUD);
    can_start();
#ifdef BENCH_VIRTUAL_WHEELS
    for (int i = 0; i < fw::prm::N_WHEELS; ++i) {
        uint8_t id = fw::prm::DM_IDS[i];
        if (!is_real(id)) g_virt[g_n_virt++].id = id;
    }
#endif
    g_fw.init();
}

static void read_inputs(uint64_t now, float dt) {
    FwHwInputs& in = g_in;
    memset(&in, 0, sizeof(in));
    in.t_us = now;
    while (in.n_can_rx < FW_CAN_MAX && CAN.available()) {
        CanMsg m = CAN.read();
        FwCanFrame& f = in.can_rx[in.n_can_rx++];
        f.ext = m.isExtendedId() ? 1 : 0;
        f.id = f.ext ? m.getExtendedId() : m.getStandardId();
        f.len = m.data_length;
        memcpy(f.data, m.data, f.len);
        f.t_rx_us = now;
        if (f.ext && ((f.id >> 8) & 0xFF) == fw::AK_STATUS && (f.id >> 16) == 0) g_heard[f.id & 0xFF] = true;
    }
    virt_step(dt, now, in);
    in.v_bus = BENCH_V_BUS;
}

static void write_outputs() {
    for (uint32_t i = 0; i < g_out.n_can_tx; ++i) {
        const FwCanFrame& f = g_out.can_tx[i];
        if (f.ext && is_real((uint8_t)(f.id & 0xFF))) can_send(f);
        else virt_command(f);
    }
}

// a flasher's 1200-baud touch: stop the real motors, detach from USB and give the port back to the ESP32-S3 bridge,
// which then resets this MCU into its ROM bootloader when the flasher touches the bridge (flash_pi.sh). Never returns
static void handoff_to_bridge() {
    for (int id = 0; id < 256; ++id) {
        if (!is_real((uint8_t)id)) continue;
        FwCanFrame f = fw::ak_current_cmd((uint8_t)id, 0.0f);
        can_send(f);
        delay(2);
    }
    ((R_USB_FS0_Type*)R_USB_FS0_BASE)->SYSCFG_b.DPRPU = 0;       // the host sees this USB device go
    delay(20);
    pinMode(PIN_USB_MUX, OUTPUT);
    digitalWrite(PIN_USB_MUX, LOW);                              // ... and the bridge arrive
    while (true) {}
}

void loop() {
    static uint64_t next = 0, last = 0;
    static uint8_t buf[256];
    if (_SerialUSB::_bps == 1200 && !_SerialUSB::_dtr) handoff_to_bridge();
    PROF_T0(t_rx);
    while (tud_cdc_available()) {                      // in bulk (see the top)
        uint32_t n = tud_cdc_read(buf, sizeof(buf));
        if (n == 0) break;
        g_fw.rx(buf, (size_t)n, micros64());
    }
    PROF_ADD(0, t_rx);
    uint64_t now = micros64();
#ifdef BENCH_PROFILE
    prof_roll(now);
#endif
    if (now < next) return;
    next = (next == 0 ? now : next) + 1000;            // 1 kHz, no drift
    if (now > next + 20000) next = now + 1000;          // badly behind (a stall): don't burst to catch up
    float dt = last ? (float)(now - last) * 1e-6f : 0.001f;
    last = now;
    PROF_T0(t_tick);
    read_inputs(now, dt);
    g_fw.tick(g_in, g_out);
    write_outputs();
    PROF_ADD(1, t_tick);
    // native USB CDC: take out only what it has room for (the rest stays queued in the core), and nothing while no
    // host holds the port open (SerialUSB drops writes then)
    PROF_T0(t_tx);
    int room = tud_cdc_connected() ? Serial.availableForWrite() : 0;      // not `Serial ?` (see the top)
    size_t n = room > 0 ? g_fw.tx(buf, min((size_t)room, TX_CHUNK)) : 0;
    if (n) Serial.write(buf, n);
    PROF_ADD(2, t_tx);
}

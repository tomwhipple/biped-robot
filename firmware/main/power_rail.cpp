#include "power_rail.h"

#include "board.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_private/panic_internal.h"
#include "esp_private/system_internal.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "shared.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#if BIMO_POWER_LITE
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#endif

namespace robot {
namespace rail {

std::atomic<bool> g_bus_up{false};

namespace {

#if BIMO_POWER_LITE
// GPIO34 is ADC1 channel 6. ADC_ATTEN_DB_12 is the same hardware setting the ESP32
// datasheet calls 11 dB (ADC_ATTEN_DB_11 is its deprecated alias in IDF 5.4).
constexpr adc_channel_t kRailChannel = ADC_CHANNEL_6;
static_assert(board::kRailV == GPIO_NUM_34, "GPIO34 is ADC1 channel 6");
// A few reads per tick against the ADC's noise (C2 already filters what it sees).
constexpr int kOversample = 4;

adc_oneshot_unit_handle_t g_adc = nullptr;
adc_cali_handle_t g_cali = nullptr;
#endif

// Written by ctrl only.
railguard::Guard g_guard;
uint32_t g_op_estops_seen = 0;
uint32_t g_req_seen = 0;

std::atomic<uint8_t> g_state{0}, g_fault{0}, g_last_drop{0};
std::atomic<bool> g_servo_en{false}, g_estop_pressed{true}, g_estop_latched{false};
std::atomic<bool> g_rail_valid{false};
std::atomic<uint16_t> g_rail_mv{0};

// Housekeeping -> ctrl: one word, so a request can never be half read. Each call
// waits for ctrl's answer (at most a tick), so none overwrites the one before.
//   bits 16-31 sequence, 8-15 op, 0-7 DropWhy
enum class Op : uint8_t { kOn = 1, kOff = 2, kDisarm = 3, kClearFault = 4 };
std::atomic<uint32_t> g_req{0};
uint32_t g_req_next = 0;                 // housekeeping only
// ctrl -> housekeeping: the sequence applied (bits 8-23) and its verdict (bits 0-7).
std::atomic<uint32_t> g_ack{0};

constexpr uint32_t reqWord(uint32_t seq, Op op, railguard::DropWhy why) {
    return (seq << 16) | (static_cast<uint32_t>(op) << 8) | static_cast<uint32_t>(why);
}

uint32_t post(Op op, railguard::DropWhy why) {
    g_req_next = (g_req_next + 1u) & 0xFFFFu;
    if (g_req_next == 0) g_req_next = 1;      // 0 is "no request yet"
    g_req.store(reqWord(g_req_next, op, why));
    return g_req_next;
}

// Wait for ctrl to apply request `seq`, up to 5 ticks. True with its answer.
bool awaitAck(uint32_t seq, uint8_t& answer) {
    for (int i = 0; i < 50; ++i) {
        const uint32_t a = g_ack.load();
        if (((a >> 8) & 0xFFFFu) == seq) {
            answer = static_cast<uint8_t>(a & 0xFFu);
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

#if BIMO_POWER_LITE
bool readRailMv(uint16_t& mv) {
    if (!g_adc || !g_cali) return false;
    int sum = 0;
    for (int i = 0; i < kOversample; ++i) {
        int raw = 0;
        if (adc_oneshot_read(g_adc, kRailChannel, &raw) != ESP_OK) return false;
        sum += raw;
    }
    int pin_mv = 0;
    if (adc_cali_raw_to_voltage(g_cali, sum / kOversample, &pin_mv) != ESP_OK) return false;
    mv = railguard::railMvFromPinMv(static_cast<uint32_t>(pin_mv < 0 ? 0 : pin_mv));
    return true;
}

void adcInit() {
    adc_oneshot_unit_init_cfg_t unit = {};
    unit.unit_id = ADC_UNIT_1;
    if (adc_oneshot_new_unit(&unit, &g_adc) != ESP_OK) return;
    adc_oneshot_chan_cfg_t chan = {};
    chan.atten = ADC_ATTEN_DB_12;
    chan.bitwidth = ADC_BITWIDTH_12;
    adc_oneshot_config_channel(g_adc, kRailChannel, &chan);
    // A failure leaves g_cali null: no reading, so no rail, so no arm, and `rail` says so.
    adc_cali_line_fitting_config_t cali = {};
    cali.unit_id = ADC_UNIT_1;
    cali.atten = ADC_ATTEN_DB_12;
    cali.bitwidth = ADC_BITWIDTH_12;
    cali.default_vref = 1100;          // for a chip with no eFuse Vref
    adc_cali_create_scheme_line_fitting(&cali, &g_cali);
}
#else
// A default image never calls these (power_rail.h): the rest of this file still compiles.
bool readRailMv(uint16_t&) { return false; }
void adcInit() {}
#endif

void driveAndPublish() {
    const bool en = g_guard.servoEn();
    if (en != g_servo_en.load()) gpio_set_level(board::kServoEn, en ? 1 : 0);
    g_state.store(static_cast<uint8_t>(g_guard.state()));
    g_fault.store(static_cast<uint8_t>(g_guard.fault()));
    g_last_drop.store(static_cast<uint8_t>(g_guard.lastDrop()));
    g_servo_en.store(en);
    g_estop_pressed.store(g_guard.estopPressed());
    g_estop_latched.store(g_guard.estopLatched());
    g_rail_valid.store(g_guard.railValid());
    g_rail_mv.store(g_guard.railMv());
    g_bus_up.store(g_guard.busUp());
}

}  // namespace

void boot() {
    // Output register first, then the direction, so the pin never drives high.
    gpio_set_level(board::kServoEn, 0);
    gpio_config_t out = {};
    out.pin_bit_mask = 1ULL << board::kServoEn;
    out.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out);
    gpio_set_level(board::kServoEn, 0);

    gpio_config_t in = {};
    in.pin_bit_mask = 1ULL << board::kEstopN;
    in.mode = GPIO_MODE_INPUT;
    in.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&in);

    adcInit();

    // CONFIG_ESP_TASK_WDT_PANIC is off, so a wedged ctrl would only print a warning
    // and keep the rail on. In this build it is a panic, which drops the rail.
    esp_task_wdt_config_t twdt = {};
    twdt.timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000;
    twdt.idle_core_mask = 0;
    twdt.trigger_panic = true;
    esp_task_wdt_reconfigure(&twdt);
}

void ctrlTick(uint32_t now_ms) {
    railguard::Sample s;
    s.now_ms = now_ms;
    s.estop_high = gpio_get_level(board::kEstopN) != 0;
    s.rail_valid = readRailMv(s.rail_mv);
    g_guard.update(s);

    const uint32_t req = g_req.load();
    const uint32_t seq = req >> 16;
    const bool have_req = seq != g_req_seen;
    uint8_t answer = 0;
    if (have_req) {
        g_req_seen = seq;
        const Op op = static_cast<Op>((req >> 8) & 0xFFu);
        const auto why = static_cast<railguard::DropWhy>(req & 0xFFu);
        switch (op) {
            case Op::kOn:
                answer = static_cast<uint8_t>(g_guard.requestOn());
                break;
            case Op::kOff:
                g_guard.drop(why);
                break;
            case Op::kDisarm:
                g_guard.drop(why);
                g_guard.disabledCommand();
                break;
            case Op::kClearFault:
                answer = g_guard.clearFault() ? 1 : 0;
                break;
        }
    }

    // The hard operator stop, in both modes (shared.h g_op_stops). A stop that lands
    // in the tick after a request wins, by being applied after it.
    const uint32_t estops = g_op_estops.load();
    if (estops != g_op_estops_seen) {
        g_op_estops_seen = estops;
        g_guard.drop(railguard::DropWhy::kOperatorStop);
    }

    driveAndPublish();
    // After the publish: whoever sees the answer sees the state that goes with it.
    if (have_req) g_ack.store((seq << 8) | answer);

    // The servos are off, so what they last said about the pack is history. 0 is the
    // beacon's "no reading", as at boot.
    if (!g_guard.busUp()) g_telemetry.vbat_mv.store(0);

    if (g_mode_request.load() == Mode::kRun && !g_guard.busUp()) {
        g_arm_result.store(static_cast<uint8_t>(
            g_guard.lastDrop() == railguard::DropWhy::kOperatorStop
                ? linkproto::ArmResult::kDisarmedStop
                : linkproto::ArmResult::kRailDown));
        g_mode_request.store(Mode::kBench);   // toward bench only: shared.h
    }
}

void dropFromCtrl(railguard::DropWhy why) {
    g_guard.drop(why);
    driveAndPublish();
}

void disabledFromCtrl() { g_guard.disabledCommand(); }

railguard::Verdict requestOn() {
    uint8_t answer = 0;
    if (!awaitAck(post(Op::kOn, railguard::DropWhy::kNone), answer)) {
        return railguard::Verdict::kNoReading;
    }
    return static_cast<railguard::Verdict>(answer);
}

void requestOff(railguard::DropWhy why) {
    uint8_t answer = 0;
    awaitAck(post(Op::kOff, why), answer);
}

void requestDisarm() {
    uint8_t answer = 0;
    awaitAck(post(Op::kDisarm, railguard::DropWhy::kDisarm), answer);
}

bool requestClearFault() {
    uint8_t answer = 0;
    return awaitAck(post(Op::kClearFault, railguard::DropWhy::kNone), answer) && answer == 1;
}

Snapshot snapshot() {
    Snapshot s;
    s.state = static_cast<railguard::State>(g_state.load());
    s.fault = static_cast<railguard::Fault>(g_fault.load());
    s.last_drop = static_cast<railguard::DropWhy>(g_last_drop.load());
    s.servo_en = g_servo_en.load();
    s.estop_pressed = g_estop_pressed.load();
    s.estop_latched = g_estop_latched.load();
    s.rail_valid = g_rail_valid.load();
    s.rail_mv = g_rail_mv.load();
    return s;
}

}  // namespace rail
}  // namespace robot

// -- a panic or a CPU reset drops the rail first (power_rail.h) ---------------------
// firmware/CMakeLists.txt adds -Wl,--wrap=... for these, in the power-lite build only.
// A register write from IRAM: this may run with the flash cache off.
#if BIMO_POWER_LITE
static_assert(board::kServoEn == GPIO_NUM_16, "the register write below is IO16's");

extern "C" {
void __real_esp_panic_handler(panic_info_t* info);
void __real_esp_restart_noos(void) __attribute__((noreturn));

void IRAM_ATTR __wrap_esp_panic_handler(panic_info_t* info) {
    REG_WRITE(GPIO_OUT_W1TC_REG, 1u << 16);
    __real_esp_panic_handler(info);
}

void IRAM_ATTR __wrap_esp_restart_noos(void) {
    REG_WRITE(GPIO_OUT_W1TC_REG, 1u << 16);
    __real_esp_restart_noos();
}
}
#endif

// Boot: bring up the hardware, create the three tasks, then get out of the way.
//
// Everything that will ever be allocated is allocated here, before any task
// runs (firmware-design section 2: no heap after init). The board comes up in
// bench mode with torque off -- a powered board must not move because nobody
// told it not to.
#include <string.h>

#include "board.h"
#include "cli.h"
#include "ctrl_task.h"
#include "imu_sampler.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "imu/imu.h"
#include "imu/qmi8658.h"
#include "link_task.h"
#include "nvs_flash.h"

#include "cal_store.h"
#include "policy/mlp.h"
#include "scs_port_idf.h"
#include "shared.h"
#include "wifi_link.h"

namespace robot {

// -- the shared state declared in shared.h ---------------------------------
std::atomic<Mode> g_mode_request{Mode::kBench};
std::atomic<bool> g_ctrl_owns_bus{false};
std::atomic<uint32_t> g_link_arm_edges{0};
std::atomic<uint32_t> g_link_home_edges{0};
std::atomic<bool> g_link_arm_level{false};
std::atomic<uint8_t> g_arm_result{
    static_cast<uint8_t>(linkproto::ArmResult::kNone)};
std::atomic<float> g_shape_hz{obs::kShaperPoleHz};
std::atomic<uint32_t> g_obs_freeze{0};
QueueHandle_t g_cmd_mailbox = nullptr;
TelemetrySnapshot g_telemetry;
ObsDump g_obs_dump;
JointPose g_joint_pose;
scsbus::Bus* g_bus = nullptr;

namespace {
constexpr const char* kTag = "bimo";

// Statically constructed, never freed.
IdfPort g_port(board::kServoUart, board::kServoTx, board::kServoRx,
               board::kServoBaud);
scsbus::Bus g_bus_obj(g_port);

// The real part is the QMI8658C on the board itself. The stub stays as the
// fallback and is not vestigial: if the IMU does not answer -- unpopulated
// bus, a wiring fault, a board revision that moved it -- the loop still runs
// end to end reporting level and still, which is exactly what bench work
// needs. What must NOT happen is a robot that boots believing a dead sensor,
// so which one is live is logged at boot and reported by `stat`.
imu::Qmi8658Imu g_qmi(imu::Qmi8658Imu::Config{
    board::kI2cSda, board::kI2cScl, static_cast<uint32_t>(board::kI2cHz),
    board::kImuAddr, true, 8, 1024});
imu::StubImu g_stub;
imu::Imu* g_imu = nullptr;

}  // namespace

// `imu reinit` (cli.cpp): recover + re-init the QMI at runtime. The sampler
// task and ctrl hold g_imu; if we booted on the stub, swap the pointer once
// the real sensor answers (the sampler dereferences it every 4 ms, so the
// swap is a single aligned pointer store).
// Bias, mount and per-axis gyro scale from NVS onto the live QMI. Called at
// boot when the part answered, and again from imuReinitLive(): a boot that
// fell back to the stub never applied them, and a later `imu reinit` used
// to bring the sensor back UNCALIBRATED (2026-09-03: bias 0, mount identity,
// 27 deg/s of raw x offset straight into the filter) with nothing but the
// `imu` status line to say so.
void applyImuCalFromNvs() {
    float bias[3] = {0.0f, 0.0f, 0.0f};
    float mount[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    {
        float gs[3];
        if (imuGyroScaleLoad(gs)) g_qmi.setGyroScale(gs);
    }
    g_imu_cal_from_nvs = imuCalLoad(bias, mount);
    if (g_imu_cal_from_nvs) {
        g_qmi.setBias(bias);
        imu::Mount m;
        m.w = mount[0]; m.x = mount[1]; m.y = mount[2]; m.z = mount[3];
        g_qmi.setMount(m);
    }
}

bool imuReinitLive() {
    const bool ok = g_qmi.reinit();
    if (ok) {
        g_imu = &g_qmi;
        imuSamplerSetSource(g_qmi);
        applyImuCalFromNvs();
    }
    return ok;
}

bool g_imu_cal_from_nvs = false;

namespace {

}  // namespace

// Exposed for the bring-up CLI; null while the loop is on the stub.
imu::Qmi8658Imu* onboardImu() {
    return (g_imu == static_cast<imu::Imu*>(&g_qmi)) ? &g_qmi : nullptr;
}

namespace {

bool initHostUart() {
    uart_config_t cfg = {};
    cfg.baud_rate = board::kHostBaud;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    if (uart_param_config(board::kHostUart, &cfg) != ESP_OK) return false;
    return uart_driver_install(board::kHostUart, 512, 512, 0, nullptr, 0) ==
           ESP_OK;
}

}  // namespace
}  // namespace robot

extern "C" void app_main(void) {
    using namespace robot;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    if (!initHostUart()) {
        ESP_LOGE(kTag, "UART0 init failed");
        return;
    }
    // The tether carries binary link frames as well as CLI text, and IDF's
    // logger writes to the same UART. Keep logs for the boot sequence, then
    // silence them so nothing interleaves with a telemetry frame.
    if (!g_port.begin()) {
        ESP_LOGE(kTag, "servo UART1 init failed");
        return;
    }
    g_bus = &g_bus_obj;

    const bool imu_ok = g_qmi.init();
    g_imu = imu_ok ? static_cast<imu::Imu*>(&g_qmi)
                   : static_cast<imu::Imu*>(&g_stub);
    if (!imu_ok) g_stub.init();

    // Gyro bias and the mounting rotation, both from NVS. Neither has a safe
    // default: an uncalibrated bias parks the attitude estimate tens of
    // degrees off (see cal_store.h), and an identity mount is only right if
    // the board happens to lie flat, which on this robot it does not. So a
    // missing blob is REPORTED, not papered over -- `imu` shows it too.
    if (imu_ok) applyImuCalFromNvs();

    // Restore servo calibration before anything can command a position. A
    // missing blob is the normal first-boot path -- defaults (zero 2048,
    // dir +1) are what a freshly "Set Middle Position"-ed servo gives you --
    // but `cal` reports which of the two you are running on, because the
    // difference is invisible until a leg moves the wrong way.
    g_cal_from_nvs = calLoad(calibration());
    bool cal_migrated = false;
    if (!g_cal_from_nvs) {
        // v1 -> v2, automatically and safely: a v1 blob that exactly matches
        // the compiled as-built table (asbuilt_cal.h, the servo-map.md
        // measurement) IS that measurement, so it migrates without an
        // operator; anything else stays rejected and `run` refuses.
        cal_migrated = calMigrateV1(calibration());
        g_cal_from_nvs = cal_migrated;
    }

    // Bench mode, torque off: docs/wiring.md's bring-up order starts with a
    // released bus, and a board that wakes up holding a pose is a board that
    // cooks servos while you are still plugging things in.
    g_bus->torqueEnable(scsbus::kBroadcastId, false);

    ESP_LOGI(kTag, "cal: %s",
             cal_migrated ? "migrated v1 -> v2 (matched as-built table)"
                          : (g_cal_from_nvs ? "restored from NVS"
                                            : "DEFAULTS"));
    ESP_LOGI(kTag, "imu: %s, cal %s", g_imu->name(),
             g_imu_cal_from_nvs ? "from NVS" : "MISSING (run `imu bias`)");
    ESP_LOGI(kTag, "bus %d baud on GPIO %d/%d, %d joints, obs %d, policy %s",
             board::kServoBaud, static_cast<int>(board::kServoTx),
             static_cast<int>(board::kServoRx), obs::kNumJoints, obs::kObsDim,
             policy::kWeightsArePlaceholder ? "PLACEHOLDER" : obs::kRunName);
    startLinkTask();

    // After startLinkTask (the mailbox must exist), before the log silence
    // (so a driver-init failure is visible on the tether). Association and
    // DHCP finish asynchronously; `wifi` reports the result.
    {
        char ssid[33], psk[65];
        const bool creds = wifiCredsLoad(ssid, sizeof ssid, psk, sizeof psk);
        ESP_LOGI(kTag, "wifi: %s%s",
                 creds ? "joining " : "no credentials (`wifi <ssid> <psk>`)",
                 creds ? ssid : "");
    }
    startWifiLink();
    esp_log_level_set("*", ESP_LOG_NONE);

    startHousekeepingTask();
    startImuSampler(*g_imu);
    startCtrlTask(*g_imu);
}

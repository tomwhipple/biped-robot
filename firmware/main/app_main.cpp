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
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "imu/imu.h"
#include "link_task.h"
#include "nvs_flash.h"

#include "cal_store.h"
#include "policy/mlp.h"
#include "scs_port_idf.h"
#include "shared.h"

namespace robot {

// -- the shared state declared in shared.h ---------------------------------
std::atomic<Mode> g_mode_request{Mode::kBench};
std::atomic<bool> g_ctrl_owns_bus{false};
std::atomic<float> g_shape_hz{obs::kShaperPoleHz};
QueueHandle_t g_cmd_mailbox = nullptr;
TelemetrySnapshot g_telemetry;
scsbus::Bus* g_bus = nullptr;

namespace {
constexpr const char* kTag = "bimo";

// Statically constructed, never freed.
IdfPort g_port(board::kServoUart, board::kServoTx, board::kServoRx,
               board::kServoBaud);
scsbus::Bus g_bus_obj(g_port);

// The BNO085 driver is a v2 seam (firmware/README.md). Until the part is in
// hand and the SH-2 component lands, the stub keeps the whole loop runnable.
imu::StubImu g_imu;

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
    g_imu.init();

    // Restore servo calibration before anything can command a position. A
    // missing blob is the normal first-boot path -- defaults (zero 2048,
    // dir +1) are what a freshly "Set Middle Position"-ed servo gives you --
    // but `cal` reports which of the two you are running on, because the
    // difference is invisible until a leg moves the wrong way.
    g_cal_from_nvs = calLoad(calibration());

    // Bench mode, torque off: docs/wiring.md's bring-up order starts with a
    // released bus, and a board that wakes up holding a pose is a board that
    // cooks servos while you are still plugging things in.
    g_bus->torqueEnable(scsbus::kBroadcastId, false);

    ESP_LOGI(kTag, "cal: %s", g_cal_from_nvs ? "restored from NVS" : "DEFAULTS");
    ESP_LOGI(kTag, "bus %d baud on GPIO %d/%d, %d joints, obs %d, policy %s",
             board::kServoBaud, static_cast<int>(board::kServoTx),
             static_cast<int>(board::kServoRx), obs::kNumJoints, obs::kObsDim,
             policy::kWeightsArePlaceholder ? "PLACEHOLDER" : obs::kRunName);
    esp_log_level_set("*", ESP_LOG_NONE);

    startLinkTask();
    startHousekeepingTask();
    startCtrlTask(g_imu);
}

#include "scs_port_idf.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace robot {
namespace {
// One SYNC READ of 10 servos is ~10 * 19 bytes of replies; 512 B of RX FIFO is
// four ticks of slack. The TX side is written synchronously.
constexpr int kRxBuf = 512;
constexpr int kTxBuf = 0;      // 0 = uart_write_bytes blocks until queued
}  // namespace

bool IdfPort::begin() {
    uart_config_t cfg = {};
    cfg.baud_rate = baud_;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;
    if (uart_param_config(port_, &cfg) != ESP_OK) return false;
    if (uart_set_pin(port_, tx_, rx_, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    // No event queue: the control loop polls inside its own tick budget rather
    // than being woken, so nothing on core 0 can be scheduled by bus traffic.
    return uart_driver_install(port_, kRxBuf, kTxBuf, 0, nullptr, 0) == ESP_OK;
}

bool IdfPort::write(const uint8_t* data, size_t len) {
    const int n = uart_write_bytes(port_, reinterpret_cast<const char*>(data),
                                   len);
    return n == static_cast<int>(len);
}

size_t IdfPort::read(uint8_t* out, size_t cap, uint32_t timeout_us) {
    // FreeRTOS ticks are 1 ms; a sub-millisecond wait would round to 0 and
    // busy-spin the caller, so round UP to at least one tick.
    const TickType_t ticks =
        static_cast<TickType_t>((timeout_us + 999) / 1000 / portTICK_PERIOD_MS);
    const int n = uart_read_bytes(port_, out, cap, ticks ? ticks : 1);
    return n < 0 ? 0u : static_cast<size_t>(n);
}

void IdfPort::flushInput() { uart_flush_input(port_); }

uint64_t IdfPort::nowUs() const {
    return static_cast<uint64_t>(esp_timer_get_time());
}

}  // namespace robot

// scsbus::Port over an ESP-IDF UART. The only hardware-touching part of the
// servo stack; everything above it is host-tested.
#pragma once
#include "driver/uart.h"
#include "scsbus/bus.h"

namespace robot {

class IdfPort : public scsbus::Port {
  public:
    IdfPort(uart_port_t port, int tx_gpio, int rx_gpio, int baud)
        : port_(port), tx_(tx_gpio), rx_(rx_gpio), baud_(baud) {}

    // Installs the UART driver. Called once, from app_main, before any task
    // that uses the bus exists (no heap after init).
    bool begin();

    bool write(const uint8_t* data, size_t len) override;
    size_t read(uint8_t* out, size_t cap, uint32_t timeout_us) override;
    void flushInput() override;
    uint64_t nowUs() const override;

  private:
    uart_port_t port_;
    int tx_;
    int rx_;
    int baud_;
};

}  // namespace robot

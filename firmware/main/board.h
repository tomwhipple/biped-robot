// Waveshare "General Driver for Robots" board map (rev 1.2).
//
// This REPLACES the "Servo Driver with ESP32" map that stood here until
// 2026-08-14. The two boards are not interchangeable and the differences are
// not cosmetic:
//
//   * The servo bus is the ONE thing that survives unchanged: UART1 at 1
//     Mbaud, GPIO 18 = RX and GPIO 19 = TX, half-duplex switched in hardware
//     off the TX line with no direction GPIO. That is why the firmware ran on
//     this board before any of the rest of this header was true -- the bus is
//     the only part of it the code actually drove.
//
//   * I2C moves from GPIO 21/22 to GPIO 32 (SDA) / 33 (SCL), and it is no
//     longer an empty bus. On-board: QMI8658C 6-axis IMU at 0x6B, AK09918C
//     magnetometer at 0x0C, BMP280 barometer at 0x77, INA219 power monitor at
//     0x42. Header P1 brings the same bus out; 0x4A/0x4B stay reserved for a
//     BNO085 on P1 (docs/sensor-expansion.md).
//
//   * There is NO OLED. The old board's SSD1306 at 0x3C does not exist here,
//     so the display constants are gone rather than wrong.
//
//   * There IS a battery-voltage sense. The old header stated flatly that
//     the board had no ADC and that pack voltage therefore had to come off
//     the servo bus at 0.1 V resolution; the INA219 at 0x42 makes that false.
//     battguard still reads the servo bus today -- switching it is a separate
//     job (the INA219 shunt sits between DC_IN and the buck's VIN, so it sees
//     pack voltage even with the servo bus released, which is strictly better
//     than register 62).
//
//   * The RGB LED moves from GPIO 23 to GPIO 4 (header H2, through a 10 R
//     series resistor).
//
//   * TWO USB-C ports, both CP2102N, and only ONE of them flashes anything.
//     The port silkscreened "USB" carries the ESP32 console with the
//     DTR/RTS auto-program circuit; the one silkscreened "LIDAR" is a
//     separate bridge to P_TX/P_RX for a host computer. Plugging into LIDAR
//     still enumerates a /dev/ttyUSB*, which is the trap -- it just never
//     syncs. Confirmed on the bench 2026-08-14: MAC 30:76:f5:7e:55:ec,
//     ESP32-D0WD-V3 rev 3.1.
//
// Sources: the vendor schematic (docs/datasheets/general-driver/
// General_Driver_for_Robots-schematic.pdf) and the connector/net map derived
// from it in docs/sensor-expansion.md section 1. Waveshare publishes no user
// manual for this board.
#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"

namespace board {

// -- servo bus (UART1) -----------------------------------------------------
constexpr uart_port_t kServoUart = UART_NUM_1;
constexpr gpio_num_t kServoRx = GPIO_NUM_18;
constexpr gpio_num_t kServoTx = GPIO_NUM_19;
constexpr int kServoBaud = 1000000;
// No direction pin: the board switches the transceiver from the TX line.
constexpr gpio_num_t kServoDirection = GPIO_NUM_NC;

// -- host link / console (UART0 over the "USB" CP2102N) --------------------
constexpr uart_port_t kHostUart = UART_NUM_0;
constexpr int kHostBaud = 115200;

// -- I2C -------------------------------------------------------------------
constexpr gpio_num_t kI2cSda = GPIO_NUM_32;
constexpr gpio_num_t kI2cScl = GPIO_NUM_33;
// 400 kHz is the parts' ceiling, but the QMI8658C sits behind an LSF0204PWR
// level shifter whose rise time is set by the bus pull-ups; if the bench scan
// is flaky at 400 k, drop to 100 k before suspecting the driver.
constexpr int kI2cHz = 400000;

constexpr uint8_t kImuAddr = 0x6B;           // QMI8658C (SA0 high)
constexpr uint8_t kImuAddrAlt = 0x6A;        // QMI8658C (SA0 low)
constexpr uint8_t kMagAddr = 0x0C;           // AK09918C -- unused, see below
constexpr uint8_t kBaroAddr = 0x77;          // BMP280 -- unused
constexpr uint8_t kPowerAddr = 0x42;         // INA219 -- unused (battguard TODO)

// The magnetometer is deliberately NOT used. The obs frame needs an up-vector
// and body rates, never a heading, and a magnetometer bolted to the same PCB
// as eight ST3215s drawing ~10 A transient measures those servos, not north.
// It stays on the bus and stays unread.

// -- misc ------------------------------------------------------------------
constexpr gpio_num_t kRgbLed = GPIO_NUM_4;   // H2, WS2812 through 10 R
constexpr gpio_num_t kBootButton = GPIO_NUM_0;

// -- power limits (docs/wiring.md) -----------------------------------------
// Inlet is the XH2.54 2-pin (connector 10), switched by connector 12, fed
// straight through to the servo rail. The wiki says 7-13 V (2S or 3S); the
// rev1.2 silkscreen says "DC 9-12.6V", which would exclude 2S. Our 3S 11.1 V
// pack is inside both, so the contradiction is unresolved but not load-
// bearing -- see docs/datasheets/general-driver/README.md.
//
// Land the robot at 3.5 V/cell: 10.5 V on 3S, 7.0 V on 2S.
//
// These are the ADVISORY thresholds -- what a human is told to do. The robot
// enforces its own in battguard::kWarn3S / kLand3S (decivolts, because the
// sense is the servos' register 62). They must agree: kWarn3S == 105 is this
// same 10.5 V. battguard is pure and cannot include this header, so the link
// is this comment.
constexpr float kLandVoltage3s = 10.5f;
constexpr float kLandVoltage2s = 7.0f;

}  // namespace board

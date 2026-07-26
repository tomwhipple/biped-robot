// Waveshare "Servo Driver with ESP32" board map.
//
// CONFIRMED 2026-07-26 against the vendor's own schematic and firmware, not
// guessed:
//   schematic  https://files.waveshare.com/wiki/Servo-Driver-with-ESP32/Servo_Driver_with_ESP32.pdf
//   bus sheet  https://files.waveshare.com/upload/d/d3/Bus_servo_control_circuit.pdf
//   manual     https://files.waveshare.com/upload/d/d4/Servo_Driver_with_ESP32_User_Manual.pdf
//   firmware   https://github.com/waveshare/Servo-Driver-with-ESP32
//              (ServoDriver.ino / STSCTRL.h / BOARD_DEV.h)
//
// Findings that matter, including where they contradict our docs:
//
//  * Servo bus is UART1 at 1 Mbaud with GPIO 18 = RX and GPIO 19 = TX.
//    docs/wiring.md says "UART1, GPIO 18/19" -- correct, and the direction is
//    18 RX / 19 TX.
//
//  * There is NO direction-enable GPIO. Half-duplex switching is done in
//    hardware: TX idling high holds a PNP (Q1) off, and TXEN drives the OE
//    pins of a '126 (TX->DATA) and a '125 (DATA->RX). TXEN is not routed to
//    the ESP32 at all. Consequence for us: we never see our own transmission,
//    so there is no echo to strip -- and no pin to get wrong.
//
//  * I2C on GPIO 21 (SDA) / 22 (SCL) carries the SSD1306 OLED at 0x3C
//    (128x32). docs/wiring.md calls this bus "shared with the IMU" -- there is
//    no IMU on this board; the BNO085 is an external breakout on the same two
//    pins. (wiring.md also still describes a BNO055 at 0x28; firmware-design
//    section 3 supersedes it with the BNO085 at 0x4A/0x4B.)
//
//  * There is NO battery-voltage ADC. Every ADC-capable pin is unconnected and
//    the vendor firmware contains no analogRead(); the "V:" on its OLED is the
//    SERVO's own reported voltage (STS register 62, 0.1 V units). So pack
//    voltage in our telemetry comes off the bus, at 0.1 V resolution, which is
//    still fine against the 10.5 V land-the-robot threshold in wiring.md.
//
//  * GPIO 23 drives two on-board WS2812B RGB LEDs (chained out to header H1).
//    GPIO 0 is the boot button. EN is a hardware reset, not a readable input.
//    Nothing else is connected, and no GPIOs are broken out.
//
//  * Module is an ESP-32S / WROOM-32-class part, 4 MB flash, PSRAM disabled
//    (the vendor's mandated Arduino build settings). The exact silkscreen
//    variant is UNVERIFIED -- check the physical board.
//
//  * The board SHIPS WITH THE SC-SERIES BUILD of the vendor firmware, not the
//    ST one; its web UI (AP "ESP32_DEV" / "12345678" at 192.168.4.1) is what
//    bring-up uses to set servo IDs today. Its boot scan only covers IDs 0-20.
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

// -- host link / console (UART0 over the CP2102) ---------------------------
constexpr uart_port_t kHostUart = UART_NUM_0;
constexpr int kHostBaud = 115200;

// -- I2C: SSD1306 OLED, and the external BNO085 breakout -------------------
constexpr gpio_num_t kI2cSda = GPIO_NUM_21;
constexpr gpio_num_t kI2cScl = GPIO_NUM_22;
constexpr int kI2cHz = 400000;
constexpr uint8_t kOledAddr = 0x3C;
constexpr int kOledWidth = 128;
constexpr int kOledHeight = 32;
constexpr uint8_t kImuAddrPrimary = 0x4A;    // BNO085 SA0 low
constexpr uint8_t kImuAddrAlt = 0x4B;        // BNO085 SA0 high

// -- misc ------------------------------------------------------------------
constexpr gpio_num_t kRgbLed = GPIO_NUM_23;  // 2 on-board WS2812B, chained out
constexpr gpio_num_t kBootButton = GPIO_NUM_0;

// -- power limits (docs/wiring.md) -----------------------------------------
// Board input 6-12.6 V, fed straight through to the servo rail. Land the robot
// at 3.5 V/cell: 10.5 V on 3S, 7.0 V on 2S.
constexpr float kLandVoltage3s = 10.5f;
constexpr float kLandVoltage2s = 7.0f;

}  // namespace board

// Feetech ST3215 (STS/SMS series) control-table addresses.
//
// Source of truth: the ST3215 memory register map v3.7 spreadsheet and the
// Feetech "Communication Protocol User Manual" that Waveshare links from
// https://www.waveshare.com/wiki/ST3215_Servo :
//   https://files.waveshare.com/upload/2/27/ST3215%20memory%20register%20map-EN.xls
//   https://files.waveshare.com/upload/2/27/Communication_Protocol_User_Manual-EN(191218-0923).pdf
// Cross-checked against the SCServo library bundled with Waveshare's board
// firmware (https://github.com/waveshare/Servo-Driver-with-ESP32).
//
// Everything two bytes wide is LITTLE-endian on STS/SMS parts (the magnetic-
// encoder line). The older potentiometer SCS line is big-endian -- the vendor
// library flips one flag (`End`) between them, and getting it backwards is the
// classic first-day bring-up bug, so it lives in one place here (Endian).
#pragma once
#include <stdint.h>

namespace scsbus {

// -- framing ---------------------------------------------------------------
constexpr uint8_t kHeader = 0xFF;      // two of these start every packet
constexpr uint8_t kBroadcastId = 0xFE; // 0xFE addresses every servo at once
constexpr uint8_t kMaxId = 0xFD;       // 0..253 are assignable

// -- instructions ----------------------------------------------------------
enum class Inst : uint8_t {
    kPing = 0x01,
    kRead = 0x02,
    kWrite = 0x03,
    kRegWrite = 0x04,   // staged write, applied by kAction
    kAction = 0x05,
    kReset = 0x06,      // factory reset (also resets the ID -- careful)
    kSyncRead = 0x82,
    kSyncWrite = 0x83,
};

// -- control table ---------------------------------------------------------
// EEPROM, read-only.
enum : uint8_t {
    kRegFirmwareMajor = 0,
    kRegFirmwareMinor = 1,
    kRegServoMajor = 3,
    kRegServoMinor = 4,
};

// EEPROM, read/write. Writes only stick while the lock flag (reg 55) is 0.
enum : uint8_t {
    kRegId = 5,              // 1 B, 0..253
    kRegBaud = 6,            // 1 B, 0=1M 1=500k 2=250k 3=128k 4=115200 ...
    kRegReturnDelay = 7,     // 1 B, units of 2 us
    kRegResponseLevel = 8,   // 1 B, 0 = reply only to READ/PING, 1 = reply to all
    kRegMinAngleLimit = 9,   // 2 B, steps; 0 in both limits = multi-turn mode
    kRegMaxAngleLimit = 11,  // 2 B, steps
    kRegMaxTemperature = 13, // 1 B, degC
    kRegMaxVoltage = 14,     // 1 B, 0.1 V
    kRegMinVoltage = 15,     // 1 B, 0.1 V
    kRegMaxTorque = 16,      // 2 B, 0.1 %; copied into reg 48 at power-on
    kRegPhase = 18,          // 1 B, magic byte (BIT4 must be set for multi-turn)
    kRegUnloadCondition = 19,// 1 B, same bit layout as kRegStatus
    kRegLedCondition = 20,   // 1 B, same bit layout as kRegStatus
    kRegPosP = 21,           // 1 B
    kRegPosD = 22,           // 1 B
    kRegPosI = 23,           // 1 B
    kRegMinStartForce = 24,  // 2 B, 0.1 %
    kRegCwDeadZone = 26,     // 1 B, steps
    kRegCcwDeadZone = 27,    // 1 B, steps
    kRegProtectCurrent = 28, // 2 B, 6.5 mA/LSB
    kRegAngleResolution = 30,// 1 B, 1..3
    kRegPosCorrection = 31,  // 2 B, +-2047 steps, SIGN IN BIT11 (not bit15)
    kRegOperationMode = 33,  // 1 B, 0=position 1=speed 2=PWM 3=step
    kRegProtectTorque = 34,  // 1 B, 1 %
    kRegProtectTime = 35,    // 1 B, 10 ms
    kRegOverloadTorque = 36, // 1 B, 1 %
    kRegSpeedP = 37,         // 1 B
    kRegOvercurrentTime = 38,// 1 B, 10 ms
    kRegSpeedI = 39,         // 1 B
};

// SRAM, read/write.
enum : uint8_t {
    kRegTorqueEnable = 40,   // 1 B, 0=off 1=on 128=calibrate here as 2048
    kRegAcceleration = 41,   // 1 B, 100 steps/s^2 per LSB
    kRegGoalPosition = 42,   // 2 B, steps (sign in BIT15 for multi-turn)
    kRegGoalTime = 44,       // 2 B, ms for the move (0 = use goal speed)
    kRegGoalSpeed = 46,      // 2 B, steps/s, <= 3400
    kRegTorqueLimit = 48,    // 2 B, 0.1 %
    kRegEepromLock = 55,     // 1 B, 0 = UNLOCKED (writes persist), 1 = locked
};

// SRAM, read-only. 56..70 is one contiguous feedback block, which is why the
// vendor library reads it in a single 15-byte transaction.
enum : uint8_t {
    kRegPresentPosition = 56,    // 2 B, steps (sign in BIT15)
    kRegPresentSpeed = 58,       // 2 B, steps/s (sign in BIT15)
    kRegPresentLoad = 60,        // 2 B, 0.1 % duty (sign in BIT10)
    kRegPresentVoltage = 62,     // 1 B, 0.1 V  <-- the only pack-voltage sense
                                 //     we have; the board has no ADC divider.
    kRegPresentTemperature = 63, // 1 B, degC
    kRegAsyncWriteFlag = 64,     // 1 B
    kRegStatus = 65,             // 1 B, error bits (see ErrorBit)
    kRegMovingFlag = 66,         // 1 B
    kRegPresentCurrent = 69,     // 2 B, 6.5 mA/LSB
};

// The write-once torque-enable magic value that latches the current shaft
// position as 2048 (the vendor web UI's "Set Middle Position"). Confirmed in
// SCServo's CalibrationOfs().
constexpr uint8_t kTorqueCalibrateMiddle = 128;

// -- error flags -----------------------------------------------------------
// Identical bit layout in the response header's ERROR byte, in reg 65
// (status), reg 19 (unload conditions) and reg 20 (LED alarm conditions).
enum ErrorBit : uint8_t {
    kErrVoltage = 1u << 0,
    kErrSensor = 1u << 1,
    kErrTemperature = 1u << 2,
    kErrCurrent = 1u << 3,
    kErrAngle = 1u << 4,
    kErrOverload = 1u << 5,
};

// -- units -----------------------------------------------------------------
constexpr int32_t kStepsPerRev = 4096;      // 0.087890625 deg/step
constexpr int32_t kCenterSteps = 2048;      // the "middle" the calibration sets
constexpr float kVoltsPerLsb = 0.1f;        // reg 62
constexpr float kAmpsPerLsb = 0.0065f;      // regs 28 and 69
constexpr int32_t kMaxGoalSpeed = 3400;     // steps/s at 7.4 V, no load

// -- signed-magnitude helpers ----------------------------------------------
// Feetech does not use two's complement: the top bit of the field is a sign
// bit and the rest is magnitude. Position/speed use bit 15; load uses bit 10;
// the EEPROM position correction (reg 31) uses bit 11.
constexpr int32_t signMag(uint16_t raw, int sign_bit) {
    const uint16_t mask = static_cast<uint16_t>((1u << sign_bit) - 1u);
    return (raw & (1u << sign_bit)) ? -static_cast<int32_t>(raw & mask)
                                    : static_cast<int32_t>(raw & mask);
}

constexpr uint16_t toSignMag(int32_t v, int sign_bit) {
    const uint16_t mask = static_cast<uint16_t>((1u << sign_bit) - 1u);
    const uint32_t mag = static_cast<uint32_t>(v < 0 ? -v : v) & mask;
    return static_cast<uint16_t>(v < 0 ? (mag | (1u << sign_bit)) : mag);
}

}  // namespace scsbus

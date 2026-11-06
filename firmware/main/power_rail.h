// The power-lite servo rail, hardware half: IO16 SERVO_EN, IO4 ESTOP_N, GPIO34
// RAIL_V. The decisions are components/railguard's; this file reads the pins,
// drives IO16, and carries requests and state across the tasks.
//
// A BUILD OPTION, off by default (`make firmware POWER_LITE=1`,
// -DBIMO_POWER_LITE=1; firmware/CMakeLists.txt). On a robot without power lite,
// IO4 is the WS2812 net, IO16 is unconnected and GPIO34 reads nothing, and this
// cannot be detected at run time: a floating or unplugged lead must read as stop.
// With the option off, every call site is an `if constexpr (rail::kFitted)` or, in
// cli.cpp, an `#if`: nothing touches these pins, and the image is the same as one
// built without any of this. `make firmware-powerlite`, a pre-push gate, keeps the
// other side compiling.
//
// ONE WRITER: ctrl owns the Guard and IO16, and calls ctrlTick() every tick in both
// modes. Housekeeping asks through requestOn() / requestOff() and reads snapshot(),
// as with the mode and the ARM edges (shared.h). A blocked housekeeping task (it
// waits here during an arm) therefore cannot delay an E-stop.
//
// THE PIN FAILS SAFE. R2 holds Q1 off while IO16 floats, through a power-on reset,
// a hardware reset and a system reset (a brown-out takes that path:
// esp_rom_software_reset_system). But esp_restart_noos is only a CPU reset: the GPIO
// registers keep their values, so IO16 would stay HIGH through the panic printout
// and the bootloader, until app_main. So boot() is app_main's first line, and the
// linker wraps esp_panic_handler and esp_restart_noos to take IO16 low from IRAM on
// the way out: a panic, the interrupt and task watchdogs, esp_restart(). The task
// watchdog panics in this build, or a wedged ctrl would hold the rail on forever (it
// only warns by default).
//
// Boot comes up with the rail off and no latch; only a fresh ARM edge or `run`
// powers it (linkproto::ArmLatch's first frame makes no edge).
#pragma once
#include <atomic>
#include <stdint.h>

#include "linkproto/protocol.h"
#include "railguard/railguard.h"

#ifndef BIMO_POWER_LITE
#define BIMO_POWER_LITE 0
#endif

namespace robot {
namespace rail {

constexpr bool kFitted = BIMO_POWER_LITE != 0;

struct Snapshot {
    railguard::State state = railguard::State::kOff;
    railguard::Fault fault = railguard::Fault::kNone;
    railguard::DropWhy last_drop = railguard::DropWhy::kNone;
    bool servo_en = false;
    bool estop_pressed = true;
    bool estop_latched = false;
    bool rail_valid = false;
    uint16_t rail_mv = 0;
};

// -- app_main ------------------------------------------------------------------
// The first line of app_main: IO16 low, then output; IO4 input with the pull-up;
// GPIO34 on ADC1 at 11 dB; the task watchdog set to panic.
void boot();

// -- ctrl (core 1) ------------------------------------------------------------
// Top of every tick, either mode: read the pins, apply requests and hard operator
// stops, step the Guard, drive IO16, publish. A run cannot outlive its rail: this
// benches the loop if the bus is not up, with kRailDown (or kDisarmedStop).
void ctrlTick(uint32_t now_ms);
// SERVO_EN low on this very tick.
void dropFromCtrl(railguard::DropWhy why);
// A frame that disables: ENABLE, ARM and ESTOP all clear (Guard::disabledCommand).
void disabledFromCtrl();
// Every valid frame from the commander, in both modes (Guard::commanderFrame). ESTOP
// set: SERVO_EN low on this very tick, whoever powered the rail.
void commanderFrame(bool estop);

// -- housekeeping (core 0) ------------------------------------------------------
// Each waits for ctrl to apply it, at most a tick or two.
// Ask for the rail (kNoReading if ctrl does not answer, and the request is cancelled).
railguard::Verdict requestOn();
void requestOff(railguard::DropWhy why);
// Off, and counts as the disabled command that clears the E-stop latch.
void requestDisarm();
// `rail clear`: false while a stuck rail is still up, or if ctrl does not answer.
bool requestClearFault();

Snapshot snapshot();

// May the bus be used? Every bus path checks this (cli.cpp claimBus and friends).
extern std::atomic<bool> g_bus_up;
inline bool busUp() {
    if constexpr (kFitted) {
        return g_bus_up.load();
    } else {
        return true;
    }
}

}  // namespace rail
}  // namespace robot

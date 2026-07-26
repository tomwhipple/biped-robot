// The only state that crosses cores, and the rules for touching it.
//
// firmware-design section 4: every shared datum goes through a FreeRTOS queue
// or an atomic, there are no mutexes in the control path, and every shared
// object has exactly one writer.
#pragma once
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "linkproto/protocol.h"
#include "obs/obs_spec.h"
#include "scsbus/bus.h"

namespace robot {

// -- who owns the servo bus ------------------------------------------------
// The bring-up CLI runs on core 0 but needs the bus, which core 1 owns during
// a run. Rather than a mutex in the control path, the bus has a single owner
// selected by a mode that only ever changes between ticks:
//
//   kBench  ctrl does nothing but service its watchdog; the CLI drives the bus
//   kRun    ctrl owns the bus; CLI bus commands are refused
//
// Boot state is kBench, so a freshly powered board never moves on its own.
enum class Mode : uint8_t { kBench = 0, kRun };

// Written by the CLI (core 0), read by ctrl (core 1).
extern std::atomic<Mode> g_mode_request;
// Written by ctrl only. The CLI must see BOTH kBench requested and this false
// before it touches the bus.
extern std::atomic<bool> g_ctrl_owns_bus;

// -- command mailbox (core 0 -> core 1) ------------------------------------
// A length-1 queue written with xQueueOverwrite: latest command wins and stale
// commands can never pile up behind a late tick.
struct CommandMsg {
    linkproto::Command cmd;
    float rx_ms;         // when the frame arrived, on ctrl's clock
};
extern QueueHandle_t g_cmd_mailbox;

// -- telemetry snapshot (core 1 -> core 0) ---------------------------------
// Single writer (ctrl), single reader (housekeeping). Read may tear across
// fields; every consumer is a human-facing 10 Hz report, so that is fine and
// cheaper than a queue.
struct TelemetrySnapshot {
    std::atomic<uint32_t> seq_echo{0};
    std::atomic<uint8_t> state{0};
    std::atomic<uint16_t> vbat_mv{0};
    std::atomic<float> up_z{0.0f};
    std::atomic<float> vx_est{0.0f};
    std::atomic<float> wz_est{0.0f};
    std::atomic<uint16_t> servo_err{0};      // bit i = joint index i faulted
    std::atomic<uint8_t> loop_late_pct{0};
    std::atomic<uint32_t> ticks{0};
    std::atomic<uint32_t> overruns{0};
    std::atomic<uint32_t> worst_tick_us{0};
};
extern TelemetrySnapshot g_telemetry;

// The bus object itself: constructed once in app_main, used by ctrl during a
// run and by the CLI while benched. See g_mode_request for the handover rule.
extern scsbus::Bus* g_bus;

// Servo IDs in policy-action order, from the generated obs spec.
inline const uint8_t* servoIds() { return obs::kServoId; }
inline int numJoints() { return obs::kNumJoints; }

}  // namespace robot

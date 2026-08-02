// The only state that crosses cores, and the rules for touching it.
//
// firmware-design section 4: every shared datum goes through a FreeRTOS queue
// or an atomic, there are no mutexes in the control path, and every shared
// object has exactly one writer.
#pragma once
#include <atomic>

#include "battguard/guard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "linkproto/protocol.h"
#include "obs/actuation.h"
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

// -- command shaping pole (core 0 -> core 1) --------------------------------
// Written by the CLI (`shape <hz>`), read by ctrl every tick. The C2 command
// shaper's pole in Hz; 0 disables shaping (raw targets, max-speed slew --
// the pre-shaper behaviour, kept as a bench A/B and an escape hatch). Live-
// tunable: changing the pole mid-run is continuous because the shaper's state
// carries over; only the gain changes.
extern std::atomic<float> g_shape_hz;

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

    // Per-phase tick cost, last completed tick (us). firmware-design section 4
    // budgeted the whole tick at ~8-10 ms; the first hardware run measured 32
    // ms with every tick late and no servo faults, which none of the budget
    // lines explains. These say where it actually goes instead of guessing.
    std::atomic<uint32_t> us_read{0};    // sync-read 10 servos
    std::atomic<uint32_t> us_imu{0};     // IMU sample (stub today)
    std::atomic<uint32_t> us_obs{0};     // obs assembly + history
    std::atomic<uint32_t> us_net{0};     // policy forward
    std::atomic<uint32_t> us_write{0};   // sync-write targets
    std::atomic<uint32_t> us_other{0};   // tick total minus the above
};
extern TelemetrySnapshot g_telemetry;

// The bus object itself: constructed once in app_main, used by ctrl during a
// run and by the CLI while benched. See g_mode_request for the handover rule.
extern scsbus::Bus* g_bus;

// Live servo calibration, owned by the control task and restored from NVS at
// boot. `ctrl` only ever READS it; the CLI is the sole writer, and every
// mutating `cal` subcommand requires bench mode -- in that state ctrl has
// already handed back the bus and returns before touching g_cal, so there is
// no cross-core race and no mutex in the control path.
obs::Calibration& calibration();

// True when boot found a valid blob in NVS. Shown by `cal` so nobody mistakes
// freshly-defaulted values for a real calibration.
extern bool g_cal_from_nvs;

// The pack under-voltage guard, owned and written by the control task. The
// CLI's `batt` reads it (a torn read is a human-facing report, same rule as
// TelemetrySnapshot) and `batt reset` WRITES it -- which is why that one is
// bench-only, exactly like the mutating `cal` subcommands: in bench mode ctrl
// returns before touching the guard, so there is no cross-core race.
battguard::Guard& battGuard();

// Servo IDs in policy-action order, from the generated obs spec.
inline const uint8_t* servoIds() { return obs::kServoId; }
inline int numJoints() { return obs::kNumJoints; }

}  // namespace robot

// The only state that crosses cores, and the rules for touching it.
//
// firmware-design section 4: every shared datum goes through a FreeRTOS queue
// or an atomic, there are no mutexes in the control path, and every shared
// object has exactly one writer.
#pragma once
#include <atomic>

#include "battguard/guard.h"
#include "imu/qmi8658.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "linkproto/protocol.h"
#include "obs/actuation.h"
#include "obs/obs_spec.h"
#include "obs_dump.h"
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

// Written by the CLI (core 0), read by ctrl (core 1) -- plus ONE write from
// ctrl itself: the fall latch stores kBench when it trips (ctrl_task.cpp),
// because a fall must end the run without waiting on core 0. That write only
// ever moves the mode TOWARD bench, so the bus-ownership rule holds: the CLI
// still waits for g_ctrl_owns_bus to go false before touching the bus.
extern std::atomic<Mode> g_mode_request;
// Written by ctrl only. The CLI must see BOTH kBench requested and this false
// before it touches the bus.
extern std::atomic<bool> g_ctrl_owns_bus;

// -- link arm/disarm requests (core 1 -> core 0) ---------------------------
// ctrl runs linkproto::ArmLatch over every frame it drains, in BOTH modes,
// and publishes each ARM edge here; the housekeeping task turns it into the
// same cmdMode() call the tethered `run`/`bench` make. The mode therefore
// keeps its single writer, and -- because housekeeping only looks between
// CLI lines -- a WiFi arm can never land in the middle of a bench-mode bus
// command. A counter plus a level rather than a request flag, so that each
// atomic also has exactly one writer: ctrl bumps, housekeeping remembers
// what it has consumed.
extern std::atomic<uint32_t> g_link_arm_edges;
extern std::atomic<bool> g_link_arm_level;     // valid once edges > 0

// -- why the loop is (not) armed (core 0 -> the WiFi beacon) ---------------
// cmdMode() is the ONE place that decides a mode request, and until 2026-08-30
// its refusal only reached the UART sink: over WiFi the robot simply stayed
// BENCH and the operator learned nothing (that day, six seconds of ARM frames
// were diagnosed only by plugging the tether). cmdMode records its verdict
// here and wifi_link.cpp puts it in the BENCH beacon.
//
// Writers: cmdMode on the housekeeping task -- a typed `run`/`bench` arrives
// through cli::execute() and a wireless ARM edge through cli::linkMode(),
// both from that one loop -- plus ctrl's fall latch, which stores
// kDisarmedFall when a fall ends the run (ctrl_task.cpp) so the beacon says
// WHY the robot is benched. The reader is a 10 Hz human-facing beacon, so it
// follows the TelemetrySnapshot rule.
extern std::atomic<uint8_t> g_arm_result;      // a linkproto::ArmResult

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

// -- observation dump (core 1 -> core 0) -----------------------------------
// The same single-writer split as TelemetrySnapshot, but the payload is too
// big to tear harmlessly -- a diagnostic that mixes tick t's q with tick
// t+1's frame would invent the very sign/frame error it is there to find. So
// it is double-buffered with a sequence counter instead (obs_dump.h): ctrl
// writes the slot the reader is not in, housekeeping copies and re-checks the
// counter. No lock in the tick, no allocation, and a copy the writer overran
// is DROPPED rather than reported.
//
// ctrl writes the buffer and only reads the mode; housekeeping writes the
// mode (`obsdump on|off|once`) and only reads the buffer.
extern ObsDump g_obs_dump;

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
// freshly-defaulted values for a real calibration, and gate-kept by cmdMode:
// no calibration, no run.
//
// Atomic since 2026-08-30, when the WiFi beacon became a second reader on
// another task (the kDiagCalOk bit). Written only by the housekeeping task --
// the `cal` subcommands -- after app_main sets it before any task starts.
extern std::atomic<bool> g_cal_from_nvs;

// The pack under-voltage guard, owned and written by the control task. The
// CLI's `batt` reads it (a torn read is a human-facing report, same rule as
// TelemetrySnapshot) and `batt reset` WRITES it -- which is why that one is
// bench-only, exactly like the mutating `cal` subcommands: in bench mode ctrl
// returns before touching the guard, so there is no cross-core race.
battguard::Guard& battGuard();

// Servo IDs in policy-action order, from the generated obs spec.
inline const uint8_t* servoIds() { return obs::kServoId; }
inline int numJoints() { return obs::kNumJoints; }

// The on-board IMU, for the bring-up CLI. Null when the part did not answer
// at boot and the loop is running on the stub -- callers must check, because
// "no IMU" is a normal bench state, not an error.
//
// Same ownership rule as the servo bus: `imu` subcommands that WRITE (bias
// calibration, mount) are bench-only, because in bench mode ctrl is not
// reading the sensor and there is no cross-core race. Reads are reports and
// follow the TelemetrySnapshot rule -- a torn value is a human's problem, not
// the control loop's.
imu::Qmi8658Imu* onboardImu();

// True when boot found a valid IMU calibration in NVS. False means the gyro
// bias is zero and the mount is identity, which on this robot is wrong on
// both counts -- `imu` says so loudly rather than letting it pass.
extern bool g_imu_cal_from_nvs;

}  // namespace robot

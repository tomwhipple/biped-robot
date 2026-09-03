// Bring-up CLI over USB serial (UART0).
//
// This is what turns the board into a bench tool: assign servo IDs, check
// enumeration, read positions, jog a joint, set the middle position, release
// torque. It is the firmware side of docs/wiring.md's bring-up checklist,
// replacing the vendor web UI's job with something scriptable over the tether.
//
// It runs on core 0 at low priority and may only touch the servo bus while the
// control loop has handed it over (shared.h: Mode::kBench). Every bus command
// checks that.
#pragma once
#include <stddef.h>

namespace cli {

// Where output goes. A function pointer, not printf, so the CLI has no
// dependency on the console and can be exercised off-target.
using Sink = void (*)(const char* text);

// Execute one line. `line` is NUL-terminated and has no trailing newline.
void execute(const char* line, Sink out);

// The banner printed at boot and by `help`.
void banner(Sink out);

// `run` / `bench` on behalf of the wireless link (an ARM edge, see
// linkproto::ArmLatch). Called from the housekeeping loop, never from the
// link tasks. Same refusal as the typed `run`: no as-built calibration, no
// run -- the refusal is printed on the tether, and the client sees the robot
// stay BENCH in telemetry.
void linkMode(bool run, Sink out);

// The wireless "reset the servos" request (kFlagHome). Benches the loop,
// waits for the bus handover, then drives every joint to its calibrated zero
// -- the standing pose. Housekeeping-task only, like every other bus command
// here. See cmdHome for why this one deliberately turns torque back ON.
void linkHome(Sink out);
// Called from the housekeeping loop every iteration: once a home's slew
// deadline has passed, read every joint back and store the REAL verdict
// (kDisarmedHome / kHomeNotReached) in g_arm_result. No-op when nothing
// is pending; cancels itself if the loop got armed in the meantime.
void homeVerify(Sink out);

}  // namespace cli

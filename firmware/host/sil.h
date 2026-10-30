// libctrl_sil -- the C boundary of the software-in-the-loop harness.
//
// See docs/sil-harness.md. The sim hands this library exactly what the bus and
// the IMU driver deliver on the robot; the library runs the REAL firmware
// control path (obs::, policy::, the actuation/Calibration maps) and hands
// back exactly what a SYNC WRITE would put on the wire. Nothing in here or
// below knows it is talking to a simulator.
//
// Plain C layout so python can ctypes it. Every array is fixed-width for a
// given build, and the widths ARE the compiled policy's (obs_spec.h, so the
// deployed v5 build is 17/7/10/147 and the v6legs profile 17/7/12/165): one
// library per policy profile (firmware/host/Makefile POLICY). sil_spec()
// reports every width, and the python harness sizes its ctypes structs from
// the same obs_spec.h and refuses a library whose widths differ, so a layout
// change can never silently reinterpret the harness's bytes.
#ifndef BIMO_SIL_H
#define BIMO_SIL_H

#include <stdint.h>

#ifdef __cplusplus
#include "obs/bus_map.h"
#include "obs/obs_spec.h"
#define SIL_NUM_BUS obs::kNumBusJoints   // every servo on the bus (bus_map.h)
#define SIL_NUM_CMD obs::kNumCmd         // command channels
#define SIL_ACT_DIM obs::kActDim         // the policy's joints
#define SIL_OBS_DIM obs::kObsDim         // the stacked observation
#endif
#if !defined(SIL_NUM_BUS) || !defined(SIL_NUM_CMD) || \
    !defined(SIL_ACT_DIM) || !defined(SIL_OBS_DIM)
#error "a C reader of sil.h defines SIL_NUM_BUS, SIL_NUM_CMD, SIL_ACT_DIM and SIL_OBS_DIM to the widths sil_spec() reports"
#endif

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// ARRAY ORDER -- read this before writing a harness against the struct.
//
// pos_ticks, vel_ticks, goal_ticks and goal_speed cover EVERY servo on the
// robot's bus (obs/bus_map.h, 17 of them), in BUS INDEX order:
//
//     slot = obs::busIndexOfId(servo_id)  (bus index, NOT id - 1)
//     policy joint j  <->  slot obs::policyToBus(j)
//
// Slot 0 is R_hip_roll (servo 52), which is policy joint 6 under the
// deployed 10-joint spec and joint 7 under the 12-joint v6legs one -- the
// permutation docs/wiring.md once called an identity, and putting it on the
// C boundary is what makes it testable end to end. The slots of servos the
// policy does not drive (the ankle rolls under a 10-joint policy; the neck
// and arms under either) are sensed but not observed, and their goals HOLD:
// the run's trained target for a servo it holds (obs_spec.h kHeld*), else
// the position sensed on the first tick after sil_reset -- ctrl_task's hold,
// without its takeover ramp.
// sil_spec() reports "sensor_index":"bus_index", the kServoId array and the
// bus map, so the python side asserts the convention rather than assume it.
// ---------------------------------------------------------------------------

typedef struct {                       // what the bus + IMU actually deliver
    uint16_t pos_ticks[SIL_NUM_BUS];   // present position, reg 56 semantics
                                       //   (sign in BIT15, sign-magnitude),
                                       //   bus order
    int16_t vel_ticks[SIL_NUM_BUS];    // present speed, reg 58 semantics: the
                                       //   BIT15 sign-magnitude word, carried
                                       //   in an int16_t
    float up[3];                       // IMU up-vector, body frame
    float gyro[3];                     // rad/s, body frame
    float cmd[SIL_NUM_CMD];            // command channels (harness-supplied)
} SilSensors;

typedef struct {
    uint16_t goal_ticks[SIL_NUM_BUS];  // SYNC WRITE view, bus order. With
                                       //   shaping on (the default, as
                                       //   deployed) the policy's slots are
                                       //   the C2-shaped targets, NOT the raw
                                       //   action map; see sil_set_shaper.
    float action[SIL_ACT_DIM];         // the raw [-1,1] policy action
                                       //   (diagnostics)
    float obs[SIL_OBS_DIM];            // the assembled obs (diagnostics/golden)
    uint16_t goal_speed[SIL_NUM_BUS];  // per-servo goal speed (reg 46,
                                       //   steps/s) the SYNC WRITE carries,
                                       //   bus order; 0 when shaping is off
} SilTargets;

// Load an exported policy and a calibration, then reset.
//   weights_path  the .silw f32 blob; its JSON sidecar is found beside it.
//                 NULL or "" -> the weights compiled into the firmware
//                 (policy/weights.h), so the harness runs before the
//                 exporter exists.
//   cal_path      JSON [{bus_id, zero_steps, dir}, ...]: one entry per servo
//                 the policy drives at least, any bus servo at most. NULL or
//                 "" -> the nominal Calibration() (zero_steps 2048, dir +1).
//   gait_freq_hz  gait-clock frequency; <= 0 -> the 1.5 Hz firmware default.
// Returns 0 on success, negative on failure (see sil_last_error()).
int sil_init(const char* weights_path, const char* cal_path,
             float gait_freq_hz);

// History refill + clock zero: the mirror of what ctrl_task does when it
// (re)takes the bus. The next sil_tick refills the frame ring from its own
// first frame, exactly like walker_env's reset.
void sil_reset(void);

// One 20 ms control tick. Returns 0 on success, negative on failure.
int sil_tick(const SilSensors* in, SilTargets* out);

// A JSON description of the obs spec this library was built against: dims
// (every struct width above among them), frame offsets, the kServoId
// permutation, the policy profile, a hash of the layout for drift detection,
// and which policy weights are loaded. Valid before sil_init.
const char* sil_spec(void);

// Additive to the ABI in docs/sil-harness.md: the text of the last failure.
// Never NULL. Kept because "sil_init returned -2" is a bad debugging story
// when the sidecar path is the thing that is wrong.
const char* sil_last_error(void);

// sil_abi 3 (the 17-servo bus arrays above; 2 added this call): the C2
// command shaper's pole in Hz, the mirror of ctrl_task's
// g_shape_hz (CLI `shape`). sil_init resets it to the firmware default
// (obs::kShaperPoleHz -- boot state), so a harness that wants the legacy raw
// targets calls sil_set_shaper(0) after init. Changing the pole mid-run is
// continuous (shaper state carries over), exactly as on the robot; sil_reset
// reseeds the shaper from the next tick's sensed positions, the mirror of
// ctrl_task reseeding from measured q on a torque (re)engage.
void sil_set_shaper(float pole_hz);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // BIMO_SIL_H

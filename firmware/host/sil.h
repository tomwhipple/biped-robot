// libctrl_sil -- the C boundary of the software-in-the-loop harness.
//
// See docs/sil-harness.md. The sim hands this library exactly what the bus and
// the IMU driver deliver on the robot; the library runs the REAL firmware
// control path (obs::, policy::, the actuation/Calibration maps) and hands
// back exactly what a SYNC WRITE would put on the wire. Nothing in here or
// below knows it is talking to a simulator.
//
// Plain C so python can ctypes it. Every array is fixed-width and every width
// is static_assert-ed against obs_spec.h in sil_lib.cpp, so an obs-layout
// retrain that changes a dimension breaks the build instead of silently
// reinterpreting the harness's bytes.
#ifndef BIMO_SIL_H
#define BIMO_SIL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// ARRAY ORDER -- read this before writing a harness against the struct.
//
// pos_ticks, vel_ticks and goal_ticks are indexed by BUS ID:
//
//     slot = bus_id - 1              (bus IDs are 1..10 on this robot)
//     joint j  <->  slot obs::kServoId[j] - 1
//
// kServoId is {9,1,2,3,4,10,5,6,7,8}, so slot 0 is servo ID 1 == L_hip_roll
// == joint 1, NOT joint 0. This is the permutation docs/wiring.md wrongly
// called an identity, and putting it on the C boundary is what makes it
// testable end to end. sil_spec() reports "sensor_index":"bus_id-1" plus the
// kServoId array so the python side can assert the convention rather than
// assume it.
// ---------------------------------------------------------------------------

typedef struct {              // what the bus + IMU actually deliver
    uint16_t pos_ticks[10];   // present position, reg 56 semantics (sign in
                              //   BIT15, sign-magnitude), bus-ID order
    int16_t vel_ticks[10];    // present speed, reg 58 semantics: the BIT15
                              //   sign-magnitude word, carried in an int16_t
    float up[3];              // IMU up-vector, body frame
    float gyro[3];            // rad/s, body frame
    float cmd[7];             // command channels (harness-supplied)
} SilSensors;

typedef struct {
    uint16_t goal_ticks[10];  // SYNC WRITE view, bus-ID order. With shaping
                              //   on (the default, as deployed) these are the
                              //   C2-shaped targets, NOT the raw action map;
                              //   see sil_set_shaper.
    float action[10];         // the raw [-1,1] policy action (diagnostics)
    float obs[147];           // the assembled obs (diagnostics/golden)
    uint16_t goal_speed[10];  // per-servo goal speed (reg 46, steps/s) the
                              //   SYNC WRITE carries, bus-ID order; 0 when
                              //   shaping is off (sil_abi 2 addition)
} SilTargets;

// Load an exported policy and a calibration, then reset.
//   weights_path  the .silw f32 blob; its JSON sidecar is found beside it.
//                 NULL or "" -> the weights compiled into the firmware
//                 (policy/weights.h), so the harness runs before the
//                 exporter exists.
//   cal_path      JSON [{bus_id, zero_steps, dir}, ...]. NULL or "" -> the
//                 nominal Calibration() (zero_steps 2048, dir +1).
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

// A JSON description of the obs spec this library was built against: dims,
// frame offsets, the kServoId permutation, a hash of the layout for drift
// detection, and which policy weights are loaded. Valid before sil_init.
const char* sil_spec(void);

// Additive to the ABI in docs/sil-harness.md: the text of the last failure.
// Never NULL. Kept because "sil_init returned -2" is a bad debugging story
// when the sidecar path is the thing that is wrong.
const char* sil_last_error(void);

// sil_abi 2: the C2 command shaper's pole in Hz, the mirror of ctrl_task's
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

// libctrl_sil -- the software-in-the-loop control library (docs/sil-harness.md).
//
// This file is glue and nothing else. Every decision the robot makes is made
// by code that is compiled from the components verbatim:
//
//   obs::stepsToAngle / stepsPerSecToRadPerSec   the calibration maps
//   obs::assembleFrame / History / GaitClock     the observation
//   policy::forwardNet                           the network (same arithmetic
//                                                as the on-target forward())
//   obs::actionToAngles / angleToSteps           the actuation map
//
// The tick sequencing below is a line-by-line mirror of main/ctrl_task.cpp's
// non-limp path: advance the gait clock, assemble the frame, refill the ring
// on the first tick after a reset, build the obs, run the policy, THEN push
// the frame and latch prev_action. Getting that order wrong shifts the history
// by one tick and is invisible in a unit test of any single piece, which is
// why the whole path is exercised here instead.
//
// Deliberately NOT mirrored (they have no meaning without hardware): the bus
// transaction, the IMU driver, the battery guard, the link watchdog and the
// torque state machine. docs/sil-harness.md scopes those out; the watchdog's
// hold-last-target behaviour is a harness-side test (pyramid item 4).
#include "sil.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <string>

#include "obs/actuation.h"
#include "obs/assembler.h"
#include "obs/obs_spec.h"
#include "policy/mlp.h"
#include "scsbus/registers.h"
#include "sil_net.h"

namespace {

// The ABI's fixed widths are the contract with python's ctypes. If a retrain
// regenerates obs_spec.h with different dimensions, this file must be looked
// at rather than silently reinterpreting the harness's bytes.
static_assert(obs::kNumBusJoints == 17, "SilSensors/goal arrays are 17 wide");
static_assert(obs::kActDim == 10, "SilTargets.action is 10 wide");
static_assert(obs::kNumCmd == 7, "SilSensors.cmd is 7 wide");
static_assert(obs::kObsDim == 147, "SilTargets.obs is 147 wide");
static_assert(obs::kFrameDim * obs::kHistLen == obs::kObsDim,
              "obs is hist_len frames");

// Slot in the servo-ID-ordered arrays for a given policy joint: its bus
// index (obs/bus_map.h). See the ARRAY ORDER note in sil.h: slot = bus_id - 1.
inline int slotOf(int joint) { return obs::policyToBus(joint); }

// The goal of a bus slot the policy does not drive: the position sensed on
// the first tick after a reset (ctrl_task holds such a servo where it was
// measured at takeover).
uint16_t g_hold[obs::kNumBusJoints];

bool g_ready = false;
bool g_primed = false;
obs::Calibration g_cal;
obs::History g_hist;
obs::GaitClock g_clock(1.5f);
sil::LoadedNet g_net;
float g_gait_hz = 1.5f;

// The C2 command shaper + goal-speed streaming, the mirror of ctrl_task's
// act path. Shaping defaults ON (kShaperPoleHz), matching the deployed boot
// state, so closed-loop SIL evaluates the policy against the plant the robot
// actually runs.
obs::CommandShaper g_shaper;
float g_shape_hz = obs::kShaperPoleHz;
static_assert(obs::kGoalSpeedMax == scsbus::kMaxGoalSpeed,
              "obs's speed ceiling must match the servo's");

float g_prev_action[obs::kActDim];
float g_frame[obs::kFrameDim];
float g_obs[obs::kObsDim];
float g_action[obs::kActDim];

std::string g_error;
std::string g_spec;

void setError(const std::string& s) { g_error = s; }

// FNV-1a over a canonical rendering of the generated spec. Any layout drift
// (a reordered frame, a different joint count, a renamed joint, a new command
// channel) changes it, so the python side can pin one number.
uint64_t specHash() {
    char buf[1024];
    int n = snprintf(
        buf, sizeof buf,
        "%s|%s|%s|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d|%d",
        obs::kRunName, obs::kPlantXml, obs::kActionMap, obs::kNumJoints,
        obs::kActDim, obs::kNumCmd, obs::kHistLen, obs::kFrameDim,
        obs::kObsDim, obs::kOffQ, obs::kOffDq, obs::kOffUp, obs::kOffLinVel,
        obs::kOffGyro, obs::kOffPrevAction, obs::kOffHeight, obs::kOffPhase,
        obs::kOffCmd, static_cast<int>(obs::kImuObs));
    if (n < 0) n = 0;
    std::string s(buf, static_cast<size_t>(n));
    for (int j = 0; j < obs::kNumJoints; ++j) {
        snprintf(buf, sizeof buf, "|%s:%d", obs::kJointNames[j],
                 static_cast<int>(obs::kServoId[j]));
        s += buf;
    }
    uint64_t h = 1469598103934665603ull;
    for (char c : s) {
        h ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
        h *= 1099511628211ull;
    }
    return h;
}

void buildSpec() {
    char buf[512];
    g_spec = "{";
    snprintf(buf, sizeof buf,
             "\"sil_abi\":3,\"run\":\"%s\",\"plant\":\"%s\","
             "\"action_map\":\"%s\",\"spec_hash\":\"0x%016llx\",",
             obs::kRunName, obs::kPlantXml, obs::kActionMap,
             static_cast<unsigned long long>(specHash()));
    g_spec += buf;
    // sil_abi 2: what the python side needs to replicate the C2 command
    // shaper (three cascaded exact first-order lags) and the goal-speed
    // streaming bit for bit. pole_hz reflects the CURRENT sil_set_shaper
    // state; 0 means shaping off.
    snprintf(buf, sizeof buf,
             "\"shaper\":{\"pole_hz\":%.9g,\"stages\":3,"
             "\"speed_headroom\":%.9g,\"speed_floor\":%d,\"speed_max\":%d},",
             static_cast<double>(g_shape_hz),
             static_cast<double>(obs::kGoalSpeedHeadroom),
             static_cast<int>(obs::kGoalSpeedFloor),
             static_cast<int>(obs::kGoalSpeedMax));
    g_spec += buf;
    snprintf(buf, sizeof buf,
             "\"num_joints\":%d,\"act_dim\":%d,\"num_cmd\":%d,"
             "\"hist_len\":%d,\"frame_dim\":%d,\"obs_dim\":%d,"
             "\"control_dt\":%.9g,\"imu_obs\":%s,\"gait_clock\":%s,",
             obs::kNumJoints, obs::kActDim, obs::kNumCmd, obs::kHistLen,
             obs::kFrameDim, obs::kObsDim,
             static_cast<double>(obs::kControlDt),
             obs::kImuObs ? "true" : "false",
             obs::kGaitClock ? "true" : "false");
    g_spec += buf;
    snprintf(buf, sizeof buf,
             "\"frame_offsets\":{\"q\":%d,\"dq\":%d,\"up\":%d,\"linvel\":%d,"
             "\"gyro\":%d,\"prev_action\":%d,\"height\":%d,\"phase\":%d,"
             "\"cmd\":%d},",
             obs::kOffQ, obs::kOffDq, obs::kOffUp, obs::kOffLinVel,
             obs::kOffGyro, obs::kOffPrevAction, obs::kOffHeight,
             obs::kOffPhase, obs::kOffCmd);
    g_spec += buf;

    g_spec += "\"servo_id\":[";
    for (int j = 0; j < obs::kNumJoints; ++j) {
        snprintf(buf, sizeof buf, "%s%d", j ? "," : "",
                 static_cast<int>(obs::kServoId[j]));
        g_spec += buf;
    }
    g_spec += "],\"joint_names\":[";
    for (int j = 0; j < obs::kNumJoints; ++j) {
        snprintf(buf, sizeof buf, "%s\"%s\"", j ? "," : "",
                 obs::kJointNames[j]);
        g_spec += buf;
    }
    // The bus map: every servo the arrays carry, servo-ID order.
    g_spec += "],\"num_bus_joints\":";
    g_spec += std::to_string(obs::kNumBusJoints);
    g_spec += ",\"bus_servo_id\":[";
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        snprintf(buf, sizeof buf, "%s%d", b ? "," : "",
                 static_cast<int>(obs::kBusServoId[b]));
        g_spec += buf;
    }
    g_spec += "],\"bus_joint_names\":[";
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        snprintf(buf, sizeof buf, "%s\"%s\"", b ? "," : "",
                 obs::kBusJointNames[b]);
        g_spec += buf;
    }
    // The one thing a harness cannot guess. Spelled out so the python side
    // asserts it instead of assuming an identity (see sil.h).
    g_spec += "],\"sensor_index\":\"bus_id-1\",";

    snprintf(buf, sizeof buf,
             "\"ready\":%s,\"gait_freq_hz\":%.9g,"
             "\"policy\":{\"source\":\"%s\",\"builtin\":%s,"
             "\"activation\":\"%s\",\"obs_dim\":%d,\"act_dim\":%d,"
             "\"placeholder_weights\":%s,\"layer_sizes\":[",
             g_ready ? "true" : "false", static_cast<double>(g_gait_hz),
             g_net.source().c_str(), g_net.builtin() ? "true" : "false",
             g_net.activation().c_str(), g_net.obsDim(), g_net.actDim(),
             (g_net.builtin() && policy::kWeightsArePlaceholder) ? "true"
                                                                 : "false");
    g_spec += buf;
    for (size_t i = 0; i < g_net.layerSizes().size(); ++i) {
        snprintf(buf, sizeof buf, "%s%d", i ? "," : "", g_net.layerSizes()[i]);
        g_spec += buf;
    }
    g_spec += "]}}";
}

}  // namespace

extern "C" {

const char* sil_last_error(void) { return g_error.c_str(); }

const char* sil_spec(void) {
    buildSpec();
    return g_spec.c_str();
}

int sil_init(const char* weights_path, const char* cal_path,
             float gait_freq_hz) {
    g_ready = false;
    g_error.clear();

    std::string err;
    if (!sil::loadCalibration(cal_path, g_cal, err)) {
        setError("calibration: " + err);
        return -1;
    }
    if (!g_net.load(weights_path, err)) {
        setError("weights: " + err);
        return -2;
    }
    if (g_net.obsDim() != obs::kObsDim) {
        char msg[160];
        snprintf(msg, sizeof msg,
                 "weights: obs_dim %d but this build assembles %d "
                 "(obs_spec.h drift -- regenerate one side)",
                 g_net.obsDim(), obs::kObsDim);
        setError(msg);
        return -3;
    }
    if (g_net.actDim() != obs::kActDim) {
        char msg[128];
        snprintf(msg, sizeof msg, "weights: act_dim %d but the plant has %d",
                 g_net.actDim(), obs::kActDim);
        setError(msg);
        return -3;
    }

    g_gait_hz = (gait_freq_hz > 0.0f) ? gait_freq_hz : 1.5f;
    g_clock.setFrequency(g_gait_hz);
    // init == boot: shaping back to the firmware default (see sil.h).
    sil_set_shaper(obs::kShaperPoleHz);
    g_ready = true;
    sil_reset();
    return 0;
}

void sil_set_shaper(float pole_hz) {
    g_shape_hz = pole_hz > 0.0f ? pole_hz : 0.0f;
    if (g_shape_hz > 0.0f) {
        g_shaper.setPole(g_shape_hz, obs::kControlDt);
    } else {
        g_shaper.invalidate();
    }
}

void sil_reset(void) {
    // ctrl_task's "we just took the bus" path: a fresh ring, a zeroed
    // prev_action and an unprimed history, so the next tick's own frame
    // fills every history slot (walker_env does the same on env.reset()).
    g_hist = obs::History();
    memset(g_prev_action, 0, sizeof g_prev_action);
    memset(g_frame, 0, sizeof g_frame);
    memset(g_obs, 0, sizeof g_obs);
    memset(g_action, 0, sizeof g_action);
    g_clock.setPhase(0.0f);
    g_primed = false;
    // ctrl_task reseeds the shaper from measured q on a torque (re)engage;
    // here that is the next tick's sensed positions.
    g_shaper.invalidate();
}

int sil_tick(const SilSensors* in, SilTargets* out) {
    if (!g_ready) {
        setError("sil_tick before a successful sil_init");
        return -1;
    }
    if (!in || !out) {
        setError("sil_tick called with a null argument");
        return -2;
    }

    // -- sense: the bus's view -> the sim's joint order ---------------------
    // Byte-for-byte what readJoints() does, minus the transport: the servo's
    // sign-magnitude words go through scsbus::signMag (NOT two's complement --
    // that is the vendor-UI byte-order bug this harness exists to catch), then
    // through the per-joint calibration.
    float q[obs::kNumJoints];
    float dq[obs::kNumJoints];
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int slot = slotOf(j);
        const int32_t pos = scsbus::signMag(in->pos_ticks[slot], 15);
        const int32_t vel =
            scsbus::signMag(static_cast<uint16_t>(in->vel_ticks[slot]), 15);
        q[j] = obs::stepsToAngle(j, pos, g_cal);
        dq[j] = obs::stepsPerSecToRadPerSec(j, vel, g_cal);
    }

    // -- observe ------------------------------------------------------------
    // Same plain-stand gate as ctrl_task.cpp: a freeze-trained policy must
    // see the frozen clock HERE too, or the SIL column grades it against a
    // stand obs training never produced (found 2026-08-30: v22fix_e_s128
    // scored 39/144 SIL vs 69/144 py with the clock always running).
    // cmd layout (walker_env.set_command): [0]=vx, [1]=vy, [2]=wz,
    // [4]=lift. Training's plain_stand gate (env_mjx): no locomotion
    // command AND no lift -- marches/balances keep their clock. Must stay
    // bit-identical to ctrl_task.cpp's gate.
    const float sil_vy = obs::kNumCmd > 1 ? in->cmd[1] : 0.0f;
    const float sil_lift = obs::kNumCmd > 4 ? in->cmd[4] : 0.0f;
    if (!obs::kClockStandFreeze
        || fabsf(in->cmd[0]) > 0.05f || fabsf(sil_vy) > 0.05f
        || fabsf(in->cmd[2]) > 0.05f || fabsf(sil_lift) >= 0.5f) {
        // speed clock: same scale as ctrl_task.cpp (kSpeedClock)
        g_clock.advanceSpeedClock(obs::kControlDt,
                                  sqrtf(in->cmd[0] * in->cmd[0] + sil_vy * sil_vy));
    }

    obs::Inputs inp{};
    memcpy(inp.q, q, sizeof q);
    memcpy(inp.dq, dq, sizeof dq);
    memcpy(inp.up, in->up, sizeof inp.up);
    memcpy(inp.gyro, in->gyro, sizeof inp.gyro);
    memcpy(inp.prev_action, g_prev_action, sizeof g_prev_action);
    inp.phase = g_clock.phase();
    memcpy(inp.cmd, in->cmd, sizeof(float) * obs::kNumCmd);

    obs::assembleFrame(inp, g_frame);
    if (!g_primed) {
        g_hist.fill(g_frame);
        g_primed = true;
        // Hold every slot the policy does not drive: at the deployed run's
        // trained target when it holds that servo (obs_spec.h kHeld*),
        // otherwise where it is now. (No takeover ramp: sil_reset is an
        // episode start, not a bench->run handover.)
        for (int b = 0; b < obs::kNumBusJoints; ++b) {
            const int h = obs::heldIndexOfBus(b);
            g_hold[b] = static_cast<uint16_t>(
                h >= 0 ? obs::busAngleToStepsRaw(b, obs::kHeldTarget[h], g_cal)
                       : scsbus::signMag(in->pos_ticks[b], 15));
        }
    }
    g_hist.build(g_frame, g_obs);

    // -- act ----------------------------------------------------------------
    policy::forwardNet(g_net.net(), g_obs, g_action);
    g_hist.push(g_frame);            // after the policy has been fed
    memcpy(g_prev_action, g_action, sizeof g_prev_action);

    float angle[obs::kNumJoints];
    obs::actionToAngles(g_action, angle);

    // -- shape: the mirror of ctrl_task's act path --------------------------
    if (g_shape_hz > 0.0f) {
        if (!g_shaper.primed()) g_shaper.reset(q);   // sensed == measured
        g_shaper.update(angle, angle);
    }

    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (obs::busToPolicy(b) >= 0) continue;
        out->goal_ticks[b] = g_hold[b];
        out->goal_speed[b] =
            g_shape_hz > 0.0f
                ? obs::goalSpeedSteps(g_hold[b],
                                      scsbus::signMag(in->pos_ticks[b], 15))
                : 0;
    }
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int32_t steps = obs::angleToSteps(j, angle[j], g_cal);
        // angleToSteps already clamps to the encoder's 0..4095; the cast is
        // the SYNC WRITE's 16-bit field.
        out->goal_ticks[slotOf(j)] = static_cast<uint16_t>(steps);
        // The goal speed rides on the same measured position ctrl_task uses
        // (g_fb[i].position there, the sensed ticks here).
        out->goal_speed[slotOf(j)] =
            g_shape_hz > 0.0f
                ? obs::goalSpeedSteps(
                      steps, scsbus::signMag(in->pos_ticks[slotOf(j)], 15))
                : 0;
    }
    memcpy(out->action, g_action, sizeof g_action);
    memcpy(out->obs, g_obs, sizeof g_obs);
    return 0;
}

}  // extern "C"

// Expected position-loop gains per servo ID -- the reference the arm gate
// reads every servo against (scsbus::checkPositionGains, cli.cpp) -- and the
// stiffness each row's gains give in the training sim's units.
//
// Registers 21 (P) and 22 (D) live in each servo's EEPROM, so a factory reset
// or a swapped spare silently comes back at P = 32. The firmware therefore
// reads 21/22 at boot and again before every arm, and refuses to arm on any
// difference from this table (ArmResult::kRefusedGains).
//
// Compiled in, not stored in NVS, on purpose: the table is a design decision
// with evidence behind it, reviewed in git beside docs/servo-map.md section 4
// "Per-ID gains". Change both in the same commit, and name the measurement.
// `gains <id> <p> <d>` writes a servo; it does not change what the robot
// expects.
//
// THE TABLE (2026-10-30): every servo at the factory P 32 / D 32. That is what
// the robot runs, it is clean on the bench (STS3215 and STS3250 alike), and it
// is the only setting whose stiffness on THIS body is measured:
//   - STS3250 (hip rolls, 51/52): limit-cycles at 18-21 Hz from P 96 up; P 64
//     / D 32 is clean but its stiffness is unmeasured (plan-b-bench
//     2026-10-23) -- raise it here only with that measurement and a new
//     kSimKpScale row.
//   - STS3215: P 96 / D 0 is the bench's best raised gain (plan-b-bench
//     2026-10-02), never run on the robot.
//
// kSimKpScale: the sim's servo_kp_scale that a row's gains realise -- the
// physical stiffness over the model's kp 12 N*m/rad. It is NOT P / 32: the
// model is soft for a real P 32. STS3215 at P 32 = 2.8 (bench bottle midpoints
// 34 +- 3 N*m/rad, plan-b-bench 2026-10-01; and on the robot the stance
// ankle-roll encoder reads 1.0-1.5 deg in the one-leg lift, the model 1.3 at
// 2.8x / 4.5 at 1x, sim/roll_compliance.py vs hw 2026-10-24). STS3250 at P 32
// = 4.0 (its stance hip-roll encoder reads 0.7-0.9 deg there: 0.74 at 4x,
// 1.45 at 2x). The sim's "asbuilt" preset (sim/mjx/env_mjx.py
// SERVO_KP_PRESETS) is these factors; tests/test_servo_gains_asbuilt.py pins
// the two equal.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "obs/bus_map.h"
#include "obs/obs_spec.h"
#include "scsbus/gains.h"

namespace robot {

inline constexpr scsbus::GainExpect kExpectedGains[] = {
    {52, scsbus::kFactoryGains},   // R_hip_roll   STS3250
    {6, scsbus::kFactoryGains},    // R_hip_pitch
    {7, scsbus::kFactoryGains},    // R_knee
    {8, scsbus::kFactoryGains},    // R_ankle
    {51, scsbus::kFactoryGains},   // L_hip_roll   STS3250
    {2, scsbus::kFactoryGains},    // L_hip_pitch
    {3, scsbus::kFactoryGains},    // L_knee
    {4, scsbus::kFactoryGains},    // L_ankle
    {10, scsbus::kFactoryGains},   // R_hip_yaw
    {9, scsbus::kFactoryGains},    // L_hip_yaw
    {11, scsbus::kFactoryGains},   // R_ankle_roll
    {12, scsbus::kFactoryGains},   // L_ankle_roll
    {13, scsbus::kFactoryGains},   // neck_yaw
    {14, scsbus::kFactoryGains},   // R_shoulder
    {15, scsbus::kFactoryGains},   // R_elbow
    {16, scsbus::kFactoryGains},   // L_shoulder
    {17, scsbus::kFactoryGains},   // L_elbow
};
inline constexpr size_t kNumExpectedGains =
    sizeof kExpectedGains / sizeof kExpectedGains[0];

// The stiffness each kExpectedGains row realises, in the sim's units
// (servo_kp_scale), row for row (same ids, same order -- static_asserted).
inline constexpr float kStock3215 = 2.8f;   // STS3215 at P 32 / D 32
inline constexpr float kStock3250 = 4.0f;   // STS3250 at P 32 / D 32
struct SimKpScale {
    uint8_t id;
    float factor;
};
inline constexpr SimKpScale kSimKpScale[] = {
    {52, kStock3250},   // R_hip_roll
    {6, kStock3215},    // R_hip_pitch
    {7, kStock3215},    // R_knee
    {8, kStock3215},    // R_ankle
    {51, kStock3250},   // L_hip_roll
    {2, kStock3215},    // L_hip_pitch
    {3, kStock3215},    // L_knee
    {4, kStock3215},    // L_ankle
    {10, kStock3215},   // R_hip_yaw
    {9, kStock3215},    // L_hip_yaw
    {11, kStock3215},   // R_ankle_roll
    {12, kStock3215},   // L_ankle_roll
    {13, kStock3215},   // neck_yaw
    {14, kStock3215},   // R_shoulder
    {15, kStock3215},   // R_elbow
    {16, kStock3215},   // L_shoulder
    {17, kStock3215},   // L_elbow
};

// The row for `id`, or nullptr when the table has none.
constexpr const scsbus::GainExpect* expectedGainsFor(uint8_t id) {
    for (size_t i = 0; i < kNumExpectedGains; ++i) {
        if (kExpectedGains[i].id == id) return &kExpectedGains[i];
    }
    return nullptr;
}

namespace detail {
// The index of the row for `id`, or kNumExpectedGains when the table has none.
//
// The compile-time checks below use this rather than `expectedGainsFor`: taking
// the address of an array element and comparing it with nullptr is not a
// constant expression under `-fsanitize=undefined` (GCC rejects it outright, and
// with -Werror the address-of check that would otherwise merely warn is fatal).
// The pointer API stays for the runtime callers -- the CLI's gain check and the
// host tests -- where it reads better and is a constant expression.
constexpr size_t expectedGainsIndexOf(uint8_t id) {
    for (size_t i = 0; i < kNumExpectedGains; ++i) {
        if (kExpectedGains[i].id == id) return i;
    }
    return kNumExpectedGains;
}
constexpr bool everyBusServoHasExpectedGains() {
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (expectedGainsIndexOf(obs::kBusServoId[b]) == kNumExpectedGains) return false;
    }
    return true;
}
constexpr bool expectedGainIdsAreUnique() {
    for (size_t i = 0; i < kNumExpectedGains; ++i) {
        if (kExpectedGains[i].gains.p == 0) return false;   // not a gain
        for (size_t k = i + 1; k < kNumExpectedGains; ++k) {
            if (kExpectedGains[i].id == kExpectedGains[k].id) return false;
        }
    }
    return true;
}
constexpr bool simKpScaleIsRowForRow() {
    if (sizeof kSimKpScale / sizeof kSimKpScale[0] != kNumExpectedGains) return false;
    for (size_t i = 0; i < kNumExpectedGains; ++i) {
        if (kSimKpScale[i].id != kExpectedGains[i].id) return false;
        if (!(kSimKpScale[i].factor > 0.0f)) return false;
    }
    return true;
}
constexpr bool isFactory(const scsbus::PositionGains& g) {
    return g.p == scsbus::kFactoryGains.p && g.d == scsbus::kFactoryGains.d;
}
// The deployed policy trained with a per-servo stiffness (obs_spec.h
// kServoKpScale, from its config.json servo_kp_scale). It must have trained on
// the stiffness the robot will actually have, servo for servo. Two contracts
// are accepted, each all-or-nothing over the plant's servos:
//   AS BUILT  every plant servo trained at exactly kSimKpScale of its row (the
//             "asbuilt" preset): the policy saw the gains this table enforces;
//   LEGACY    every plant servo trained at 1.0 on a factory row: a policy from
//             before 2026-10-30 on the model's uncalibrated stock servo (the
//             deployed 10-joint prototype policy). Softer than the robot, but
//             the contract it was deployed under; nothing raised.
// Anything else -- e.g. the "robot" preset (hip roll 4, knee and ankle roll
// 2.8, the rest 1), which credits a raised P the robot does not run on half
// the servos and models the other half 2.8x soft -- does not build.
constexpr bool gainsMatchTheTrainedStiffness() {
    bool as_built = true;
    bool legacy = true;
    for (int i = 0; i < obs::kNumPlantServos; ++i) {
        const size_t idx = expectedGainsIndexOf(obs::kPlantServoId[i]);
        if (idx == kNumExpectedGains) return false;
        const float trained = obs::kServoKpScale[i];
        if (trained != kSimKpScale[idx].factor) as_built = false;
        if (trained != 1.0f || !isFactory(kExpectedGains[idx].gains)) legacy = false;
    }
    return as_built || legacy;
}
}  // namespace detail
static_assert(detail::simKpScaleIsRowForRow(),
              "kSimKpScale must list kExpectedGains' ids in the same order, "
              "each with a positive factor");
static_assert(detail::gainsMatchTheTrainedStiffness(),
              "the deployed policy (obs_spec.h kServoKpScale) did not train on "
              "the stiffness main/servo_gains.h gives the robot: retrain with "
              "--servo-kp-scale asbuilt (or the table's factors), or change the "
              "table with a measurement");
static_assert(detail::everyBusServoHasExpectedGains(),
              "a servo on the bus (obs/bus_map.h) has no expected-gain row");
static_assert(detail::expectedGainIdsAreUnique(),
              "kExpectedGains: duplicate id or P == 0");

}  // namespace robot

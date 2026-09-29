// Expected position-loop gains per servo ID -- the reference the arm gate
// reads every servo against (scsbus::checkPositionGains, cli.cpp).
//
// Plan B (DESIGN.md section 4) raises register 21 (P, with D in 22) on the
// roll and knee servos. Those values live in each servo's EEPROM, so a
// factory reset or a swapped spare silently comes back at P = 32 and the
// robot falls at its first crossover. The firmware therefore reads 21/22 at
// boot and again before every arm, and refuses to arm on any difference from
// this table (ArmResult::kRefusedGains).
//
// Compiled in, not stored in NVS, on purpose: the table is a design decision
// with evidence behind it (the Plan B bench test, issue #73), reviewed in git
// beside docs/servo-map.md section 4 "Per-ID gains". Change both in the same
// commit, and name the measurement. `gains <id> <p> <d>` writes a servo; it
// does not change what the robot expects.
//
// Every row is the factory 32/32 until #73 measures the raised P: the
// prototype runs stock servos, and a value is modelled only once measured.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "obs/bus_map.h"
#include "obs/obs_spec.h"
#include "scsbus/gains.h"

namespace robot {

inline constexpr scsbus::GainExpect kExpectedGains[] = {
    {1, scsbus::kFactoryGains},    // R_hip_roll   (Plan B: raised, #73)
    {2, scsbus::kFactoryGains},    // R_hip_pitch
    {3, scsbus::kFactoryGains},    // R_knee       (Plan B: raised, #73)
    {4, scsbus::kFactoryGains},    // R_ankle
    {5, scsbus::kFactoryGains},    // L_hip_roll   (Plan B: raised, #73)
    {6, scsbus::kFactoryGains},    // L_hip_pitch
    {7, scsbus::kFactoryGains},    // L_knee       (Plan B: raised, #73)
    {8, scsbus::kFactoryGains},    // L_ankle
    {9, scsbus::kFactoryGains},    // R_hip_yaw
    {10, scsbus::kFactoryGains},   // L_hip_yaw
    {11, scsbus::kFactoryGains},   // R_ankle_roll (Plan B: raised, #73)
    {12, scsbus::kFactoryGains},   // L_ankle_roll (Plan B: raised, #73)
    {13, scsbus::kFactoryGains},   // neck_yaw
    {14, scsbus::kFactoryGains},   // R_shoulder
    {15, scsbus::kFactoryGains},   // R_elbow
    {16, scsbus::kFactoryGains},   // L_shoulder
    {17, scsbus::kFactoryGains},   // L_elbow
};
inline constexpr size_t kNumExpectedGains =
    sizeof kExpectedGains / sizeof kExpectedGains[0];

// The row for `id`, or nullptr when the table has none.
constexpr const scsbus::GainExpect* expectedGainsFor(uint8_t id) {
    for (size_t i = 0; i < kNumExpectedGains; ++i) {
        if (kExpectedGains[i].id == id) return &kExpectedGains[i];
    }
    return nullptr;
}

namespace detail {
constexpr bool everyBusServoHasExpectedGains() {
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (expectedGainsFor(obs::kBusServoId[b]) == nullptr) return false;
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
// The deployed policy trained with a per-servo stiffness (obs_spec.h
// kServoKpScale; Plan B = x4 on hip roll, ankle roll and knee). The table
// must raise P exactly on the servos the policy was trained stiffer on: a
// Plan B policy flashed onto a factory-P table (or the reverse) does not
// build. The register VALUE that realises a factor is the #73 bench
// measurement, so this checks the pattern, not the number.
constexpr bool gainsMatchTheTrainedStiffness() {
    for (int i = 0; i < obs::kNumPlantServos; ++i) {
        const scsbus::GainExpect* e = expectedGainsFor(obs::kPlantServoId[i]);
        if (e == nullptr) return false;
        const bool raised = e->gains.p != scsbus::kFactoryGains.p;
        const bool trained_stiffer = obs::kServoKpScale[i] != 1.0f;
        if (raised != trained_stiffer) return false;
    }
    return true;
}
}  // namespace detail
static_assert(detail::gainsMatchTheTrainedStiffness(),
              "main/servo_gains.h does not raise P exactly where the deployed "
              "policy trained with a stiffer servo (obs_spec.h kServoKpScale)");
static_assert(detail::everyBusServoHasExpectedGains(),
              "a servo on the bus (obs/bus_map.h) has no expected-gain row");
static_assert(detail::expectedGainIdsAreUnique(),
              "kExpectedGains: duplicate id or P == 0");

}  // namespace robot

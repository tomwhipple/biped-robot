// The BUS joint set: every servo on the robot's bus, by servo ID.
//
// Two joint sets meet in this firmware and must not be confused:
//
//   bus joints     every servo the robot carries (this file, hand-written):
//                  calibration, the mechanical envelope, `pose`/`home`, the
//                  readback, the telemetry joint block and the SIL arrays are
//                  all bus-joint wide, in bus-index order. Bus index b is
//                  joint kBusJointNames[b], whose servo is kBusServoId[b] --
//                  NOT servo ID b + 1 (look an ID up with busIndexOfId).
//   policy joints  the joints the compiled policy observes and commands
//                  (obs_spec.h, generated from the deployed run): kNumJoints
//                  of them, in the sim's actuator order, with kServoId naming
//                  each one's servo. Always a subset of the bus joints.
//
// The map below is docs/servo-map.md section 2.1: the v6 legs AS ASSEMBLED
// (2026-10-23, read off the robot on the stand and written on each case by the
// owner -- servos are not renumbered to fit a table). The legs' yaw / pitch /
// knee / ankle IDs sit on the opposite side from the old prototype map, and the
// STS3250 hip rolls are 52 (right) and 51 (left); the ankle rolls 11 / 12 and
// the neck and arms 13-17 are unchanged. Change it, tools/gen_obs_spec.py
// ID_BY_ROLE, tools/bus_cal.py BUS, servo_gains.h and servo-map.md in one
// commit -- the static_asserts below and the SIL suite check that they agree
// with the generated policy spec.
#pragma once
#include <stdint.h>

#include "obs/obs_spec.h"

namespace obs {

inline constexpr int kNumBusJoints = 17;

// Bus index order (kept from the prototype, so every bus-indexed table and
// wire layout is unchanged). Names are the plant's joint names
// (sim/bimo_biped_v6ar.xml, arms from sim/gen_plant_v6.py); the comment is the
// servo ID, as kBusServoId.
inline constexpr const char* kBusJointNames[kNumBusJoints] = {
    "R_hip_roll",    // 52  STS3250
    "R_hip_pitch",   // 6
    "R_knee",        // 7
    "R_ankle",       // 8   pitch
    "L_hip_roll",    // 51  STS3250
    "L_hip_pitch",   // 2
    "L_knee",        // 3
    "L_ankle",       // 4   pitch
    "R_hip_yaw",     // 10
    "L_hip_yaw",     // 9
    "R_ankle_roll",  // 11
    "L_ankle_roll",  // 12
    "neck_yaw",      // 13
    "R_shoulder",    // 14
    "R_elbow",       // 15
    "L_shoulder",    // 16
    "L_elbow",       // 17
};
inline constexpr uint8_t kBusServoId[kNumBusJoints] = {
    52, 6, 7, 8, 51, 2, 3, 4, 10, 9, 11, 12, 13, 14, 15, 16, 17};
// The board port each servo's chain hangs off: 'A' = H5 (the robot's right
// side), 'B' = H6 (its left). The neck rides port A's upper branch. Both ports
// carry the same bus, so which port each leg is actually plugged into is not
// visible from software (unverified on the 2026-10-23 assembly).
inline constexpr char kBusPort[kNumBusJoints] = {
    'A', 'A', 'A', 'A', 'B', 'B', 'B', 'B', 'A', 'B',
    'A', 'B', 'A', 'A', 'A', 'B', 'B'};

// Bus index of servo `id`, or -1 when no bus joint has it.
constexpr int busIndexOfId(int id) {
    for (int b = 0; b < kNumBusJoints; ++b) {
        if (kBusServoId[b] == id) return b;
    }
    return -1;
}

namespace detail {
constexpr bool sameName(const char* a, const char* b) {
    while (*a && *a == *b) {
        ++a;
        ++b;
    }
    return *a == *b;
}
}  // namespace detail

// Bus index of the joint called `name`, or -1.
constexpr int busIndexOfName(const char* name) {
    for (int b = 0; b < kNumBusJoints; ++b) {
        if (detail::sameName(kBusJointNames[b], name)) return b;
    }
    return -1;
}

namespace detail {
struct PolicyBusMap {
    int bus[kNumJoints];
};
constexpr PolicyBusMap makePolicyBusMap() {
    PolicyBusMap m{};
    for (int j = 0; j < kNumJoints; ++j) m.bus[j] = busIndexOfId(kServoId[j]);
    return m;
}
constexpr bool busIdsAreUniqueAndValid() {
    for (int a = 0; a < kNumBusJoints; ++a) {
        if (kBusServoId[a] < 1 || kBusServoId[a] > 253) return false;
        for (int b = a + 1; b < kNumBusJoints; ++b) {
            if (kBusServoId[a] == kBusServoId[b]) return false;
        }
    }
    return true;
}
constexpr bool policyIsASubsetOfTheBus() {
    for (int j = 0; j < kNumJoints; ++j) {
        const int b = busIndexOfId(kServoId[j]);
        if (b < 0) return false;
        // The same servo must be the same joint on both sides.
        if (!sameName(kJointNames[j], kBusJointNames[b])) return false;
        for (int k = j + 1; k < kNumJoints; ++k) {
            if (kServoId[k] == kServoId[j]) return false;
        }
    }
    return true;
}
constexpr bool busNamesAreUnique() {
    for (int a = 0; a < kNumBusJoints; ++a) {
        for (int b = a + 1; b < kNumBusJoints; ++b) {
            if (sameName(kBusJointNames[a], kBusJointNames[b])) return false;
        }
    }
    return true;
}
}  // namespace detail

inline constexpr detail::PolicyBusMap kPolicyBusMap =
    detail::makePolicyBusMap();

// Bus index of policy joint j.
constexpr int policyToBus(int j) { return kPolicyBusMap.bus[j]; }

// Policy index of bus joint b, or -1 when the policy does not drive it.
constexpr int busToPolicy(int b) {
    for (int j = 0; j < kNumJoints; ++j) {
        if (kPolicyBusMap.bus[j] == b) return j;
    }
    return -1;
}

// The deployed run's HELD servos (obs_spec.h kHeld*: the robot's neck and
// arms, at a fixed trained target): index into kHeld* of bus joint b, or -1.
constexpr int heldIndexOfBus(int b) {
    // cppcheck-suppress knownConditionTrueFalse -- kNumHeld is 0 for a
    // prototype run and not for a robot run: generated, not a constant.
    for (int h = 0; h < kNumHeld; ++h) {
        if (kHeldServoId[h] == kBusServoId[b]) return h;
    }
    return -1;
}

namespace detail {
constexpr bool heldServosAreUndrivenBusJoints() {
    for (int h = 0; h < kNumHeld; ++h) {
        const int b = busIndexOfId(kHeldServoId[h]);
        if (b < 0) return false;
        if (!sameName(kHeldNames[h], kBusJointNames[b])) return false;
        for (int j = 0; j < kNumJoints; ++j) {
            if (kServoId[j] == kHeldServoId[h]) return false;
        }
    }
    return true;
}
constexpr bool plantServosAreBusJoints() {
    for (int i = 0; i < kNumPlantServos; ++i) {
        if (busIndexOfId(kPlantServoId[i]) < 0) return false;
    }
    return true;
}
}  // namespace detail

static_assert(detail::heldServosAreUndrivenBusJoints(),
              "a held servo of the deployed spec is not a bus joint the "
              "policy leaves alone");
static_assert(detail::plantServosAreBusJoints(),
              "a servo of the deployed run's plant is not on the bus");

static_assert(kNumBusJoints >= kNumJoints,
              "the policy drives more joints than the bus carries");
static_assert(detail::busIdsAreUniqueAndValid(),
              "bus servo IDs must be unique and 1..253");
static_assert(detail::policyIsASubsetOfTheBus(),
              "every policy joint's servo must be on the bus, under the same "
              "joint name, once");
static_assert(detail::busNamesAreUnique(), "duplicate bus joint name");

}  // namespace obs

// Obs assembler, history ring, gait clock, action/angle/tick maps, IMU maths.
//
// The frames in obs_vectors.h came out of walker_env._obs() itself (see
// tools/gen_obs_spec.py). If anybody reorders the observation in the sim and
// regenerates, this test is what goes red.
#include "../main/asbuilt_cal.h"
#include "../main/cal_store.h"
#include "imu/imu.h"
#include "obs/actuation.h"
#include "obs/assembler.h"
#include <string.h>

#include "test_util.h"
#include "vectors/obs_vectors.h"

namespace {

void testSpecMatchesVectors() {
    CHECK_EQ(obs::kNumJoints, obs_vectors::kNumJoints);
    CHECK_EQ(obs::kNumCmd, obs_vectors::kNumCmd);
    CHECK_EQ(obs::kFrameDim, obs_vectors::kFrameDim);
    CHECK_EQ(obs::kObsDim, obs::kFrameDim * obs::kHistLen);
    // The deployed plant is the 10-DOF v3yaw one (MEMORY: all sims on
    // bimo_biped_v3yaw.xml), so the joint count is 10, not 8.
    CHECK_EQ(obs::kNumJoints, 10);
    CHECK(obs::kImuObs);
}

void testServoIdMapIsAPermutation() {
    // docs/wiring.md's bus IDs vs. the sim's action order. On the 10-DOF plant
    // this is NOT the identity, and the 2026-08-02 errata widened the gap: the
    // two bus chains went onto the opposite legs, so port A (9,1,2,3,4) is the
    // RIGHT leg and port B (10,5,6,7,8) the LEFT. See ID_BY_ROLE in
    // tools/gen_obs_spec.py for the observations that pinned it down.
    bool seen[16] = {};
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const uint8_t id = obs::kServoId[i];
        CHECK(id >= 1 && id <= 10);
        CHECK(!seen[id]);
        seen[id] = true;
    }
    CHECK_EQ(obs::kServoId[0], 10);     // L_hip_yaw  -- port B, the LEFT leg
    CHECK_EQ(obs::kServoId[1], 5);      // L_hip_roll -- port B
    CHECK_EQ(obs::kServoId[5], 9);      // R_hip_yaw  -- port A, the RIGHT leg
    CHECK_EQ(obs::kServoId[9], 4);      // R_ankle    -- port A
}

void testFramesMatchTheSim() {
    for (size_t c = 0; c < obs_vectors::kNumCases; ++c) {
        const auto& k = obs_vectors::kCases[c];
        obs::Inputs in{};
        for (int i = 0; i < obs::kNumJoints; ++i) {
            in.q[i] = k.q[i];
            in.dq[i] = k.dq[i];
            in.prev_action[i] = k.prev_action[i];
        }
        for (int i = 0; i < 3; ++i) {
            in.up[i] = k.up[i];
            in.gyro[i] = k.gyro[i];
        }
        for (int i = 0; i < obs::kNumCmd; ++i) in.cmd[i] = k.cmd[i];
        in.phase = k.phase;

        float frame[obs::kFrameDim];
        obs::assembleFrame(in, frame);
        for (int i = 0; i < obs::kFrameDim; ++i) {
            CHECK_NEAR(frame[i], k.frame[i], 1e-6);
        }
        // The two channels the robot cannot sense are zero, on purpose.
        CHECK_EQ(frame[obs::kOffLinVel + 0] == 0.0f, true);
        CHECK_EQ(frame[obs::kOffHeight] == 0.0f, true);
    }
}

void testHistoryOrdering() {
    // walker_env: obs at tick t is [f_t, f_{t-1}, f_{t-2}], and the ring is
    // pushed AFTER the policy has been fed.
    float f[3][obs::kFrameDim];
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < obs::kFrameDim; ++i) {
            f[k][i] = static_cast<float>(k * 1000 + i);
        }
    }
    obs::History h;
    h.fill(f[0]);                       // reset semantics: all slots = f0
    float o[obs::kObsDim];
    h.build(f[0], o);
    for (int k = 0; k < obs::kHistLen; ++k) {
        CHECK_NEAR(o[k * obs::kFrameDim + 5], f[0][5], 0.0);
    }
    h.push(f[0]);

    h.build(f[1], o);
    CHECK_NEAR(o[5], f[1][5], 0.0);
    CHECK_NEAR(o[obs::kFrameDim + 5], f[0][5], 0.0);
    CHECK_NEAR(o[2 * obs::kFrameDim + 5], f[0][5], 0.0);
    h.push(f[1]);

    h.build(f[2], o);
    CHECK_NEAR(o[5], f[2][5], 0.0);
    CHECK_NEAR(o[obs::kFrameDim + 5], f[1][5], 0.0);
    CHECK_NEAR(o[2 * obs::kFrameDim + 5], f[0][5], 0.0);
}

void testGaitClockWraps() {
    // (phase + 2*pi*dt*f + pi) % (2*pi) - pi, i.e. always in (-pi, pi].
    obs::GaitClock clk(1.5f);
    clk.setPhase(3.0f);
    for (int i = 0; i < 500; ++i) {
        clk.advance(0.02f);
        CHECK(clk.phase() >= -3.14159266f && clk.phase() <= 3.14159266f);
    }
    // One full cycle at 1.5 Hz is 1/1.5 s; after that the phase returns.
    obs::GaitClock c2(1.5f);
    c2.setPhase(0.0f);
    for (int i = 0; i < 100; ++i) c2.advance(1.0f / 150.0f);   // exactly 1 rev
    CHECK_NEAR(c2.phase(), 0.0, 1e-3);
}

void testActionToAngles() {
    // action_map "full": action 0 is the standing pose, +-1 the joint limits.
    float action[obs::kNumJoints], angle[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 0.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointDefault[i], 1e-6);
    }
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 1.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointHi[i], 1e-6);
    }
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = -1.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointLo[i], 1e-6);
    }
    // Out-of-range actions are clamped, not extrapolated past the mechanism.
    for (int i = 0; i < obs::kNumJoints; ++i) action[i] = 5.0f;
    obs::actionToAngles(action, angle);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(angle[i], obs::kJointHi[i], 1e-6);
    }
    // Round trip through the inverse.
    const float probe[] = {-1.0f, -0.5f, 0.0f, 0.25f, 0.9f};
    for (float a : probe) {
        for (int i = 0; i < obs::kNumJoints; ++i) action[i] = a;
        obs::actionToAngles(action, angle);
        float back[obs::kNumJoints];
        obs::anglesToAction(angle, back);
        for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(back[i], a, 1e-5);
    }
}

void testAngleToSteps() {
    obs::Calibration cal;
    CHECK_EQ(obs::angleToSteps(0, 0.0f, cal), 2048);
    // 45 deg is an eighth of 4096 ticks. Joint 0 (hip yaw) travels exactly
    // +-45 deg, so a commanded quarter turn clamps to the same tick.
    CHECK_EQ(obs::angleToSteps(0, obs::kJointHi[0], cal), 2048 + 512);
    const float quarter = 1.5707963f;
    CHECK_EQ(obs::angleToSteps(0, quarter, cal), 2048 + 512);
    // Direction and zero offset are honoured -- on the policy joint's BUS
    // row (the calibration is bus-indexed, obs/bus_map.h).
    cal.dir[obs::policyToBus(3)] = -1;
    cal.zero_steps[obs::policyToBus(3)] = 1900;
    const float a = -0.5f;
    const int32_t s = obs::angleToSteps(3, a, cal);
    CHECK_NEAR(obs::stepsToAngle(3, s, cal), a, 2e-3);
    CHECK(s > 1900);                            // negative angle, inverted dir
}

void testBusAngleToStepsRaw() {
    obs::Calibration cal;
    const int b0 = obs::policyToBus(0), b3 = obs::policyToBus(3);
    // Inside the policy range the bus map and the policy map agree exactly.
    CHECK_EQ(obs::busAngleToStepsRaw(b0, 0.0f, cal),
             obs::angleToSteps(0, 0.0f, cal));
    CHECK_EQ(obs::busAngleToStepsRaw(b0, obs::kJointHi[0], cal),
             obs::angleToSteps(0, obs::kJointHi[0], cal));
    // Past the policy range: angleToSteps clamps (SIL-pinned act-path
    // behavior), the bus map converts through -- the bench envelope clamp
    // (main/mech_envelope.h) depends on this.
    // DERIVED from kJointHi, not hard-coded: this was 0.5f, picked when the
    // knee policy range stopped at +5 deg. Opening it to the measured +-95 on
    // 2026-08-06 put 0.5f rad INSIDE the range, so nothing clamped and the
    // check silently inverted. Anything past the limit exercises the clamp.
    const float hyper = obs::kJointHi[3] + 0.4f;
    CHECK_EQ(obs::angleToSteps(3, hyper, cal),
             obs::busAngleToStepsRaw(b3, obs::kJointHi[3], cal));
    const int32_t r = obs::busAngleToStepsRaw(b3, hyper, cal);
    CHECK(r != obs::angleToSteps(3, hyper, cal));
    CHECK_NEAR(obs::stepsToAngle(3, r, cal), hyper, 2e-3);
    CHECK_NEAR(obs::busStepsToAngle(b3, r, cal), hyper, 2e-3);
    // Encoder-domain guard still applies.
    CHECK_EQ(obs::busAngleToStepsRaw(b0, 100.0f, cal), 4095);
    CHECK_EQ(obs::busAngleToStepsRaw(b0, -100.0f, cal), 0);
    // A bus joint no policy drives converts through its own row.
    constexpr int neck = obs::busIndexOfId(13);
    static_assert(neck >= 0, "servo 13 (neck) is on the bus");
    CHECK(obs::busToPolicy(neck) < 0);
    cal.zero_steps[neck] = 1000;
    cal.dir[neck] = -1;
    CHECK_EQ(obs::busAngleToStepsRaw(neck, 0.0f, cal), 1000);
    CHECK(obs::busAngleToStepsRaw(neck, 0.5f, cal) < 1000);
}

// The bus joint set (obs/bus_map.h) against the generated policy spec: the
// map the telemetry block, the SIL arrays and the calibration all index by.
void testBusMap() {
    CHECK_EQ(obs::kNumBusJoints, 17);
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        CHECK_EQ(obs::kBusServoId[b], b + 1);          // index b == ID b + 1
        CHECK_EQ(obs::busIndexOfId(b + 1), b);
        CHECK(obs::kBusPort[b] == 'A' || obs::kBusPort[b] == 'B');
    }
    CHECK_EQ(obs::busIndexOfId(0), -1);
    CHECK_EQ(obs::busIndexOfId(18), -1);
    // Every policy joint is a bus joint with the same name and servo, and
    // the two maps are inverse on the policy's joints.
    int driven = 0;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int b = obs::policyToBus(j);
        if (b < 0 || b >= obs::kNumBusJoints) {
            CHECK(false);
            continue;
        }
        CHECK_EQ(obs::kBusServoId[b], obs::kServoId[j]);
        CHECK(strcmp(obs::kBusJointNames[b], obs::kJointNames[j]) == 0);
        CHECK_EQ(obs::busToPolicy(b), j);
    }
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        if (obs::busToPolicy(b) >= 0) ++driven;
    }
    CHECK_EQ(driven, obs::kNumJoints);
    // The proposed map (docs/servo-map.md 2.1): IDs 1-10 the prototype's,
    // then the ankle rolls, the neck and the arms.
    CHECK(strcmp(obs::kBusJointNames[obs::busIndexOfId(9)], "R_hip_yaw") == 0);
    CHECK(strcmp(obs::kBusJointNames[obs::busIndexOfId(11)], "R_ankle_roll") == 0);
    CHECK(strcmp(obs::kBusJointNames[obs::busIndexOfId(13)], "neck_yaw") == 0);
    CHECK(strcmp(obs::kBusJointNames[obs::busIndexOfId(17)], "L_elbow") == 0);
}

void testVelocityEstimator() {
    obs::VelocityEstimator est(1.0f);           // raw difference
    float q[obs::kNumJoints] = {}, v[obs::kNumJoints];
    est.reset(q);
    for (int i = 0; i < obs::kNumJoints; ++i) q[i] = 0.02f;
    est.update(q, 0.02f, v);
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(v[i], 1.0, 1e-5);

    obs::Calibration cal;
    // 4096 steps/s is exactly one revolution per second.
    CHECK_NEAR(obs::stepsPerSecToRadPerSec(0, 4096, cal), 6.2831853, 1e-4);
}

// The IMU calibration blob is a SEPARATE NVS record from the servo one, and
// it has to round-trip exactly: an uncalibrated gyro bias parks the attitude
// estimate ~64 degrees off (cal_store.h), so a blob that silently fails to
// load is not a small problem.
void testImuCalBlob() {
    const float bias[3] = {-0.45349f, 0.06721f, -0.00259f};
    const float mount[4] = {0.70710678f, 0.0f, 0.70710678f, 0.0f};

    robot::ImuCalBlob blob{};
    robot::imuCalPack(bias, mount, blob);
    CHECK(blob.magic == robot::kImuCalMagic);
    CHECK(blob.version == robot::kImuCalVersion);

    float b[3] = {0}, m[4] = {0};
    CHECK(robot::imuCalUnpack(blob, b, m));
    for (int i = 0; i < 3; ++i) CHECK_NEAR(b[i], bias[i], 1e-9);
    for (int i = 0; i < 4; ++i) CHECK_NEAR(m[i], mount[i], 1e-9);

    // A flipped bit anywhere must fail the CRC rather than load a plausible-
    // looking wrong calibration.
    robot::ImuCalBlob bad = blob;
    bad.gyro_bias[0] = 99.0f;
    CHECK(!robot::imuCalUnpack(bad, b, m));

    bad = blob;
    bad.magic ^= 1u;
    CHECK(!robot::imuCalUnpack(bad, b, m));

    // A zero quaternion passes CRC but is not a rotation -- reject it rather
    // than normalise it into something plausible.
    robot::ImuCalBlob zero{};
    const float nomount[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    robot::imuCalPack(bias, nomount, zero);
    CHECK(!robot::imuCalUnpack(zero, b, m));
}

void testImuMaths() {
    // Level: the torso z-axis is world +z.
    float up[3];
    imu::upFromQuaternion(1.0f, 0.0f, 0.0f, 0.0f, up);
    CHECK_NEAR(up[0], 0.0, 1e-6);
    CHECK_NEAR(up[1], 0.0, 1e-6);
    CHECK_NEAR(up[2], 1.0, 1e-6);

    // 90 deg pitch about +y tips the body z-axis onto world +x.
    const float s = 0.70710678f;
    imu::upFromQuaternion(s, 0.0f, s, 0.0f, up);
    CHECK_NEAR(up[0], 1.0, 1e-5);
    CHECK_NEAR(up[2], 0.0, 1e-5);

    // The mounting rotation is the firmware twin of walker_env's imu_R @ up.
    imu::Mount m;                      // identity
    const float in[3] = {0.1f, -0.2f, 0.97f};
    float out[3];
    imu::applyMount(m, in, out);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(out[i], in[i], 1e-6);

    m.w = s; m.z = s;                  // +90 deg about z
    imu::applyMount(m, in, out);
    CHECK_NEAR(out[0], 0.2, 1e-5);
    CHECK_NEAR(out[1], 0.1, 1e-5);
    CHECK_NEAR(out[2], 0.97, 1e-5);

    imu::StubImu stub;
    imu::Sample smp{};
    CHECK(stub.init());
    CHECK(stub.read(smp));
    CHECK(smp.valid);
    CHECK_NEAR(smp.up[2], 1.0, 1e-6);
}

}  // namespace

// -- calibration persistence + the steps clamp -----------------------------
// The blob format is plain data, so it is testable here even though the NVS
// side of cal_store only compiles under ESP-IDF.
void testCalBlob() {
    // v3: bus-indexed, every servo on the bus, with the fitted set.
    obs::Calibration cal;
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        cal.zero_steps[b] = 1000 + b * 37;
        cal.dir[b] = (b % 3 == 0) ? -1 : 1;
        cal.fitted[b] = b < 10 ? 1 : 0;              // the prototype
    }
    robot::CalBlob blob{};
    robot::calPack(cal, blob);
    CHECK_EQ(blob.version, 3);
    CHECK_EQ(blob.joints, obs::kNumBusJoints);

    obs::Calibration back;
    CHECK(robot::calUnpack(blob, back));   // round-trip unpack succeeds
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        CHECK(back.zero_steps[b] == cal.zero_steps[b]);   // zero survives
        CHECK(back.dir[b] == cal.dir[b]);                 // dir survives
        CHECK(back.fitted[b] == cal.fitted[b]);           // fitted survives
    }

    // Every rejection path must leave the destination untouched, so a bad
    // blob falls back to defaults instead of half-applying.
    obs::Calibration guard;
    const int32_t sentinel = guard.zero_steps[0];

    robot::CalBlob bad = blob;
    bad.crc ^= 0xFFu;
    CHECK(!robot::calUnpack(bad, guard));   // corrupt CRC rejected
    CHECK(guard.zero_steps[0] == sentinel);   // rejected blob did not mutate

    bad = blob; bad.magic = 0xDEADBEEF;
    CHECK(!robot::calUnpack(bad, guard));   // wrong magic rejected

    bad = blob; bad.version = 99;
    CHECK(!robot::calUnpack(bad, guard));   // wrong version rejected

    // A different joint list (8 -> 10 DOF once, 10 -> 17 servos now) is
    // exactly why the count is stored.
    bad = blob; bad.joints = 10;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // joint-count mismatch rejected

    bad = blob; bad.zero_steps[2] = 9999;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // out-of-range zero rejected

    bad = blob; bad.dir[1] = 7;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // invalid dir rejected

    bad = blob; bad.fitted[12] = 2;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // invalid fitted flag rejected

    // The blob only means anything under the servo map it was measured with
    // (the 2026-08-02 leg-swap errata: cal measured, THEN the map changed).
    // A blob carrying a different map must be rejected, not silently applied
    // to the wrong servos.
    bad = blob;
    const uint8_t tmp_id = bad.servo_id[0];
    bad.servo_id[0] = bad.servo_id[5];
    bad.servo_id[5] = tmp_id;
    bad.crc = robot::calCrc32(&bad, sizeof bad - sizeof bad.crc);
    CHECK(!robot::calUnpack(bad, guard));   // stale servo map rejected
    CHECK(guard.zero_steps[0] == sentinel);
    // ... and calPack records the CURRENT bus map, so a fresh cal round-trips.
    for (int b = 0; b < obs::kNumBusJoints; ++b) {
        CHECK_EQ(blob.servo_id[b], obs::kBusServoId[b]);
    }

    // v2 -> v3 (the automatic boot migration): a prototype v2 blob, joint-
    // indexed in the legacy map, lands each servo's zero and direction on
    // that servo's bus joint; those ten are fitted and IDs 11-17 are not.
    robot::CalBlobV2 v2{};
    v2.magic = robot::kCalMagic;
    v2.version = 2;
    v2.joints = robot::kLegacyJoints;
    for (int i = 0; i < robot::kLegacyJoints; ++i) {
        v2.zero_steps[i] = robot::kAsBuiltZeroSteps[i];
        v2.dir[i] = robot::kAsBuiltDir[i];
        v2.servo_id[i] = robot::kLegacyServoId[i];
    }
    v2.crc = robot::calCrc32(&v2, sizeof v2 - sizeof v2.crc);
    obs::Calibration m2;
    CHECK(robot::calUnpackV2(v2, m2));
    for (int i = 0; i < robot::kLegacyJoints; ++i) {
        const int b = obs::busIndexOfId(robot::kLegacyServoId[i]);
        if (b < 0) {
            CHECK(false);
            continue;
        }
        CHECK_EQ(m2.zero_steps[b], v2.zero_steps[i]);
        CHECK_EQ(m2.dir[b], v2.dir[i]);
        CHECK_EQ(m2.fitted[b], 1);
    }
    for (int id = 11; id <= 17; ++id) {
        CHECK_EQ(m2.fitted[obs::busIndexOfId(id)], 0);
    }
    // It is the as-built measurement, and after a v3 round trip it still is.
    CHECK(robot::calIsAsBuilt(m2));
    robot::CalBlob v3{};
    robot::calPack(m2, v3);
    obs::Calibration m3;
    CHECK(robot::calUnpack(v3, m3));
    CHECK(robot::calIsAsBuilt(m3));
    // The policy's view through the migrated cal equals the v2 blob's: the
    // prototype's joints read exactly as before the change.
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int b = obs::policyToBus(j);
        CHECK_NEAR(obs::stepsToAngle(j, 2048, m3),
                   static_cast<float>(2048 - m3.zero_steps[b]) *
                       (6.283185307f / 4096.0f) *
                       static_cast<float>(m3.dir[b]),
                   1e-6);
    }
    // A v2 blob under another map proves nothing and is refused.
    robot::CalBlobV2 v2bad = v2;
    v2bad.servo_id[0] = 9;
    v2bad.servo_id[5] = 10;
    v2bad.crc = robot::calCrc32(&v2bad, sizeof v2bad - sizeof v2bad.crc);
    CHECK(!robot::calUnpackV2(v2bad, guard));
    v2bad = v2; v2bad.crc ^= 1u;
    CHECK(!robot::calUnpackV2(v2bad, guard));    // corrupt v2 rejected
    CHECK(guard.zero_steps[0] == sentinel);

    // The explicit v1 migration path (`cal migrate`): a well-formed v1 blob
    // unpacks THERE (and only there -- boot's calUnpack requires v3).
    robot::CalBlobV1 v1{};
    v1.magic = robot::kCalMagic;
    v1.version = 1;
    v1.joints = robot::kLegacyJoints;
    for (int i = 0; i < robot::kLegacyJoints; ++i) {
        v1.zero_steps[i] = 1500 + i;
        v1.dir[i] = (i % 2) ? -1 : 1;
    }
    v1.crc = robot::calCrc32(&v1, sizeof v1 - sizeof v1.crc);
    obs::Calibration mig;
    CHECK(robot::calUnpackV1(v1, mig));
    for (int i = 0; i < robot::kLegacyJoints; ++i) {
        const int b = obs::busIndexOfId(robot::kLegacyServoId[i]);
        if (b < 0) {
            CHECK(false);
            continue;
        }
        CHECK(mig.zero_steps[b] == v1.zero_steps[i]);
        CHECK(mig.dir[b] == v1.dir[i]);
        CHECK(mig.fitted[b] == 1);
    }
    CHECK(!robot::calIsAsBuilt(mig));            // not the as-built values
    robot::CalBlobV1 v1bad = v1;
    v1bad.crc ^= 1u;
    CHECK(!robot::calUnpackV1(v1bad, guard));    // corrupt v1 rejected
    v1bad = v1; v1bad.version = 2;
    v1bad.crc = robot::calCrc32(&v1bad, sizeof v1bad - sizeof v1bad.crc);
    CHECK(!robot::calUnpackV1(v1bad, guard));    // v2 does not sneak in here

    // The auto-migration gate: any single-value drift from the as-built table
    // (a different measurement), or an unfitted prototype servo, is refused.
    obs::Calibration asb = m2;
    const int l_ankle = obs::busIndexOfId(8);
    asb.zero_steps[l_ankle] += 1;                // one tick off on L_ankle
    CHECK(!robot::calIsAsBuilt(asb));
    asb = m2;
    asb.dir[obs::busIndexOfId(7)] =
        static_cast<int8_t>(-asb.dir[obs::busIndexOfId(7)]);
    CHECK(!robot::calIsAsBuilt(asb));
    asb = m2;
    asb.fitted[obs::busIndexOfId(3)] = 0;
    CHECK(!robot::calIsAsBuilt(asb));

    CHECK(robot::calCrc32("123456789", 9) == 0xCBF43926u);   // CRC-32 matches the standard check value
}

void testStepsClamp() {
    // A wrong zero_steps must not produce a target off the encoder range --
    // that is a horn parked on a hard stop drawing stall current.
    obs::Calibration cal;
    for (int b = 0; b < obs::kNumBusJoints; ++b) cal.zero_steps[b] = 4090;
    float angles[obs::kNumJoints];
    float act[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) act[i] = 1.0f;
    obs::actionToAngles(act, angles);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int32_t st = obs::angleToSteps(i, angles[i], cal);
        CHECK(st >= 0 && st <= 4095);   // clamped high-side zero stays in range
    }
    for (int b = 0; b < obs::kNumBusJoints; ++b) cal.zero_steps[b] = 5;
    for (int i = 0; i < obs::kNumJoints; ++i) act[i] = -1.0f;
    obs::actionToAngles(act, angles);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        const int32_t st = obs::angleToSteps(i, angles[i], cal);
        CHECK(st >= 0 && st <= 4095);   // clamped low-side zero stays in range
    }
}

void testCommandShaper() {
    const float dt = obs::kControlDt;
    const float k = 1.0f - expf(-2.0f * 3.14159265358979f *
                                obs::kShaperPoleHz * dt);
    obs::CommandShaper sh;
    CHECK_NEAR(sh.poleHz(), obs::kShaperPoleHz, 0.0);

    // Seeding: after reset(q0), one update toward u lands exactly k^3 of the
    // way there (three cascaded stages, each stepping by k, seeded equal).
    float q0[obs::kNumJoints], u[obs::kNumJoints], y[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) {
        q0[i] = 0.1f * static_cast<float>(i) - 0.4f;
        u[i] = q0[i] + 0.5f;
    }
    sh.reset(q0);
    CHECK(sh.primed());
    sh.update(u, y);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_NEAR(y[i], q0[i] + k * k * k * 0.5f, 1e-5);
    }

    // Step response: monotone (all poles real -- no overshoot), converges,
    // and never leaves the [seed, target] hull.
    float zero[obs::kNumJoints] = {};
    float one[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) one[i] = 1.0f;
    sh.reset(zero);
    float prev = 0.0f;
    float resp[200];
    for (int t = 0; t < 200; ++t) {
        sh.update(one, y);
        resp[t] = y[0];
        CHECK(y[0] >= prev - 1e-7f);       // monotone
        CHECK(y[0] <= 1.0f + 1e-6f);       // no overshoot
        prev = y[0];
    }
    CHECK_NEAR(resp[199], 1.0f, 1e-4);     // converged (DC gain exactly 1)

    // Smoothness -- the reason this class exists. The discrete second
    // difference (acceleration) of the step response must be a small fraction
    // of the step; the RAW staircase concentrates the whole unit step in one
    // sample (|d2| == 1). This is "second derivative continuous" in the
    // sampled domain: bounded and slowly-varying, not impulsive.
    float d2max = 0.0f;
    for (int t = 2; t < 200; ++t) {
        const float d2 = fabsf((resp[t] - resp[t - 1]) -
                               (resp[t - 1] - resp[t - 2]));
        if (d2 > d2max) d2max = d2;
    }
    CHECK(d2max < 0.15f);                  // vs 1.0 for the raw staircase

    // In-place update is safe (ctrl_task shapes `angle` into itself).
    sh.reset(zero);
    float inout[obs::kNumJoints];
    for (int i = 0; i < obs::kNumJoints; ++i) inout[i] = 1.0f;
    sh.update(inout, inout);
    CHECK_NEAR(inout[0], k * k * k, 1e-5);

    // invalidate(): the next update self-seeds from its target (defensive
    // path; ctrl_task/sil reseed from measured q first).
    sh.invalidate();
    CHECK(!sh.primed());
    sh.update(u, y);
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(y[i], u[i], 0.0);

    // A pole change mid-run is continuous: state carries over, only the
    // per-stage gain changes.
    sh.setPole(3.0f, dt);
    CHECK_NEAR(sh.poleHz(), 3.0f, 0.0);
    sh.update(u, y);
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(y[i], u[i], 1e-6);
}

void testGoalSpeed() {
    const float dt = obs::kControlDt;
    // Zero error: the floor, never 0 (0 == UNLIMITED on the wire, which is
    // exactly the slam this path exists to remove).
    CHECK_EQ(obs::goalSpeedSteps(2048, 2048), obs::kGoalSpeedFloor);
    // Mid-range: |err| / dt with headroom, symmetric in sign.
    const int32_t want20 = static_cast<int32_t>(
        lrintf(20.0f / dt * obs::kGoalSpeedHeadroom));
    CHECK_EQ(obs::goalSpeedSteps(2068, 2048), want20);
    CHECK_EQ(obs::goalSpeedSteps(2028, 2048), want20);
    // Large error (a disturbance, or a joint that never answered): clamps to
    // the servo's max -- the legacy stiffness is the ceiling, not lost.
    CHECK_EQ(obs::goalSpeedSteps(4095, 0), obs::kGoalSpeedMax);
    CHECK_EQ(obs::goalSpeedSteps(0, 4095), obs::kGoalSpeedMax);
}

int main() {
    testSpecMatchesVectors();
    testServoIdMapIsAPermutation();
    testFramesMatchTheSim();
    testHistoryOrdering();
    testGaitClockWraps();
    testActionToAngles();
    testAngleToSteps();
    testBusAngleToStepsRaw();
    testBusMap();
    testVelocityEstimator();
    testCommandShaper();
    testGoalSpeed();
    testImuMaths();
    testCalBlob();
    testImuCalBlob();
    testStepsClamp();
    return testutil::report("obs");
}

// flailguard::Detector -- the armed loop's "the policy is doing something
// clearly wrong" guard (ctrl_task.cpp).
//
// Asserted:
//   * each criterion (pose / rate / tilt / chatter) trips on what it should
//     and not on what it should not: a short excursion, a joint inside its
//     limit, a walk command (pose only), the takeover ramp (pose only), a
//     smooth gait-like target, an attitude gap
//   * one event per run; reset() is a fresh run
//   * THE RECORDING: the 2026-11-01 policy-stand failure (vectors/
//     flail_vectors.h, tools/gen_flail_vectors.py) -- no criterion trips in
//     the clean first 3 s, and the guard trips by 3.5 s. Each criterion's own
//     trip time is printed (docs/first-policy-stand.md quotes them).
#include "flailguard/flailguard.h"

#include <math.h>
#include <stdio.h>

#include "test_util.h"
#include "vectors/flail_vectors.h"

using flailguard::Detector;
using flailguard::Input;
using flailguard::Limits;
using flailguard::Trip;

namespace {

constexpr uint32_t kDt = 20;   // ms, the 50 Hz loop
constexpr int kN = 12;
constexpr float kDeg = 1.0f / 57.29578f;

struct Rig {
    float target[kN] = {};
    float raw[kN] = {};
    float stand[kN] = {};
    float limit[kN];
    float up[3] = {0.0f, 0.0f, 1.0f};
    float gyro[3] = {0.0f, 0.0f, 0.0f};
    bool stand_cmd = true;
    bool pose_armed = true;
    bool imu_valid = true;
    bool walking = false;
    Rig() {
        for (float& l : limit) l = 15.0f * kDeg;
    }
    Input in() const {
        Input i;
        i.n = kN;
        i.target = target;
        i.raw = raw;
        i.stand = stand;
        i.pose_limit = limit;
        i.stand_cmd = stand_cmd;
        i.pose_armed = pose_armed;
        i.imu_valid = imu_valid;
        i.walking = walking;
        i.up = up;
        i.gyro = gyro;
        return i;
    }
    void tilt(float deg) {
        up[0] = 0.0f;
        up[1] = -sinf(deg * kDeg);
        up[2] = cosf(deg * kDeg);
    }
};

// Run `ms` of the rig unchanged; returns the first trip and the ms into the
// run it came at (or kNone).
Trip run(Detector& d, const Rig& r, uint32_t ms, uint32_t* at = nullptr) {
    for (uint32_t t = 0; t < ms; t += kDt) {
        const Trip e = d.update(r.in(), kDt);
        if (e != Trip::kNone) {
            if (at) *at = t;
            return e;
        }
    }
    return Trip::kNone;
}

void testTable() {
    const Limits& L = flailguard::kLimits;
    CHECK_EQ(L.pose_hold_ms, 300u);
    CHECK_NEAR(L.rate_rad_s, 1.5, 1e-6);
    CHECK_NEAR(L.rate_tau_s, 0.2, 1e-6);
    CHECK_NEAR(L.tilt_deg, 20.0, 1e-6);
    CHECK_EQ(L.tilt_hold_ms, 200u);
    CHECK_NEAR(L.chatter_rad_s, 7.0, 1e-6);
    CHECK_NEAR(L.chatter_tau_s, 0.1, 1e-6);
    CHECK_EQ(L.criteria, flailguard::kCritAll);
    // Earlier than the fall guard, which stays 45 deg (fallguard.h).
    CHECK(L.tilt_deg < 45.0f);
    // Every joint of both policy specs has a row; unknown names a default.
    const char* names[] = {"L_hip_yaw", "R_hip_roll", "L_hip_pitch", "R_knee",
                           "L_ankle", "R_ankle_roll", "neck", nullptr};
    for (const char* n : names) {
        const float l = flailguard::poseLimitDeg(n);
        CHECK(l >= 10.0f && l <= 30.0f);
    }
    CHECK_NEAR(flailguard::poseLimitDeg("R_knee"), 15.0, 1e-6);
    CHECK(!strcmp(flailguard::tripName(Trip::kChatter), "chatter"));
}

void testQuietStand() {
    // Small steady offsets, a still torso: ten seconds, nothing.
    Detector d;
    Rig r;
    for (int j = 0; j < kN; ++j) {
        r.target[j] = r.raw[j] = static_cast<float>(j % 3 - 1) * 4.0f * kDeg;
    }
    r.tilt(3.0f);
    r.gyro[0] = 0.05f;
    CHECK(run(d, r, 10000) == Trip::kNone);
    CHECK(!d.tripped());
    CHECK(d.peakTiltDeg() > 2.9f && d.peakTiltDeg() < 3.1f);
    CHECK(d.peakPoseDeg() > 3.9f && d.peakPoseDeg() < 4.1f);
}

void testPose() {
    {
        // One joint 16 deg out, held: trips exactly 300 ms after it started.
        Detector d;
        Rig r;
        r.target[3] = -16.0f * kDeg;
        uint32_t at = 0;
        CHECK(run(d, r, 2000, &at) == Trip::kPose);
        CHECK_EQ(at, 300u);
        CHECK_EQ(d.tripJoint(), 3);
        CHECK_NEAR(d.tripValue(), 16.0, 1e-3);
        CHECK_EQ(d.ticks(), at / kDt + 1);   // counted up to the tripping tick
        // ... and only once.
        CHECK(run(d, r, 2000) == Trip::kNone);
        CHECK(d.trip() == Trip::kPose);
        CHECK_EQ(d.ticks(), at / kDt + 1);   // a tripped guard stops counting
        d.reset();
        CHECK(!d.tripped());
        CHECK_EQ(d.ticks(), 0u);
    }
    {
        // 14 deg: inside the limit, forever.
        Detector d;
        Rig r;
        r.target[7] = 14.0f * kDeg;
        CHECK(run(d, r, 5000) == Trip::kNone);
    }
    {
        // 280 ms out, back for one tick, 280 ms out: the run restarts.
        Detector d;
        Rig r;
        for (int k = 0; k < 5; ++k) {
            r.target[1] = 20.0f * kDeg;
            CHECK(run(d, r, 280) == Trip::kNone);
            r.target[1] = 0.0f;
            CHECK(run(d, r, 20) == Trip::kNone);
        }
    }
    {
        // A walk command: the joints are meant to leave the stand pose.
        Detector d;
        Rig r;
        r.stand_cmd = false;
        r.target[2] = 40.0f * kDeg;
        CHECK(run(d, r, 3000) == Trip::kNone);
        // The same pose at the stand command trips.
        r.stand_cmd = true;
        CHECK(run(d, r, 3000) == Trip::kPose);
    }
    {
        // The takeover ramp blends from wherever the robot was armed: pose is
        // not judged until it is done.
        Detector d;
        Rig r;
        r.pose_armed = false;
        r.target[9] = -30.0f * kDeg;
        CHECK(run(d, r, 1000) == Trip::kNone);
        r.pose_armed = true;
        uint32_t at = 0;
        CHECK(run(d, r, 1000, &at) == Trip::kPose);
        CHECK_EQ(at, 300u);
    }
    {
        // Per-joint limits: a joint given 25 deg tolerates 20.
        Detector d;
        Rig r;
        r.limit[4] = 25.0f * kDeg;
        r.target[4] = 20.0f * kDeg;
        CHECK(run(d, r, 2000) == Trip::kNone);
    }
}

void testRate() {
    {
        // 2 rad/s held: the 0.2 s EMA crosses 1.5 at -0.2 ln(0.25) = 0.277 s.
        Detector d;
        Rig r;
        r.gyro[1] = 2.0f;
        uint32_t at = 0;
        CHECK(run(d, r, 2000, &at) == Trip::kRate);
        CHECK(at >= 240u && at <= 300u);
        CHECK(d.tripValue() > 1.5f);
    }
    {
        // A 60 ms 3 rad/s jolt (a footfall, a bump) does not.
        Detector d;
        Rig r;
        r.gyro[0] = 3.0f;
        CHECK(run(d, r, 60) == Trip::kNone);
        r.gyro[0] = 0.0f;
        CHECK(run(d, r, 2000) == Trip::kNone);
        CHECK(d.peakRate() < 1.5f);
    }
    {
        // 1.4 rad/s forever: under the line.
        Detector d;
        Rig r;
        r.gyro[2] = 1.4f;
        CHECK(run(d, r, 5000) == Trip::kNone);
    }
    {
        // An invalid attitude is no decision: a fast gyro on invalid samples
        // never trips anything.
        Detector d;
        Rig r;
        r.imu_valid = false;
        r.gyro[0] = 9.0f;
        r.tilt(60.0f);
        CHECK(run(d, r, 3000) == Trip::kNone);
        CHECK_NEAR(d.rateEma(), 0.0, 1e-9);
    }
}

void testTilt() {
    {
        Detector d;
        Rig r;
        r.tilt(25.0f);
        uint32_t at = 0;
        CHECK(run(d, r, 2000, &at) == Trip::kTilt);
        CHECK_EQ(at, 200u);
        CHECK_NEAR(d.tripValue(), 25.0, 0.01);
    }
    {
        Detector d;
        Rig r;
        r.tilt(19.0f);
        CHECK(run(d, r, 5000) == Trip::kNone);
    }
    {
        // A gap in the attitude breaks the run.
        Detector d;
        Rig r;
        for (int k = 0; k < 5; ++k) {
            r.tilt(30.0f);
            r.imu_valid = true;
            CHECK(run(d, r, 180) == Trip::kNone);
            r.imu_valid = false;
            CHECK(run(d, r, 20) == Trip::kNone);
        }
    }
}

void testChatter() {
    {
        // +/-8.6 deg sign-flipping every tick on every joint: 0.3 rad per tick
        // of target change = 15 rad/s. Trips within a few ticks.
        Detector d;
        Rig r;
        uint32_t at = 0;
        Trip got = Trip::kNone;
        for (int k = 0; k < 100 && got == Trip::kNone; ++k) {
            for (int j = 0; j < kN; ++j) r.raw[j] = (k & 1 ? 0.15f : -0.15f);
            got = d.update(r.in(), kDt);
            at = static_cast<uint32_t>(k) * kDt;
        }
        CHECK(got == Trip::kChatter);
        CHECK(at <= 200u);
    }
    {
        // A smooth gait-like target on every joint (1.5 Hz, +/-0.5 rad, mean
        // |rate| 3 rad/s, peak 4.7): not chatter.
        Detector d;
        Rig r;
        r.stand_cmd = false;    // a walk
        for (int k = 0; k < 500; ++k) {
            const float t = static_cast<float>(k) * 0.02f;
            for (int j = 0; j < kN; ++j) {
                r.raw[j] = r.target[j] = 0.5f * sinf(6.2831853f * 1.5f * t + 0.5f * static_cast<float>(j));
            }
            CHECK(d.update(r.in(), kDt) == Trip::kNone);
        }
        CHECK(d.peakChatter() > 2.0f && d.peakChatter() < 7.0f);
    }
    {
        // A non-finite target restarts the difference instead of poisoning it.
        Detector d;
        Rig r;
        r.raw[0] = NAN;
        CHECK(run(d, r, 200) == Trip::kNone);
        r.raw[0] = 0.0f;
        CHECK(run(d, r, 2000) == Trip::kNone);
        CHECK(isfinite(d.chatterEma()));
    }
}

void testCriteriaMask() {
    Limits L = flailguard::kLimits;
    L.criteria = flailguard::kCritPose;
    Detector d(L);
    Rig r;
    r.tilt(40.0f);
    r.gyro[0] = 5.0f;
    CHECK(run(d, r, 3000) == Trip::kNone);    // only pose judged
    CHECK(d.peakRate() > 4.0f);               // ... the others still measured
}

// -- the recording ------------------------------------------------------------

struct Replay {
    Trip trip = Trip::kNone;
    float t = -1.0f;        // s from the arm
    float value = 0.0f;
    int joint = -1;
    float peak_rate_3s = 0, peak_chatter_3s = 0, peak_pose_3s = 0, peak_tilt_3s = 0;
};

Replay replay(uint8_t criteria) {
    Limits L = flailguard::kLimits;
    L.criteria = criteria;
    Detector d(L);
    float stand[flailvec::kNumJoints] = {};
    float limit[flailvec::kNumJoints];
    for (int j = 0; j < flailvec::kNumJoints; ++j) {
        limit[j] = flailguard::poseLimitDeg(flailvec::kJointNames[j]) * kDeg;
    }
    Replay out;
    unsigned prev_tick = flailvec::kTicks[0].tick - 1;
    for (int k = 0; k < flailvec::kNumTicks; ++k) {
        const flailvec::Tick& tk = flailvec::kTicks[k];
        Input in;
        in.n = flailvec::kNumJoints;
        in.target = tk.target;
        in.raw = tk.raw;
        in.stand = stand;
        in.pose_limit = limit;
        // ctrl_task's stand gate: locomotion and lift at zero, crouch 1.
        in.stand_cmd = fabsf(tk.cmd[0]) <= 0.05f && fabsf(tk.cmd[1]) <= 0.05f &&
                       fabsf(tk.cmd[2]) <= 0.05f && fabsf(tk.cmd[3] - 1.0f) <= 0.05f &&
                       fabsf(tk.cmd[4]) < 0.5f;
        // The takeover ramp: 50 acting ticks (kArmRampMs 1000).
        in.pose_armed = k >= 50;
        in.imu_valid = tk.imu_valid;
        in.up = tk.up;
        in.gyro = tk.gyro;
        const uint32_t dt = 20u * (tk.tick - prev_tick);
        prev_tick = tk.tick;
        const Trip e = d.update(in, dt);
        if (tk.t < 3.0f) {
            out.peak_rate_3s = d.peakRate();
            out.peak_chatter_3s = d.peakChatter();
            out.peak_pose_3s = d.peakPoseDeg();
            out.peak_tilt_3s = d.peakTiltDeg();
        }
        if (e != Trip::kNone) {
            out.trip = e;
            out.t = tk.t;
            out.value = d.tripValue();
            out.joint = d.tripJoint();
            break;
        }
    }
    return out;
}

void testRecording() {
    // The fixture is the one this test was tuned on: armed at tick 11232, all
    // stand commands, 4 s.
    CHECK_EQ(flailvec::kNumJoints, 12);
    CHECK(flailvec::kNumTicks >= 195);
    CHECK(flailvec::kTicks[flailvec::kNumTicks - 1].t > 3.9f);

    struct Case {
        const char* name;
        uint8_t mask;
    };
    const Case cases[] = {
        {"pose", flailguard::kCritPose},       {"rate", flailguard::kCritRate},
        {"tilt", flailguard::kCritTilt},       {"chatter", flailguard::kCritChatter},
        {"ALL (as flashed)", flailguard::kCritAll},
    };
    Replay all;
    for (const Case& c : cases) {
        const Replay r = replay(c.mask);
        if (r.trip == Trip::kNone) {
            printf("  flail replay %-17s no trip in the first %.1f s\n", c.name,
                   static_cast<double>(flailvec::kTicks[flailvec::kNumTicks - 1].t));
        } else {
            printf("  flail replay %-17s trips at %.2f s (%s, %.2f%s%s%s)\n", c.name,
                   static_cast<double>(r.t), flailguard::tripName(r.trip),
                   static_cast<double>(r.value),
                   (r.trip == Trip::kPose || r.trip == Trip::kTilt) ? " deg" : " rad/s",
                   r.joint >= 0 ? ", " : "", r.joint >= 0 ? flailvec::kJointNames[r.joint] : "");
        }
        // Nothing in the clean first 3 s, whatever the criterion.
        CHECK(r.trip == Trip::kNone || r.t >= 3.0f);
        if (c.mask == flailguard::kCritAll) all = r;
    }
    printf("  clean 0-3 s peaks: rate EMA %.2f rad/s (limit %.1f), chatter EMA %.2f rad/s "
           "(limit %.1f), written-target deviation %.1f deg (limit 15, not sustained), "
           "tilt %.1f deg (limit %.0f)\n",
           static_cast<double>(all.peak_rate_3s), static_cast<double>(flailguard::kLimits.rate_rad_s),
           static_cast<double>(all.peak_chatter_3s),
           static_cast<double>(flailguard::kLimits.chatter_rad_s),
           static_cast<double>(all.peak_pose_3s), static_cast<double>(all.peak_tilt_3s),
           static_cast<double>(flailguard::kLimits.tilt_deg));
    // The guard as flashed trips by 3.5 s.
    CHECK(all.trip != Trip::kNone);
    CHECK(all.t >= 3.0f && all.t <= 3.5f);
    // Rate and chatter each catch it on their own by 3.5 s; pose by 3.6.
    const Replay rate = replay(flailguard::kCritRate);
    const Replay chat = replay(flailguard::kCritChatter);
    const Replay pose = replay(flailguard::kCritPose);
    CHECK(rate.trip == Trip::kRate && rate.t <= 3.5f);
    CHECK(chat.trip == Trip::kChatter && chat.t <= 3.5f);
    CHECK(pose.trip == Trip::kPose && pose.t <= 3.6f);
    // Margins in the clean 3 s: each filtered criterion stayed under 80 % of
    // its line, so the thresholds are not tuned to a hair.
    CHECK(all.peak_rate_3s < 0.8f * flailguard::kLimits.rate_rad_s);
    CHECK(all.peak_chatter_3s < 0.8f * flailguard::kLimits.chatter_rad_s);
    CHECK(all.peak_tilt_3s < 0.5f * flailguard::kLimits.tilt_deg);
}

// WalkRecovery (Tom 2026-11-05): a trip during a walk -> the stand, not a hold.
//   * a trip with no moving command this run, or after recent_ms of stand, holds
//   * a trip during a walk command, and one up to recent_ms after it ends (the
//     robot's 11-05 trip came 0.2 s after the one-cycle stride command), recovers
//   * once per run: a second trip holds; reset() is a fresh run
//   * the guard pauses for grace_ms, graceJustEnded() fires exactly once
//   * the command reaches the stand over ramp_ms, monotonically, never overshooting
// The walking rate limit (Tom 2026-11-05, for a 5-step walk): 2.5 rad/s while
// walking, 1.5 at the stand; tilt is the same either way.
void testWalkRate() {
    CHECK_NEAR(flailguard::kLimits.rate_walk_rad_s, 2.5f, 1e-6f);
    {   // 2.0 rad/s walking: under the walking line
        Detector d; Rig r; r.walking = true; r.stand_cmd = false; r.gyro[2] = 2.0f;
        CHECK(run(d, r, 5000) == Trip::kNone);
    }
    {   // the same 2.0 at the stand: trips, as before
        Detector d; Rig r; r.gyro[2] = 2.0f;
        CHECK(run(d, r, 2000) == Trip::kRate);
    }
    {   // 3.0 rad/s walking: trips on the walking line
        Detector d; Rig r; r.walking = true; r.stand_cmd = false; r.gyro[1] = 3.0f;
        CHECK(run(d, r, 2000) == Trip::kRate);
        CHECK(d.tripValue() > 2.5f);
    }
    {   // tilt is unchanged while walking: 25 deg for 200 ms still trips
        Detector d; Rig r; r.walking = true; r.stand_cmd = false; r.tilt(25.0f);
        CHECK(run(d, r, 1000) == Trip::kTilt);
    }
    {   // WalkRecovery::walkingRecent: a walk now / within recent_ms, and not once recovered
        flailguard::WalkRecovery w;
        CHECK(!w.walkingRecent());
        w.tick(true, kDt);
        CHECK(w.walkingRecent());
        for (int i = 0; i < 100; ++i) w.tick(false, kDt);     // 2000 ms of stand
        CHECK(w.walkingRecent());
        w.tick(false, kDt);                                     // 2020 ms
        CHECK(!w.walkingRecent());
        w.tick(true, kDt);
        CHECK(w.onTrip());
        CHECK(!w.walkingRecent());                              // recovering: the stand line
    }
}

void testWalkRecovery() {
    using flailguard::WalkRecovery;
    WalkRecovery::Config c;
    c.recent_ms = 2000; c.grace_ms = 2000; c.ramp_ms = 300;

    {   // stand only: holds
        WalkRecovery w(c);
        for (int i = 0; i < 100; ++i) w.tick(false, kDt);
        CHECK(w.judging());
        CHECK(!w.onTrip());
        CHECK(!w.standOverride());
        CHECK(w.overrideFrac() == 0.0f);
    }
    {   // a stand long after a walk: holds
        WalkRecovery w(c);
        for (int i = 0; i < 25; ++i) w.tick(true, kDt);
        for (int i = 0; i < 101; ++i) w.tick(false, kDt);   // 2020 ms of stand
        CHECK(!w.onTrip());
    }
    {   // the 11-05 case: the trip 200 ms after the stride command ended -> recovers
        WalkRecovery w(c);
        for (int i = 0; i < 100; ++i) w.tick(false, kDt);     // 2 s stand
        for (int i = 0; i < 33; ++i) w.tick(true, kDt);       // 0.667 s stride command
        for (int i = 0; i < 10; ++i) w.tick(false, kDt);      // 200 ms later
        CHECK(w.onTrip());
        CHECK(w.standOverride());
        CHECK(!w.judging());                                  // grace: the guard pauses
        // the ramp: monotonic, 0 -> 1 over 300 ms, then held at 1
        float prev = w.overrideFrac();
        CHECK(prev == 0.0f);
        bool ended = false;
        int ends = 0;
        for (int i = 1; i <= 150; ++i) {                      // 3 s
            w.tick(i < 20, kDt);                               // the link may still say "walk"
            const float f = w.overrideFrac();
            CHECK(f >= prev && f <= 1.0f);
            prev = f;
            if (i * static_cast<int>(kDt) >= 300) CHECK(f == 1.0f);
            if (w.graceJustEnded()) { ++ends; ended = true; CHECK(i * static_cast<int>(kDt) == 2000); }
            CHECK(w.judging() == (i * static_cast<int>(kDt) >= 2000));
        }
        CHECK(ended && ends == 1);
        CHECK(w.standOverride());                              // latched for the run
        CHECK(!w.onTrip());                                    // a second trip holds
        w.reset();                                             // a fresh run
        CHECK(!w.standOverride() && w.judging() && w.overrideFrac() == 0.0f);
        CHECK(!w.onTrip());                                    // no walk in this run yet
    }
    {   // ramp 0: the stand at once
        WalkRecovery::Config c0 = c; c0.ramp_ms = 0;
        WalkRecovery w(c0);
        w.tick(true, kDt);
        CHECK(w.onTrip());
        CHECK(w.overrideFrac() == 1.0f);
    }
}

}  // namespace

int main() {
    testTable();
    testQuietStand();
    testPose();
    testRate();
    testTilt();
    testChatter();
    testCriteriaMask();
    testRecording();
    testWalkRecovery();
    testWalkRate();
    return testutil::report("test_flailguard");
}

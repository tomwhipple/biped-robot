// The observation dump: double buffer + CSV wire format (main/obs_dump.h).
//
// This is the half of the instrument that can be wrong silently. A torn
// record -- tick t's q beside tick t+1's frame -- would manufacture exactly
// the sign/scale disagreement the dump exists to find, and a truncated line
// would be mis-parsed by the host tool rather than dropped. Both are checked
// here, off-target, before anything is flashed.
#include <stdlib.h>
#include <string.h>

#include "../main/joint_pose.h"
#include "../main/obs_dump.h"
#include "test_util.h"

namespace {

using robot::ObsDump;
using robot::ObsDumpMode;
using robot::ObsRecord;

// A record whose every channel is distinguishable from every other, so a
// transposition or an off-by-one in the CSV writer cannot pass.
obs::Inputs makeInputs(float base) {
    obs::Inputs in{};
    for (int i = 0; i < obs::kNumJoints; ++i) {
        in.q[i] = base + 0.01f * static_cast<float>(i);
        in.dq[i] = base + 0.10f * static_cast<float>(i);
    }
    for (int i = 0; i < 3; ++i) {
        in.up[i] = base + 1.0f * static_cast<float>(i);
        in.gyro[i] = base + 2.0f * static_cast<float>(i);
    }
    in.phase = base + 3.0f;
    for (int i = 0; i < obs::kNumCmd; ++i) {
        in.cmd[i] = base + 10.0f * static_cast<float>(i);
    }
    return in;
}

void makeFrame(float base, float* frame) {
    for (int i = 0; i < obs::kFrameDim; ++i) {
        frame[i] = base + 100.0f * static_cast<float>(i);
    }
}

void checkMatches(const ObsRecord& r, const obs::Inputs& in, const float* frame,
                  uint32_t tick) {
    CHECK_EQ(r.tick, tick);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        CHECK_EQ(r.q[i] == in.q[i], true);
        CHECK_EQ(r.dq[i] == in.dq[i], true);
    }
    for (int i = 0; i < 3; ++i) {
        CHECK_EQ(r.up[i] == in.up[i], true);
        CHECK_EQ(r.gyro[i] == in.gyro[i], true);
    }
    CHECK_EQ(r.phase == in.phase, true);
    for (int i = 0; i < obs::kNumCmd; ++i) CHECK_EQ(r.cmd[i] == in.cmd[i], true);
    for (int i = 0; i < obs::kFrameDim; ++i) {
        CHECK_EQ(r.frame[i] == frame[i], true);
    }
}

// -- double buffer ---------------------------------------------------------

void testEmptyBufferReadsNothing() {
    ObsDump d;
    ObsRecord r{};
    uint32_t seq = 12345;
    CHECK(!d.read(r, seq));
    CHECK_EQ(d.sequence(), 0);
    CHECK_EQ(seq, 12345);            // untouched on failure
    CHECK(!d.enabled());
    CHECK(d.mode() == ObsDumpMode::kOff);
}

void testPublishReadRoundTrip() {
    ObsDump d;
    const obs::Inputs in = makeInputs(0.5f);
    float frame[obs::kFrameDim];
    makeFrame(-0.25f, frame);
    d.publish(in, frame, 7);

    ObsRecord r{};
    uint32_t seq = 0;
    CHECK(d.read(r, seq));
    CHECK_EQ(seq, 1);
    CHECK_EQ(d.sequence(), 1);
    checkMatches(r, in, frame, 7);
}

// Every publish must land in the slot the reader is NOT holding, and the
// reader must always come back with the newest record -- including across the
// slot flip, which is where an off-by-one in the index arithmetic hides.
void testAlternatingSlots() {
    ObsDump d;
    float frame[obs::kFrameDim];
    uint32_t prev_seq = 0;
    for (uint32_t tick = 1; tick <= 9; ++tick) {
        const obs::Inputs in = makeInputs(static_cast<float>(tick));
        makeFrame(static_cast<float>(tick) * 0.5f, frame);
        d.publish(in, frame, tick);

        ObsRecord r{};
        uint32_t seq = 0;
        CHECK(d.read(r, seq));
        CHECK_EQ(seq, tick);
        CHECK(seq != prev_seq);
        prev_seq = seq;
        checkMatches(r, in, frame, tick);
    }
}

// A record the reader already printed must be recognisable as stale: the
// streamer decides "the loop published nothing" purely from the sequence.
void testSequenceIsStableWithoutPublish() {
    ObsDump d;
    const obs::Inputs in = makeInputs(1.0f);
    float frame[obs::kFrameDim];
    makeFrame(1.0f, frame);
    d.publish(in, frame, 1);
    ObsRecord r{};
    uint32_t a = 0, b = 0;
    CHECK(d.read(r, a));
    CHECK(d.read(r, b));
    CHECK_EQ(a, b);
}

void testModeIsIndependentOfData() {
    ObsDump d;
    CHECK(!d.enabled());
    d.setMode(ObsDumpMode::kStream);
    CHECK(d.enabled());
    CHECK(d.mode() == ObsDumpMode::kStream);
    d.setMode(ObsDumpMode::kOnce);
    CHECK(d.enabled());
    d.setMode(ObsDumpMode::kOff);
    CHECK(!d.enabled());
    // Turning the dump off does not discard what was already published.
    const obs::Inputs in = makeInputs(2.0f);
    float frame[obs::kFrameDim];
    makeFrame(2.0f, frame);
    d.publish(in, frame, 3);
    d.setMode(ObsDumpMode::kOff);
    ObsRecord r{};
    uint32_t seq = 0;
    CHECK(d.read(r, seq));
    CHECK_EQ(r.tick, 3);
}

// -- CSV -------------------------------------------------------------------

int countFields(const char* line) {
    int n = 1;
    for (const char* p = line; *p && *p != '\r' && *p != '\n'; ++p) {
        if (*p == ',') ++n;
    }
    return n;
}

// Split a CSV line into its comma-separated fields (destructive, like the
// firmware's own tokeniser).
int splitFields(char* line, char** out, int max) {
    int n = 0;
    char* p = line;
    out[n++] = p;
    while (*p && *p != '\r' && *p != '\n') {
        if (*p == ',') {
            *p = 0;
            if (n < max) out[n++] = p + 1;
        }
        ++p;
    }
    *p = 0;
    return n;
}

void testRecordLineShape() {
    ObsDump d;
    const obs::Inputs in = makeInputs(0.125f);
    float frame[obs::kFrameDim];
    makeFrame(-3.5f, frame);
    d.publish(in, frame, 4242);
    ObsRecord r{};
    uint32_t seq = 0;
    CHECK(d.read(r, seq));

    char line[robot::kObsDumpLineMax];
    const size_t n = robot::formatObsRecord(r, line, sizeof line);
    CHECK(n > 0);
    CHECK_EQ(strlen(line), n);
    // Terminated like every other console line, and tagged so the host tool
    // can pick it out of mixed output.
    CHECK_EQ(strncmp(line, "OBS,", 4), 0);
    CHECK_EQ(strcmp(line + n - 2, "\r\n"), 0);
    // Tag plus every value column.
    CHECK_EQ(countFields(line), robot::kObsDumpCols + 1);

    char buf[robot::kObsDumpLineMax];
    memcpy(buf, line, n + 1);
    char* f[robot::kObsDumpCols + 2];
    const int nf = splitFields(buf, f, robot::kObsDumpCols + 2);
    CHECK_EQ(nf, robot::kObsDumpCols + 1);
    CHECK_EQ(strcmp(f[0], "OBS"), 0);
    CHECK_EQ(atoi(f[1]), 4242);
    // The values must survive the round trip to six significant digits, in
    // the documented order: q, dq, up, gyro, phase, cmd, frame.
    int c = 2;
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(atof(f[c++]), r.q[i], 1e-5);
    for (int i = 0; i < obs::kNumJoints; ++i) CHECK_NEAR(atof(f[c++]), r.dq[i], 1e-5);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(atof(f[c++]), r.up[i], 1e-5);
    for (int i = 0; i < 3; ++i) CHECK_NEAR(atof(f[c++]), r.gyro[i], 1e-5);
    CHECK_NEAR(atof(f[c++]), r.phase, 1e-5);
    for (int i = 0; i < obs::kNumCmd; ++i) CHECK_NEAR(atof(f[c++]), r.cmd[i], 1e-5);
    for (int i = 0; i < obs::kFrameDim; ++i) {
        CHECK_NEAR(atof(f[c++]), r.frame[i], 1e-2);   // up to 4800 here
    }
    CHECK_EQ(c, robot::kObsDumpCols + 1);
}

void testHeaderMatchesRecord() {
    char hdr[robot::kObsDumpLineMax];
    const size_t n = robot::formatObsHeader(hdr, sizeof hdr);
    CHECK(n > 0);
    CHECK_EQ(strncmp(hdr, "OBSHDR,", 7), 0);
    // One name per value column -- the host tool trusts this to label its
    // summary, so a mismatch would silently rename every channel.
    CHECK_EQ(countFields(hdr), robot::kObsDumpCols + 1);

    char buf[robot::kObsDumpLineMax];
    memcpy(buf, hdr, n + 1);
    char* f[robot::kObsDumpCols + 2];
    const int nf = splitFields(buf, f, robot::kObsDumpCols + 2);
    CHECK_EQ(nf, robot::kObsDumpCols + 1);
    CHECK_EQ(strcmp(f[1], "tick"), 0);
    // Joint names come from the generated spec, not a second copy of the list.
    CHECK(strstr(f[2], obs::kJointNames[0]) != nullptr);
    CHECK_EQ(strncmp(f[2], "q_", 2), 0);
    CHECK_EQ(strncmp(f[2 + obs::kNumJoints], "dq_", 3), 0);
    CHECK_EQ(strcmp(f[2 + 2 * obs::kNumJoints], "up_x"), 0);
    CHECK_EQ(strcmp(f[2 + 2 * obs::kNumJoints + 3], "gyro_x"), 0);
    CHECK_EQ(strcmp(f[2 + 2 * obs::kNumJoints + 6], "phase"), 0);
    CHECK_EQ(strcmp(f[2 + 2 * obs::kNumJoints + 7], "cmd_vx"), 0);
    // The frame block is named by SECTION, so a wrong assembler offset shows
    // up as a channel whose name disagrees with its neighbours.
    const int frame0 = robot::kObsDumpCols + 1 - obs::kFrameDim;
    CHECK_EQ(strncmp(f[frame0 + obs::kOffQ], "f_q_", 4), 0);
    CHECK_EQ(strncmp(f[frame0 + obs::kOffDq], "f_dq_", 5), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffUp], "f_up_x"), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffLinVel], "f_linvel_x"), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffGyro], "f_gyro_x"), 0);
    CHECK_EQ(strncmp(f[frame0 + obs::kOffPrevAction], "f_pa_", 5), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffHeight], "f_height"), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffPhase], "f_sin"), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffPhase + 1], "f_cos"), 0);
    CHECK_EQ(strcmp(f[frame0 + obs::kOffCmd], "f_cmd_vx"), 0);
}

// A truncated record is worse than a missing one: the host would parse the
// short line as a valid record with the wrong channel count. Both writers
// must refuse rather than clip.
void testTruncationRefuses() {
    ObsRecord r{};
    r.tick = 1;
    char small[32];
    memset(small, 'x', sizeof small);
    CHECK_EQ(robot::formatObsRecord(r, small, sizeof small), 0);
    CHECK_EQ(robot::formatObsHeader(small, sizeof small), 0);
    // ... and must not have run off the end of the caller's buffer.
    CHECK(strlen(small) < sizeof small);
}

// The worst case the line buffer has to hold: every channel a long negative
// exponent-form value. If this does not fit, a real capture would go silent
// exactly when the numbers are most interesting.
void testWorstCaseLineFits() {
    ObsDump d;
    obs::Inputs in{};
    for (int i = 0; i < obs::kNumJoints; ++i) {
        in.q[i] = -1.234567e-21f;
        in.dq[i] = -9.876543e+21f;
    }
    for (int i = 0; i < 3; ++i) { in.up[i] = -1.234567e-21f; in.gyro[i] = -1.234567e-21f; }
    in.phase = -1.234567e-21f;
    for (int i = 0; i < obs::kNumCmd; ++i) in.cmd[i] = -1.234567e-21f;
    float frame[obs::kFrameDim];
    for (int i = 0; i < obs::kFrameDim; ++i) frame[i] = -9.876543e-21f;
    d.publish(in, frame, 4294967295u);

    ObsRecord r{};
    uint32_t seq = 0;
    CHECK(d.read(r, seq));
    char line[robot::kObsDumpLineMax];
    const size_t n = robot::formatObsRecord(r, line, sizeof line);
    CHECK(n > 0);
    CHECK(n < robot::kObsDumpLineMax);
    CHECK_EQ(countFields(line), robot::kObsDumpCols + 1);

    char hdr[robot::kObsDumpLineMax];
    CHECK(robot::formatObsHeader(hdr, sizeof hdr) > 0);
}

// The dump's whole point is measuring real sensor numbers; a NaN out of a
// dead IMU must reach the host as a NaN, not as a dropped line.
void testNonFiniteSurvives() {
    ObsDump d;
    obs::Inputs in = makeInputs(0.0f);
    in.up[0] = 0.0f / 0.0f;
    float frame[obs::kFrameDim];
    makeFrame(0.0f, frame);
    d.publish(in, frame, 1);
    ObsRecord r{};
    uint32_t seq = 0;
    CHECK(d.read(r, seq));
    char line[robot::kObsDumpLineMax];
    CHECK(robot::formatObsRecord(r, line, sizeof line) > 0);
    CHECK(strstr(line, "nan") != nullptr);
    CHECK_EQ(countFields(line), robot::kObsDumpCols + 1);
}

void testColumnCountMatchesTheSpec() {
    CHECK_EQ(robot::kObsDumpCols,
             1 + 2 * obs::kNumJoints + 3 + 3 + 1 + obs::kNumCmd +
                 obs::kFrameDim);
    CHECK_EQ(robot::kObsDumpCols, 84);       // 10-DOF, 7 cmd, 49-wide frame
    CHECK_EQ(robot::kObsDumpPeriodMs, 200);
}

// -- the mirror-mode pose buffer (main/joint_pose.h) --------------------------
// Same double-buffer-plus-counter as ObsDump, so the same three properties:
// nothing before the first publish, the newest pose after any number of
// publishes (both slots), and a sequence that stands still when the loop does
// -- which is what wifi_link.cpp uses to tell a fresh pose from a stale one.

void poseOf(float base, float* q) {
    for (int i = 0; i < obs::kNumJoints; ++i) {
        q[i] = base + 0.01f * static_cast<float>(i);
    }
}

void testJointPoseEmptyReadsNothing() {
    robot::JointPose p;
    float q[obs::kNumJoints];
    uint32_t seq = 99;
    CHECK_EQ(p.sequence(), 0u);
    CHECK_EQ(p.read(q, seq), false);
    CHECK_EQ(seq, 99u);   // untouched on failure
}

void testJointPoseNewestWins() {
    robot::JointPose p;
    float in[obs::kNumJoints], out[obs::kNumJoints];
    uint32_t seq = 0, tick = 0;
    int64_t t_us = 0;
    // Enough publishes to use both slots several times over.
    for (uint32_t k = 1; k <= 7; ++k) {
        poseOf(static_cast<float>(k), in);
        p.publish(in, 1000u + k, 20000 * static_cast<int64_t>(k));
        CHECK_EQ(p.sequence(), k);
        CHECK_EQ(p.read(out, seq, &tick, &t_us), true);
        CHECK_EQ(seq, k);
        CHECK_EQ(tick, 1000u + k);
        // The read instant travels with the pose it belongs to.
        CHECK_EQ(t_us == 20000 * static_cast<int64_t>(k), true);
        for (int i = 0; i < obs::kNumJoints; ++i) {
            CHECK_EQ(out[i] == in[i], true);
        }
    }
}

void testJointPoseSequenceIsStillWhenTheLoopIs() {
    // wifi_link.cpp's freshness rule: the same sequence twice means ctrl is
    // benched and the beacon must drop back to the classic frame.
    robot::JointPose p;
    float in[obs::kNumJoints], out[obs::kNumJoints];
    poseOf(3.0f, in);
    p.publish(in, 1, 0);
    uint32_t s1 = 0, s2 = 0;
    CHECK_EQ(p.read(out, s1), true);
    CHECK_EQ(p.read(out, s2), true);
    CHECK_EQ(s1, s2);
    p.publish(in, 2, 0);
    CHECK_EQ(p.read(out, s2), true);
    CHECK_EQ(s2, s1 + 1u);
}

void testJointPoseIsTheWireWidth() {
    // The beacon memcpy's the buffer straight into linkproto::Telemetry
    // .joints; shared.h static_asserts the counts agree, this pins the
    // payload size that assertion is protecting.
    CHECK_EQ(sizeof(float) * static_cast<size_t>(obs::kNumJoints), 40u);
}

}  // namespace

int main() {
    testEmptyBufferReadsNothing();
    testPublishReadRoundTrip();
    testAlternatingSlots();
    testSequenceIsStableWithoutPublish();
    testModeIsIndependentOfData();
    testRecordLineShape();
    testHeaderMatchesRecord();
    testTruncationRefuses();
    testWorstCaseLineFits();
    testNonFiniteSurvives();
    testColumnCountMatchesTheSpec();
    testJointPoseEmptyReadsNothing();
    testJointPoseNewestWins();
    testJointPoseSequenceIsStillWhenTheLoopIs();
    testJointPoseIsTheWireWidth();
    return testutil::report("obsdump");
}

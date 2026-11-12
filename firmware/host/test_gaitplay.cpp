// gaitplay::Track -- the on-board gait track behind `track` (#297 B2), and the phase -> support mapping
// the stance trim shares with a policy's gait clock.
#include "gaitplay/track.h"

#include <math.h>
#include <string.h>

#include "test_util.h"

using gaitplay::Frame;
using gaitplay::Sample;
using gaitplay::State;
using gaitplay::Track;

namespace {

Frame frame(uint32_t t, int16_t base, float phi, uint8_t flags = 0) {
    Frame f;
    f.t_ms = t;
    for (int j = 0; j < gaitplay::kJoints; ++j) f.q[j] = static_cast<int16_t>(base + j);
    f.phi = phi;
    f.flags = flags;
    return f;
}

void testParseFormatRoundTrip() {
    Frame f = frame(1100, 2000, 1.0f, gaitplay::kLift | gaitplay::kStepEnd);
    char hex[43];
    gaitplay::formatFrame(f, 1000, hex);
    CHECK_EQ(strlen(hex), 42);
    Frame g;
    CHECK(gaitplay::parseFrame(hex, 1000, &g));
    CHECK_EQ(g.t_ms, 1100);
    for (int j = 0; j < gaitplay::kJoints; ++j) CHECK_EQ(g.q[j], f.q[j]);
    CHECK_NEAR(g.phi, 1.0f, 6.2832 / 256.0);
    CHECK_EQ(g.flags, f.flags);
    CHECK(!gaitplay::parseFrame("xyz", 0, &g));
    char bad[43];
    memcpy(bad, hex, 43);
    bad[10] = 'g';
    CHECK(!gaitplay::parseFrame(bad, 0, &g));
}

void testPassesThroughKeyframesAndAppendsWhilePlaying() {
    Track tr;
    CHECK(tr.add(frame(0, 1000, 0.0f)));
    CHECK(tr.add(frame(100, 1100, 0.3f)));
    CHECK(!tr.add(frame(100, 1200, 0.4f)));        // not after the last: refused
    CHECK(tr.start());
    Sample s;
    tr.sample(0, &s);
    CHECK_EQ(s.q[0], 1000);
    tr.sample(100, &s);                            // the last frame, nothing after it yet: held
    CHECK_EQ(s.q[0], 1100);
    // appended while playing: playback continues into it
    CHECK(tr.add(frame(200, 1300, 0.6f)));
    CHECK(tr.add(frame(300, 1400, 0.9f)));
    tr.sample(150, &s);
    CHECK(s.q[0] > 1100 && s.q[0] < 1300);
    tr.sample(200, &s);
    CHECK_EQ(s.q[0], 1300);
    CHECK_NEAR(s.phi, 0.6f, 1e-4);
    CHECK(tr.state() == State::kPlaying);
    // a linear ramp is reproduced exactly between interior keyframes
    Track lin;
    for (int k = 0; k < 6; ++k) CHECK(lin.add(frame(static_cast<uint32_t>(100 * k), static_cast<int16_t>(1000 + 100 * k), 0.0f)));
    CHECK(lin.start());
    for (uint32_t t = 0; t <= 300; t += 20) lin.sample(t, &s);
    CHECK_EQ(s.q[0], 1300);
    lin.sample(250, &s);
    CHECK_EQ(s.q[0], 1300);                        // monotone clock: an earlier time is not rewound
}

void testStopEndsAtStepEndThenRests() {
    Track tr;
    CHECK(tr.add(frame(0, 1000, 0.0f, gaitplay::kLift)));
    CHECK(tr.add(frame(100, 1100, 0.5f, gaitplay::kLift)));
    CHECK(tr.add(frame(200, 1200, 1.0f, gaitplay::kStepEnd)));
    Frame rest = frame(200, 900, 0.0f, gaitplay::kRest);
    CHECK(tr.add(rest));
    CHECK(tr.add(frame(300, 1300, 1.5f, gaitplay::kLift)));
    CHECK(tr.add(frame(400, 1400, 2.0f, gaitplay::kStepEnd)));
    CHECK(tr.start());
    Sample s;
    tr.sample(50, &s);
    CHECK(s.lift);
    tr.stop();
    CHECK(tr.state() == State::kStopping);
    CHECK(!tr.add(frame(500, 1500, 2.5f)));       // nothing is appended after a stop
    tr.sample(150, &s);
    CHECK(tr.state() == State::kStopping);
    tr.sample(200, &s);                            // the step end: resting toward the rest pose
    CHECK(s.step_end);
    CHECK(tr.state() == State::kResting);
    CHECK_EQ(tr.queued(), 0);                      // what was queued after the step end is dropped
    CHECK(tr.haveRest());
    tr.sample(200 + tr.restMs(), &s);
    CHECK(tr.state() == State::kDone);
    CHECK_EQ(s.q[0], 900);
    tr.sample(5000, &s);
    CHECK_EQ(s.q[0], 900);
    CHECK_EQ(tr.steps(), 1);
}

void testDryAtStepEndRestsAndDryMidStepHolds() {
    Track tr;
    CHECK(tr.add(frame(0, 1000, 0.0f)));
    CHECK(tr.add(frame(100, 1100, 0.5f, gaitplay::kStepEnd)));
    CHECK(tr.add(frame(100, 800, 0.0f, gaitplay::kRest)));
    CHECK(tr.start());
    Sample s;
    tr.sample(100, &s);
    CHECK(tr.state() == State::kResting);
    tr.sample(100 + tr.restMs() + 20, &s);
    CHECK_EQ(s.q[0], 800);
    Track mid;
    CHECK(mid.add(frame(0, 1000, 0.0f, gaitplay::kLift)));
    CHECK(mid.add(frame(100, 1100, 0.5f, gaitplay::kLift)));
    CHECK(mid.start());
    mid.sample(300, &s);
    CHECK(mid.state() == State::kPlaying);
    CHECK_EQ(s.q[0], 1100);
    CHECK(!s.lift);
}

void testSupportFromPhase() {
    using gaitplay::supportFromPhase;
    const float pi = 3.14159265f;
    CHECK_EQ(supportFromPhase(0.5f * pi, false).stance, 0);          // no lift: no support state
    CHECK_EQ(supportFromPhase(0.1f * pi, true).stance, 0);           // double support
    auto a = supportFromPhase(0.25f * pi + 0.01f, true);             // left swings: right stance, raising
    CHECK_EQ(a.stance, 'R');
    CHECK(a.progress < 0.1f);
    auto b = supportFromPhase(0.5f * pi, true);
    CHECK_EQ(b.stance, 'R');
    CHECK_NEAR(b.progress, 1.0f, 1e-6);
    CHECK(!b.lowering);
    auto c = supportFromPhase(0.7f * pi, true);
    CHECK(c.lowering);
    auto d = supportFromPhase(1.5f * pi, true);                      // right swings: left stance
    CHECK_EQ(d.stance, 'L');
    CHECK_EQ(supportFromPhase(NAN, true).stance, 0);
}

}  // namespace

int main() {
    testParseFormatRoundTrip();
    testPassesThroughKeyframesAndAppendsWhilePlaying();
    testStopEndsAtStepEndThenRests();
    testDryAtStepEndRestsAndDryMidStepHolds();
    testSupportFromPhase();
    return testutil::report("gaitplay");
}

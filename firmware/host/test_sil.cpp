// The SIL library's C boundary, end to end (docs/sil-harness.md pyramid 1).
//
// What is actually at stake here, in the order the tests appear:
//   * the calibration map survives a round trip through the encoder's 4096
//     quantisation under a NOMINAL and a PERTURBED calibration -- a wrong
//     zero or a flipped dir is the classic bring-up failure;
//   * the kServoId permutation is applied on BOTH sides of the boundary. On
//     the 10-DOF plant this is not the identity (wiring.md said it was), so
//     the test asserts an identity mapping would give a different answer;
//   * the exported-weight reader agrees, float for float, with an
//     independently written forward pass -- byte order, row-major kernels,
//     swish, the 2*act_dim head and the tanh mode;
//   * one tick with zeroed sensors produces the documented obs layout and
//     finite outputs.
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "obs/actuation.h"
#include "obs/obs_spec.h"
#include "scsbus/registers.h"
#include "sil.h"
#include "sil_json.h"
#include "sil_net.h"
#include "test_util.h"

namespace {

constexpr float kTwoPi = 6.283185307179586f;
constexpr float kQuantum = kTwoPi / 4096.0f;   // one encoder tick, radians

// Deterministic, dependency-free PRNG -- the perturbed calibration and the
// synthetic net must be identical on every machine that runs the gate.
struct Rng {
    uint64_t s = 0x2545F4914F6CDD1Dull;
    uint32_t next() {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<uint32_t>(s >> 32);
    }
    // Uniform in [lo, hi).
    float uniform(float lo, float hi) {
        const float u = static_cast<float>(next() & 0xFFFFFFu) /
                        static_cast<float>(0x1000000u);
        return lo + (hi - lo) * u;
    }
    int range(int lo, int hi) {   // inclusive
        return lo + static_cast<int>(next() % static_cast<uint32_t>(hi - lo + 1));
    }
};

// -- file helpers ----------------------------------------------------------

std::string g_tmp_prefix = "sil_test_";

bool writeText(const std::string& path, const std::string& text) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    const size_t n = fwrite(text.data(), 1, text.size(), f);
    fclose(f);
    return n == text.size();
}

bool writeF32LE(const std::string& path, const std::vector<float>& v) {
    FILE* f = fopen(path.c_str(), "wb");
    if (!f) return false;
    for (float x : v) {
        uint32_t u = 0;
        memcpy(&u, &x, sizeof u);
        const unsigned char b[4] = {
            static_cast<unsigned char>(u & 0xFFu),
            static_cast<unsigned char>((u >> 8) & 0xFFu),
            static_cast<unsigned char>((u >> 16) & 0xFFu),
            static_cast<unsigned char>((u >> 24) & 0xFFu)};
        if (fwrite(b, 1, 4, f) != 4) { fclose(f); return false; }
    }
    fclose(f);
    return true;
}

std::string num(double x) {
    char buf[64];
    snprintf(buf, sizeof buf, "%.9g", x);
    return buf;
}

// -- calibration fixtures --------------------------------------------------

// The nominal file the harness generates: every servo "Set Middle Position"-ed
// at the CAD-neutral pose, dir from the sim's sign convention.
std::string nominalCalJson() {
    std::string s = "[\n";
    for (int j = 0; j < obs::kNumJoints; ++j) {
        s += "  {\"bus_id\": " + num(obs::kServoId[j]) +
             ", \"zero_steps\": 2048, \"dir\": 1}";
        s += (j + 1 < obs::kNumJoints) ? ",\n" : "\n";
    }
    return s + "]\n";
}

// The perturbed file: random zeros and directions, but always ones under
// which the joint's full MJCF range still fits inside 0..4095 -- otherwise
// angleToSteps' safety clamp (which is correct, and tested in test_obs)
// would swallow the round trip and prove nothing.
std::string perturbedCalJson(Rng& rng) {
    std::string s = "[\n";
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int lo_ticks =
            static_cast<int>(ceilf(fabsf(obs::kJointLo[j]) / kQuantum)) + 1;
        const int hi_ticks =
            static_cast<int>(ceilf(fabsf(obs::kJointHi[j]) / kQuantum)) + 1;
        const int span = (lo_ticks > hi_ticks) ? lo_ticks : hi_ticks;
        const int zero = rng.range(span, 4095 - span);
        const int dir = (rng.next() & 1u) ? 1 : -1;
        s += "  {\"bus_id\": " + num(obs::kServoId[j]) +
             ", \"zero_steps\": " + num(zero) + ", \"dir\": " + num(dir) + "}";
        s += (j + 1 < obs::kNumJoints) ? ",\n" : "\n";
    }
    return s + "]\n";
}

// -- 1. angle -> ticks -> angle -------------------------------------------

void checkRoundTrip(const obs::Calibration& cal, const char* what) {
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const float lo = obs::kJointLo[j], hi = obs::kJointHi[j];
        for (int k = 0; k <= 64; ++k) {
            const float a =
                lo + (hi - lo) * (static_cast<float>(k) / 64.0f);
            const int32_t steps = obs::angleToSteps(j, a, cal);
            // The safety clamp must not be what is being measured.
            CHECK(steps > 0 && steps < 4095);
            const float back = obs::stepsToAngle(j, steps, cal);
            if (fabsf(back - a) > kQuantum) {
                fprintf(stderr, "  (%s joint %d a=%.6f back=%.6f)\n", what, j,
                        static_cast<double>(a), static_cast<double>(back));
            }
            CHECK(fabsf(back - a) <= kQuantum);
        }
    }
    // And the other direction: every reachable tick decodes to an angle that
    // re-encodes to the same tick.
    for (int j = 0; j < obs::kNumJoints; ++j) {
        for (int s = 200; s < 3900; s += 37) {
            const float a = obs::stepsToAngle(j, s, cal);
            if (a < obs::kJointLo[j] || a > obs::kJointHi[j]) continue;
            CHECK_EQ(obs::angleToSteps(j, a, cal), s);
        }
    }
}

void testTickRoundTrip() {
    Rng rng;
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    const std::string per = g_tmp_prefix + "cal_perturbed.json";
    CHECK(writeText(nom, nominalCalJson()));
    CHECK(writeText(per, perturbedCalJson(rng)));

    obs::Calibration cal;
    std::string err;
    CHECK(sil::loadCalibration(nom.c_str(), cal, err));
    for (int j = 0; j < obs::kNumJoints; ++j) {
        CHECK_EQ(cal.zero_steps[j], 2048);
        CHECK_EQ(cal.dir[j], 1);
    }
    checkRoundTrip(cal, "nominal");

    obs::Calibration pcal;
    CHECK(sil::loadCalibration(per.c_str(), pcal, err));
    bool any_flipped = false, any_moved = false;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        if (pcal.dir[j] == -1) any_flipped = true;
        if (pcal.zero_steps[j] != 2048) any_moved = true;
    }
    CHECK(any_flipped);   // the perturbation must actually perturb
    CHECK(any_moved);
    checkRoundTrip(pcal, "perturbed");

    // The loader is the harness's contract too: a partial or nonsense file is
    // a hard failure, never a silent fall back to 2048.
    obs::Calibration guard;
    const std::string bad = g_tmp_prefix + "cal_bad.json";
    CHECK(writeText(bad, "[{\"bus_id\": 1, \"zero_steps\": 2048, \"dir\": 1}]"));
    CHECK(!sil::loadCalibration(bad.c_str(), guard, err));
    CHECK(writeText(bad, "[{\"bus_id\": 42, \"zero_steps\": 2048, \"dir\": 1}]"));
    CHECK(!sil::loadCalibration(bad.c_str(), guard, err));
    CHECK(!sil::loadCalibration("no/such/cal.json", guard, err));
    for (int j = 0; j < obs::kNumJoints; ++j) CHECK_EQ(guard.zero_steps[j], 2048);
}

// -- 2. the kServoId permutation, end to end -------------------------------

int slotOf(int joint) { return static_cast<int>(obs::kServoId[joint]) - 1; }

void testServoPermutationEndToEnd() {
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    CHECK_EQ(sil_init(nullptr, nom.c_str(), 1.5f), 0);

    obs::Calibration cal;
    std::string err;
    CHECK(sil::loadCalibration(nom.c_str(), cal, err));

    // A distinct, decodable position per BUS SLOT, so a mis-permuted read
    // cannot accidentally agree.
    SilSensors in{};
    for (int slot = 0; slot < obs::kNumJoints; ++slot) {
        in.pos_ticks[slot] = static_cast<uint16_t>(2000 + 17 * slot);
        in.vel_ticks[slot] = static_cast<int16_t>(
            scsbus::toSignMag((slot % 2) ? -(3 * slot + 1) : (3 * slot + 1), 15));
    }
    in.up[2] = 1.0f;
    SilTargets out{};
    CHECK_EQ(sil_tick(&in, &out), 0);

    // Sensors: obs joint j must carry the servo whose BUS ID is kServoId[j].
    bool differs_from_identity = false;
    for (int j = 0; j < obs::kNumJoints; ++j) {
        const int32_t want_pos = scsbus::signMag(in.pos_ticks[slotOf(j)], 15);
        const float want_q = obs::stepsToAngle(j, want_pos, cal);
        CHECK_NEAR(out.obs[obs::kOffQ + j], want_q, 1e-6);

        const int32_t want_vel = scsbus::signMag(
            static_cast<uint16_t>(in.vel_ticks[slotOf(j)]), 15);
        CHECK_NEAR(out.obs[obs::kOffDq + j],
                   obs::stepsPerSecToRadPerSec(j, want_vel, cal), 1e-6);

        // What the identity mapping wiring.md claimed would have produced.
        if (slotOf(j) != j) {
            const float identity_q =
                obs::stepsToAngle(j, scsbus::signMag(in.pos_ticks[j], 15), cal);
            if (fabsf(identity_q - want_q) > 1e-6f) differs_from_identity = true;
        }
    }
    CHECK(differs_from_identity);   // the permutation is exercised, not vacuous
    CHECK_EQ(slotOf(0), 9);         // L_hip_yaw is bus ID 10 (assembly errata)
    CHECK_EQ(slotOf(5), 8);         // R_hip_yaw is bus ID 9  (assembly errata)
    CHECK_EQ(slotOf(1), 0);         // L_hip_roll is bus ID 1

    // Targets: the SYNC WRITE view must be indexed the same way.
    float angle[obs::kNumJoints];
    obs::actionToAngles(out.action, angle);
    for (int j = 0; j < obs::kNumJoints; ++j) {
        CHECK_EQ(out.goal_ticks[slotOf(j)], obs::angleToSteps(j, angle[j], cal));
    }
    // Velocity is sign-magnitude, not two's complement. Joint 2 sits on bus
    // ID 2 == slot 1, which was given a NEGATIVE speed: its word is 0x8004,
    // which read as two's complement would be -32764 steps/s (-50 rad/s)
    // rather than -4 steps/s. That is the vendor-UI byte-order bug, so it gets
    // its own assertion rather than riding on the loop above.
    CHECK_EQ(slotOf(2), 1);
    CHECK(out.obs[obs::kOffDq + 2] < 0.0f);
    CHECK(fabsf(out.obs[obs::kOffDq + 2]) < 0.05f);
}

// -- 3. the exported-weight reader vs an independent forward pass ----------

struct SynthNet {
    std::vector<int> sizes;
    std::vector<float> mean, sd;
    std::vector<std::vector<float>> w, b;
};

SynthNet makeSynthNet(Rng& rng) {
    SynthNet n;
    n.sizes = {obs::kObsDim, 12, 2 * obs::kActDim};
    n.mean.resize(static_cast<size_t>(obs::kObsDim));
    n.sd.resize(static_cast<size_t>(obs::kObsDim));
    for (int i = 0; i < obs::kObsDim; ++i) {
        n.mean[static_cast<size_t>(i)] = rng.uniform(-0.5f, 0.5f);
        n.sd[static_cast<size_t>(i)] = rng.uniform(0.5f, 2.0f);
    }
    for (size_t l = 0; l + 1 < n.sizes.size(); ++l) {
        const size_t in = static_cast<size_t>(n.sizes[l]);
        const size_t out = static_cast<size_t>(n.sizes[l + 1]);
        std::vector<float> w(in * out), b(out);
        for (float& x : w) x = rng.uniform(-0.3f, 0.3f);
        for (float& x : b) x = rng.uniform(-0.1f, 0.1f);
        n.w.push_back(w);
        n.b.push_back(b);
    }
    return n;
}

// The reference. Written from docs/sil-harness.md and mlp.h's description of
// brax's semantics, on purpose NOT by calling policy::.
void referenceForward(const SynthNet& n, const float* obs, float* action) {
    std::vector<float> x(static_cast<size_t>(obs::kObsDim));
    for (int i = 0; i < obs::kObsDim; ++i) {
        const size_t u = static_cast<size_t>(i);
        x[u] = (obs[i] - n.mean[u]) / n.sd[u];
    }
    for (size_t l = 0; l + 1 < n.sizes.size(); ++l) {
        const size_t in = static_cast<size_t>(n.sizes[l]);
        const size_t out = static_cast<size_t>(n.sizes[l + 1]);
        std::vector<float> y(out);
        for (size_t j = 0; j < out; ++j) y[j] = n.b[l][j];
        for (size_t i = 0; i < in; ++i) {
            for (size_t j = 0; j < out; ++j) {
                y[j] += x[i] * n.w[l][i * out + j];   // row-major (in, out)
            }
        }
        const bool last = (l + 2 == n.sizes.size());
        if (!last) {
            for (size_t j = 0; j < out; ++j) {
                y[j] = y[j] / (1.0f + expf(-y[j]));   // swish
            }
        }
        x.swap(y);
    }
    for (int i = 0; i < obs::kActDim; ++i) {
        action[i] = tanhf(x[static_cast<size_t>(i)]);
    }
}

std::string sidecarJson(const SynthNet& n) {
    std::string s = "{\n  \"layer_sizes\": [";
    for (size_t i = 0; i < n.sizes.size(); ++i) {
        s += (i ? ", " : "") + num(n.sizes[i]);
    }
    s += "],\n  \"activation\": \"swish\",\n  \"obs_dim\": " +
         num(obs::kObsDim) + ",\n  \"act_dim\": " + num(obs::kActDim) +
         ",\n  \"normalizer\": {\n    \"mean\": [";
    for (size_t i = 0; i < n.mean.size(); ++i) {
        s += (i ? ", " : "") + num(static_cast<double>(n.mean[i]));
    }
    s += "],\n    \"std\": [";
    for (size_t i = 0; i < n.sd.size(); ++i) {
        s += (i ? ", " : "") + num(static_cast<double>(n.sd[i]));
    }
    return s + "]\n  }\n}\n";
}

void testExportedWeights() {
    Rng rng;
    rng.s = 0x9E3779B97F4A7C15ull;
    const SynthNet n = makeSynthNet(rng);

    std::vector<float> blob;
    for (size_t l = 0; l < n.w.size(); ++l) {
        blob.insert(blob.end(), n.w[l].begin(), n.w[l].end());
        blob.insert(blob.end(), n.b[l].begin(), n.b[l].end());
    }
    const std::string wpath = g_tmp_prefix + "net.silw";
    CHECK(writeF32LE(wpath, blob));
    CHECK(writeText(wpath + ".json", sidecarJson(n)));

    sil::LoadedNet loaded;
    std::string err;
    if (!loaded.load(wpath.c_str(), err)) {
        fprintf(stderr, "  load failed: %s\n", err.c_str());
    }
    CHECK(loaded.load(wpath.c_str(), err));
    CHECK_EQ(loaded.obsDim(), obs::kObsDim);
    CHECK_EQ(loaded.actDim(), obs::kActDim);
    CHECK(!loaded.builtin());

    for (int c = 0; c < 5; ++c) {
        std::vector<float> o(static_cast<size_t>(obs::kObsDim));
        for (float& x : o) x = rng.uniform(-3.0f, 3.0f);
        float got[obs::kActDim], want[obs::kActDim];
        loaded.forward(o.data(), got);
        referenceForward(n, o.data(), want);
        for (int i = 0; i < obs::kActDim; ++i) CHECK_NEAR(got[i], want[i], 1e-5);
    }

    // A run exported with the wrong activation must be refused, not run
    // through swish and quietly believed.
    std::string bad = sidecarJson(n);
    const size_t at = bad.find("\"swish\"");
    CHECK(at != std::string::npos);
    bad.replace(at, 7, "\"tanh\"");
    CHECK(writeText(wpath + ".json", bad));
    sil::LoadedNet refused;
    CHECK(!refused.load(wpath.c_str(), err));
    CHECK(err.find("swish") != std::string::npos);
    CHECK(writeText(wpath + ".json", sidecarJson(n)));

    // A truncated blob is a size mismatch, not a buffer overrun.
    const std::string trunc = g_tmp_prefix + "trunc.silw";
    std::vector<float> half(blob.begin(), blob.begin() + 8);
    CHECK(writeF32LE(trunc, half));
    CHECK(writeText(trunc + ".json", sidecarJson(n)));
    CHECK(!refused.load(trunc.c_str(), err));

    // A missing sidecar names the paths it tried.
    CHECK(!refused.load("no/such/net.silw", err));

    // And the whole thing works through the C ABI.
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    CHECK_EQ(sil_init(wpath.c_str(), nom.c_str(), 1.5f), 0);
    SilSensors in{};
    for (int s = 0; s < obs::kNumJoints; ++s) in.pos_ticks[s] = 2048;
    in.up[2] = 1.0f;
    SilTargets out{};
    CHECK_EQ(sil_tick(&in, &out), 0);
    float want[obs::kActDim];
    referenceForward(n, out.obs, want);
    for (int i = 0; i < obs::kActDim; ++i) CHECK_NEAR(out.action[i], want[i], 1e-5);
}

// -- 4. one tick with zeroed sensors --------------------------------------

void testZeroedSmoke() {
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    CHECK_EQ(sil_init(nullptr, nom.c_str(), 1.5f), 0);

    SilSensors in{};        // every channel zero, including pos_ticks
    SilTargets out{};
    CHECK_EQ(sil_tick(&in, &out), 0);

    for (int i = 0; i < obs::kObsDim; ++i) CHECK(isfinite(out.obs[i]));
    for (int i = 0; i < obs::kActDim; ++i) {
        CHECK(isfinite(out.action[i]));
        CHECK(out.action[i] >= -1.0f && out.action[i] <= 1.0f);
    }
    for (int j = 0; j < obs::kNumJoints; ++j) CHECK(out.goal_ticks[j] <= 4095);

    // The documented layout, channel by channel.
    for (int j = 0; j < obs::kNumJoints; ++j) {
        // Tick 0 with a nominal calibration reads as -pi: the servo says 0,
        // the zero is 2048. That is the honest answer, not a bug.
        CHECK_NEAR(out.obs[obs::kOffQ + j], -3.14159265, 1e-3);
        CHECK_NEAR(out.obs[obs::kOffDq + j], 0.0, 0.0);
        CHECK_NEAR(out.obs[obs::kOffPrevAction + j], 0.0, 0.0);
    }
    for (int i = 0; i < 3; ++i) {
        CHECK_NEAR(out.obs[obs::kOffUp + i], 0.0, 0.0);
        CHECK_NEAR(out.obs[obs::kOffLinVel + i], 0.0, 0.0);   // no sensor
        CHECK_NEAR(out.obs[obs::kOffGyro + i], 0.0, 0.0);
    }
    CHECK_NEAR(out.obs[obs::kOffHeight], 0.0, 0.0);           // no sensor
    // ctrl_task advances the clock BEFORE assembling, so the first tick is
    // already one control step into the gait cycle.
    const float phase = kTwoPi * obs::kControlDt * 1.5f;
    CHECK_NEAR(out.obs[obs::kOffPhase + 0], sinf(phase), 1e-6);
    CHECK_NEAR(out.obs[obs::kOffPhase + 1], cosf(phase), 1e-6);
    for (int i = 0; i < obs::kNumCmd; ++i) {
        CHECK_NEAR(out.obs[obs::kOffCmd + i], 0.0, 0.0);
    }
    // First tick after a reset: every history slot is this frame (env.reset()).
    for (int h = 1; h < obs::kHistLen; ++h) {
        for (int i = 0; i < obs::kFrameDim; ++i) {
            CHECK_NEAR(out.obs[h * obs::kFrameDim + i], out.obs[i], 0.0);
        }
    }
}

// -- 5. tick sequencing vs ctrl_task --------------------------------------

void testSequencingAndReset() {
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    CHECK_EQ(sil_init(nullptr, nom.c_str(), 1.5f), 0);

    SilSensors in{};
    for (int s = 0; s < obs::kNumJoints; ++s) {
        in.pos_ticks[s] = static_cast<uint16_t>(2048 + 5 * s);
    }
    in.up[2] = 1.0f;
    in.cmd[3] = 1.0f;

    SilTargets t1{}, t2{};
    CHECK_EQ(sil_tick(&in, &t1), 0);
    CHECK_EQ(sil_tick(&in, &t2), 0);

    // prev_action at tick 2 is tick 1's action; history slot 1 is tick 1's
    // frame. Both would still "look fine" if the push happened before the
    // policy ran, which is why they are checked explicitly.
    for (int j = 0; j < obs::kActDim; ++j) {
        CHECK_NEAR(t2.obs[obs::kOffPrevAction + j], t1.action[j], 0.0);
    }
    for (int i = 0; i < obs::kFrameDim; ++i) {
        CHECK_NEAR(t2.obs[obs::kFrameDim + i], t1.obs[i], 0.0);
    }
    // The clock keeps advancing.
    const float p1 = kTwoPi * obs::kControlDt * 1.5f;
    CHECK_NEAR(t2.obs[obs::kOffPhase + 0], sinf(2.0f * p1), 1e-6);

    // Reset must make tick 1 reproducible bit for bit.
    for (int k = 0; k < 5; ++k) CHECK_EQ(sil_tick(&in, &t2), 0);
    sil_reset();
    SilTargets again{};
    CHECK_EQ(sil_tick(&in, &again), 0);
    for (int i = 0; i < obs::kObsDim; ++i) CHECK_NEAR(again.obs[i], t1.obs[i], 0.0);
    for (int i = 0; i < obs::kActDim; ++i) {
        CHECK_NEAR(again.action[i], t1.action[i], 0.0);
    }
    for (int j = 0; j < obs::kNumJoints; ++j) {
        CHECK_EQ(again.goal_ticks[j], t1.goal_ticks[j]);
    }

    // A different gait frequency is honoured.
    CHECK_EQ(sil_init(nullptr, nom.c_str(), 1.25f), 0);
    SilTargets slow{};
    CHECK_EQ(sil_tick(&in, &slow), 0);
    CHECK_NEAR(slow.obs[obs::kOffPhase + 0],
               sinf(kTwoPi * obs::kControlDt * 1.25f), 1e-6);
}

// -- 6. sil_spec ----------------------------------------------------------

void testSpec() {
    const std::string nom = g_tmp_prefix + "cal_nominal.json";
    CHECK_EQ(sil_init(nullptr, nom.c_str(), 1.5f), 0);

    siljson::Value v;
    std::string err;
    if (!siljson::parse(sil_spec(), v, err)) {
        fprintf(stderr, "  sil_spec is not valid JSON: %s\n  %s\n", err.c_str(),
                sil_spec());
    }
    CHECK(siljson::parse(sil_spec(), v, err));
    CHECK(v.isObj());

    const siljson::Value* d = v.get("obs_dim");
    CHECK(d && d->isNum() && static_cast<int>(d->num) == obs::kObsDim);
    d = v.get("num_joints");
    CHECK(d && static_cast<int>(d->num) == obs::kNumJoints);
    d = v.get("act_dim");
    CHECK(d && static_cast<int>(d->num) == obs::kActDim);
    d = v.get("num_cmd");
    CHECK(d && static_cast<int>(d->num) == obs::kNumCmd);
    d = v.get("hist_len");
    CHECK(d && static_cast<int>(d->num) == obs::kHistLen);
    d = v.get("frame_dim");
    CHECK(d && static_cast<int>(d->num) == obs::kFrameDim);

    // The permutation and its indexing convention are published, not implied.
    d = v.get("servo_id");
    std::vector<int> ids;
    CHECK(d && d->asInts(ids) &&
          ids.size() == static_cast<size_t>(obs::kNumJoints));
    for (int j = 0; j < obs::kNumJoints; ++j) {
        CHECK_EQ(ids[static_cast<size_t>(j)], obs::kServoId[j]);
    }
    d = v.get("sensor_index");
    CHECK(d && d->isStr() && d->str == "bus_id-1");

    d = v.get("spec_hash");
    CHECK(d && d->isStr() && d->str.size() == 18);   // 0x + 16 hex digits
    d = v.get("ready");
    CHECK(d && d->type == siljson::Value::kBool && d->b);

    const siljson::Value* p = v.get("policy");
    CHECK(p && p->isObj());
    d = p->get("activation");
    CHECK(d && d->str == "swish");
    d = p->get("builtin");
    CHECK(d && d->b);      // sil_init(nullptr, ...) uses the compiled weights

    // Callable before init, too -- the harness reads it to decide what to send.
    CHECK(sil_spec() != nullptr);
    CHECK(strlen(sil_spec()) > 100);
}

// -- 7. error paths -------------------------------------------------------

void testErrors() {
    CHECK(sil_init("no/such/weights.silw", nullptr, 1.5f) != 0);
    CHECK(strlen(sil_last_error()) > 0);
    // A failed init leaves the library unusable rather than half-configured.
    SilSensors in{};
    SilTargets out{};
    CHECK(sil_tick(&in, &out) != 0);
    CHECK(sil_init(nullptr, "no/such/cal.json", 1.5f) != 0);

    // No weights and no calibration is the documented "use the firmware's
    // own" path, and it must work.
    CHECK_EQ(sil_init(nullptr, nullptr, 0.0f), 0);
    CHECK_EQ(sil_tick(&in, &out), 0);
    CHECK(sil_tick(nullptr, &out) != 0);
    CHECK(sil_tick(&in, nullptr) != 0);
}

void cleanup() {
    const char* names[] = {"cal_nominal.json", "cal_perturbed.json",
                           "cal_bad.json",     "net.silw",
                           "net.silw.json",    "trunc.silw",
                           "trunc.silw.json"};
    for (const char* n : names) remove((g_tmp_prefix + n).c_str());
}

}  // namespace

int main(int argc, char** argv) {
    // ctest runs from the build directory, make from firmware/host; either
    // way the fixtures land beside the binary's working directory.
    if (argc > 1) g_tmp_prefix = std::string(argv[1]) + "/" + g_tmp_prefix;

    testTickRoundTrip();
    testServoPermutationEndToEnd();
    testExportedWeights();
    testZeroedSmoke();
    testSequencingAndReset();
    testSpec();
    testErrors();
    cleanup();
    return testutil::report("sil");
}

// The firmware against the sim's own numbers (docs/sil-harness.md pyramid
// item 1: "obs frame vs ... NEW vectors generated from walker_env states;
// policy forward vs brax golden pairs, atol 1e-5").
//
// Agent B generates sim/sil/golden/*.json from the same run the exporter
// reads. Until those files exist this test SKIPS -- loudly, naming the
// directories it looked in -- rather than failing the gate for work that has
// not landed. A skip is a green ctest; a mismatch is a red one.
//
// Two kinds of file are recognised, by the shape of their cases:
//
//   obs      {"offsets": {...}, "cases": [{"inputs": {q, dq, up, gyro,
//             prev_action, phase, cmd}, "frame": [frame_dim]}]}
//            -> obs::assembleFrame must reproduce `frame` exactly.
//   policy   {"run": "...", "atol": 1e-5,
//             "cases": [{"obs": [obs_dim], "obs_norm": [obs_dim],
//                        "action": [act_dim]}]}
//            -> the exported .silw must reproduce `action`, and its
//               normalizer must reproduce `obs_norm`.
//
// The .silw is found from the golden file's "weights" key if it has one, else
// from "run" in a sibling weights/ directory, else as the only *.silw beside
// the vectors.
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "obs/assembler.h"
#include "obs/obs_spec.h"
#include "sil_json.h"
#include "sil_net.h"
#include "test_util.h"

namespace {

std::string join(const std::string& dir, const std::string& name) {
    return dir.empty() ? name : dir + "/" + name;
}

bool listFiles(const std::string& dir, const char* suffix,
               std::vector<std::string>& out) {
    DIR* d = opendir(dir.c_str());
    if (!d) return false;
    const size_t slen = strlen(suffix);
    while (struct dirent* e = readdir(d)) {
        const std::string name = e->d_name;
        if (name.size() > slen &&
            name.compare(name.size() - slen, slen, suffix) == 0) {
            out.push_back(join(dir, name));
        }
    }
    closedir(d);
    return true;
}

// sim/sil/golden is relative to the repo root; the binary runs from
// firmware/host (make) or firmware/host/build (ctest).
std::string findGoldenDir() {
    if (const char* env = getenv("SIL_GOLDEN_DIR")) {
        if (*env) return env;
    }
    const char* kCandidates[] = {
        "sim/sil/golden",       "../sim/sil/golden",
        "../../sim/sil/golden", "../../../sim/sil/golden",
        "../../../../sim/sil/golden"};
    for (const char* c : kCandidates) {
        DIR* d = opendir(c);
        if (d) {
            closedir(d);
            return c;
        }
    }
    return std::string();
}

std::string dirOf(const std::string& p) {
    const size_t slash = p.find_last_of('/');
    return slash == std::string::npos ? std::string(".") : p.substr(0, slash);
}

std::string resolveWeights(const std::string& json_path,
                           const siljson::Value& root) {
    const std::string dir = dirOf(json_path);
    // Directories a .silw plausibly lives in, relative to the vectors.
    const std::string dirs[] = {dir, join(dir, "weights"),
                                join(dir, "../weights"),
                                join(dir, "../../weights")};

    static const char* kKeys[] = {"weights", "weights_file", "silw", "blob"};
    if (root.isObj()) {
        if (const siljson::Value* w = root.getAny(kKeys, 4)) {
            if (w->isStr() && !w->str.empty()) {
                if (w->str[0] == '/') return w->str;
                for (const std::string& d : dirs) {
                    const std::string cand = join(d, w->str);
                    if (siljson::fileExists(cand.c_str())) return cand;
                }
                for (const char* up : {"", "../", "../../", "../../../"}) {
                    const std::string cand = std::string(up) + w->str;
                    if (siljson::fileExists(cand.c_str())) return cand;
                }
            }
        }
        // No explicit pointer: the exporter names its output after the run.
        if (const siljson::Value* r = root.get("run")) {
            if (r->isStr() && !r->str.empty()) {
                for (const std::string& d : dirs) {
                    const std::string cand = join(d, r->str + ".silw");
                    if (siljson::fileExists(cand.c_str())) return cand;
                }
            }
        }
    }
    // Last resort: exactly one .silw nearby is unambiguous, more than one
    // is not.
    for (const std::string& d : dirs) {
        std::vector<std::string> silw;
        listFiles(d, ".silw", silw);
        if (silw.size() == 1) return silw[0];
    }
    return std::string();
}

const siljson::Value* casesOf(const siljson::Value& root) {
    if (root.isArr()) return &root;
    static const char* kKeys[] = {"cases", "vectors", "pairs", "samples"};
    return root.getAny(kKeys, 4);
}

bool floatsOf(const siljson::Value& obj, const char* const* names, size_t n,
              std::vector<float>& out) {
    const siljson::Value* v = obj.getAny(names, n);
    return v && v->asFloats(out);
}

int g_files = 0, g_obs_cases = 0, g_pol_cases = 0, g_skipped = 0;
double g_worst_action = 0.0, g_worst_norm = 0.0, g_worst_frame = 0.0;

void note(double& worst, float got, float want) {
    const double d = fabs(static_cast<double>(got) - static_cast<double>(want));
    if (d > worst) worst = d;
}

// -- obs vectors -----------------------------------------------------------

void runObsFile(const std::string& path, const siljson::Value& root,
                const siljson::Value& cases) {
    // Layout drift check first: if the generator's offsets and this build's
    // obs_spec.h disagree, every frame below would be wrong for one reason
    // and the diff would be unreadable.
    if (const siljson::Value* off = root.get("offsets")) {
        struct { const char* key; int want; } kOff[] = {
            {"q", obs::kOffQ},           {"dq", obs::kOffDq},
            {"up", obs::kOffUp},         {"linvel", obs::kOffLinVel},
            {"gyro", obs::kOffGyro},     {"prev", obs::kOffPrevAction},
            {"height", obs::kOffHeight}, {"phase", obs::kOffPhase},
            {"cmd", obs::kOffCmd}};
        for (const auto& o : kOff) {
            if (const siljson::Value* v = off->get(o.key)) {
                CHECK_EQ(static_cast<int>(v->num), o.want);
            }
        }
    }
    if (const siljson::Value* v = root.get("frame_dim")) {
        CHECK_EQ(static_cast<int>(v->num), obs::kFrameDim);
    }
    if (const siljson::Value* v = root.get("obs_dim")) {
        CHECK_EQ(static_cast<int>(v->num), obs::kObsDim);
    }
    if (const siljson::Value* v = root.get("hist_len")) {
        CHECK_EQ(static_cast<int>(v->num), obs::kHistLen);
    }

    ++g_files;
    printf("  %s: %zu obs cases\n", path.c_str(), cases.arr.size());

    for (const siljson::Value& c : cases.arr) {
        const siljson::Value* ins = c.get("inputs");
        static const char* kFrameKeys[] = {"frame", "expected_frame"};
        std::vector<float> want;
        if (!ins || !ins->isObj() || !floatsOf(c, kFrameKeys, 2, want)) {
            fprintf(stderr, "FAIL %s: case is not {inputs, frame}\n",
                    path.c_str());
            ++testutil::g_fails;
            return;
        }
        CHECK_EQ(want.size(), static_cast<size_t>(obs::kFrameDim));
        if (want.size() != static_cast<size_t>(obs::kFrameDim)) return;

        std::vector<float> q, dq, up, gyro, prev, cmd;
        static const char* kQ[] = {"q"};
        static const char* kDq[] = {"dq"};
        static const char* kUp[] = {"up"};
        static const char* kGyro[] = {"gyro"};
        static const char* kPrev[] = {"prev_action", "prev"};
        static const char* kCmd[] = {"cmd"};
        const bool ok = floatsOf(*ins, kQ, 1, q) && floatsOf(*ins, kDq, 1, dq) &&
                        floatsOf(*ins, kUp, 1, up) &&
                        floatsOf(*ins, kGyro, 1, gyro) &&
                        floatsOf(*ins, kPrev, 2, prev) &&
                        floatsOf(*ins, kCmd, 1, cmd);
        const siljson::Value* ph = ins->get("phase");
        if (!ok || !ph || !ph->isNum() ||
            q.size() != static_cast<size_t>(obs::kNumJoints) ||
            dq.size() != static_cast<size_t>(obs::kNumJoints) ||
            prev.size() != static_cast<size_t>(obs::kActDim) ||
            up.size() != 3 || gyro.size() != 3 ||
            cmd.size() != static_cast<size_t>(obs::kNumCmd)) {
            fprintf(stderr, "FAIL %s: inputs have the wrong widths\n",
                    path.c_str());
            ++testutil::g_fails;
            return;
        }

        obs::Inputs in{};
        for (int i = 0; i < obs::kNumJoints; ++i) {
            in.q[i] = q[static_cast<size_t>(i)];
            in.dq[i] = dq[static_cast<size_t>(i)];
        }
        for (int i = 0; i < obs::kActDim; ++i) {
            in.prev_action[i] = prev[static_cast<size_t>(i)];
        }
        for (int i = 0; i < 3; ++i) {
            in.up[i] = up[static_cast<size_t>(i)];
            in.gyro[i] = gyro[static_cast<size_t>(i)];
        }
        for (int i = 0; i < obs::kNumCmd; ++i) {
            in.cmd[i] = cmd[static_cast<size_t>(i)];
        }
        in.phase = static_cast<float>(ph->num);

        float frame[obs::kFrameDim];
        obs::assembleFrame(in, frame);
        for (int i = 0; i < obs::kFrameDim; ++i) {
            note(g_worst_frame, frame[i], want[static_cast<size_t>(i)]);
            CHECK_NEAR(frame[i], want[static_cast<size_t>(i)], 1e-6);
        }
        ++g_obs_cases;
    }
}

// -- policy vectors --------------------------------------------------------

void runPolicyFile(const std::string& path, const siljson::Value& root,
                   const siljson::Value& cases) {
    const std::string wpath = resolveWeights(path, root);
    if (wpath.empty()) {
        printf("  skip %s: no .silw named or found beside it\n", path.c_str());
        ++g_skipped;
        return;
    }
    sil::LoadedNet net;
    std::string err;
    if (!net.load(wpath.c_str(), err)) {
        // Once a golden file names weights, an unreadable blob IS the
        // exporter/reader disagreement this test exists to catch.
        fprintf(stderr, "FAIL %s: %s\n", path.c_str(), err.c_str());
        ++testutil::g_fails;
        return;
    }
    double atol = 1e-5;
    if (const siljson::Value* t = root.get("atol")) {
        if (t->isNum()) atol = t->num;
    }
    if (const siljson::Value* a = root.get("activation")) {
        if (a->isStr()) CHECK(a->str == net.activation());
    }

    ++g_files;
    printf("  %s: %zu policy cases vs %s (atol %g)\n", path.c_str(),
           cases.arr.size(), wpath.c_str(), atol);

    CHECK_EQ(net.obsDim(), obs::kObsDim);
    CHECK_EQ(net.actDim(), obs::kActDim);
    if (net.obsDim() != obs::kObsDim || net.actDim() != obs::kActDim) return;

    for (const siljson::Value& c : cases.arr) {
        static const char* kObsKeys[] = {"obs", "observation"};
        static const char* kActKeys[] = {"action", "act", "expected_action"};
        static const char* kNormKeys[] = {"obs_norm", "normalized_obs"};
        std::vector<float> o, want, want_norm;
        if (!floatsOf(c, kObsKeys, 2, o) || !floatsOf(c, kActKeys, 3, want)) {
            fprintf(stderr, "FAIL %s: case is not {obs, action}\n",
                    path.c_str());
            ++testutil::g_fails;
            return;
        }
        CHECK_EQ(o.size(), static_cast<size_t>(obs::kObsDim));
        CHECK_EQ(want.size(), static_cast<size_t>(obs::kActDim));
        if (o.size() != static_cast<size_t>(obs::kObsDim) ||
            want.size() != static_cast<size_t>(obs::kActDim)) {
            return;
        }

        // The normalizer separately, so a golden mismatch says which half is
        // wrong. The tolerance carries a representation term: the generator
        // picked obs_norm first and wrote obs = mean + std*obs_norm as
        // float32, so recovering obs_norm costs half an ULP of obs (and of
        // mean) divided by std. On a near-constant channel -- this run has
        // several with std = 1e-6 -- that is ~5% and it is arithmetic, not
        // divergence. Every channel with an ordinary std is still held to
        // atol.
        if (floatsOf(c, kNormKeys, 2, want_norm) &&
            want_norm.size() == o.size()) {
            for (int i = 0; i < obs::kObsDim; ++i) {
                const size_t u = static_cast<size_t>(i);
                const float mean = net.net().norm_mean[i];
                const float sd = net.net().norm_std[i];
                const float got = (o[u] - mean) / sd;
                const double kHalfUlp = 5.9604645e-8;   // 2^-24
                const double tol =
                    atol + 2.0 *
                               (fabs(static_cast<double>(o[u])) +
                                fabs(static_cast<double>(mean))) *
                               kHalfUlp / static_cast<double>(sd);
                note(g_worst_norm, got, want_norm[u]);
                CHECK_NEAR(got, want_norm[u], tol);
            }
        }

        float got[obs::kActDim];
        net.forward(o.data(), got);
        for (int i = 0; i < obs::kActDim; ++i) {
            note(g_worst_action, got[i], want[static_cast<size_t>(i)]);
            CHECK_NEAR(got[i], want[static_cast<size_t>(i)], atol);
        }
        ++g_pol_cases;
    }
}

void runFile(const std::string& path) {
    siljson::Value root;
    std::string err;
    if (!siljson::parseFile(path.c_str(), root, err)) {
        fprintf(stderr, "FAIL golden %s: %s\n", path.c_str(), err.c_str());
        ++testutil::g_fails;
        return;
    }
    const siljson::Value* cases = casesOf(root);
    if (!cases || !cases->isArr() || cases->arr.empty()) {
        printf("  skip %s: no case array\n", path.c_str());
        ++g_skipped;
        return;
    }
    const siljson::Value& first = cases->arr[0];
    if (first.get("frame") || first.get("inputs")) {
        runObsFile(path, root, *cases);
    } else if (first.get("obs") || first.get("action")) {
        runPolicyFile(path, root, *cases);
    } else {
        printf("  skip %s: cases are neither obs nor policy pairs\n",
               path.c_str());
        ++g_skipped;
    }
}

}  // namespace

int main() {
    const std::string dir = findGoldenDir();
    if (dir.empty()) {
        printf("sil_golden       SKIP: no sim/sil/golden (Agent B's vectors "
               "have not landed); looked upwards from the working directory "
               "and at $SIL_GOLDEN_DIR\n");
        return 0;
    }
    std::vector<std::string> files;
    listFiles(dir, ".json", files);
    if (files.empty()) {
        printf("sil_golden       SKIP: %s has no *.json vectors yet\n",
               dir.c_str());
        return 0;
    }
    for (const std::string& f : files) runFile(f);
    if (g_files == 0) {
        printf("sil_golden       SKIP: %d file(s) in %s, none usable yet\n",
               g_skipped, dir.c_str());
        return 0;
    }
    printf("  worst |err|: frame %.3g, obs_norm %.3g, action %.3g\n",
           g_worst_frame, g_worst_norm, g_worst_action);
    printf("sil_golden       %d file(s), %d obs case(s), %d policy case(s)\n",
           g_files, g_obs_cases, g_pol_cases);
    return testutil::report("sil_golden");
}

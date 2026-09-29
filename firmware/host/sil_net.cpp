#include "sil_net.h"

#include <stdio.h>
#include <string.h>

#include "obs/obs_spec.h"
#include "policy/weights.h"
#include "sil_json.h"

namespace sil {
namespace {

float readF32LE(const unsigned char* p) {
    const uint32_t u = static_cast<uint32_t>(p[0]) |
                       (static_cast<uint32_t>(p[1]) << 8) |
                       (static_cast<uint32_t>(p[2]) << 16) |
                       (static_cast<uint32_t>(p[3]) << 24);
    float f = 0.0f;
    memcpy(&f, &u, sizeof f);
    return f;
}

uint32_t readU32LE(const unsigned char* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

bool readBytes(const char* path, std::vector<unsigned char>& out,
               std::string& err) {
    FILE* f = fopen(path, "rb");
    if (!f) {
        err = std::string("cannot open weight blob ") + path;
        return false;
    }
    out.clear();
    unsigned char buf[8192];
    size_t got = 0;
    while ((got = fread(buf, 1, sizeof buf, f)) > 0) {
        out.insert(out.end(), buf, buf + got);
    }
    const bool bad = ferror(f) != 0;
    fclose(f);
    if (bad) {
        err = std::string("read error on ") + path;
        return false;
    }
    return true;
}

// The optional 64-byte binary header. Present == the file starts with "SILW".
struct BlobHeader {
    bool present = false;
    uint32_t version = 0;
    size_t header_bytes = 0;
    int obs_dim = 0;
    int act_dim = 0;
    int num_layers = 0;
    std::string activation;
    std::vector<int> sizes;
};

bool parseHeader(const std::vector<unsigned char>& b, const std::string& path,
                 BlobHeader& h, std::string& err) {
    if (b.size() < 4 || memcmp(b.data(), "SILW", 4) != 0) return true;  // none
    if (b.size() < 44) {
        err = path + ": SILW header is truncated";
        return false;
    }
    h.present = true;
    h.version = readU32LE(&b[4]);
    h.header_bytes = readU32LE(&b[8]);
    h.obs_dim = static_cast<int>(readU32LE(&b[12]));
    h.act_dim = static_cast<int>(readU32LE(&b[16]));
    h.num_layers = static_cast<int>(readU32LE(&b[20]));
    const uint32_t num_sizes = readU32LE(&b[24]);
    if (h.version != 1u) {
        char msg[96];
        snprintf(msg, sizeof msg, ": SILW version %u, this build reads 1",
                 h.version);
        err = path + msg;
        return false;
    }
    // activation: a NUL-padded char[16] at offset 28.
    const size_t act_off = 28, act_len = 16;
    if (b.size() < act_off + act_len + 4u * num_sizes) {
        err = path + ": SILW header is shorter than its own field list";
        return false;
    }
    for (size_t i = 0; i < act_len && b[act_off + i]; ++i) {
        h.activation.push_back(static_cast<char>(b[act_off + i]));
    }
    const size_t sizes_off = act_off + act_len;
    if (num_sizes < 2u || num_sizes > 64u ||
        static_cast<int>(num_sizes) != h.num_layers + 1) {
        err = path + ": SILW num_sizes disagrees with num_layers";
        return false;
    }
    for (uint32_t i = 0; i < num_sizes; ++i) {
        h.sizes.push_back(
            static_cast<int>(readU32LE(&b[sizes_off + 4u * i])));
    }
    if (h.header_bytes < sizes_off + 4u * num_sizes ||
        h.header_bytes > b.size() || h.header_bytes % 4u != 0u) {
        err = path + ": SILW header_bytes is out of range";
        return false;
    }
    return true;
}

bool close(float a, float b) {
    const float d = a > b ? a - b : b - a;
    const float mag = (a < 0.0f ? -a : a) + 1.0f;
    return d <= 1e-4f * mag;
}

std::string stripExt(const std::string& p) {
    const size_t dot = p.find_last_of('.');
    const size_t slash = p.find_last_of("/\\");
    if (dot == std::string::npos) return p;
    if (slash != std::string::npos && dot < slash) return p;
    return p.substr(0, dot);
}

bool endsWith(const std::string& s, const char* suffix) {
    const size_t n = strlen(suffix);
    return s.size() >= n && s.compare(s.size() - n, n, suffix) == 0;
}

std::string dirOf(const std::string& p) {
    const size_t slash = p.find_last_of("/\\");
    return slash == std::string::npos ? std::string(".") : p.substr(0, slash);
}

// Resolve blob + sidecar from whichever of the two the caller handed us.
bool resolvePaths(const std::string& given, std::string& blob,
                  std::string& sidecar, std::string& err) {
    if (endsWith(given, ".json")) {
        sidecar = given;
        siljson::Value v;
        std::string perr;
        if (siljson::parseFile(sidecar.c_str(), v, perr)) {
            static const char* kKeys[] = {"blob", "weights", "weights_file"};
            if (const siljson::Value* w = v.getAny(kKeys, 3)) {
                if (w->isStr()) {
                    blob = w->str.empty() || w->str[0] == '/'
                               ? w->str
                               : dirOf(sidecar) + "/" + w->str;
                    if (siljson::fileExists(blob.c_str())) return true;
                }
            }
        }
        const std::string base = stripExt(given);
        const char* kExts[] = {"", ".silw", ".bin", ".f32"};
        for (const char* e : kExts) {
            const std::string cand = base + e;
            if (cand != given && siljson::fileExists(cand.c_str())) {
                blob = cand;
                return true;
            }
        }
        err = "no weight blob found beside the sidecar " + given;
        return false;
    }

    blob = given;
    if (!siljson::fileExists(blob.c_str())) {
        err = "no such weight file: " + blob;
        return false;
    }
    // A missing sidecar is only fatal for a headerless blob, so it is not an
    // error here -- `sidecar` just stays empty.
    const std::string cands[] = {given + ".json", stripExt(given) + ".json",
                                 given + ".sidecar.json"};
    for (const std::string& c : cands) {
        if (siljson::fileExists(c.c_str())) {
            sidecar = c;
            break;
        }
    }
    return true;
}

}  // namespace

void LoadedNet::bind() {
    net_.obs_dim = sizes_.empty() ? 0 : sizes_.front();
    net_.num_layers = static_cast<int>(layers_.size());
    net_.layers = layers_.empty() ? nullptr : layers_.data();
    net_.norm_mean = mean_.empty() ? nullptr : mean_.data();
    net_.norm_std = std_.empty() ? nullptr : std_.data();

    size_t width = static_cast<size_t>(net_.obs_dim);
    for (const policy::Dense& l : layers_) {
        if (l.out > width) width = l.out;
    }
    scratch_a_.assign(width, 0.0f);
    scratch_b_.assign(width, 0.0f);
    net_.scratch_a = scratch_a_.data();
    net_.scratch_b = scratch_b_.data();
}

void LoadedNet::loadBuiltin() {
    builtin_ = true;
    source_ = "policy/weights.h (generated)";
    activation_ = "swish";
    sizes_.clear();
    sizes_.push_back(policy::kLayers[0].in);
    layers_.clear();
    for (int i = 0; i < policy::kNumLayers; ++i) {
        layers_.push_back(policy::kLayers[i]);
        sizes_.push_back(policy::kLayers[i].out);
    }
    mean_.assign(policy::kNormMean, policy::kNormMean + policy::kObsDim);
    std_.assign(policy::kNormStd, policy::kNormStd + policy::kObsDim);
    net_.act_dim = policy::kActDim;
    bind();
}

bool LoadedNet::load(const char* weights_path, std::string& err) {
    if (!weights_path || !*weights_path) {
        loadBuiltin();
        return true;
    }
    std::string blob_path, sidecar_path;
    if (!resolvePaths(weights_path, blob_path, sidecar_path, err)) return false;

    std::vector<unsigned char> bytes;
    if (!readBytes(blob_path.c_str(), bytes, err)) return false;
    BlobHeader head;
    if (!parseHeader(bytes, blob_path, head, err)) return false;

    // -- the sidecar, when there is one ------------------------------------
    siljson::Value side;
    bool have_side = false;
    if (!sidecar_path.empty()) {
        if (!siljson::parseFile(sidecar_path.c_str(), side, err)) return false;
        if (!side.isObj()) {
            err = sidecar_path + ": sidecar must be a JSON object";
            return false;
        }
        have_side = true;
    } else if (!head.present) {
        err = "no JSON sidecar for " + blob_path + " and no SILW header in it "
              "(tried " + blob_path + ".json and " + stripExt(blob_path) +
              ".json)";
        return false;
    }

    // -- shape: the header wins, the sidecar must agree --------------------
    std::vector<int> side_sizes;
    if (have_side) {
        static const char* kSizeKeys[] = {"layer_sizes", "layers", "sizes"};
        const siljson::Value* sizes_v = side.getAny(kSizeKeys, 3);
        if (!sizes_v || !sizes_v->asInts(side_sizes) || side_sizes.size() < 2) {
            err = sidecar_path + ": layer_sizes must be an array of >= 2 ints";
            return false;
        }
    }
    sizes_ = head.present ? head.sizes : side_sizes;
    if (head.present && have_side && side_sizes != sizes_) {
        err = sidecar_path + ": layer_sizes disagree with the SILW header in " +
              blob_path;
        return false;
    }
    for (int s : sizes_) {
        if (s <= 0 || s > 65535) {
            err = blob_path + ": layer size out of range";
            return false;
        }
    }

    activation_ = head.present ? head.activation : std::string();
    if (have_side) {
        if (const siljson::Value* a = side.get("activation")) {
            if (!a->isStr()) {
                err = sidecar_path + ": activation must be a string";
                return false;
            }
            if (!activation_.empty() && activation_ != a->str) {
                err = sidecar_path + ": activation \"" + a->str +
                      "\" disagrees with the SILW header's \"" + activation_ +
                      "\"";
                return false;
            }
            activation_ = a->str;
        }
    }
    if (activation_.empty()) activation_ = "swish";
    // The firmware net has exactly one hidden activation compiled in. A run
    // exported with anything else would be silently mis-evaluated here, which
    // is the exact deploy bug docs/sil-harness.md names, so refuse it.
    if (activation_ != "swish") {
        err = blob_path + ": activation \"" + activation_ +
              "\" but this build implements swish only";
        return false;
    }

    int obs_dim = sizes_.front();
    int act_dim = sizes_.back() / 2;
    if (head.present) {
        obs_dim = head.obs_dim;
        act_dim = head.act_dim;
    }
    if (have_side) {
        if (const siljson::Value* v = side.get("obs_dim")) {
            if (!v->isNum() || static_cast<int>(v->num) != obs_dim) {
                err = sidecar_path + ": obs_dim disagrees with the blob";
                return false;
            }
        }
        if (const siljson::Value* v = side.get("act_dim")) {
            if (!v->isNum() || static_cast<int>(v->num) != act_dim) {
                err = sidecar_path + ": act_dim disagrees with the blob";
                return false;
            }
        }
    }
    if (obs_dim != sizes_.front()) {
        err = blob_path + ": obs_dim disagrees with layer_sizes[0]";
        return false;
    }
    // brax's NormalTanhDistribution head is 2 * action_size wide.
    if (sizes_.back() != 2 * act_dim) {
        err = blob_path + ": output layer is not 2 * act_dim";
        return false;
    }

    // -- the float payload -------------------------------------------------
    const size_t off_bytes = head.present ? head.header_bytes : 0u;
    if ((bytes.size() - off_bytes) % 4u != 0u) {
        err = blob_path + ": payload length is not a multiple of 4 bytes";
        return false;
    }
    std::vector<float> blob((bytes.size() - off_bytes) / 4u);
    for (size_t i = 0; i < blob.size(); ++i) {
        blob[i] = readF32LE(&bytes[off_bytes + i * 4u]);
    }
    if (have_side) {
        if (const siljson::Value* b = side.get("blob")) {
            if (const siljson::Value* n = b->get("n_floats")) {
                if (n->isNum() &&
                    static_cast<size_t>(n->num) != blob.size()) {
                    char msg[128];
                    snprintf(msg, sizeof msg,
                             ": sidecar says %zu floats, the file holds %zu",
                             static_cast<size_t>(n->num), blob.size());
                    err = blob_path + msg;
                    return false;
                }
            }
        }
    }

    size_t need = 0;
    for (size_t i = 0; i + 1 < sizes_.size(); ++i) {
        need += static_cast<size_t>(sizes_[i]) * static_cast<size_t>(sizes_[i + 1]);
        need += static_cast<size_t>(sizes_[i + 1]);
    }

    // -- normalizer ---------------------------------------------------------
    // It may lead the float payload (what the exporter writes) or ride in the
    // sidecar (the plainest reading of the design doc). Both are accepted;
    // when both are present they must agree, because a normalizer mismatch is
    // exactly the "firmware net consumes NORMALIZED obs" failure this file is
    // here to prevent.
    std::vector<float> side_mean, side_std;
    if (have_side) {
        if (const siljson::Value* n = side.get("normalizer")) {
            static const char* kMeanKeys[] = {"mean", "means"};
            static const char* kStdKeys[] = {"std", "stddev"};
            if (const siljson::Value* m = n->getAny(kMeanKeys, 2)) {
                m->asFloats(side_mean);
            }
            if (const siljson::Value* s = n->getAny(kStdKeys, 2)) {
                s->asFloats(side_std);
            }
        }
    }

    const size_t norm_floats = 2u * static_cast<size_t>(obs_dim);
    size_t off = 0;
    mean_.clear();
    std_.clear();
    if (blob.size() == need + norm_floats) {
        mean_.assign(blob.begin(), blob.begin() + static_cast<long>(obs_dim));
        std_.assign(blob.begin() + static_cast<long>(obs_dim),
                    blob.begin() + static_cast<long>(norm_floats));
        off = norm_floats;
        if (side_mean.size() == mean_.size() && side_std.size() == std_.size()) {
            for (size_t i = 0; i < mean_.size(); ++i) {
                if (!close(mean_[i], side_mean[i]) ||
                    !close(std_[i], side_std[i])) {
                    char msg[160];
                    snprintf(msg, sizeof msg,
                             ": normalizer channel %zu differs between the "
                             "blob and the sidecar",
                             i);
                    err = blob_path + msg;
                    return false;
                }
            }
        }
    } else if (blob.size() == need) {
        mean_ = side_mean;
        std_ = side_std;
    } else {
        char msg[192];
        snprintf(msg, sizeof msg,
                 ": payload has %zu floats; layer_sizes need %zu (%zu with a "
                 "leading normalizer)",
                 blob.size(), need, need + norm_floats);
        err = blob_path + msg;
        return false;
    }

    if (mean_.size() != static_cast<size_t>(obs_dim) ||
        std_.size() != static_cast<size_t>(obs_dim)) {
        err = blob_path +
              ": normalizer mean/std must each be obs_dim floats (the "
              "firmware net consumes NORMALIZED obs)";
        return false;
    }
    for (float s : std_) {
        if (!(s > 0.0f)) {
            err = blob_path + ": normalizer std has a non-positive entry";
            return false;
        }
    }

    params_.assign(blob.begin() + static_cast<long>(off), blob.end());
    layers_.clear();
    size_t p = 0;
    for (size_t i = 0; i + 1 < sizes_.size(); ++i) {
        const uint16_t in = static_cast<uint16_t>(sizes_[i]);
        const uint16_t out = static_cast<uint16_t>(sizes_[i + 1]);
        policy::Dense d{};
        d.in = in;
        d.out = out;
        d.w = params_.data() + p;
        p += static_cast<size_t>(in) * static_cast<size_t>(out);
        d.b = params_.data() + p;
        p += out;
        layers_.push_back(d);
    }

    builtin_ = false;
    source_ = blob_path;
    net_.act_dim = act_dim;
    bind();
    return true;
}

void LoadedNet::forward(const float* obs, float* action) const {
    policy::forwardNet(net_, obs, action);
}

bool loadCalibration(const char* path, obs::Calibration& cal,
                     std::string& err) {
    if (!path || !*path) {
        cal = obs::Calibration();
        return true;
    }
    siljson::Value root;
    if (!siljson::parseFile(path, root, err)) return false;

    const siljson::Value* list = &root;
    if (root.isObj()) {
        static const char* kKeys[] = {"joints", "calibration", "servos",
                                      "cal"};
        list = root.getAny(kKeys, 4);
        if (!list) {
            err = std::string(path) +
                  ": expected an array of {bus_id, zero_steps, dir}";
            return false;
        }
    }
    // One entry per servo the policy drives at least (every one of them
    // must be calibrated), any bus servo at most (obs/bus_map.h).
    if (!list->isArr() ||
        list->arr.size() < static_cast<size_t>(obs::kNumJoints) ||
        list->arr.size() > static_cast<size_t>(obs::kNumBusJoints)) {
        char msg[128];
        snprintf(msg, sizeof msg, ": expected %d..%d calibration entries",
                 obs::kNumJoints, obs::kNumBusJoints);
        err = std::string(path) + msg;
        return false;
    }

    obs::Calibration out;
    bool seen[obs::kNumBusJoints] = {};
    for (const siljson::Value& e : list->arr) {
        const siljson::Value* bus = e.get("bus_id");
        const siljson::Value* zero = e.get("zero_steps");
        const siljson::Value* dir = e.get("dir");
        if (!bus || !zero || !dir || !bus->isNum() || !zero->isNum() ||
            !dir->isNum()) {
            err = std::string(path) +
                  ": each entry needs numeric bus_id, zero_steps and dir";
            return false;
        }
        const int id = static_cast<int>(bus->num);
        const int joint = obs::busIndexOfId(id);   // the bus slot
        if (joint < 0) {
            char msg[96];
            snprintf(msg, sizeof msg, ": bus_id %d is not on the bus", id);
            err = std::string(path) + msg;
            return false;
        }
        if (seen[joint]) {
            char msg[96];
            snprintf(msg, sizeof msg, ": bus_id %d appears twice", id);
            err = std::string(path) + msg;
            return false;
        }
        seen[joint] = true;
        const int z = static_cast<int>(zero->num);
        const int d = static_cast<int>(dir->num);
        if (z < 0 || z > 4095) {
            err = std::string(path) + ": zero_steps outside 0..4095";
            return false;
        }
        if (d != 1 && d != -1) {
            err = std::string(path) + ": dir must be +1 or -1";
            return false;
        }
        out.zero_steps[joint] = z;
        out.dir[joint] = static_cast<int8_t>(d);
    }
    for (int j = 0; j < obs::kNumJoints; ++j) {
        if (!seen[obs::policyToBus(j)]) {
            char msg[96];
            snprintf(msg, sizeof msg, ": no entry for bus_id %d",
                     static_cast<int>(obs::kServoId[j]));
            err = std::string(path) + msg;
            return false;
        }
    }
    cal = out;
    return true;
}

}  // namespace sil

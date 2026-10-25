#include "bench.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace bimo {

// -- JsonObj ----------------------------------------------------------------
namespace {

const char* skipWs(const char* p, const char* e) {
    while (p < e && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) ++p;
    return p;
}

// Reads a "..." string into out (truncating, which is harmless for names).
// Returns the position after the closing quote, or nullptr.
const char* readString(const char* p, const char* e, char* out, size_t cap) {
    if (p >= e || *p != '"') return nullptr;
    ++p;
    size_t w = 0;
    while (p < e && *p != '"') {
        char c = *p++;
        if (c == '\\') {
            if (p >= e) return nullptr;
            c = *p++;
            switch (c) {
                case 'n': c = '\n'; break;
                case 't': c = '\t'; break;
                case '"': case '\\': case '/': break;
                case 'u':               // \uXXXX: keep ASCII, else '?'
                    if (e - p < 4) return nullptr;
                    {
                        char hex[5] = {p[0], p[1], p[2], p[3], 0};
                        char* end = nullptr;
                        const long v = strtol(hex, &end, 16);
                        if (end != hex + 4) return nullptr;
                        c = (v > 0 && v < 128) ? static_cast<char>(v) : '?';
                    }
                    p += 4;
                    break;
                default: return nullptr;
            }
        } else if (static_cast<unsigned char>(c) < 0x20) {
            return nullptr;
        }
        if (w + 1 < cap) out[w++] = c;
    }
    if (p >= e) return nullptr;
    if (cap > 0) out[w] = 0;
    return p + 1;
}

}  // namespace

bool JsonObj::parse(const char* s, size_t n) {
    n_ = 0;
    const char* p = s;
    const char* e = s + n;
    p = skipWs(p, e);
    if (p >= e || *p != '{') return false;
    ++p;
    p = skipWs(p, e);
    if (p < e && *p == '}') return true;
    for (;;) {
        if (n_ >= kMax) return false;
        KV& kv = kv_[n_];
        kv.num = 0.0;
        kv.str[0] = 0;
        p = skipWs(p, e);
        p = readString(p, e, kv.key, sizeof kv.key);
        if (p == nullptr) return false;
        p = skipWs(p, e);
        if (p >= e || *p != ':') return false;
        p = skipWs(p + 1, e);
        if (p >= e) return false;
        if (*p == '"') {
            kv.type = kStr;
            p = readString(p, e, kv.str, sizeof kv.str);
            if (p == nullptr) return false;
        } else if (e - p >= 4 && !strncmp(p, "true", 4)) {
            kv.type = kBool; kv.num = 1.0; p += 4;
        } else if (e - p >= 5 && !strncmp(p, "false", 5)) {
            kv.type = kBool; kv.num = 0.0; p += 5;
        } else if (e - p >= 4 && !strncmp(p, "null", 4)) {
            kv.type = kNull; p += 4;
        } else {
            char buf[40];
            size_t w = 0;
            while (p < e && w + 1 < sizeof buf &&
                   (*p == '-' || *p == '+' || *p == '.' || *p == 'e' ||
                    *p == 'E' || (*p >= '0' && *p <= '9'))) {
                buf[w++] = *p++;
            }
            buf[w] = 0;
            if (w == 0) return false;      // also: '{' and '[' (nesting)
            char* end = nullptr;
            kv.num = strtod(buf, &end);
            if (end != buf + w || !isfinite(kv.num)) return false;
            kv.type = kNum;
        }
        ++n_;
        p = skipWs(p, e);
        if (p >= e) return false;
        if (*p == ',') { ++p; continue; }
        if (*p == '}') { return skipWs(p + 1, e) == e; }
        return false;
    }
}

const JsonObj::KV* JsonObj::find(const char* key) const {
    for (int i = 0; i < n_; ++i) {
        if (!strcmp(kv_[i].key, key)) return &kv_[i];
    }
    return nullptr;
}
bool JsonObj::has(const char* key) const { return find(key) != nullptr; }
bool JsonObj::num(const char* key, double* out) const {
    const KV* kv = find(key);
    if (kv == nullptr || kv->type != kNum) return false;
    *out = kv->num;
    return true;
}
double JsonObj::numOr(const char* key, double dflt) const {
    double v;
    return num(key, &v) ? v : dflt;
}
const char* JsonObj::str(const char* key) const {
    const KV* kv = find(key);
    return (kv != nullptr && kv->type == kStr) ? kv->str : nullptr;
}
bool JsonObj::flag(const char* key, bool dflt) const {
    const KV* kv = find(key);
    return (kv != nullptr && kv->type == kBool) ? kv->num != 0.0 : dflt;
}

// -- TimeSeries ---------------------------------------------------------------
void TimeSeries::push(double t_ms, float v) {
    t_[off_] = t_ms;
    v_[off_] = v;
    off_ = (off_ + 1) % kCap;
    if (n_ < kCap) ++n_;
}

float TimeSeries::last() const {
    return n_ == 0 ? 0.0f : v_[(off_ + kCap - 1) % kCap];
}

int TimeSeries::resample(double now_ms, double window_ms, float* out,
                         int bins) const {
    if (bins <= 0) return 0;
    const double t0 = now_ms - window_ms;
    const double bw = window_ms / bins;
    // Samples are pushed in time order, so walk oldest -> newest once.
    const int start = (n_ < kCap) ? 0 : off_;
    int i = 0;                         // next sample to consume
    bool have = false;
    float cur = 0.0f;
    int first = bins;
    for (int b = 0; b < bins; ++b) {
        const double end = t0 + bw * (b + 1);
        while (i < n_ && t_[(start + i) % kCap] <= end) {
            cur = v_[(start + i) % kCap];
            have = true;
            ++i;
        }
        if (have) {
            if (first == bins) first = b;
            out[b] = cur;
        }
    }
    return first;
}

// -- BenchModel -----------------------------------------------------------------
namespace {
void cpy(char* dst, size_t cap, const char* src) {
    snprintf(dst, cap, "%s", src != nullptr ? src : "");
}
}  // namespace

bool BenchModel::apply(const char* data, size_t n, double now_ms) {
    JsonObj j;
    if (!j.parse(data, n) || j.numOr("v", 0) != 1.0 || j.str("k") == nullptr) {
        ++rejected;
        return false;
    }
    const char* k = j.str("k");
    const double prev_rx = last_rx_ms;
    ++datagrams;
    last_rx_ms = now_ms;
    if (!strcmp(k, "hb")) {
        last_hb_ms = now_ms;
        hb_polling = j.flag("polling", false);
        hb_rx_age = j.numOr("rx_age", -1.0);
        hb_polls = static_cast<uint32_t>(j.numOr("polls", 0));
    } else if (!strcmp(k, "joint") || !strcmp(k, "cal")) {
        const int id = static_cast<int>(j.numOr("id", -1));
        if (id < 0 || id >= kBenchMaxId) {
            // Not a sample after all: it must not count as hearing the bridge.
            --datagrams;
            last_rx_ms = prev_rx;
            ++rejected;
            return false;
        }
        BenchJoint& b = joints[id];
        b.id = id;
        const char* nm = j.str("name");
        if (nm != nullptr && nm[0]) cpy(b.name, sizeof b.name, nm);
        if (!strcmp(k, "joint")) {
            b.have = true;
            b.pos = static_cast<int>(j.numOr("pos", 0));
            b.spd = static_cast<int>(j.numOr("spd", 0));
            b.load = static_cast<int>(j.numOr("load", 0));
            b.temp = static_cast<int>(j.numOr("temp", 0));
            b.err = static_cast<int>(j.numOr("err", 0));
            b.volt = static_cast<float>(j.numOr("volt", 0));
            b.t_ms = now_ms;
        } else {
            b.have_cal = true;
            b.zero = static_cast<int>(j.numOr("zero", 0));
            b.dir = static_cast<int>(j.numOr("dir", 1));
            b.fitted = j.flag("fitted", true);
        }
    } else if (!strcmp(k, "bal_roll")) {
        double v;
        roll_na = !j.num("roll", &v);
        if (!roll_na) {
            have_roll = true;
            roll = static_cast<float>(v);
            t_roll = now_ms;
            s_roll.push(now_ms, roll);
        }
    } else if (!strcmp(k, "bal_state")) {
        cpy(state, sizeof state, j.str("state"));
        const char* st = j.str("stance");
        stance = (st != nullptr) ? st[0] : 0;
        since = static_cast<float>(j.numOr("since", 0));
        t_state = now_ms;
    } else if (!strcmp(k, "bal_ref")) {
        have_ref = true;
        ref = static_cast<float>(j.numOr("ref", 0));
        err = static_cast<float>(j.numOr("err", 0));
        integ = static_cast<float>(j.numOr("I", 0));
        u = static_cast<float>(j.numOr("u", 0));
        t_ref = now_ms;
        s_err.push(now_ms, err);
        s_u.push(now_ms, u);
    } else if (!strcmp(k, "bal_corr")) {
        have_corr = true;
        const char* sd = j.str("side");
        corr_side = sd != nullptr ? sd[0] : 0;
        ankle_deg = static_cast<float>(j.numOr("ankle_deg", 0));
        hip_deg = static_cast<float>(j.numOr("hip_deg", 0));
        ankle_ticks = static_cast<int>(j.numOr("ankle_ticks", 0));
        hip_ticks = static_cast<int>(j.numOr("hip_ticks", 0));
        clamped = static_cast<int>(j.numOr("clamped", 0));
    } else if (!strcmp(k, "bal_gains")) {
        have_gains = true;
        kp = static_cast<float>(j.numOr("kp", 0));
        ki = static_cast<float>(j.numOr("ki", 0));
        clamp = static_cast<float>(j.numOr("clamp", 0));
        abort_deg = static_cast<float>(j.numOr("abort", 0));
    } else if (!strcmp(k, "imu_up")) {
        have_up = true;
        up[0] = static_cast<float>(j.numOr("x", 0));
        up[1] = static_cast<float>(j.numOr("y", 0));
        up[2] = static_cast<float>(j.numOr("z", 0));
        t_up = now_ms;
    } else if (!strcmp(k, "imu_proj")) {
        have_proj = true;
        proj[0] = static_cast<float>(j.numOr("x", 0));
        proj[1] = static_cast<float>(j.numOr("y", 0));
        proj[2] = static_cast<float>(j.numOr("z", 0));
    } else if (!strcmp(k, "imu_gyro")) {
        have_gyro = true;
        gyro[0] = static_cast<float>(j.numOr("x", 0));
        gyro[1] = static_cast<float>(j.numOr("y", 0));
        gyro[2] = static_cast<float>(j.numOr("z", 0));
    } else if (!strcmp(k, "imu_level")) {
        have_level = true;
        levelled = j.flag("levelled", false);
        streak = static_cast<int>(j.numOr("streak", 0));
    } else if (!strcmp(k, "imu_raw")) {
        have_raw = true;
        acc[0] = static_cast<float>(j.numOr("ax", 0));
        acc[1] = static_cast<float>(j.numOr("ay", 0));
        acc[2] = static_cast<float>(j.numOr("az", 0));
        accmag = static_cast<float>(j.numOr("amag", 0));
        graw[0] = static_cast<float>(j.numOr("gx", 0));
        graw[1] = static_cast<float>(j.numOr("gy", 0));
        graw[2] = static_cast<float>(j.numOr("gz", 0));
        t_raw = now_ms;
    }
    // Unknown kinds are counted as heard (the bridge is alive) and ignored:
    // a newer bridge may send more than this console knows.
    return true;
}

int BenchModel::jointsSeen() const {
    int c = 0;
    for (int i = 0; i < kBenchMaxId; ++i) c += joints[i].have ? 1 : 0;
    return c;
}

// -- BenchLink --------------------------------------------------------------------
bool BenchLink::open(const char* spec, int default_port) {
    close();
    char host[96];
    int port = default_port;
    snprintf(host, sizeof host, "%s", spec);
    char* colon = strrchr(host, ':');
    if (colon != nullptr) {
        char* end = nullptr;
        const long p = strtol(colon + 1, &end, 10);
        if (end == colon + 1 || *end != 0 || p < 1 || p > 65535) {
            snprintf(err_, sizeof err_, "bad port in `%s`", spec);
            return false;
        }
        port = static_cast<int>(p);
        *colon = 0;
    }
    if (host[0] == 0 || port < 1 || port > 65535) {
        snprintf(err_, sizeof err_, "expected HOST[:PORT], got `%s`", spec);
        return false;
    }
    addrinfo hints;
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* res = nullptr;
    char ps[8];
    snprintf(ps, sizeof ps, "%d", port);
    const int rc = getaddrinfo(host, ps, &hints, &res);
    if (rc != 0 || res == nullptr) {
        snprintf(err_, sizeof err_, "cannot resolve `%s`: %s", host,
                 gai_strerror(rc));
        return false;
    }
    if (res->ai_addrlen > sizeof addr_) {
        freeaddrinfo(res);
        snprintf(err_, sizeof err_, "address too long");
        return false;
    }
    memcpy(addr_, res->ai_addr, res->ai_addrlen);
    addr_len_ = res->ai_addrlen;
    freeaddrinfo(res);
    fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        snprintf(err_, sizeof err_, "socket: %s", strerror(errno));
        return false;
    }
    fcntl(fd_, F_SETFL, fcntl(fd_, F_GETFL, 0) | O_NONBLOCK);
    snprintf(target_, sizeof target_, "%s:%d", host, port);
    err_[0] = 0;
    next_sub_ms_ = 0.0;
    model = BenchModel();
    return true;
}

void BenchLink::close() {
    if (fd_ >= 0) {
        static const char kBye[] = "{\"unsub\":1}";
        sendto(fd_, kBye, sizeof kBye - 1, 0,
               reinterpret_cast<const sockaddr*>(addr_),
               static_cast<socklen_t>(addr_len_));
        ::close(fd_);
        fd_ = -1;
    }
}

void BenchLink::poll(double now_ms) {
    if (fd_ < 0) return;
    if (now_ms >= next_sub_ms_) {
        // The lease is 10 s; 2 s keeps it through four lost datagrams.
        static const char kSub[] = "{\"sub\":1}";
        sendto(fd_, kSub, sizeof kSub - 1, 0,
               reinterpret_cast<const sockaddr*>(addr_),
               static_cast<socklen_t>(addr_len_));
        next_sub_ms_ = now_ms + 2000.0;
    }
    char buf[1500];
    for (int i = 0; i < 256; ++i) {
        const ssize_t n = recv(fd_, buf, sizeof buf, 0);
        if (n <= 0) break;
        if (model.apply(buf, static_cast<size_t>(n), now_ms) && sink_ != nullptr) {
            sink_(sink_ctx_, buf, static_cast<size_t>(n), now_ms);
        }
    }
}

}  // namespace bimo

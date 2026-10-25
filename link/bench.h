// The bench telemetry client: what tools/board_bridge.py publishes while the
// robot is driven over its serial tether and has no WiFi beacon to chart.
//
// Transport is one JSON object per UDP datagram (docs/bench-console.md). This
// console SUBSCRIBES -- it sends {"sub":1} to the bridge every 2 s -- so the
// bridge never needs to know where a Mac is, and a console that goes away
// costs the bridge nothing after its 10 s lease. This file never sends
// anything else: the bench side is watch-only by construction.
//
// Like mjpeg.h, the parts with logic in them (the JSON reader, the model, the
// time-series binning) are split from the socket so firmware/host/test_client
// can drive them with canned datagrams, including the hostile ones.
//
// Every time here is THIS machine's monotonic clock at arrival, never the
// bridge's `t`: a Mac and mira disagree about the time by seconds, and a
// 30 s chart drawn on the wrong clock would silently show the wrong 30 s.
#ifndef BIMO_LINK_BENCH_H_
#define BIMO_LINK_BENCH_H_

#include <stddef.h>
#include <stdint.h>

namespace bimo {

// A flat JSON object: string, number, true/false/null values, no nesting.
// That is all a datagram carries; anything else is rejected, not guessed at.
class JsonObj {
  public:
    static constexpr int kMax = 24;
    bool parse(const char* s, size_t n);
    bool has(const char* key) const;
    // False if absent, null or not a number.
    bool num(const char* key, double* out) const;
    double numOr(const char* key, double dflt) const;
    // nullptr if absent or not a string.
    const char* str(const char* key) const;
    bool flag(const char* key, bool dflt) const;

  private:
    enum Type { kNum, kStr, kBool, kNull };
    struct KV {
        char key[20];
        Type type;
        double num;
        char str[80];
    };
    const KV* find(const char* key) const;
    KV kv_[kMax];
    int n_ = 0;
};

// Samples over time, binned for a fixed-width chart.
class TimeSeries {
  public:
    static constexpr int kCap = 1024;
    void push(double t_ms, float v);
    int count() const { return n_; }
    // Fills out[0..bins) with the value in force at the END of each bin of
    // [now - window, now] (sample-and-hold). Returns the index of the first
    // bin that has data; bins before it are not valid. Returns bins when
    // nothing falls inside or before the window.
    int resample(double now_ms, double window_ms, float* out, int bins) const;
    float last() const;

  private:
    double t_[kCap] = {0};
    float v_[kCap] = {0};
    int off_ = 0, n_ = 0;
};

struct BenchJoint {
    bool have = false;
    int id = 0;
    char name[24] = "";
    int pos = 0, spd = 0, load = 0, temp = 0, err = 0;
    float volt = 0.0f;
    double t_ms = 0.0;
    bool have_cal = false;
    int zero = 0, dir = 1;
    bool fitted = true;
};

constexpr int kBenchMaxId = 64;
constexpr double kBenchWindowMs = 30000.0;
// Heartbeats come at 1 Hz; this long without ANY datagram is a dead bridge,
// a dead tunnel or a lease that lapsed -- and must say so.
constexpr double kBenchStaleMs = 3500.0;

struct BenchModel {
    BenchJoint joints[kBenchMaxId];
    // balance
    bool have_roll = false, roll_na = false;
    float roll = 0.0f;
    double t_roll = 0.0;
    char state[64] = "";
    char stance = 0;
    float since = 0.0f;
    double t_state = 0.0;
    bool have_ref = false;
    float ref = 0.0f, err = 0.0f, integ = 0.0f, u = 0.0f;
    double t_ref = 0.0;
    bool have_corr = false;
    char corr_side = 0;
    float ankle_deg = 0.0f, hip_deg = 0.0f;
    int ankle_ticks = 0, hip_ticks = 0, clamped = 0;
    bool have_gains = false;
    float kp = 0, ki = 0, clamp = 0, abort_deg = 0;
    // imu
    bool have_up = false;
    float up[3] = {0, 0, 1};
    double t_up = 0.0;
    bool have_level = false, levelled = false;
    int streak = 0;
    bool have_proj = false;
    float proj[3] = {0, 0, 1};
    bool have_gyro = false;
    float gyro[3] = {0, 0, 0};
    bool have_raw = false;
    float acc[3] = {0, 0, 0}, accmag = 0, graw[3] = {0, 0, 0};
    double t_raw = 0.0;

    TimeSeries s_roll, s_err, s_u;

    // bridge
    uint32_t datagrams = 0, rejected = 0;
    double last_rx_ms = 0.0;     // any datagram
    double last_hb_ms = 0.0;
    bool hb_polling = false;
    double hb_rx_age = -1.0;     // seconds the bridge has gone without a board byte
    uint32_t hb_polls = 0;

    // Returns false for a datagram that is not a valid v1 sample.
    bool apply(const char* data, size_t n, double now_ms);
    int jointsSeen() const;
    // The same model fed from the Wi-Fi beacon instead of the serial bridge:
    // the attitude vector is the one field both carry in the same convention
    // (body-frame up). Does NOT count as hearing the bridge (live()).
    void applyBeaconUp(float x, float y, float z, double now_ms) {
        have_up = true;
        up[0] = x; up[1] = y; up[2] = z;
        t_up = now_ms;
    }
    // The bridge: has it been heard from lately?
    bool live(double now_ms) const {
        return datagrams > 0 && now_ms - last_rx_ms < kBenchStaleMs;
    }
};

class BenchLink {
  public:
    BenchLink() = default;
    ~BenchLink() { close(); }
    BenchLink(const BenchLink&) = delete;
    BenchLink& operator=(const BenchLink&) = delete;

    // "host:port" (host may be a name). Resolves once; false on failure,
    // with err() saying why.
    bool open(const char* spec, int default_port);
    void close();
    bool active() const { return fd_ >= 0; }
    // Re-subscribes every 2 s and drains whatever has arrived.
    void poll(double now_ms);
    // Called with each datagram the model ACCEPTED, verbatim, so the console
    // can record exactly what it displayed.
    typedef void (*Sink)(void* ctx, const char* data, size_t n, double now_ms);
    void setSink(Sink fn, void* ctx) { sink_ = fn; sink_ctx_ = ctx; }
    const char* err() const { return err_; }
    const char* target() const { return target_; }

    BenchModel model;

  private:
    Sink sink_ = nullptr;
    void* sink_ctx_ = nullptr;
    int fd_ = -1;
    uint8_t addr_[32] = {0};
    size_t addr_len_ = 0;
    double next_sub_ms_ = 0.0;
    char err_[160] = "";
    char target_[112] = "";
};

}  // namespace bimo

#endif  // BIMO_LINK_BENCH_H_

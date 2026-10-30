#include "flightrec/record.h"

#include <math.h>
#include <string.h>

namespace flightrec {
namespace {

// Little-endian writer/reader over a caller buffer. Every put/get checks the
// bound, so an encoder handed a short buffer returns 0 instead of overrunning,
// and a decoder handed a short packet returns false.
class W {
  public:
    W(uint8_t* p, size_t cap) : p_(p), cap_(cap) {}
    void u8(uint8_t v) {
        if (n_ + 1 > cap_) { bad_ = true; return; }
        p_[n_++] = v;
    }
    void i8(int8_t v) { u8(static_cast<uint8_t>(v)); }
    void u16(uint16_t v) {
        u8(static_cast<uint8_t>(v & 0xFFu));
        u8(static_cast<uint8_t>(v >> 8));
    }
    void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }
    void u32(uint32_t v) {
        for (int i = 0; i < 4; ++i) u8(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
    }
    void i32(int32_t v) { u32(static_cast<uint32_t>(v)); }
    void u64(uint64_t v) {
        for (int i = 0; i < 8; ++i) u8(static_cast<uint8_t>((v >> (8 * i)) & 0xFFu));
    }
    void f32(float f) {
        uint32_t v = 0;
        memcpy(&v, &f, sizeof v);
        u32(v);
    }
    void bytes(const void* s, size_t n) {
        if (n_ + n > cap_) { bad_ = true; return; }
        memcpy(p_ + n_, s, n);
        n_ += n;
    }
    size_t done() const { return bad_ ? 0 : n_; }

  private:
    uint8_t* p_;
    size_t cap_;
    size_t n_ = 0;
    bool bad_ = false;
};

class R {
  public:
    R(const uint8_t* p, size_t len) : p_(p), len_(len) {}
    uint8_t u8() {
        if (n_ + 1 > len_) { bad_ = true; return 0; }
        return p_[n_++];
    }
    int8_t i8() { return static_cast<int8_t>(u8()); }
    uint16_t u16() {
        const uint16_t lo = u8();
        const uint16_t hi = u8();
        return static_cast<uint16_t>(lo | (hi << 8));
    }
    int16_t i16() { return static_cast<int16_t>(u16()); }
    uint32_t u32() {
        uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<uint32_t>(u8()) << (8 * i);
        return v;
    }
    int32_t i32() { return static_cast<int32_t>(u32()); }
    uint64_t u64() {
        uint64_t v = 0;
        for (int i = 0; i < 8; ++i) v |= static_cast<uint64_t>(u8()) << (8 * i);
        return v;
    }
    float f32() {
        const uint32_t v = u32();
        float f = 0.0f;
        memcpy(&f, &v, sizeof f);
        return f;
    }
    void bytes(void* d, size_t n) {
        if (n_ + n > len_) { bad_ = true; return; }
        memcpy(d, p_ + n_, n);
        n_ += n;
    }
    size_t pos() const { return n_; }
    bool ok() const { return !bad_; }

  private:
    const uint8_t* p_;
    size_t len_;
    size_t n_ = 0;
    bool bad_ = false;
};

void putHeader(W& w, const Header& h) {
    w.u8(kMagic0);
    w.u8(kMagic1);
    w.u8(kVersion);
    w.u8(static_cast<uint8_t>(h.type));
    w.u32(h.boot_id);
    w.u32(h.seq);
    w.u64(h.t_utc_us);
    w.u64(h.t_boot_us);
    w.u32(h.drop_ring);
    w.u32(h.drop_net);
}

bool getHeader(R& r, Header& h) {
    if (r.u8() != kMagic0 || r.u8() != kMagic1) return false;
    if (r.u8() != kVersion) return false;
    h.type = static_cast<Type>(r.u8());
    h.boot_id = r.u32();
    h.seq = r.u32();
    h.t_utc_us = r.u64();
    h.t_boot_us = r.u64();
    h.drop_ring = r.u32();
    h.drop_net = r.u32();
    return r.ok();
}

}  // namespace

int16_t sat16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return static_cast<int16_t>(v);
}

size_t encodeTick(uint8_t* out, size_t cap, Header h, const Tick& t) {
    if (t.njoint > kMaxJoints) return 0;
    h.type = Type::kTick;
    W w(out, cap);
    putHeader(w, h);
    // fixed block: kTickFixedLen bytes
    w.u8(static_cast<uint8_t>(t.source));
    w.u8(t.flags);
    w.u8(t.link_state);
    w.u8(t.njoint);
    w.u32(t.tick);
    w.u32(t.work_us);
    w.i32(t.fb_off_us);
    w.u32(t.fb_read_us);
    w.i32(t.imu_age_us);
    for (float v : t.accel) w.f32(v);
    for (float v : t.gyro) w.f32(v);
    for (float v : t.quat) w.f32(v);
    for (float v : t.up) w.f32(v);
    w.f32(t.roll);
    w.f32(t.pitch);
    const Balance& b = t.bal;
    w.u8(b.state);
    w.u8(b.side);
    w.i8(b.ankle_bus);
    w.i8(b.hip_bus);
    w.f32(b.ref);
    w.f32(b.err);
    w.f32(b.integral);
    w.f32(b.u);
    w.f32(b.rate);
    w.f32(b.roll);
    w.f32(b.ankle_rad);
    w.f32(b.hip_rad);
    w.i16(b.ankle_ticks);
    w.i16(b.hip_ticks);
    w.u32(b.clamps);
    const Ctrl& c = t.ctrl;
    for (float v : c.cmd) w.f32(v);
    w.f32(c.phase);
    w.u16(c.vbat_mv);
    w.u8(c.batt);
    w.u8(c.home_stage);
    for (int k = 0; k < t.njoint; ++k) {
        const Joint& j = t.joint[k];
        w.u8(j.bus);
        w.u8(j.flags);
        w.u8(j.status);
        w.u8(j.temp_c);
        w.u8(j.voltage_dv);
        w.u8(j.moving);
        w.i16(j.nominal);
        w.i16(j.goal);
        w.u16(j.goal_speed);
        w.i16(j.pos);
        w.i16(j.speed);
        w.i16(j.load);
        w.i16(j.current);
    }
    return w.done();
}

bool decodeHeader(const uint8_t* in, size_t len, Header& h) {
    R r(in, len);
    return getHeader(r, h);
}

bool decodeTick(const uint8_t* in, size_t len, Header& h, Tick& t) {
    R r(in, len);
    if (!getHeader(r, h) || h.type != Type::kTick) return false;
    t.source = static_cast<Source>(r.u8());
    t.flags = r.u8();
    t.link_state = r.u8();
    t.njoint = r.u8();
    if (t.njoint > kMaxJoints) return false;
    t.tick = r.u32();
    t.work_us = r.u32();
    t.fb_off_us = r.i32();
    t.fb_read_us = r.u32();
    t.imu_age_us = r.i32();
    for (float& v : t.accel) v = r.f32();
    for (float& v : t.gyro) v = r.f32();
    for (float& v : t.quat) v = r.f32();
    for (float& v : t.up) v = r.f32();
    t.roll = r.f32();
    t.pitch = r.f32();
    Balance& b = t.bal;
    b.state = r.u8();
    b.side = r.u8();
    b.ankle_bus = r.i8();
    b.hip_bus = r.i8();
    b.ref = r.f32();
    b.err = r.f32();
    b.integral = r.f32();
    b.u = r.f32();
    b.rate = r.f32();
    b.roll = r.f32();
    b.ankle_rad = r.f32();
    b.hip_rad = r.f32();
    b.ankle_ticks = r.i16();
    b.hip_ticks = r.i16();
    b.clamps = r.u32();
    Ctrl& c = t.ctrl;
    for (float& v : c.cmd) v = r.f32();
    c.phase = r.f32();
    c.vbat_mv = r.u16();
    c.batt = r.u8();
    c.home_stage = r.u8();
    if (r.pos() != kHeaderLen + kTickFixedLen) return false;
    for (int k = 0; k < t.njoint; ++k) {
        Joint& j = t.joint[k];
        j.bus = r.u8();
        j.flags = r.u8();
        j.status = r.u8();
        j.temp_c = r.u8();
        j.voltage_dv = r.u8();
        j.moving = r.u8();
        j.nominal = r.i16();
        j.goal = r.i16();
        j.goal_speed = r.u16();
        j.pos = r.i16();
        j.speed = r.i16();
        j.load = r.i16();
        j.current = r.i16();
    }
    return r.ok() && r.pos() == len;
}

void setText(Event& e, const char* s) {
    size_t n = s ? strlen(s) : 0;
    while (n > 0 && (s[n - 1] == '\n' || s[n - 1] == '\r')) --n;
    if (n > static_cast<size_t>(kMaxText)) n = static_cast<size_t>(kMaxText);
    if (n) memcpy(e.text, s, n);
    e.len = static_cast<uint16_t>(n);
}

size_t encodeEvent(uint8_t* out, size_t cap, Header h, const Event& e) {
    if (e.len > kMaxText) return 0;
    h.type = Type::kEvent;
    W w(out, cap);
    putHeader(w, h);
    w.u8(static_cast<uint8_t>(e.code));
    w.u8(e.src);
    w.u16(e.len);
    w.i32(e.a);
    w.i32(e.b);
    w.bytes(e.text, e.len);
    return w.done();
}

bool decodeEvent(const uint8_t* in, size_t len, Header& h, Event& e) {
    R r(in, len);
    if (!getHeader(r, h) || h.type != Type::kEvent) return false;
    e.code = static_cast<Ev>(r.u8());
    e.src = r.u8();
    e.len = r.u16();
    e.a = r.i32();
    e.b = r.i32();
    if (e.len > kMaxText) return false;
    r.bytes(e.text, e.len);
    return r.ok() && r.pos() == len;
}

size_t encodeHello(uint8_t* out, size_t cap, Header h, const Hello& m) {
    h.type = Type::kHello;
    W w(out, cap);
    putHeader(w, h);
    w.bytes(m.fw, sizeof m.fw);
    w.u8(m.utc_synced);
    w.u8(m.enabled);
    w.u8(m.idle_hz);
    w.u8(m.cal_ok);
    const uint32_t c[] = {m.produced, m.sent, m.drop_ring, m.drop_net,
                          m.max_push_ctrl_us, m.max_push_house_us, m.bench_read_us,
                          m.ctrl_ticks, m.ctrl_late, m.ctrl_worst_us,
                          m.us_read, m.us_imu, m.us_obs, m.us_net, m.us_write,
                          m.us_other, m.uptime_ms, m.ntp_syncs, m.reserved};
    for (uint32_t v : c) w.u32(v);
    w.u8(static_cast<uint8_t>(kMaxJoints));
    w.u8(0);
    w.u8(0);
    w.u8(0);
    for (int b = 0; b < kMaxJoints; ++b) {
        w.u8(m.id[b]);
        w.u8(m.fitted[b]);
        w.i8(m.dir[b]);
        w.i16(m.zero[b]);
    }
    return w.done();
}

bool decodeHello(const uint8_t* in, size_t len, Header& h, Hello& m) {
    R r(in, len);
    if (!getHeader(r, h) || h.type != Type::kHello) return false;
    r.bytes(m.fw, sizeof m.fw);
    m.utc_synced = r.u8();
    m.enabled = r.u8();
    m.idle_hz = r.u8();
    m.cal_ok = r.u8();
    uint32_t* c[] = {&m.produced, &m.sent, &m.drop_ring, &m.drop_net,
                     &m.max_push_ctrl_us, &m.max_push_house_us, &m.bench_read_us,
                     &m.ctrl_ticks, &m.ctrl_late, &m.ctrl_worst_us,
                     &m.us_read, &m.us_imu, &m.us_obs, &m.us_net, &m.us_write,
                     &m.us_other, &m.uptime_ms, &m.ntp_syncs, &m.reserved};
    for (uint32_t* p : c) *p = r.u32();
    if (r.u8() != kMaxJoints) return false;
    r.u8();
    r.u8();
    r.u8();
    for (int b = 0; b < kMaxJoints; ++b) {
        m.id[b] = r.u8();
        m.fitted[b] = r.u8();
        m.dir[b] = r.i8();
        m.zero[b] = r.i16();
    }
    return r.ok() && r.pos() == len;
}

void rollPitchFromQuat(const float q[4], float* roll, float* pitch) {
    const float w = q[0], x = q[1], y = q[2], z = q[3];
    if (roll) *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    if (pitch) {
        float s = 2.0f * (w * y - z * x);
        if (s > 1.0f) s = 1.0f;
        if (s < -1.0f) s = -1.0f;
        *pitch = asinf(s);
    }
}

}  // namespace flightrec

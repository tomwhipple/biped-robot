#include "obs_dump.h"

#include <stdio.h>
#include <string.h>

namespace robot {
namespace {

// A truncation-safe append cursor. Every writer here checks `ok` before doing
// anything, so one overflow poisons the whole line and formatObsRecord()
// returns 0 rather than handing the host a half-record it would silently
// mis-parse.
struct Cursor {
    char* out;
    size_t cap;
    size_t pos = 0;
    bool ok = true;

    void put(const char* s) {
        if (!ok) return;
        const size_t n = strlen(s);
        if (pos + n + 1 > cap) { ok = false; return; }
        memcpy(out + pos, s, n);
        pos += n;
        out[pos] = '\0';
    }

    void putFloat(float v) {
        if (!ok) return;
        char tmp[24];
        // 6 significant digits: fp32 carries ~7, and the extra column width
        // over 83 channels costs more UART time than the last digit is worth.
        const int w = snprintf(tmp, sizeof tmp, ",%.6g", static_cast<double>(v));
        if (w < 0 || static_cast<size_t>(w) >= sizeof tmp) { ok = false; return; }
        put(tmp);
    }

    void putUint(uint32_t v) {
        if (!ok) return;
        char tmp[16];
        const int w = snprintf(tmp, sizeof tmp, ",%lu",
                               static_cast<unsigned long>(v));
        if (w < 0 || static_cast<size_t>(w) >= sizeof tmp) { ok = false; return; }
        put(tmp);
    }
};

// walker_env.set_command's channel order. Kept as names rather than indices
// because "cmd3 is 1.0" tells an operator nothing and "crouch is 1.0" does.
const char* const kCmdNames[7] = {"vx",   "vy",   "wz",     "crouch",
                                  "lift", "foot_dx", "foot_dz"};

void putCmdName(Cursor& c, int i) {
    if (i >= 0 && i < 7) { c.put(kCmdNames[i]); return; }
    char tmp[16];
    snprintf(tmp, sizeof tmp, "c%d", i);
    c.put(tmp);
}

const char* const kAxis[3] = {"x", "y", "z"};

// Name one column of the assembled frame from the generated offsets. The
// frame is the half of the dump that catches an ASSEMBLER fault (a section
// written at the wrong offset, a stale prev_action), so its columns are
// named by section instead of f00..f48.
void putFrameName(Cursor& c, int i) {
    static_assert(obs::kOffQ == 0, "frame starts at q");
    static_assert(obs::kOffCmd + obs::kNumCmd == obs::kFrameDim,
                  "cmd is the last frame section");
    static_assert(obs::kActDim == obs::kNumJoints,
                  "prev_action columns are named after the joints");
    if (i < obs::kOffDq) { c.put("f_q_"); c.put(obs::kJointNames[i - obs::kOffQ]); return; }
    if (i < obs::kOffUp) { c.put("f_dq_"); c.put(obs::kJointNames[i - obs::kOffDq]); return; }
    if (i < obs::kOffLinVel) { c.put("f_up_"); c.put(kAxis[i - obs::kOffUp]); return; }
    if (i < obs::kOffGyro) { c.put("f_linvel_"); c.put(kAxis[i - obs::kOffLinVel]); return; }
    if (i < obs::kOffPrevAction) { c.put("f_gyro_"); c.put(kAxis[i - obs::kOffGyro]); return; }
    if (i < obs::kOffHeight) {
        c.put("f_pa_");
        c.put(obs::kJointNames[i - obs::kOffPrevAction]);
        return;
    }
    if (i < obs::kOffPhase) { c.put("f_height"); return; }
    if (i < obs::kOffCmd) {
        c.put((i - obs::kOffPhase) == 0 ? "f_sin" : "f_cos");
        return;
    }
    c.put("f_cmd_");
    putCmdName(c, i - obs::kOffCmd);
}

}  // namespace

void ObsDump::publish(const obs::Inputs& in, const float* frame,
                      uint32_t tick) {
    const uint32_t s = seq_.load(std::memory_order_relaxed);
    ObsRecord& dst = slot_[(s + 1u) & 1u];
    dst.tick = tick;
    memcpy(dst.q, in.q, sizeof dst.q);
    memcpy(dst.dq, in.dq, sizeof dst.dq);
    memcpy(dst.up, in.up, sizeof dst.up);
    memcpy(dst.gyro, in.gyro, sizeof dst.gyro);
    dst.phase = in.phase;
    memcpy(dst.cmd, in.cmd, sizeof dst.cmd);
    memcpy(dst.frame, frame, sizeof dst.frame);
    // Release: the slot's contents must be visible before the counter that
    // advertises them.
    seq_.store(s + 1u, std::memory_order_release);
}

bool ObsDump::read(ObsRecord& out, uint32_t& seq_out) const {
    for (int attempt = 0; attempt < 4; ++attempt) {
        const uint32_t s0 = seq_.load(std::memory_order_acquire);
        if (s0 == 0) return false;              // nothing published yet
        out = slot_[s0 & 1u];
        // The slot we just copied is next written when the counter reaches
        // s0 + 1 (that publish targets the OTHER slot) and the one after it
        // starts. Requiring the counter not to have moved at all is the
        // conservative version of that test, and at 50 Hz against a copy of a
        // few hundred bytes it succeeds on the first attempt.
        if (seq_.load(std::memory_order_acquire) == s0) {
            seq_out = s0;
            return true;
        }
    }
    return false;
}

size_t formatObsRecord(const ObsRecord& r, char* out, size_t n) {
    Cursor c{out, n};
    c.put("OBS");
    c.putUint(r.tick);
    for (int i = 0; i < obs::kNumJoints; ++i) c.putFloat(r.q[i]);
    for (int i = 0; i < obs::kNumJoints; ++i) c.putFloat(r.dq[i]);
    for (int i = 0; i < 3; ++i) c.putFloat(r.up[i]);
    for (int i = 0; i < 3; ++i) c.putFloat(r.gyro[i]);
    c.putFloat(r.phase);
    for (int i = 0; i < obs::kNumCmd; ++i) c.putFloat(r.cmd[i]);
    for (int i = 0; i < obs::kFrameDim; ++i) c.putFloat(r.frame[i]);
    c.put("\r\n");
    return c.ok ? c.pos : 0;
}

size_t formatObsHeader(char* out, size_t n) {
    Cursor c{out, n};
    c.put("OBSHDR,tick");
    for (int i = 0; i < obs::kNumJoints; ++i) {
        c.put(",q_");
        c.put(obs::kJointNames[i]);
    }
    for (int i = 0; i < obs::kNumJoints; ++i) {
        c.put(",dq_");
        c.put(obs::kJointNames[i]);
    }
    for (int i = 0; i < 3; ++i) { c.put(",up_"); c.put(kAxis[i]); }
    for (int i = 0; i < 3; ++i) { c.put(",gyro_"); c.put(kAxis[i]); }
    c.put(",phase");
    for (int i = 0; i < obs::kNumCmd; ++i) {
        c.put(",cmd_");
        putCmdName(c, i);
    }
    for (int i = 0; i < obs::kFrameDim; ++i) {
        c.put(",");
        putFrameName(c, i);
    }
    c.put("\r\n");
    return c.ok ? c.pos : 0;
}

}  // namespace robot

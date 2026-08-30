// The observation dump: what the ARMED robot actually feeds the policy.
//
// WHY THIS EXISTS
// ---------------
// The same policy walks in SIL (docs/sil-harness.md) and produces garbage on
// hardware. Every layer between the two has been eliminated except one: the
// numbers the real sensors put into obs::Inputs. A unit error (deg vs rad),
// a sign flip, a swapped axis or a permuted joint index all look identical
// from outside -- the loop runs, the servos move, the robot falls over -- and
// none of them can be diagnosed by reading code, only by MEASURING the vector.
//
// So this publishes, every tick, both halves of the observation:
//   * the RAW obs::Inputs the loop built from sensors, and
//   * the assembled kFrameDim frame handed to the history ring,
// and the CLI streams them as one CSV line per record. tools/obs_capture.py
// turns those into per-channel min/max/mean/RMS, which is a number that can be
// compared against the same statistic from the sim.
//
// OWNERSHIP (shared.h section 4 rules)
// ------------------------------------
// Exactly one writer per datum, no locks in the control path:
//   * the CONTROL task writes the double buffer (publish()) and only READS
//     the mode;
//   * the HOUSEKEEPING task writes the mode (setMode()) and only READS the
//     buffer (read()).
// The buffer is two slots plus a sequence counter, so a publish never lands
// in the slot a reader is copying unless the reader is two ticks slow -- and
// read() detects that case and retries rather than returning a torn record.
// Nothing here allocates, and publish() is a pair of memcpy-sized copies.
//
// It lives in main/ rather than components/ for the same reason cal_store
// does: it is loop plumbing, not a pure module. It deliberately depends on
// nothing but <atomic> and the obs headers, so the host gate compiles it.
#pragma once

#include <atomic>
#include <stddef.h>
#include <stdint.h>

#include "obs/assembler.h"
#include "obs/obs_spec.h"

namespace robot {

// One tick of the policy's input, raw and assembled. Fixed size, POD, sized
// entirely from the generated obs spec -- a retrain that changes the layout
// changes this record with it.
struct ObsRecord {
    uint32_t tick;                   // g_telemetry.ticks at publish time
    float q[obs::kNumJoints];        // rad, sim sign convention
    float dq[obs::kNumJoints];       // rad/s
    float up[3];                     // torso up-vector, body frame
    float gyro[3];                   // rad/s, body frame
    float phase;                     // gait-clock phase, rad
    float cmd[obs::kNumCmd];         // command channels
    float frame[obs::kFrameDim];     // what assembleFrame() produced
};

// CSV value columns after the "OBS" tag: tick, then the raw inputs in the
// order above, then the whole frame.
inline constexpr int kObsDumpCols = 1 + 2 * obs::kNumJoints + 3 + 3 + 1 +
                                    obs::kNumCmd + obs::kFrameDim;

// Enough for the longest record line: kObsDumpCols values at "%.6g" (12 chars
// worst case, e.g. "-1.23457e-06") plus a separator each, the tag, and CRLF.
inline constexpr size_t kObsDumpLineMax = 1536;

// Stream rate. The loop publishes at 50 Hz; the console decimates, because a
// ~1.1 kB record at 50 Hz is 550 kB/s and the tether is 115200 baud (11.5
// kB/s). 5 Hz is ~5.5 kB/s -- about half the link, leaving room for the 10 Hz
// binary telemetry and typed commands. The dump is for measuring the SCALE
// and SIGN of channels over seconds, not for reconstructing a trajectory, so
// undersampling costs nothing that matters.
inline constexpr int kObsDumpHz = 5;
inline constexpr int kObsDumpPeriodMs = 1000 / kObsDumpHz;

enum class ObsDumpMode : uint8_t {
    kOff = 0,
    kStream,     // publish every tick, housekeeping decimates to ~5 Hz
    kOnce,       // publish until housekeeping has printed one record
};

class ObsDump {
  public:
    // -- control task (the writer) ----------------------------------------
    // Cheap enough to call unconditionally, but the caller checks first so a
    // disabled dump costs one relaxed load per tick and nothing else.
    bool enabled() const {
        return mode_.load(std::memory_order_relaxed) != ObsDumpMode::kOff;
    }
    void publish(const obs::Inputs& in, const float* frame, uint32_t tick);

    // -- housekeeping task (the reader) ------------------------------------
    ObsDumpMode mode() const { return mode_.load(std::memory_order_acquire); }
    void setMode(ObsDumpMode m) { mode_.store(m, std::memory_order_release); }

    // Copy the newest published record. Returns false when nothing has been
    // published yet, or when the writer overran the copy four times running
    // (which on a 50 Hz tick means the reader was descheduled for >80 ms --
    // a dropped sample is the honest answer, not a torn one).
    bool read(ObsRecord& out, uint32_t& seq_out) const;

    // Publish count. Zero until the first publish; the housekeeping streamer
    // compares it against what it last printed so a stalled loop is visible
    // as silence rather than as a repeated record.
    uint32_t sequence() const { return seq_.load(std::memory_order_acquire); }

  private:
    ObsRecord slot_[2] = {};
    // Incremented once per publish, AFTER the slot is written. The slot last
    // written is (seq_ & 1); the next publish goes to the other one.
    std::atomic<uint32_t> seq_{0};
    std::atomic<ObsDumpMode> mode_{ObsDumpMode::kOff};
};

// Format one record as a single CSV line, "OBS,<tick>,<q0>,...", terminated
// with CRLF like every other console line. Returns the number of characters
// written (excluding the NUL), or 0 if the record did not fit -- a truncated
// record is worse than a missing one, so it is never emitted.
size_t formatObsRecord(const ObsRecord& r, char* out, size_t n);

// The matching column-name line, "OBSHDR,tick,q_L_hip_yaw,...". Emitted once
// when a dump is enabled so the host tool reads the channel names (and the
// joint order) off the firmware instead of duplicating obs_spec.h in python.
size_t formatObsHeader(char* out, size_t n);

}  // namespace robot

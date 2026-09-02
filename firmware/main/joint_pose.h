// The measured joint pose, published by the control loop for the WiFi beacon.
//
// WHY THIS EXISTS
// ---------------
// Mirror mode (docs/mirror-mode.md): bimo_gui drives the real robot while
// sil_twin runs the same command through the same policy and draws the
// robot's OBSERVED pose over its own as a ghost. The observed pose is
// ctrl_task's g_q -- calibrated joint angles in radians, obs_spec order, the
// exact vector the policy is fed -- and it has to cross from core 1 to the
// housekeeping core, where wifi_link.cpp builds the beacon.
//
// OWNERSHIP (shared.h section 4 rules)
// ------------------------------------
// The same shape as ObsDump, for the same reason shared.h gives there: a
// 10-wide payload is too big to tear harmlessly. Ten loose atomics would let
// a beacon carry tick t's q[0] beside tick t+1's q[9], which is precisely the
// kind of skew mirror mode exists to make visible -- a lie the instrument
// would tell about the very thing it measures. So: two slots and a sequence
// counter. The CONTROL task writes the slot the reader is not in and then
// bumps the counter (release); the READER copies the advertised slot and
// re-checks the counter (acquire), and a copy the writer overran is DROPPED
// rather than sent. No lock in the tick, no allocation, one 40-byte copy.
//
// The counter doubles as the freshness signal: while benched the loop does
// not read the bus (the CLI owns it) and publishes nothing, so a reader that
// sees the same sequence twice knows the pose is stale and can fall back to
// the classic frame instead of beaconing yesterday's angles as measured.
//
// Header-only and ESP-IDF-free on purpose, so the host gate compiles it
// (firmware/host/test_obsdump.cpp).
#pragma once

#include <atomic>
#include <stdint.h>
#include <string.h>

#include "obs/obs_spec.h"

namespace robot {

class JointPose {
  public:
    // -- control task (the writer) ----------------------------------------
    // q: obs::kNumJoints radians, obs_spec order. t_us: when the bus was
    // read, on esp_timer's clock (boot-relative microseconds) -- the beacon
    // turns it into wall time (timesync.h) so the stamp on a pose frame is
    // the measurement instant, not the send instant. Cheap enough to call
    // every tick unconditionally.
    void publish(const float* q, uint32_t tick, int64_t t_us) {
        const uint32_t s = seq_.load(std::memory_order_relaxed);
        Slot& dst = slot_[(s + 1u) & 1u];
        dst.tick = tick;
        dst.t_us = t_us;
        memcpy(dst.q, q, sizeof dst.q);
        // Release: the slot's contents must be visible before the counter
        // that advertises them.
        seq_.store(s + 1u, std::memory_order_release);
    }

    // -- housekeeping / wifi task (the reader) -----------------------------
    // Copy the newest published pose. Returns false when nothing has been
    // published yet, or when the writer overran the copy four times running
    // (a dropped sample is the honest answer, not a torn one). seq_out is
    // the publish count the copy belongs to; the same value twice means the
    // loop has stopped publishing -- see the header comment.
    bool read(float* q_out, uint32_t& seq_out, uint32_t* tick_out = nullptr,
              int64_t* t_us_out = nullptr) const {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const uint32_t s0 = seq_.load(std::memory_order_acquire);
            if (s0 == 0) return false;              // nothing published yet
            const Slot& src = slot_[s0 & 1u];
            float q[obs::kNumJoints];
            memcpy(q, src.q, sizeof q);
            const uint32_t tick = src.tick;
            const int64_t t_us = src.t_us;
            // The slot just copied is next written when the counter reaches
            // s0 + 2; requiring it not to have moved at all is the
            // conservative version of that test, and at 50 Hz against a
            // 40-byte copy it succeeds on the first attempt.
            if (seq_.load(std::memory_order_acquire) == s0) {
                memcpy(q_out, q, sizeof q);
                if (tick_out) *tick_out = tick;
                if (t_us_out) *t_us_out = t_us;
                seq_out = s0;
                return true;
            }
        }
        return false;
    }

    // Publish count; zero until the first publish.
    uint32_t sequence() const { return seq_.load(std::memory_order_acquire); }

  private:
    struct Slot {
        uint32_t tick;                 // g_ticks at publish time
        int64_t t_us;                  // esp_timer us when the bus was read
        float q[obs::kNumJoints];      // rad, sim sign convention
    };
    Slot slot_[2] = {};
    // Incremented once per publish, AFTER the slot is written. The slot last
    // written is (seq_ & 1); the next publish goes to the other one.
    std::atomic<uint32_t> seq_{0};
};

}  // namespace robot

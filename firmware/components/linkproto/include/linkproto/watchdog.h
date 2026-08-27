// C++ port of link/protocol.py:Watchdog -- what the robot does with (and
// without) commands.
//
// The clock is injected as now_ms on every call rather than read from a timer,
// exactly like the Python reference, so link loss is tested deterministically
// instead of slept through. On the board `now_ms` is esp_timer_get_time()/1000.
#pragma once
#include <stdint.h>

#include "linkproto/protocol.h"

namespace linkproto {

class Watchdog {
  public:
    explicit Watchdog(float stale_ms = kStaleMs, float relax_ms = kRelaxMs)
        : stale_ms_(stale_ms), relax_ms_(relax_ms) {}

    // Mirrors protocol.py's `if not 0 < stale_ms < relax_ms: raise`. We do not
    // use exceptions, so construction is checked explicitly.
    bool valid() const { return stale_ms_ > 0.0f && stale_ms_ < relax_ms_; }

    // Apply a decoded command. false = dropped as stale/reordered.
    //
    // UDP may reorder, so an older seq is not evidence of a live link and must
    // not pet the watchdog -- otherwise a reordered burst could hold the robot
    // LIVE on a command that is already history.
    bool accept(const Command& pkt, float now_ms);

    LinkState state(float now_ms) const;

    // The (vx, wz) to hand the policy right now.
    void command(float now_ms, float& vx, float& wz) const;

    uint32_t lastSeq() const { return have_seq_ ? last_seq_ : 0; }
    uint32_t rejected() const { return rejected_; }

  private:
    float stale_ms_;
    float relax_ms_;
    float last_rx_ms_ = 0.0f;
    bool have_rx_ = false;
    uint32_t last_seq_ = 0;
    bool have_seq_ = false;
    float cmd_vx_ = 0.0f;
    float cmd_wz_ = 0.0f;
    bool estop_ = false;
    uint32_t rejected_ = 0;    // stale/duplicate frames, for telemetry
};

// C++ port of protocol.py:ArmLatch -- the ARM bit's edges become bench/run
// requests. 0 -> 1 arms (the tethered `run`), 1 -> 0 benches. The first frame
// from a sender never counts, and a sender that never sets the bit never
// makes an edge, so it cannot bench a robot armed over the tether.
//
// Lives outside Watchdog on purpose: the watchdog is re-created on every
// bench -> run handover, and this must outlive it to see the edge that ends
// the run. Feed it every DECODED frame in arrival order, duplicates included.
class ArmLatch {
  public:
    // true = the frame requests a mode change; `want_armed` says which way.
    bool update(uint8_t flags, bool& want_armed);

    bool haveLevel() const { return have_level_; }
    bool level() const { return level_; }

  private:
    bool have_level_ = false;
    bool level_ = false;
};

}  // namespace linkproto

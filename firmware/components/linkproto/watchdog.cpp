#include "linkproto/watchdog.h"

namespace linkproto {

bool Watchdog::accept(const Command& pkt, float now_ms) {
    if (have_seq_ && pkt.seq <= last_seq_ &&
        (last_seq_ - pkt.seq) < kSeqResyncGap) {
        // Stale duplicate or reordered packet. A backwards jump BIGGER than
        // the resync gap is read as "the commander restarted at seq 0", not
        // "ancient packet", so a restarted sender is not ignored forever.
        ++rejected_;
        return false;
    }

    last_seq_ = pkt.seq;
    have_seq_ = true;
    last_rx_ms_ = now_ms;
    have_rx_ = true;

    if (pkt.estop()) {
        estop_ = true;
    } else if (estop_ && !pkt.enabled()) {
        // Leaving E-stop requires passing through a disabled command, so a
        // released dead-man cannot re-arm straight back into motion.
        estop_ = false;
    }

    if (pkt.enabled()) {
        clampToEnvelope(pkt.vx, pkt.wz, cmd_vx_, cmd_wz_);
    } else {
        cmd_vx_ = 0.0f;
        cmd_wz_ = 0.0f;
    }
    return true;
}

LinkState Watchdog::state(float now_ms) const {
    if (estop_) return LinkState::kEstop;
    if (!have_rx_) {
        // Never heard from anyone: stand, do not relax into a heap. The robot
        // may be powered up and already on its feet.
        return LinkState::kStand;
    }
    const float age = now_ms - last_rx_ms_;
    if (age >= relax_ms_) return LinkState::kRelax;
    if (age >= stale_ms_) return LinkState::kStand;
    return LinkState::kLive;
}

void Watchdog::command(float now_ms, float& vx, float& wz) const {
    if (state(now_ms) == LinkState::kLive) {
        vx = cmd_vx_;
        wz = cmd_wz_;
    } else {
        vx = 0.0f;
        wz = 0.0f;
    }
}

bool ArmLatch::update(uint8_t flags, bool& want_armed) {
    const bool level = (flags & kFlagArm) != 0;
    const bool had = have_level_;
    const bool prev = level_;
    have_level_ = true;
    level_ = level;
    if (!had || prev == level) return false;
    want_armed = level;
    return true;
}

}  // namespace linkproto

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
        // ENABLE gates the extended channels too: disabled = a plain stand
        // (extras at trained defaults), exactly like vx/wz.
        Command clamped = pkt;
        clampExtToEnvelope(clamped);
        cmd_vy_ = clamped.vy;
        cmd_crouch_ = clamped.crouch;
        cmd_lift_ = clamped.lift;
        cmd_fdx_ = clamped.foot_dx;
        cmd_fdz_ = clamped.foot_dz;
    } else {
        cmd_vx_ = 0.0f;
        cmd_wz_ = 0.0f;
        cmd_vy_ = 0.0f;
        cmd_crouch_ = 1.0f;
        cmd_lift_ = 0.0f;
        cmd_fdx_ = 0.0f;
        cmd_fdz_ = 0.0f;
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

void Watchdog::commandExt(float now_ms, float* cmd7) const {
    // walker_env ext_cmd layout: vx, vy, wz, crouch, lift, foot_dx, foot_dz.
    // A stale link decays to the trained stand command, same as command().
    if (state(now_ms) == LinkState::kLive) {
        cmd7[0] = cmd_vx_;
        cmd7[1] = cmd_vy_;
        cmd7[2] = cmd_wz_;
        cmd7[3] = cmd_crouch_;
        cmd7[4] = cmd_lift_;
        cmd7[5] = cmd_fdx_;
        cmd7[6] = cmd_fdz_;
    } else {
        cmd7[0] = 0.0f;
        cmd7[1] = 0.0f;
        cmd7[2] = 0.0f;
        cmd7[3] = 1.0f;
        cmd7[4] = 0.0f;
        cmd7[5] = 0.0f;
        cmd7[6] = 0.0f;
    }
}

bool ArmLatch::update(uint8_t flags, bool& want_armed) {
    // cppcheck-suppress shadowFunction -- `level` shadows the level() accessor,
    // not a variable; the local name is the clearest for the extracted bit.
    const bool level = (flags & kFlagArm) != 0;
    const bool had = have_level_;
    const bool prev = level_;
    have_level_ = true;
    level_ = level;
    if (!had || prev == level) return false;
    want_armed = level;
    return true;
}

bool HomeLatch::update(uint8_t flags) {
    // cppcheck-suppress shadowFunction -- same as ArmLatch::update above.
    const bool level = (flags & kFlagHome) != 0;
    const bool had = have_level_;
    const bool prev = level_;
    have_level_ = true;
    level_ = level;
    return had && !prev && level;
}

}  // namespace linkproto

#include "railguard/railguard.h"

namespace railguard {

void Guard::update(const Sample& s) {
    have_sample_ = true;
    now_ms_ = s.now_ms;
    rail_valid_ = s.rail_valid;
    if (s.rail_valid) rail_mv_ = s.rail_mv;
    updateEstop(s);
    updateState(s);
    updateStuck(s);
}

void Guard::updateEstop(const Sample& s) {
    if (s.estop_high) {
        low_cand_ = false;
        if (!high_cand_) {
            high_cand_ = true;
            high_since_ = s.now_ms;
        }
        if (static_cast<uint32_t>(s.now_ms - high_since_) < kEstopPressMs) return;
        pressed_ = true;
        if (!latched_) {
            latched_ = true;
            drop(DropWhy::kEstopInput);
        }
        return;
    }
    high_cand_ = false;
    if (!low_cand_) {
        low_cand_ = true;
        low_since_ = s.now_ms;
    }
    if (static_cast<uint32_t>(s.now_ms - low_since_) >= kEstopReleaseMs) pressed_ = false;
}

void Guard::updateState(const Sample& s) {
    if (state_ == State::kOff) return;
    if (state_ == State::kRising) {
        if (s.rail_valid && s.rail_mv >= kRailOnMv) {
            // Two samples in a row: one reading above the line could be noise.
            if (rise_cand_) {
                state_ = State::kSettling;
                state_since_ = s.now_ms;
                lost_cand_ = false;
                return;
            }
            rise_cand_ = true;
        } else {
            rise_cand_ = false;
        }
        if (static_cast<uint32_t>(s.now_ms - state_since_) >= kRiseTimeoutMs) {
            fault_ = Fault::kRiseTimeout;
            drop(DropWhy::kRiseTimeout);
        }
        return;
    }
    if (s.rail_valid) {
        if (s.rail_mv >= kRailLostMv) {
            lost_cand_ = false;
        } else {
            if (!lost_cand_) {
                lost_cand_ = true;
                lost_since_ = s.now_ms;
            }
            if (static_cast<uint32_t>(s.now_ms - lost_since_) >= kRailLostMs) {
                fault_ = Fault::kRailLost;
                drop(DropWhy::kRailLost);
                return;
            }
        }
    }
    if (state_ == State::kSettling &&
        static_cast<uint32_t>(s.now_ms - state_since_) >= kServoBootMs) {
        state_ = State::kOn;
    }
}

void Guard::updateStuck(const Sample& s) {
    if (servoEn() && !s.estop_high) {
        stuck_cand_ = false;
        return;
    }
    if (!s.rail_valid) return;                 // no reading: hold, do not count
    if (s.rail_mv < kRailOnMv) {
        stuck_cand_ = false;
        return;
    }
    if (fault_ == Fault::kStuck) return;
    if (!stuck_cand_) {
        stuck_cand_ = true;
        stuck_low_mv_ = s.rail_mv;
        stuck_since_ = s.now_ms;
        return;
    }
    if (static_cast<uint32_t>(s.rail_mv) + kStuckDecayMv <= stuck_low_mv_) {
        stuck_low_mv_ = s.rail_mv;             // still falling: not welded
        stuck_since_ = s.now_ms;
        return;
    }
    if (static_cast<uint32_t>(s.now_ms - stuck_since_) >= kStuckMs) fault_ = Fault::kStuck;
}

Verdict Guard::requestOn() {
    if (state_ != State::kOff) return Verdict::kOk;
    if (!have_sample_) return Verdict::kNoReading;
    if (pressed_) return Verdict::kEstopPressed;
    if (latched_) return Verdict::kEstopLatched;
    if (fault_ == Fault::kStuck) return Verdict::kStuckFault;
    if (!rail_valid_) return Verdict::kNoReading;
    if (rail_mv_ >= kRailOnMv) return Verdict::kRailStillUp;
    fault_ = Fault::kNone;
    state_ = State::kRising;
    state_since_ = now_ms_;
    rise_cand_ = false;
    lost_cand_ = false;
    return Verdict::kOk;
}

void Guard::drop(DropWhy why) {
    if (state_ == State::kOff) return;
    state_ = State::kOff;
    last_drop_ = why;
    rise_cand_ = false;
    lost_cand_ = false;
}

void Guard::disabledCommand() {
    if (latched_ && !pressed_) latched_ = false;
}

bool Guard::clearFault() {
    if (fault_ == Fault::kStuck && (!rail_valid_ || rail_mv_ >= kRailOnMv)) return false;
    fault_ = Fault::kNone;
    stuck_cand_ = false;
    return true;
}

const char* stateName(State s) {
    switch (s) {
        case State::kOff: return "OFF";
        case State::kRising: return "RISING";
        case State::kSettling: return "SETTLING";
        case State::kOn: return "ON";
    }
    return "?";
}

const char* dropName(DropWhy w) {
    switch (w) {
        case DropWhy::kNone: return "none";
        case DropWhy::kDisarm: return "disarm";
        case DropWhy::kOperatorStop: return "operator STOP";
        case DropWhy::kLinkRelax: return "link RELAX";
        case DropWhy::kLinkEstop: return "link ESTOP";
        case DropWhy::kBattSafe: return "pack guard SAFE";
        case DropWhy::kFall: return "fall";
        case DropWhy::kEstopInput: return "E-stop pressed";
        case DropWhy::kRiseTimeout: return "no rail after SERVO_EN";
        case DropWhy::kRailLost: return "rail collapsed";
    }
    return "?";
}

const char* faultName(Fault f) {
    switch (f) {
        case Fault::kNone: return "none";
        case Fault::kRiseTimeout: return "NO RAIL (SERVO_EN high, RAIL_V never rose)";
        case Fault::kRailLost: return "RAIL LOST (RAIL_V collapsed while on)";
        case Fault::kStuck: return "STUCK RAIL (RAIL_V up with SERVO_EN low: K1 welded?)";
    }
    return "?";
}

const char* verdictName(Verdict v) {
    switch (v) {
        case Verdict::kOk: return "ok";
        case Verdict::kEstopPressed: return "the E-stop is pressed (or its lead is out, or F2 blown)";
        case Verdict::kEstopLatched: return "the E-stop latch stands: a disarm first (`bench`, or ARM off)";
        case Verdict::kStuckFault: return "STUCK RAIL fault: K1 may be welded (`rail`, `rail clear`)";
        case Verdict::kRailStillUp: return "the rail is still discharging: try again in a few seconds";
        case Verdict::kNoReading: return "no RAIL_V reading";
    }
    return "?";
}

}  // namespace railguard

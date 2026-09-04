// Pack under-voltage supervisor -- the only over-discharge protection on this
// robot.
//
// A Tattu-class RC LiPo has NO protection circuit inside it (unlike a
// protected 18650), and the STS3215's own under-voltage flag trips near its
// ~6 V operating floor -- far below the ~9 V (3.0 V/cell) where a 3S pack
// starts taking permanent damage. So nothing in the hardware path stops a
// discharge: not the pack, not the servos, and not the driver board, whose
// servo V+ is a bare passthrough with no fuse or e-fuse anywhere on it
// (docs/wiring.md "Power path"). This class is that missing protection.
//
// Pure, exactly like linkproto::Watchdog: the clock arrives as now_ms on every
// call rather than being read from a timer, so the landing ramp is tested
// deterministically on the host instead of slept through.
#pragma once
#include <stdint.h>

namespace battguard {

// Thresholds in DECIVOLTS -- the resolution of the servos' register 62, which
// is the only pack-voltage sense on this robot (the driver board has no ADC
// divider; see docs/wiring.md). Per-cell figures are the standard LiPo ones:
// 3.5 V/cell is "land now", 3.3 V/cell is the last point before the knee.
constexpr uint8_t kWarn3S = 105;   // 10.5 V = 3.50 V/cell
constexpr uint8_t kLand3S = 99;    //  9.9 V = 3.30 V/cell
constexpr uint8_t kWarn2S = 70;    //  7.0 V
constexpr uint8_t kLand2S = 66;    //  6.6 V

// A trip needs the reading to STAY low, not merely dip: the current budget has
// 6.8 A millisecond gait transients against a 1.4 A RMS draw (docs/wiring.md),
// and that step sags the rail through the pack leads and the barrel jack. An
// instantaneous comparator would false-fire mid-stride. 25 ticks == 0.5 s at
// the 50 Hz control rate.
constexpr int kConfirmTicks = 25;

// How long the crouch ramp takes before torque is released. Long enough to
// reach the pose under control, short enough that it is not "carry on walking
// on a flat pack".
constexpr float kLandMs = 1500.0f;

// walker_env's crouch_range[0]: the LOWEST commanded height the deployed
// policy was trained on. Landing rides the bottom of the trained
// distribution -- it does not invent an out-of-distribution pose while the
// pack is already sagging.
constexpr float kCrouchFloor = 0.60f;

enum class Level : uint8_t {
    kOk = 0,
    kWarn = 1,       // below the land-it-yourself line; still fully commanded
    kLanding = 2,    // confirmed flat: crouching under control, then limp
    kSafe = 3,       // latched, torque off, and it stays that way
};

class Guard {
  public:
    explicit Guard(uint8_t warn_dv = kWarn3S, uint8_t land_dv = kLand3S,
                   int confirm_ticks = kConfirmTicks, float land_ms = kLandMs)
        : warn_dv_(warn_dv), land_dv_(land_dv),
          confirm_(confirm_ticks), land_ms_(land_ms) {}

    // Mirrors Watchdog::valid(): no exceptions here, so a nonsense
    // configuration is checked rather than thrown.
    bool valid() const {
        return land_dv_ <= warn_dv_ && confirm_ > 0 && land_ms_ > 0.0f;
    }

    // One control tick. `dv` is the freshest pack reading in decivolts, or 0
    // when no servo answered this tick -- 0 means "no data", NEVER "0 volts",
    // which is the difference between riding out a dropped bus frame and
    // sitting the robot down every time one arrives late.
    void update(uint8_t dv, float now_ms);

    Level level() const { return level_; }

    // True once the pack has tripped, through both the landing and the limp
    // that follows. Never clears on its own -- see the note on update().
    bool latched() const {
        return level_ == Level::kLanding || level_ == Level::kSafe;
    }

    // True once the robot must be limp and must stay limp.
    bool torqueMustRelease() const { return level_ == Level::kSafe; }

    // The crouch-height command (walker_env ext channel 3) for this tick:
    // 1.0 = full height, ramping to kCrouchFloor across the landing.
    float crouch() const { return crouch_; }

    // Last real reading, decivolts; 0 until a servo has answered once.
    uint8_t lastDv() const { return last_dv_; }

    // Consecutive ticks at or below the land threshold. For `batt` on the CLI.
    int belowTicks() const { return below_land_; }

    uint8_t warnDv() const { return warn_dv_; }
    uint8_t landDv() const { return land_dv_; }

    // Clear a latched trip. BENCH ONLY, after a pack swap: calling this on a
    // live robot re-engages torque on a flat pack.
    void reset();

  private:
    uint8_t warn_dv_;
    uint8_t land_dv_;
    int confirm_;
    float land_ms_;

    Level level_ = Level::kOk;
    float crouch_ = 1.0f;
    float land_started_ms_ = 0.0f;
    uint8_t last_dv_ = 0;
    int below_warn_ = 0;
    int below_land_ = 0;
};

}  // namespace battguard

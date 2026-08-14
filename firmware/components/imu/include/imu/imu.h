// IMU interface + mounting-offset maths.
//
// The part is the QMI8658C ON the General Driver board (I2C 0x6B on GPIO
// 32/33 -- board.h), not the external GY-BNO085 breakout the earlier design
// assumed. That swap matters more than a part number: the BNO085 was a
// sensor HUB that fused on-chip and handed us a quaternion, and the QMI8658C
// is raw, so the fusion runs here. See fusion.h.
//
//   StubImu     always level, zero rates -- lets the whole control loop, the
//               obs assembler and the CLI be exercised with no part fitted
//   Qmi8658Imu  I2C + host-side fusion (qmi8658.h; ESP-only, needs a bus)
//
// The BNO085 remains the documented upgrade path onto header P1 -- 0x4A/0x4B
// are reserved for it (docs/sensor-expansion.md) -- so this interface stays
// the seam and neither driver is privileged.
//
// The mounting rotation is applied here rather than in the obs assembler
// because it is a property of how the board is screwed to the torso, and the
// Open Duck runtime does the same at deploy. It is pure maths, so it is
// host-tested.
#pragma once
#include <stdint.h>

namespace imu {

struct Sample {
    // The torso's OWN z-axis expressed in the WORLD frame -- the third COLUMN
    // of the body->world rotation. Upright is (0,0,1).
    //
    // Read that twice: it is NOT gravity in the body frame. The two are
    // transposes of each other and they are easy to confuse, because they
    // agree at every upright pose and differ only once tilted -- under a pure
    // roll they differ by the SIGN of the y component, so getting it backwards
    // yields a robot that falls consistently to one side and a filter that
    // looks correct on the bench.
    //
    // The authority is the sim, because the policy was trained on it:
    // bimo_biped_v5body.xml declares <framezaxis objtype="site" objname="imu">
    // and MuJoCo's framezaxis reports the frame's z-axis in the GLOBAL frame.
    // Verified against MuJoCo directly, not read off the docs.
    //
    // Consequence, and it is a sharp one: this vector is YAW-DEPENDENT. Roll
    // 20 deg gives (0, -0.342, 0.940); the same roll at yaw +90 deg gives
    // (+0.342, 0, 0.940). Heading is not observable to a 6-axis IMU, so the
    // fusion has to dead-reckon yaw and it will drift. fusion.h explains what
    // that costs and what the yaw-invariant alternative would be.
    float up[3];
    float gyro[3];      // rad/s, body frame
    uint64_t t_us;      // timestamp of the report
    bool valid;
};

// Unit quaternion (w, x, y, z) describing how the sensor is mounted relative
// to the robot body frame (+x forward, +z up).
struct Mount {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

// Rotate a body-frame-relative vector by the mounting quaternion.
void applyMount(const Mount& m, const float in[3], float out[3]);

// `Sample::up` from a body->world rotation quaternion: the third COLUMN of
// R(q), i.e. the body's z-axis expressed in the world frame. Matches MuJoCo's
// framezaxis sensor. See the long note on Sample::up before touching this --
// the transpose is a different vector and a silent, one-sided failure.
void upFromQuaternion(float qw, float qx, float qy, float qz, float out[3]);

// The yaw-INVARIANT alternative: the world +z axis expressed in the body
// frame (the third ROW of R(q)) -- "projected gravity", what most locomotion
// stacks feed their policies. Not what our sim currently emits, so it is not
// what the deployed policy expects; provided so the swap is a one-line change
// in the driver if the sim ever adopts it. See fusion.h.
void projectedGravityFromQuaternion(float qw, float qx, float qy, float qz,
                                    float out[3]);

class Imu {
  public:
    virtual ~Imu() = default;
    virtual bool init() = 0;
    // Non-blocking: returns the most recent fused report. false = no new data
    // (the caller reuses the previous sample; at 400 Hz reporting vs a 50 Hz
    // loop this should never happen twice in a row).
    virtual bool read(Sample& out) = 0;
    virtual const char* name() const = 0;

    void setMount(const Mount& m) { mount_ = m; }
    const Mount& mount() const { return mount_; }

  protected:
    Mount mount_;
};

// Always-level stand-in. Not a no-op: it reports valid=true so the control
// loop and CLI run end to end without the part, which is the whole point.
class StubImu : public Imu {
  public:
    bool init() override { return true; }
    bool read(Sample& out) override;
    const char* name() const override { return "stub"; }

    // Let a test or the CLI drive it.
    void set(const float up[3], const float gyro[3]);

  private:
    float up_[3] = {0.0f, 0.0f, 1.0f};
    float gyro_[3] = {0.0f, 0.0f, 0.0f};
    uint64_t t_us_ = 0;
};

}  // namespace imu

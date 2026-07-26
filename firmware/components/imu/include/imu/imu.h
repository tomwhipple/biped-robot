// IMU interface + mounting-offset maths.
//
// The part (GY-BNO085, SH-2 sensor hub, I2C 0x4A/0x4B -- firmware-design
// section 3) may not be in hand, so the interface is the deliverable and the
// concrete drivers plug in behind it:
//
//   StubImu     always level, zero rates -- lets the whole control loop, the
//               obs assembler and the CLI be exercised with no part fitted
//   Bno085Imu   SH-2 over I2C (v2 seam; see firmware/README.md)
//
// The mounting rotation is applied here rather than in the obs assembler
// because it is a property of how the board is screwed to the torso, and the
// Open Duck runtime does the same at deploy. It is pure maths, so it is
// host-tested.
//
// One correction to docs/wiring.md: that doc says the IMU shares the ESP32's
// I2C bus with the OLED, "GPIO 21 SDA / 22 SCL". The pins are right, but there
// is no IMU on the Waveshare board -- the SSD1306 is the only device on that
// bus and the BNO085 is an external breakout wired to the same two pins. Also
// wiring.md still describes a BNO055 at 0x28; firmware-design section 3
// supersedes it with the BNO085 at 0x4A/0x4B.
#pragma once
#include <stdint.h>

namespace imu {

struct Sample {
    float up[3];        // torso up-vector in the BODY frame; (0,0,1) = upright
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

// Gravity/up-vector from a rotation quaternion: the third row of R(q)^T, i.e.
// the world +z axis expressed in the body frame.
void upFromQuaternion(float qw, float qx, float qy, float qz, float out[3]);

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

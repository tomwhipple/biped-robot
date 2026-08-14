// QMI8658C driver -- the 6-axis IMU on the General Driver board.
//
// ESP-only: this is the file with I2C in it, which is why it is separated
// from imu.cpp and fusion.cpp. Those two are pure and run on the host under
// sanitizers; this one cannot, and the seam is deliberate (firmware-design
// section 7). The interesting half of the work -- the attitude filter -- is
// on the pure side, so it is tested without hardware.
//
//
// TWO WAYS TO READ THIS CHIP, and the difference matters
//
// The obvious one is raw: sample accelerometer and gyroscope registers at the
// tick rate and integrate. At 50 Hz against a walking biped that aliases --
// the torso's angular rate through a footfall has real energy well above
// 25 Hz, and what a 50 Hz sampler makes of it is not what happened.
//
// The other is the AttitudeEngine: an on-chip vector DSP that integrates at
// 1 kHz with full coning and sculling compensation and hands back a
// quaternion INCREMENT over the interval. That is strictly more information
// than a 50 Hz sample of the same motion, and it costs the same single I2C
// burst.
//
// The catch is the rate. AE output rates are powers of two (1-64 Hz), so
// there is no 50 Hz setting to align with our tick. Motion on Demand is the
// way out: it lets the host ASK for the increment accumulated so far, at
// whatever instant it likes, which is exactly a 20 ms tick boundary.
//
// The second catch is that our datasheet copy (QST rev 0.6, mirrored in
// docs/datasheets/general-driver/) is stamped ADVANCE INFORMATION and is
// internally inconsistent about where MoD leaves its result -- the prose says
// registers 0x25-0x3D, the register map puts dQ/dV at 0x49-0x56. So AE is a
// bench question, not a paper one. `probe()` answers it, the driver falls
// back to the raw path on its own, and neither path changes the observation
// the policy sees.
#pragma once
#include <stdint.h>

#include "driver/gpio.h"
#include "imu/fusion.h"
#include "imu/imu.h"

namespace imu {

class Qmi8658Imu : public Imu {
  public:
    struct Config {
        gpio_num_t sda = GPIO_NUM_32;
        gpio_num_t scl = GPIO_NUM_33;
        uint32_t hz = 400000;
        uint8_t addr = 0x6B;

        // Try the AttitudeEngine first and fall back to raw if it does not
        // answer. Set false to force the raw path.
        bool try_attitude_engine = true;

        // Full scales. +/-8 g and +/-1024 dps: a walking torso lives well
        // inside both, and a fall does not clip. Clipping the gravity vector
        // would be worse than noise -- it biases rather than blurs.
        uint8_t accel_fs_g = 8;
        uint16_t gyro_fs_dps = 1024;
    };

    // What the bench needs to know, filled by probe().
    struct Probe {
        bool present = false;
        uint8_t who_am_i = 0;      // expect 0x05
        uint8_t revision = 0;      // expect 0x79
        bool attitude_engine = false;
        uint8_t addr_found = 0;
    };

    explicit Qmi8658Imu(const Config& cfg) : cfg_(cfg) {}

    bool init() override;
    bool read(Sample& out) override;
    const char* name() const override {
        return use_ae_ ? "qmi8658c/ae" : "qmi8658c/raw";
    }

    // -- bench helpers, not used by the control loop -----------------------

    // Bring the bus up (if it is not already) and identify the part. Safe to
    // call before init().
    bool probe(Probe& out);

    // Every address that ACKs on the bus, for `imu scan`. Returns the count;
    // writes up to `max` addresses.
    int scanBus(uint8_t* found, int max);

    // Raw sensor-frame readings, mount NOT applied: m/s^2 and rad/s. This is
    // the call that tells you which physical axis is which, which is the
    // whole job of bring-up.
    bool readRaw(float accel[3], float gyro[3]);

    // Read the AttitudeEngine increment directly (unit quaternion, m/s).
    // False if AE is not running or produced nothing.
    bool readIncrement(float dq[4], float dv[3]);

    // Gyro zero-rate bias, in the SENSOR frame. Hold the robot still.
    // Blocking: BiasEstimator::kSamples reads at the tick rate.
    bool calibrateBias(float bias_out[3], float spread_out[3]);
    void setBias(const float bias[3]);
    void bias(float out[3]) const;

    // Bench diagnostic: force the AttitudeEngine on and report what the dQ/dV
    // registers actually contain. This exists because the datasheet copy we
    // have is ADVANCE INFORMATION and contradicts itself about MoD, so "does
    // AE work on this silicon" is a question only the part can answer.
    struct AeDiag {
        bool ctrl_written = false;
        uint8_t ctrl6 = 0, ctrl7 = 0, status0 = 0;
        uint8_t dq_bytes[8] = {0};
        uint8_t dv_bytes[6] = {0};
        bool any_nonzero = false;
    };
    bool aeDiagnose(AeDiag& out);

    // Drop the attitude estimate and re-align from the next sample. Required
    // after ANY mount change: the filter's quaternion is expressed in the
    // body frame, so re-defining sensor->body silently invalidates it. The
    // estimate would eventually be dragged back by the gravity correction,
    // but "eventually" is seconds of confidently wrong attitude and there is
    // no reason to live through it.
    void realign() {
        fusion_.reset();
        aligned_ = false;
    }

    Fusion& fusion() { return fusion_; }

  private:
    bool busInit();
    bool regRead(uint8_t reg, uint8_t* buf, size_t len);
    bool regWrite(uint8_t reg, uint8_t val);
    // Sensor-frame read with NO bias subtraction. Only the bias calibration
    // wants this; everything else must go through readRaw().
    bool readRawUnbiased(float accel[3], float gyro[3]);
    bool configure();
    bool requestMotionOnDemand();

    Config cfg_;
    Fusion fusion_;
    void* bus_ = nullptr;      // i2c_master_bus_handle_t
    void* dev_ = nullptr;      // i2c_master_dev_handle_t
    bool use_ae_ = false;
    bool ready_ = false;
    bool aligned_ = false;
    float bias_[3] = {0.0f, 0.0f, 0.0f};
    float accel_scale_ = 0.0f;   // LSB -> m/s^2
    float gyro_scale_ = 0.0f;    // LSB -> rad/s
    uint64_t last_us_ = 0;
};

}  // namespace imu

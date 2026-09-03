#include "imu/qmi8658.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace imu {
namespace {

// Register map, QST rev 0.6 (docs/datasheets/general-driver/).
constexpr uint8_t kRegWhoAmI = 0x00;
constexpr uint8_t kRegRevision = 0x01;
constexpr uint8_t kRegCtrl1 = 0x02;
constexpr uint8_t kRegCtrl2 = 0x03;   // accel: aFS 6:4, aODR 3:0
constexpr uint8_t kRegCtrl3 = 0x04;   // gyro:  gFS 6:4, gODR 3:0
constexpr uint8_t kRegCtrl5 = 0x06;   // low-pass filters
constexpr uint8_t kRegCtrl6 = 0x07;   // sMoD 7, sODR 2:0
constexpr uint8_t kRegCtrl7 = 0x08;   // syncSmpl 7, sEN 3, gEN 1, aEN 0
constexpr uint8_t kRegCtrl9 = 0x0A;   // command port
constexpr uint8_t kRegStatus0 = 0x2E;
constexpr uint8_t kRegAccel = 0x35;   // AX_L .. AZ_H
constexpr uint8_t kRegGyro = 0x3B;    // GX_L .. GZ_H
constexpr uint8_t kRegDq = 0x49;      // dQW_L .. dQZ_H
constexpr uint8_t kRegDv = 0x51;      // dVX_L .. dVZ_H

constexpr uint8_t kWhoAmIExpected = 0x05;

constexpr uint8_t kCmdReqMoD = 0x0C;

// CTRL1 bit 6: serial-interface address auto-increment. Without it a burst
// read returns the same register N times, which looks exactly like a sensor
// stuck at a constant -- set it before trusting any multi-byte read.
constexpr uint8_t kCtrl1AddrAutoInc = 1 << 6;

// CTRL7 enables.
constexpr uint8_t kCtrl7aEN = 1 << 0;
constexpr uint8_t kCtrl7gEN = 1 << 1;
constexpr uint8_t kCtrl7sEN = 1 << 3;
// syncSmpl: lock the output registers while they are being read. Without
// it a 12-byte burst can straddle a sample update and one axis comes back
// with a torn 16-bit value -- measured 2026-09-03 as isolated +-256 LSB
// (exactly 8 deg/s at the 1024 dps range) spikes on gyro z, ~1 in 100.
constexpr uint8_t kCtrl7SyncSmpl = 1 << 7;

// AE quaternion is Q14: 16384 LSB per unit. dV is 1024 LSB per m/s.
constexpr float kDqScale = 1.0f / 16384.0f;
constexpr float kDvScale = 1.0f / 1024.0f;

constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = 3.14159265358979f / 180.0f;

constexpr int kOdr500Hz = 0x04;   // aODR/gODR setting for 500 Hz, normal mode

int16_t le16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

uint8_t accelFsBits(uint8_t g) {
    switch (g) {
        case 2: return 0;
        case 4: return 1;
        case 8: return 2;
        default: return 3;   // 16 g
    }
}

uint8_t gyroFsBits(uint16_t dps) {
    switch (dps) {
        case 16: return 0;
        case 32: return 1;
        case 64: return 2;
        case 128: return 3;
        case 256: return 4;
        case 512: return 5;
        case 1024: return 6;
        default: return 7;   // 2048 dps
    }
}

}  // namespace

// I2C bus recovery (2026-09-03): a reset that lands mid-transaction -- and
// with the 250 Hz sampler running, every flash does -- leaves the QMI8658C
// holding SDA low, and the next boot's scan finds NOTHING on the bus (seen
// twice tonight; the firmware then silently runs on the StubImu). The cure
// is the standard one: clock SCL until the slave releases SDA, then a STOP.
static bool busRecover(gpio_num_t sda, gpio_num_t scl) {
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    gpio_set_direction(sda, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction(scl, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(sda, 1);
    gpio_set_level(scl, 1);
    esp_rom_delay_us(20);
    bool recovered = false;
    // Up to three rounds: 32 clocks at ~25 kHz (the slave may be mid-byte
    // and the QMI8658C's I2C engine is slow to notice), then a STOP; the
    // 16-pulse single round (first version, 2026-09-03) freed the bus once
    // and failed the next time.
    for (int round = 0; round < 3 && gpio_get_level(sda) == 0; ++round) {
        for (int i = 0; i < 32 && gpio_get_level(sda) == 0; ++i) {
            gpio_set_level(scl, 0);
            esp_rom_delay_us(20);
            gpio_set_level(scl, 1);
            esp_rom_delay_us(20);
        }
        // START then STOP: SDA low while SCL high, then SDA high while SCL high
        gpio_set_level(sda, 0);
        esp_rom_delay_us(20);
        gpio_set_level(scl, 1);
        esp_rom_delay_us(20);
        gpio_set_level(sda, 1);
        esp_rom_delay_us(50);
        recovered = gpio_get_level(sda) == 1;
    }
    gpio_reset_pin(sda);
    gpio_reset_pin(scl);
    return recovered;
}

bool Qmi8658Imu::busInit() {
    if (bus_ != nullptr) return true;
    if (busRecover(cfg_.sda, cfg_.scl)) {
        ESP_LOGW("imu", "I2C bus was held low (SDA stuck) -- recovered by clocking it out");
    }

    i2c_master_bus_config_t bc = {};
    bc.i2c_port = I2C_NUM_0;
    bc.sda_io_num = cfg_.sda;
    bc.scl_io_num = cfg_.scl;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    // The board carries its own pull-ups on this bus (and an LSF0204PWR in
    // the IMU's path), so the internal ones stay off -- doubling them up
    // slows the rise time, which is the opposite of helpful at 400 kHz.
    bc.flags.enable_internal_pullup = false;

    i2c_master_bus_handle_t bus = nullptr;
    if (i2c_new_master_bus(&bc, &bus) != ESP_OK) return false;
    bus_ = bus;

    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address = cfg_.addr;
    dc.scl_speed_hz = cfg_.hz;

    i2c_master_dev_handle_t dev = nullptr;
    if (i2c_master_bus_add_device(bus, &dc, &dev) != ESP_OK) return false;
    dev_ = dev;
    return true;
}

// One mutex per device: the 250 Hz sampler task (imu_sampler.cpp) and the
// bench CLI (`imu raw`, `imu bias`) share this bus from different cores.
static SemaphoreHandle_t s_i2c_lock = nullptr;
static void lockInit() {
    if (s_i2c_lock == nullptr) s_i2c_lock = xSemaphoreCreateMutex();
}

bool Qmi8658Imu::regRead(uint8_t reg, uint8_t* buf, size_t len) {
    if (dev_ == nullptr) return false;
    lockInit();
    if (xSemaphoreTake(s_i2c_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    const bool ok = i2c_master_transmit_receive(
               static_cast<i2c_master_dev_handle_t>(dev_), &reg, 1, buf, len,
               100) == ESP_OK;
    xSemaphoreGive(s_i2c_lock);
    return ok;
}

bool Qmi8658Imu::regWrite(uint8_t reg, uint8_t val) {
    if (dev_ == nullptr) return false;
    lockInit();
    if (xSemaphoreTake(s_i2c_lock, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    const bool ok = [&]() {
    const uint8_t tx[2] = {reg, val};
        return i2c_master_transmit(static_cast<i2c_master_dev_handle_t>(dev_), tx,
                               2, 100) == ESP_OK;
    }();
    xSemaphoreGive(s_i2c_lock);
    return ok;
}

int Qmi8658Imu::scanBus(uint8_t* found, int max) {
    if (!busInit()) return 0;
    int n = 0;
    for (uint8_t a = 0x08; a < 0x78 && n < max; ++a) {
        if (i2c_master_probe(static_cast<i2c_master_bus_handle_t>(bus_), a,
                             50) == ESP_OK) {
            found[n++] = a;
        }
    }
    return n;
}

bool Qmi8658Imu::probe(Probe& out) {
    out = Probe{};
    if (!busInit()) return false;

    uint8_t id = 0;
    if (!regRead(kRegWhoAmI, &id, 1)) return false;
    out.who_am_i = id;
    out.addr_found = cfg_.addr;
    out.present = (id == kWhoAmIExpected);
    if (!out.present) return false;

    regRead(kRegRevision, &out.revision, 1);
    out.attitude_engine = use_ae_;
    return true;
}

bool Qmi8658Imu::configure() {
    // Auto-increment first: every burst below depends on it.
    if (!regWrite(kRegCtrl1, kCtrl1AddrAutoInc)) return false;

    const uint8_t a = static_cast<uint8_t>(
        (accelFsBits(cfg_.accel_fs_g) << 4) | kOdr500Hz);
    const uint8_t g = static_cast<uint8_t>(
        (gyroFsBits(cfg_.gyro_fs_dps) << 4) | kOdr500Hz);
    if (!regWrite(kRegCtrl2, a)) return false;
    if (!regWrite(kRegCtrl3, g)) return false;

    // No on-chip low-pass. The AttitudeEngine wants the full-rate signal, and
    // on the raw path the complementary filter IS the low-pass -- stacking a
    // second one only adds phase lag, and phase lag in the up-vector is a lie
    // about where the torso is.
    regWrite(kRegCtrl5, 0x00);

    accel_scale_ = (static_cast<float>(cfg_.accel_fs_g) * kGravity) / 32768.0f;
    gyro_scale_ =
        (static_cast<float>(cfg_.gyro_fs_dps) * kDegToRad) / 32768.0f;

    uint8_t ctrl7 = kCtrl7aEN | kCtrl7gEN | kCtrl7SyncSmpl;
    use_ae_ = false;

    if (cfg_.try_attitude_engine) {
        // AE needs all three enables; MoD then needs sMoD in CTRL6.
        if (regWrite(kRegCtrl7, static_cast<uint8_t>(ctrl7 | kCtrl7sEN)) &&
            regWrite(kRegCtrl6, 0x80)) {
            vTaskDelay(pdMS_TO_TICKS(50));
            float dq[4], dv[3];
            // Two attempts: the first increment after enabling is allowed to
            // be empty while the engine spins up.
            requestMotionOnDemand();
            vTaskDelay(pdMS_TO_TICKS(30));
            if (readIncrement(dq, dv)) {
                use_ae_ = true;
            }
        }
    }

    if (!use_ae_) {
        if (!regWrite(kRegCtrl6, 0x00)) return false;
        if (!regWrite(kRegCtrl7, ctrl7)) return false;
    }

    vTaskDelay(pdMS_TO_TICKS(20));
    return true;
}

bool Qmi8658Imu::reinit() {
    ready_ = false;
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(static_cast<i2c_master_dev_handle_t>(dev_));
        dev_ = nullptr;
    }
    if (bus_ != nullptr) {
        i2c_del_master_bus(static_cast<i2c_master_bus_handle_t>(bus_));
        bus_ = nullptr;
    }
    return init();
}

bool Qmi8658Imu::init() {
    if (!busInit()) return false;

    Probe p;
    if (!probe(p) || !p.present) return false;
    if (!configure()) return false;

    fusion_.reset();
    last_us_ = static_cast<uint64_t>(esp_timer_get_time());
    ready_ = true;
    return true;
}

bool Qmi8658Imu::requestMotionOnDemand() {
    return regWrite(kRegCtrl9, kCmdReqMoD);
}

bool Qmi8658Imu::readIncrement(float dq[4], float dv[3]) {
    uint8_t buf[8];
    if (!regRead(kRegDq, buf, sizeof buf)) return false;
    float q[4];
    for (int i = 0; i < 4; ++i) {
        q[i] = static_cast<float>(le16(&buf[i * 2])) * kDqScale;
    }
    // An all-zero increment is not a valid unit quaternion -- it is the
    // engine saying "nothing here". Treat it as no-data rather than as a
    // rotation, which is what feeding it forward would amount to.
    const float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] +
                          q[3] * q[3]);
    if (n < 0.5f) return false;
    const float inv = 1.0f / n;
    for (int i = 0; i < 4; ++i) dq[i] = q[i] * inv;

    uint8_t vbuf[6];
    if (!regRead(kRegDv, vbuf, sizeof vbuf)) return false;
    for (int i = 0; i < 3; ++i) {
        dv[i] = static_cast<float>(le16(&vbuf[i * 2])) * kDvScale;
    }
    return true;
}

bool Qmi8658Imu::aeDiagnose(AeDiag& out) {
    out = AeDiag{};
    if (dev_ == nullptr) return false;

    const uint8_t ctrl7 = kCtrl7aEN | kCtrl7gEN | kCtrl7sEN;
    out.ctrl_written = regWrite(kRegCtrl7, ctrl7) && regWrite(kRegCtrl6, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));
    requestMotionOnDemand();
    vTaskDelay(pdMS_TO_TICKS(50));

    regRead(kRegCtrl6, &out.ctrl6, 1);
    regRead(kRegCtrl7, &out.ctrl7, 1);
    regRead(kRegStatus0, &out.status0, 1);
    regRead(kRegDq, out.dq_bytes, sizeof out.dq_bytes);
    regRead(kRegDv, out.dv_bytes, sizeof out.dv_bytes);

    for (size_t i = 0; i < sizeof out.dq_bytes; ++i) {
        if (out.dq_bytes[i]) out.any_nonzero = true;
    }
    for (size_t i = 0; i < sizeof out.dv_bytes; ++i) {
        if (out.dv_bytes[i]) out.any_nonzero = true;
    }

    // Put the part back the way the control loop expects it.
    if (!use_ae_) {
        regWrite(kRegCtrl6, 0x00);
        regWrite(kRegCtrl7, static_cast<uint8_t>(kCtrl7aEN | kCtrl7gEN));
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return true;
}

bool Qmi8658Imu::readRawUnbiased(float accel[3], float gyro[3]) {
    // Accel and gyro are contiguous (0x35..0x40), so one burst gets both.
    //
    // TORN READS (2026-09-03): a burst can straddle a sample update and one
    // axis comes back with mismatched low/high bytes -- measured as isolated
    // +-256 LSB spikes (exactly 8 deg/s at 1024 dps) on gyro z, ~1 in 100
    // samples, and syncSmpl did not stop them. So: read twice; if any axis
    // disagrees by more than kTearLsb, read a third time and take the
    // per-axis median. Two clean reads 0.4 ms apart agree to a few LSB even
    // in motion; a tear is an outlier by hundreds.
    constexpr int kTearLsb = 96;
    uint8_t b1[12], b2[12], b3[12];
    if (!regRead(kRegAccel, b1, sizeof b1)) return false;
    if (!regRead(kRegAccel, b2, sizeof b2)) return false;
    int16_t v1[6], v2[6], v3[6];
    bool torn = false;
    for (int i = 0; i < 6; ++i) {
        v1[i] = le16(&b1[i * 2]);
        v2[i] = le16(&b2[i * 2]);
        const int d = static_cast<int>(v1[i]) - static_cast<int>(v2[i]);
        if (d > kTearLsb || d < -kTearLsb) torn = true;
    }
    int16_t* use = v2;
    if (torn) {
        if (!regRead(kRegAccel, b3, sizeof b3)) return false;
        for (int i = 0; i < 6; ++i) {
            v3[i] = le16(&b3[i * 2]);
            // median of three
            const int16_t a = v1[i], b = v2[i], c = v3[i];
            v2[i] = (a > b) ? ((b > c) ? b : (a > c ? c : a))
                            : ((a > c) ? a : (b > c ? c : b));
        }
        ++tear_count_;
    }
    for (int i = 0; i < 3; ++i) {
        accel[i] = static_cast<float>(use[i]) * accel_scale_;
        gyro[i] = static_cast<float>(use[3 + i]) * gyro_scale_;
    }
    return true;
}

bool Qmi8658Imu::readRaw(float accel[3], float gyro[3]) {
    if (!readRawUnbiased(accel, gyro)) return false;
    for (int i = 0; i < 3; ++i) gyro[i] -= bias_[i];
    return true;
}

void Qmi8658Imu::setBias(const float b[3]) {
    for (int i = 0; i < 3; ++i) bias_[i] = b[i];
}

void Qmi8658Imu::bias(float out[3]) const {
    for (int i = 0; i < 3; ++i) out[i] = bias_[i];
}

bool Qmi8658Imu::calibrateBias(float bias_out[3], float spread_out[3]) {
    BiasEstimator est;
    est.reset();

    // Read the gyro with no bias applied -- but WITHOUT zeroing bias_ to do
    // it. An earlier version zeroed the member for the duration, which was a
    // cross-task bug: the control task calls read() at 50 Hz throughout, so
    // for the two seconds of calibration the fusion integrated a completely
    // uncorrected gyro. At the ~0.44 rad/s this part actually offsets by,
    // that is ~50 degrees of fictitious rotation injected into the attitude
    // estimate -- which looked exactly like the robot slumping backwards, and
    // was diagnosed as such twice before the numbers gave it away.
    bool ok = true;
    for (int i = 0; i < BiasEstimator::kSamples; ++i) {
        float a[3], g[3];
        if (!readRawUnbiased(a, g)) {
            ok = false;
            break;
        }
        est.accumulate(g);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    if (!ok) return false;

    est.bias(bias_out);
    est.spread(spread_out);
    setBias(bias_out);

    // The estimate accumulated under the OLD bias, so drop it and re-align
    // from gravity rather than carrying that error forward.
    realign();
    return true;
}

bool Qmi8658Imu::read(Sample& out) {
    if (!ready_) return false;

    float accel_s[3], gyro_s[3];
    if (!readRaw(accel_s, gyro_s)) return false;

    // Sensor frame -> body frame. Everything downstream, the filter included,
    // is body frame; this is the only place the mounting rotation appears.
    float accel_b[3], gyro_b[3];
    applyMount(mount_, accel_s, accel_b);
    applyMount(mount_, gyro_s, gyro_b);

    const uint64_t now = static_cast<uint64_t>(esp_timer_get_time());
    float dt = static_cast<float>(now - last_us_) * 1e-6f;
    last_us_ = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;   // first tick, or a stall

    // First valid sample sets the attitude outright; see Fusion::alignTo.
    if (!aligned_) {
        fusion_.alignTo(accel_b);
        aligned_ = true;
    }

    if (use_ae_) {
        float dq_s[4], dv_s[3];
        requestMotionOnDemand();
        if (readIncrement(dq_s, dv_s)) {
            // Rotate the increment's vector part into the body frame; the
            // scalar part is frame-independent.
            float axis_b[3];
            const float axis_s[3] = {dq_s[1], dq_s[2], dq_s[3]};
            applyMount(mount_, axis_s, axis_b);
            const float dq_b[4] = {dq_s[0], axis_b[0], axis_b[1], axis_b[2]};
            fusion_.updateDeltaQuat(dq_b, accel_b);
        } else {
            fusion_.updateRate(gyro_b, accel_b, dt);
        }
    } else {
        fusion_.updateRate(gyro_b, accel_b, dt);
    }

    fusion_.up(out.up);
    // The obs wants an instantaneous body rate, which is what the sim's
    // qvel[3:6] is -- so the gyro channel comes from the raw register even on
    // the AE path, where dQ would only give the interval average.
    for (int i = 0; i < 3; ++i) out.gyro[i] = gyro_b[i];
    out.t_us = now;
    out.valid = true;
    return true;
}

}  // namespace imu

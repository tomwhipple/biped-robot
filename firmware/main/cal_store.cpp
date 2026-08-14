#include "cal_store.h"

#include <stddef.h>
#include <string.h>

#include "asbuilt_cal.h"

#ifdef ESP_PLATFORM
#include "nvs.h"
#include "nvs_flash.h"
#endif

namespace robot {
#ifdef ESP_PLATFORM
namespace {
constexpr const char* kNs = "bimo";
constexpr const char* kKey = "cal";
constexpr const char* kImuKey = "imucal";
}  // namespace
#endif

uint32_t calCrc32(const void* data, size_t len) {
    // Bitwise CRC-32 (IEEE, reflected). No table: this runs twice per boot at
    // most, and a 1 KB table is not worth the .rodata in a no-heap firmware.
    const uint8_t* p = static_cast<const uint8_t*>(data);
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
        }
    }
    return ~crc;
}

void calPack(const obs::Calibration& cal, CalBlob& out) {
    memset(&out, 0, sizeof out);
    out.magic = kCalMagic;
    out.version = kCalVersion;
    out.joints = static_cast<uint16_t>(obs::kNumJoints);
    for (int i = 0; i < obs::kNumJoints; ++i) {
        out.zero_steps[i] = cal.zero_steps[i];
        out.dir[i] = cal.dir[i];
        out.servo_id[i] = obs::kServoId[i];
    }
    out.crc = calCrc32(&out, sizeof out - sizeof out.crc);
}

bool calUnpack(const CalBlob& blob, obs::Calibration& out) {
    if (blob.magic != kCalMagic) return false;
    if (blob.version != kCalVersion) return false;
    if (blob.joints != static_cast<uint16_t>(obs::kNumJoints)) return false;
    if (blob.crc != calCrc32(&blob, sizeof blob - sizeof blob.crc)) return false;
    for (int i = 0; i < obs::kNumJoints; ++i) {
        // Reject implausible values rather than trusting a blob that passed
        // CRC but was written by a build with a different convention.
        if (blob.zero_steps[i] < 0 || blob.zero_steps[i] > 4095) return false;
        if (blob.dir[i] != 1 && blob.dir[i] != -1) return false;
        // The map the calibration was measured under must be THIS build's
        // map -- a joint-indexed blob under a different kServoId pairs zeros
        // with the wrong physical servos (see the v2 note in cal_store.h).
        if (blob.servo_id[i] != obs::kServoId[i]) return false;
    }
    for (int i = 0; i < obs::kNumJoints; ++i) {
        out.zero_steps[i] = blob.zero_steps[i];
        out.dir[i] = blob.dir[i];
    }
    return true;
}

bool calUnpackV1(const CalBlobV1& blob, obs::Calibration& out) {
    if (blob.magic != kCalMagic) return false;
    if (blob.version != 1) return false;
    if (blob.joints != static_cast<uint16_t>(obs::kNumJoints)) return false;
    if (blob.crc != calCrc32(&blob, sizeof blob - sizeof blob.crc)) {
        return false;
    }
    for (int i = 0; i < obs::kNumJoints; ++i) {
        if (blob.zero_steps[i] < 0 || blob.zero_steps[i] > 4095) return false;
        if (blob.dir[i] != 1 && blob.dir[i] != -1) return false;
    }
    for (int i = 0; i < obs::kNumJoints; ++i) {
        out.zero_steps[i] = blob.zero_steps[i];
        out.dir[i] = blob.dir[i];
    }
    return true;
}

bool calIsAsBuilt(const obs::Calibration& cal) {
    for (int i = 0; i < obs::kNumJoints; ++i) {
        if (cal.zero_steps[i] != kAsBuiltZeroSteps[i]) return false;
        if (cal.dir[i] != kAsBuiltDir[i]) return false;
    }
    return true;
}

#ifdef ESP_PLATFORM

bool calMigrateV1(obs::Calibration& out) {
    obs::Calibration v1;
    if (!calLoadV1(v1)) return false;
    if (!calIsAsBuilt(v1)) return false;   // unknown v1 values: recalibrate
    out = v1;
    return calSave(out);                   // re-persist as v2, map-bound
}

bool calLoadV1(obs::Calibration& out) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return false;
    CalBlobV1 blob{};
    size_t len = sizeof blob;
    const esp_err_t err = nvs_get_blob(h, kKey, &blob, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof blob) return false;
    return calUnpackV1(blob, out);
}

bool calLoad(obs::Calibration& out) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return false;
    CalBlob blob{};
    size_t len = sizeof blob;
    const esp_err_t err = nvs_get_blob(h, kKey, &blob, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof blob) return false;
    return calUnpack(blob, out);
}

bool calSave(const obs::Calibration& cal) {
    CalBlob blob{};
    calPack(cal, blob);
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t err = nvs_set_blob(h, kKey, &blob, sizeof blob);
    const esp_err_t cerr = (err == ESP_OK) ? nvs_commit(h) : err;
    nvs_close(h);
    return cerr == ESP_OK;
}

bool calErase() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t err = nvs_erase_key(h, kKey);
    const esp_err_t cerr = nvs_commit(h);
    nvs_close(h);
    return (err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND) && cerr == ESP_OK;
}

#endif  // ESP_PLATFORM



// -- IMU calibration -------------------------------------------------------

void imuCalPack(const float bias[3], const float mount[4], ImuCalBlob& out) {
    out = ImuCalBlob{};
    out.magic = kImuCalMagic;
    out.version = kImuCalVersion;
    for (int i = 0; i < 3; ++i) out.gyro_bias[i] = bias[i];
    for (int i = 0; i < 4; ++i) out.mount[i] = mount[i];
    out.crc = calCrc32(&out, offsetof(ImuCalBlob, crc));
}

bool imuCalUnpack(const ImuCalBlob& blob, float bias_out[3],
                  float mount_out[4]) {
    if (blob.magic != kImuCalMagic) return false;
    if (blob.version != kImuCalVersion) return false;
    if (blob.crc != calCrc32(&blob, offsetof(ImuCalBlob, crc))) return false;
    // A zero quaternion is not a rotation. A blob that passes CRC but carries
    // one is a bug upstream, not a calibration -- reject rather than
    // normalising it into something plausible.
    const float n = blob.mount[0] * blob.mount[0] +
                    blob.mount[1] * blob.mount[1] +
                    blob.mount[2] * blob.mount[2] +
                    blob.mount[3] * blob.mount[3];
    if (n < 0.9f || n > 1.1f) return false;
    for (int i = 0; i < 3; ++i) bias_out[i] = blob.gyro_bias[i];
    for (int i = 0; i < 4; ++i) mount_out[i] = blob.mount[i];
    return true;
}

#ifdef ESP_PLATFORM

bool imuCalLoad(float bias_out[3], float mount_out[4]) {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READONLY, &h) != ESP_OK) return false;
    ImuCalBlob blob{};
    size_t len = sizeof blob;
    const esp_err_t err = nvs_get_blob(h, kImuKey, &blob, &len);
    nvs_close(h);
    if (err != ESP_OK || len != sizeof blob) return false;
    return imuCalUnpack(blob, bias_out, mount_out);
}

bool imuCalSave(const float bias[3], const float mount[4]) {
    ImuCalBlob blob{};
    imuCalPack(bias, mount, blob);
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    const esp_err_t err = nvs_set_blob(h, kImuKey, &blob, sizeof blob);
    const esp_err_t cerr = (err == ESP_OK) ? nvs_commit(h) : err;
    nvs_close(h);
    return cerr == ESP_OK;
}

bool imuCalErase() {
    nvs_handle_t h;
    if (nvs_open(kNs, NVS_READWRITE, &h) != ESP_OK) return false;
    nvs_erase_key(h, kImuKey);
    const esp_err_t cerr = nvs_commit(h);
    nvs_close(h);
    return cerr == ESP_OK;
}

#endif  // ESP_PLATFORM

}  // namespace robot

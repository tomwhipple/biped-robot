#include "cal_store.h"

#include <string.h>

#ifdef ESP_PLATFORM
#include "nvs.h"
#include "nvs_flash.h"
#endif

namespace robot {
#ifdef ESP_PLATFORM
namespace {
constexpr const char* kNs = "bimo";
constexpr const char* kKey = "cal";
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

#ifdef ESP_PLATFORM

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

}  // namespace robot

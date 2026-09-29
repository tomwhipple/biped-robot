// Persisting servo calibration across reboots.
//
// obs::Calibration's own comment has always said "restored from NVS at boot".
// Until 2026-07-26 nothing did that: g_cal was a plain global that
// default-constructed to zero_steps = 2048, dir = +1 on every power-up, so a
// calibration performed at the CAD-neutral pose survived exactly until the
// next reset. This is the missing half.
//
// It lives in main/ rather than components/obs/ on purpose: obs/ is one of the
// pure modules that compiles and runs under sanitizers on the host
// (firmware-design section 7), and pulling nvs_flash.h in there would end
// that. The blob format below is plain data, so the host tests can still
// exercise pack/unpack without ESP-IDF.
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "obs/actuation.h"

namespace robot {

// Versioned, self-describing blob. The joint count is stored because the plant
// changed from 8 to 10 DOF, and then the bus from 10 to 17 servos: restoring a
// calibration into a build with a different joint list would silently leave
// joints at defaults, which is the kind of thing that is only noticed when a
// leg moves wrong.
//
// Every version also stores the SERVO MAP it was measured under: on
// 2026-08-02 the whole-chain leg-swap errata changed the map after an
// as-built calibration had been persisted, silently pairing 8 of 10 joints'
// zeros and directions with the WRONG physical servos. A map mismatch is
// rejected -- the robot boots uncalibrated and `run` refuses, instead of
// twisting.
//
// v3 (issue #81) is indexed by BUS joint (obs/bus_map.h: index b is servo ID
// b + 1), covers all 17 servos, and records which of them are fitted. v2 was
// indexed by the prototype's 10 policy joints; a v2 blob migrates to v3
// exactly and automatically at boot (calMigrateV2), so the robot keeps its
// calibration across the change.
constexpr int kCalPad = 4 - (3 * obs::kNumBusJoints) % 4;   // 1..4
struct CalBlob {
    uint32_t magic;          // kCalMagic
    uint16_t version;        // kCalVersion
    uint16_t joints;         // must equal obs::kNumBusJoints
    int32_t zero_steps[obs::kNumBusJoints];
    int8_t dir[obs::kNumBusJoints];
    uint8_t fitted[obs::kNumBusJoints];     // 1 = on the robot, 0 = absent
    uint8_t servo_id[obs::kNumBusJoints];   // kBusServoId at measurement time
    uint8_t pad[kCalPad];
    uint32_t crc;            // CRC-32 over everything above
};

constexpr uint32_t kCalMagic = 0x424D4331;   // "BMC1"
constexpr uint16_t kCalVersion = 3;

// Pure, host-testable: pack/unpack + integrity. Unpack returns false and
// leaves `out` untouched on any mismatch, so a corrupt or stale blob falls
// back to defaults rather than half-applying.
void calPack(const obs::Calibration& cal, CalBlob& out);
bool calUnpack(const CalBlob& blob, obs::Calibration& out);
uint32_t calCrc32(const void* data, size_t len);

// -- the prototype-era layouts ----------------------------------------------
// v1 and v2 were indexed by the 10-DOF prototype's POLICY joint order, under
// this servo map (the generated kServoId of every 10-joint run since the
// 2026-08-02 errata). It is spelled out here, not taken from obs_spec.h, so a
// future policy with a different joint list cannot change how an old blob is
// read.
inline constexpr int kLegacyJoints = 10;
inline constexpr uint8_t kLegacyServoId[kLegacyJoints] = {10, 5, 6, 7, 8,
                                                          9,  1, 2, 3, 4};

// v2: joint-indexed, with its servo map. Proves its map, so it migrates to v3
// automatically: each servo's zero and direction land on that servo's bus
// joint, the ten are marked fitted, and IDs 11-17 are marked absent.
struct CalBlobV2 {
    uint32_t magic;
    uint16_t version;        // 2
    uint16_t joints;         // 10
    int32_t zero_steps[kLegacyJoints];
    int8_t dir[kLegacyJoints];
    uint8_t servo_id[kLegacyJoints];
    uint8_t pad[3];
    uint32_t crc;
};

bool calUnpackV2(const CalBlobV2& blob, obs::Calibration& out);   // pure
bool calMigrateV2(obs::Calibration& out);                         // ESP side

// The v1 layout, kept verbatim for one purpose: an EXPLICIT operator-driven
// migration (CLI `cal migrate`). A v1 blob cannot prove which servo map it
// was measured under, so boot rejects it unless it equals the as-built table;
// otherwise a HUMAN verifies the values against docs/servo-map.md, and
// `cal save` re-saves them as v3. The machine never trusts v1 on its own.
struct CalBlobV1 {
    uint32_t magic;
    uint16_t version;        // 1
    uint16_t joints;
    int32_t zero_steps[kLegacyJoints];
    int8_t dir[kLegacyJoints];
    uint8_t pad[3];
    uint32_t crc;
};

bool calUnpackV1(const CalBlobV1& blob, obs::Calibration& out);   // pure
bool calLoadV1(obs::Calibration& out);                            // ESP side

// True iff `cal` holds the compiled as-built table (asbuilt_cal.h) exactly on
// the prototype's ten servos, all fitted. This is what lets a v1 blob migrate
// WITHOUT an operator eyeballing it: the blob cannot prove its servo map, but
// matching the table -- which was measured under the current map -- proves
// it is that measurement.
bool calIsAsBuilt(const obs::Calibration& cal);

// The v1 automatic path, called at boot when the v3 and v2 loads fail: v1
// blob present AND equal to the as-built table -> adopt it and re-save as v3,
// return true. Anything else -> false, robot boots uncalibrated, `run`
// refuses.
bool calMigrateV1(obs::Calibration& out);                         // ESP side

// ESP-IDF side. calLoad leaves `out` at its constructed defaults and returns
// false when nothing is stored yet -- that is the normal first-boot path, not
// an error.
bool calLoad(obs::Calibration& out);
bool calSave(const obs::Calibration& cal);
bool calErase();

// -- IMU calibration -------------------------------------------------------
//
// A SEPARATE blob from the servo one, deliberately: the servo calibration is
// bound to the servo map and gets rejected wholesale when that map changes,
// and there is no reason for an IMU bias to die with it.
//
// Both fields have to persist or the robot is not usable from a cold boot:
//
//  * gyro_bias -- measured on the bench, and NOT optional. The part's
//    untrimmed zero-rate offset on this board is ~0.45 rad/s on x. Against
//    the filter's 0.5/s correction gain that settles at sin(e) = 0.9, i.e.
//    ~64 degrees of steady-state attitude error. An uncalibrated boot does
//    not degrade the up-vector, it destroys it.
//
//  * mount -- the sensor->body rotation. On this board the driver stands
//    vertical and transverse in the pelvis recess, so this is a ~90 degree
//    rotation and not a trim.
struct ImuCalBlob {
    uint32_t magic;          // kImuCalMagic
    uint16_t version;        // kImuCalVersion
    uint16_t pad;
    float gyro_bias[3];      // rad/s, SENSOR frame
    float mount[4];          // w, x, y, z -- sensor -> body
    uint32_t crc;            // CRC-32 over everything above
};

constexpr uint32_t kImuCalMagic = 0x424D4931;   // "BMI1"
constexpr uint16_t kImuCalVersion = 1;

// Pure, host-testable.
void imuCalPack(const float bias[3], const float mount[4], ImuCalBlob& out);
bool imuCalUnpack(const ImuCalBlob& blob, float bias_out[3],
                  float mount_out[4]);

// ESP side. imuCalLoad returns false when nothing is stored -- the normal
// first-boot path. The caller must then treat the IMU as UNCALIBRATED rather
// than assume zero bias is fine, because it is not.
bool imuCalLoad(float bias_out[3], float mount_out[4]);
bool imuCalSave(const float bias[3], const float mount[4]);
// Per-axis gyro SCALE correction (sensor frame), its own NVS record so the
// bias/mount blob keeps its version. Absent = (1, 1, 1). 2026-09-03: the
// QMI8658C's x-axis gyro (body pitch after the mount) over-reads by 18 %
// against the accelerometer's tilt on a clamped-leg test; y and z by 2-3 %.
bool imuGyroScaleLoad(float scale_out[3]);
bool imuGyroScaleSave(const float scale[3]);
bool imuCalErase();

}  // namespace robot

#include "recorder.h"

#include <string.h>
#include <sys/stat.h>
#include <time.h>
#ifdef __linux__
#include <sys/timex.h>
#endif

namespace bimo {

bool Recorder::start(const char* dir, const char* host, double now_ms,
                     char* err, size_t cap) {
    if (active()) return true;
    mkdir(dir, 0755);
    time_t now = time(nullptr);
    tm utc;
    gmtime_r(&now, &utc);
    char stamp[32];
    strftime(stamp, sizeof stamp, "%Y-%m-%dT%H-%M-%SZ", &utc);
    snprintf(path, sizeof path, "%s/%s-%s.jsonl", dir, stamp, host);
    f = fopen(path, "w");
    if (f == nullptr) {
        snprintf(err, cap, "cannot write %s", path);
        path[0] = 0;
        return false;
    }
    n_cmd = 0;
    n_tlm = 0;
    t0_ms = now_ms;
    last_flush_ms = now_ms;
    // A header record, so a reader knows what it is holding without guessing
    // from the first data line -- including whether the `utc` column that
    // follows can be trusted against a webcam frame.
    long maxerr = -1;
    const char* clock = hostClockState(&maxerr);
    fprintf(f,
            "{\"rec\":\"header\",\"host\":\"%s\",\"utc\":\"%s\","
            "\"utc_s\":%.6f,\"host_clock\":\"%s\",\"maxerror_us\":%ld}\n",
            host, stamp, wallNow(), clock, maxerr);
    return true;
}

double Recorder::wallNow() {
    timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<double>(ts.tv_sec) +
           static_cast<double>(ts.tv_nsec) / 1e9;
}

const char* Recorder::hostClockState(long* maxerror_us) {
#ifdef __linux__
    // The kernel's STA_UNSYNC flag, which chrony/ntpd/timesyncd maintain and
    // `timedatectl show -p NTPSynchronized` reads. TIME_ERROR = unsynced.
    ntptimeval ntv;
    memset(&ntv, 0, sizeof ntv);
    const int r = ntp_gettime(&ntv);
    if (maxerror_us) *maxerror_us = ntv.maxerror;
    return r == TIME_ERROR ? "unsynced" : "ntp";
#else
    if (maxerror_us) *maxerror_us = -1;
    return "unknown";
#endif
}

void Recorder::stop() {
    if (f == nullptr) return;
    fclose(f);
    f = nullptr;
}

void Recorder::command(double now_ms, uint32_t seq, float vx, float vy,
                       float wz, const Ext& ext, uint8_t flags, size_t len) {
    if (f == nullptr) return;
    fprintf(f,
            "{\"t\":%.3f,\"utc\":%.6f,\"dir\":\"cmd\",\"seq\":%u,\"len\":%zu,"
            "\"flags\":%u,"
            "\"vx\":%.3f,\"vy\":%.3f,\"wz\":%.3f,\"crouch\":%.3f,"
            "\"lift\":%.3f,\"foot_dx\":%.3f,\"foot_dz\":%.3f}\n",
            (now_ms - t0_ms) / 1000.0, wallNow(), seq, len, flags,
            static_cast<double>(vx), static_cast<double>(vy),
            static_cast<double>(wz), static_cast<double>(ext.crouch),
            static_cast<double>(ext.lift), static_cast<double>(ext.foot_dx),
            static_cast<double>(ext.foot_dz));
    ++n_cmd;
}

void Recorder::telemetry(double now_ms, const Telemetry& t) {
    if (f == nullptr) return;
    char robot[32];
    if (t.t_us == 0) {
        snprintf(robot, sizeof robot, "null");
    } else {
        snprintf(robot, sizeof robot, "%llu.%06u",
                 static_cast<unsigned long long>(t.t_us / 1000000ull),
                 static_cast<unsigned>(t.t_us % 1000000ull));
    }
    fprintf(f,
            "{\"t\":%.3f,\"utc\":%.6f,\"robot_utc\":%s,\"dir\":\"tlm\","
            "\"state\":\"%s\",\"seq_echo\":%u,"
            "\"vbat\":%.3f,\"up_z\":%.3f,\"vx_est\":%.3f,\"wz_est\":%.3f,"
            "\"servo_err\":%u,\"late_pct\":%u",
            (now_ms - t0_ms) / 1000.0, wallNow(), robot, stateName(t.state),
            t.seq_echo,
            static_cast<double>(t.vbat_v), static_cast<double>(t.up_z),
            static_cast<double>(t.vx_est), static_cast<double>(t.wz_est),
            t.servo_err, t.loop_late_pct);
    // The measured pose, when the beacon carried one (mirror mode): this is
    // the motion a video frame is correlated against, so it is logged.
    if (t.n_joints == linkproto::kNumJoints) {
        fputs(",\"q\":[", f);
        for (size_t i = 0; i < linkproto::kNumJoints; ++i) {
            fprintf(f, "%s%.4f", i ? "," : "", static_cast<double>(t.joints[i]));
        }
        fputc(']', f);
    }
    fputs("}\n", f);
    ++n_tlm;
}

void Recorder::maybeFlush(double now_ms) {
    if (f == nullptr) return;
    if (now_ms - last_flush_ms < 1000.0) return;
    fflush(f);
    last_flush_ms = now_ms;
}

}  // namespace bimo

// Wall-clock time on a robot that has none of its own.
//
// The board has no RTC: no clock chip, no 32 kHz crystal, no backup cell
// (the vendor schematic, read end to end; GPIO32/33, the only pins that could
// take a slow crystal, are the I2C bus). The ESP32's own RTC domain runs off
// a ~150 kHz RC oscillator and loses the time at every power cycle. So every
// boot starts at the epoch, and the only thing that can put the real time on
// this robot is the network: SNTP over the same WiFi link the commands ride.
//
// WHY IT MATTERS
// --------------
// Telemetry carries a u64 `t_us` (linkproto::Telemetry, docs/control-channel
// .md "Time on the wire") so a beacon -- and in mirror mode the joint angles
// in it -- can be laid against a webcam frame stamped by the same NTP time
// base on the host. Without it a hardware take lines up with its video to
// about a second; with it, to a frame.
//
// SERVER
// ------
// From DHCP option 42 first (CONFIG_LWIP_DHCP_GET_NTP_SRV; the LAN router
// hands it out), with pool.ntp.org as the static fallback in the second
// slot. Sync every CONFIG_LWIP_SNTP_UPDATE_DELAY ms (30 s: between steps the
// clock free-runs on the 40 MHz crystal via esp_timer, so drift stays in the
// low milliseconds). The RFC 4330 startup delay is OFF -- a random 1-5 min
// wait before the first request is fine for a thermostat and useless for a
// robot that syncs on every power-up.
//
// HONESTY
// -------
// Until the first reply lands the clock reads 1970 and every function here
// says so with a 0, which is what the wire field means by "unsynced". Nothing
// ever sends the boot-relative time dressed up as an epoch.
#pragma once
#include <stdint.h>

namespace robot {

struct TimeStatus {
    bool synced;            // at least one SNTP reply has set the clock
    uint32_t syncs;         // replies applied since boot
    uint64_t now_us;        // epoch microseconds, 0 if not synced
    int64_t since_sync_us;  // esp_timer microseconds since the last reply
    int32_t last_step_ms;   // how far the last reply moved the clock
    char server0[64];       // slot 0: DHCP-provided, or "" if none yet
    char server1[64];       // slot 1: the static fallback
    bool started;           // esp_netif_sntp_start() has been called
};

// Configure the SNTP client (does not start it). Call once, after
// esp_event_loop_create_default() and before the first IP event.
void startTimeSync();

// Start polling. Idempotent; called from the GOT_IP handler.
void timeSyncStart();

// Epoch microseconds now, or 0 until the first sync.
uint64_t timeNowUs();

// Epoch microseconds for an instant measured on esp_timer's clock (boot-
// relative microseconds), or 0 until the first sync. The offset between the
// two clocks is constant between SNTP steps, so this is how a pose read at
// tick time gets stamped from the beacon task without either task holding
// the other's clock.
uint64_t timeFromBootUs(int64_t boot_us);

void timeGetStatus(TimeStatus& out);

}  // namespace robot

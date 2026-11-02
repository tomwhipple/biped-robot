// The authenticated Wi-Fi CLI -- the bench CLI over UDP, for one paired
// client holding the shared key. docs/wifi-cli.md is the manual and the
// threat model; components/wcli is the protocol, the session logic and the
// allowlist (pure, host-tested); this file is the ESP32 glue.
//
// TWO TASKS, ONE LOCK. The "wcli" task (core 0) owns the UDP socket on port
// 4213: it feeds datagrams to wcli::Server and sends the output chunks. A
// line the Server accepts is posted to the SAME queue the tether's lines go
// to (link_task.cpp), so it runs on the housekeeping task, through the same
// cli::execute(), in turn with tether lines -- never concurrently with any
// other bus command. Output comes back through the sink here. A FreeRTOS
// mutex guards the Server; neither the control loop (core 1) nor the UART
// link task ever takes it.
//
// The key lives in NVS ("bimo"/"wcli_key", 32 bytes) and is set over the
// TETHER only: `wcli key <64 hex>` (bench mode). No key, no Wi-Fi CLI: every
// datagram is dropped unanswered.
#pragma once
#include <stddef.h>
#include <stdint.h>

namespace robot {

// app_main, after startLinkTask() (the CLI queue) and startWifiLink() (the
// network stack). Loads the key and starts the wcli task.
void startWifiCli();

// -- housekeeping task only ---------------------------------------------------
// Run one Wi-Fi line (queued by the wcli task with its command seq): the
// at-most-once check, the bench-only/allowlist gate, the flight-recorder
// event, cli::execute() with the Wi-Fi sink, and the DONE marker.
void wcliExecute(const char* line, uint32_t seq);
// Text the housekeeping loop printed outside any command (pose/balance/home
// verdicts, link edges): copied to the Wi-Fi client while one is listening.
// Never blocks -- the balance loop runs on this task.
void wcliAsyncOutput(const char* text);
// Once per housekeeping iteration: make pending async output sendable, and
// log session changes to the flight recorder.
void wcliHouseTick();

// -- the `wcli` CLI command (tether only; allowlisted over Wi-Fi as status) --
struct WcliStatus {
    bool have_key;
    char fingerprint[9];     // SHA-256(key)[:4] as hex, "" without a key
    bool session;
    uint32_t peer_ip;        // host order
    uint16_t peer_port;
    uint32_t last_seq;
    uint8_t state;           // wcli::CmdState
    uint32_t out_next, oldest;
    uint32_t heard_ms_ago;
    uint32_t rx_ok, rx_bad, rx_nokey, rx_stale, rx_wrong_peer, replays;
    uint32_t sessions, cmds, busy, lost_chunks;
};
void wcliGetStatus(WcliStatus& out);
// Parse 64 hex digits, store in NVS, re-key (drops the session). The hex is
// never echoed or logged. false = not 64 hex digits, or NVS failed.
bool wcliKeySet(const char* hex, char fingerprint_out[9]);
bool wcliKeyClear();
void wcliDropSession();

}  // namespace robot

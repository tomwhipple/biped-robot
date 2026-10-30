// Core-0 tasks: the tethered command link and housekeeping.
//
// v1 carries the protocol over UART0 (the USB tether). That is the must-have
// path -- docs/control-channel.md specifies identical framing over UDP and
// UART0, so adding the WiFi socket later is a second producer into the same
// mailbox, not a second protocol. See firmware/README.md for the v2 seam.
#include "link_task.h"
#include "timesync.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "cli.h"
#include "flight_rec.h"
#include "wcli/wcli.h"
#include "wifi_cli.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "linkproto/framing.h"
#include "linkproto/protocol.h"
#include "shared.h"

namespace robot {
namespace {

constexpr int kLinkStackWords = 3072;
constexpr int kHouseStackWords = 8192;   // bytes on ESP-IDF; 4096 left 1264 B free once the flight recorder encoded from here (10-30)
constexpr size_t kCliLineMax = linkproto::kMaxCliLine + 1;
constexpr int kCliQueueLen = 4;

StackType_t g_link_stack[kLinkStackWords];
StaticTask_t g_link_tcb;
StackType_t g_house_stack[kHouseStackWords];
StaticTask_t g_house_tcb;

// Static queue storage: no heap after init. One queue for both channels, so
// tether and Wi-Fi lines run one at a time, in arrival order, on one task.
enum class CliSrc : uint8_t { kTether = 0, kWifi = 1 };
struct CliLine {
    char text[kCliLineMax];
    CliSrc src;
    uint32_t seq;   // the Wi-Fi CLI command seq (wifi_cli.h); 0 for the tether
};
uint8_t g_cli_storage[kCliQueueLen * sizeof(CliLine)];
StaticQueue_t g_cli_qbuf;
QueueHandle_t g_cli_queue = nullptr;

uint8_t g_cmd_storage[sizeof(CommandMsg)];
StaticQueue_t g_cmd_qbuf;

linkproto::Demux g_demux;

// The observation dump's one line buffer (obs_dump.h). File scope rather than
// on the housekeeping stack: 1.5 KB is most of a small task stack, and there
// is exactly one writer of this UART anyway.
char g_obs_line[kObsDumpLineMax];

void uartSay(const char* s) {
    uart_write_bytes(board::kHostUart, s, strlen(s));
}

// The sink for a TETHER line's reply: the tether, and -- when the flight
// recorder is on -- one EVENT per printed line, so every verdict the CLI
// gives (refusals, readbacks) is in the recording with its timestamp, not
// only in whatever caught the serial port.
void tetherSay(const char* s) {
    uartSay(s);
    flight::outputText(s, flightrec::kChanTether);
}

// The housekeeping loop's own sink, for what it prints OUTSIDE any command
// line (balance ON/ARMED/ABORT/landed from poseTick, home readbacks, link
// arm/home edges): the tether and the recorder as above, and the Wi-Fi CLI
// client if one is listening -- these verdicts are robot state, whichever
// channel started the motion. A tether line's own reply is not copied to
// the Wi-Fi client (tetherSay), nor a Wi-Fi reply to the tether.
void houseSay(const char* s) {
    tetherSay(s);
    wcliAsyncOutput(s);
}

void linkTask(void*) {
    uint8_t rx[64];
    for (;;) {
        const int n = uart_read_bytes(board::kHostUart, rx, sizeof rx,
                                      pdMS_TO_TICKS(20));
        if (n <= 0) continue;
        for (int i = 0; i < n; ++i) {
            switch (g_demux.push(rx[i])) {
                case linkproto::Demux::Event::kFrame: {
                    linkproto::Command pkt{};
                    if (linkproto::decodeCommand(g_demux.frame(),
                                                 linkproto::kCmdLen, pkt) !=
                        linkproto::Err::kOk) {
                        break;    // CRC did its job; drop it silently
                    }
                    CommandMsg msg;
                    msg.cmd = pkt;
                    msg.rx_ms =
                        static_cast<float>(esp_timer_get_time()) / 1000.0f;
                    // Latest wins: a stale command must never queue up behind
                    // a late tick (firmware-design section 4).
                    xQueueOverwrite(g_cmd_mailbox, &msg);
                    break;
                }
                case linkproto::Demux::Event::kLine: {
                    CliLine line;
                    strncpy(line.text, g_demux.line(), kCliLineMax - 1);
                    line.text[kCliLineMax - 1] = 0;
                    line.src = CliSrc::kTether;
                    line.seq = 0;
                    // Drop, do not block: the link path must never wait on the
                    // CLI consumer.
                    xQueueSend(g_cli_queue, &line, 0);
                    break;
                }
                case linkproto::Demux::Event::kOverflow:
                    uartSay("line too long\r\n");
                    break;
                case linkproto::Demux::Event::kNone:
                    break;
            }
        }
    }
}

void houseTask(void*) {
    flight::bindHouseTask();
    cli::banner(&uartSay);
    TickType_t next_tlm = xTaskGetTickCount();
    uint32_t arm_edges_seen = g_link_arm_edges.load();
    uint32_t home_edges_seen = g_link_home_edges.load();
    // Observation dump state, owned by this loop alone.
    ObsDumpMode obs_started = ObsDumpMode::kOff;
    TickType_t next_obs = xTaskGetTickCount();
    uint32_t obs_seq_seen = 0;
    for (;;) {
        CliLine line;
        while (xQueueReceive(g_cli_queue, &line, 0) == pdTRUE) {
            if (line.src == CliSrc::kWifi) {
                // Authenticated by the wcli task; the bench-only gate, the
                // allowlist, the recorder event and the reply are there.
                wcliExecute(line.text, line.seq);
                continue;
            }
            // The recorder ships events over UDP in clear: never a secret.
            static char logged[kCliLineMax];   // housekeeping only
            wcli::redactForLog(line.text, logged, sizeof logged);
            flight::event(flightrec::Ev::kCmd, logged, flightrec::kChanTether);
            cli::execute(line.text, &tetherSay);
        }

        // Wireless arm/disarm (shared.h g_link_arm_*): consumed here, between
        // CLI lines, so the mode keeps one writer and a WiFi arm can never
        // land inside a bench-mode bus command. Same gate as `run`.
        const uint32_t edges = g_link_arm_edges.load();
        if (edges != arm_edges_seen) {
            arm_edges_seen = edges;
            flight::event(flightrec::Ev::kArmEdge, nullptr,
                          g_link_arm_level.load() ? 1 : 0);
            cli::linkMode(g_link_arm_level.load(), &houseSay);
        }

        // Wireless "reset the servos" (shared.h g_link_home_edges). Consumed
        // in the same place and for the same reason as the arm edge: this is
        // the only task allowed to drive the bus while benched. It benches
        // the loop itself, so it is honoured from ANY state -- benched,
        // fallen, or E-stopped, which is the entire point of the request.
        const uint32_t homes = g_link_home_edges.load();
        if (homes != home_edges_seen) {
            home_edges_seen = homes;
            flight::event(flightrec::Ev::kHomeEdge, nullptr);
            cli::linkHome(&houseSay);
        }
        // The home's readback, once its slew deadline has passed (typed
        // `home` and the wireless request both arm it). Stores the verdict
        // the beacon actually means: at the zeros, or not.
        cli::homeVerify(&houseSay);
        cli::poseTick(&houseSay);
        // The flight recorder's bench half (flight_rec.h). What it spends on
        // the bus comes back out of this iteration's sleep below, so the
        // stream's tick phase -- and the bench balance loop riding on it --
        // is the same with the recorder on as off.
        const uint32_t rec_us = cli::recorderTick();

        // 10 Hz telemetry, back up the same tether the commands came down.
        //
        // The tether stays CLASSIC-ONLY (kTlmLen), deliberately, even when a
        // tethered commander sets kFlagPose. Mirror mode (docs/mirror-mode.md)
        // is a bimo_gui-over-UDP feature -- wifi_link.cpp answers kFlagPose
        // with the kTlmLenExt frame; nothing consumes joint angles over the
        // UART, and this tether is 115200 baud already shared with CLI text
        // and the 5 Hz obs dump, where 20 more bytes per beacon is not free.
        // Honouring the flag here would also mean carrying the commander's
        // level from linkTask to this task across the mailbox. If a tethered
        // consumer ever appears, do it the way wifi_link.cpp does; until
        // then the decision is: the UART beacon is always kTlmLen (31 B).
        if (xTaskGetTickCount() >= next_tlm) {
            next_tlm += pdMS_TO_TICKS(100);
            if (g_mode_request.load() == Mode::kRun) {
                linkproto::Telemetry t{};
                t.seq_echo = g_telemetry.seq_echo.load();
                t.state = static_cast<linkproto::LinkState>(
                    g_telemetry.state.load());
                t.vbat_v = g_telemetry.vbat_mv.load() / 1000.0f;
                t.up_z = g_telemetry.up_z.load();
                t.vx_est = g_telemetry.vx_est.load();
                t.wz_est = g_telemetry.wz_est.load();
                // Bit b = bus joint b (servo kBusServoId[b]), all 17 of them.
                t.servo_err = g_telemetry.servo_err.load();
                t.loop_late_pct = g_telemetry.loop_late_pct.load();
                t.t_us = timeNowUs();   // 0 until SNTP answers (timesync.h)
                uint8_t wire[linkproto::kTlmLen];
                linkproto::encodeTelemetry(wire, t);
                uart_write_bytes(board::kHostUart,
                                 reinterpret_cast<const char*>(wire),
                                 linkproto::kTlmLen);
            }
        }

        // -- observation dump (obs_dump.h) ---------------------------------
        //
        // Text records on the same tether as the binary telemetry above, but
        // never interleaved WITHIN a line: both are written from this one
        // task, so a record reaches the host whole and a host tool can find
        // it by its "OBS," tag in otherwise mixed console output.
        const ObsDumpMode dump = g_obs_dump.mode();
        if (dump == ObsDumpMode::kOff) {
            obs_started = ObsDumpMode::kOff;
        } else if (g_mode_request.load() != Mode::kRun) {
            // The loop went back to bench under us -- it stopped assembling
            // observations, so the dump would go quiet with no explanation.
            // Say so and turn it off; the mode's only writer is this task.
            g_obs_dump.setMode(ObsDumpMode::kOff);
            obs_started = ObsDumpMode::kOff;
            uartSay("obsdump: off -- the loop is benched, no observations\r\n");
        } else {
            if (obs_started == ObsDumpMode::kOff) {
                // First line of every capture: the column names, straight
                // from the generated obs spec.
                if (formatObsHeader(g_obs_line, sizeof g_obs_line)) {
                    uartSay(g_obs_line);
                }
                obs_seq_seen = 0;
                next_obs = xTaskGetTickCount();
                obs_started = dump;
            }
            if (xTaskGetTickCount() >= next_obs) {
                // Rearmed from NOW, not by adding a period: a diagnostic that
                // fell behind must not then burst to catch up on a tether it
                // already half fills.
                next_obs = xTaskGetTickCount() +
                           pdMS_TO_TICKS(kObsDumpPeriodMs);
                ObsRecord rec;
                uint32_t seq = 0;
                // seq unchanged == the loop published nothing since the last
                // record: silence is the honest report, not a repeat.
                if (g_obs_dump.read(rec, seq) && seq != obs_seq_seen) {
                    obs_seq_seen = seq;
                    if (formatObsRecord(rec, g_obs_line, sizeof g_obs_line)) {
                        uartSay(g_obs_line);
                    }
                    if (dump == ObsDumpMode::kOnce) {
                        g_obs_dump.setMode(ObsDumpMode::kOff);
                        obs_started = ObsDumpMode::kOff;
                    }
                }
            }
        }
        // Async output printed this iteration becomes sendable to the
        // Wi-Fi CLI client now, not when a chunk fills.
        wcliHouseTick();
        const TickType_t sleep = pdMS_TO_TICKS(10);
        const TickType_t give_back = pdMS_TO_TICKS(rec_us / 1000u);
        vTaskDelay(give_back < sleep ? sleep - give_back : 1);
    }
}

}  // namespace

bool submitWifiCliLine(const char* text, uint32_t seq) {
    if (g_cli_queue == nullptr) return false;
    CliLine line;
    strncpy(line.text, text, kCliLineMax - 1);
    line.text[kCliLineMax - 1] = 0;
    line.src = CliSrc::kWifi;
    line.seq = seq;
    return xQueueSend(g_cli_queue, &line, 0) == pdTRUE;
}

void startLinkTask() {
    g_cmd_mailbox = xQueueCreateStatic(1, sizeof(CommandMsg), g_cmd_storage,
                                       &g_cmd_qbuf);
    g_cli_queue = xQueueCreateStatic(kCliQueueLen, sizeof(CliLine),
                                     g_cli_storage, &g_cli_qbuf);
    xTaskCreateStaticPinnedToCore(linkTask, "link", kLinkStackWords, nullptr,
                                  5, g_link_stack, &g_link_tcb, 0);
}

void startHousekeepingTask() {
    xTaskCreateStaticPinnedToCore(houseTask, "housekeeping", kHouseStackWords,
                                  nullptr, 2, g_house_stack, &g_house_tcb, 0);
}

}  // namespace robot

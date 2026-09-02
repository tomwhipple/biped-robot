// Core-0 tasks: the tethered command link and housekeeping.
//
// v1 carries the protocol over UART0 (the USB tether). That is the must-have
// path -- docs/control-channel.md specifies identical framing over UDP and
// UART0, so adding the WiFi socket later is a second producer into the same
// mailbox, not a second protocol. See firmware/README.md for the v2 seam.
#include "link_task.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "cli.h"
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
constexpr int kHouseStackWords = 4096;
constexpr size_t kCliLineMax = 96;
constexpr int kCliQueueLen = 4;

StackType_t g_link_stack[kLinkStackWords];
StaticTask_t g_link_tcb;
StackType_t g_house_stack[kHouseStackWords];
StaticTask_t g_house_tcb;

// Static queue storage: no heap after init.
struct CliLine { char text[kCliLineMax]; };
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
            cli::execute(line.text, &uartSay);
        }

        // Wireless arm/disarm (shared.h g_link_arm_*): consumed here, between
        // CLI lines, so the mode keeps one writer and a WiFi arm can never
        // land inside a bench-mode bus command. Same gate as `run`.
        const uint32_t edges = g_link_arm_edges.load();
        if (edges != arm_edges_seen) {
            arm_edges_seen = edges;
            cli::linkMode(g_link_arm_level.load(), &uartSay);
        }

        // Wireless "reset the servos" (shared.h g_link_home_edges). Consumed
        // in the same place and for the same reason as the arm edge: this is
        // the only task allowed to drive the bus while benched. It benches
        // the loop itself, so it is honoured from ANY state -- benched,
        // fallen, or E-stopped, which is the entire point of the request.
        const uint32_t homes = g_link_home_edges.load();
        if (homes != home_edges_seen) {
            home_edges_seen = homes;
            cli::linkHome(&uartSay);
        }

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
        // then the decision is: the UART beacon is always 20 B.
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
                // The wire field is 8 bits (one bit per servo ID 1-8); the
                // 10-DOF plant has two more joints, so the hip-yaw faults are
                // folded into the top bits. Widening the frame is a protocol
                // change and therefore a link/protocol.py change first.
                const uint16_t f = g_telemetry.servo_err.load();
                t.servo_err = static_cast<uint8_t>(f & 0xFF) |
                              static_cast<uint8_t>((f >> 8) & 0x03);
                t.loop_late_pct = g_telemetry.loop_late_pct.load();
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

}  // namespace

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

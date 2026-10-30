#pragma once
#include <stdint.h>

namespace robot {

// Core 0. Reads UART0, splits the stream into binary command frames and CLI
// lines (linkproto::Demux), posts frames to the command mailbox and lines to
// the housekeeping task.
void startLinkTask();

// Core 0, low priority. Runs CLI lines and emits the 10 Hz telemetry frame.
void startHousekeepingTask();

// The Wi-Fi CLI's way in (wifi_cli.cpp, from the wcli task): queue one
// authenticated line, with its command seq, behind/among the tether's lines
// -- the housekeeping task runs them all, one at a time. Never blocks; false
// when the queue is full (the client is told BUSY and retries).
bool submitWifiCliLine(const char* text, uint32_t seq);

}  // namespace robot

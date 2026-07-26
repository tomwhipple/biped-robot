#pragma once

namespace robot {

// Core 0. Reads UART0, splits the stream into binary command frames and CLI
// lines (linkproto::Demux), posts frames to the command mailbox and lines to
// the housekeeping task.
void startLinkTask();

// Core 0, low priority. Runs CLI lines and emits the 10 Hz telemetry frame.
void startHousekeepingTask();

}  // namespace robot

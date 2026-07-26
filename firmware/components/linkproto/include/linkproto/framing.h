// UART0 byte-stream demultiplexer: binary link frames vs. ASCII CLI lines.
//
// The tethered path (docs/control-channel.md: "identical framing over UDP and
// over UART0") and the bring-up CLI both want the USB-serial port, and the
// board has exactly one. Rather than a modal switch that can be left in the
// wrong state, this splits the stream by shape:
//
//   * at the start of a line, the two bytes 'B','M' begin a 14-byte binary
//     command frame -- capture it, hand it to decodeCommand()
//   * anything else accumulates until '\n' and is handed to the CLI
//
// The false-positive risk is a CLI line literally starting with "BM"; none of
// our commands do, and decodeCommand()'s CRC rejects the frame anyway (the
// bytes are then reported as a bad frame, not silently executed). Pure and
// host-tested for exactly that reason.
#pragma once
#include <stddef.h>
#include <stdint.h>

#include "linkproto/protocol.h"

namespace linkproto {

constexpr size_t kMaxCliLine = 96;

class Demux {
  public:
    enum class Event : uint8_t {
        kNone = 0,
        kFrame,     // frame() holds kCmdLen bytes
        kLine,      // line() holds a NUL-terminated command line
        kOverflow,  // the line buffer filled up; contents were dropped
    };

    // Feed one byte. Returns what (if anything) that byte completed.
    Event push(uint8_t b);

    const uint8_t* frame() const { return buf_; }
    const char* line() const { return reinterpret_cast<const char*>(buf_); }
    size_t lineLen() const { return line_len_; }

    void reset() { n_ = 0; line_len_ = 0; in_frame_ = false; }

  private:
    uint8_t buf_[kMaxCliLine + 1] = {};
    size_t n_ = 0;            // bytes held in buf_
    size_t line_len_ = 0;     // set when kLine is returned
    bool in_frame_ = false;

    static_assert(kMaxCliLine + 1 >= kCmdLen, "buffer must hold a full frame");
};

}  // namespace linkproto

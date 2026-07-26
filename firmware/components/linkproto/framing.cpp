#include "linkproto/framing.h"

namespace linkproto {

Demux::Event Demux::push(uint8_t b) {
    if (in_frame_) {
        buf_[n_++] = b;
        if (n_ == kCmdLen) {
            n_ = 0;
            in_frame_ = false;
            return Event::kFrame;
        }
        return Event::kNone;
    }

    // A 'B' at the start of a line might be the magic; hold it and decide on
    // the next byte.
    if (n_ == 0 && b == kMagicCmd[0]) {
        buf_[n_++] = b;
        return Event::kNone;
    }
    if (n_ == 1 && buf_[0] == kMagicCmd[0] && b == kMagicCmd[1]) {
        buf_[n_++] = b;
        in_frame_ = true;
        return Event::kNone;
    }

    if (b == '\n' || b == '\r') {
        if (n_ == 0) return Event::kNone;       // ignore blank lines / CRLF
        line_len_ = n_;
        buf_[n_] = 0;
        n_ = 0;
        return Event::kLine;
    }
    if (n_ >= kMaxCliLine) {
        n_ = 0;
        return Event::kOverflow;
    }
    buf_[n_++] = b;
    return Event::kNone;
}

}  // namespace linkproto

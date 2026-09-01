// A client for the MJPEG live view sim/sil_twin.py already serves.
//
// The house convention is multipart/x-mixed-replace over HTTP, latest frame
// wins, ~10 fps: sim/sil_twin.py:49 (port 8645) and tools/cam_live.py:25
// (8646) are the two servers. This is the consumer, so the sim can be watched
// inside the console driving it rather than in a browser beside it.
//
// The parser is split from the socket deliberately: a stream parser fed by a
// network is the one part of a GUI that can be tested exhaustively, and
// firmware/host/test_client.cpp does exactly that -- split reads, a header
// block with no Content-Length, a truncated frame.
//
// A stalled stream must be VISIBLY stalled. tools/cam_live.py:87 learned this
// the other way round: a dead producer left the server showing a frozen frame
// that looked live. Hence lastFrameMs(), which the UI draws as STALE.
#ifndef BIMO_LINK_MJPEG_H_
#define BIMO_LINK_MJPEG_H_

#include <pthread.h>
#include <stddef.h>
#include <stdint.h>

namespace bimo {

// Incremental multipart/x-mixed-replace parser. Requires Content-Length on
// each part, which both house servers send; a header block without one (the
// HTTP response's own headers, above all) is skipped rather than guessed at.
class MjpegParser {
  public:
    MjpegParser();
    ~MjpegParser();

    void feed(const uint8_t* data, size_t n);

    // True once a complete part is buffered. The pointer stays valid until
    // the next feed() or take() -- consume or copy it before either.
    bool take(const uint8_t** out, size_t* len);

    void reset();
    bool overflowed() const { return overflow_; }

  private:
    enum State { kHeaders, kBody };

    void advance();

    State state_ = kHeaders;
    uint8_t* acc_ = nullptr;       // bytes off the wire, not yet consumed
    size_t acc_len_ = 0, acc_cap_ = 0;
    uint8_t* out_ = nullptr;       // the completed frame take() hands back
    size_t out_len_ = 0, out_cap_ = 0;
    size_t want_ = 0;              // Content-Length of the part being read
    bool ready_ = false;
    bool overflow_ = false;
};

// Pulls one URL on a thread of its own, keeping only the newest frame.
class MjpegStream {
  public:
    MjpegStream();
    ~MjpegStream();

    // http://host:port/path. Stops any stream already running.
    bool start(const char* url);
    void stop();

    bool running() const { return running_; }

    // Copies the newest frame out if it is newer than `*seq` (which is then
    // updated). `out` must be freed with freeFrame(). Returns false if there
    // is nothing newer.
    bool latest(uint64_t* seq, uint8_t** out, size_t* len);
    static void freeFrame(uint8_t* p);

    uint32_t frames() const { return frames_; }
    double lastFrameMs() const { return last_frame_ms_; }
    // Empty while healthy; otherwise why the stream is not delivering. Copies
    // under the lock: the stream thread rewrites it.
    void error(char* out, size_t cap) const;
    char url[512] = "";

  private:
    static void* trampoline(void* self);
    void run();
    bool attempt(const char* host, const char* port, const char* path);
    void setError(const char* fmt, const char* a);
    void setErrorPair(const char* fmt, const char* a, const char* b);

    pthread_t thread_ = {};
    mutable pthread_mutex_t lock_ = PTHREAD_MUTEX_INITIALIZER;
    bool started_ = false;
    volatile bool running_ = false;
    volatile bool stop_ = false;

    uint8_t* frame_ = nullptr;
    size_t frame_len_ = 0;
    uint64_t frame_seq_ = 0;
    uint32_t frames_ = 0;
    double last_frame_ms_ = 0.0;
    char err_[192] = "";
};

}  // namespace bimo

#endif  // BIMO_LINK_MJPEG_H_

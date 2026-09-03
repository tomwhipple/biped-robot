// Starting and stopping sim/sil_twin.py from the console that drives it.
//
// sil_twin runs the REAL firmware control stack behind the real UDP link at
// 1x real time and serves its render as MJPEG, so "drive the sim" and "drive
// the robot" differ only in the --host. Making the console able to start it
// means one window and one thing to remember, rather than a terminal beside
// the terminal.
//
// A child that dies must be VISIBLY dead: the exit status goes in the log,
// because a console that keeps showing the last frame of a crashed sim is
// exactly the failure tools/cam_live.py:87 already had to fix once.
#ifndef BIMO_LINK_SIMPROC_H_
#define BIMO_LINK_SIMPROC_H_

#include <stddef.h>
#include <sys/types.h>

namespace bimo {

constexpr int kSimLogLines = 200;
constexpr int kSimLogCols = 200;
constexpr int kMaxRuns = 128;

// The runs sil_twin can actually load: a sim/runs/<name>/config.json AND an
// exported sim/sil/weights/<name>.silw.json. Anything else fails at startup
// with a traceback, which is a poor dropdown entry.
struct RunList {
    char name[kMaxRuns][64];
    int n = 0;
    void scan(const char* repo);
    int indexOf(const char* want) const;
};

struct SimProc {
    pid_t pid = -1;
    int out_fd = -1;
    int in_fd = -1;
    char log[kSimLogLines][kSimLogCols];
    int log_n = 0;                     // total lines ever, for the ring
    char partial[kSimLogCols];
    size_t partial_len = 0;
    char err[192] = "";
    // sil_twin's own one-line description of what it is running (run name and
    // plant). Lifted from its stdout so the panel can state what is on screen
    // instead of implying it from controls the operator has not touched.
    char ident[200] = "";

    bool running() const { return pid > 0; }

    // Spawns `<repo>/.venv/bin/python <repo>/sim/sil_twin.py ...`.
    // `viewer` starts it as a VISUALISER: no policy, no physics, no beacon --
    // the plant is posed from the `pose` lines this console pushes and
    // rendered. That is what mirror mode wants, and running the sim forward
    // beside the robot is what it must not do (docs/mirror-mode.md).
    bool start(const char* repo, const char* run_name, bool hang, bool viewer,
               int stream_port, int cmd_port, int tlm_port);
    // SIGTERM, then reap. Safe to call when nothing is running.
    void stop();
    // Drains stdout into the ring and reaps a child that exited on its own.
    // Call every frame.
    void poll();

    const char* line(int i) const;     // 0 = oldest kept
    int lines() const;

    void pushLine(const char* s);
    // A line down the child's stdin (sil_twin understands `reset`). False if
    // nothing is running to hear it.
    bool tell(const char* line);
};

// The repo root: <dir of argv[0]>/../../.. for firmware/host/build/bimo_gui.
void guessRepoRoot(const char* argv0, char* out, size_t cap);

}  // namespace bimo

#endif  // BIMO_LINK_SIMPROC_H_

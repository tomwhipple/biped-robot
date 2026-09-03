#include "simproc.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

namespace bimo {
namespace {

bool exists(const char* path) {
    struct stat st;
    return stat(path, &st) == 0;
}

int cmpName(const void* a, const void* b) {
    return strcmp(static_cast<const char*>(a), static_cast<const char*>(b));
}

}  // namespace

void RunList::scan(const char* repo) {
    n = 0;
    char dir[512];
    snprintf(dir, sizeof dir, "%s/sim/runs", repo);
    DIR* d = opendir(dir);
    if (d == nullptr) return;
    dirent* e;
    while ((e = readdir(d)) != nullptr && n < kMaxRuns) {
        if (e->d_name[0] == '.') continue;
        char cfg[768], weights[768];
        snprintf(cfg, sizeof cfg, "%s/sim/runs/%s/config.json", repo, e->d_name);
        snprintf(weights, sizeof weights, "%s/sim/sil/weights/%s.silw.json",
                 repo, e->d_name);
        if (!exists(cfg) || !exists(weights)) continue;
        snprintf(name[n], sizeof name[0], "%s", e->d_name);
        ++n;
    }
    closedir(d);
    qsort(name, static_cast<size_t>(n), sizeof name[0], cmpName);
}

int RunList::indexOf(const char* want) const {
    for (int i = 0; i < n; ++i) {
        if (!strcmp(name[i], want)) return i;
    }
    return -1;
}

void SimProc::pushLine(const char* s) {
    snprintf(log[log_n % kSimLogLines], kSimLogCols, "%s", s);
    ++log_n;
    // "sil_twin: <run> on :<port> ... plant=<xml> spec=..." -- keep the part
    // up to the spec blob, which is the operator-legible half.
    if (!strncmp(s, "sil_twin: ", 10) && strstr(s, "plant=") != nullptr) {
        snprintf(ident, sizeof ident, "%s", s + 10);
        char* spec = strstr(ident, "  spec=");
        if (spec != nullptr) *spec = 0;
    }
}

int SimProc::lines() const {
    return log_n < kSimLogLines ? log_n : kSimLogLines;
}

const char* SimProc::line(int i) const {
    const int nl = lines();
    if (i < 0 || i >= nl) return "";
    const int first = log_n - nl;
    return log[(first + i) % kSimLogLines];
}

bool SimProc::start(const char* repo, const char* run_name, bool hang,
                    bool viewer, int stream_port, int cmd_port, int tlm_port) {
    if (running()) return true;
    err[0] = 0;
    char python[512], script[512];
    snprintf(python, sizeof python, "%s/.venv/bin/python", repo);
    snprintf(script, sizeof script, "%s/sim/sil_twin.py", repo);
    if (!exists(python)) {
        snprintf(err, sizeof err, "no interpreter at %s", python);
        return false;
    }
    if (!exists(script)) {
        snprintf(err, sizeof err, "no %s", script);
        return false;
    }

    int fds[2], cmd_fds[2];
    if (pipe(fds) != 0) {
        snprintf(err, sizeof err, "pipe: %s", strerror(errno));
        return false;
    }
    if (pipe(cmd_fds) != 0) {
        snprintf(err, sizeof err, "pipe: %s", strerror(errno));
        ::close(fds[0]);
        ::close(fds[1]);
        return false;
    }

    char sp[16], cp[16], tp[16];
    snprintf(sp, sizeof sp, "%d", stream_port);
    snprintf(cp, sizeof cp, "%d", cmd_port);
    snprintf(tp, sizeof tp, "%d", tlm_port);

    const pid_t child = fork();
    if (child < 0) {
        snprintf(err, sizeof err, "fork: %s", strerror(errno));
        ::close(fds[0]);
        ::close(fds[1]);
        ::close(cmd_fds[0]);
        ::close(cmd_fds[1]);
        return false;
    }
    if (child == 0) {
        ::close(fds[0]);
        ::close(cmd_fds[1]);
        dup2(cmd_fds[0], STDIN_FILENO);
        ::close(cmd_fds[0]);
        dup2(fds[1], STDOUT_FILENO);
        dup2(fds[1], STDERR_FILENO);
        ::close(fds[1]);
        // Its own process group, so stop() cannot signal this console and a
        // ctrl-C in the launching terminal does not race us to the child.
        setpgid(0, 0);
        // Run from sim/. A run config's xml_path is often a BARE filename
        // (loco_v11gait: "bimo_biped_v3yaw.xml"), which MuJoCo resolves against
        // the cwd -- so spawning from the repo root kills every plant except
        // the --hang one, whose path sil_twin builds absolutely. Found by
        // ticking the hang box off.
        char simdir[600];
        snprintf(simdir, sizeof simdir, "%s/sim", repo);
        if (chdir(simdir) != 0) {
            fprintf(stderr, "cannot enter %s: %s\n", simdir, strerror(errno));
            _exit(126);
        }
        const char* argv[13];
        int a = 0;
        argv[a++] = python;
        argv[a++] = script;
        argv[a++] = "--run-name";
        argv[a++] = run_name;
        argv[a++] = "--stream-port";
        argv[a++] = sp;
        argv[a++] = "--port";
        argv[a++] = cp;
        argv[a++] = "--tlm-port";
        argv[a++] = tp;
        if (hang) argv[a++] = "--hang";
        if (viewer) argv[a++] = "--viewer";
        argv[a] = nullptr;
        execv(python, const_cast<char**>(argv));
        // execv only returns on failure, and this is a forked child: say so
        // down the pipe the parent is already reading, then leave.
        fprintf(stderr, "exec %s failed: %s\n", python, strerror(errno));
        _exit(127);
    }

    ::close(fds[1]);
    ::close(cmd_fds[0]);
    fcntl(fds[0], F_SETFL, O_NONBLOCK);
    out_fd = fds[0];
    in_fd = cmd_fds[1];
    pid = child;
    partial_len = 0;
    ident[0] = 0;
    char msg[256];
    snprintf(msg, sizeof msg, "$ sil_twin.py --run-name %s%s --stream-port %s",
             run_name, hang ? " --hang" : "", sp);
    pushLine(msg);
    return true;
}

void SimProc::stop() {
    if (pid > 0) {
        kill(-pid, SIGTERM);          // the group; it set its own
        kill(pid, SIGTERM);
        for (int i = 0; i < 200; ++i) {
            int status = 0;
            const pid_t r = waitpid(pid, &status, WNOHANG);
            if (r == pid || (r < 0 && errno == ECHILD)) break;
            usleep(10 * 1000);
        }
        // Still there after 2 s: it is not going to leave politely.
        int status = 0;
        if (waitpid(pid, &status, WNOHANG) == 0) {
            kill(pid, SIGKILL);
            waitpid(pid, &status, 0);
        }
        pid = -1;
        err[0] = 0;                   // a deliberate stop is not a failure
        pushLine("-- sim stopped");
    }
    if (out_fd >= 0) {
        ::close(out_fd);
        out_fd = -1;
    }
    if (in_fd >= 0) {
        ::close(in_fd);
        in_fd = -1;
    }
}

bool SimProc::tell(const char* line) {
    if (in_fd < 0 || pid <= 0) return false;
    // The child may already be dead: poll() reaps at the END of a tick, so a
    // sim killed by a signal between ticks still looks alive here. Writing to
    // a pipe with no reader raises SIGPIPE, whose default action would kill
    // this console outright -- taking the link with it, on a robot that may be
    // armed, without ever sending the disarm. Reap first, then write with
    // SIGPIPE ignored (installed in main) and treat EPIPE as "it is gone".
    int status = 0;
    if (waitpid(pid, &status, WNOHANG) == pid) {
        pid = -1;
        pushLine("-- sim gone (noticed while writing to it)");
        return false;
    }
    char buf[192];
    const int n = snprintf(buf, sizeof buf, "%s\n", line);
    if (n <= 0 || static_cast<size_t>(n) >= sizeof buf) return false;
    const ssize_t w = write(in_fd, buf, static_cast<size_t>(n));
    if (w != n) {
        if (errno == EPIPE) {
            snprintf(err, sizeof err, "the sim closed its input");
            pushLine("-- sim closed its input");
        }
        return false;
    }
    return true;
}

void SimProc::poll() {
    if (out_fd >= 0) {
        char buf[4096];
        for (;;) {
            const ssize_t n = read(out_fd, buf, sizeof buf);
            if (n <= 0) break;
            for (ssize_t i = 0; i < n; ++i) {
                const char c = buf[i];
                if (c == '\n' || partial_len + 1 >= sizeof partial) {
                    partial[partial_len] = 0;
                    if (partial_len > 0) pushLine(partial);
                    partial_len = 0;
                    if (c != '\n') partial[partial_len++] = c;
                } else if (c != '\r') {
                    partial[partial_len++] = c;
                }
            }
        }
    }
    if (pid > 0) {
        int status = 0;
        const pid_t r = waitpid(pid, &status, WNOHANG);
        if (r == pid) {
            char msg[128];
            if (WIFEXITED(status)) {
                snprintf(msg, sizeof msg, "-- sim exited (status %d)",
                         WEXITSTATUS(status));
            } else if (WIFSIGNALED(status)) {
                snprintf(msg, sizeof msg, "-- sim killed (signal %d)",
                         WTERMSIG(status));
            } else {
                snprintf(msg, sizeof msg, "-- sim gone");
            }
            pushLine(msg);
            snprintf(err, sizeof err, "%s", msg + 3);
            pid = -1;
        }
    }
}

void guessRepoRoot(const char* argv0, char* out, size_t cap) {
    // firmware/host/build/bimo_gui -> three levels up.
    char path[1024];
    if (argv0 != nullptr && strchr(argv0, '/') != nullptr) {
        snprintf(path, sizeof path, "%s", argv0);
    } else {
        snprintf(path, sizeof path, "%s", "./bimo_gui");
    }
    char real[1024];
    if (realpath(path, real) == nullptr) {
        snprintf(real, sizeof real, "%s", path);
    }
    for (int i = 0; i < 4; ++i) {           // strip binary + 3 dirs
        char* slash = strrchr(real, '/');
        if (slash == nullptr) break;
        *slash = 0;
    }
    if (real[0] == 0) snprintf(real, sizeof real, ".");
    snprintf(out, cap, "%s", real);
}

}  // namespace bimo

# Git hooks

The repo ships a versioned pre-push hook that runs the full acceptance-gate
set locally before a push. GitHub Actions CI is disabled (2026-09-16), so the
hook is the **only** gate — everything CI used to check now runs on your
machine.

## Enable (once per clone)

```bash
git config core.hooksPath .githooks
```

This points git at the versioned `.githooks/` directory instead of the
per-clone `.git/hooks/` (which is not portable and does not survive a fresh
clone). Everyone who clones the repo runs the same hook.

## What it runs

The hook diffs each pushed ref against the remote's tip (a new branch
against its merge base with `origin/main`, renames counted as a delete and an
add) and runs only the gates the change can reach. It prints which gates ran
and why. A push it cannot diff runs everything.

- **Docs-only push**: every changed file is one no gate reads. That means
  anything under `docs/`, `*.md`, images (`png`, `jpg`, `gif`, `svg`,
  `webp`), PDFs, HTML reports (`night_summary.html`), `.github/` and
  `.claude/`. It runs no gate. Neither the test suites, the modules they
  import, nor the firmware build read these files; code mentions them only
  in comments, or writes them.
- **C/C++ push**: some other changed file is a C/C++ source or header
  anywhere, anything under `firmware/`, or the root `.clang-tidy`. It runs
  every gate.
- **Any other push** runs the Python gates. The suites read `sim/`, `cad/`
  (the plant's meshes), `tools/`, `link/`, `firmware/` sources and headers,
  and `experiments/`, so every non-docs file triggers both.

Python gates (every push but a docs-only one):

- **link-tests** — `python3 -m pytest tests/ -q`: protocol, sources, and the
  console e2e suite. **Hard** (needs pytest + numpy installed).
- **sil-suite** — `python3 -m pytest sim/sil -q`: the firmware's control code
  against the CPU plant. **Hard**; tests that need the deployed run's
  gitignored artifacts skip with a reason. On a push with no C/C++ changes the hook
  first builds only its library, `make -C firmware/host sil`.

C/C++ pushes only:

- **host check** — `make -C firmware/host check`: the ASan/UBSan host gate
  (the thing that caught the `mjpeg.cpp` format-truncation break) plus the
  `bimo_tui` console build. On macOS also `deps` + `gui`; on Linux the gui
  half is tolerant until the GUI port lands. **Hard.**
- **cmake-ctest** — configure/build/ctest of `firmware/host`, keeping
  CMakeLists.txt and the Makefile in step. **Hard when cmake is installed**;
  skips with a note when it isn't.
- **clang-tidy** (host) — `run-clang-tidy` over the host build with the
  repo's `.clang-tidy` (WarningsAsErrors). Runs when clang-tidy is installed.
- **clang-tidy-gui** — `firmware/host/clang-tidy-gui.sh` over the GUI/console
  sources. Runs when clang-tidy is installed (fetches the pinned imgui/stb
  deps first).
- **cppcheck** — `firmware/host/cppcheck.sh`, whole-program, fails on ANY
  finding. Runs when cppcheck is installed.
- **esp32-build** — `idf.py build` of the project in `firmware/` (the TARGET
  link; no flashing). Detection: `idf.py` on PATH, else `$IDF_PATH/export.sh`,
  else `~/.espressif/frameworks/esp-idf-v5.4`. Blocks the push when no
  toolchain is found unless `SKIP_ESP32=1` is set.

Toolchain-guarded gates (cmake, clang-tidy, cppcheck, esp32) print a SKIP
note when the tool is absent, and the C/C++ and Python gates print one when
the push does not reach them — a skip is visible in the hook output, never
silent.

## Escape hatches

```bash
git push --no-verify    # skip all gates (you know what you're doing)
SKIP_ESP32=1 git push   # skip just the esp32-build gate
PREPUSH_ALL=1 git push  # run every gate, whatever the push changes
```

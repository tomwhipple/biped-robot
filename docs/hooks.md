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

- **host check** — `make -C firmware/host check`: the ASan/UBSan host gate
  (the thing that caught the `mjpeg.cpp` format-truncation break) plus the
  `bimo_tui` console build. On macOS also `deps` + `gui`; on Linux the gui
  half is tolerant until the GUI port lands. **Hard, always.**
- **link-tests** — `python3 -m pytest tests/ -q`: protocol, sources, and the
  console e2e suite. **Hard, always** (needs pytest + numpy installed).
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
- **esp32-build** — `idf.py build` of `firmware/main` (the TARGET link; no
  flashing). Detection: `idf.py` on PATH, else `$IDF_PATH/export.sh`, else
  `~/.espressif/frameworks/esp-idf-v5.4`. Blocks the push when no toolchain
  is found unless `SKIP_ESP32=1` is set.

Toolchain-guarded gates (cmake, clang-tidy, cppcheck, esp32) print a SKIP
note when the tool is absent — a skip is visible in the hook output, never
silent.

## Escape hatches

```bash
git push --no-verify   # skip all gates (you know what you're doing)
SKIP_ESP32=1 git push  # skip just the esp32-build gate
```

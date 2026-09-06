# Git hooks

The repo ships a versioned pre-push hook that runs the fast local checks
before a push, so a broken build is caught on your laptop instead of burning
GitHub Actions minutes.

## Enable (once per clone)

```bash
git config core.hooksPath .githooks
```

This points git at the versioned `.githooks/` directory instead of the
per-clone `.git/hooks/` (which is not portable and does not survive a fresh
clone). Everyone who clones the repo runs the same hook.

## What it runs

The hook runs `make -C firmware/host check` once over the whole working tree
(not per-ref), then blocks the push if it fails. `make check` is the **same**
gate set CI's `host-tests` / `link-tests` / `console` jobs run, so a pass in
the hook cannot be a fail in CI (and vice versa) — the command list is never
forked between the two.

`make check` runs:

- `test` — the ASan/UBSan host gate (the exact thing that caught the
  `mjpeg.cpp` format-truncation break). **Hard, always.**
- `tui` — compiles `bimo_tui`. **Hard, always.**
- `deps` + `gui` — fetch/verify the pinned imgui/stb deps and compile
  `bimo_gui`. **Hard on macOS** (where the GUI builds today); **tolerant on
  Linux** until the Linux GUI port lands on main — a Linux dev without
  `libglfw3-dev` gets a warning, not a blocked push.

The slow/heavy gates (ESP-IDF build, clang-tidy, cppcheck, cmake-ctest) are
deliberately **not** in the hook — they stay in CI, where the toolchain and
minutes belong.

## Escape hatch

If you need to push past a failing check (and you know what you're doing):

```bash
git push --no-verify
```

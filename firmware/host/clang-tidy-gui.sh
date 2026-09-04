#!/usr/bin/env bash
# clang-tidy over the GUI sources (link/gui.cpp + its peers) on Linux.
#
# bimo_gui links OpenGL.framework and only builds on macOS, but its pure logic
# (client, keymap, recorder, mjpeg, simproc, protocol) already host-compiles,
# and gui.cpp itself is parseable on Linux once <OpenGL/gl3.h> is mapped onto
# imgui's own GL loader (see gl-shim/OpenGL/gl3.h). This script runs clang-tidy
# over the whole GUI translation-unit set so the window client gets the same
# static-analysis gate as the firmware, without a macOS runner.
#
# Requires: `make -C firmware/host deps` already run (fetches the pinned imgui
# + stb into third_party/), and a compile_commands.json from the host build.
set -euo pipefail

HOST_DIR="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HOST_DIR/../.." && pwd)"
BUILD="$HOST_DIR/build"

if [ ! -f "$BUILD/compile_commands.json" ]; then
    echo "clang-tidy-gui: $BUILD/compile_commands.json missing; run cmake first" >&2
    exit 1
fi

# The GUI sources, in the order the Makefile links them. gui.cpp is the only
# one that needs the GL shim; the rest are pure and already in the host build.
GUI_SRCS=(
    "$ROOT/link/gui.cpp"
    "$ROOT/link/client.cpp"
    "$ROOT/link/keymap.cpp"
    "$ROOT/link/recorder.cpp"
    "$ROOT/link/mjpeg.cpp"
    "$ROOT/link/simproc.cpp"
)

# gui.cpp is not in the host compile_commands.json (it only links on macOS),
# so it is analyzed with an explicit command line. The include paths mirror
# the Makefile's GUI_INC plus the gl-shim that stands in for <OpenGL/gl3.h>.
GUI_INC=(
    -I"$HOST_DIR"
    -I"$ROOT/firmware/components/linkproto/include"
    -I"$ROOT/firmware/components/obs/include"
    -I"$ROOT/firmware/components/policy/include"
    -I"$ROOT/firmware/components/imu/include"
    -I"$ROOT/firmware/components/battguard/include"
    -I"$ROOT/firmware/components/scsbus/include"
    -I"$ROOT/link"
    -I"$ROOT/third_party"
    -I"$ROOT/third_party/imgui"
    -I"$ROOT/third_party/imgui/backends"
    -I"$HOST_DIR/gl-shim"
)

rc=0
for src in "${GUI_SRCS[@]}"; do
    if [ "$src" = "$ROOT/link/gui.cpp" ]; then
        clang-tidy -p "$BUILD" "$src" -- -std=c++17 "${GUI_INC[@]}" \
            -DGL_SILENCE_DEPRECATION || rc=1
    else
        clang-tidy -p "$BUILD" "$src" || rc=1
    fi
done

exit "$rc"

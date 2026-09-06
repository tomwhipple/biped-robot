// Linux shim for macOS's <OpenGL/gl3.h>, used ONLY by clang-tidy to parse
// link/gui.cpp on the CI host (see firmware/host/clang-tidy-gui.sh).
//
// gui.cpp includes <OpenGL/gl3.h> because bimo_gui links OpenGL.framework on
// macOS. On Linux that header does not exist, so this shim maps it onto
// imgui's own GL loader (imgui_impl_opengl3_loader.h), which declares the
// same core-profile symbols gui.cpp uses (glGenTextures, glTexImage2D, ...).
//
// This header is never compiled into a binary: bimo_gui still links the real
// OpenGL.framework on macOS, and the shim only exists so a Linux clang-tidy
// run can parse the GUI sources without a macOS toolchain.
#ifndef BIMO_GL_SHIM_GL3_H_
#define BIMO_GL_SHIM_GL3_H_

#include "imgui_impl_opengl3_loader.h"

#endif  // BIMO_GL_SHIM_GL3_H_

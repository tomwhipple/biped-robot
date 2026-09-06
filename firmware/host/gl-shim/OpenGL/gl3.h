// Linux shim for macOS's <OpenGL/gl3.h>.
//
// gui.cpp includes <OpenGL/gl3.h> because bimo_gui links OpenGL.framework on
// macOS. On Linux that header does not exist, so this shim maps it onto
// imgui's own GL loader (imgui_impl_opengl3_loader.h), which declares the
// same core-profile symbols gui.cpp uses (glGenTextures, glTexImage2D, ...).
//
// Two consumers, one header:
//   * clang-tidy parses link/gui.cpp on the CI host through this shim
//     (see firmware/host/clang-tidy-gui.sh).
//   * `make gui` on Linux compiles gui.cpp against it for real: the loader
//     header's gl* symbols are #define'd onto imgl3wProcs.gl.*, which
//     imgui_impl_opengl3.cpp (already in the imgui objects) defines and fills
//     in at runtime via glfwGetProcAddress. So the shim is not just a parse
//     stand-in -- it is what makes bimo_gui LINK on Linux.
//
// On macOS this header is never seen: the Darwin branch of the Makefile does
// not add -Igl-shim, so gui.cpp gets the real <OpenGL/gl3.h>.
#ifndef BIMO_GL_SHIM_GL3_H_
#define BIMO_GL_SHIM_GL3_H_

#include "imgui_impl_opengl3_loader.h"

#endif  // BIMO_GL_SHIM_GL3_H_

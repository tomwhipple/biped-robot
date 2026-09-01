// The one thing bimo_gui needs from stb_image: a JPEG off the sim's MJPEG
// stream, decoded to RGBA for a GL texture. Declared here and implemented in
// link/stb_impl.cpp so the third-party header is compiled exactly once, in a
// translation unit of its own with warnings off -- our sources keep -Werror
// -Wconversion, which no vendored single-header library survives.
#ifndef BIMO_LINK_JPEG_H_
#define BIMO_LINK_JPEG_H_

#include <stddef.h>

namespace bimo {

// RGBA, 4 bytes per pixel. Null on a frame that is not a JPEG.
unsigned char* decodeJpeg(const unsigned char* data, size_t len, int* w,
                          int* h);
void freePixels(unsigned char* p);

}  // namespace bimo

#endif  // BIMO_LINK_JPEG_H_

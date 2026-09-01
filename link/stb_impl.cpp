// stb_image, instantiated once. Compiled with warnings off (see the Makefile's
// $(BUILD)/thirdparty rule); nothing of ours lives in this file.
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#include "stb_image.h"

#include "jpeg.h"

namespace bimo {

unsigned char* decodeJpeg(const unsigned char* data, size_t len, int* w,
                          int* h) {
    int comp = 0;
    return stbi_load_from_memory(data, (int)len, w, h, &comp, 4);
}

void freePixels(unsigned char* p) { stbi_image_free(p); }

}  // namespace bimo

#include "image_loader.h"
#include <cstdlib>
#include <cstring>
#include <jpeglib.h>
#include <png.h>
#include <setjmp.h>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
struct JpegError {
  jpeg_error_mgr base;
  jmp_buf jump;
  char text[JMSG_LENGTH_MAX];
};
void jpegError(j_common_ptr c) {
  auto *error = reinterpret_cast<JpegError *>(c->err);
  c->err->format_message(c, error->text);
  longjmp(error->jump, 1);
}
struct JpegState {
  jpeg_decompress_struct decoder{};
  JpegError error{};
  FILE *file = nullptr;
  unsigned char *pixels = nullptr;
  unsigned char *row = nullptr;
  bool created = false;
  ~JpegState() {
    if (created)
      jpeg_destroy_decompress(&decoder);
    if (file)
      fclose(file);
    free(row);
    free(pixels);
  }
};
bool jpeg(ImageTexture &image, const char *path) {
  auto *state = new JpegState;
  state->file = fopen(path, "rb");
  if (!state->file) {
    delete state;
    throw std::runtime_error(std::string("Cannot open JPEG: ") + path);
  }
  state->decoder.err = jpeg_std_error(&state->error.base);
  state->error.base.error_exit = jpegError;
  if (setjmp(state->error.jump)) {
    std::string message = state->error.text;
    delete state;
    throw std::runtime_error(std::string("JPEG decode failed: ") + path + ": " + message);
  }
  jpeg_create_decompress(&state->decoder);
  state->created = true;
  jpeg_stdio_src(&state->decoder, state->file);
  jpeg_read_header(&state->decoder, TRUE);
  state->decoder.out_color_space = JCS_RGB;
  jpeg_start_decompress(&state->decoder);
  size_t w = state->decoder.output_width, h = state->decoder.output_height;
  if (!w || !h || w * h > 100000000) {
    delete state;
    throw std::runtime_error("Invalid JPEG dimensions");
  }
  state->pixels = static_cast<unsigned char *>(malloc(w * h * 4));
  state->row = static_cast<unsigned char *>(malloc(w * 3));
  if (!state->pixels || !state->row) {
    delete state;
    throw std::bad_alloc();
  }
  while (state->decoder.output_scanline < h) {
    size_t y = h - 1 - state->decoder.output_scanline;
    JSAMPROW row = state->row;
    jpeg_read_scanlines(&state->decoder, &row, 1);
    for (size_t x = 0; x < w; ++x) {
      auto *out = state->pixels + 4 * (y * w + x);
      memcpy(out, row + 3 * x, 3);
      out[3] = 255;
    }
  }
  jpeg_finish_decompress(&state->decoder);
  image.w = w;
  image.h = h;
  image.imageData = state->pixels;
  state->pixels = nullptr;
  delete state;
  return true;
}
bool png(ImageTexture &image, const char *path) {
  png_image p{};
  p.version = PNG_IMAGE_VERSION;
  if (!png_image_begin_read_from_file(&p, path)) {
    std::string message = p.message;
    png_image_free(&p);
    throw std::runtime_error("PNG decode failed: " + message);
  }
  p.format = PNG_FORMAT_RGB;
  if (!p.width || !p.height || size_t(p.width) * p.height > 100000000) {
    png_image_free(&p);
    throw std::runtime_error("Invalid PNG dimensions");
  }
  std::vector<unsigned char> rgb(PNG_IMAGE_SIZE(p));
  if (!png_image_finish_read(&p, nullptr, rgb.data(), 0, nullptr)) {
    std::string message = p.message;
    png_image_free(&p);
    throw std::runtime_error("PNG decode failed: " + message);
  }
  image.w = p.width;
  image.h = p.height;
  image.imageData = static_cast<unsigned char *>(malloc(size_t(image.w) * image.h * 4));
  if (!image.imageData) {
    png_image_free(&p);
    throw std::bad_alloc();
  }
  for (size_t y = 0; y < image.h; ++y)
    for (size_t x = 0; x < image.w; ++x) {
      auto *out = image.imageData + 4 * ((image.h - 1 - y) * image.w + x);
      memcpy(out, rgb.data() + 3 * (y * image.w + x), 3);
      out[3] = 255;
    }
  png_image_free(&p);
  return true;
}
} 
bool loadGpuImage(ImageTexture &image, const char *path) {
  if (getenv("GPU_MAGICK_IMAGES"))
    return false;
  const char *dot = strrchr(path, '.');
  if (!dot)
    return false;
  std::string ext(dot);
  for (auto &c : ext)
    if (c >= 'A' && c <= 'Z')
      c += 32;
  if (ext == ".jpg" || ext == ".jpeg")
    return jpeg(image, path);
  if (ext == ".png")
    return png(image, path);
  return false;
}

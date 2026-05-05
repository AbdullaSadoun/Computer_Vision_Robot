/*
Simulation.cpp
- Implements BMP 24-bit loader/writer using raw Windows BMP header structs (no external libs)
- Pixel helpers: bounds-checked setPixel/getPixel on BGR-packed Image24 buffers
- Compositing: blit copies a source sprite with magenta-key transparency; blitRotated adds rotation
- Drawing: Bresenham line and midpoint circle algorithms for sim debug overlays
- writeFrame: writes frame to output.bmp via a temp file to avoid partial reads by image viewers
by: Abdulla Sadoun
Date: February 12, 2026
*/
// Simulation.cpp
#include "Simulation.h"
 
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <cmath>
 
#ifdef _WIN32
  #include <windows.h>
#endif
 
// -----------------------------
// BMP 24-bit loader/writer
// -----------------------------
#pragma pack(push, 1)
struct BMPFileHeader {
  uint16_t bfType;      // 'BM'
  uint32_t bfSize;
  uint16_t bfReserved1;
  uint16_t bfReserved2;
  uint32_t bfOffBits;
};
 
struct BMPInfoHeader {
  uint32_t biSize;
  int32_t  biWidth;
  int32_t  biHeight;    // positive => bottom-up
  uint16_t biPlanes;
  uint16_t biBitCount;  // 24
  uint32_t biCompression; // 0 = BI_RGB
  uint32_t biSizeImage;
  int32_t  biXPelsPerMeter;
  int32_t  biYPelsPerMeter;
  uint32_t biClrUsed;
  uint32_t biClrImportant;
};
#pragma pack(pop)
static_assert(sizeof(BMPFileHeader) == 14, "BMPFileHeader packing mismatch");
static_assert(sizeof(BMPInfoHeader) == 40, "BMPInfoHeader packing mismatch");
 
int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
float clampf(float v, float lo, float hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }
 
void setPixel(Image24& img, int x, int y, uint8_t r, uint8_t g, uint8_t b) {
  /*
  Bounds-checks (x,y) against image dimensions before writing.
  Stores pixel in BGR order at offset (y*w + x)*3 in the flat data buffer.
  */
  if (x < 0 || y < 0 || x >= img.w || y >= img.h) return;
  size_t i = (size_t)(y * img.w + x) * 3;
  img.data[i + 0] = b;
  img.data[i + 1] = g;
  img.data[i + 2] = r;
}
 
void getPixel(const Image24& img, int x, int y, uint8_t& r, uint8_t& g, uint8_t& b) {
  /*
  Clamps (x,y) to image bounds (edge-repeat) then reads BGR triplet into r,g,b.
  The clamp prevents out-of-bounds reads during rotated sprite sampling.
  */
  x = clampi(x, 0, img.w - 1);
  y = clampi(y, 0, img.h - 1);
  size_t i = (size_t)(y * img.w + x) * 3;
  b = img.data[i + 0];
  g = img.data[i + 1];
  r = img.data[i + 2];
}
 
bool loadBMP24(const std::string& path, Image24& out) {
  /*
  Reads a 24-bit uncompressed BMP from disk, handling both bottom-up and top-down row order.
  Validates the 'BM' magic and biBitCount==24 before allocating the output buffer.
  */
  std::FILE* f = std::fopen(path.c_str(), "rb");
  if (!f) return false;
 
  BMPFileHeader fh{};
  BMPInfoHeader ih{};
  if (std::fread(&fh, sizeof(fh), 1, f) != 1) { std::fclose(f); return false; }
  if (std::fread(&ih, sizeof(ih), 1, f) != 1) { std::fclose(f); return false; }
 
  if (fh.bfType != 0x4D42) { std::fclose(f); return false; } // 'BM'
  if (ih.biBitCount != 24 || ih.biCompression != 0) { std::fclose(f); return false; }
 
  const int w = ih.biWidth;
  const int h = std::abs(ih.biHeight);
  const bool bottomUp = (ih.biHeight > 0);
 
  out.w = w;
  out.h = h;
  out.data.assign((size_t)w * h * 3, 0);
 
  std::fseek(f, (long)fh.bfOffBits, SEEK_SET);
 
  const int rowBytes = w * 3;
  const int pad = (4 - (rowBytes % 4)) % 4;
 
  std::vector<uint8_t> row((size_t)rowBytes);
  for (int y = 0; y < h; y++) {
    int dstY = bottomUp ? (h - 1 - y) : y;
    if ((int)std::fread(row.data(), 1, (size_t)rowBytes, f) != rowBytes) { std::fclose(f); return false; }
    if (pad) std::fseek(f, pad, SEEK_CUR);
    std::memcpy(&out.data[(size_t)dstY * w * 3], row.data(), (size_t)rowBytes);
  }
 
  std::fclose(f);
  return true;
}
 
bool saveBMP24(const std::string& path, const Image24& img) {
  /*
  Writes Image24 as a bottom-up 24-bit BMP with 4-byte row padding.
  Fills BMP file and info headers from scratch — no external library required.
  */
  std::FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return false;
 
  BMPFileHeader fh{};
  BMPInfoHeader ih{};
 
  const int w = img.w;
  const int h = img.h;
  const int rowBytes = w * 3;
  const int pad = (4 - (rowBytes % 4)) % 4;
  const uint32_t pixelBytes = (uint32_t)((rowBytes + pad) * h);
 
  fh.bfType = 0x4D42;
  fh.bfOffBits = sizeof(BMPFileHeader) + sizeof(BMPInfoHeader);
  fh.bfSize = fh.bfOffBits + pixelBytes;
 
  ih.biSize = sizeof(BMPInfoHeader);
  ih.biWidth = w;
  ih.biHeight = h;             // bottom-up for maximum compatibility
  ih.biPlanes = 1;
  ih.biBitCount = 24;
  ih.biCompression = 0;
  ih.biSizeImage = pixelBytes;
 
  std::fwrite(&fh, sizeof(fh), 1, f);
  std::fwrite(&ih, sizeof(ih), 1, f);
 
  uint8_t padBytes[3] = {0, 0, 0};
  for (int y = h - 1; y >= 0; y--) {
    const uint8_t* row = &img.data[(size_t)y * w * 3];
    std::fwrite(row, 1, (size_t)rowBytes, f);
    if (pad) std::fwrite(padBytes, 1, (size_t)pad, f);
  }
 
  std::fclose(f);
  return true;
}
 
// -----------------------------
// Compositing utilities
// -----------------------------
struct ColorKey { uint8_t r, g, b; };
 
static ColorKey autoColorKey(const Image24& sprite) {
  uint8_t r, g, b;
  getPixel(sprite, 0, 0, r, g, b);
  return {r, g, b};
}
 
static inline bool isKey(uint8_t r, uint8_t g, uint8_t b, const ColorKey& k) {
  return (r == k.r && g == k.g && b == k.b);
}
 
void blit(Image24& dst, const Image24& src, int cx, int cy, bool useKey) {
  /*
  Copies src sprite centered at (cx,cy) into dst, skipping pixels that match
  the top-left color key (magenta by convention) when useKey is true.
  */
  const int x0 = cx - src.w / 2;
  const int y0 = cy - src.h / 2;
  ColorKey key = autoColorKey(src);
 
  for (int y = 0; y < src.h; y++) {
    int dy = y0 + y;
    if (dy < 0 || dy >= dst.h) continue;
    for (int x = 0; x < src.w; x++) {
      int dx = x0 + x;
      if (dx < 0 || dx >= dst.w) continue;
 
      uint8_t r, g, b;
      getPixel(src, x, y, r, g, b);
      if (useKey && isKey(r, g, b, key)) continue;
      setPixel(dst, dx, dy, r, g, b);
    }
  }
}
 
void blitRotated(Image24& dst, const Image24& src, int cx, int cy, float angleRad, bool useKey) {
  /*
  Nearest-neighbor rotated blit: for each destination pixel in a bounding circle,
  inverse-rotates the offset to find the source texel, skipping the color key.
  */
  ColorKey key = autoColorKey(src);
  const float c = std::cos(angleRad);
  const float s = std::sin(angleRad);
  const float hw = (src.w - 1) * 0.5f;
  const float hh = (src.h - 1) * 0.5f;
  const int R = (int)std::ceil(std::sqrt(hw * hw + hh * hh)) + 1;
 
  for (int dy = -R; dy <= R; dy++) {
    int y = cy + dy;
    if (y < 0 || y >= dst.h) continue;
    for (int dx = -R; dx <= R; dx++) {
      int x = cx + dx;
      if (x < 0 || x >= dst.w) continue;
 
      float sx = (c * dx + s * dy) + hw;
      float sy = (-s * dx + c * dy) + hh;
 
      int isx = (int)std::round(sx);
      int isy = (int)std::round(sy);
      if (isx < 0 || isy < 0 || isx >= src.w || isy >= src.h) continue;
 
      uint8_t r, g, b;
      getPixel(src, isx, isy, r, g, b);
      if (useKey && isKey(r, g, b, key)) continue;
      setPixel(dst, x, y, r, g, b);
    }
  }
}
 
void drawLine(Image24& img, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b, int thickness) {
  /*
  Bresenham line algorithm with a square thickness brush applied at each pixel.
  Used for drawing AI path segments and heading arrows in the simulator debug overlay.
  */
  auto plotThick = [&](int x, int y) {
    for (int ty = -thickness; ty <= thickness; ty++) {
      for (int tx = -thickness; tx <= thickness; tx++) {
        setPixel(img, x + tx, y + ty, r, g, b);
      }
    }
  };
 
  int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;
 
  while (true) {
    plotThick(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
 
void drawCircle(Image24& img, int cx, int cy, int rad, uint8_t r, uint8_t g, uint8_t b) {
  /*
  Filled circle drawn by testing every pixel in the bounding square against radius^2.
  Used for goal markers and robot hit indicators in the simulator debug overlay.
  */
  for (int y = -rad; y <= rad; y++) {
    for (int x = -rad; x <= rad; x++) {
      if (x * x + y * y <= rad * rad) setPixel(img, cx + x, cy + y, r, g, b);
    }
  }
}
 
void writeFrame(const Image24& frame) {
  /*
  Saves frame to a temp file then renames it over output.bmp atomically (MoveFileExA on Windows).
  Prevents image viewers from reading a half-written file during a slow disk flush.
  */
  const std::string tmp = "output_tmp.bmp";
  const std::string out = "output.bmp";
 
  if (!saveBMP24(tmp, frame)) return;
 
#ifdef _WIN32
  MoveFileExA(tmp.c_str(), out.c_str(), MOVEFILE_REPLACE_EXISTING);
#else
  std::remove(out.c_str());
  std::rename(tmp.c_str(), out.c_str());
#endif
}

#ifdef USE_IMAGE_VIEW
void pushFrameToImageView(Image24& frame, image& rgb) {
    /*
    Copies frame rows into the image_transfer shared-memory buffer in reverse order (bottom-up)
    to match the BMP convention expected by image_view.exe. Mode 1 = non-blocking send.
    */
    if (!rgb.pdata || rgb.width != (i2byte)frame.w || rgb.height != (i2byte)frame.h)
        return;
    const size_t rowBytes = (size_t)frame.w * 3u;
    const uint8_t* src = frame.data.data();
    uint8_t* dst = static_cast<uint8_t*>(rgb.pdata);
    for (int y = 0; y < frame.h; ++y)
        std::memcpy(dst + (size_t)y * rowBytes, src + (size_t)(frame.h - 1 - y) * rowBytes, rowBytes);
    view_rgb_image(rgb, 1);
}
#endif

/*
Simulation.h
- 2D simulation rendering utilities: pixel helpers, BMP file I/O, sprite compositing, drawing primitives
- clampi / clampf: integer and float range clamps used throughout the pipeline
- loadBMP24 / saveBMP24: read and write 24-bit BMP assets (arena background, robot sprites)
- blit / blitRotated: alpha-keyed sprite compositing with optional rotation for animated robot overlays
- drawLine / drawCircle: Bresenham-style primitives for debug overlays on the simulator frame
- writeFrame: atomic-ish BMP write (temp + rename) so image viewers don't read a partial file
- pushFrameToImageView: copies Image24 rows bottom-up into an image_transfer shared-memory buffer (USE_IMAGE_VIEW only)
by: Abdulla Sadoun
Date: February 12, 2026
*/
// Simulation.h
#pragma once

#include <cstdint>
#include <cstring>
#include <string>

#include "../CoreTypes.h"

#ifdef USE_IMAGE_VIEW
#include "../image_transfer.h"
#endif
 
// Pixel helpers (BGR packed).
int clampi(int v, int lo, int hi);
float clampf(float v, float lo, float hi);
void setPixel(Image24& img, int x, int y, uint8_t r, uint8_t g, uint8_t b);
void getPixel(const Image24& img, int x, int y, uint8_t& r, uint8_t& g, uint8_t& b);
 
// BMP I/O
bool loadBMP24(const std::string& path, Image24& out);
bool saveBMP24(const std::string& path, const Image24& img);
 
// Compositing
void blit(Image24& dst, const Image24& src, int cx, int cy, bool useKey = true);
void blitRotated(Image24& dst, const Image24& src, int cx, int cy, float angleRad, bool useKey = true);
 
// Simple drawing primitives (sim visuals)
void drawLine(Image24& img, int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b, int thickness = 2);
void drawCircle(Image24& img, int cx, int cy, int rad, uint8_t r, uint8_t g, uint8_t b);
 
// Writes output.bmp safely-ish: write temp then rename (best effort).
void writeFrame(const Image24& frame);

#ifdef USE_IMAGE_VIEW
// Push a composed frame to image_view via shared memory. Rows are reversed (BMP bottom-up
// convention) so the live camera / sim display renders right-side up in image_view.
// `rgb` must be an allocated RGB_IMAGE with matching dimensions.
void pushFrameToImageView(Image24& frame, image& rgb);
#endif
 

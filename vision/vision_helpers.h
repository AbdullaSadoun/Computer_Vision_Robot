#pragma once

#include <vector>
#include "../image_transfer.h"
#include "VisionTypes.h"
#include "../CoreTypes.h"
#include "../Simulation/Safety.h"
#include "../Simulation/DebugOverlay.h"

// ── Perspective Warp ──────────────────────────────────────────────────────────

// Compute 3×3 homography H (row-major [9]) mapping src[i]→dst[i] for 4 points.
// Returns false if the system is degenerate (collinear corners, etc.).
bool computeHomography(const float src[4][2], const float dst[4][2], float H[9]);

// Inverse-mapped bilinear warp: fills dst using H applied to each dst pixel.
// dst must be pre-allocated as RGB_IMAGE with the desired output dimensions.
void warpImageLegacy(const image& src, image& dst, const float H[9]);

// Rectangular crop: copies [x0, x0+cropW) x [y0, y0+cropH) from src into dst.
// dst must be pre-allocated RGB_IMAGE with width=cropW, height=cropH.
void cropImageLegacy(const image& src, image& dst, int x0, int y0, int cropW, int cropH);

// ── Path Planner ─────────────────────────────────────────────────────────────

struct LegacyWaypoints {
    static constexpr int kMaxPts = 32;
    float x[kMaxPts]{};
    float y[kMaxPts]{};
    int   count = 0;
    bool  valid = false;
};

// BFS grid planner from (sx,sy) to (gx,gy).
// Obstacle cells: all cells within (obs.radius + robotRadius + safetyMargin).
// Cells outside the arena polygon (if latched) are also marked occupied.
// cellSz: pixels per grid cell (default 10).
LegacyWaypoints planPathLegacy(
    float sx, float sy, float gx, float gy,
    const std::vector<Obstacle>& obstacles,
    const ArenaInfo& arena,
    float robotRadius, float safetyMargin,
    int imgW, int imgH, int cellSz = 10);

// ── Image Transformations ─────────────────────────────────────────────────────
// Flip image in-place. flip_x mirrors horizontally, flip_y vertically.
// Applying both gives a 180° rotation (upside-down camera correction).
void flipImageLegacy(image& img, bool flip_x, bool flip_y);

// ── Drawing Primitives ────────────────────────────────────────────────────────

void legacySetPixelRGB(image& img, int x, int y, uint8_t R, uint8_t G, uint8_t B);
void legacyDrawCircle(image& img, int cx, int cy, int r, uint8_t R, uint8_t G, uint8_t B);
void legacyDrawFilledCircle(image& img, int cx, int cy, int r, uint8_t R, uint8_t G, uint8_t B);
void legacyDrawLine(image& img, int x0, int y0, int x1, int y1, uint8_t R, uint8_t G, uint8_t B);
// Arrow from (ax,ay) in direction angle (radians from +x axis), length len px.
void legacyDrawArrow(image& img, float ax, float ay, float angle, int len,
                     uint8_t R, uint8_t G, uint8_t B);

// ── Overlay Compositors ───────────────────────────────────────────────────────

// Annotate detected robots, arena outline, and obstacle circles.
// self_turret_rel: turret angle relative to body (radians).
void drawLegacyVisionOverlay(image& img, const GameState& gs,
                              float self_turret_rel = 0.0f);

// Draw keepout circles, AI path waypoints, and goal marker.
void drawLegacyDebugOverlay(image& img, const GameState& gs,
                             const WorldEnv& env, const AIDebug& dbg,
                             const LegacyWaypoints* path = nullptr);

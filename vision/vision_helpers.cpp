#include <cmath>
#include <cstring>
#include <cstdlib>
#include <vector>
#include <queue>
#include <algorithm>
#include <climits>

#include "../image_transfer.h"
#include "vision.h"
#include "vision_helpers.h"
#include "../Simulation/Safety.h"
#include "../Parameters.h"

// ── Perspective Warp ──────────────────────────────────────────────────────────

bool computeHomography(const float src[4][2], const float dst[4][2], float H[9]) {
    // Each point correspondence gives 2 linear equations in h0..h7 (h8=1).
    // Build 8x9 augmented matrix and solve with partial-pivot Gaussian elimination.
    double A[8][9];
    for (int i = 0; i < 4; ++i) {
        double sx = src[i][0], sy = src[i][1];
        double dx = dst[i][0], dy = dst[i][1];
        // row 2i:   sx*h0 + sy*h1 + h2 + 0 - dx*sx*h6 - dx*sy*h7 = dx
        A[2*i][0] = sx;  A[2*i][1] = sy;  A[2*i][2] = 1.0;
        A[2*i][3] = 0.0; A[2*i][4] = 0.0; A[2*i][5] = 0.0;
        A[2*i][6] = -dx*sx; A[2*i][7] = -dx*sy; A[2*i][8] = dx;
        // row 2i+1: 0 + sx*h3 + sy*h4 + h5 - dy*sx*h6 - dy*sy*h7 = dy
        A[2*i+1][0] = 0.0; A[2*i+1][1] = 0.0; A[2*i+1][2] = 0.0;
        A[2*i+1][3] = sx;  A[2*i+1][4] = sy;  A[2*i+1][5] = 1.0;
        A[2*i+1][6] = -dy*sx; A[2*i+1][7] = -dy*sy; A[2*i+1][8] = dy;
    }

    // Partial-pivot Gaussian elimination.
    for (int col = 0; col < 8; ++col) {
        // Find pivot.
        int pivot = col;
        double best = std::abs(A[col][col]);
        for (int row = col + 1; row < 8; ++row) {
            if (std::abs(A[row][col]) > best) { best = std::abs(A[row][col]); pivot = row; }
        }
        if (best < 1e-9) return false;  // degenerate

        // Swap rows.
        if (pivot != col)
            for (int j = 0; j <= 8; ++j) std::swap(A[col][j], A[pivot][j]);

        // Eliminate below.
        for (int row = col + 1; row < 8; ++row) {
            double factor = A[row][col] / A[col][col];
            for (int j = col; j <= 8; ++j)
                A[row][j] -= factor * A[col][j];
        }
    }

    // Back-substitution.
    double h[8];
    for (int row = 7; row >= 0; --row) {
        double s = A[row][8];
        for (int col = row + 1; col < 8; ++col) s -= A[row][col] * h[col];
        h[row] = s / A[row][row];
    }

    for (int i = 0; i < 8; ++i) H[i] = (float)h[i];
    H[8] = 1.0f;
    return true;
}

// Invert a 3×3 matrix using the adjugate method. Returns false if singular.
static bool invertMat3(const float M[9], float Minv[9]) {
    // Cofactors (transposed = adjugate).
    double m[9];
    for (int i = 0; i < 9; ++i) m[i] = M[i];

    double c00 = m[4]*m[8] - m[5]*m[7];
    double c01 = m[5]*m[6] - m[3]*m[8];
    double c02 = m[3]*m[7] - m[4]*m[6];
    double det = m[0]*c00 + m[1]*c01 + m[2]*c02;
    if (std::abs(det) < 1e-12) return false;

    double inv = 1.0 / det;
    Minv[0] = (float)(c00 * inv);
    Minv[1] = (float)((m[2]*m[7] - m[1]*m[8]) * inv);
    Minv[2] = (float)((m[1]*m[5] - m[2]*m[4]) * inv);
    Minv[3] = (float)(c01 * inv);
    Minv[4] = (float)((m[0]*m[8] - m[2]*m[6]) * inv);
    Minv[5] = (float)((m[2]*m[3] - m[0]*m[5]) * inv);
    Minv[6] = (float)(c02 * inv);
    Minv[7] = (float)((m[1]*m[6] - m[0]*m[7]) * inv);
    Minv[8] = (float)((m[0]*m[4] - m[1]*m[3]) * inv);
    return true;
}

void warpImageLegacy(const image& src, image& dst, const float H[9]) {
    if (!src.pdata || !dst.pdata) return;
    if (src.type != RGB_IMAGE || dst.type != RGB_IMAGE) return;

    float Hinv[9];
    if (!invertMat3(H, Hinv)) return;

    const int dW = dst.width;
    const int dH = dst.height;
    const int sW = src.width;
    const int sH = src.height;

    for (int dy = 0; dy < dH; ++dy) {
        for (int dx = 0; dx < dW; ++dx) {
            // Apply inverse homography to map dst pixel back to src.
            float sx_ = Hinv[0]*dx + Hinv[1]*dy + Hinv[2];
            float sy_ = Hinv[3]*dx + Hinv[4]*dy + Hinv[5];
            float w_  = Hinv[6]*dx + Hinv[7]*dy + Hinv[8];
            float sx  = sx_ / w_;
            float sy  = sy_ / w_;

            // Bilinear interpolation.
            int x0 = (int)sx, y0 = (int)sy;
            int x1 = x0 + 1, y1 = y0 + 1;
            float fx = sx - x0, fy = sy - y0;

            // Clamp to source boundaries.
            x0 = std::max(0, std::min(sW - 1, x0));
            x1 = std::max(0, std::min(sW - 1, x1));
            y0 = std::max(0, std::min(sH - 1, y0));
            y1 = std::max(0, std::min(sH - 1, y1));

            const ibyte* p00 = src.pdata + (y0 * sW + x0) * 3;
            const ibyte* p10 = src.pdata + (y0 * sW + x1) * 3;
            const ibyte* p01 = src.pdata + (y1 * sW + x0) * 3;
            const ibyte* p11 = src.pdata + (y1 * sW + x1) * 3;

            ibyte* out = dst.pdata + (dy * dW + dx) * 3;
            for (int ch = 0; ch < 3; ++ch) {
                float val = (1.0f - fx) * (1.0f - fy) * p00[ch]
                          +         fx  * (1.0f - fy) * p10[ch]
                          + (1.0f - fx) *         fy  * p01[ch]
                          +         fx  *         fy  * p11[ch];
                out[ch] = (ibyte)(int)val;
            }
        }
    }
}

void cropImageLegacy(const image& src, image& dst, int x0, int y0, int cropW, int cropH) {
    if (!src.pdata || !dst.pdata) return;
    if (src.type != RGB_IMAGE || dst.type != RGB_IMAGE) return;
    if (dst.width != (i2byte)cropW || dst.height != (i2byte)cropH) return;

    const int sW = src.width;
    const int rowBytes = cropW * 3;
    for (int row = 0; row < cropH; ++row) {
        int srcRow = y0 + row;
        if (srcRow < 0 || srcRow >= src.height) {
            memset(dst.pdata + row * rowBytes, 0, (size_t)rowBytes);
            continue;
        }
        const ibyte* srcPtr = src.pdata + (srcRow * sW + x0) * 3;
        memcpy(dst.pdata + row * rowBytes, srcPtr, (size_t)rowBytes);
    }
}

// ── BFS Path Planner ──────────────────────────────────────────────────────────

LegacyWaypoints planPathLegacy(float sx, float sy, float gx, float gy,
                                const std::vector<Obstacle>& obstacles,
                                const ArenaInfo& arena,
                                float robotRadius, float safetyMargin,
                                int imgW, int imgH, int cellSz) {
    LegacyWaypoints wp{};
    if (cellSz <= 0) cellSz = 10;

    const int gridW = (imgW + cellSz - 1) / cellSz;
    const int gridH = (imgH + cellSz - 1) / cellSz;
    const int gridN = gridW * gridH;

    std::vector<uint8_t> grid(gridN, 0);  // 0=free, 1=occupied

    float inflateR = robotRadius + safetyMargin;

    // Mark obstacle-inflated cells.
    for (const auto& obs : obstacles) {
        float r = (obs.img ? obs.img->w * 0.5f : obs.radius) + inflateR;
        int cx0 = (int)((obs.x - r) / cellSz);
        int cy0 = (int)((obs.y - r) / cellSz);
        int cx1 = (int)((obs.x + r) / cellSz);
        int cy1 = (int)((obs.y + r) / cellSz);
        for (int cy = cy0; cy <= cy1; ++cy) {
            for (int cx = cx0; cx <= cx1; ++cx) {
                if (cx < 0 || cx >= gridW || cy < 0 || cy >= gridH) continue;
                float pcx = (cx + 0.5f) * cellSz;
                float pcy = (cy + 0.5f) * cellSz;
                float dx  = pcx - obs.x;
                float dy  = pcy - obs.y;
                if (dx * dx + dy * dy < r * r)
                    grid[cy * gridW + cx] = 1;
            }
        }
    }

    // Mark cells outside the arena polygon as occupied.
    if (arena.has_polygon) {
        for (int cy = 0; cy < gridH; ++cy) {
            for (int cx = 0; cx < gridW; ++cx) {
                float pcx = (cx + 0.5f) * cellSz;
                float pcy = (cy + 0.5f) * cellSz;
                if (!point_in_polygon(pcx, pcy, arena.corners, 4))
                    grid[cy * gridW + cx] = 1;
            }
        }
    }

    // Start and goal cells.
    int startCx = std::max(0, std::min(gridW - 1, (int)(sx / cellSz)));
    int startCy = std::max(0, std::min(gridH - 1, (int)(sy / cellSz)));
    int goalCx  = std::max(0, std::min(gridW - 1, (int)(gx / cellSz)));
    int goalCy  = std::max(0, std::min(gridH - 1, (int)(gy / cellSz)));

    int startCell = startCy * gridW + startCx;
    int goalCell  = goalCy  * gridW + goalCx;

    if (grid[startCell] == 1 || grid[goalCell] == 1) {
        // Endpoints blocked; return a direct straight-line path as fallback.
        wp.x[0] = gx; wp.y[0] = gy; wp.count = 1; wp.valid = true;
        return wp;
    }

    if (startCell == goalCell) {
        wp.x[0] = gx; wp.y[0] = gy; wp.count = 1; wp.valid = true;
        return wp;
    }

    // BFS from goal to start (so we can trace path by following parent).
    std::vector<int> parent(gridN, -1);
    std::vector<bool> visited(gridN, false);
    std::queue<int> q;
    q.push(goalCell);
    visited[goalCell] = true;
    parent[goalCell]  = goalCell;
    bool found = false;

    const int dx4[4] = { 1, -1, 0,  0 };
    const int dy4[4] = { 0,  0, 1, -1 };

    while (!q.empty()) {
        int cur = q.front(); q.pop();
        if (cur == startCell) { found = true; break; }
        int curX = cur % gridW, curY = cur / gridW;
        for (int d = 0; d < 4; ++d) {
            int nx = curX + dx4[d], ny = curY + dy4[d];
            if (nx < 0 || nx >= gridW || ny < 0 || ny >= gridH) continue;
            int nb = ny * gridW + nx;
            if (visited[nb] || grid[nb] == 1) continue;
            visited[nb] = true;
            parent[nb]  = cur;
            q.push(nb);
        }
    }

    if (!found) {
        // No path: return direct goal as single waypoint.
        wp.x[0] = gx; wp.y[0] = gy; wp.count = 1; wp.valid = true;
        return wp;
    }

    // Trace path from start to goal following parent links.
    std::vector<int> rawPath;
    int cur = startCell;
    while (cur != goalCell && rawPath.size() < (size_t)(gridN + 1)) {
        rawPath.push_back(cur);
        cur = parent[cur];
    }
    rawPath.push_back(goalCell);

    // Convert cells to pixel centers, then thin collinear waypoints.
    // Always keep first, last, and any cell that turns.
    std::vector<std::pair<float,float>> pts;
    for (int idx : rawPath) {
        float px = (idx % gridW + 0.5f) * cellSz;
        float py = (idx / gridW + 0.5f) * cellSz;
        pts.emplace_back(px, py);
    }

    // Ramer-Douglas-Peucker-style: keep point if not collinear with neighbors.
    std::vector<std::pair<float,float>> thinned;
    thinned.push_back(pts.front());
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        float ax = pts[i].first  - thinned.back().first;
        float ay = pts[i].second - thinned.back().second;
        float bx = pts[i+1].first  - thinned.back().first;
        float by = pts[i+1].second - thinned.back().second;
        float cross = std::abs(ax * by - ay * bx);
        if (cross > 5.0f) thinned.push_back(pts[i]);
    }
    thinned.push_back(pts.back());

    // Copy into LegacyWaypoints (skip the start cell itself; we're already there).
    int count = 0;
    for (size_t i = 1; i < thinned.size() && count < LegacyWaypoints::kMaxPts; ++i) {
        wp.x[count] = thinned[i].first;
        wp.y[count] = thinned[i].second;
        ++count;
    }
    // Always make the last waypoint exactly the goal position.
    if (count > 0) {
        wp.x[count - 1] = gx;
        wp.y[count - 1] = gy;
    }
    wp.count = count;
    wp.valid = (count > 0);
    return wp;
}

// ── Image Transformations ─────────────────────────────────────────────────────

void flipImageLegacy(image& img, bool flip_x, bool flip_y) {
    if (!img.pdata || (!flip_x && !flip_y)) return;
    const int w = img.width, h = img.height;
    const int bpp = (img.type == RGB_IMAGE) ? 3 : (img.type == LABEL_IMAGE ? 2 : 1);

    if (flip_x && flip_y) {
        // 180° rotation: reverse the entire pixel sequence.
        int total = w * h;
        for (int i = 0, j = total - 1; i < j; ++i, --j)
            for (int b = 0; b < bpp; ++b)
                std::swap(img.pdata[i*bpp + b], img.pdata[j*bpp + b]);
    } else if (flip_y) {
        // Vertical flip: swap row y with row (h-1-y).
        const size_t rowBytes = (size_t)w * bpp;
        std::vector<ibyte> tmp(rowBytes);
        for (int y = 0; y < h / 2; ++y) {
            ibyte* rowA = img.pdata + (size_t)y       * rowBytes;
            ibyte* rowB = img.pdata + (size_t)(h-1-y) * rowBytes;
            memcpy(tmp.data(), rowA, rowBytes);
            memcpy(rowA, rowB, rowBytes);
            memcpy(rowB, tmp.data(), rowBytes);
        }
    } else {
        // Horizontal flip: mirror pixels within each row.
        for (int y = 0; y < h; ++y) {
            ibyte* row = img.pdata + (size_t)y * w * bpp;
            for (int x = 0; x < w / 2; ++x)
                for (int b = 0; b < bpp; ++b)
                    std::swap(row[x*bpp + b], row[(w-1-x)*bpp + b]);
        }
    }
}

// ── Drawing Primitives ────────────────────────────────────────────────────────

void legacySetPixelRGB(image& img, int x, int y, uint8_t R, uint8_t G, uint8_t B) {
    if (x < 0 || x >= img.width || y < 0 || y >= img.height) return;
    ibyte* p = img.pdata + (y * img.width + x) * 3;
    p[0] = (ibyte)B;
    p[1] = (ibyte)G;
    p[2] = (ibyte)R;
}

void legacyDrawCircle(image& img, int cx, int cy, int r,
                      uint8_t R, uint8_t G, uint8_t B) {
    if (r <= 0) { legacySetPixelRGB(img, cx, cy, R, G, B); return; }
    // Midpoint circle algorithm with 8-way symmetry.
    int x = r, y = 0, err = 0;
    while (x >= y) {
        legacySetPixelRGB(img, cx+x, cy+y, R,G,B);
        legacySetPixelRGB(img, cx+y, cy+x, R,G,B);
        legacySetPixelRGB(img, cx-y, cy+x, R,G,B);
        legacySetPixelRGB(img, cx-x, cy+y, R,G,B);
        legacySetPixelRGB(img, cx-x, cy-y, R,G,B);
        legacySetPixelRGB(img, cx-y, cy-x, R,G,B);
        legacySetPixelRGB(img, cx+y, cy-x, R,G,B);
        legacySetPixelRGB(img, cx+x, cy-y, R,G,B);
        if (err <= 0) { ++y; err += 2*y + 1; }
        if (err >  0) { --x; err -= 2*x + 1; }
    }
}

void legacyDrawFilledCircle(image& img, int cx, int cy, int r,
                             uint8_t R, uint8_t G, uint8_t B) {
    if (r <= 0) { legacySetPixelRGB(img, cx, cy, R, G, B); return; }
    for (int dy = -r; dy <= r; ++dy) {
        int half = (int)std::sqrt((float)(r*r - dy*dy));
        for (int dx = -half; dx <= half; ++dx)
            legacySetPixelRGB(img, cx+dx, cy+dy, R, G, B);
    }
}

void legacyDrawLine(image& img, int x0, int y0, int x1, int y1,
                    uint8_t R, uint8_t G, uint8_t B) {
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    while (true) {
        legacySetPixelRGB(img, x0, y0, R, G, B);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void legacyDrawArrow(image& img, float ax, float ay, float angle, int len,
                     uint8_t R, uint8_t G, uint8_t B) {
    float ex = ax + std::cos(angle) * len;
    float ey = ay + std::sin(angle) * len;
    legacyDrawLine(img, (int)ax, (int)ay, (int)ex, (int)ey, R, G, B);
    // Arrowhead: two lines at ±150° from the tip direction.
    float headLen = len * 0.25f;
    for (float side : {3.14159265f * 5.0f / 6.0f, -3.14159265f * 5.0f / 6.0f}) {
        float ha = angle + side;
        float hx = ex + std::cos(ha) * headLen;
        float hy = ey + std::sin(ha) * headLen;
        legacyDrawLine(img, (int)ex, (int)ey, (int)hx, (int)hy, R, G, B);
    }
}

// ── Color helpers ─────────────────────────────────────────────────────────────

static void markerColorRGB(MarkerColor c, uint8_t& R, uint8_t& G, uint8_t& B) {
    switch (c) {
        case MarkerColor::Black:  R=40;  G=40;  B=40;  return;
        case MarkerColor::Blue:   R=0;   G=0;   B=255; return;
        case MarkerColor::Green:  R=0;   G=200; B=0;   return;
        case MarkerColor::Orange: R=255; G=140; B=0;   return;
        case MarkerColor::Red:    R=255; G=0;   B=0;   return;
        default:                  R=200; G=200; B=200; return;
    }
}

// ── Overlay Compositors ───────────────────────────────────────────────────────

void drawLegacyVisionOverlay(image& img, const GameState& gs, float self_turret_rel) {
    // Arena outline.
    if (gs.arena.valid) {
        const auto& c = gs.arena.corners;
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            legacyDrawLine(img, (int)c[i].x, (int)c[i].y,
                               (int)c[j].x, (int)c[j].y,
                               255, 255, 0);
        }
    }

    // Obstacle circles (grey).
    for (const auto& vo : gs.obstacles) {
        if (!vo.valid) continue;
        legacyDrawCircle(img, (int)vo.x, (int)vo.y, (int)vo.radius, 180, 180, 180);
    }

    // Self robot.
    if (gs.self.pose.valid) {
        const float px = gs.self.pose.x;
        const float py = gs.self.pose.y;
        const float th = gs.self.pose.theta;

        legacyDrawCircle(img, (int)px, (int)py, 28, 255, 255, 255);
        legacyDrawArrow(img, px, py, th, 40, 255, 255, 255);  // heading

        // Turret arrow (body heading + turret offset).
        float turretAngle = th + self_turret_rel;
        legacyDrawArrow(img, px, py, turretAngle, 50, 255, 140, 0);

        // Front / rear marker circles.
        if (gs.self.frontMarker.valid) {
            uint8_t mr, mg, mb;
            markerColorRGB(gs.self.frontMarker.color, mr, mg, mb);
            legacyDrawCircle(img, (int)gs.self.frontMarker.x, (int)gs.self.frontMarker.y,
                             (int)gs.self.frontMarker.radius, mr, mg, mb);
        }
        if (gs.self.rearMarker.valid) {
            legacyDrawCircle(img, (int)gs.self.rearMarker.x, (int)gs.self.rearMarker.y,
                             (int)gs.self.rearMarker.radius, 0, 0, 255);
        }

        // Identity dot (5×5 white block via draw_point_rgb).
        draw_point_rgb(img, (int)px, (int)py, 255, 255, 255);
    }

    // Enemy robot.
    if (gs.enemy.pose.valid) {
        const float px = gs.enemy.pose.x;
        const float py = gs.enemy.pose.y;
        const float th = gs.enemy.pose.theta;

        legacyDrawCircle(img, (int)px, (int)py, 28, 0, 255, 255);
        legacyDrawArrow(img, px, py, th, 40, 0, 255, 255);

        if (gs.enemy.frontMarker.valid) {
            uint8_t mr, mg, mb;
            markerColorRGB(gs.enemy.frontMarker.color, mr, mg, mb);
            legacyDrawCircle(img, (int)gs.enemy.frontMarker.x, (int)gs.enemy.frontMarker.y,
                             (int)gs.enemy.frontMarker.radius, mr, mg, mb);
        }
        if (gs.enemy.rearMarker.valid) {
            legacyDrawCircle(img, (int)gs.enemy.rearMarker.x, (int)gs.enemy.rearMarker.y,
                             (int)gs.enemy.rearMarker.radius, 0, 0, 255);
        }

        draw_point_rgb(img, (int)px, (int)py, 0, 255, 255);
    }
}

void drawLegacyDebugOverlay(image& img, const GameState& gs,
                             const WorldEnv& env, const AIDebug& dbg,
                             const LegacyWaypoints* path) {
    // Obstacle keepout circles (orange).
    if (env.obstacles) {
        for (const auto& obs : *env.obstacles) {
            float r = (obs.img ? obs.img->w * 0.5f : obs.radius) + env.robot_radius + env.safety_margin;
            legacyDrawCircle(img, obs.x, obs.y, (int)r, 255, 140, 0);
        }
    }

    // Enemy keepout circle (cyan).
    if (env.other_robot) {
        float r = env.robot_radius * 2.0f + env.safety_margin;
        legacyDrawCircle(img, (int)env.other_robot->x, (int)env.other_robot->y,
                         (int)r, 0, 255, 255);
    }

    // AI path waypoints (yellow lines + dots).
    if (path && path->valid && path->count > 0) {
        float prevX = gs.self.pose.valid ? gs.self.pose.x : 0.0f;
        float prevY = gs.self.pose.valid ? gs.self.pose.y : 0.0f;
        for (int i = 0; i < path->count; ++i) {
            legacyDrawLine(img, (int)prevX, (int)prevY,
                               (int)path->x[i], (int)path->y[i], 255, 255, 0);
            draw_point_rgb(img, (int)path->x[i], (int)path->y[i], 255, 255, 0);
            prevX = path->x[i];
            prevY = path->y[i];
        }
    } else if (!dbg.path.empty()) {
        // Fall back to AIDebug path if no LegacyWaypoints provided.
        for (size_t i = 0; i + 1 < dbg.path.size(); ++i) {
            legacyDrawLine(img,
                (int)dbg.path[i].first,   (int)dbg.path[i].second,
                (int)dbg.path[i+1].first, (int)dbg.path[i+1].second,
                255, 255, 0);
            draw_point_rgb(img, (int)dbg.path[i].first, (int)dbg.path[i].second, 255, 255, 0);
        }
    }

    // Goal marker (cyan filled circle, r=6).
    if (dbg.goal_valid) {
        legacyDrawFilledCircle(img, (int)dbg.goal_x, (int)dbg.goal_y, 6, 0, 255, 255);
    }
}

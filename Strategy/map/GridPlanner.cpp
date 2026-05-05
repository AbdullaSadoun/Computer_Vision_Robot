#include "GridPlanner.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float dist2(float ax, float ay, float bx, float by) {
    const float dx = ax - bx;
    const float dy = ay - by;
    return dx * dx + dy * dy;
}

// Squared distance from point P to segment AB.
float distPointSegSq(float px, float py, float ax, float ay, float bx, float by) {
    const float abx = bx - ax;
    const float aby = by - ay;
    const float apx = px - ax;
    const float apy = py - ay;
    const float ab2 = abx * abx + aby * aby;
    if (ab2 < 1e-6f) return dist2(px, py, ax, ay);
    float t = (apx * abx + apy * aby) / ab2;
    t = std::max(0.0f, std::min(1.0f, t));
    const float qx = ax + t * abx;
    const float qy = ay + t * aby;
    return dist2(px, py, qx, qy);
}

float minDistToPolygonEdges(float x, float y, const float (*poly)[2], int n) {
    float best = std::numeric_limits<float>::max();
    for (int k = 0; k < n; ++k) {
        const int kn = (k + 1) % n;
        const float d = std::sqrt(distPointSegSq(x, y, poly[k][0], poly[k][1], poly[kn][0], poly[kn][1]));
        best = std::min(best, d);
    }
    return best;
}

bool cellCenterFree(float wx, float wy, const ArenaInfo& arena, const std::vector<Obstacle>& obstacles,
                    float robot_radius_px, float clearance_px, float polygon_edge_extra_px) {
    if (!arena.has_polygon) {
        return false;
    }
    if (!point_in_polygon(wx, wy, arena.corners, 4)) {
        return false;
    }
    const float edge_pad = robot_radius_px + clearance_px + polygon_edge_extra_px;
    if (minDistToPolygonEdges(wx, wy, arena.corners, 4) < edge_pad) {
        return false;
    }
    const float inflate = robot_radius_px + clearance_px;
    for (const auto& o : obstacles) {
        const float dx = wx - (float)o.x;
        const float dy = wy - (float)o.y;
        const float rr = o.radius + inflate;
        if (dx * dx + dy * dy < rr * rr) {
            return false;
        }
    }
    return true;
}

bool rayClearToTarget(float x0, float y0, float x1, float y1, const ArenaInfo& arena,
                      const std::vector<Obstacle>& obstacles, float inflate_for_ray, float los_step) {
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    if (dx * dx + dy * dy < 1e-6f) {
        return true;
    }
    const float frac_step = std::max(0.008f, std::min(0.1f, los_step));
    for (float t = 0.0f; t <= 1.0f; t += frac_step) {
        const float x = x0 + t * dx;
        const float y = y0 + t * dy;
        if (!arena.has_polygon || !point_in_polygon(x, y, arena.corners, 4)) {
            return false;
        }
        for (const auto& o : obstacles) {
            const float ox = x - (float)o.x;
            const float oy = y - (float)o.y;
            const float rr = o.radius + inflate_for_ray;
            if (ox * ox + oy * oy < rr * rr) {
                return false;
            }
        }
    }
    return true;
}

struct Grid {
    float origin_x = 0.0f;
    float origin_y = 0.0f;
    float cell = 8.0f;
    int gw = 0;
    int gh = 0;
    std::vector<uint8_t> blocked;  // 1 = blocked

    int idx(int x, int y) const { return y * gw + x; }
    bool inBounds(int x, int y) const { return x >= 0 && y >= 0 && x < gw && y < gh; }
    void worldToGrid(float wx, float wy, int& gx, int& gy) const {
        gx = (int)std::floor((wx - origin_x) / cell);
        gy = (int)std::floor((wy - origin_y) / cell);
        gx = std::max(0, std::min(gw - 1, gx));
        gy = std::max(0, std::min(gh - 1, gy));
    }
    void gridToWorld(int gx, int gy, float& wx, float& wy) const {
        wx = origin_x + (gx + 0.5f) * cell;
        wy = origin_y + (gy + 0.5f) * cell;
    }
};

bool buildGrid(const ArenaInfo& arena, const std::vector<Obstacle>& obstacles, float cell_px,
               float robot_radius_px, float clearance_px, float polygon_edge_extra_px, Grid& g) {
    if (!arena.has_polygon || cell_px < 2.0f) {
        return false;
    }
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int k = 0; k < 4; ++k) {
        minx = std::min(minx, arena.corners[k][0]);
        miny = std::min(miny, arena.corners[k][1]);
        maxx = std::max(maxx, arena.corners[k][0]);
        maxy = std::max(maxy, arena.corners[k][1]);
    }
    const float pad = cell_px * 3.0f;
    minx -= pad;
    miny -= pad;
    maxx += pad;
    maxy += pad;
    g.origin_x = minx;
    g.origin_y = miny;
    g.cell = cell_px;
    g.gw = std::max(3, (int)std::ceil((maxx - minx) / cell_px));
    g.gh = std::max(3, (int)std::ceil((maxy - miny) / cell_px));
    g.blocked.assign((size_t)g.gw * (size_t)g.gh, 0);
    for (int y = 0; y < g.gh; ++y) {
        for (int x = 0; x < g.gw; ++x) {
            float wx = 0.0f, wy = 0.0f;
            g.gridToWorld(x, y, wx, wy);
            if (!cellCenterFree(wx, wy, arena, obstacles, robot_radius_px, clearance_px,
                                 polygon_edge_extra_px)) {
                g.blocked[g.idx(x, y)] = 1;
            }
        }
    }
    return true;
}

bool astar(const Grid& g, int sx, int sy, int gx, int gy, std::vector<int>& out_path_cells) {
    out_path_cells.clear();
    const int N = g.gw * g.gh;
    if (g.blocked[g.idx(sx, sy)] || g.blocked[g.idx(gx, gy)]) {
        return false;
    }
    const float INF = 1e18f;
    std::vector<float> gscore((size_t)N, INF);
    std::vector<int> came((size_t)N, -1);
    const int start = g.idx(sx, sy);
    const int goal_idx = g.idx(gx, gy);
    gscore[(size_t)start] = 0.0f;

    auto heuristic = [&](int i) {
        const int ix = i % g.gw;
        const int iy = i / g.gw;
        return std::fabs((float)(ix - gx)) + std::fabs((float)(iy - gy));
    };

    using Node = std::pair<float, int>;  // f, idx
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    open.push({0.0f, start});

    static const int dx[] = {1, -1, 0, 0};
    static const int dy[] = {0, 0, 1, -1};

    while (!open.empty()) {
        const int cur = open.top().second;
        open.pop();
        if (cur == goal_idx) {
            for (int at = goal_idx;; at = came[(size_t)at]) {
                out_path_cells.push_back(at);
                if (at == start) {
                    break;
                }
            }
            return true;
        }
        const int cx = cur % g.gw;
        const int cy = cur / g.gw;
        const float base = gscore[(size_t)cur];
        for (int dir = 0; dir < 4; ++dir) {
            const int nx = cx + dx[dir];
            const int ny = cy + dy[dir];
            if (!g.inBounds(nx, ny)) continue;
            const int ni = g.idx(nx, ny);
            if (g.blocked[(size_t)ni]) continue;
            const float tentative = base + 1.0f;
            if (tentative < gscore[(size_t)ni]) {
                came[(size_t)ni] = cur;
                gscore[(size_t)ni] = tentative;
                const float f = tentative + heuristic(ni);
                open.push({f, ni});
            }
        }
    }
    return false;
}

}  // namespace

GridPlanResult GridPlanner::plan(const ArenaInfo& arena, const std::vector<Obstacle>& obstacles,
                                 float start_x, float start_y, float enemy_x, float enemy_y, float cell_px,
                                 float robot_radius_px, float clearance_px, float shoot_standoff_px,
                                 float los_step, float polygon_edge_extra_px) {
    GridPlanResult out;
    Grid g;
    if (!buildGrid(arena, obstacles, cell_px, robot_radius_px, clearance_px, polygon_edge_extra_px, g)) {
        return out;
    }
    int sx = 0, sy = 0, gx = 0, gy = 0;
    g.worldToGrid(start_x, start_y, sx, sy);
    if (g.blocked[g.idx(sx, sy)]) {
        // nudge toward center of arena
        float cx = 0.0f, cy = 0.0f;
        for (int k = 0; k < 4; ++k) {
            cx += arena.corners[k][0];
            cy += arena.corners[k][1];
        }
        cx *= 0.25f;
        cy *= 0.25f;
        g.worldToGrid(cx, cy, sx, sy);
    }
    if (g.blocked[g.idx(sx, sy)]) {
        return out;
    }

    struct Cand {
        int x, y;
        float prio;  // sort ascending: closest shooting cell to the robot (not to the enemy)
    };
    std::vector<Cand> cands;
    cands.reserve(220);

    // Prefer staying put: current cell if it is traversable and has LOS to the enemy.
    {
        int scx = 0, scy = 0;
        g.worldToGrid(start_x, start_y, scx, scy);
        if (g.inBounds(scx, scy) && !g.blocked[g.idx(scx, scy)]) {
            float pwx = 0.0f, pwy = 0.0f;
            g.gridToWorld(scx, scy, pwx, pwy);
            const float thin = 3.0f;
            if (cellCenterFree(pwx, pwy, arena, obstacles, robot_radius_px, clearance_px,
                               polygon_edge_extra_px) &&
                rayClearToTarget(pwx, pwy, enemy_x, enemy_y, arena, obstacles, thin, los_step)) {
                cands.push_back({scx, scy, 0.0f});
            }
        }
    }

    // Rings at several radii so the closest LOS pose to us is not forced onto one
    // large standoff circle around the enemy (which often reads as "drive toward them").
    static const float kRadiusFrac[] = {0.35f, 0.55f, 0.8f, 1.0f, 1.35f, 1.85f, 2.5f};
    constexpr int kAngSamples = 48;
    for (float rf : kRadiusFrac) {
        const float rad = std::max(8.0f, shoot_standoff_px * rf);
        for (int k = 0; k < kAngSamples; ++k) {
            const float ang = (float)k * (2.0f * kPi / (float)kAngSamples);
            const float wx = enemy_x + std::cos(ang) * rad;
            const float wy = enemy_y + std::sin(ang) * rad;
            if (!cellCenterFree(wx, wy, arena, obstacles, robot_radius_px, clearance_px,
                               polygon_edge_extra_px)) {
                continue;
            }
            const float thin = 3.0f;
            if (!rayClearToTarget(wx, wy, enemy_x, enemy_y, arena, obstacles, thin, los_step)) {
                continue;
            }
            int cx = 0, cy = 0;
            g.worldToGrid(wx, wy, cx, cy);
            if (!g.inBounds(cx, cy) || g.blocked[g.idx(cx, cy)]) continue;
            float pwx = 0.0f, pwy = 0.0f;
            g.gridToWorld(cx, cy, pwx, pwy);
            const float pr = dist2(pwx, pwy, start_x, start_y);
            cands.push_back({cx, cy, pr});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.prio < b.prio; });

    std::vector<int> path_cells;
    for (const Cand& c : cands) {
        if (astar(g, sx, sy, c.x, c.y, path_cells)) {
            out.ok = true;
            out.waypoints.reserve(path_cells.size());
            for (int i = (int)path_cells.size() - 1; i >= 0; --i) {
                const int id = path_cells[(size_t)i];
                float wx = 0.0f, wy = 0.0f;
                g.gridToWorld(id % g.gw, id / g.gw, wx, wy);
                out.waypoints.push_back({wx, wy});
            }
            // Subsample for smoother overlay / driving
            if (out.waypoints.size() > 48) {
                std::vector<std::pair<float, float>> sparse;
                const size_t step = out.waypoints.size() / 24;
                for (size_t i = 0; i < out.waypoints.size(); i += step) {
                    sparse.push_back(out.waypoints[i]);
                }
                sparse.push_back(out.waypoints.back());
                out.waypoints.swap(sparse);
            }
            break;
        }
    }
    return out;
}

/*
Safety.cpp
- Implements all arena containment and obstacle clearance checks used by AI and robot control
- point_in_polygon: ray-casting even-odd rule for convex or concave quads
- segment_clear: walks a segment in 8px steps, testing arena walls and all obstacles with robot-radius pad
- predicted_pose_safe: integrates unicycle kinematics 5 steps ahead and checks each pose
- steer_avoid: sweeps heading offsets ±90° in 10° increments to find the first clear direction
by: Abdulla Sadoun
Date: February 15, 2026
*/
// Safety.cpp
#include "Safety.h"
 
#include <algorithm>
#include <cmath>
 
bool point_in_polygon(float x, float y, const float (*poly)[2], int n) {
    /*
    Even-odd ray-casting: counts how many polygon edges cross a horizontal ray from (x,y) rightward.
    Returns true when the count is odd (point inside), false otherwise.
    */
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        const float xi = poly[i][0], yi = poly[i][1];
        const float xj = poly[j][0], yj = poly[j][1];
        const bool intersect = ((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-6f) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}
 
bool point_in_arena(const WorldEnv& env, float x, float y) {
    /*
    Delegates to point_in_polygon when a calibrated arena polygon exists,
    otherwise falls back to a simple [0, img_w) x [0, img_h) rectangle test.
    */
    if (env.arena.has_polygon) {
        return point_in_polygon(x, y, env.arena.corners, 4);
    }
    return (x >= 0.0f && y >= 0.0f &&
            x <  (float)env.arena.img_w && y < (float)env.arena.img_h);
}
 
float distance_to_obstacle(float x, float y, const Obstacle& o) {
    /*
    For sprite obstacles returns the Euclidean distance from (x,y) to the nearest AABB edge.
    For circle obstacles subtracts the radius from the center-to-point distance (clamped to 0).
    */
    if (o.img) {
        const float minX = (float)o.x - o.img->w * 0.5f;
        const float maxX = (float)o.x + o.img->w * 0.5f;
        const float minY = (float)o.y - o.img->h * 0.5f;
        const float maxY = (float)o.y + o.img->h * 0.5f;
        const float cx = std::max(minX, std::min(x, maxX));
        const float cy = std::max(minY, std::min(y, maxY));
        const float dx = x - cx;
        const float dy = y - cy;
        return std::sqrt(dx * dx + dy * dy);
    }
    const float dx = x - (float)o.x;
    const float dy = y - (float)o.y;
    const float d = std::sqrt(dx * dx + dy * dy) - o.radius;
    return d > 0.0f ? d : 0.0f;
}
 
bool segment_clear(const WorldEnv& env, float x0, float y0, float x1, float y1) {
    /*
    Samples the segment at ceil(len/8) equal steps, checking each sample against arena walls,
    all obstacles, and the other robot with a combined (robot_radius + safety_margin) pad.
    */
    const float pad = env.robot_radius + env.safety_margin;
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    const float len = std::sqrt(dx * dx + dy * dy);
    const int steps = std::max(4, (int)std::ceil(len / 8.0f));
 
    for (int i = 0; i <= steps; ++i) {
        const float t = (float)i / (float)steps;
        const float x = x0 + t * dx;
        const float y = y0 + t * dy;
 
        if (!point_in_arena(env, x, y)) return false;
        if (env.arena.has_polygon) {
            for (int k = 0; k < 4; ++k) {
                const int kn = (k + 1) % 4;
                const float ax = env.arena.corners[k][0];
                const float ay = env.arena.corners[k][1];
                const float bx = env.arena.corners[kn][0];
                const float by = env.arena.corners[kn][1];
                const float ex = bx - ax;
                const float ey = by - ay;
                const float wx = x - ax;
                const float wy = y - ay;
                const float L2 = ex * ex + ey * ey;
                if (L2 < 1e-3f) continue;
                const float tt = std::max(0.0f, std::min(1.0f, (wx * ex + wy * ey) / L2));
                const float px = ax + tt * ex - x;
                const float py = ay + tt * ey - y;
                if (px * px + py * py < pad * pad) return false;
            }
        } else {
            if (x < pad || y < pad || x > env.arena.img_w - pad || y > env.arena.img_h - pad)
                return false;
        }
 
        if (env.obstacles) {
            for (const auto& o : *env.obstacles) {
                if (distance_to_obstacle(x, y, o) < pad) return false;
            }
        }
        if (env.other_robot && env.other_robot->is_alive) {
            const float odx = x - env.other_robot->x;
            const float ody = y - env.other_robot->y;
            const float r = env.robot_radius * 2.0f + env.safety_margin;
            if (odx * odx + ody * ody < r * r) return false;
        }
    }
    return true;
}
 
bool predicted_pose_safe(const Robot& r, float v, float w, float lookahead_s, const WorldEnv& env) {
    /*
    Integrates unicycle kinematics (x += v*cos(theta+pi/2)*dt) over 5 equal substeps.
    Returns false as soon as any intermediate pose fails a segment_clear point-check.
    */
    const int substeps = 5;
    const float dt = lookahead_s / (float)substeps;
 
    float x = r.x, y = r.y, theta = r.theta;
    for (int i = 0; i <= substeps; ++i) {
        if (!segment_clear(env, x, y, x, y)) return false;
 
        const float h = theta + 3.1415926f / 2.0f;
        x += v * std::cos(h) * dt;
        y += v * std::sin(h) * dt;
        theta += w * dt;
    }
    return true;
}
 
float steer_avoid(const Robot& self, float desired_heading_world, const WorldEnv& env, float lookahead, bool& blocked) {
    /*
    Tries desired_heading first, then sweeps ±10°, ±20°, ... ±90° until segment_clear passes.
    Sets blocked=true if all 19 candidates fail; returns the first clear heading found.
    */
    blocked = false;
    const int max_offset_deg = 90;
    const int step_deg = 10;
 
    for (int off = 0; off <= max_offset_deg; off += step_deg) {
        for (int sign : {+1, -1}) {
            if (off == 0 && sign == -1) continue;
            const float a = desired_heading_world + sign * off * 3.1415926f / 180.0f;
            const float ex = self.x + lookahead * std::cos(a);
            const float ey = self.y + lookahead * std::sin(a);
            if (segment_clear(env, self.x, self.y, ex, ey)) {
                return a;
            }
        }
    }
    blocked = true;
    return desired_heading_world;
}
 
float wrapAngle(float a) {
    /*
    Iteratively adds or subtracts 2π until the result is in (-π, π].
    Used after computing angle differences to prevent wrap-around steering errors.
    */
    while (a > 3.1415926f) a -= 2.0f * 3.1415926f;
    while (a < -3.1415926f) a += 2.0f * 3.1415926f;
    return a;
}

bool check_collision(const Robot& robot, const std::vector<Obstacle>& obstacles, int img_w, int img_h) {
    /*
    Ground-truth collision used only by the simulator (no real latency). Returns true if the
    robot body (radius 25 px) overlaps any image boundary or obstacle AABB / circle.
    */
    const int robot_radius = 25;
    if (robot.x - robot_radius < 0 || robot.x + robot_radius >= img_w ||
        robot.y - robot_radius < 0 || robot.y + robot_radius >= img_h)
        return true;

    for (const auto& o : obstacles) {
        if (o.img) {
            float hw = o.img->w * 0.5f, hh = o.img->h * 0.5f;
            float cx = std::max((float)o.x - hw, std::min(robot.x, (float)o.x + hw));
            float cy = std::max((float)o.y - hh, std::min(robot.y, (float)o.y + hh));
            float dx = robot.x - cx, dy = robot.y - cy;
            if (dx * dx + dy * dy < (float)(robot_radius * robot_radius)) return true;
        } else {
            float dx = robot.x - (float)o.x, dy = robot.y - (float)o.y;
            float rr = o.radius + robot_radius;
            if (dx * dx + dy * dy < rr * rr) return true;
        }
    }
    return false;
}

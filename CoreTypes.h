/*
CoreTypes.h
- Defines fundamental POD types shared across simulation, safety, AI, and robot I/O subsystems
- Image24: BGR pixel buffer for BMP assets and the shared-memory image_view pipeline
- Robot: 2D pose (x, y, theta), velocities, turret angle, shot/alive flags
- Obstacle: either a sprite-backed AABB or a vision-detected circle (radius > 0, img == nullptr)
- Mode/Target enums: runtime selection of AI vs user control, simulator vs real robot
- Inline helpers: point_hits_obstacle (AABB or disk test), obstacle_half_extent
by: Abdulla Sadoun
Date: February 10, 2026
*/
// CoreTypes.h
//
// Shared data structures used across subsystems (simulation, safety, AI, robot IO).
//
// NOTE:
// - `Image24` is the basic BGR image container used by both BMP assets and the
//   shared-memory image_view pipeline.
// - `Obstacle` can represent either a sprite-backed rectangle (img != nullptr)
//   or a vision-detected disk (img == nullptr, radius > 0).
//
#pragma once
 
#include <cstdint>
#include <vector>
 
struct Image24 {
    int w = 0, h = 0;
    // BGR packed: data[(y*w + x)*3 + 0] = B, +1=G, +2=R
    std::vector<uint8_t> data;
};
 
struct Robot {
    float x = 0, y = 0;      // pixels
    float theta = 0;         // radians (body angle; forward = theta + pi/2)
    float turret = 0;        // radians relative to body
    bool  shotUsed = false;
    float v = 0.0f;          // linear velocity (px/s)
    float w = 0.0f;          // angular velocity (rad/s)
    bool  is_alive = true;
};
 
struct Obstacle {
    int x = 0;
    int y = 0;
    Image24* img = nullptr;  // nullptr => circle obstacle (vision)
    float radius = 0.0f;
};
 
enum class Mode {
    USER_CONTROL,
    AI_ATTACK,
    AI_DEFEND
};
 
enum class Target {
    SIMULATOR,
    ROBOT
};
 
// True if the axis-aligned bounding box or disk of `o` contains (x, y).
static inline bool point_hits_obstacle(int x, int y, const Obstacle& o) {
    /*
    For sprite obstacles checks if (x,y) is inside the AABB centered on (o.x, o.y).
    For circle obstacles (img==nullptr) does a squared-distance vs radius^2 test.
    */
    if (o.img) {
        return (x > o.x - o.img->w / 2 && x < o.x + o.img->w / 2 &&
                y > o.y - o.img->h / 2 && y < o.y + o.img->h / 2);
    }
    const float dx = (float)x - (float)o.x;
    const float dy = (float)y - (float)o.y;
    return (dx * dx + dy * dy) < (o.radius * o.radius);
}
 
static inline float obstacle_half_extent(const Obstacle& o) {
    /*
    Returns half the width of a sprite obstacle or the radius of a circle obstacle.
    Used by the defend AI to compute perpendicular detour offsets around cover objects.
    */
    return o.img ? o.img->w / 2.0f : o.radius;
}
 

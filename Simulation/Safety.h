/*
Safety.h
- Collision and safety geometry for the arena, obstacles, and robots
- ArenaInfo: stores calibrated 4-corner polygon or fallback image-rect bounds
- WorldEnv: bundles arena + obstacles + other robot into a single query context
- point_in_polygon: ray-cast containment test for 4-corner arena boundary
- point_in_arena / distance_to_obstacle: fundamental clearance queries
- segment_clear: samples a robot-radius-wide corridor along a line segment
- predicted_pose_safe: 5-step forward integration safety lookahead
- steer_avoid: samples headings ±90° in 10° steps to find a clear escape direction
- wrapAngle: normalizes angle to [-π, π]
- check_collision: tests robot against image boundary and all obstacle shapes (sim ground-truth)
by: Abdulla Sadoun
Date: February 15, 2026
*/
// Safety.h
#pragma once
 
#include "../CoreTypes.h"
 
struct ArenaInfo {
    bool  has_polygon = false;
    float corners[4][2] = {};   // CW or CCW; only used when has_polygon
    int   img_w = 0;
    int   img_h = 0;
};
 
struct WorldEnv {
    const std::vector<Obstacle>* obstacles = nullptr;
    const Robot* other_robot = nullptr;     // null if no other robot to avoid
    ArenaInfo arena;
    float robot_radius  = 25.0f;
    float safety_margin = 10.0f;            // extra pad on top of robot_radius
};
 
bool point_in_polygon(float x, float y, const float (*poly)[2], int n);
bool point_in_arena(const WorldEnv& env, float x, float y);
float distance_to_obstacle(float x, float y, const Obstacle& o);
bool segment_clear(const WorldEnv& env, float x0, float y0, float x1, float y1);
bool predicted_pose_safe(const Robot& r, float v, float w, float lookahead_s, const WorldEnv& env);
float steer_avoid(const Robot& self, float desired_heading_world, const WorldEnv& env, float lookahead, bool& blocked);
float wrapAngle(float a);

// Simulator ground-truth collision: returns true if the robot body (r=25px) overlaps any wall
// or obstacle AABB/circle. Used to revert position in the simulator each frame.
bool check_collision(const Robot& robot, const std::vector<Obstacle>& obstacles, int img_w, int img_h);
 

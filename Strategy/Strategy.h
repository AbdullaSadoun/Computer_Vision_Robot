/*
Strategy.h
- Declares the three AI behavior functions: attack, defend, and return-home
- has_clear_los: stepped ray-cast to test if the attacker has an unobstructed shot
- AttackAiStability: debounce struct preventing LOS flicker and body-rotate oscillation
- update_attack_ai: LOS-first attack with turret aim, body rotation fallback, and seek-shot detour
- update_defend_ai: finds nearest cover obstacle and hides the defender behind it
- update_return_home_ai: breadcrumb trail follower that steers back to the spawn pose after firing
- slew_turret_toward_neutral_rad: smoothly returns turret to 0 rad at a fixed rate
by: Abdulla Sadoun
Date: March 5, 2026
*/
// Strategy.h
#pragma once
 
#include <vector>
#include <utility>
 
#include "../CoreTypes.h"
#include "../Simulation/Safety.h"
#include "../Simulation/DebugOverlay.h"
 
bool has_clear_los(float x0, float y0, float x1, float y1, const std::vector<Obstacle>& obstacles);

// Debounces obstacle LOS so vision noise does not flip AI_ATTACK between
// aim-at-target and detour every frame (steer_avoid uses stricter segment_clear).
struct AttackAiStability {
    bool los_inited = false;
    bool los_filtered = false;
    int los_pending = 0;

    // Schmitt for in_arc vs rotate_body: exit rotate when |turret err| <= limit;
    // after the first rotate session, re-enter body-rotate only if |err| > limit+hi.
    bool turret_body_rotate_latched = false;
    bool turret_body_rotate_rearm = false;

    void reset() {
        los_inited = false;
        los_filtered = false;
        los_pending = 0;
        turret_body_rotate_latched = false;
        turret_body_rotate_rearm = false;
    }

    static constexpr int kLosDebounceFrames = 4;
};

void update_attack_ai(Robot& attacker, const Robot& target, const std::vector<Obstacle>& obstacles,
                      const WorldEnv& env, float dt, bool& fired, float t, float& fire_time, bool& hit, float& hit_time,
                      AIDebug* dbg = nullptr, AttackAiStability* stability = nullptr);
 
void update_defend_ai(Robot& defender, const Robot& enemy, const std::vector<Obstacle>& obstacles,
                      const WorldEnv& env, float dt, AIDebug* dbg = nullptr);
 
void update_return_home_ai(Robot& r,
                           float start_x, float start_y, float start_theta,
                           std::vector<std::pair<float,float>>& trail,
                           const std::vector<Obstacle>& obstacles,
                           float dt,
                           AIDebug* dbg = nullptr);

// Body-relative turret → 0 rad at the same rate as return-home (for post-shot manual handoff).
void slew_turret_toward_neutral_rad(float& turret_rad, float dt);


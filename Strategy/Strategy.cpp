/*
Strategy.cpp
- Implements the complete game AI: attack, defend, return-home, and turret slew behaviors
- has_clear_los: walks the attacker-to-target segment in params().los_step increments
- update_attack_ai: prioritizes turret-only aim when target is in arc; moves to seek-shot position otherwise
- update_defend_ai: seeks the nearest cover obstacle and routes around it if path is blocked
- update_return_home_ai: prunes breadcrumb trail using LOS shortcuts, then steers toward next waypoint
by: Abdulla Sadoun
Date: March 5, 2026
*/
// Strategy.cpp
#include "Strategy.h"
 
#include <cmath>
 
#include "../Parameters.h"
 
bool has_clear_los(float x0, float y0, float x1, float y1, const std::vector<Obstacle>& obstacles) {
    /*
    Walks the segment from (x0,y0) to (x1,y1) in params().los_step fractional increments.
    Returns false immediately if any sample point hits an obstacle, true if all are clear.
    */
    const float step = params().los_step;
    for (float i = 0.0f; i < 1.0f; i += step) {
        int x = (int)(x0 + i * (x1 - x0));
        int y = (int)(y0 + i * (y1 - y0));
        for (auto& o : obstacles) {
            if (point_hits_obstacle(x, y, o)) return false;
        }
    }
    return true;
}
 
void update_attack_ai(Robot& attacker, const Robot& target, const std::vector<Obstacle>& obstacles,
                      const WorldEnv& env, float /*dt*/, bool& fired, float t, float& fire_time, bool& hit, float& hit_time,
                      AIDebug* dbg, AttackAiStability* stability) {
    /*
    If LOS is clear, aims the turret; fires when aligned within 0.1 rad and sets shotUsed.
    If LOS is blocked, samples 16 positions around the target at 150px radius and steers toward the nearest clear one.
    */
    const float attack_speed = 120.0f;
    const float attack_turn_rate = 2.5f;
    const float turret_limit = 170.0f * 3.1415926f / 180.0f / 2.0f;
 
    attacker.v = 0.0f;
    attacker.w = 0.0f;
 
    const bool los_raw = has_clear_los(attacker.x, attacker.y, target.x, target.y, obstacles);
    bool los = los_raw;
    if (stability) {
        if (!stability->los_inited) {
            stability->los_filtered = los_raw;
            stability->los_inited = true;
        } else if (los_raw != stability->los_filtered) {
            stability->los_pending++;
            if (stability->los_pending >= AttackAiStability::kLosDebounceFrames) {
                stability->los_filtered = los_raw;
                stability->los_pending = 0;
            }
        } else {
            stability->los_pending = 0;
        }
        los = stability->los_filtered;
    }
    if (!los && stability) {
        stability->turret_body_rotate_latched = false;
        stability->turret_body_rotate_rearm = false;
    }
    if (los) {
        if (dbg) {
            dbg->state = BehaviorState::SeekShot;
            dbg->detail = "LOS";
            dbg->goal_valid = true;
            dbg->goal_x = target.x;
            dbg->goal_y = target.y;
            dbg->path = {{attacker.x, attacker.y}, {target.x, target.y}};
            dbg->lookahead_px = params().steer_lookahead_px;
        }
        float angle_to_target_world = std::atan2(target.y - attacker.y, target.x - attacker.x);
        // Same as body-forward to target bearing; turret offset needed if we only slewed turret.
        float desired_turret_angle_rel =
            wrapAngle(angle_to_target_world - (attacker.theta + 3.1415926f / 2.0f));
        // Schmitt: exit rotate as soon as |err| <= turret_limit. First time we need body
        // turn, use strict a > limit; after that, require a > limit+hi to re-enter (vision chatter).
        const float turretArcReenterHi = 0.10f;

        bool need_body_for_turret = std::fabs(desired_turret_angle_rel) > turret_limit;
        if (stability) {
            const float a = std::fabs(desired_turret_angle_rel);
            if (!stability->turret_body_rotate_latched) {
                const float enterTh =
                    stability->turret_body_rotate_rearm ? (turret_limit + turretArcReenterHi) : turret_limit;
                need_body_for_turret = a > enterTh;
                if (need_body_for_turret) {
                    stability->turret_body_rotate_latched = true;
                }
            } else {
                need_body_for_turret = true;
                if (a <= turret_limit) {
                    stability->turret_body_rotate_latched = false;
                    stability->turret_body_rotate_rearm = true;
                    need_body_for_turret = false;
                }
            }
        }

        if (!need_body_for_turret) {
            attacker.w = 0.0f;
            attacker.turret = desired_turret_angle_rel;
            if (dbg) {
                dbg->state = BehaviorState::AimTurret;
                dbg->detail = "in_arc";
            }
            if (!attacker.shotUsed) {
                if (std::fabs(attacker.turret - desired_turret_angle_rel) < 0.1f) {
                    attacker.shotUsed = true;
                    fired = true;
                    fire_time = t;
                    hit = true;
                    hit_time = t;
                    if (dbg) {
                        dbg->detail = "fire";
                    }
                }
            }
        } else {
            attacker.v = 0.0f;
            const float angle_diff = desired_turret_angle_rel;
            // Deadband stops bang-bang on theta noise when nearly aligned.
            const float w_deadband = 0.06f;
            if (std::fabs(angle_diff) < w_deadband) {
                attacker.w = 0.0f;
            } else {
                attacker.w = (angle_diff > 0.0f) ? attack_turn_rate : -attack_turn_rate;
            }
            if (dbg) {
                dbg->state = BehaviorState::AimTurret;
                dbg->detail = "rotate_body";
            }
        }
        return;
    }
 
    float best_pos_x = -1.0f, best_pos_y = -1.0f;
    float min_dist_sq = -1.0f;
    const int num_samples = 16;
    const float sample_radius = 150.0f;
 
    for (int i = 0; i < num_samples; ++i) {
        float angle = (float)i / num_samples * 2.0f * 3.1415926f;
        float px = target.x + sample_radius * cos(angle);
        float py = target.y + sample_radius * sin(angle);
        if (has_clear_los(px, py, target.x, target.y, obstacles)) {
            float dx = px - attacker.x;
            float dy = py - attacker.y;
            float dist_sq = dx * dx + dy * dy;
            if (min_dist_sq < 0 || dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
                best_pos_x = px;
                best_pos_y = py;
            }
        }
    }
 
    float goal_x, goal_y;
    if (min_dist_sq >= 0.0f) {
        goal_x = best_pos_x;
        goal_y = best_pos_y;
    } else {
        goal_x = target.x;
        goal_y = target.y;
    }
 
    float desired_heading = std::atan2(goal_y - attacker.y, goal_x - attacker.x);
    bool blocked = false;
    float safe_heading = steer_avoid(attacker, desired_heading, env, params().steer_lookahead_px, blocked);
    if (dbg) {
        dbg->state = blocked ? BehaviorState::AvoidObstacle : BehaviorState::MoveToGoal;
        dbg->detail = blocked ? "blocked" : "clear";
        dbg->goal_valid = true;
        dbg->goal_x = goal_x;
        dbg->goal_y = goal_y;
        dbg->path = {{attacker.x, attacker.y}, {goal_x, goal_y}};
        dbg->lookahead_px = params().steer_lookahead_px;
    }
 
    float current_forward_heading = attacker.theta + 3.1415926f / 2.0f;
    float angle_diff = wrapAngle(safe_heading - current_forward_heading);
    if (fabs(angle_diff) > 0.1f) {
        attacker.w = (angle_diff > 0.0f) ? attack_turn_rate : -attack_turn_rate;
    }
    attacker.v = blocked ? -attack_speed * 0.5f : attack_speed;
}
 
void update_defend_ai(Robot& defender, const Robot& enemy, const std::vector<Obstacle>& obstacles,
                      const WorldEnv& env, float /*dt*/, AIDebug* dbg) {
    /*
    Finds the nearest cover obstacle, computes a hide-spot 75px beyond it along the enemy-to-cover vector.
    Routes around the obstacle via the shorter of two perpendicular detour points if direct path is blocked.
    */
    const float defend_speed = 150.0f;
    const float defend_turn_rate = 2.5f;
    const float safe_distance_from_cover = 75.0f;
    const float obstacle_clearance = 40.0f;
 
    defender.v = 0.0f;
    defender.w = 0.0f;
 
    Obstacle* best_cover = nullptr;
    float best_dist_sq = -1.0f;
    for (const auto& o : obstacles) {
        float dx = o.x - defender.x;
        float dy = o.y - defender.y;
        float dist_sq = dx * dx + dy * dy;
        if (best_dist_sq < 0 || dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            best_cover = (Obstacle*)&o;
        }
    }
    if (!best_cover) return;
 
    float vec_enemy_to_cover_x = best_cover->x - enemy.x;
    float vec_enemy_to_cover_y = best_cover->y - enemy.y;
    float len = std::sqrt(vec_enemy_to_cover_x * vec_enemy_to_cover_x + vec_enemy_to_cover_y * vec_enemy_to_cover_y);
 
    float hide_pos_x = best_cover->x;
    float hide_pos_y = best_cover->y;
    if (len > 0.01f) {
        hide_pos_x += (vec_enemy_to_cover_x / len) * safe_distance_from_cover;
        hide_pos_y += (vec_enemy_to_cover_y / len) * safe_distance_from_cover;
    }
 
    bool is_exposed = has_clear_los(defender.x, defender.y, enemy.x, enemy.y, obstacles);
    float dist_to_hide_spot_sq = (hide_pos_x - defender.x) * (hide_pos_x - defender.x) +
                                 (hide_pos_y - defender.y) * (hide_pos_y - defender.y);
 
    if (is_exposed || dist_to_hide_spot_sq > 25.0f * 25.0f) {
        float target_pos_x = hide_pos_x;
        float target_pos_y = hide_pos_y;
 
        if (!has_clear_los(defender.x, defender.y, hide_pos_x, hide_pos_y, {*best_cover})) {
            float vec_to_hide_x = hide_pos_x - defender.x;
            float vec_to_hide_y = hide_pos_y - defender.y;
            float dist_to_hide = std::sqrt(dist_to_hide_spot_sq);
            float perp_x = -vec_to_hide_y / dist_to_hide;
            float perp_y =  vec_to_hide_x / dist_to_hide;
 
            float detour_dist = obstacle_half_extent(*best_cover) + obstacle_clearance;
            float detour1_x = best_cover->x + perp_x * detour_dist;
            float detour1_y = best_cover->y + perp_y * detour_dist;
            float detour2_x = best_cover->x - perp_x * detour_dist;
            float detour2_y = best_cover->y - perp_y * detour_dist;
 
            float d1 = std::sqrt((detour1_x - defender.x) * (detour1_x - defender.x) +
                                 (detour1_y - defender.y) * (detour1_y - defender.y)) +
                       std::sqrt((hide_pos_x - detour1_x) * (hide_pos_x - detour1_x) +
                                 (hide_pos_y - detour1_y) * (hide_pos_y - detour1_y));
            float d2 = std::sqrt((detour2_x - defender.x) * (detour2_x - defender.x) +
                                 (detour2_y - defender.y) * (detour2_y - defender.y)) +
                       std::sqrt((hide_pos_x - detour2_x) * (hide_pos_x - detour2_x) +
                                 (hide_pos_y - detour2_y) * (hide_pos_y - detour2_y));
 
            if (d1 < d2) { target_pos_x = detour1_x; target_pos_y = detour1_y; }
            else         { target_pos_x = detour2_x; target_pos_y = detour2_y; }
        }
 
        float desired_heading = std::atan2(target_pos_y - defender.y, target_pos_x - defender.x);
        bool blocked = false;
        float safe_heading = steer_avoid(defender, desired_heading, env, params().steer_lookahead_px, blocked);
        if (dbg) {
            dbg->state = blocked ? BehaviorState::AvoidObstacle : BehaviorState::MoveToGoal;
            dbg->detail = blocked ? "blocked" : (is_exposed ? "exposed" : "reposition");
            dbg->goal_valid = true;
            dbg->goal_x = target_pos_x;
            dbg->goal_y = target_pos_y;
            dbg->path = {{defender.x, defender.y}, {target_pos_x, target_pos_y}};
            dbg->lookahead_px = params().steer_lookahead_px;
        }
 
        float current_forward_heading = defender.theta + 3.1415926f / 2.0f;
        float angle_diff = wrapAngle(safe_heading - current_forward_heading);
 
        if (blocked) {
            defender.v = -defend_speed * 0.5f;
            if (fabs(angle_diff) > 0.1f) defender.w = (angle_diff > 0.0f) ? defend_turn_rate : -defend_turn_rate;
        } else if (fabs(angle_diff) > 3.1415926f / 2.0f) {
            defender.v = -defend_speed;
            float reverse_angle_diff = wrapAngle(angle_diff - 3.1415926f);
            if (fabs(reverse_angle_diff) > 0.1f) defender.w = (reverse_angle_diff > 0.0f) ? defend_turn_rate : -defend_turn_rate;
        } else {
            defender.v = defend_speed;
            if (fabs(angle_diff) > 0.1f) defender.w = (angle_diff > 0.0f) ? defend_turn_rate : -defend_turn_rate;
        }
    }
    else if (dbg) {
        dbg->state = BehaviorState::Idle;
        dbg->detail = "in_cover";
        dbg->goal_valid = true;
        dbg->goal_x = hide_pos_x;
        dbg->goal_y = hide_pos_y;
        dbg->path = {{defender.x, defender.y}, {hide_pos_x, hide_pos_y}};
        dbg->lookahead_px = params().steer_lookahead_px;
    }
}
 
void slew_turret_toward_neutral_rad(float& turret_rad, float dt) {
    /*
    Moves turret_rad toward 0 by at most (3.0 * dt) radians per call.
    Called every frame during return-home so the turret centers smoothly instead of snapping.
    */
    const float turret_return_rate = 3.0f;
    float step = turret_return_rate * dt;
    if (turret_rad > step) turret_rad -= step;
    else if (turret_rad < -step) turret_rad += step;
    else turret_rad = 0.0f;
}

void update_return_home_ai(Robot& r,
                           float start_x, float start_y, float start_theta,
                           std::vector<std::pair<float,float>>& trail,
                           const std::vector<Obstacle>& obstacles,
                           float dt,
                           AIDebug* dbg) {
    /*
    Prunes the breadcrumb trail by removing waypoints that have clear LOS from the current position.
    Steers toward the next trail waypoint, or aligns to start_theta once within 20px of spawn.
    */
    const float home_speed = 120.0f;
    const float turn_rate = 2.5f;
    const float waypoint_dist = 20.0f;
    const float heading_tol = 0.08f;
 
    r.v = 0.0f;
    r.w = 0.0f;
 
    slew_turret_toward_neutral_rad(r.turret, dt);
 
    while (trail.size() >= 2) {
        const auto& earlier = trail[trail.size() - 2];
        if (has_clear_los(r.x, r.y, earlier.first, earlier.second, obstacles)) trail.pop_back();
        else break;
    }
    if (!trail.empty()) {
        float dx = trail.back().first - r.x;
        float dy = trail.back().second - r.y;
        if (dx * dx + dy * dy < waypoint_dist * waypoint_dist) trail.pop_back();
    }
 
    float goal_x, goal_y;
    bool final_leg = trail.empty();
    if (final_leg) { goal_x = start_x; goal_y = start_y; }
    else { goal_x = trail.back().first; goal_y = trail.back().second; }

    if (dbg) {
        dbg->state = BehaviorState::ReturnHome;
        dbg->detail = final_leg ? "spawn" : "breadcrumb";
        dbg->goal_valid = true;
        dbg->goal_x = goal_x;
        dbg->goal_y = goal_y;
        dbg->path.clear();
        dbg->path.push_back({r.x, r.y});
        if (!trail.empty()) {
            // Draw a short prefix of the trail for readability.
            const size_t maxPts = 20;
            size_t start = (trail.size() > maxPts) ? (trail.size() - maxPts) : 0;
            for (size_t i = start; i < trail.size(); ++i) dbg->path.push_back(trail[i]);
        } else {
            dbg->path.push_back({goal_x, goal_y});
        }
        dbg->lookahead_px = 0.0f;
    }
 
    float dx = goal_x - r.x;
    float dy = goal_y - r.y;
    float dist = std::sqrt(dx * dx + dy * dy);
    if (final_leg && dist < waypoint_dist) {
        float angle_diff = wrapAngle(start_theta - r.theta);
        if (fabs(angle_diff) > heading_tol) r.w = (angle_diff > 0.0f) ? turn_rate : -turn_rate;
        return;
    }
 
    float desired_heading = std::atan2(dy, dx);
    float current_forward = r.theta + 3.1415926f / 2.0f;
    float angle_diff = wrapAngle(desired_heading - current_forward);
    if (fabs(angle_diff) > 0.1f) r.w = (angle_diff > 0.0f) ? turn_rate : -turn_rate;
    r.v = home_speed;
}
 

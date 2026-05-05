/*
ArenaMap.h
- Persistent arena model built once from a raw vision snapshot and reconciled periodically (Mode 5)
- Stores self/enemy poses, static obstacles (colored circles), chassis ghosts (black blobs on the robot body),
  possible enemy marker pairs, and the calibrated arena walls + px/in scale
- buildFromSnapshot: initializes the map from a single warped-frame VisionDetectionSnapshot
- reconcileSnapshot: updates marker positions with minimal association edits (aborts if too many changes)
- Static obstacles drive the GridPlanner; chassis ghosts are excluded from obstacle lists to avoid self-collision
by: Abdulla Sadoun
Date: April 1, 2026
*/
#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "../../CoreTypes.h"
#include "../../Simulation/Safety.h"
#include "../../vision/VisionTypes.h"

struct VisionParameters;

// Black blob on chassis: remembered in body frame (forward, left) for reconcile.
struct ChassisGhost {
    float rel_fwd = 0.0f;   // m along body forward (rear->front)
    float rel_left = 0.0f;  // m along body left
    float radius = 0.0f;
};

struct PossibleEnemyPair {
    int idx_a = -1;
    int idx_b = -1;
    float dist_px = 0.0f;
};

// Persistent arena model for mode-5 map-based attack (built once, reconciled periodically).
class ArenaMap {
public:
    void clear();

    // Build from a single raw snapshot (warped frame). Returns false if self pair or scale missing.
    bool buildFromSnapshot(const VisionDetectionSnapshot& snap, MarkerColor self_front_color,
                           const ArenaInfo& walls, const VisionParameters& vision_params,
                           float robot_radius_px, float chassis_slack_px);

    bool isValid() const { return valid_; }

    const ArenaInfo& walls() const { return walls_; }
    float pxPerInch() const { return px_per_inch_; }

    Pose2D selfPose() const { return self_pose_; }
    Pose2D enemyPose() const { return enemy_pose_; }

    MarkerColor enemyFrontColor() const { return enemy_front_color_; }
    MarkerColor enemyRearColor() const { return enemy_rear_color_; }

    const std::vector<Obstacle>& staticObstacles() const { return static_obstacles_; }
    const std::vector<ChassisGhost>& chassisGhosts() const { return chassis_ghosts_; }
    const std::vector<PossibleEnemyPair>& possibleEnemyPairs() const { return possible_enemy_pairs_; }

    // Circles frozen at last successful build/reconcile (for debug overlay).
    const std::vector<VisionCircle>& buildCircles() const { return build_circles_; }

    // Returns number of association edits applied, or -1 if aborted (exceeds max_changes).
    int reconcileSnapshot(const VisionDetectionSnapshot& snap, MarkerColor self_front_color,
                          const VisionParameters& vision_params, float robot_radius_px,
                          float chassis_slack_px, float match_gate_px, int max_changes);

private:
    bool findSelfPair(const std::vector<VisionCircle>& c, MarkerColor self_front, float target_px,
                      float d_min, float d_max, int& out_a, int& out_b) const;

    bool valid_ = false;
    ArenaInfo walls_{};
    float px_per_inch_ = 0.0f;

    Pose2D self_pose_{};
    Pose2D enemy_pose_{};
    MarkerColor enemy_front_color_ = MarkerColor::Unknown;
    MarkerColor enemy_rear_color_ = MarkerColor::Unknown;

    std::vector<Obstacle> static_obstacles_{};
    std::vector<ChassisGhost> chassis_ghosts_{};
    std::vector<PossibleEnemyPair> possible_enemy_pairs_{};
    std::vector<VisionCircle> build_circles_{};
};

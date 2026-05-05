/*
Mode5AttackPlanner.h
- Orchestrates map-first attack for Mode 5 (robot + OpenCV vision + AI_ATTACK)
- tryInit: captures one VisionDetectionSnapshot to build the ArenaMap; retries until scale and self pair are found
- tickReconcile: periodically calls ArenaMap::reconcileSnapshot to keep obstacle positions fresh
- update: drives the robot along GridPlanner waypoints toward a shoot-standoff position, then switches to aim/fire
- Internally tracks Drive → Aim → Done phases; resets back to Drive when a new plan is needed
by: Abdulla Sadoun
Date: April 1, 2026
*/
#pragma once

#ifdef USE_VISION

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "ArenaMap.h"
#include "../../CoreTypes.h"
#include "../../Simulation/DebugOverlay.h"
#include "GridPlanner.h"
#include "../../Simulation/Safety.h"
#include "../../vision/VisionTypes.h"

class VisionSystem;  // captureDetections / config

// Map-first attack for mode 5 (robot + vision + AI_ATTACK).
class Mode5AttackPlanner {
public:
    void reset();

    bool isActive() const { return active_; }
    bool mapReady() const { return map_ready_; }
    bool hasPath() const { return !waypoints_.empty(); }

    // Call each frame until true (needs warped BGR + latched arena + px/in).
    bool tryInit(VisionSystem& vision, const cv::Mat& warped_bgr, const ArenaInfo& walls,
                 MarkerColor self_front);

    void tickReconcile(int frame_index, VisionSystem& vision, const cv::Mat& warped_bgr,
                       MarkerColor self_front, const ArenaInfo& walls);

    // Sync game robots from map; run waypoint follow or aim; fill overlay path.
    // `los_obstacles` should match the list used for hit tests / legacy attack (vision disks).
    void update(Robot& my_robot, Robot& target_robot, const std::vector<Obstacle>& los_obstacles,
                float dt, float t, bool& fired, float& fire_time, bool& hit, float& hit_time, AIDebug& dbg);

    const std::string& statusLine() const { return status_; }
    const std::vector<Obstacle>& plannerObstacles() const { return map_.staticObstacles(); }

private:
    bool replan(const ArenaInfo& walls, float start_x, float start_y);

    bool active_ = false;
    bool map_ready_ = false;
    ArenaMap map_;
    std::vector<std::pair<float, float>> waypoints_;
    size_t waypoint_index_ = 0;

    int last_reconcile_frame_ = -1000000;
    int last_plan_frame_ = -1000000;
    float last_enemy_x_ = 0.0f;
    float last_enemy_y_ = 0.0f;
    bool have_last_enemy_ = false;

    enum class Phase { Drive, Aim, Done };
    Phase phase_ = Phase::Drive;

    std::string status_{};
    ArenaInfo walls_{};
    bool walls_valid_ = false;
    int aligned_for_fire_frames_ = 0;
};

#endif  // USE_VISION

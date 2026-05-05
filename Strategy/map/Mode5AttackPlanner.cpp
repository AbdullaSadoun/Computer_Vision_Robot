#ifdef USE_VISION

#include "Mode5AttackPlanner.h"

#include <cmath>
#include <iostream>
#include <sstream>

#include "../../Parameters.h"
#include "../../Simulation/Safety.h"
#include "../Strategy.h"
#include "../../vision/VisionSystem.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float wrap(float a) {
    while (a > kPi) a -= 2.0f * kPi;
    while (a < -kPi) a += 2.0f * kPi;
    return a;
}

}  // namespace

namespace {
// Prints mode5 debug at most once per second, and also on key state changes.
struct Mode5DebugLimiter {
    float last_print_t = -1e9f;
    int last_wp = -1;
    int last_phase = -1;
    bool last_los = false;
};

inline const char* phaseName(int p) {
    switch (p) {
        case 0: return "Drive";
        case 1: return "Aim";
        case 2: return "Done";
        default: return "?";
    }
}
}  // namespace

void Mode5AttackPlanner::reset() {
    active_ = false;
    map_ready_ = false;
    map_.clear();
    waypoints_.clear();
    waypoint_index_ = 0;
    last_reconcile_frame_ = -1000000;
    last_plan_frame_ = -1000000;
    have_last_enemy_ = false;
    phase_ = Phase::Drive;
    status_.clear();
    walls_valid_ = false;
    aligned_for_fire_frames_ = 0;
}

bool Mode5AttackPlanner::tryInit(VisionSystem& vision, const cv::Mat& warped_bgr, const ArenaInfo& walls,
                                 MarkerColor self_front) {
    reset();
    walls_ = walls;
    walls_valid_ = true;
    active_ = true;
    map_ready_ = false;
    waypoints_.clear();
    waypoint_index_ = 0;
    phase_ = Phase::Drive;
    status_ = "M5: building map...";

    VisionDetectionSnapshot snap;
    if (!vision.captureDetections(warped_bgr, snap)) {
        status_ = "M5: captureDetections failed";
        return false;
    }
    if (!map_.buildFromSnapshot(snap, self_front, walls, vision.config(), params().robot_radius_px,
                               params().map_chassis_slack_px)) {
        status_ = "M5: map build failed (self pair / scale)";
        return false;
    }
    map_ready_ = true;
    if (!replan(walls, map_.selfPose().x, map_.selfPose().y)) {
        status_ = "M5: no path (planner)";
        map_ready_ = false;
        return false;
    }
    std::cout << "[M5] init ok. self=(" << (int)map_.selfPose().x << "," << (int)map_.selfPose().y
              << ") enemy=(" << (int)map_.enemyPose().x << "," << (int)map_.enemyPose().y << ") "
              << "phase=" << phaseName((int)phase_) << " waypoints=" << waypoints_.size() << "\n";
    last_plan_frame_ = 0;
    last_reconcile_frame_ = 0;
    if (map_.enemyPose().valid) {
        last_enemy_x_ = map_.enemyPose().x;
        last_enemy_y_ = map_.enemyPose().y;
        have_last_enemy_ = true;
    }
    return true;
}

bool Mode5AttackPlanner::replan(const ArenaInfo& walls, float start_x, float start_y) {
    waypoints_.clear();
    waypoint_index_ = 0;
    phase_ = Phase::Drive;
    if (!map_.isValid() || !map_.enemyPose().valid) {
        return false;
    }
    const float pxi = map_.pxPerInch();
    if (pxi <= 0.0f) {
        return false;
    }
    const auto& obs = map_.staticObstacles();
    const float ex = map_.enemyPose().x;
    const float ey = map_.enemyPose().y;
    // Match legacy attack: only move if vision obstacles break LOS; never "close distance"
    // for its own sake—either hold and shoot or A* to the LOS pose nearest *us*.
    if (has_clear_los(start_x, start_y, ex, ey, obs)) {
        phase_ = Phase::Aim;
        aligned_for_fire_frames_ = 0;
        status_ = "M5: LOS clear (aim/shoot)";
        std::cout << "[M5] replan skipped (LOS clear). start=(" << (int)start_x << "," << (int)start_y
                  << ") enemy=(" << (int)ex << "," << (int)ey << ")\n";
        return true;
    }
    const float cell_px = std::max(4.0f, params().planner_grid_cell_in * pxi);
    const float clearance_px = params().planner_safety_clearance_in * pxi;
    const float standoff = params().planner_shoot_standoff_in * pxi;
    GridPlanResult res = GridPlanner::plan(
        walls, obs, start_x, start_y, ex, ey, cell_px, params().robot_radius_px, clearance_px, standoff,
        params().los_step, 0.0f);
    if (!res.ok || res.waypoints.empty()) {
        std::cout << "[M5] replan failed (no path). start=(" << (int)start_x << "," << (int)start_y
                  << ") enemy=(" << (int)ex << "," << (int)ey << ")\n";
        return false;
    }
    waypoints_ = std::move(res.waypoints);
    waypoint_index_ = 0;
    phase_ = Phase::Drive;
    aligned_for_fire_frames_ = 0;
    status_ = "M5: map+path OK";
    std::cout << "[M5] replan ok. waypoints=" << waypoints_.size()
              << " start=(" << (int)start_x << "," << (int)start_y << ") "
              << "enemy=(" << (int)ex << "," << (int)ey << ")\n";
    // Print the actual path once per plan, so we can see if it hugs walls or oscillates.
    std::cout << "[M5] path:";
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        std::cout << (i == 0 ? " " : " -> ") << "(" << (int)waypoints_[i].first << "," << (int)waypoints_[i].second
                  << ")";
    }
    std::cout << "\n";
    return true;
}

void Mode5AttackPlanner::tickReconcile(int frame_index, VisionSystem& vision, const cv::Mat& warped_bgr,
                                       MarkerColor self_front, const ArenaInfo& walls) {
    walls_ = walls;
    walls_valid_ = true;
    if (!map_ready_) return;
    const int interval = std::max(1, params().planner_reconcile_every_n_frames);
    if (frame_index - last_reconcile_frame_ < interval) {
        return;
    }
    last_reconcile_frame_ = frame_index;

    VisionDetectionSnapshot snap;
    if (!vision.captureDetections(warped_bgr, snap)) {
        return;
    }
    const int chg = map_.reconcileSnapshot(snap, self_front, vision.config(), params().robot_radius_px,
                                           params().map_chassis_slack_px, params().planner_match_gate_px,
                                           params().planner_max_association_changes);
    if (chg < 0) {
        std::ostringstream oss;
        oss << "M5: reconcile aborted (> " << params().planner_max_association_changes << " changes)";
        status_ = oss.str();
        std::cout << "[M5] reconcile aborted: too many association changes\n";
        return;
    }
    if (map_.enemyPose().valid && have_last_enemy_) {
        const float d = std::hypot(map_.enemyPose().x - last_enemy_x_, map_.enemyPose().y - last_enemy_y_);
        if (d > params().planner_replan_enemy_move_px) {
            if (replan(walls_, map_.selfPose().x, map_.selfPose().y)) {
                last_plan_frame_ = frame_index;
            }
        }
    }
    if (map_.enemyPose().valid) {
        last_enemy_x_ = map_.enemyPose().x;
        last_enemy_y_ = map_.enemyPose().y;
        have_last_enemy_ = true;
    }
}

void Mode5AttackPlanner::update(Robot& my_robot, Robot& target_robot,
                                const std::vector<Obstacle>& los_obstacles, float dt, float t, bool& fired,
                                float& fire_time, bool& hit, float& hit_time, AIDebug& dbg) {
    (void)dt;
    static Mode5DebugLimiter lim;
    dbg = AIDebug{};
    dbg.overlay_note = status_;
    if (!map_ready_) {
        dbg.state = BehaviorState::Idle;
        dbg.detail = status_;
        return;
    }

    // Drive game-layer enemy pose from map for shooting geometry.
    if (map_.enemyPose().valid) {
        target_robot.x = map_.enemyPose().x;
        target_robot.y = map_.enemyPose().y;
        target_robot.theta = map_.enemyPose().theta;
        target_robot.is_alive = true;
    }

    dbg.goal_valid = map_.enemyPose().valid;
    dbg.goal_x = map_.enemyPose().x;
    dbg.goal_y = map_.enemyPose().y;

    // Lost obstacle LOS while holding fire (waypoints were cleared): get a new plan from here.
    if (walls_valid_ && phase_ == Phase::Aim && !my_robot.shotUsed && map_.enemyPose().valid &&
        waypoints_.empty() &&
        !has_clear_los(my_robot.x, my_robot.y, target_robot.x, target_robot.y, los_obstacles)) {
        std::cout << "[M5] LOS lost while holding; replanning from (" << (int)my_robot.x << "," << (int)my_robot.y
                  << ")\n";
        (void)replan(walls_, my_robot.x, my_robot.y);
    }

    // Every frame: if nothing blocks the shot (same test as legacy attack), do not drive.
    const bool los_now =
        map_.enemyPose().valid && has_clear_los(my_robot.x, my_robot.y, target_robot.x, target_robot.y, los_obstacles);
    if (map_.enemyPose().valid && !my_robot.shotUsed && los_now) {
        phase_ = Phase::Aim;
        waypoints_.clear();
        waypoint_index_ = 0;
        dbg.path.clear();
        dbg.state = BehaviorState::AimTurret;
        dbg.detail = "M5_LOS_HOLD";
    } else {
        dbg.path = waypoints_;
        dbg.state = BehaviorState::MoveToGoal;
        dbg.detail = "M5_PATH";
    }

    // Rate-limited debug summary + on change.
    const int phase_i = (int)phase_;
    const bool changed =
        (phase_i != lim.last_phase) || ((int)waypoint_index_ != lim.last_wp) || (los_now != lim.last_los);
    if (changed || (t - lim.last_print_t) > 1.0f) {
        lim.last_print_t = t;
        lim.last_phase = phase_i;
        lim.last_wp = (int)waypoint_index_;
        lim.last_los = los_now;
        std::cout << "[M5] t=" << (int)t << "s "
                  << "phase=" << phaseName(phase_i) << " "
                  << "los=" << (los_now ? "Y" : "N") << " "
                  << "wp=" << (int)waypoint_index_ << "/" << waypoints_.size() << " "
                  << "pos=(" << (int)my_robot.x << "," << (int)my_robot.y << ") "
                  << "v=" << (int)my_robot.v << " w=" << my_robot.w << " "
                  << "dbg=" << dbg.detail << "\n";
    }

    if (phase_ == Phase::Drive) {
        if (waypoint_index_ >= waypoints_.size()) {
            phase_ = Phase::Aim;
            aligned_for_fire_frames_ = 0;
        } else {
            const float wx = waypoints_[waypoint_index_].first;
            const float wy = waypoints_[waypoint_index_].second;
            const float dx = wx - my_robot.x;
            const float dy = wy - my_robot.y;
            const float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < params().planner_waypoint_arrival_px) {
                ++waypoint_index_;
            } else {
                const float heading = std::atan2(dy, dx);
                const float fwd = my_robot.theta + kPi / 2.0f;
                const float turn = wrap(heading - fwd);
                my_robot.v = params().planner_follow_speed_px_s;
                my_robot.w = (std::fabs(turn) > 0.12f) ? (turn > 0.0f ? 2.2f : -2.2f) : 0.0f;
            }
        }
    }

    if (phase_ == Phase::Aim) {
        my_robot.v = 0.0f;
        my_robot.w = 0.0f;
        dbg.state = BehaviorState::AimTurret;
        dbg.detail = "M5_AIM";
        if (!map_.enemyPose().valid) {
            phase_ = Phase::Done;
            return;
        }
        const float ang_to =
            std::atan2(map_.enemyPose().y - my_robot.y, map_.enemyPose().x - my_robot.x);
        const float desired_turret = wrap(ang_to - (my_robot.theta + kPi / 2.0f));
        const float turret_limit = 170.0f * kPi / 180.0f / 2.0f;
        if (std::fabs(desired_turret) <= turret_limit) {
            my_robot.turret = desired_turret;
            const float err = std::fabs(wrap(my_robot.turret - desired_turret));
            if (err < 0.12f) {
                ++aligned_for_fire_frames_;
            } else {
                aligned_for_fire_frames_ = 0;
            }
            const int need = std::max(1, params().planner_fire_settle_frames);
            if (!my_robot.shotUsed && aligned_for_fire_frames_ >= need) {
                my_robot.shotUsed = true;
                fired = true;
                fire_time = t;
                hit = true;
                hit_time = t;
                phase_ = Phase::Done;
                aligned_for_fire_frames_ = 0;
                dbg.detail = "M5_FIRE";
                std::cout << "[M5] FIRE t=" << t << " pos=(" << (int)my_robot.x << "," << (int)my_robot.y
                          << ") enemy=(" << (int)map_.enemyPose().x << "," << (int)map_.enemyPose().y << ")\n";
            }
        } else {
            aligned_for_fire_frames_ = 0;
            const float turn = wrap(ang_to - (my_robot.theta + kPi / 2.0f));
            my_robot.w = (turn > 0.0f) ? 2.2f : -2.2f;
        }
    }

    if (phase_ == Phase::Done) {
        my_robot.v = 0.0f;
        my_robot.w = 0.0f;
        dbg.state = BehaviorState::Idle;
        dbg.detail = "M5_DONE";
    }
}

#endif  // USE_VISION

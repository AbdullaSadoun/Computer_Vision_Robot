#include "ArenaMap.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include "../../vision/VisionConfig.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;

inline float dist2(float ax, float ay, float bx, float by) {
    const float dx = ax - bx;
    const float dy = ay - by;
    return dx * dx + dy * dy;
}

Pose2D poseFromMarkers(const VisionCircle& front, const VisionCircle& rear) {
    Pose2D p{};
    p.x = 0.5f * (front.x + rear.x);
    p.y = 0.5f * (front.y + rear.y);
    p.theta = std::atan2(front.y - rear.y, front.x - rear.x);
    while (p.theta > kPi) p.theta -= 2.0f * kPi;
    while (p.theta < -kPi) p.theta += 2.0f * kPi;
    p.valid = true;
    return p;
}

inline void unitForwardLeft(float theta, float& fwd_x, float& fwd_y, float& left_x, float& left_y) {
    fwd_x = std::cos(theta);
    fwd_y = std::sin(theta);
    left_x = -fwd_y;
    left_y = fwd_x;
}

}  // namespace

void ArenaMap::clear() {
    valid_ = false;
    walls_ = ArenaInfo{};
    px_per_inch_ = 0.0f;
    self_pose_ = Pose2D{};
    enemy_pose_ = Pose2D{};
    enemy_front_color_ = MarkerColor::Unknown;
    enemy_rear_color_ = MarkerColor::Unknown;
    static_obstacles_.clear();
    chassis_ghosts_.clear();
    possible_enemy_pairs_.clear();
    build_circles_.clear();
}

bool ArenaMap::findSelfPair(const std::vector<VisionCircle>& c, MarkerColor self_front, float target_px,
                            float d_min, float d_max, int& out_a, int& out_b) const {
    const float d_min2 = d_min * d_min;
    const float d_max2 = d_max * d_max;
    float best_score = std::numeric_limits<float>::max();
    bool found = false;
    for (size_t i = 0; i + 1 < c.size(); ++i) {
        for (size_t j = i + 1; j < c.size(); ++j) {
            const float d2 = dist2(c[i].x, c[i].y, c[j].x, c[j].y);
            if (d2 < d_min2 || d2 > d_max2) continue;
            const bool ab = (c[i].color == MarkerColor::Blue && c[j].color == self_front);
            const bool ba = (c[j].color == MarkerColor::Blue && c[i].color == self_front);
            if (!ab && !ba) continue;
            const float d = std::sqrt(d2);
            const float score = std::fabs(d - target_px);
            if (score < best_score) {
                best_score = score;
                out_a = static_cast<int>(i);
                out_b = static_cast<int>(j);
                found = true;
            }
        }
    }
    return found;
}

bool ArenaMap::buildFromSnapshot(const VisionDetectionSnapshot& snap, MarkerColor self_front_color,
                                 const ArenaInfo& walls, const VisionParameters& vp,
                                 float robot_radius_px, float chassis_slack_px) {
    clear();
    if (!snap.frame_ok || snap.circles.size() < 2 || !snap.pxPerInchValid) {
        return false;
    }
    walls_ = walls;
    px_per_inch_ = snap.pxPerInch;
    build_circles_ = snap.circles;

    const std::vector<VisionCircle>& c = build_circles_;
    const float target_px = px_per_inch_ * vp.robotMarkerSpacingInches;
    const float tol = vp.pairTolerancePct;
    const float d_min = target_px * (1.0f - tol);
    const float d_max = target_px * (1.0f + tol);

    int self_a = -1, self_b = -1;
    if (!findSelfPair(c, self_front_color, target_px, d_min, d_max, self_a, self_b)) {
        return false;
    }

    const VisionCircle& self_circ_a = c[static_cast<size_t>(self_a)];
    const VisionCircle& self_circ_b = c[static_cast<size_t>(self_b)];
    const VisionCircle& self_front = (self_circ_a.color == self_front_color) ? self_circ_a : self_circ_b;
    const VisionCircle& self_rear = (self_circ_a.color == MarkerColor::Blue) ? self_circ_a : self_circ_b;
    self_pose_ = poseFromMarkers(self_front, self_rear);

    std::set<int> consumed;
    consumed.insert(self_a);
    consumed.insert(self_b);

    const float d_min2 = d_min * d_min;
    const float d_max2 = d_max * d_max;

    struct PairCand {
        int a, b;
        float d;
        float score;
    };
    std::vector<PairCand> pairs;
    pairs.reserve(c.size() * 2);
    for (size_t i = 0; i + 1 < c.size(); ++i) {
        for (size_t j = i + 1; j < c.size(); ++j) {
            const float d2 = dist2(c[i].x, c[i].y, c[j].x, c[j].y);
            if (d2 < d_min2 || d2 > d_max2) continue;
            const float d = std::sqrt(d2);
            pairs.push_back({static_cast<int>(i), static_cast<int>(j), d, std::fabs(d - target_px)});
        }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const PairCand& a, const PairCand& b) { return a.score < b.score; });

    // All 9-inch pairs except our robot (enemy may still be in this list for debug).
    possible_enemy_pairs_.clear();
    for (const auto& p : pairs) {
        if (p.a == self_a || p.a == self_b || p.b == self_a || p.b == self_b) continue;
        possible_enemy_pairs_.push_back({p.a, p.b, p.d});
    }

    const PairCand* best_enemy = nullptr;
    for (const auto& p : pairs) {
        if (consumed.count(p.a) || consumed.count(p.b)) continue;
        best_enemy = &p;
        break;
    }

    if (best_enemy) {
        const VisionCircle& a = c[static_cast<size_t>(best_enemy->a)];
        const VisionCircle& b = c[static_cast<size_t>(best_enemy->b)];
        const VisionCircle* front = &a;
        const VisionCircle* rear = &b;
        if (b.x < a.x) {
            front = &b;
            rear = &a;
        }
        enemy_pose_ = poseFromMarkers(*front, *rear);
        enemy_front_color_ = front->color;
        enemy_rear_color_ = rear->color;
        consumed.insert(best_enemy->a);
        consumed.insert(best_enemy->b);
    } else {
        enemy_pose_.valid = false;
    }

    // Chassis ghosts: black circles near self center (not consumed as markers).
    chassis_ghosts_.clear();
    float fwd_x = 0.0f, fwd_y = 0.0f, left_x = 0.0f, left_y = 0.0f;
    unitForwardLeft(self_pose_.theta, fwd_x, fwd_y, left_x, left_y);
    const float cut = robot_radius_px + chassis_slack_px;
    for (size_t i = 0; i < c.size(); ++i) {
        if (consumed.count(static_cast<int>(i))) continue;
        if (c[i].color != MarkerColor::Black) continue;
        const float dx = c[i].x - self_pose_.x;
        const float dy = c[i].y - self_pose_.y;
        if (dx * dx + dy * dy > (cut + c[i].radius) * (cut + c[i].radius)) continue;
        ChassisGhost g{};
        g.rel_fwd = dx * fwd_x + dy * fwd_y;
        g.rel_left = dx * left_x + dy * left_y;
        g.radius = c[i].radius;
        chassis_ghosts_.push_back(g);
        consumed.insert(static_cast<int>(i));
    }

    static_obstacles_.clear();
    for (size_t i = 0; i < c.size(); ++i) {
        if (consumed.count(static_cast<int>(i))) continue;
        Obstacle o{};
        o.x = (int)std::lround(c[i].x);
        o.y = (int)std::lround(c[i].y);
        o.img = nullptr;
        o.radius = c[i].radius;
        static_obstacles_.push_back(o);
    }

    valid_ = true;
    return true;
}

int ArenaMap::reconcileSnapshot(const VisionDetectionSnapshot& snap, MarkerColor self_front_color,
                                const VisionParameters& vp, float robot_radius_px, float chassis_slack_px,
                                float match_gate_px, int max_changes) {
    (void)robot_radius_px;
    (void)chassis_slack_px;
    if (!valid_ || !snap.frame_ok || !snap.pxPerInchValid) {
        return 0;
    }

    const std::vector<VisionCircle>& c = snap.circles;
    const float target_px = snap.pxPerInch * vp.robotMarkerSpacingInches;
    const float tol = vp.pairTolerancePct;
    const float d_min = target_px * (1.0f - tol);
    const float d_max = target_px * (1.0f + tol);

    int self_a = -1, self_b = -1;
    if (!findSelfPair(c, self_front_color, target_px, d_min, d_max, self_a, self_b)) {
        return 0;
    }

    const VisionCircle& ca = c[static_cast<size_t>(self_a)];
    const VisionCircle& cb = c[static_cast<size_t>(self_b)];
    const VisionCircle& sfront = (ca.color == self_front_color) ? ca : cb;
    const VisionCircle& srear = (ca.color == MarkerColor::Blue) ? ca : cb;
    Pose2D new_self = poseFromMarkers(sfront, srear);

    int changes = 0;
    auto bump = [&](bool hit) {
        if (hit) ++changes;
    };

    const float gate2 = match_gate_px * match_gate_px;
    bump(dist2(self_pose_.x, self_pose_.y, new_self.x, new_self.y) > gate2 * 0.25f);
    self_pose_ = new_self;

    if (enemy_pose_.valid) {
        const float d_min2 = d_min * d_min;
        const float d_max2 = d_max * d_max;
        int ea = -1, eb = -1;
        float best_err = 1e9f;
        for (size_t i = 0; i + 1 < c.size(); ++i) {
            if (static_cast<int>(i) == self_a || static_cast<int>(i) == self_b) continue;
            for (size_t j = i + 1; j < c.size(); ++j) {
                if (static_cast<int>(j) == self_a || static_cast<int>(j) == self_b) continue;
                const float d2 = dist2(c[i].x, c[i].y, c[j].x, c[j].y);
                if (d2 < d_min2 || d2 > d_max2) continue;
                const VisionCircle& ci = c[i];
                const VisionCircle& cj = c[j];
                const bool col_ok =
                    (ci.color == enemy_front_color_ && cj.color == enemy_rear_color_) ||
                    (cj.color == enemy_front_color_ && ci.color == enemy_rear_color_);
                if (!col_ok) continue;
                const float err = std::fabs(std::sqrt(d2) - target_px);
                if (err < best_err) {
                    best_err = err;
                    ea = static_cast<int>(i);
                    eb = static_cast<int>(j);
                }
            }
        }
        if (ea >= 0) {
            const VisionCircle& a = c[static_cast<size_t>(ea)];
            const VisionCircle& b = c[static_cast<size_t>(eb)];
            const VisionCircle* front = &a;
            const VisionCircle* rear = &b;
            const float fwd_x = std::cos(enemy_pose_.theta);
            const float fwd_y = std::sin(enemy_pose_.theta);
            const float ax = a.x - enemy_pose_.x;
            const float ay = a.y - enemy_pose_.y;
            const float bx = b.x - enemy_pose_.x;
            const float by = b.y - enemy_pose_.y;
            if (bx * fwd_x + by * fwd_y > ax * fwd_x + ay * fwd_y) {
                front = &b;
                rear = &a;
            }
            Pose2D new_enemy = poseFromMarkers(*front, *rear);
            bump(dist2(enemy_pose_.x, enemy_pose_.y, new_enemy.x, new_enemy.y) > gate2 * 0.25f);
            enemy_pose_ = new_enemy;
        }
    }

    if (changes > max_changes) {
        return -1;
    }
    build_circles_ = snap.circles;
    px_per_inch_ = snap.pxPerInch;
    return changes;
}

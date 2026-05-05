/*
DebugOverlay.cpp
- Implements the vision and simulator debug overlays drawn on top of each processed frame
- markerColorBgr: returns an OpenCV BGR scalar for each MarkerColor enum value
- drawVisionOverlay: annotates detected robots (circles, arrows, labels) and obstacles onto the camera frame
- drawVisionDebug: draws arena polygon, obstacle keepout rings, other-robot keepout disk, AI path, goal, and HUD text using OpenCV
- drawSimDebug: draws only the AI path and goal circle onto an Image24 frame using the Simulation Bresenham primitives
by: Abdulla Sadoun
Date: March 5, 2026
*/
// DebugOverlay.cpp
#include "DebugOverlay.h"

#include "Simulation.h"

#ifdef USE_VISION
#include <opencv2/imgproc.hpp>

cv::Scalar markerColorBgr(MarkerColor c) {
    /*
    Returns the OpenCV BGR display color for a detected marker color.
    Used by drawVisionOverlay to color-code circle outlines and text labels.
    */
    switch (c) {
        case MarkerColor::Black:  return cv::Scalar(40, 40, 40);
        case MarkerColor::Blue:   return cv::Scalar(255, 0, 0);
        case MarkerColor::Green:  return cv::Scalar(0, 255, 0);
        case MarkerColor::Orange: return cv::Scalar(0, 140, 255);
        case MarkerColor::Red:    return cv::Scalar(0, 0, 255);
        default:                  return cv::Scalar(200, 200, 200);
    }
}

void drawVisionOverlay(cv::Mat& bgr, const GameState& s, float self_turret_rel) {
    /*
    Draws self (white) and enemy (cyan) body circles, heading arrows, turret aim arrow, and
    marker outlines. Obstacle circles are labeled with index and color name. px/in scale shown
    at the bottom when valid. All coordinates are in the warped-arena pixel frame.
    */
    if (bgr.empty()) return;

    if (s.arena.valid) {
        std::vector<cv::Point> pts;
        pts.reserve(4);
        for (const auto& c : s.arena.corners)
            pts.emplace_back(cv::Point((int)c.x, (int)c.y));
        const cv::Point* pp = pts.data();
        int npts = (int)pts.size();
        cv::polylines(bgr, &pp, &npts, 1, true, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }

    auto drawRobotOverlay = [&](const RobotTarget& r,
                                cv::Scalar bodyColor,
                                const std::string& label,
                                bool draw_turret,
                                float turret_rel) {
        if (r.frontMarker.valid) {
            cv::circle(bgr, cv::Point((int)r.frontMarker.x, (int)r.frontMarker.y),
                       std::max(4, (int)r.frontMarker.radius),
                       markerColorBgr(r.frontMarker.color), 2, cv::LINE_AA);
        }
        if (r.rearMarker.valid) {
            cv::circle(bgr, cv::Point((int)r.rearMarker.x, (int)r.rearMarker.y),
                       std::max(4, (int)r.rearMarker.radius),
                       markerColorBgr(r.rearMarker.color), 2, cv::LINE_AA);
        }
        if (r.pose.valid) {
            cv::Point center((int)r.pose.x, (int)r.pose.y);
            cv::circle(bgr, center, 28, bodyColor, 2, cv::LINE_AA);

            float fx = std::cos(r.pose.theta);
            float fy = std::sin(r.pose.theta);
            cv::Point tip((int)(r.pose.x + fx * 38.0f), (int)(r.pose.y + fy * 38.0f));
            cv::arrowedLine(bgr, center, tip, bodyColor, 2, cv::LINE_AA, 0, 0.25);

            if (draw_turret) {
                const float aim = r.pose.theta + turret_rel;
                const float tx = std::cos(aim);
                const float ty = std::sin(aim);
                cv::Point ttip((int)(r.pose.x + tx * 44.0f), (int)(r.pose.y + ty * 44.0f));
                cv::arrowedLine(bgr, center, ttip, cv::Scalar(255, 128, 0), 2, cv::LINE_AA, 0, 0.2);
            }

            std::string fullLabel = label;
            if (r.frontMarker.valid) {
                fullLabel += " (front=";
                fullLabel += markerColorName(r.frontMarker.color);
                fullLabel += ")";
            }
            cv::putText(bgr, fullLabel, cv::Point(center.x + 32, center.y - 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, bodyColor, 2, cv::LINE_AA);
        }
    };

    drawRobotOverlay(s.self,  cv::Scalar(255, 255, 255), "OURS",  true,  self_turret_rel);
    drawRobotOverlay(s.enemy, cv::Scalar(0,   255, 255), "ENEMY", false, 0.0f);

    int obs_idx = 0;
    for (const auto& o : s.obstacles) {
        if (!o.valid) continue;
        cv::Point c((int)o.x, (int)o.y);
        cv::Scalar oc = markerColorBgr(o.color);
        cv::circle(bgr, c, std::max(6, (int)o.radius), oc, 2, cv::LINE_AA);
        cv::circle(bgr, c, 2, oc, -1);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "OBS%d (%s)", obs_idx++, markerColorName(o.color));
        cv::putText(bgr, buf, cv::Point(c.x + (int)o.radius + 4, c.y - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42, oc, 1, cv::LINE_AA);
    }

    if (s.pxPerInchValid) {
        char scaleBuf[64];
        std::snprintf(scaleBuf, sizeof(scaleBuf), "px/in: %.2f", s.pxPerInch);
        cv::putText(bgr, scaleBuf, cv::Point(8, bgr.rows - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    }
}

static cv::Scalar hudColor() { return cv::Scalar(240, 240, 240); }
static cv::Scalar pathColor() { return cv::Scalar(255, 200, 0); }
static cv::Scalar goalColor() { return cv::Scalar(0, 255, 255); }
static cv::Scalar keepoutColor() { return cv::Scalar(0, 180, 255); }
static cv::Scalar insetColor() { return cv::Scalar(180, 180, 180); }

static void drawInsetRect(cv::Mat& bgr, const WorldEnv& env, float inset_px) {
    if (bgr.empty()) return;
    if (!env.arena.has_polygon) {
        cv::Rect r((int)std::round(inset_px),
                   (int)std::round(inset_px),
                   std::max(1, bgr.cols - (int)std::round(inset_px * 2.0f)),
                   std::max(1, bgr.rows - (int)std::round(inset_px * 2.0f)));
        cv::rectangle(bgr, r, insetColor(), 1, cv::LINE_AA);
        return;
    }

    // For polygon arenas (warped rect is also a polygon), approximate an inset by drawing
    // parallel edges: sample points along each edge and draw small normals inward is complex;
    // instead, draw per-edge distance bands using the existing edge-reject logic: show as
    // a second polyline offset is non-trivial. We fallback to drawing the original polyline
    // and a small text showing the inset value. The primary safety visualization is via
    // keepout circles + margin text.
}

void drawVisionDebug(cv::Mat& bgr,
                     const GameState& vision,
                     const WorldEnv& env,
                     const AIDebug& ai,
                     const Parameters& p) {
    /*
    Draws the calibrated arena polygon (yellow), obstacle AABB/disk keepout rings (orange),
    AI planned path (yellow polyline), goal marker (cyan circle), and a black HUD bar with behavior state text.
    */
    if (bgr.empty()) return;

    // --- Arena + inset hint ---
    if (vision.arena.valid) {
        std::vector<cv::Point> pts;
        pts.reserve(4);
        for (const auto& c : vision.arena.corners) pts.emplace_back((int)c.x, (int)c.y);
        const cv::Point* pp = pts.data();
        int npts = (int)pts.size();
        cv::polylines(bgr, &pp, &npts, 1, true, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    }

    const float baseInset = env.robot_radius + env.safety_margin;
    drawInsetRect(bgr, env, baseInset);

    // --- Obstacle inflated boundaries ---
    if (env.obstacles) {
        for (const auto& o : *env.obstacles) {
            if (o.img) {
                // Sprite obstacle: draw AABB and inflated AABB.
                const float hw = o.img->w * 0.5f;
                const float hh = o.img->h * 0.5f;
                cv::Rect box((int)std::round(o.x - hw), (int)std::round(o.y - hh),
                             (int)std::round(hw * 2.0f), (int)std::round(hh * 2.0f));
                cv::rectangle(bgr, box, cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
                cv::Rect inf((int)std::round(o.x - hw - baseInset), (int)std::round(o.y - hh - baseInset),
                             (int)std::round((hw + baseInset) * 2.0f), (int)std::round((hh + baseInset) * 2.0f));
                cv::rectangle(bgr, inf, keepoutColor(), 1, cv::LINE_AA);
            } else {
                // Vision obstacle: draw disk + inflated disk.
                cv::Point c((int)std::round(o.x), (int)std::round(o.y));
                const float r = std::max(2.0f, o.radius);
                cv::circle(bgr, c, (int)std::round(r), cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
                cv::circle(bgr, c, (int)std::round(r + baseInset), keepoutColor(), 1, cv::LINE_AA);
            }
        }
    }

    // --- Other robot keepout disk (if present) ---
    if (env.other_robot && env.other_robot->is_alive) {
        cv::Point c((int)std::round(env.other_robot->x), (int)std::round(env.other_robot->y));
        const float r = env.robot_radius * 2.0f + env.safety_margin;
        cv::circle(bgr, c, (int)std::round(r), cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    // --- Planned path + goal ---
    if (!ai.path.empty()) {
        std::vector<cv::Point> pts;
        pts.reserve(ai.path.size());
        for (const auto& pp : ai.path) pts.emplace_back((int)std::round(pp.first), (int)std::round(pp.second));
        cv::polylines(bgr, pts, false, pathColor(), 2, cv::LINE_AA);
        for (const auto& pt : pts) cv::circle(bgr, pt, 2, pathColor(), -1);
    }
    if (ai.goal_valid) {
        cv::Point g((int)std::round(ai.goal_x), (int)std::round(ai.goal_y));
        cv::circle(bgr, g, 6, goalColor(), 2, cv::LINE_AA);
        cv::circle(bgr, g, 2, goalColor(), -1);
    }

    // --- HUD text ---
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "AI: %s%s%s | inset=%.1fpx | lookahead=%.0fpx",
                  behaviorStateName(ai.state),
                  ai.detail.empty() ? "" : " (",
                  ai.detail.empty() ? "" : ai.detail.c_str(),
                  baseInset,
                  (ai.lookahead_px > 0.0f ? ai.lookahead_px : p.steer_lookahead_px));

    if (!ai.detail.empty()) {
        // close paren manually (avoid snprintf complexity)
        std::strncat(buf, ")", sizeof(buf) - std::strlen(buf) - 1);
    }

    const int hud_h = !ai.overlay_note.empty() ? 40 : 22;
    cv::rectangle(bgr, cv::Rect(0, 0, bgr.cols, hud_h), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(bgr, buf, cv::Point(8, 16), cv::FONT_HERSHEY_SIMPLEX, 0.45, hudColor(), 1, cv::LINE_AA);
    if (!ai.overlay_note.empty()) {
        cv::putText(bgr, ai.overlay_note, cv::Point(8, 34), cv::FONT_HERSHEY_SIMPLEX, 0.38, hudColor(), 1,
                    cv::LINE_AA);
    }
}

#endif

void drawSimDebug(Image24& frame, const AIDebug& ai) {
    /*
    Draws the AI path as yellow line segments and a cyan filled circle at the goal position.
    Uses only Simulation drawLine/drawCircle primitives — no OpenCV dependency required.
    */
    // Planned path + goal only (minimal), to keep simulator aligned with robot debug.
    if (ai.path.size() >= 2) {
        for (size_t i = 1; i < ai.path.size(); ++i) {
            int x0 = (int)std::lround(ai.path[i - 1].first);
            int y0 = (int)std::lround(ai.path[i - 1].second);
            int x1 = (int)std::lround(ai.path[i].first);
            int y1 = (int)std::lround(ai.path[i].second);
            drawLine(frame, x0, y0, x1, y1, 255, 200, 0, 1);
        }
    }
    if (ai.goal_valid) {
        drawCircle(frame, (int)std::lround(ai.goal_x), (int)std::lround(ai.goal_y), 6, 0, 255, 255);
    }
}


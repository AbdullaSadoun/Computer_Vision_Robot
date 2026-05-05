/*
DebugOverlay.h
- Declares AI state enums, the AIDebug struct, and overlay drawing functions for both simulator and robot modes
- BehaviorState enum: labels the current AI phase (Idle, SeekShot, AimTurret, MoveToGoal, AvoidObstacle, EdgeRecover, ReturnHome)
- AIDebug struct: carries the active behavior state, detail string, planned path, and goal position for display
- markerColorBgr: maps MarkerColor enum to an OpenCV BGR scalar for vision overlay drawing (USE_VISION only)
- drawVisionOverlay: burns robot circles, heading arrows, turret arrow, and obstacle outlines onto a camera frame (USE_VISION only)
- drawVisionDebug: OpenCV overlay showing arena outline, obstacle keepout rings, AI path, and HUD text (USE_VISION only)
- drawSimDebug: minimal Image24 overlay showing just the planned path and goal marker for the simulator
by: Abdulla Sadoun
Date: March 5, 2026
*/
// DebugOverlay.h
#pragma once

#include <string>
#include <vector>

#include "../CoreTypes.h"
#include "Safety.h"
#include "../Parameters.h"

#ifdef USE_VISION
  #include "../vision/VisionTypes.h"
  #include <opencv2/core.hpp>
#endif

enum class BehaviorState {
    Idle = 0,
    SeekShot,
    AimTurret,
    MoveToGoal,
    AvoidObstacle,
    EdgeRecover,
    ReturnHome
};

inline const char* behaviorStateName(BehaviorState s) {
    switch (s) {
        case BehaviorState::Idle:         return "IDLE";
        case BehaviorState::SeekShot:     return "SEEK_SHOT";
        case BehaviorState::AimTurret:    return "AIM_TURRET";
        case BehaviorState::MoveToGoal:   return "MOVE_TO_GOAL";
        case BehaviorState::AvoidObstacle:return "AVOID_OBSTACLE";
        case BehaviorState::EdgeRecover:  return "EDGE_RECOVER";
        case BehaviorState::ReturnHome:   return "RETURN_HOME";
        default:                          return "UNKNOWN";
    }
}

struct AIDebug {
    BehaviorState state = BehaviorState::Idle;
    std::string detail; // small reason string: "blocked", "detour", etc.
    // Optional second line (e.g. mode-5 map planner status).
    std::string overlay_note;

    bool goal_valid = false;
    float goal_x = 0.0f;
    float goal_y = 0.0f;

    // Path in px coordinates (warped arena frame in robot vision modes).
    std::vector<std::pair<float, float>> path;

    float lookahead_px = 0.0f;
};

#ifdef USE_VISION
// Maps MarkerColor to an OpenCV BGR scalar for circle/text overlays on camera frames.
cv::Scalar markerColorBgr(MarkerColor c);

// Burns robot detection overlays onto the camera frame: body circles, heading arrows,
// turret aim arrow (self only), marker outlines, obstacle labels, and px/in readout.
void drawVisionOverlay(cv::Mat& bgr, const GameState& s, float self_turret_rel = 0.0f);

// Draw additional debug overlays on top of an already-annotated vision frame.
// Assumes all coords are in the same pixel space as `bgr` (warped frame in robot modes).
void drawVisionDebug(cv::Mat& bgr,
                     const GameState& vision,
                     const WorldEnv& env,
                     const AIDebug& ai,
                     const Parameters& p);
#endif

// Optional: draw a minimal debug overlay onto the simulator Image24 frame.
// This uses basic primitives (line/circle) so it works without OpenCV windows.
void drawSimDebug(Image24& frame, const AIDebug& ai);


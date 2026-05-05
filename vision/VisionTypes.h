#pragma once

#include <array>
#include <vector>

// Plain-old-data types shared between the vision module and the game loop.
// Kept OpenCV-free so program.cpp compiles even when USE_VISION is not set
// and the vision module is entirely excluded from the build.

enum class MarkerColor {
    Black = 0,
    Blue,
    Green,
    Orange,
    Red,
    Unknown
};

inline const char* markerColorName(MarkerColor c) {
    switch (c) {
        case MarkerColor::Black:  return "black";
        case MarkerColor::Blue:   return "blue";
        case MarkerColor::Green:  return "green";
        case MarkerColor::Orange: return "orange";
        case MarkerColor::Red:    return "red";
        default:                  return "?";
    }
}

struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
    bool valid = false;
};

struct Marker2D {
    float x = 0.0f;
    float y = 0.0f;
    float radius = 0.0f;
    MarkerColor color = MarkerColor::Unknown;
    bool valid = false;
};

struct RobotTarget {
    Pose2D pose;
    Marker2D frontMarker;
    Marker2D rearMarker;
};

struct VisionObstacle {
    float x = 0.0f;
    float y = 0.0f;
    float radius = 0.0f;
    MarkerColor color = MarkerColor::Unknown;
    bool valid = false;
};

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

struct ArenaBoundary {
    std::array<Point2D, 4> corners{};
    bool valid = false;
};

struct GameState {
    RobotTarget self;
    RobotTarget enemy;
    std::vector<VisionObstacle> obstacles;
    ArenaBoundary arena;
    float pxPerInch = 0.0f;
    bool  pxPerInchValid = false;
};

// OpenCV-free snapshot of raw circle detections for map build / reconcile (mode 5 planner).
struct VisionCircle {
    float x = 0.0f;
    float y = 0.0f;
    float radius = 0.0f;
    MarkerColor color = MarkerColor::Unknown;
    int index = -1;
};

struct VisionDetectionSnapshot {
    std::vector<VisionCircle> circles;
    ArenaBoundary arena;
    float pxPerInch = 0.0f;
    bool pxPerInchValid = false;
    bool frame_ok = false;
};

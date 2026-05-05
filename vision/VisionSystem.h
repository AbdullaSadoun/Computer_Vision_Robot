#pragma once

#include <chrono>
#include <opencv2/opencv.hpp>

#include "VisionConfig.h"
#include "VisionTypes.h"

// Overhead-camera vision module.
//
// Pipeline per frame:
//   1. Threshold the five marker colors (black, blue, green, orange, red) and
//      collect every circle as a tagged CircleDetection.
//   2. Auto-derive pixels-per-inch from the median marker diameter (markers
//      are physically 3 inches across).
//   3. Enumerate marker pairs whose center-to-center distance lies in the
//      9-inch +/- tolerance band. The pair containing one Blue circle plus
//      the operator-selected front color is OUR robot (with continuity
//      preference if multiple candidates exist). Best remaining pair is the
//      ENEMY. All circles not assigned to either robot become OBSTACLES.
//   4. Apply pose-hold timeout so single-frame dropouts don't yank the AI.
//
// The five-color palette means a marker can match an obstacle's color, but
// pair-distance disambiguates: a stray blue circle next to a green obstacle
// only counts as "ours" if their center-to-center distance is exactly the
// expected 9 inches in pixel space, AND the candidate is closer to our
// previously-latched pose than competing candidates.
class VisionSystem {
public:
    VisionSystem();

    bool initialize(int cameraIndex);
    bool grabFrame(cv::Mat& frame);

    GameState processFrame(const cv::Mat& frame);

    // Raw detections for persistent map (no pose-hold). Fails if frame empty.
    bool captureDetections(const cv::Mat& frame, VisionDetectionSnapshot& out);

    VisionParameters& config();
    const VisionParameters& config() const;

    // Operator selects which color is on the front of OUR robot for this run.
    // The rear is hard-wired to Blue. Default is Red.
    void setSelfFrontColor(MarkerColor c);
    MarkerColor getSelfFrontColor() const;

    // Bypass detectArenaBoundary and use the supplied polygon as the arena.
    // Used when the caller is feeding processFrame() a perspective-warped
    // frame whose entire extent IS the arena (in which case the natural
    // contour-finding approach can't recover a polygon since there's no
    // surrounding floor for contrast).
    void setArenaOverride(const ArenaBoundary& a);
    void clearArenaOverride();
    bool hasArenaOverride() const;

    // Fills six mask images for the calibration dashboard. Order:
    // black, blue, green, orange, red, arena. All masks are CV_8UC1.
    void buildDebugMasks(
        const cv::Mat& frame,
        cv::Mat& blackMask,
        cv::Mat& blueMask,
        cv::Mat& greenMask,
        cv::Mat& orangeMask,
        cv::Mat& redMask,
        cv::Mat& arenaMask
    );

private:
    struct CircleDetection {
        cv::Point2f center;
        float radius = 0.0f;
        MarkerColor color = MarkerColor::Unknown;
        int index = -1;  // index into the flat circles vector (set by caller)
    };

    cv::VideoCapture cap;
    VisionParameters parameters;

    MarkerColor selfFrontColor = MarkerColor::Red;

    bool arenaOverrideActive = false;
    ArenaBoundary arenaOverride{};

    Pose2D previousSelfPose{};
    Pose2D previousEnemyPose{};
    std::chrono::steady_clock::time_point lastSelfSeen{};
    std::chrono::steady_clock::time_point lastEnemySeen{};
    bool selfEverSeen = false;
    bool enemyEverSeen = false;

    // Last good px/in survives single-frame drops.
    float lastPxPerInch = 0.0f;

    cv::Mat preprocessFrame(const cv::Mat& frame);
    cv::Mat thresholdRange(const cv::Mat& hsv, const HSVRange& range);
    cv::Mat thresholdTwoBands(const cv::Mat& hsv, const HSVRange& low, const HSVRange& high);
    cv::Mat cleanMask(const cv::Mat& mask);

    // Detects every circle in `mask`, tags them with `color`, and appends to `out`.
    void detectCircles(const cv::Mat& mask, MarkerColor color, float minArea,
                       std::vector<CircleDetection>& out);

    ArenaBoundary detectArenaBoundary(const cv::Mat& hsv);
    bool isInsideArena(const ArenaBoundary& arena, float x, float y) const;
    static std::array<cv::Point2f, 4> sortCorners(const std::vector<cv::Point>& polygon);

    // Median diameter -> px/in. Returns 0 if no usable circles.
    float estimatePxPerInch(const std::vector<CircleDetection>& circles) const;

    RobotTarget buildRobotTargetFromPair(const CircleDetection& front,
                                         const CircleDetection& rear);

    // Apply pose-hold timeout. If detected is invalid but prev was seen
    // within the timeout, the returned robot reports prev as still valid.
    RobotTarget applyPoseHold(
        const RobotTarget& detected,
        Pose2D& previousPose,
        std::chrono::steady_clock::time_point& lastSeen,
        bool& everSeen
    );

    // Shared first stage of processFrame / captureDetections.
    bool collectRawDetections(const cv::Mat& frame, ArenaBoundary& arenaOut, float& pxPerInchOut,
                              bool& pxValidOut, std::vector<CircleDetection>& allOut);
};

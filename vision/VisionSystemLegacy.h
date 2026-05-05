#pragma once

#include <chrono>
#include <vector>
#include "../image_transfer.h"
#include "LegacyVisionConfig.h"
#include "VisionTypes.h"

class VisionSystemLegacy {
public:
    bool initialize(int cameraIndex);
    void shutdown();

    // Acquire one camera frame into internal buffer.
    bool grabFrame();

    // Load an externally-warped frame (e.g. from warpImageLegacy) into the
    // internal buffer so processFrame() operates on the rectified image.
    void loadFrame(const image& src);

    // Run the full detection pipeline on the stored frame. Returns GameState
    // using the same VisionTypes.h structs as VisionSystem (OpenCV path).
    GameState processFrame();

    void        setSelfFrontColor(MarkerColor c) { selfFrontColor_ = c; }
    MarkerColor getSelfFrontColor() const        { return selfFrontColor_; }

    void setArenaOverride(const ArenaBoundary& a) {
        arenaOverride_ = a;
        arenaOverrideActive_ = true;
    }
    bool hasArenaOverride() const { return arenaOverrideActive_; }

    LegacyVisionParams&       config()       { return params_; }
    const LegacyVisionParams& config() const { return params_; }

    // Mode 9: per-color binary mask after threshold, preserved for debug display.
    const image* getDebugMask(MarkerColor c) const;

    // Direct access to the live RGB frame for overlay drawing and display.
    image& getCurrentFrame() { return rgbFrame_; }

private:
    struct CircleDetection {
        float x      = 0.0f;
        float y      = 0.0f;
        float radius = 0.0f;
        MarkerColor color = MarkerColor::Unknown;
        int index = -1;
    };

    int  camIndex_  = 0;
    int  camWidth_  = 640;
    int  camHeight_ = 480;
    bool initialized_ = false;

    LegacyVisionParams params_;

    // Allocated once in initialize(), freed in shutdown().
    image rgbFrame_{};
    image greyTemp_{};
    image greyTemp2_{};
    image labelImg_{};
    image maskBlack_{};
    image maskBlue_{};
    image maskGreen_{};
    image maskOrange_{};
    image maskRed_{};

    MarkerColor selfFrontColor_      = MarkerColor::Red;
    bool        arenaOverrideActive_ = false;
    ArenaBoundary arenaOverride_{};

    Pose2D prevSelfPose_{};
    Pose2D prevEnemyPose_{};
    std::chrono::steady_clock::time_point lastSelfSeen_{};
    std::chrono::steady_clock::time_point lastEnemySeen_{};
    bool  selfEverSeen_  = false;
    bool  enemyEverSeen_ = false;
    float lastPxPerInch_ = 0.0f;

    void extractColorMask(MarkerColor color, image& outMask);

    void detectCircles(const image& binaryMask, MarkerColor color,
                       std::vector<CircleDetection>& out);

    float estimatePxPerInch(const std::vector<CircleDetection>& circles) const;

    RobotTarget buildRobotTarget(const CircleDetection& front,
                                 const CircleDetection& rear) const;

    RobotTarget applyPoseHold(const RobotTarget& detected,
                              Pose2D& prevPose,
                              std::chrono::steady_clock::time_point& lastSeen,
                              bool& everSeen);

    static image makeGreyImage(int w, int h);
    static image makeRgbImage(int w, int h);
    static image makeLabelImage(int w, int h);
};

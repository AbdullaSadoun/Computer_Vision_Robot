#include "VisionSystem.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>

namespace {
constexpr float kPi = 3.14159265358979323846f;

inline float dist2(const cv::Point2f& a, const cv::Point2f& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return dx * dx + dy * dy;
}
}

VisionSystem::VisionSystem() {}

bool VisionSystem::initialize(int cameraIndex) {
    cap.open(cameraIndex);
    if (!cap.isOpened()) {
        std::cout << "VisionSystem: failed to open camera " << cameraIndex << "\n";
        return false;
    }
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    return true;
}

bool VisionSystem::grabFrame(cv::Mat& frame) {
    return cap.read(frame);
}

VisionParameters& VisionSystem::config() { return parameters; }
const VisionParameters& VisionSystem::config() const { return parameters; }

void VisionSystem::setSelfFrontColor(MarkerColor c) {
    if (c == MarkerColor::Blue || c == MarkerColor::Unknown) {
        // Blue is reserved for our rear; refuse to set it as front.
        std::cerr << "VisionSystem::setSelfFrontColor: invalid front color, keeping "
                  << markerColorName(selfFrontColor) << "\n";
        return;
    }
    selfFrontColor = c;
}

MarkerColor VisionSystem::getSelfFrontColor() const {
    return selfFrontColor;
}

void VisionSystem::setArenaOverride(const ArenaBoundary& a) {
    arenaOverride = a;
    arenaOverrideActive = a.valid;
}

void VisionSystem::clearArenaOverride() {
    arenaOverrideActive = false;
}

bool VisionSystem::hasArenaOverride() const {
    return arenaOverrideActive;
}

bool VisionSystem::collectRawDetections(const cv::Mat& frame, ArenaBoundary& arenaOut, float& pxPerInchOut,
                                        bool& pxValidOut, std::vector<CircleDetection>& allOut) {
    allOut.clear();
    pxValidOut = false;
    pxPerInchOut = 0.0f;
    if (frame.empty()) {
        return false;
    }

    const cv::Mat hsv = preprocessFrame(frame);
    arenaOut = arenaOverrideActive ? arenaOverride : detectArenaBoundary(hsv);

    allOut.reserve(32);
    const float minArea = static_cast<float>(parameters.minMarkerArea);
    detectCircles(thresholdRange(hsv, parameters.blackMarker), MarkerColor::Black, minArea, allOut);
    detectCircles(thresholdRange(hsv, parameters.blueMarker), MarkerColor::Blue, minArea, allOut);
    detectCircles(thresholdRange(hsv, parameters.greenMarker), MarkerColor::Green, minArea, allOut);
    detectCircles(thresholdRange(hsv, parameters.orangeMarker), MarkerColor::Orange, minArea, allOut);
    detectCircles(thresholdTwoBands(hsv, parameters.redMarkerLow, parameters.redMarkerHigh),
                  MarkerColor::Red, minArea, allOut);

    if (arenaOut.valid) {
        allOut.erase(std::remove_if(allOut.begin(), allOut.end(),
                         [&](const CircleDetection& c) {
                             return !isInsideArena(arenaOut, c.center.x, c.center.y);
                         }),
                     allOut.end());
    }
    for (size_t i = 0; i < allOut.size(); ++i) {
        allOut[i].index = static_cast<int>(i);
    }

    float pxPerInch = estimatePxPerInch(allOut);
    if (pxPerInch <= 0.0f && lastPxPerInch > 0.0f) {
        pxPerInch = lastPxPerInch;
    }
    if (pxPerInch > 0.0f) {
        lastPxPerInch = pxPerInch;
        pxPerInchOut = pxPerInch;
        pxValidOut = true;
    }
    return true;
}

bool VisionSystem::captureDetections(const cv::Mat& frame, VisionDetectionSnapshot& out) {
    out = VisionDetectionSnapshot{};
    ArenaBoundary arena;
    float pxi = 0.0f;
    bool pxv = false;
    std::vector<CircleDetection> all;
    if (!collectRawDetections(frame, arena, pxi, pxv, all)) {
        return false;
    }
    out.arena = arena;
    out.pxPerInch = pxi;
    out.pxPerInchValid = pxv;
    out.frame_ok = true;
    out.circles.reserve(all.size());
    for (const auto& c : all) {
        VisionCircle vc{};
        vc.x = c.center.x;
        vc.y = c.center.y;
        vc.radius = c.radius;
        vc.color = c.color;
        vc.index = c.index;
        out.circles.push_back(vc);
    }
    return true;
}

GameState VisionSystem::processFrame(const cv::Mat& frame) {
    GameState state{};
    if (frame.empty()) {
        return state;
    }

    ArenaBoundary arena;
    float pxPerInch = 0.0f;
    bool pxValid = false;
    std::vector<CircleDetection> all;
    if (!collectRawDetections(frame, arena, pxPerInch, pxValid, all)) {
        return state;
    }
    state.arena = arena;
    if (pxValid) {
        state.pxPerInch = pxPerInch;
        state.pxPerInchValid = true;
    }

    // 3) Find candidate robot pairs in the 9-in band.
    RobotTarget selfRaw{};
    RobotTarget enemyRaw{};
    std::set<int> consumed;  // indices already used by a robot

    if (pxPerInch > 0.0f && all.size() >= 2) {
        const float targetPx = pxPerInch * parameters.robotMarkerSpacingInches;
        const float tol = parameters.pairTolerancePct;
        const float dMin = targetPx * (1.0f - tol);
        const float dMax = targetPx * (1.0f + tol);
        const float dMin2 = dMin * dMin;
        const float dMax2 = dMax * dMax;

        struct PairCandidate {
            int aIdx;
            int bIdx;
            float distance;
            float score;  // |distance - targetPx|, lower is better
        };
        std::vector<PairCandidate> pairs;
        pairs.reserve(all.size() * 2);

        for (size_t i = 0; i + 1 < all.size(); ++i) {
            for (size_t j = i + 1; j < all.size(); ++j) {
                const float d2 = dist2(all[i].center, all[j].center);
                if (d2 < dMin2 || d2 > dMax2) continue;
                const float d = std::sqrt(d2);
                pairs.push_back({all[i].index, all[j].index, d,
                                 std::fabs(d - targetPx)});
            }
        }
        std::sort(pairs.begin(), pairs.end(),
            [](const PairCandidate& a, const PairCandidate& b) { return a.score < b.score; });

        // 3a) OURS = pair containing exactly one Blue and exactly one selfFrontColor.
        // Tie-break by closeness to previousSelfPose if we've ever seen ourselves.
        const PairCandidate* bestOurs = nullptr;
        float bestOursTie = std::numeric_limits<float>::max();
        for (const auto& p : pairs) {
            const auto& a = all[p.aIdx];
            const auto& b = all[p.bIdx];
            const bool ab = (a.color == MarkerColor::Blue && b.color == selfFrontColor);
            const bool ba = (b.color == MarkerColor::Blue && a.color == selfFrontColor);
            if (!ab && !ba) continue;

            float tie = p.score;
            if (selfEverSeen) {
                const float midX = 0.5f * (a.center.x + b.center.x);
                const float midY = 0.5f * (a.center.y + b.center.y);
                const float dx = midX - previousSelfPose.x;
                const float dy = midY - previousSelfPose.y;
                tie = std::sqrt(dx * dx + dy * dy);
            }
            if (tie < bestOursTie) {
                bestOursTie = tie;
                bestOurs = &p;
            }
        }
        if (bestOurs) {
            const auto& a = all[bestOurs->aIdx];
            const auto& b = all[bestOurs->bIdx];
            const auto& front = (a.color == selfFrontColor) ? a : b;
            const auto& rear  = (a.color == MarkerColor::Blue) ? a : b;
            selfRaw = buildRobotTargetFromPair(front, rear);
            consumed.insert(bestOurs->aIdx);
            consumed.insert(bestOurs->bIdx);
        }

        // 3b) ENEMY = best remaining pair that doesn't share a circle with OURS.
        // Front/rear is unknown a priori; resolve by continuity (closest match
        // to previousEnemyPose mid+heading), else default to "smaller-x is front".
        const PairCandidate* bestEnemy = nullptr;
        for (const auto& p : pairs) {
            if (consumed.count(p.aIdx) || consumed.count(p.bIdx)) continue;
            bestEnemy = &p;
            break;  // pairs is sorted by score, first one wins
        }
        if (bestEnemy) {
            const auto& a = all[bestEnemy->aIdx];
            const auto& b = all[bestEnemy->bIdx];

            // Choose which end is "front" using pose continuity if available.
            const CircleDetection* front = &a;
            const CircleDetection* rear  = &b;
            if (enemyEverSeen) {
                const float fwdX = std::cos(previousEnemyPose.theta);
                const float fwdY = std::sin(previousEnemyPose.theta);
                const float ax = a.center.x - previousEnemyPose.x;
                const float ay = a.center.y - previousEnemyPose.y;
                const float bx = b.center.x - previousEnemyPose.x;
                const float by = b.center.y - previousEnemyPose.y;
                const float aDot = ax * fwdX + ay * fwdY;
                const float bDot = bx * fwdX + by * fwdY;
                if (bDot > aDot) { front = &b; rear = &a; }
            } else if (b.center.x < a.center.x) {
                front = &b; rear = &a;
            }
            enemyRaw = buildRobotTargetFromPair(*front, *rear);
            consumed.insert(bestEnemy->aIdx);
            consumed.insert(bestEnemy->bIdx);
        }
    }

    state.self  = applyPoseHold(selfRaw,  previousSelfPose,  lastSelfSeen,  selfEverSeen);
    state.enemy = applyPoseHold(enemyRaw, previousEnemyPose, lastEnemySeen, enemyEverSeen);

    // 4) Obstacles = every circle not consumed by a robot pair.
    state.obstacles.reserve(all.size());
    for (const auto& c : all) {
        if (consumed.count(c.index)) continue;
        VisionObstacle o{};
        o.x = c.center.x;
        o.y = c.center.y;
        o.radius = c.radius;
        o.color = c.color;
        o.valid = true;
        state.obstacles.push_back(o);
    }

    return state;
}

void VisionSystem::buildDebugMasks(
    const cv::Mat& frame,
    cv::Mat& blackMask,
    cv::Mat& blueMask,
    cv::Mat& greenMask,
    cv::Mat& orangeMask,
    cv::Mat& redMask,
    cv::Mat& arenaMask
) {
    const cv::Mat hsv = preprocessFrame(frame);
    blackMask  = thresholdRange(hsv, parameters.blackMarker);
    blueMask   = thresholdRange(hsv, parameters.blueMarker);
    greenMask  = thresholdRange(hsv, parameters.greenMarker);
    orangeMask = thresholdRange(hsv, parameters.orangeMarker);
    redMask    = thresholdTwoBands(hsv, parameters.redMarkerLow, parameters.redMarkerHigh);
    arenaMask  = thresholdRange(hsv, parameters.arenaSurface);
}

cv::Mat VisionSystem::preprocessFrame(const cv::Mat& frame) {
    int k = std::max(1, parameters.blurKernelSize);
    if (k % 2 == 0) ++k;

    cv::Mat blurred;
    cv::GaussianBlur(frame, blurred, cv::Size(k, k), 0.0);

    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    return hsv;
}

cv::Mat VisionSystem::thresholdRange(const cv::Mat& hsv, const HSVRange& range) {
    cv::Mat mask;
    cv::inRange(
        hsv,
        cv::Scalar(range.hMin, range.sMin, range.vMin),
        cv::Scalar(range.hMax, range.sMax, range.vMax),
        mask
    );
    return cleanMask(mask);
}

cv::Mat VisionSystem::thresholdTwoBands(const cv::Mat& hsv, const HSVRange& low, const HSVRange& high) {
    cv::Mat lowMask, highMask;
    cv::inRange(
        hsv,
        cv::Scalar(low.hMin, low.sMin, low.vMin),
        cv::Scalar(low.hMax, low.sMax, low.vMax),
        lowMask
    );
    cv::inRange(
        hsv,
        cv::Scalar(high.hMin, high.sMin, high.vMin),
        cv::Scalar(high.hMax, high.sMax, high.vMax),
        highMask
    );
    cv::Mat combined;
    cv::bitwise_or(lowMask, highMask, combined);
    return cleanMask(combined);
}

cv::Mat VisionSystem::cleanMask(const cv::Mat& mask) {
    int k = std::max(1, parameters.morphKernelSize);
    if (k % 2 == 0) ++k;

    cv::Mat cleaned;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(k, k));
    cv::morphologyEx(mask, cleaned, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(cleaned, cleaned, cv::MORPH_CLOSE, kernel);
    return cleaned;
}

void VisionSystem::detectCircles(const cv::Mat& mask, MarkerColor color, float minArea,
                                 std::vector<CircleDetection>& out) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < minArea) continue;

        // Reject very non-circular blobs: actual area / minEnclosingCircle area
        // should be > ~0.55 for our 3-inch disks.
        CircleDetection c{};
        c.color = color;
        cv::minEnclosingCircle(contour, c.center, c.radius);
        if (c.radius < 1.0f) continue;
        const double circleArea = kPi * c.radius * c.radius;
        if (circleArea > 0.0 && area / circleArea < 0.55) continue;

        out.push_back(c);
    }
}

float VisionSystem::estimatePxPerInch(const std::vector<CircleDetection>& circles) const {
    if (circles.empty()) return 0.0f;

    std::vector<float> diameters;
    diameters.reserve(circles.size());
    for (const auto& c : circles) diameters.push_back(c.radius * 2.0f);

    std::sort(diameters.begin(), diameters.end());
    const float median = diameters[diameters.size() / 2];
    if (median <= 0.0f || parameters.markerDiameterInches <= 0.0f) return 0.0f;

    float pxPerInch = median / parameters.markerDiameterInches;
    pxPerInch = std::max(parameters.minPxPerInchClamp,
                std::min(parameters.maxPxPerInchClamp, pxPerInch));
    return pxPerInch;
}

ArenaBoundary VisionSystem::detectArenaBoundary(const cv::Mat& hsv) {
    ArenaBoundary arena{};

    const cv::Mat arenaMask = thresholdRange(hsv, parameters.arenaSurface);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(arenaMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestArea = 0.0;
    std::vector<cv::Point> bestPolygon;
    for (const auto& contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < static_cast<double>(parameters.minArenaArea)) continue;

        const double perimeter = cv::arcLength(contour, true);
        std::vector<cv::Point> polygon;
        cv::approxPolyDP(contour, polygon, 0.02 * perimeter, true);

        if (polygon.size() != 4 || !cv::isContourConvex(polygon)) continue;

        if (area > bestArea) {
            bestArea = area;
            bestPolygon = polygon;
        }
    }

    if (bestPolygon.size() == 4) {
        const auto sorted = sortCorners(bestPolygon);
        for (size_t i = 0; i < sorted.size(); ++i) {
            arena.corners[i] = Point2D{sorted[i].x, sorted[i].y};
        }
        arena.valid = true;
    }
    return arena;
}

bool VisionSystem::isInsideArena(const ArenaBoundary& arena, float x, float y) const {
    if (!arena.valid) {
        return true;
    }
    std::vector<cv::Point2f> polygon;
    polygon.reserve(arena.corners.size());
    for (const auto& corner : arena.corners) {
        polygon.emplace_back(corner.x, corner.y);
    }
    return cv::pointPolygonTest(polygon, cv::Point2f(x, y), false) >= 0.0;
}

std::array<cv::Point2f, 4> VisionSystem::sortCorners(const std::vector<cv::Point>& polygon) {
    std::array<cv::Point2f, 4> sorted{};
    std::vector<cv::Point2f> corners;
    corners.reserve(4);
    for (const auto& point : polygon) {
        corners.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
    }

    std::sort(corners.begin(), corners.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) {
            if (a.y == b.y) return a.x < b.x;
            return a.y < b.y;
        });

    cv::Point2f topLeft, topRight, bottomLeft, bottomRight;
    if (corners[0].x < corners[1].x) { topLeft = corners[0]; topRight = corners[1]; }
    else                             { topLeft = corners[1]; topRight = corners[0]; }
    if (corners[2].x < corners[3].x) { bottomLeft = corners[2]; bottomRight = corners[3]; }
    else                             { bottomLeft = corners[3]; bottomRight = corners[2]; }

    sorted[0] = topLeft;
    sorted[1] = topRight;
    sorted[2] = bottomRight;
    sorted[3] = bottomLeft;
    return sorted;
}

RobotTarget VisionSystem::buildRobotTargetFromPair(const CircleDetection& front,
                                                   const CircleDetection& rear) {
    RobotTarget robot{};

    robot.frontMarker.x = front.center.x;
    robot.frontMarker.y = front.center.y;
    robot.frontMarker.radius = front.radius;
    robot.frontMarker.color = front.color;
    robot.frontMarker.valid = true;

    robot.rearMarker.x = rear.center.x;
    robot.rearMarker.y = rear.center.y;
    robot.rearMarker.radius = rear.radius;
    robot.rearMarker.color = rear.color;
    robot.rearMarker.valid = true;

    robot.pose.x = (front.center.x + rear.center.x) * 0.5f;
    robot.pose.y = (front.center.y + rear.center.y) * 0.5f;
    robot.pose.theta = std::atan2(front.center.y - rear.center.y, front.center.x - rear.center.x);
    robot.pose.valid = true;

    while (robot.pose.theta > kPi) robot.pose.theta -= 2.0f * kPi;
    while (robot.pose.theta < -kPi) robot.pose.theta += 2.0f * kPi;

    return robot;
}

RobotTarget VisionSystem::applyPoseHold(
    const RobotTarget& detected,
    Pose2D& previousPose,
    std::chrono::steady_clock::time_point& lastSeen,
    bool& everSeen
) {
    const auto now = std::chrono::steady_clock::now();

    if (detected.pose.valid) {
        previousPose = detected.pose;
        lastSeen = now;
        everSeen = true;
        return detected;
    }

    if (!everSeen) {
        return RobotTarget{};
    }

    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastSeen).count();
    if (elapsed <= parameters.poseHoldMs) {
        RobotTarget held{};
        held.pose = previousPose;
        held.pose.valid = true;
        return held;
    }

    return RobotTarget{};
}

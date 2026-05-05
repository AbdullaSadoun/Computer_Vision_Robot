#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <algorithm>
#include <chrono>

#include "../image_transfer.h"
#include "vision.h"
#include "VisionSystemLegacy.h"
#include "../Simulation/Safety.h"

// ── image helpers ─────────────────────────────────────────────────────────────

image VisionSystemLegacy::makeGreyImage(int w, int h) {
    image img{};
    img.type   = GREY_IMAGE;
    img.width  = (i2byte)w;
    img.height = (i2byte)h;
    img.pdata  = (ibyte*)malloc((size_t)w * h);
    if (img.pdata) memset(img.pdata, 0, (size_t)w * h);
    return img;
}

image VisionSystemLegacy::makeRgbImage(int w, int h) {
    image img{};
    img.type   = RGB_IMAGE;
    img.width  = (i2byte)w;
    img.height = (i2byte)h;
    img.pdata  = (ibyte*)malloc((size_t)w * h * 3);
    if (img.pdata) memset(img.pdata, 0, (size_t)w * h * 3);
    return img;
}

image VisionSystemLegacy::makeLabelImage(int w, int h) {
    image img{};
    img.type   = LABEL_IMAGE;
    img.width  = (i2byte)w;
    img.height = (i2byte)h;
    img.pdata  = (ibyte*)malloc((size_t)w * h * 2);
    if (img.pdata) memset(img.pdata, 0, (size_t)w * h * 2);
    return img;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

bool VisionSystemLegacy::initialize(int cameraIndex) {
    if (initialized_) shutdown();

    camIndex_  = cameraIndex;
    camWidth_  = params_.camWidth;
    camHeight_ = params_.camHeight;

    // NOTE: activate_camera takes (cam, HEIGHT, WIDTH) — height comes first.
    if (activate_camera(camIndex_, camHeight_, camWidth_) != 0)
        return false;

    // rgbFrame_ needs shared memory so acquire_image can write into it.
    rgbFrame_.type   = RGB_IMAGE;
    rgbFrame_.width  = (i2byte)camWidth_;
    rgbFrame_.height = (i2byte)camHeight_;
    if (allocate_image(rgbFrame_) != 0) {
        stop_camera(camIndex_);
        return false;
    }

    // Processing images use plain malloc — they never go to shared memory display.
    greyTemp_  = makeGreyImage(camWidth_, camHeight_);
    greyTemp2_ = makeGreyImage(camWidth_, camHeight_);
    labelImg_  = makeLabelImage(camWidth_, camHeight_);
    maskBlack_  = makeGreyImage(camWidth_, camHeight_);
    maskBlue_   = makeGreyImage(camWidth_, camHeight_);
    maskGreen_  = makeGreyImage(camWidth_, camHeight_);
    maskOrange_ = makeGreyImage(camWidth_, camHeight_);
    maskRed_    = makeGreyImage(camWidth_, camHeight_);

    initialized_ = true;
    return true;
}

void VisionSystemLegacy::shutdown() {
    if (!initialized_) return;

    free_image(rgbFrame_);
    stop_camera(camIndex_);

    free(greyTemp_.pdata);   greyTemp_.pdata  = nullptr;
    free(greyTemp2_.pdata);  greyTemp2_.pdata = nullptr;
    free(labelImg_.pdata);   labelImg_.pdata  = nullptr;
    free(maskBlack_.pdata);  maskBlack_.pdata  = nullptr;
    free(maskBlue_.pdata);   maskBlue_.pdata   = nullptr;
    free(maskGreen_.pdata);  maskGreen_.pdata  = nullptr;
    free(maskOrange_.pdata); maskOrange_.pdata = nullptr;
    free(maskRed_.pdata);    maskRed_.pdata    = nullptr;

    initialized_ = false;
}

bool VisionSystemLegacy::grabFrame() {
    if (!initialized_) return false;
    return acquire_image(rgbFrame_, camIndex_) == 0;
}

void VisionSystemLegacy::loadFrame(const image& src) {
    if (!initialized_) return;
    if (src.width != rgbFrame_.width || src.height != rgbFrame_.height) return;
    if (src.type == RGB_IMAGE)
        memcpy(rgbFrame_.pdata, src.pdata, (size_t)camWidth_ * camHeight_ * 3);
}

const image* VisionSystemLegacy::getDebugMask(MarkerColor c) const {
    switch (c) {
        case MarkerColor::Black:  return &maskBlack_;
        case MarkerColor::Blue:   return &maskBlue_;
        case MarkerColor::Green:  return &maskGreen_;
        case MarkerColor::Orange: return &maskOrange_;
        case MarkerColor::Red:    return &maskRed_;
        default: return nullptr;
    }
}

// ── color extraction ──────────────────────────────────────────────────────────

static inline int clamp255(int v) {
    return v < 0 ? 0 : (v > 255 ? 255 : v);
}

void VisionSystemLegacy::extractColorMask(MarkerColor color, image& outMask) {
    const int N = camWidth_ * camHeight_;
    const ibyte* src = rgbFrame_.pdata;
    ibyte* dst = greyTemp_.pdata;

    const LegacyColorConfig* cfg = nullptr;
    switch (color) {
        case MarkerColor::Black:  cfg = &params_.black;  break;
        case MarkerColor::Blue:   cfg = &params_.blue;   break;
        case MarkerColor::Green:  cfg = &params_.green;  break;
        case MarkerColor::Orange: cfg = &params_.orange; break;
        case MarkerColor::Red:    cfg = &params_.red;    break;
        default: return;
    }

    // Channel-difference extraction: BGR layout (byte 0=B, 1=G, 2=R).
    for (int i = 0; i < N; ++i) {
        int B = src[3 * i + 0];
        int G = src[3 * i + 1];
        int R = src[3 * i + 2];
        int val = 0;
        switch (color) {
            case MarkerColor::Black:
                val = clamp255(128 - std::max({R, G, B}));
                break;
            case MarkerColor::Blue:
                val = clamp255(B - std::max(R, G));
                break;
            case MarkerColor::Green:
                // max(G,B)-R detects both pure green AND cyan/teal (G≈B).
                // Gate G>=B*0.55 prevents pure blue (G<<B) from triggering.
                val = (G >= (int)(B * 0.55f)) ? clamp255(std::max(G, B) - R) : 0;
                break;
            case MarkerColor::Orange:
                val = (R > (int)(G * 1.3f)) ? clamp255(R - B) : 0;
                break;
            case MarkerColor::Red:
                val = (G < (int)(R * 0.75f)) ? clamp255(2 * R - G - B) : 0;
                break;
            default:
                break;
        }
        dst[i] = (ibyte)val;
    }

    // Gaussian smooth → threshold → morphological open/close.
    gaussian_filter(greyTemp_, greyTemp2_);
    threshold(greyTemp2_, outMask, cfg->channelThreshold);

    for (int p = 0; p < cfg->morphPasses; ++p) {
        erode(outMask, greyTemp_);
        dialate(greyTemp_, outMask);
        dialate(outMask, greyTemp_);
        copy(greyTemp_, outMask);
    }
}

// ── circle detection ──────────────────────────────────────────────────────────

void VisionSystemLegacy::detectCircles(const image& binaryMask, MarkerColor color,
                                        std::vector<CircleDetection>& out) {
    // label_image destructively zeroes the first (width+1) pixels of its input.
    // Copy the mask into greyTemp_ so outMask (= maskColor_) is preserved for
    // Mode 9 debug display.
    copy(const_cast<image&>(binaryMask), greyTemp_);

    int nlabels = 0;
    // Zero the label image before re-use to avoid stale labels.
    memset(labelImg_.pdata, 0, (size_t)camWidth_ * camHeight_ * 2);
    label_image(greyTemp_, labelImg_, nlabels);
    if (nlabels <= 0) return;

    const int minArea = (color == MarkerColor::Black)  ? params_.black.minAreaPx
                      : (color == MarkerColor::Blue)   ? params_.blue.minAreaPx
                      : (color == MarkerColor::Green)  ? params_.green.minAreaPx
                      : (color == MarkerColor::Orange) ? params_.orange.minAreaPx
                      :                                  params_.red.minAreaPx;

    // Count pixel area per label in one O(W×H) pass.
    std::vector<int> areas(nlabels + 1, 0);
    const i2byte* lp = reinterpret_cast<const i2byte*>(labelImg_.pdata);
    for (int k = 0; k < camWidth_ * camHeight_; ++k) {
        if (lp[k] > 0 && lp[k] <= nlabels) areas[lp[k]]++;
    }

    for (int label = 1; label <= nlabels; ++label) {
        if (areas[label] < minArea) continue;

        double ic = 0.0, jc = 0.0;
        // centroid needs GREY_IMAGE for 'a'; greyTemp_ still holds the copy
        // (label_image may have zeroed the first row, but centroid only needs
        // the label image for region membership — intensity weights just come
        // from whatever is in greyTemp_; uniform weight is fine since after
        // label_image the first row is zeroed but all other blob pixels remain).
        centroid(greyTemp_, labelImg_, label, ic, jc);

        float radius = std::sqrt((float)areas[label] / 3.14159265f);

        CircleDetection cd;
        cd.x      = (float)ic;
        cd.y      = (float)jc;
        cd.radius = radius;
        cd.color  = color;
        cd.index  = label;
        out.push_back(cd);
    }
}

// ── helpers ───────────────────────────────────────────────────────────────────

float VisionSystemLegacy::estimatePxPerInch(const std::vector<CircleDetection>& circles) const {
    if (circles.empty() || params_.markerDiameterIn <= 0.0f) return 0.0f;

    std::vector<float> estimates;
    estimates.reserve(circles.size());
    for (const auto& c : circles) {
        if (c.radius > 0.0f) {
            float ppi = (c.radius * 2.0f) / params_.markerDiameterIn;
            estimates.push_back(ppi);
        }
    }
    if (estimates.empty()) return 0.0f;

    std::sort(estimates.begin(), estimates.end());
    float median = estimates[estimates.size() / 2];
    // Clamp to a sane range: 4..30 px/in
    return std::max(4.0f, std::min(30.0f, median));
}

RobotTarget VisionSystemLegacy::buildRobotTarget(const CircleDetection& front,
                                                   const CircleDetection& rear) const {
    RobotTarget rt{};
    float cx = (front.x + rear.x) * 0.5f;
    float cy = (front.y + rear.y) * 0.5f;
    float dx = front.x - rear.x;
    float dy = front.y - rear.y;
    float dist = std::sqrt(dx * dx + dy * dy);

    rt.pose.x = cx;
    rt.pose.y = cy;
    rt.pose.theta = std::atan2(dy, dx);
    rt.pose.valid = true;

    rt.frontMarker.x      = front.x;
    rt.frontMarker.y      = front.y;
    rt.frontMarker.radius = front.radius;
    rt.frontMarker.color  = front.color;
    rt.frontMarker.valid  = true;

    rt.rearMarker.x      = rear.x;
    rt.rearMarker.y      = rear.y;
    rt.rearMarker.radius = rear.radius;
    rt.rearMarker.color  = MarkerColor::Blue;
    rt.rearMarker.valid  = true;

    (void)dist;
    return rt;
}

RobotTarget VisionSystemLegacy::applyPoseHold(const RobotTarget& detected,
                                               Pose2D& prevPose,
                                               std::chrono::steady_clock::time_point& lastSeen,
                                               bool& everSeen) {
    auto now = std::chrono::steady_clock::now();

    if (detected.pose.valid) {
        prevPose = detected.pose;
        lastSeen = now;
        everSeen = true;
        return detected;
    }

    if (!everSeen) return detected;  // never detected: return invalid

    float elapsed_ms = std::chrono::duration<float, std::milli>(now - lastSeen).count();
    if (elapsed_ms < (float)params_.poseHoldMs) {
        RobotTarget held = detected;
        held.pose = prevPose;
        held.pose.valid = true;
        return held;
    }

    return detected;  // dropout expired: invalid pose
}

// ── processFrame ─────────────────────────────────────────────────────────────

GameState VisionSystemLegacy::processFrame() {
    GameState gs{};
    if (!initialized_) return gs;

    // 1. Extract color masks for all 5 colors.
    extractColorMask(MarkerColor::Black,  maskBlack_);
    extractColorMask(MarkerColor::Blue,   maskBlue_);
    extractColorMask(MarkerColor::Green,  maskGreen_);
    extractColorMask(MarkerColor::Orange, maskOrange_);
    extractColorMask(MarkerColor::Red,    maskRed_);

    // 2. Detect circles per color.
    std::vector<CircleDetection> allCircles;

    // Per-color vectors for counting before merging.
    std::vector<CircleDetection> dbg_black, dbg_blue, dbg_green, dbg_orange, dbg_red;
    detectCircles(maskBlack_,  MarkerColor::Black,  dbg_black);
    detectCircles(maskBlue_,   MarkerColor::Blue,   dbg_blue);
    detectCircles(maskGreen_,  MarkerColor::Green,  dbg_green);
    detectCircles(maskOrange_, MarkerColor::Orange, dbg_orange);
    detectCircles(maskRed_,    MarkerColor::Red,    dbg_red);
    for (auto& c : dbg_black)  allCircles.push_back(c);
    for (auto& c : dbg_blue)   allCircles.push_back(c);
    for (auto& c : dbg_green)  allCircles.push_back(c);
    for (auto& c : dbg_orange) allCircles.push_back(c);
    for (auto& c : dbg_red)    allCircles.push_back(c);

    // Throttled debug: print counts every 60 frames.
    static int s_dbgN = 0;
    const bool dbgOn = (++s_dbgN % 60 == 1);
    if (dbgOn) {
        printf("[VS] circles: black=%d blue=%d green=%d orange=%d red=%d  frontColor=%d\n",
               (int)dbg_black.size(), (int)dbg_blue.size(), (int)dbg_green.size(),
               (int)dbg_orange.size(), (int)dbg_red.size(), (int)selfFrontColor_);
    }

    // 3. Optional arena gate: discard circles outside the polygon.
    std::vector<CircleDetection> circles;
    if (arenaOverrideActive_ && arenaOverride_.valid) {
        float poly[4][2];
        for (int k = 0; k < 4; ++k) {
            poly[k][0] = arenaOverride_.corners[k].x;
            poly[k][1] = arenaOverride_.corners[k].y;
        }
        for (const auto& c : allCircles) {
            if (point_in_polygon(c.x, c.y, poly, 4))
                circles.push_back(c);
        }
        gs.arena = arenaOverride_;
    } else {
        circles = allCircles;
    }

    // 4. Estimate px/in from all circle radii; cache last valid value.
    float ppi = estimatePxPerInch(circles);
    if (ppi > 0.0f) lastPxPerInch_ = ppi;
    gs.pxPerInch      = lastPxPerInch_;
    gs.pxPerInchValid = (lastPxPerInch_ > 0.0f);

    // 5. Pair matching: identify self and enemy robot marker pairs.
    //    Self  = (Blue rear) + (selfFrontColor_ front)
    //    Enemy = best remaining pair of any two colors within distance gate

    const int pairMinPx = params_.pairMinPx;
    const int pairMaxPx = params_.pairMaxPx;

    // Separate circles by color.
    std::vector<CircleDetection> blueCircles, frontColorCircles;
    std::vector<CircleDetection> remaining;
    for (const auto& c : circles) {
        if (c.color == MarkerColor::Blue)
            blueCircles.push_back(c);
        else if (c.color == selfFrontColor_)
            frontColorCircles.push_back(c);
        else
            remaining.push_back(c);
    }

    // Find the best (closest within gate) blue+selfFrontColor pair = self robot.
    RobotTarget selfDetected{};
    int bestBlueIdx  = -1;
    int bestFrontIdx = -1;
    float bestSelfDist = 1e9f;

    for (int bi = 0; bi < (int)blueCircles.size(); ++bi) {
        for (int fi = 0; fi < (int)frontColorCircles.size(); ++fi) {
            float dx = blueCircles[bi].x - frontColorCircles[fi].x;
            float dy = blueCircles[bi].y - frontColorCircles[fi].y;
            float d  = std::sqrt(dx * dx + dy * dy);
            if (d >= pairMinPx && d <= pairMaxPx && d < bestSelfDist) {
                bestSelfDist  = d;
                bestBlueIdx   = bi;
                bestFrontIdx  = fi;
            }
        }
    }

    if (bestBlueIdx >= 0 && bestFrontIdx >= 0) {
        selfDetected = buildRobotTarget(frontColorCircles[bestFrontIdx],
                                        blueCircles[bestBlueIdx]);
    }

    // Debug: print self-pair search details every 60 frames.
    if (dbgOn) {
        printf("[VS] pairGate=[%d, %d] blue=%d front=%d\n",
               pairMinPx, pairMaxPx,
               (int)blueCircles.size(), (int)frontColorCircles.size());
        for (int bi = 0; bi < (int)blueCircles.size(); ++bi) {
            printf("  blue[%d] x=%.1f y=%.1f r=%.1f\n",
                   bi, blueCircles[bi].x, blueCircles[bi].y, blueCircles[bi].radius);
        }
        for (int fi = 0; fi < (int)frontColorCircles.size(); ++fi) {
            printf("  front[%d] x=%.1f y=%.1f r=%.1f\n",
                   fi, frontColorCircles[fi].x, frontColorCircles[fi].y, frontColorCircles[fi].radius);
        }
        // Print all pair distances.
        for (int bi = 0; bi < (int)blueCircles.size(); ++bi) {
            for (int fi = 0; fi < (int)frontColorCircles.size(); ++fi) {
                float dx = blueCircles[bi].x - frontColorCircles[fi].x;
                float dy = blueCircles[bi].y - frontColorCircles[fi].y;
                float d  = std::sqrt(dx*dx + dy*dy);
                const char* verdict = (d >= pairMinPx && d <= pairMaxPx) ? "MATCH" : "REJECT";
                printf("  pair b%d+f%d dist=%.1f -> %s\n", bi, fi, d, verdict);
            }
        }
        printf("[VS] self=%s bestDist=%.1f\n",
               (bestBlueIdx>=0?"FOUND":"NOT_FOUND"), bestSelfDist);
    }

    // Collect circles NOT used by self into the candidate pool for enemy + obstacles.
    // Also exclude circles within 15px of either matched self marker — the teal rear
    // marker bleeds into the green channel, and its ghost at ~0px offset would otherwise
    // pair with a nearby black/orange blob to form a fake "enemy" at the user's own robot.
    const float kSelfExcludeSq = 15.0f * 15.0f;
    float selfRearX = (bestBlueIdx >= 0)  ? blueCircles[bestBlueIdx].x      : -9999.f;
    float selfRearY = (bestBlueIdx >= 0)  ? blueCircles[bestBlueIdx].y      : -9999.f;
    float selfFrontX = (bestFrontIdx >= 0) ? frontColorCircles[bestFrontIdx].x : -9999.f;
    float selfFrontY = (bestFrontIdx >= 0) ? frontColorCircles[bestFrontIdx].y : -9999.f;

    auto nearSelf = [&](const CircleDetection& c) {
        float dxr = c.x - selfRearX,  dyr = c.y - selfRearY;
        float dxf = c.x - selfFrontX, dyf = c.y - selfFrontY;
        return (dxr*dxr + dyr*dyr < kSelfExcludeSq) ||
               (dxf*dxf + dyf*dyf < kSelfExcludeSq);
    };

    std::vector<CircleDetection> pool;
    for (int bi = 0; bi < (int)blueCircles.size(); ++bi)
        if (bi != bestBlueIdx && !nearSelf(blueCircles[bi])) pool.push_back(blueCircles[bi]);
    for (int fi = 0; fi < (int)frontColorCircles.size(); ++fi)
        if (fi != bestFrontIdx && !nearSelf(frontColorCircles[fi])) pool.push_back(frontColorCircles[fi]);
    for (const auto& c : remaining)
        if (!nearSelf(c)) pool.push_back(c);

    // Find best pair in pool = enemy robot (any two colors within gate).
    RobotTarget enemyDetected{};
    int bestEnemyA = -1, bestEnemyB = -1;
    float bestEnemyDist = 1e9f;

    for (int ai = 0; ai < (int)pool.size(); ++ai) {
        for (int bi2 = ai + 1; bi2 < (int)pool.size(); ++bi2) {
            float dx = pool[ai].x - pool[bi2].x;
            float dy = pool[ai].y - pool[bi2].y;
            float d  = std::sqrt(dx * dx + dy * dy);
            if (d >= pairMinPx && d <= pairMaxPx && d < bestEnemyDist) {
                bestEnemyDist = d;
                bestEnemyA    = ai;
                bestEnemyB    = bi2;
            }
        }
    }

    if (bestEnemyA >= 0 && bestEnemyB >= 0) {
        enemyDetected = buildRobotTarget(pool[bestEnemyA], pool[bestEnemyB]);
    }

    if (dbgOn) {
        printf("[VS] enemy=%s pool=%d\n",
               (bestEnemyA>=0?"FOUND":"NOT_FOUND"), (int)pool.size());
        if (bestEnemyA >= 0) {
            printf("  enemy pair: pool[%d](%.1f,%.1f,col=%d) + pool[%d](%.1f,%.1f,col=%d) dist=%.1f\n",
                   bestEnemyA, pool[bestEnemyA].x, pool[bestEnemyA].y, (int)pool[bestEnemyA].color,
                   bestEnemyB, pool[bestEnemyB].x, pool[bestEnemyB].y, (int)pool[bestEnemyB].color,
                   bestEnemyDist);
        }
    }

    // 6. Pose hold.
    gs.self  = applyPoseHold(selfDetected,  prevSelfPose_,  lastSelfSeen_,  selfEverSeen_);
    gs.enemy = applyPoseHold(enemyDetected, prevEnemyPose_, lastEnemySeen_, enemyEverSeen_);

    // 7. Obstacles = circles not consumed by either robot pair.
    std::vector<bool> usedPool(pool.size(), false);
    if (bestEnemyA >= 0) usedPool[bestEnemyA] = true;
    if (bestEnemyB >= 0) usedPool[bestEnemyB] = true;

    for (int i = 0; i < (int)pool.size(); ++i) {
        if (usedPool[i]) continue;
        VisionObstacle vo{};
        vo.x      = pool[i].x;
        vo.y      = pool[i].y;
        vo.radius = pool[i].radius;
        vo.color  = pool[i].color;
        vo.valid  = true;
        gs.obstacles.push_back(vo);
    }

    return gs;
}

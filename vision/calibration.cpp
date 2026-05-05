/*
calibration.cpp
- Mode 7: HSV calibration dashboard — prompts for optional corner re-pick then runs runCalibrationDashboard()
- Mode 8: Vision debug loop — grabs warped camera frames, runs detection, draws overlay, streams to image_view
- runMode7: entry point called from program.cpp for mode '7'
- runMode8: entry point called from program.cpp for mode '8'
by: Abdulla Sadoun
Date: March 25, 2026
*/

#ifdef USE_VISION

#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

#include <conio.h>
#include <windows.h>
#include <opencv2/opencv.hpp>

#include "../Parameters.h"
#include "../CoreTypes.h"
#include "../Simulation/Simulation.h"
#include "../Simulation/DebugOverlay.h"
#include "VisionSystem.h"

// Forward declarations from VisionCalibration.cpp.
void runCalibrationDashboard(VisionSystem& vision);
bool loadArenaCorners(const std::string& path, cv::Point2f out[4]);
bool saveArenaCorners(const std::string& path, const cv::Point2f in[4]);
bool runArenaCornerPicker(VisionSystem& vision, cv::Point2f outCorners[4]);
ArenaBoundary makeWarpedArenaBoundary();
cv::Mat buildWarpToRect(const cv::Point2f src[4]);

static inline int kWarpedFrameW() { return params().warped_w; }
static inline int kWarpedFrameH() { return params().warped_h; }
static inline const char* kArenaCornersFile() { return params().arena_corners_file; }
static inline int kCameraIndex() { return params().camera_index; }

void runMode7() {
    /*
    Opens the camera and optionally re-picks arena corners before handing off to the
    HSV slider dashboard. ESC inside the picker aborts the pick without touching the
    saved file.
    */
    VisionSystem visionOnly;
    if (!visionOnly.initialize(kCameraIndex())) {
        std::cerr << "Camera open failed (mode 7). Check camera index " << kCameraIndex() << ".\n";
        return;
    }

    std::cout << "\n[Mode 7] Re-pick arena corners first? [y/n]: " << std::flush;
    bool rePick = false;
    while (true) {
        if (_kbhit()) {
            char c = (char)std::tolower(_getch());
            if (c == 'y') { rePick = true; std::cout << "yes\n"; break; }
            if (c == 'n') { std::cout << "no\n"; break; }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (rePick) {
        cv::Point2f corners[4];
        if (runArenaCornerPicker(visionOnly, corners)) {
            if (saveArenaCorners(std::string(kArenaCornersFile()), corners))
                std::cout << "Arena corners saved to " << kArenaCornersFile() << "\n";
            else
                std::cerr << "Failed to save arena corners.\n";
            visionOnly.setArenaOverride(makeWarpedArenaBoundary());
        }
    } else {
        cv::Point2f corners[4];
        if (loadArenaCorners(std::string(kArenaCornersFile()), corners)) {
            visionOnly.setArenaOverride(makeWarpedArenaBoundary());
            std::cout << "Loaded existing arena corners from " << kArenaCornersFile() << "\n";
        } else {
            std::cout << "No existing corners found; calibrating without warp.\n";
        }
    }

    runCalibrationDashboard(visionOnly);
}

void runMode8() {
    /*
    Vision debug loop: grabs warped frames, runs full detection pipeline, draws detection
    overlay (robots, obstacles, arena), and streams the annotated frame to image_view.
    ESC exits. Useful for verifying HSV config after Mode 7 calibration.
    */
    activate_vision();

    const int W = kWarpedFrameW();
    const int H = kWarpedFrameH();

    image displayImg{};
    displayImg.type   = RGB_IMAGE;
    displayImg.width  = (i2byte)W;
    displayImg.height = (i2byte)H;
    if (allocate_image(displayImg) != 0) {
        std::cerr << "[mode8] allocate_image failed.\n";
        deactivate_vision();
        return;
    }

    VisionSystem vs;
    if (!vs.initialize(kCameraIndex())) {
        std::cerr << "Camera open failed (mode 8). Check camera index " << kCameraIndex() << ".\n";
        free_image(displayImg);
        deactivate_vision();
        return;
    }

    // Load warp matrix and arena override.
    cv::Mat mode8WarpMatrix;
    {
        cv::Point2f corners[4];
        if (loadArenaCorners(std::string(kArenaCornersFile()), corners)) {
            mode8WarpMatrix = buildWarpToRect(corners);
            vs.setArenaOverride(makeWarpedArenaBoundary());
            std::cout << "[mode8] Warp active from " << kArenaCornersFile() << "\n";
        } else {
            std::cout << "[mode8] No arena corners file; running without warp.\n";
        }
    }

    std::cout << "[Mode 8] Vision debug running. ESC=exit.\n";

    cv::Mat frame;
    int frameCount = 0;
    auto lastPrint = std::chrono::steady_clock::now();

    while (true) {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) break;

        if (!vs.grabFrame(frame) || frame.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(16));
            continue;
        }

        // Apply perspective warp if calibrated.
        if (!mode8WarpMatrix.empty()) {
            cv::Mat warped;
            cv::warpPerspective(frame, warped, mode8WarpMatrix, cv::Size(W, H));
            frame = warped;
        }

        GameState gs = vs.processFrame(frame);

        // Draw overlay onto the frame (no turret in debug mode).
        drawVisionOverlay(frame, gs, 0.0f);

        // Copy to image_view display buffer (flip vertically: OpenCV y=0 is top, image_view y=0 is bottom).
        if (displayImg.pdata && frame.cols == W && frame.rows == H) {
            const size_t rowBytes = (size_t)W * 3;
            for (int y = 0; y < H; ++y) {
                const uint8_t* src = frame.ptr<uint8_t>(y);
                ibyte* dst = displayImg.pdata + (size_t)(H - 1 - y) * rowBytes;
                // OpenCV is BGR, image_view expects BGR — direct copy.
                memcpy(dst, src, rowBytes);
            }
            view_rgb_image(displayImg, 1);
        }

        ++frameCount;
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(now - lastPrint).count() >= 1.0f) {
            std::cout << "  [mode8] self:" << (gs.self.pose.valid ? "OK" : "--")
                      << "  enemy:" << (gs.enemy.pose.valid ? "OK" : "--")
                      << "  obs:" << gs.obstacles.size()
                      << "  fps~" << frameCount << "\n";
            frameCount = 0;
            lastPrint  = now;
        }
    }

    free_image(displayImg);
    deactivate_vision();
    std::cout << "[mode 8] Stopped. Returning to menu.\n";
}

#endif // USE_VISION

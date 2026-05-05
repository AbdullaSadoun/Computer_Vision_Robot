// Vision calibration dashboard.
//
// Five-color marker scheme: every detected circle is one of black, blue,
// green, orange, or red. The dashboard exposes one HSV section per color
// plus an arena tape section, a detection-tuning section, and a live
// readout of the auto-derived pixels-per-inch and 9-inch pair band.
//
// Press ESC to exit, S to dump the current HSV values to stdout for
// pasting back into VisionConfig.h.
//
// This file is only compiled when USE_VISION is defined.

#include <algorithm>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "VisionSystem.h"

namespace {
struct SliderBinding {
    std::string label;
    int* value;
    int minValue;
    int maxValue;
};

struct CalibrationSection {
    std::string heading;
    std::vector<SliderBinding> sliders;
};

struct SliderLayout {
    int sectionIndex;
    int sliderIndex;
    cv::Rect trackRect;
};

struct CalibrationUIState {
    std::vector<CalibrationSection> sections;
    std::vector<SliderLayout> layout;
    int scrollOffset = 0;
    int contentHeight = 0;
    bool dragging = false;
    int activeSection = -1;
    int activeSlider = -1;
};

constexpr int kCalibrationWidth = 560;
constexpr int kCalibrationHeight = 600;
constexpr int kCalibrationMargin = 18;
constexpr int kHeadingHeight = 28;
constexpr int kSliderSpacing = 30;
constexpr int kSectionSpacing = 16;
constexpr int kTrackWidth = 150;
constexpr int kTrackHeight = 6;
constexpr int kValueBoxWidth = 48;
constexpr int kScrollStep = 36;

// Six mask tiles arranged 3x2: black, blue, green / orange, red, arena.
constexpr int kMaskTileWidth = 260;
constexpr int kMaskTileHeight = 200;
constexpr int kMaskColumns = 3;
constexpr int kMaskRows = 2;
constexpr int kMaskOverviewWidth = kMaskTileWidth * kMaskColumns;
constexpr int kMaskOverviewHeight = kMaskTileHeight * kMaskRows;

constexpr int kDashboardGap = 18;
constexpr int kLogPanelHeight = 160;
constexpr int kDashboardTitleHeight = 58;
constexpr int kDashboardOuterMargin = 16;

void updateSliderValue(CalibrationUIState& ui, int sectionIndex, int sliderIndex, int mouseX) {
    if (sectionIndex < 0 || sliderIndex < 0) return;
    for (const auto& item : ui.layout) {
        if (item.sectionIndex != sectionIndex || item.sliderIndex != sliderIndex) continue;
        SliderBinding& slider = ui.sections[sectionIndex].sliders[sliderIndex];
        const int clampedX = std::clamp(mouseX, item.trackRect.x, item.trackRect.x + item.trackRect.width);
        const float ratio = item.trackRect.width > 0
            ? static_cast<float>(clampedX - item.trackRect.x) / static_cast<float>(item.trackRect.width)
            : 0.0f;
        const int span = slider.maxValue - slider.minValue;
        *slider.value = slider.minValue + static_cast<int>(std::round(ratio * span));
        *slider.value = std::clamp(*slider.value, slider.minValue, slider.maxValue);
        break;
    }
}

void onCalibrationMouse(int event, int x, int y, int flags, void* userdata) {
    if (!userdata) return;
    auto* ui = static_cast<CalibrationUIState*>(userdata);

    const int originX = kDashboardOuterMargin + kMaskOverviewWidth + kDashboardGap;
    const int originY = kDashboardTitleHeight + 8;
    const int localX = x - originX;
    const int localY = y - originY;

    if (localX < 0 || localX >= kCalibrationWidth || localY < 0 || localY >= kCalibrationHeight) {
        if (event == cv::EVENT_LBUTTONUP) {
            ui->dragging = false;
            ui->activeSection = -1;
            ui->activeSlider = -1;
        }
        return;
    }

    if (event == cv::EVENT_MOUSEWHEEL) {
        ui->scrollOffset -= cv::getMouseWheelDelta(flags) / 120 * kScrollStep;
        const int maxScroll = std::max(0, ui->contentHeight - kCalibrationHeight);
        ui->scrollOffset = std::clamp(ui->scrollOffset, 0, maxScroll);
        return;
    }

    if (event == cv::EVENT_LBUTTONDOWN) {
        for (const auto& item : ui->layout) {
            cv::Rect hitBox = item.trackRect;
            hitBox.y -= 8;
            hitBox.height += 16;
            if (hitBox.contains(cv::Point(localX, localY))) {
                ui->dragging = true;
                ui->activeSection = item.sectionIndex;
                ui->activeSlider = item.sliderIndex;
                updateSliderValue(*ui, item.sectionIndex, item.sliderIndex, localX);
                return;
            }
        }
    }

    if (event == cv::EVENT_MOUSEMOVE && ui->dragging) {
        updateSliderValue(*ui, ui->activeSection, ui->activeSlider, localX);
        return;
    }

    if (event == cv::EVENT_LBUTTONUP) {
        ui->dragging = false;
        ui->activeSection = -1;
        ui->activeSlider = -1;
    }
}

std::vector<SliderBinding> hsvSliders(HSVRange& r) {
    return {
        {"Hue Min", &r.hMin, 0, 179},
        {"Hue Max", &r.hMax, 0, 179},
        {"Sat Min", &r.sMin, 0, 255},
        {"Sat Max", &r.sMax, 0, 255},
        {"Val Min", &r.vMin, 0, 255},
        {"Val Max", &r.vMax, 0, 255},
    };
}

void buildSections(CalibrationUIState& ui, VisionParameters& c) {
    ui.sections = {
        { "BLACK MARKER",  hsvSliders(c.blackMarker)  },
        { "BLUE MARKER (rear of OUR robot)",   hsvSliders(c.blueMarker)   },
        { "GREEN MARKER",  hsvSliders(c.greenMarker)  },
        { "ORANGE MARKER", hsvSliders(c.orangeMarker) },
        { "RED MARKER (low band, hue wraps)",  hsvSliders(c.redMarkerLow)  },
        { "RED MARKER (high band, hue wraps)", hsvSliders(c.redMarkerHigh) },
        { "ARENA SURFACE (WHITE)", {
            {"Hue Min", &c.arenaSurface.hMin, 0, 179},
            {"Hue Max", &c.arenaSurface.hMax, 0, 179},
            {"Sat Min", &c.arenaSurface.sMin, 0, 255},
            {"Sat Max", &c.arenaSurface.sMax, 0, 255},
            {"Val Min", &c.arenaSurface.vMin, 0, 255},
            {"Val Max", &c.arenaSurface.vMax, 0, 255},
            {"Min Arena Area", &c.minArenaArea, 0, 200000},
        }},
        { "DETECTION THRESHOLDS", {
            {"Min Marker Area", &c.minMarkerArea, 0, 5000},
            {"Blur Kernel",     &c.blurKernelSize, 1, 21},
            {"Morph Kernel",    &c.morphKernelSize, 1, 21},
            {"Pose Hold (ms)",  &c.poseHoldMs, 0, 2000},
        }},
    };
}

void normalizeParameters(VisionParameters& c) {
    if (c.blurKernelSize < 1) c.blurKernelSize = 1;
    if (c.blurKernelSize % 2 == 0) ++c.blurKernelSize;
    if (c.morphKernelSize < 1) c.morphKernelSize = 1;
    if (c.morphKernelSize % 2 == 0) ++c.morphKernelSize;

    auto clampBand = [](HSVRange& r) {
        if (r.hMin > r.hMax) std::swap(r.hMin, r.hMax);
        if (r.sMin > r.sMax) std::swap(r.sMin, r.sMax);
        if (r.vMin > r.vMax) std::swap(r.vMin, r.vMax);
    };
    clampBand(c.blackMarker);
    clampBand(c.blueMarker);
    clampBand(c.greenMarker);
    clampBand(c.orangeMarker);
    clampBand(c.redMarkerLow);
    clampBand(c.redMarkerHigh);
    clampBand(c.arenaSurface);
}

cv::Mat makeMaskTile(const cv::Mat& mask, const std::string& title) {
    cv::Mat resized;
    cv::resize(mask, resized, cv::Size(kMaskTileWidth, kMaskTileHeight), 0.0, 0.0, cv::INTER_NEAREST);
    cv::Mat tile;
    cv::cvtColor(resized, tile, cv::COLOR_GRAY2BGR);
    cv::putText(tile, title, cv::Point(10, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(180, 220, 255), 2);
    return tile;
}

cv::Mat buildMaskOverview(const cv::Mat& black, const cv::Mat& blue, const cv::Mat& green,
                          const cv::Mat& orange, const cv::Mat& red, const cv::Mat& arena) {
    cv::Mat topRow, bottomRow;
    cv::hconcat(std::vector<cv::Mat>{
        makeMaskTile(black, "Black"),
        makeMaskTile(blue,  "Blue"),
        makeMaskTile(green, "Green")
    }, topRow);
    cv::hconcat(std::vector<cv::Mat>{
        makeMaskTile(orange, "Orange"),
        makeMaskTile(red,    "Red"),
        makeMaskTile(arena,  "Arena Surface")
    }, bottomRow);
    cv::Mat overview;
    cv::vconcat(std::vector<cv::Mat>{topRow, bottomRow}, overview);
    return overview;
}

cv::Mat drawCalibrationPanel(CalibrationUIState& ui) {
    cv::Mat canvas(kCalibrationHeight, kCalibrationWidth, CV_8UC3, cv::Scalar(245, 245, 245));
    ui.layout.clear();

    int contentY = kCalibrationMargin;
    for (size_t sectionIndex = 0; sectionIndex < ui.sections.size(); ++sectionIndex) {
        const auto& section = ui.sections[sectionIndex];
        const int headingY = contentY - ui.scrollOffset;
        if (headingY > -kHeadingHeight && headingY < kCalibrationHeight + kHeadingHeight) {
            cv::putText(canvas, section.heading, cv::Point(kCalibrationMargin, headingY + 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(40, 40, 40), 1);
            cv::line(canvas,
                     cv::Point(kCalibrationMargin, headingY + 24),
                     cv::Point(kCalibrationWidth - kCalibrationMargin, headingY + 24),
                     cv::Scalar(180, 180, 180), 1);
        }
        contentY += kHeadingHeight;

        for (size_t sliderIndex = 0; sliderIndex < section.sliders.size(); ++sliderIndex) {
            const auto& slider = section.sliders[sliderIndex];
            const int rowTop = contentY - ui.scrollOffset;
            const int trackX = kCalibrationWidth - kCalibrationMargin - kValueBoxWidth - kTrackWidth;
            const int trackY = rowTop + 8;
            const cv::Rect trackRect(trackX, trackY, kTrackWidth, kTrackHeight);
            ui.layout.push_back({static_cast<int>(sectionIndex), static_cast<int>(sliderIndex), trackRect});

            if (rowTop > -kSliderSpacing && rowTop < kCalibrationHeight + kSliderSpacing) {
                cv::putText(canvas, slider.label, cv::Point(kCalibrationMargin, rowTop + 14),
                            cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(55, 55, 55), 1);
                cv::rectangle(canvas, trackRect, cv::Scalar(190, 190, 190), cv::FILLED);
                const float ratio = slider.maxValue > slider.minValue
                    ? static_cast<float>(*slider.value - slider.minValue) / static_cast<float>(slider.maxValue - slider.minValue)
                    : 0.0f;
                const int knobX = trackRect.x + static_cast<int>(std::round(ratio * trackRect.width));
                cv::line(canvas,
                         cv::Point(knobX, trackRect.y - 6),
                         cv::Point(knobX, trackRect.y + trackRect.height + 6),
                         cv::Scalar(40, 120, 220), 2);
                const cv::Rect valueBox(trackRect.x + trackRect.width + 10, rowTop + 1, kValueBoxWidth, 18);
                cv::rectangle(canvas, valueBox, cv::Scalar(255, 255, 255), cv::FILLED);
                cv::rectangle(canvas, valueBox, cv::Scalar(180, 180, 180), 1);
                cv::putText(canvas, std::to_string(*slider.value),
                            cv::Point(valueBox.x + 5, valueBox.y + 13),
                            cv::FONT_HERSHEY_SIMPLEX, 0.44, cv::Scalar(50, 50, 50), 1);
            }
            contentY += kSliderSpacing;
        }
        contentY += kSectionSpacing;
    }

    ui.contentHeight = contentY + kCalibrationMargin;
    const int maxScroll = std::max(0, ui.contentHeight - kCalibrationHeight);
    ui.scrollOffset = std::clamp(ui.scrollOffset, 0, maxScroll);

    if (maxScroll > 0) {
        const int barHeight = std::max(40, static_cast<int>(
            static_cast<float>(kCalibrationHeight) / static_cast<float>(ui.contentHeight) * kCalibrationHeight));
        const int barY = static_cast<int>(
            static_cast<float>(ui.scrollOffset) / static_cast<float>(maxScroll) * (kCalibrationHeight - barHeight));
        cv::rectangle(canvas, cv::Rect(kCalibrationWidth - 10, 0, 10, kCalibrationHeight),
                      cv::Scalar(235, 235, 235), cv::FILLED);
        cv::rectangle(canvas, cv::Rect(kCalibrationWidth - 9, barY, 8, barHeight),
                      cv::Scalar(160, 160, 160), cv::FILLED);
    }

    cv::putText(canvas, "Wheel to scroll. Drag the lines. S = print HSV. ESC = exit.",
                cv::Point(kCalibrationMargin, kCalibrationHeight - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(90, 90, 90), 1);
    return canvas;
}

cv::Mat buildLogPanel(const std::deque<std::string>& logLines, int width) {
    cv::Mat panel(kLogPanelHeight, width, CV_8UC3, cv::Scalar(22, 24, 28));
    cv::rectangle(panel, cv::Rect(0, 0, panel.cols, panel.rows), cv::Scalar(65, 65, 70), 2);
    cv::rectangle(panel, cv::Rect(0, 0, panel.cols, 32), cv::Scalar(16, 18, 22), cv::FILLED);
    cv::rectangle(panel, cv::Rect(0, 0, 12, 32), cv::Scalar(255, 170, 70), cv::FILLED);
    cv::putText(panel, "Vision Output", cv::Point(22, 22),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(235, 235, 235), 2);
    int y = 52;
    for (const auto& line : logLines) {
        cv::putText(panel, line, cv::Point(16, y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(218, 218, 218), 1);
        y += 20;
        if (y > panel.rows - 12) break;
    }
    return panel;
}

cv::Mat buildDashboard(const cv::Mat& maskOverview, const cv::Mat& calibrationPanel,
                      const std::deque<std::string>& logLines) {
    const int contentWidth = maskOverview.cols + kDashboardGap + calibrationPanel.cols;
    const int totalWidth = contentWidth + 32;
    const int totalHeight = kDashboardTitleHeight + 12 +
                            std::max(maskOverview.rows, calibrationPanel.rows) +
                            kDashboardGap + kLogPanelHeight + 16;

    cv::Mat dashboard(totalHeight, totalWidth, CV_8UC3, cv::Scalar(228, 230, 234));
    cv::rectangle(dashboard, cv::Rect(0, 0, dashboard.cols, kDashboardTitleHeight),
                  cv::Scalar(27, 31, 38), cv::FILLED);
    cv::putText(dashboard, "o2D_Sim Vision Calibration", cv::Point(18, 36),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(245, 245, 245), 2);
    cv::putText(dashboard, "Tune HSV ranges until each mask isolates only its color",
                cv::Point(20, 53), cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(190, 198, 210), 1);

    const cv::Rect maskRect(16, kDashboardTitleHeight + 8, maskOverview.cols, maskOverview.rows);
    const cv::Rect calibrationRect(maskRect.x + maskRect.width + kDashboardGap,
                                   kDashboardTitleHeight + 8,
                                   calibrationPanel.cols, calibrationPanel.rows);
    const cv::Rect logRect(16, kDashboardTitleHeight + 8 +
                           std::max(maskOverview.rows, calibrationPanel.rows) + kDashboardGap,
                           contentWidth, kLogPanelHeight);

    maskOverview.copyTo(dashboard(maskRect));
    calibrationPanel.copyTo(dashboard(calibrationRect));
    buildLogPanel(logLines, logRect.width).copyTo(dashboard(logRect));
    return dashboard;
}

void appendLog(std::deque<std::string>& logLines, const std::string& message) {
    if (message.empty()) return;
    if (logLines.empty() || logLines.front() != message) {
        logLines.push_front(message);
    }
    while (logLines.size() > 5) logLines.pop_back();
}

std::string formatHSVRange(const char* name, const HSVRange& r) {
    std::ostringstream os;
    os << name << "{" << r.hMin << "," << r.sMin << "," << r.vMin
       << "," << r.hMax << "," << r.sMax << "," << r.vMax << "}";
    return os.str();
}

void printHSVDump(const VisionParameters& c) {
    std::cout << "\n--- Paste into VisionConfig.h defaults ---\n";
    std::cout << "  HSVRange " << formatHSVRange("blackMarker",   c.blackMarker)   << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("blueMarker",    c.blueMarker)    << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("greenMarker",   c.greenMarker)   << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("orangeMarker",  c.orangeMarker)  << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("redMarkerLow",  c.redMarkerLow)  << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("redMarkerHigh", c.redMarkerHigh) << ";\n";
    std::cout << "  HSVRange " << formatHSVRange("arenaSurface",   c.arenaSurface)   << ";\n";
    std::cout << "  minMarkerArea="  << c.minMarkerArea  << ";\n";
    std::cout << "  minArenaArea="   << c.minArenaArea   << ";\n";
    std::cout << "  blurKernelSize=" << c.blurKernelSize << ";\n";
    std::cout << "  morphKernelSize="<< c.morphKernelSize<< ";\n";
    std::cout << "  poseHoldMs="     << c.poseHoldMs     << ";\n";
    std::cout << "------------------------------------------\n\n";
}

cv::Scalar markerBgr(MarkerColor c) {
    switch (c) {
        case MarkerColor::Black:  return cv::Scalar(40, 40, 40);
        case MarkerColor::Blue:   return cv::Scalar(255, 0, 0);
        case MarkerColor::Green:  return cv::Scalar(0, 255, 0);
        case MarkerColor::Orange: return cv::Scalar(0, 140, 255);
        case MarkerColor::Red:    return cv::Scalar(0, 0, 255);
        default:                  return cv::Scalar(200, 200, 200);
    }
}
} // namespace

void runCalibrationDashboard(VisionSystem& vision) {
    VisionParameters& config = vision.config();
    CalibrationUIState ui;
    buildSections(ui, config);

    const std::string cameraWindow = "Vision Camera";
    const std::string dashboardWindow = "Vision Calibration Dashboard";
    cv::namedWindow(cameraWindow);
    cv::namedWindow(dashboardWindow);
    cv::setMouseCallback(dashboardWindow, onCalibrationMouse, &ui);

    std::deque<std::string> logLines;
    appendLog(logLines, "Calibration started. ESC to quit, S to dump HSV values.");

    cv::Mat frame;
    while (true) {
        if (!vision.grabFrame(frame) || frame.empty()) {
            std::cerr << "Camera read failed; exiting calibration.\n";
            break;
        }
        normalizeParameters(config);

        GameState state = vision.processFrame(frame);

        cv::Mat black, blue, green, orange, red, arenaMask;
        vision.buildDebugMasks(frame, black, blue, green, orange, red, arenaMask);

        cv::Mat annotated = frame.clone();
        auto drawRobot = [&](const RobotTarget& r, const cv::Scalar& color, const char* label) {
            if (!r.pose.valid) return;
            cv::Point c(static_cast<int>(std::round(r.pose.x)), static_cast<int>(std::round(r.pose.y)));
            cv::Point h(static_cast<int>(std::round(r.pose.x + 35.0f * std::cos(r.pose.theta))),
                        static_cast<int>(std::round(r.pose.y + 35.0f * std::sin(r.pose.theta))));
            cv::circle(annotated, c, 5, color, -1);
            cv::arrowedLine(annotated, c, h, color, 2);
            cv::putText(annotated, label, c + cv::Point(10, -10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        };
        drawRobot(state.self,  cv::Scalar(255, 255, 255), "Self");
        drawRobot(state.enemy, cv::Scalar(0,   255, 255), "Enemy");

        for (const auto& o : state.obstacles) {
            if (!o.valid) continue;
            cv::circle(annotated,
                       cv::Point(static_cast<int>(std::round(o.x)), static_cast<int>(std::round(o.y))),
                       static_cast<int>(std::round(o.radius)),
                       markerBgr(o.color), 2);
        }
        if (state.arena.valid) {
            for (size_t i = 0; i < state.arena.corners.size(); ++i) {
                const auto& a = state.arena.corners[i];
                const auto& b = state.arena.corners[(i + 1) % state.arena.corners.size()];
                cv::line(annotated,
                         cv::Point(static_cast<int>(std::round(a.x)), static_cast<int>(std::round(a.y))),
                         cv::Point(static_cast<int>(std::round(b.x)), static_cast<int>(std::round(b.y))),
                         cv::Scalar(255, 200, 0), 3);
            }
        }

        cv::putText(annotated,
                    std::string("Self (front=") + markerColorName(vision.getSelfFrontColor()) + "): " +
                    (state.self.pose.valid ? "OK" : "--"),
                    cv::Point(15, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 2);
        cv::putText(annotated,
                    std::string("Enemy: ") + (state.enemy.pose.valid ? "OK" : "--"),
                    cv::Point(15, 50), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        cv::putText(annotated,
                    std::string("Obstacles: ") + std::to_string(state.obstacles.size()),
                    cv::Point(15, 75), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 2);
        char arenaBuf[80];
        if (state.arena.valid) {
            float minX = state.arena.corners[0].x, maxX = minX;
            float minY = state.arena.corners[0].y, maxY = minY;
            for (const auto& cn : state.arena.corners) {
                minX = std::min(minX, cn.x); maxX = std::max(maxX, cn.x);
                minY = std::min(minY, cn.y); maxY = std::max(maxY, cn.y);
            }
            std::snprintf(arenaBuf, sizeof(arenaBuf), "Arena: OK   (~%.0f x %.0f px)",
                          maxX - minX, maxY - minY);
        } else {
            std::snprintf(arenaBuf, sizeof(arenaBuf), "Arena: --");
        }
        cv::putText(annotated, arenaBuf,
                    cv::Point(15, 100), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 200, 0), 2);

        // Live px/in + 9-in band readout. This is the crucial number for the
        // pair-distance gate; if it drifts the operator sees it immediately.
        char scaleBuf[128];
        if (state.pxPerInchValid) {
            const float band = state.pxPerInch * config.robotMarkerSpacingInches;
            std::snprintf(scaleBuf, sizeof(scaleBuf),
                          "px/in: %.2f   9-in target: %.0f px (+/- %.0f%%)",
                          state.pxPerInch, band, config.pairTolerancePct * 100.0f);
        } else {
            std::snprintf(scaleBuf, sizeof(scaleBuf),
                          "px/in: -- (need at least one detected marker)");
        }
        cv::putText(annotated, scaleBuf, cv::Point(15, 125),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(220, 220, 220), 1);

        cv::imshow(cameraWindow, annotated);
        cv::imshow(dashboardWindow,
                   buildDashboard(
                       buildMaskOverview(black, blue, green, orange, red, arenaMask),
                       drawCalibrationPanel(ui),
                       logLines));

        const int key = cv::waitKey(1);
        if (key == 27) break;
        if (key == 's' || key == 'S') {
            printHSVDump(config);
            appendLog(logLines, "HSV values printed to console.");
        }
    }
    cv::destroyWindow(cameraWindow);
    cv::destroyWindow(dashboardWindow);
}

// ── Arena corner persistence and warp helpers ─────────────────────────────────
// Moved here from program.cpp (were static; now extern so calibration.cpp and
// program.cpp can both call them without a separate header).

#include "../Parameters.h"
#include <cstdio>
#include <thread>

static inline int kWarpedFrameW() { return params().warped_w; }
static inline int kWarpedFrameH() { return params().warped_h; }
static inline const char* kArenaCornersFile() { return params().arena_corners_file; }

bool loadArenaCorners(const std::string& path, cv::Point2f out[4]) {
    std::FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    int read = 0;
    for (int i = 0; i < 4; ++i) {
        float x = 0, y = 0;
        if (std::fscanf(f, "%f %f", &x, &y) == 2) {
            out[i] = cv::Point2f(x, y);
            ++read;
        }
    }
    std::fclose(f);
    return read == 4;
}

bool saveArenaCorners(const std::string& path, const cv::Point2f in[4]) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    for (int i = 0; i < 4; ++i)
        std::fprintf(f, "%.3f %.3f\n", in[i].x, in[i].y);
    std::fclose(f);
    return true;
}

namespace {
struct CornerPickerState {
    std::vector<cv::Point2f> picks;
};

void onCornerPickerMouse(int event, int x, int y, int /*flags*/, void* userdata) {
    auto* s = static_cast<CornerPickerState*>(userdata);
    if (!s) return;
    if (event == cv::EVENT_LBUTTONDOWN && s->picks.size() < 4)
        s->picks.emplace_back((float)x, (float)y);
    else if (event == cv::EVENT_RBUTTONDOWN && !s->picks.empty())
        s->picks.pop_back();
}
} // namespace

bool runArenaCornerPicker(VisionSystem& vision, cv::Point2f outCorners[4]) {
    const std::string win = "Arena Corner Picker";
    cv::namedWindow(win);
    CornerPickerState state{};
    cv::setMouseCallback(win, onCornerPickerMouse, &state);

    const char* labels[4] = {"TL", "TR", "BR", "BL"};
    const cv::Scalar pickColor[4] = {
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 255, 255),
        cv::Scalar(0, 0, 255),
        cv::Scalar(255, 0, 255),
    };

    std::cout << "\nArena Corner Picker:\n"
              << "  Click 4 corners in order: TL, TR, BR, BL.\n"
              << "  Right-click or BACKSPACE to undo, ENTER/SPACE to confirm, ESC to cancel.\n";

    cv::Mat raw;
    bool confirmed = false;
    bool cancelled = false;
    while (!confirmed && !cancelled) {
        if (!vision.grabFrame(raw) || raw.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(33));
            continue;
        }
        cv::Mat disp = raw.clone();
        for (size_t i = 0; i < state.picks.size(); ++i) {
            const cv::Point2f& p = state.picks[i];
            cv::circle(disp, p, 8, pickColor[i], 2, cv::LINE_AA);
            cv::circle(disp, p, 2, pickColor[i], -1);
            cv::putText(disp, labels[i], cv::Point((int)p.x + 12, (int)p.y - 8),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, pickColor[i], 2, cv::LINE_AA);
        }
        if (state.picks.size() >= 2) {
            for (size_t i = 0; i + 1 < state.picks.size(); ++i)
                cv::line(disp, state.picks[i], state.picks[i + 1],
                         cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            if (state.picks.size() == 4)
                cv::line(disp, state.picks[3], state.picks[0],
                         cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
        char prompt[128];
        if (state.picks.size() < 4)
            std::snprintf(prompt, sizeof(prompt),
                          "Click corner %zu/4 (%s).  R-click/BKSP=undo  ESC=cancel",
                          state.picks.size() + 1, labels[state.picks.size()]);
        else
            std::snprintf(prompt, sizeof(prompt),
                          "All 4 picked.  ENTER/SPACE = confirm  BKSP = redo last  ESC = cancel");
        cv::rectangle(disp, cv::Rect(0, 0, disp.cols, 24), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(disp, prompt, cv::Point(8, 17),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
        cv::imshow(win, disp);
        int key = cv::waitKey(15);
        if (key == 27) cancelled = true;
        else if (key == 8) { if (!state.picks.empty()) state.picks.pop_back(); }
        else if ((key == 13 || key == 32) && state.picks.size() == 4) confirmed = true;
    }
    cv::destroyWindow(win);
    if (cancelled || state.picks.size() != 4) return false;
    for (int i = 0; i < 4; ++i) outCorners[i] = state.picks[i];
    return true;
}

bool ensureArenaCorners(VisionSystem& vision, cv::Point2f outCorners[4], bool forcePicker) {
    if (!forcePicker && loadArenaCorners(std::string(kArenaCornersFile()), outCorners)) {
        std::cout << "Loaded arena corners from " << kArenaCornersFile() << "\n";
        return true;
    }
    if (!runArenaCornerPicker(vision, outCorners)) {
        std::cerr << "Arena corner picker cancelled / failed.\n";
        return false;
    }
    if (saveArenaCorners(std::string(kArenaCornersFile()), outCorners))
        std::cout << "Saved arena corners to " << kArenaCornersFile() << "\n";
    else
        std::cerr << "Could not write " << kArenaCornersFile() << "\n";
    return true;
}

cv::Mat buildWarpToRect(const cv::Point2f src[4]) {
    const cv::Point2f dst[4] = {
        cv::Point2f(0.0f,                           0.0f),
        cv::Point2f((float)(kWarpedFrameW() - 1),   0.0f),
        cv::Point2f((float)(kWarpedFrameW() - 1),   (float)(kWarpedFrameH() - 1)),
        cv::Point2f(0.0f,                           (float)(kWarpedFrameH() - 1)),
    };
    return cv::getPerspectiveTransform(src, dst);
}

ArenaBoundary makeWarpedArenaBoundary() {
    ArenaBoundary a{};
    a.corners[0] = Point2D{0.0f,                           0.0f};
    a.corners[1] = Point2D{(float)(kWarpedFrameW() - 1),   0.0f};
    a.corners[2] = Point2D{(float)(kWarpedFrameW() - 1),   (float)(kWarpedFrameH() - 1)};
    a.corners[3] = Point2D{0.0f,                           (float)(kWarpedFrameH() - 1)};
    a.valid = true;
    return a;
}

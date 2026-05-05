#pragma once

struct HSVRange {
    int hMin = 0, sMin = 0, vMin = 0;
    int hMax = 179, sMax = 255, vMax = 255;
};

// Five-color marker palette: black, blue, green, orange, red. Every detected
// circle on the field belongs to exactly one of these colors. A "robot" is a
// pair of circles spaced robotMarkerSpacingInches center-to-center; everything
// else is treated as an obstacle.
//
// Our robot's REAR marker is always BLUE. The FRONT color is selected at
// runtime via VisionSystem::setSelfFrontColor based on the operator prompt.
struct VisionParameters {
    // Black marker - low V is the discriminator. Same band as the arena tape
    // works for most lighting; tune in the calibration dashboard if obstacles
    // bleed into the arena polygon.
    HSVRange blackMarker  {0,   0,   26,   35, 110, 92};
    // after calibration, black = hmin0 hmax35 satmin0 satmax110 valmin26 valmax92

    // after calibration, blue = hmin84 hmax177 satmin0 satmax220 valmin60 valmax255
    // after calibration, green = hmin50 hmax81 satmin80 satmax255 valmin60 valmax255
    HSVRange blueMarker   {84, 0, 60,  177, 220, 255};
    HSVRange greenMarker  {50,  80,  60,  90,  255, 255};
    HSVRange orangeMarker {6,   58, 162, 23,  145, 255};

    // Red wraps around hue 0/179, so it needs two bands OR'd together.
    HSVRange redMarkerLow  {0,   140, 80,  10,  255, 255};
    HSVRange redMarkerHigh {170, 140, 80,  179, 255, 255};

    // Arena playing surface: bright (white-ish) area surrounded by darker
    // floor / wood. Detection finds the largest convex 4-sided contour that
    // matches this band. Tune in the calibration dashboard if the arena is
    // a different color or the lighting changes.
    HSVRange arenaSurface {0, 0, 180, 179, 70, 255};

    // Physical geometry of the markers (used for auto px/in calibration and
    // the robot-pair distance gate).
    float markerDiameterInches     = 3.0f;
    float robotMarkerSpacingInches = 9.0f;
    float pairTolerancePct         = 0.20f;  // +/- 20% on the 9-in rule

    // Sanity clamp on the auto-derived px/in. Frames where every detected
    // circle is bogus (e.g. specular highlights) shouldn't push the scale
    // outside this band.
    float minPxPerInchClamp = 4.0f;
    float maxPxPerInchClamp = 30.0f;

    int minMarkerArea  = 120;
    int minArenaArea   = 20000;
    int blurKernelSize = 5;
    int morphKernelSize = 5;

    // If a robot is not seen this frame, hold its last valid pose for this
    // many milliseconds before reporting it as lost. Smooths single-frame
    // dropouts so the AI doesn't flip into fallback behavior on every blink.
    int poseHoldMs = 500;
};

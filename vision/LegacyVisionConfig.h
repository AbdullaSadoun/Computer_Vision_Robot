#pragma once

// Per-color tuning knobs for the channel-difference extraction pipeline.
// All fields are exposed in Mode 9 via Q/A and W/S keyboard shortcuts.
struct LegacyColorConfig {
    int channelThreshold = 60;  // binary threshold applied after channel extraction (0-255)
    int minAreaPx        = 100; // reject blobs smaller than this (pixels)
    int morphPasses      = 1;   // erode + dialate*2 iterations per pass
};

struct LegacyVisionParams {
    LegacyColorConfig black  = {  50,  80, 1 };
    LegacyColorConfig blue   = {  20,  80, 1 };  // lowered: teal-blue markers have small B-G gap
    LegacyColorConfig green  = {  45,  80, 1 };
    LegacyColorConfig orange = {  55,  80, 1 };
    LegacyColorConfig red    = {  55,  80, 1 };

    // Marker pair distance gate in absolute pixels (no perspective warp -> variable scale).
    // At a typical overhead camera height, 9 inch marker spacing ~ 80-160 px.
    int   pairMinPx        = 30;
    int   pairMaxPx        = 250;

    float markerDiameterIn = 3.0f;  // physical marker diameter (inches) for px/in estimate
    int   poseHoldMs       = 500;   // hold last valid pose this many ms after dropout
    int   camWidth         = 640;
    int   camHeight        = 480;
};

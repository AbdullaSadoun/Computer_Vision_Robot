/*
Parameters.h
- Central runtime configuration struct for the entire o2D simulation and robot control stack
- Groups all tunable knobs: camera index, COM port, arena dimensions, safety margins,
  AI lookahead distances, hardware inversion flags, and Mode 5/9/A/B planner settings
- Exposes a single global accessor params() so every subsystem reads the same instance
- Units: px = warped-arena pixels (640x480 default), in = physical arena inches (72x54)
by: Abdulla Sadoun
Date: February 10, 2026
*/
// Parameters.h
//
// Central place for parameters that are frequently calibrated/tuned.
// Keep this file small, explicit, and *high-signal*.
//
// Design goals:
// - Anything you tweak often should live here (camera index, COM port, safety margins, etc.)
// - Vision HSV ranges remain in vision/VisionConfig.h (because the calibration UI edits those),
//   but higher-level policy knobs (warp size, safety horizons, AI lookaheads) live here.
//
// Notes on units:
// - "px" refers to coordinates in the *simulation / warped arena* frame (640x480 by default).
// - "in" refers to physical inches of the arena (72x54 by spec).
// - Speeds in the AI use px/s to match the simulator kinematics.
// TODO params tl 38, 85. tr  x=496  y=58 - x=584  y=351  x=21  y=402
#pragma once
 
#include <cstdint>
 
struct Parameters {
    // ----------------------------
    // Camera / robot connectivity
    // ----------------------------
    // Which webcam index OpenCV opens for overhead vision.
    // Impact: wrong index -> no vision in robot modes.
    int camera_index = 1;
 
    // HC-05 (or similar) serial port and baud for robot control.
    // Impact: wrong COM port -> robot modes fall back to simulator.
    const char* robot_com_port = "COM8";
    int robot_baud = 9600;
 
    // ----------------------------
    // Arena + perspective warp
    // ----------------------------
    // Warped output frame size. The whole pipeline (vision overlays, AI, safety)
    // runs in this coordinate system when warp is active.
    // Impact: changing these changes px/in scaling and all margins/lookaheads.
    int warped_w = 640;
    int warped_h = 480;
 
    // Physical arena size used for px/in readouts and sanity checks.
    // Impact: used for display and future physical calibration; keep accurate.
    float arena_w_in = 72.0f;
    float arena_h_in = 54.0f;
 
    // Corner file used by the 4-click picker.
    // Impact: delete this file to force re-pick at scenario start.
    const char* arena_corners_file = "arena_corners.txt";
 
    // ----------------------------
    // Hitboxes / safety margins
    // ----------------------------
    // Approximate robot radius in pixels for wall/obstacle clearance.
    // Impact: larger -> more conservative (stays further from walls/obstacles).
    float robot_radius_px = 25.0f;
 
    // Base margin added on top of robot_radius when we have a valid arena polygon
    // (warped-rect is always valid once calibrated).
    // Impact: smaller -> robot uses more floor but risks clipping due to latency.
    float safety_margin_poly_px = 10.0f; // =========== was 15.0f
 
    // Base margin used when we DON'T have a polygon (fallback to image rect).
    // Impact: should be generous; too small can cause instant DQ if arena not calibrated.
    float safety_margin_fallback_px = 25.0f; // =========== was 30.0f
 
    // Real-world latency compensation: used only when driving the real robot.
    // Impact: higher -> earlier slowdowns/stops; too high -> overly timid robot.
    float cmd_roundtrip_s = 0.35f; // camera+compute+BT+Arduino+mechanical
    float brake_margin_s  = 0.10f; // extra coast after stop command
 
    // Horizons used by the graded clamp (seconds into the future).
    // Impact: longer horizons -> earlier slowing; if too long, can look indecisive.
    float safety_horizons_s[4] = {0.25f, 0.45f, 0.65f, 0.85f};
 
    // Speed multipliers corresponding to the horizons above.
    // Impact: these shape the slowdown curve near walls.
    float safety_scales[4] = {0.00f, 0.33f, 0.66f, 0.85f};
 
    // Edge escape behavior (robot modes): if we are clamped for several frames,
    // take a small verified step back and begin turning away.
    int   edge_recover_hold_frames = 5;
    float edge_recover_back_v_px_s = -34.0f;
    float edge_recover_turn_w_rad_s = 1.35f;
    float edge_recover_check_s = 0.22f;
 
    // ----------------------------
    // AI / avoidance knobs
    // ----------------------------
    // How far ahead (in px) the steering sampler checks for a clear segment.
    // Impact: larger -> anticipates walls sooner; too large -> more detours/spins.
    float steer_lookahead_px = 100.0f;
 
    // AI clearance margin (added to robot radius) when we have a real arena polygon.
    // Impact: smaller -> tighter play; larger -> avoids edges more aggressively.
    float ai_margin_poly_px = 2.0f; // obstacles margins =========== was 27.0f
    float ai_margin_other_px = 2.0f; // "" margins =========== was 16.0f
 
    // ----------------------------
    // Rendering / debug
    // ----------------------------
    // LOS sampling step (0..1). Smaller -> more accurate but slightly more CPU.
    // Impact: too large can cause missed blocks / missed clear shots.
    float los_step = 0.025f;

    // ----------------------------
    // Robot hardware conventions
    // ----------------------------
    // If the turret servo rotates opposite to the math used in the overlay,
    // invert the commanded servo degrees (deg := 180 - deg).
    bool turret_servo_inverted = true;

    // If the camera feed is mirrored/flipped by the driver, flip the image
    // *before* warping and vision so detection/overlay/control are consistent.
    bool vision_flip_x = false; // left-right mirror 
    bool vision_flip_y = false; // up-down mirror // true works with evething else false

    // AI-only drivetrain convention fixes (does NOT affect user-control WASD packets).
    // Use when logs show cmd_w sign mismatches observed dtheta/dt.
    bool ai_invert_v = false;   // v := -v before vw_to_lr (AI only) // true works but drives backwards
    bool ai_invert_w = true;   // w := -w before vw_to_lr
    bool ai_swap_lr = false;    // swap computed L/R after vw_to_lr

    // Mode 9/A/B (legacy vision) command corrections — frame is x-mirrored vs OpenCV modes 5/6.
    // Tune these without touching ai_invert_v/ai_invert_w which only affect modes 5/6.
    bool lv_invert_v      = true;   // negate v (robot moved opposite direction in legacy frame)
    bool lv_invert_w      = false;  // negate w (set true if rotation direction is wrong)
    bool lv_invert_turret = true;   // negate turret angle (laser fired mirror direction)

    // When true, skip the graded OOB safety clamp for robot USER_CONTROL and for
    // manual post-shot drive in AI_ATTACK (WASD L/R). Set false to re-enable clamp.
    bool robot_user_disable_wall_clamp = true;

    // ----------------------------
    // Mode 5 map-based attack planner (robot + vision + AI_ATTACK)
    // ----------------------------
    float planner_safety_clearance_in = 1.75f;
    float planner_grid_cell_in = 0.4f;
    int planner_reconcile_every_n_frames = 15;
    int planner_max_association_changes = 5;
    float planner_replan_enemy_move_px = 45.0f;
    float planner_shoot_standoff_in = 15.0f;
    float planner_follow_speed_px_s = 95.0f;
    float planner_waypoint_arrival_px = 22.0f;
    float planner_match_gate_px = 55.0f;
    float map_chassis_slack_px = 6.0f;
    // Consecutive frames turret must be on-target before Mode5 consumes the shot
    // (prevents instant shotUsed -> return-home when LOS-shortcut enters Aim on frame 1).
    int planner_fire_settle_frames = 10;
};
 
// Global accessor (keeps call sites short without introducing a full config system).
const Parameters& params();

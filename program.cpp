/*
program.cpp
- Main entry point and mode-dispatch loop for the o2D simulation and robot control stack
- Outer menu loop: presents modes 1-9/A/B, reads key input, then delegates to the selected handler
- Mode 1/2/3 (simulator): BMP compositor + sprite physics + AI for offline testing
- Mode 4/5/6 (robot + OpenCV vision): serial command loop with perspective warp and AI (USE_VISION)
- Mode 7/8 (calibration / vision debug): moved to calibration.cpp
- Mode 9/A/B (professor "legacy" vision): moved to vision/vision_main.cpp
- 
by: Abdulla Sadoun
Date: February 12, 2026
*/

#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include "Parameters.h"
#include "CoreTypes.h"
#include "Simulation/Safety.h"
#include "Simulation/Simulation.h"
#include "robot_command/RobotIO.h"
#include "Strategy/Strategy.h"
#include "Simulation/DebugOverlay.h"

// Warped arena size and physical extent from Parameters (needed even when the
// OpenCV vision block below is ifdef'd out, e.g. for image_view buffer sizing).
static inline int kWarpedFrameW() { return params().warped_w; }
static inline int kWarpedFrameH() { return params().warped_h; }
static inline float kArenaWidthInches() { return params().arena_w_in; }
static inline float kArenaHeightInches() { return params().arena_h_in; }
static inline const char* kArenaCornersFile() { return params().arena_corners_file; }
static inline int kCameraIndex() { return params().camera_index; }

#ifdef _WIN32
  #include <conio.h>
  #include <windows.h>
  #include "robot_command/serial.h" // For robot hardware communication
#endif

// Post-shot robot: WASD / manual BT only after turret nears neutral (see return-home AI).
static constexpr float kPostShotTurretHomeTolRad = 0.02f;

#ifdef USE_VISION
  #include <opencv2/opencv.hpp>
  #include "Strategy/map/Mode5AttackPlanner.h"
  #include "vision/VisionSystem.h"
  // Functions defined in vision/VisionCalibration.cpp (forward-declared to avoid a new header).
  void runCalibrationDashboard(VisionSystem& vision);
  bool loadArenaCorners(const std::string& path, cv::Point2f out[4]);
  bool saveArenaCorners(const std::string& path, const cv::Point2f in[4]);
  bool runArenaCornerPicker(VisionSystem& vision, cv::Point2f outCorners[4]);
  bool ensureArenaCorners(VisionSystem& vision, cv::Point2f outCorners[4], bool forcePicker = false);
  cv::Mat buildWarpToRect(const cv::Point2f src[4]);
  ArenaBoundary makeWarpedArenaBoundary();
  // Functions defined in calibration.cpp.
  void runMode7();
  void runMode8();
#endif

#ifdef USE_LEGACY_VISION
  #include "vision/VisionSystemLegacy.h"
  #include "vision/vision_helpers.h"
  #include "vision/LegacyVisionConfig.h"
  // Functions defined in vision/vision_main.cpp.
  void runMode9();
  void runModeAB(char choice);
#endif


int main() {
  // Static visual assets are loaded once and reused across every scenario run.
  Image24 background, robotA, robotB;
  // Asset lookup: prefer assets/ but keep legacy cwd fallback.
  auto tryLoad = [&](const char* name, Image24& out) -> bool {
    std::string p1 = std::string("assets\\") + name;
    if (loadBMP24(p1, out)) return true;
    std::string p2 = std::string("assets/") + name;
    if (loadBMP24(p2, out)) return true;
    return loadBMP24(std::string(name), out);
  };

  if (!tryLoad("background.bmp", background)) {
    std::cerr << "Failed to load background.bmp (tried assets/ and cwd)\n";
    return 1;
  }
  if (!tryLoad("robot_A.bmp", robotA)) {
    std::cerr << "Failed to load robot_A.bmp (tried assets/ and cwd)\n";
    return 1;
  }
  if (!tryLoad("robot_B.bmp", robotB)) {
    std::cerr << "Failed to load robot_B.bmp (tried assets/ and cwd)\n";
    return 1;
  }

  // Optional obstacle sprites (sim mode only).
  Image24 obsBlack, obsGreen, obsBlue;
  bool hasBlack = tryLoad("obstacle_black.bmp", obsBlack);
  bool hasGreen = tryLoad("obstacle_green.bmp", obsGreen);
  bool hasBlue  = tryLoad("obstacle_blue.bmp", obsBlue);

  // Outer menu loop: each iteration runs exactly one scenario then returns
  // here so the operator can pick the next one without restarting the program.
  while (true) {
    std::cout << "\nSelect a scenario:\n";
    std::cout << "1. Simulator User Control\n";
    std::cout << "2. Simulator AI Attack\n";
    std::cout << "3. Simulator AI Defend\n";
    std::cout << "4. Robot User Control\n";
    std::cout << "5. Robot AI Attack\n";
    std::cout << "6. Robot AI Defend\n";
#ifdef USE_VISION
    std::cout << "7. Vision Calibration (HSV tuning, no robot / sim)\n";
    std::cout << "8. Vision Debug (camera + detection overlay, no robot / sim)\n";
#endif
#ifdef USE_LEGACY_VISION
    std::cout << "9. Legacy Vision Debug (camera + detection overlay, no robot)\n";
    std::cout << "A. Robot AI Attack with Legacy Vision\n";
    std::cout << "B. Robot AI Defend with Legacy Vision\n";
#endif
    std::cout << "0. Quit\n";

    Mode mode = Mode::USER_CONTROL;
    Target target = Target::SIMULATOR;
    char choice = '1';

#ifdef _WIN32
    // Drain any key state left over from the previous mode (e.g. ESC still held).
    // GetAsyncKeyState works regardless of which window has focus, so the menu
    // is reachable even when image_view.exe is in the foreground.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    { const int drain_keys[] = {VK_ESCAPE,'0','1','2','3','4','5','6','7','8','9','A','B'};
      for (int ki : drain_keys)
          while (GetAsyncKeyState(ki) & 0x8000)
              std::this_thread::sleep_for(std::chrono::milliseconds(5)); }

    while (true) {
      if (GetAsyncKeyState('0') & 0x8000) { choice = '0'; break; }
      if (GetAsyncKeyState('1') & 0x8000) { choice = '1'; break; }
      if (GetAsyncKeyState('2') & 0x8000) { choice = '2'; break; }
      if (GetAsyncKeyState('3') & 0x8000) { choice = '3'; break; }
      if (GetAsyncKeyState('4') & 0x8000) { choice = '4'; break; }
      if (GetAsyncKeyState('5') & 0x8000) { choice = '5'; break; }
      if (GetAsyncKeyState('6') & 0x8000) { choice = '6'; break; }
#ifdef USE_VISION
      if (GetAsyncKeyState('7') & 0x8000) { choice = '7'; break; }
      if (GetAsyncKeyState('8') & 0x8000) { choice = '8'; break; }
#endif
#ifdef USE_LEGACY_VISION
      if (GetAsyncKeyState('9') & 0x8000) { choice = '9'; break; }
      if (GetAsyncKeyState('A') & 0x8000) { choice = 'A'; break; }
      if (GetAsyncKeyState('B') & 0x8000) { choice = 'B'; break; }
#endif
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    // Wait for the chosen key to be released before the mode handler starts,
    // so it doesn't immediately re-trigger its own exit condition.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
#endif

    if (choice == '0') break;

#ifdef USE_VISION
    if (choice == '7') { runMode7(); continue; }
    if (choice == '8') { runMode8(); continue; }
#endif

#ifdef USE_LEGACY_VISION
    if (choice == '9') { runMode9(); continue; }
    if (choice == 'A' || choice == 'B') { runModeAB(choice); continue; }
#endif

  switch (choice) {
    case '1': mode = Mode::USER_CONTROL; target = Target::SIMULATOR; break;
    case '2': mode = Mode::AI_ATTACK;    target = Target::SIMULATOR; break;
      case '3': mode = Mode::AI_DEFEND;    target = Target::SIMULATOR; break;
    case '4': mode = Mode::USER_CONTROL; target = Target::ROBOT;     break;
    case '5': mode = Mode::AI_ATTACK;    target = Target::ROBOT;     break;
      case '6': mode = Mode::AI_DEFEND;    target = Target::ROBOT;     break;
  }

  // --- Robot Hardware Setup ---
  Serial* robot_port = nullptr;
  if (target == Target::ROBOT) {
      try {
            // IMPORTANT: If you change COM ports often, update Parameters.robot_com_port.
            robot_port = new Serial(params().robot_com_port, params().robot_baud);
          if (robot_port->is_open()) {
                std::cout << "Successfully connected to robot on " << params().robot_com_port << ".\n";
          } else {
                std::cerr << "Robot port " << params().robot_com_port << " is not open. Running simulator instead.\n";
              target = Target::SIMULATOR;
          }
      } catch (const std::exception& e) {
          std::cerr << "ERROR: Failed to connect to robot: " << e.what() << ". Running simulator instead.\n";
          target = Target::SIMULATOR;
      }
  }

    // --- Front marker color prompt (robot modes only; sim uses BMP sprites) ---
#ifdef USE_VISION
    MarkerColor self_front_color = MarkerColor::Red;  // sensible default
    if (target == Target::ROBOT) {
      std::cout << "\nOur robot's REAR marker is BLUE.\n";
      std::cout << "Which color is the FRONT marker today? [b]lack / [g]reen / [o]range / [r]ed: ";
      std::cout.flush();
      while (true) {
        if (_kbhit()) {
          char c = (char)std::tolower(_getch());
          switch (c) {
            case 'b': self_front_color = MarkerColor::Black;  break;
            case 'g': self_front_color = MarkerColor::Green;  break;
            case 'o': self_front_color = MarkerColor::Orange; break;
            case 'r': self_front_color = MarkerColor::Red;    break;
            default:  continue;
          }
          std::cout << markerColorName(self_front_color) << "\n";
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
#endif

    // --- Overhead Vision Setup (only when driving the real robot) ---
#ifdef USE_VISION
    VisionSystem vision;
    bool vision_active = false;
    bool arena_calibrated = false;
    cv::Mat warp_matrix;
    cv::Point2f arena_src_corners[4]{};
    if (target == Target::ROBOT) {
      vision.setSelfFrontColor(self_front_color);
      if (vision.initialize(kCameraIndex())) {
        vision_active = true;
        std::cout << "Overhead vision enabled (camera index " << kCameraIndex()
                  << "). Self front=" << markerColorName(self_front_color) << ".\n";

        // Load existing arena corners or run the picker now. We always go
        // through the warp path in robot mode so the arena is rectified to
        // a fixed 640x480 (= 6 ft x 4.5 ft, ~8.89 px/in everywhere).
        if (ensureArenaCorners(vision, arena_src_corners, false)) {
          warp_matrix = buildWarpToRect(arena_src_corners);
          vision.setArenaOverride(makeWarpedArenaBoundary());
          arena_calibrated = true;
          std::cout << "Arena perspective warp ready (" << kWarpedFrameW() << "x"
                    << kWarpedFrameH() << ", ~"
                    << (kWarpedFrameW() / kArenaWidthInches()) << " px/in).\n";
        } else {
          std::cerr << "Arena calibration cancelled; running without warp.\n";
        }
      } else {
        std::cerr << "Camera open failed; vision disabled. Run continues open-loop.\n";
      }
    }
#else
    const bool vision_active = false;
#endif

  Image24 frame = background;

  // In vision mode the image_view buffer must match the camera frame size,
  // so reshape `frame` to the warped arena size before allocating the shared-memory buffer.
  if (vision_active) {
    frame.w = kWarpedFrameW();
    frame.h = kWarpedFrameH();
    frame.data.assign((size_t)frame.w * frame.h * 3u, 0);
  }

#ifdef USE_IMAGE_VIEW
  // Same pipeline as Week7: activate_vision + allocate_image, then view_rgb_image each frame.
  activate_vision();
  image rgb{};
  rgb.type = RGB_IMAGE;
  rgb.width = (i2byte)frame.w;
  rgb.height = (i2byte)frame.h;
  if (allocate_image(rgb) != 0) {
      std::cerr << "allocate_image failed - check image_transfer.lib / dimensions\n";
    deactivate_vision();
      if (robot_port) { delete robot_port; robot_port = nullptr; }
      continue;
    }
    std::cout << "\nVision display buffer ready - start image_view.exe first if not already running.\n";
    std::cout << "ESC = abort to menu.";
#ifdef USE_VISION
    if (vision_active) std::cout << "  C = re-pick arena corners.";
#endif
    std::cout << "\n";
#endif

  // Place some static obstacles similar to screenshot
  std::vector<Obstacle> obstacles;
  if (hasBlack) obstacles.push_back({ background.w/2, background.h/2, &obsBlack });
  if (hasGreen) obstacles.push_back({ background.w/3, (background.h*2)/3, &obsGreen });
  if (hasBlue)  obstacles.push_back({ background.w/4, background.h/2, &obsBlue });

  Robot my_robot, target_robot;

  // Initial positions (tune as you like)

  my_robot.x = 50.0f;
  my_robot.y = (float)background.h - 50.0f;
  my_robot.theta = -3.1415926f / 2.0f; // Point up

  // Remember our spawn pose so AI_ATTACK can drive back here after firing.
  // Non-const because in vision mode we re-latch this on the first valid
  // self detection (the hardcoded seed above is sim-only).
  float start_x = my_robot.x;
  float start_y = my_robot.y;
  float start_theta = my_robot.theta;
  bool start_pose_latched = !vision_active; // already valid when not using vision

  // Sticky arena polygon: once vision sees the arena once, hold it for the
  // rest of the run since the camera and arena don't move. The safety clamp
  // and the AI's WorldEnv both read from this. If never detected, the helpers
  // fall back to the image rect (with a generous margin) per the spec.
  ArenaInfo latched_arena{};
  latched_arena.has_polygon = false;
  latched_arena.img_w = frame.w;
  latched_arena.img_h = frame.h;
#ifdef USE_VISION
  // When the perspective warp is active, the arena IS the entire warped frame
  // - so we can latch immediately, no detection needed. Coordinates from the
  // vision pipeline now live in this rectified space everywhere downstream.
  if (arena_calibrated) {
    latched_arena.has_polygon = true;
    latched_arena.corners[0][0] = 0.0f;                            latched_arena.corners[0][1] = 0.0f;
    latched_arena.corners[1][0] = (float)(kWarpedFrameW() - 1);      latched_arena.corners[1][1] = 0.0f;
    latched_arena.corners[2][0] = (float)(kWarpedFrameW() - 1);      latched_arena.corners[2][1] = (float)(kWarpedFrameH() - 1);
    latched_arena.corners[3][0] = 0.0f;                              latched_arena.corners[3][1] = (float)(kWarpedFrameH() - 1);
    latched_arena.img_w = kWarpedFrameW();
    latched_arena.img_h = kWarpedFrameH();
  }
#endif

  target_robot.x = (float)background.w - 50.0f;
  target_robot.y = 50.0f;

  // orbit parameters
  const float cx = background.w * 0.55f;
  const float cy = background.h * 0.45f;
  const float radius = std::min(background.w, background.h) * 0.28f;

  auto tStart = std::chrono::steady_clock::now();
  float lastT = 0.0f;

  bool fired = false;
  bool hit = false;
  float hit_time = -1.0f;
  float fire_time = -1.0f;

  // Breadcrumb trail of outbound positions for post-shot return-home retrace.
  // Populated every frame while !my_robot.shotUsed, consumed by update_return_home_ai.
  std::vector<std::pair<float, float>> trail;
  
  uint8_t robot_cmd_seq = 0;
  AttackAiStability attack_ai_stab{};

#ifdef USE_VISION
  Mode5AttackPlanner mode5_planner;
  const bool use_mode5_map_planner =
      (mode == Mode::AI_ATTACK && target == Target::ROBOT && vision_active && arena_calibrated);
  int mode5_frame_counter = 0;
#endif

  // New scenario always starts in attack phase (not post-shot return-home).
  my_robot.shotUsed = false;
  fired = false;
  hit = false;
  hit_time = -1.0f;
  fire_time = -1.0f;
  trail.clear();
  target_robot.is_alive = true;
  attack_ai_stab.reset();

#ifdef USE_VISION
  cv::Mat vision_bgr;
  GameState vision_state{};
  bool vision_self_valid = false;
  bool vision_enemy_valid = false;
  bool c_was_down = false;
#endif

  // Main loop
  while (true) {
    auto now = std::chrono::steady_clock::now();
    float t = std::chrono::duration<float>(now - tStart).count();
    float dt = t - lastT;
    lastT = t;

    // Stop after ~60 seconds (edit as you like)
    if (t > 60.0f) break;

    // ESC at any time aborts the scenario and returns to the main menu.
    if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) {
      std::cout << "ESC pressed - aborting scenario.\n";
      break;
    }

#ifdef USE_VISION
    // 'C' triggers a fresh arena corner pick mid-run. Edge-detected so
    // holding the key doesn't loop the picker.
    bool c_now_down = (GetAsyncKeyState('C') & 0x8000) != 0;
    if (vision_active && c_now_down && !c_was_down) {
      std::cout << "C pressed - re-picking arena corners.\n";
      // Send a stop to the robot so it doesn't keep moving while the
      // operator clicks corners.
      if (target == Target::ROBOT && robot_port) {
        CmdPacket stopCmd{};
        stopCmd.seq = robot_cmd_seq++;
        stopCmd.chk = checksum_xor(stopCmd);
        sendRobotCommand(*robot_port, stopCmd);
      }
      cv::Point2f new_corners[4]{};
      if (ensureArenaCorners(vision, new_corners, true)) {
        for (int k = 0; k < 4; ++k) arena_src_corners[k] = new_corners[k];
        warp_matrix = buildWarpToRect(arena_src_corners);
        vision.setArenaOverride(makeWarpedArenaBoundary());
        arena_calibrated = true;
        // Refresh the latched polygon to the warped rect.
        latched_arena.has_polygon = true;
        latched_arena.corners[0][0] = 0.0f;                       latched_arena.corners[0][1] = 0.0f;
        latched_arena.corners[1][0] = (float)(kWarpedFrameW() - 1); latched_arena.corners[1][1] = 0.0f;
        latched_arena.corners[2][0] = (float)(kWarpedFrameW() - 1); latched_arena.corners[2][1] = (float)(kWarpedFrameH() - 1);
        latched_arena.corners[3][0] = 0.0f;                         latched_arena.corners[3][1] = (float)(kWarpedFrameH() - 1);
        latched_arena.img_w = kWarpedFrameW();
        latched_arena.img_h = kWarpedFrameH();
        std::cout << "Arena warp refreshed.\n";
      }
    }
    c_was_down = c_now_down;
#endif

#ifdef USE_VISION
    // 0) Pull a camera frame and run the vision pipeline. Replaces open-loop
    //    dead reckoning and the BMP obstacle layout in robot mode.
    vision_self_valid = false;
    vision_enemy_valid = false;
    cv::Mat raw_bgr;
    if (vision_active && vision.grabFrame(raw_bgr)) {
      // Some webcam drivers deliver a mirrored feed. If that's the case, flip here so
      // *everything* (warp, detection, overlay, control) shares the same orientation.
      if (params().vision_flip_x || params().vision_flip_y) {
        const int flip_code = (params().vision_flip_x && params().vision_flip_y) ? -1
                            : (params().vision_flip_x ? 1 : 0);
        cv::flip(raw_bgr, raw_bgr, flip_code);
      }
      // Rectify the camera frame to a screen-aligned arena rectangle so the
      // rest of the pipeline (marker detection, AI, safety clamp) operates
      // in a uniform top-down coordinate system. When calibration was
      // skipped, fall back to the raw frame.
      if (arena_calibrated && !warp_matrix.empty()) {
        cv::warpPerspective(raw_bgr, vision_bgr, warp_matrix,
                            cv::Size(kWarpedFrameW(), kWarpedFrameH()));
      } else {
        vision_bgr = raw_bgr;
      }

      vision_state = vision.processFrame(vision_bgr);

      // Latch the arena polygon on first sighting and never invalidate.
      // Subsequent valid frames refresh the corners (camera/arena are static
      // but small contour wobble is fine to absorb). When the warp is active
      // the latch was already populated above; this branch is the no-warp
      // path (e.g. user cancelled corner picking).
      if (!arena_calibrated && vision_state.arena.valid) {
        latched_arena.has_polygon = true;
        for (int k = 0; k < 4; ++k) {
          latched_arena.corners[k][0] = vision_state.arena.corners[k].x;
          latched_arena.corners[k][1] = vision_state.arena.corners[k].y;
        }
        if (latched_arena.img_w == 0 || latched_arena.img_h == 0) {
          latched_arena.img_w = frame.w;
          latched_arena.img_h = frame.h;
        }
      }

      if (vision_state.self.pose.valid) {
        my_robot.x = vision_state.self.pose.x;
        my_robot.y = vision_state.self.pose.y;
        my_robot.theta = visionHeadingToRobotTheta(vision_state.self.pose.theta);
        vision_self_valid = true;

        if (!start_pose_latched) {
          start_x = my_robot.x;
          start_y = my_robot.y;
          start_theta = my_robot.theta;
          start_pose_latched = true;
          std::cout << "Spawn latched from vision: (" << start_x << ", " << start_y
                    << ") theta=" << start_theta << "\n";
        }
      }
      if (vision_state.enemy.pose.valid) {
        target_robot.x = vision_state.enemy.pose.x;
        target_robot.y = vision_state.enemy.pose.y;
        target_robot.theta = visionHeadingToRobotTheta(vision_state.enemy.pose.theta);
        target_robot.is_alive = true;
        vision_enemy_valid = true;
      }

      // Rebuild obstacles list from vision each frame. 0..N circular obstacles.
      // Drop circles that overlap our body (black hull is often picked as an obstacle).
      obstacles.clear();
      obstacles.reserve(vision_state.obstacles.size());
      const float self_r = params().robot_radius_px;
      const float selfObsSlack = 6.0f; // contour / centroid noise (px)
      for (const auto& vo : vision_state.obstacles) {
        if (!vo.valid) continue;
        if (vision_self_valid) {
          const float dx = vo.x - my_robot.x;
          const float dy = vo.y - my_robot.y;
          const float cut = self_r + vo.radius + selfObsSlack;
          if (dx * dx + dy * dy < cut * cut) continue;
        }
        Obstacle o{};
        o.x = (int)vo.x;
        o.y = (int)vo.y;
        o.img = nullptr;
        o.radius = vo.radius;
        obstacles.push_back(o);
      }
    }
#endif

#ifdef USE_VISION
    if (use_mode5_map_planner) {
      ++mode5_frame_counter;
      if (vision_active && arena_calibrated && !vision_bgr.empty()) {
        if (!mode5_planner.mapReady()) {
          mode5_planner.tryInit(vision, vision_bgr, latched_arena, self_front_color);
        } else {
          obstacles = mode5_planner.plannerObstacles();
          mode5_planner.tickReconcile(mode5_frame_counter, vision, vision_bgr, self_front_color,
                                      latched_arena);
        }
      }
    }
#endif

	// 1) Update motion
	my_robot.v = 0.0f;
	my_robot.w = 0.0f;
    int8_t robot_L_cmd = 0;
    int8_t robot_R_cmd = 0;
    float turret_w = 0.0f;
    AIDebug ai_dbg{};

    switch(mode) {
        case Mode::USER_CONTROL:
        {
	        if (GetAsyncKeyState('W') & 0x8000) my_robot.v = 150.0f;
	        if (GetAsyncKeyState('S') & 0x8000) my_robot.v = -150.0f;
	        if (GetAsyncKeyState('A') & 0x8000) my_robot.w = 2.0f;
	        if (GetAsyncKeyState('D') & 0x8000) my_robot.w = -2.0f;
	        if (GetAsyncKeyState('Q') & 0x8000) turret_w = 2.0f;
	        if (GetAsyncKeyState('E') & 0x8000) turret_w = -2.0f;
	        if (!my_robot.shotUsed && (GetAsyncKeyState('F') & 0x8000)) {
		        my_robot.shotUsed = true;
		        fired = true;
		        fire_time = t;

		        // check for hit
		        if (has_clear_los(my_robot.x, my_robot.y, target_robot.x, target_robot.y, obstacles)) {
			        float angle_to_target = atan2(target_robot.y - my_robot.y, target_robot.x - my_robot.x);
			        float turret_world_angle = my_robot.theta + my_robot.turret + 3.1415926f / 2.0f;
			        float angle_diff = wrapAngle(turret_world_angle - angle_to_target);

			        if (fabs(angle_diff) < 0.2f) { // Generous aiming tolerance
				        hit = true;
				        hit_time = t;
				        target_robot.is_alive = false;
			        }
		        }
	        }
            if (target == Target::ROBOT) {
                apply_robot_wasd_lr(robot_L_cmd, robot_R_cmd);
            }
            break;
        }
        case Mode::AI_ATTACK:
        case Mode::AI_DEFEND: {
            // Build the path-clearance environment for the AI: obstacles +
            // enemy as a moving disk + the latched arena polygon (or the
            // image rect if vision hasn't found the arena yet this run).
            WorldEnv env{};
            env.obstacles = &obstacles;
            env.other_robot = target_robot.is_alive ? &target_robot : nullptr;
            env.robot_radius = 25.0f;
            // The AI margin is inflated only when driving the real robot
            // through the perspective-warped arena - that path has real
            // command latency to budget for. In simulator mode (or robot
            // mode without vision/arena), keep the original tighter margin
            // so the AI doesn't refuse to leave a corner spawn.
            const bool real_with_arena = (target == Target::ROBOT) && latched_arena.has_polygon;
            env.safety_margin = real_with_arena ? params().ai_margin_poly_px
                                               : params().ai_margin_other_px;
            env.arena = latched_arena;
            if (env.arena.img_w == 0 || env.arena.img_h == 0) {
                env.arena.img_w = frame.w;
                env.arena.img_h = frame.h;
            }

            if (mode == Mode::AI_ATTACK) {
                if (my_robot.shotUsed) {
                    if (target == Target::ROBOT) {
                        my_robot.v = 0.0f;
                        my_robot.w = 0.0f;
                        if (std::fabs(my_robot.turret) > kPostShotTurretHomeTolRad) {
                            slew_turret_toward_neutral_rad(my_robot.turret, dt);
                            ai_dbg.detail = "post_shot_turret_return";
                            turret_w = 0.0f;
                        } else {
                            apply_robot_wasd_lr(robot_L_cmd, robot_R_cmd);
                            ai_dbg.detail = "manual_post_shot";
                            if (GetAsyncKeyState('Q') & 0x8000) turret_w = 2.0f;
                            if (GetAsyncKeyState('E') & 0x8000) turret_w = -2.0f;
                        }
                    } else {
                        update_return_home_ai(my_robot, start_x, start_y, start_theta, trail, obstacles, dt,
                                              &ai_dbg);
                    }
                } else {
#ifdef USE_VISION
                    if (use_mode5_map_planner && mode5_planner.mapReady()) {
                        mode5_planner.update(my_robot, target_robot, obstacles, dt, t, fired, fire_time, hit,
                                             hit_time, ai_dbg);
                    } else
#endif
                    {
                        update_attack_ai(my_robot, target_robot, obstacles, env, dt, fired, t, fire_time, hit, hit_time,
                                         &ai_dbg, &attack_ai_stab);
                    }
                }
                if (hit) target_robot.is_alive = false;
            } else {
                update_defend_ai(my_robot, target_robot, obstacles, env, dt, &ai_dbg);
            }

            // Terminal debug: what the AI *requested* this frame (rate-limited).
            {
              static auto last_ai_print = std::chrono::steady_clock::time_point{};
              auto now_ai = std::chrono::steady_clock::now();
              if (std::chrono::duration<float>(now_ai - last_ai_print).count() > 1.0f) {
                std::cout << "[AI] mode=" << (mode == Mode::AI_ATTACK ? "ATTACK" : "DEFEND")
                          << " v=" << (int)my_robot.v << " w=" << my_robot.w
                          << " shotUsed=" << (my_robot.shotUsed ? "Y" : "N")
                          << " state=" << (int)ai_dbg.state
                          << " detail=" << ai_dbg.detail
                          << " pos=(" << (int)my_robot.x << "," << (int)my_robot.y << ")"
                          << " goal=(" << (int)ai_dbg.goal_x << "," << (int)ai_dbg.goal_y << ")"
                          << "\n";
                last_ai_print = now_ai;
              }
            }
            break;
        }
    }

    my_robot.turret += turret_w * dt;
    float limit = 170.0f * 3.1415926f / 180.0f / 2.0f;
    if (my_robot.turret > limit) my_robot.turret = limit;
    if (my_robot.turret < -limit) my_robot.turret = -limit;

    // --- Universal out-of-bounds safety clamp ---
    // Predict where the robot would be ~0.3 s from now using the currently
    // requested v / w (or, in user-control robot mode, the equivalent v/w
    // implied by the WASD wheel commands). If that prediction would exit
    // the arena polygon (or the image rect when the polygon hasn't been
    // detected yet) with a generous margin, zero out motion for this frame.
    // Applies to AI modes, return-home, and user control alike.
    const bool skip_robot_manual_wall_clamp =
        params().robot_user_disable_wall_clamp && target == Target::ROBOT &&
        ((mode == Mode::USER_CONTROL) || (mode == Mode::AI_ATTACK && my_robot.shotUsed));
    if (!skip_robot_manual_wall_clamp) {
      WorldEnv safety_env{};
      safety_env.obstacles = nullptr;          // walls only - obstacles handled by AI
      safety_env.other_robot = nullptr;
      safety_env.robot_radius = 25.0f;
      safety_env.arena = latched_arena;
      if (safety_env.arena.img_w == 0 || safety_env.arena.img_h == 0) {
        safety_env.arena.img_w = frame.w;
        safety_env.arena.img_h = frame.h;
      }
      // Slightly tighter than before so the robot uses more of the floor
      // without trading away the latency coast padding (applied below).
      safety_env.safety_margin = latched_arena.has_polygon ? params().safety_margin_poly_px
                                                          : params().safety_margin_fallback_px;

      // Map manual wheel commands to an equivalent v/w for the predict.
      float check_v = my_robot.v;
      float check_w = my_robot.w;
      if (target == Target::ROBOT &&
          (mode == Mode::USER_CONTROL ||
           (mode == Mode::AI_ATTACK && my_robot.shotUsed))) {
        const float v_per_unit = 150.0f / 100.0f;
        const float wheel_base = 50.0f;
        const float vL = robot_L_cmd * v_per_unit;
        const float vR = robot_R_cmd * v_per_unit;
        check_v = 0.5f * (vL + vR);
        check_w = (vR - vL) / wheel_base;
      }

      // ---------------------------------------------------------------
      // Latency-aware graded safety clamp - REAL ROBOT ONLY.
      //
      // Real-world end-to-end delay (camera buffer + warp/process + BT
      // serial + Arduino + mechanical inertia) is ~250-350 ms. A binary
      // 300 ms lookahead reacts too late: the robot coasts past the
      // boundary by the time the STOP command lands.
      //
      // Mitigation (when target == ROBOT):
      //   * Inflate the effective radius by the coasting distance.
      //   * Sample safety at four horizons. Earliest unsafe horizon
      //     determines a speed scale (0 / 33 / 66 / 85 %).
      //   * If translation is fully clamped (scale==0), still allow
      //     in-place rotation when that alone keeps the footprint inside
      //     the arena — so attack mode can finish lining up a shot at the edge.
      //   * If we are hard-stopped with no rotation for several frames,
      //     apply a small verified back-up + turn toward the goal.
      //
      // The simulator path is intentionally skipped: there's no command
      // latency in sim, and the existing position-revert (check_collision
      // + arena polygon revert below) cleanly handles boundaries without
      // the inflated margin choking AI behavior at corner spawns.
      // ---------------------------------------------------------------
      bool robot_safety_full_translation_stop = false;
      if (target == Target::ROBOT) {
        const float v_before = my_robot.v;
        const float w_before = my_robot.w;
        const float kCommandRoundTripSec = params().cmd_roundtrip_s;
        const float kBrakeMarginSec      = params().brake_margin_s;
        const float coast_px = std::fabs(check_v) * (kCommandRoundTripSec + kBrakeMarginSec);
        safety_env.safety_margin += coast_px;

        const float* horizons = params().safety_horizons_s;
        const float* scales   = params().safety_scales;

        int unsafe_index = -1;
        for (int hi = 0; hi < 4; ++hi) {
          if (!predicted_pose_safe(my_robot, check_v, check_w, horizons[hi], safety_env)) {
            unsafe_index = hi;
            break;
          }
        }

        if (unsafe_index >= 0) {
          const float scale = scales[unsafe_index];
          my_robot.v *= scale;
          robot_L_cmd = (int8_t)std::lround(robot_L_cmd * scale);
          robot_R_cmd = (int8_t)std::lround(robot_R_cmd * scale);
          if (scale == 0.0f) {
            robot_safety_full_translation_stop = true;
            // Pure rotation is often still safe when forward motion is not.
            // IMPORTANT: rotation-in-place does not change the robot center. If the center is still
            // inside the arena polygon, allow rotation even when the padded footprint check fails.
            // This avoids "stuck aiming" at the boundary where turning the body is required to
            // bring the target into the turret arc.
            const bool center_in = point_in_arena(safety_env, my_robot.x, my_robot.y);
            if (center_in || predicted_pose_safe(my_robot, 0.0f, check_w, horizons[0], safety_env)) {
              my_robot.w = check_w;
            } else {
              my_robot.w = 0.0f;
            }
          }

          static auto last_warn = std::chrono::steady_clock::time_point{};
          auto now_warn = std::chrono::steady_clock::now();
          if (std::chrono::duration<float>(now_warn - last_warn).count() > 1.0f) {
            std::cout << "[SAFETY] OOB at horizon " << horizons[unsafe_index]
                      << "s -> speed x" << scale << " at ("
                      << (int)my_robot.x << ", " << (int)my_robot.y << ")\n";
            last_warn = now_warn;
          }
        }

        // Extra debug when safety changes motion (rate-limited).
        {
          static auto last_sdbg = std::chrono::steady_clock::time_point{};
          auto now_sdbg = std::chrono::steady_clock::now();
          const bool motion_changed =
              (std::fabs(my_robot.v - v_before) > 0.5f) || (std::fabs(my_robot.w - w_before) > 0.08f);
          if (motion_changed && std::chrono::duration<float>(now_sdbg - last_sdbg).count() > 0.6f) {
            std::cout << "[SAFETY] clamp v:" << (int)v_before << "->" << (int)my_robot.v
                      << " w:" << w_before << "->" << my_robot.w
                      << " coast_px=" << (int)coast_px
                      << " base_margin=" << (latched_arena.has_polygon ? params().safety_margin_poly_px
                                                                       : params().safety_margin_fallback_px)
                      << " fullStop=" << (robot_safety_full_translation_stop ? "Y" : "N")
                      << "\n";
            last_sdbg = now_sdbg;
          }
        }

        // Edge escape: translation clamped to ~0 and no useful rotation for a
        // short hold -> nudge backward while turning toward the goal so the AI
        // doesn't ponder at the tape for seconds.
        static int s_edgeRecoverFrames = 0;
        if (vision_active &&
            (mode == Mode::AI_ATTACK || mode == Mode::AI_DEFEND) &&
            robot_safety_full_translation_stop) {
          if (std::fabs(my_robot.v) < 0.05f && std::fabs(my_robot.w) < 0.09f) {
            s_edgeRecoverFrames++;
          } else {
            s_edgeRecoverFrames = 0;
          }
          if (s_edgeRecoverFrames >= params().edge_recover_hold_frames) {
            s_edgeRecoverFrames = 0;
            WorldEnv rec{};
            rec.obstacles = nullptr;
            rec.other_robot = nullptr;
            rec.robot_radius = 25.0f;
            rec.arena = latched_arena;
            if (rec.arena.img_w == 0 || rec.arena.img_h == 0) {
              rec.arena.img_w = frame.w;
              rec.arena.img_h = frame.h;
            }
            // Recovery check uses slightly tighter margins so that a small
            // retreat is possible even when the main clamp is conservative.
            rec.safety_margin = latched_arena.has_polygon ? (params().safety_margin_poly_px - 2.0f)
                                                         : (params().safety_margin_fallback_px - 2.0f);

            const float kBackV = params().edge_recover_back_v_px_s;
            const float goal_h =
                std::atan2(target_robot.y - my_robot.y, target_robot.x - my_robot.x);
            const float fwd_h = my_robot.theta + 3.1415926f / 2.0f;
            const float turn_w =
                (wrapAngle(goal_h - fwd_h) > 0.0f) ? params().edge_recover_turn_w_rad_s
                                                   : -params().edge_recover_turn_w_rad_s;
            if (predicted_pose_safe(my_robot, kBackV, turn_w, params().edge_recover_check_s, rec)) {
              my_robot.v = kBackV;
              my_robot.w = turn_w;
            }
          }
        } else if (target == Target::ROBOT) {
          s_edgeRecoverFrames = 0;
        }
      }
    }

    // --- Pose integration (shared) ---
    // Simulator: this IS the robot's pose.
    // Robot target without vision: open-loop dead reckoning.
    // Robot target with vision: pose already injected above, skip integration.
	    float old_x = my_robot.x;
	    float old_y = my_robot.y;
    if (!vision_active) {
      my_robot.theta += my_robot.w * dt;
	    my_robot.x += my_robot.v * cos(my_robot.theta + 3.1415926f / 2.0f) * dt;
	    my_robot.y += my_robot.v * sin(my_robot.theta + 3.1415926f / 2.0f) * dt;
    }

    // --- SIMULATOR OR ROBOT ---
    if (target == Target::SIMULATOR) {
        // In the simulator, obstacles/walls are ground truth – revert on collision.
	    if (check_collision(my_robot, obstacles, background.w, background.h)) {
		    my_robot.x = old_x;
		    my_robot.y = old_y;
	    }
        // Also revert if we just stepped outside the (latched) arena polygon -
        // mirrors what the safety clamp will do on the real robot.
        if (latched_arena.has_polygon &&
            !point_in_polygon(my_robot.x, my_robot.y, latched_arena.corners, 4)) {
		    my_robot.x = old_x;
		    my_robot.y = old_y;
	    }
    } else { // target == Target::ROBOT
        const bool robot_post_shot_manual_ready =
            mode == Mode::AI_ATTACK && target == Target::ROBOT && my_robot.shotUsed &&
            std::fabs(my_robot.turret) <= kPostShotTurretHomeTolRad;
        const bool robot_manual_wheels =
            (mode == Mode::USER_CONTROL && target == Target::ROBOT) || robot_post_shot_manual_ready;

        CmdPacket cmd{};
        cmd.seq = robot_cmd_seq++;
        cmd.flags = 0;
        if (mode == Mode::AI_ATTACK && !robot_manual_wheels) cmd.flags |= FLAG_MODE_ATTACK;
        if (mode == Mode::AI_DEFEND) cmd.flags |= FLAG_MODE_DEF;

        if (robot_manual_wheels) {
            cmd.left = robot_L_cmd;
            cmd.right = robot_R_cmd;
        } else {
            float v_cmd = my_robot.v;
            if (params().ai_invert_v) v_cmd = -v_cmd;
            float w_cmd = my_robot.w;
            if (params().ai_invert_w) w_cmd = -w_cmd;
            vw_to_lr(v_cmd, w_cmd, cmd.left, cmd.right);
            if (params().ai_swap_lr) {
                int8_t tmp = cmd.left;
                cmd.left = cmd.right;
                cmd.right = tmp;
            }
        }

        // Turret angle needs to be converted from radians relative to body to 0-180 degrees
        // Arduino's 0-180 maps to our -limit to +limit
        float turret_deg_f = (my_robot.turret / limit) * 90.0f + 90.0f;
        cmd.turret_deg = (uint8_t)clampi((int)turret_deg_f, 0, 180);
        if (params().turret_servo_inverted) {
            cmd.turret_deg = (uint8_t)(180 - cmd.turret_deg);
        }

        cmd.laser = 0;
        if (fired && t - fire_time < 0.5f) { // Send fire command for a duration
            cmd.flags |= FLAG_FIRE_ARMED;
            cmd.laser = 1;
        }
        
        cmd.chk = checksum_xor(cmd);
        // Rate-limited debug: print outgoing BT packets + turret angles.
        {
          static auto last_pkt = std::chrono::steady_clock::time_point{};
          static float last_theta_dbg = 0.0f;
          static float last_x_dbg = 0.0f;
          static float last_y_dbg = 0.0f;
          static float last_t_dbg = -1.0f;
          static bool have_last_dbg = false;
          auto now_pkt = std::chrono::steady_clock::now();
          if (std::chrono::duration<float>(now_pkt - last_pkt).count() > 0.20f) {
            const float turret_world = my_robot.theta + my_robot.turret + 3.1415926f / 2.0f;
            // Sanity checks: does observed theta change match commanded w sign?
            float obs_w = 0.0f;
            float move_px = 0.0f;
            if (have_last_dbg && last_t_dbg >= 0.0f) {
              const float dt_dbg = std::max(1e-3f, t - last_t_dbg);
              obs_w = wrapAngle(my_robot.theta - last_theta_dbg) / dt_dbg;
              const float dx = my_robot.x - last_x_dbg;
              const float dy = my_robot.y - last_y_dbg;
              move_px = std::sqrt(dx * dx + dy * dy);
            }
            std::cout << "[BT] seq=" << (int)cmd.seq
                      << " flags=0x" << std::hex << (int)cmd.flags << std::dec
                      << " L=" << (int)cmd.left << " R=" << (int)cmd.right
                      << " turret_deg=" << (int)cmd.turret_deg
                      << " laser=" << (int)cmd.laser
                      << " chk=0x" << std::hex << (int)cmd.chk << std::dec
                      << " | pose=(" << (int)my_robot.x << "," << (int)my_robot.y << ")"
                      << " theta=" << my_robot.theta
                      << " turret_rel=" << my_robot.turret
                      << " turret_world=" << turret_world
                      << " obs_w=" << obs_w
                      << " move_px=" << move_px
                      << "\n";

            // Warnings (help diagnose "axes reversed" vs "vision pose jump").
            if (have_last_dbg) {
              if (std::fabs(my_robot.w) > 0.6f && std::fabs(obs_w) > 0.4f) {
                const bool sign_mismatch = (my_robot.w > 0.0f && obs_w < 0.0f) || (my_robot.w < 0.0f && obs_w > 0.0f);
                if (sign_mismatch) {
                  std::cout << "[BT][WARN] turn sign mismatch: cmd_w=" << my_robot.w << " obs_w=" << obs_w
                            << " (possible L/R swap or w sign convention mismatch)\n";
                }
              }
              if (std::fabs(my_robot.v) < 1.0f && move_px > 18.0f) {
                std::cout << "[BT][WARN] pose moved while v≈0: move_px=" << move_px
                          << " (likely vision pose jitter / marker re-ID flip)\n";
              }
            }

            last_theta_dbg = my_robot.theta;
            last_x_dbg = my_robot.x;
            last_y_dbg = my_robot.y;
            last_t_dbg = t;
            have_last_dbg = true;
            last_pkt = now_pkt;
          }
        }
        sendRobotCommand(*robot_port, cmd);
    }
	
    // Record outbound breadcrumb for post-shot return-home retrace.
    // Only while the shot hasn't been fired yet; min-spacing filter keeps the
    // trail compact and also naturally filters small noise (future vision pose).
    if (!my_robot.shotUsed) {
        const float min_spacing_sq = 15.0f * 15.0f;
        if (trail.empty()) {
            trail.push_back({my_robot.x, my_robot.y});
        } else {
            float bx = trail.back().first;
            float by = trail.back().second;
            float ddx = my_robot.x - bx;
            float ddy = my_robot.y - by;
            if (ddx * ddx + ddy * ddy >= min_spacing_sq) {
                trail.push_back({my_robot.x, my_robot.y});
            }
        }
    }

    // target robot motion (scripted orbit, sim-only).
    // In vision mode the enemy pose is whatever the camera sees, so skip this.
    if (!vision_active) {
	const float target_speed = 100.0f; // pixels/sec
	const float target_turn_rate = 2.0f; // rad/s

	// Find a point slightly ahead on the circle to aim for
	float future_ang = -0.6f * (t + 0.5f) + 1.3f;
	float target_x = cx + radius * std::cos(future_ang);
	float target_y = cy + radius * std::sin(future_ang);

	float desired_heading = std::atan2(target_y - target_robot.y, target_x - target_robot.x);
	float current_forward_heading = target_robot.theta + 3.1415926f / 2.0f;
	float angle_diff = wrapAngle(desired_heading - current_forward_heading);
	
	float turn_amount = 0.0f;
      if (angle_diff > 0.01f)      turn_amount =  target_turn_rate;
      else if (angle_diff < -0.01f) turn_amount = -target_turn_rate;

	target_robot.theta += turn_amount * dt;
	
	float old_tx = target_robot.x;
	float old_ty = target_robot.y;
	target_robot.x += target_speed * cos(current_forward_heading) * dt;
	target_robot.y += target_speed * sin(current_forward_heading) * dt;

	if (check_collision(target_robot, obstacles, background.w, background.h)) {
		target_robot.x = old_tx;
		target_robot.y = old_ty;
      }
	}

    // Turrets: point at opponent (world frame)
    target_robot.turret = std::atan2(my_robot.y - target_robot.y, my_robot.x - target_robot.x);

    // 2) Render: camera frame (vision mode) or background sprite (sim mode)
#ifdef USE_VISION
    if (vision_active && !vision_bgr.empty() &&
        vision_bgr.cols == frame.w && vision_bgr.rows == frame.h &&
        vision_bgr.type() == CV_8UC3) {
      // Burn detection overlays onto the camera frame before copying so the
      // operator sees what vision is currently latched onto.
      drawVisionOverlay(vision_bgr, vision_state, my_robot.turret);
      // Extra debug overlays: planned path, inflated hitboxes, and AI intent label.
      WorldEnv overlay_env{};
      overlay_env.obstacles = &obstacles;
      overlay_env.other_robot = target_robot.is_alive ? &target_robot : nullptr;
      overlay_env.robot_radius = params().robot_radius_px;
      overlay_env.safety_margin = params().ai_margin_poly_px;
      overlay_env.arena = latched_arena;
      if (overlay_env.arena.img_w == 0 || overlay_env.arena.img_h == 0) {
        overlay_env.arena.img_w = frame.w;
        overlay_env.arena.img_h = frame.h;
      }
      drawVisionDebug(vision_bgr, vision_state, overlay_env, ai_dbg, params());

      // cv::Mat BGR matches Image24 byte order, so a plain memcpy works.
      const size_t rowBytes = (size_t)frame.w * 3u;
      if (vision_bgr.isContinuous()) {
        std::memcpy(frame.data.data(), vision_bgr.data, rowBytes * (size_t)frame.h);
      } else {
        for (int y = 0; y < frame.h; ++y) {
          std::memcpy(&frame.data[(size_t)y * rowBytes], vision_bgr.ptr(y), rowBytes);
        }
      }
    } else
#endif
    {
    frame = background;
    }

    // Sprite blits only in sim mode; in vision mode obstacles and robots are
    // already baked into the camera feed.
    if (!vision_active) {
    for (auto &o : obstacles) {
        if (o.img) blit(frame, *o.img, o.x, o.y, true);
    }
    blitRotated(frame, robotA, (int)my_robot.x, (int)my_robot.y, my_robot.theta, true);
      if (target_robot.is_alive)
		blitRotated(frame, robotB, (int)target_robot.x, (int)target_robot.y, target_robot.theta, true);

      // Optional: draw the planned path + goal marker in simulator modes too.
      drawSimDebug(frame, ai_dbg);
    }
    // LEDs (front/back indicator) - you can match your real LED colors here
    auto drawRobotLEDs = [&](const Robot &R, uint8_t fr, uint8_t fg, uint8_t fb,
                             uint8_t rr, uint8_t rg, uint8_t rb) {
      float fx = std::cos(R.theta), fy = std::sin(R.theta);
      // front point and rear point offsets (tune these)
      int frontX = (int)(R.x + fx * 22.0f);
      int frontY = (int)(R.y + fy * 22.0f);
      int rearX  = (int)(R.x - fx * 22.0f);
      int rearY  = (int)(R.y - fy * 22.0f);

      drawCircle(frame, frontX, frontY, 5, fr, fg, fb);
      drawCircle(frame, rearX, rearY, 5, rr, rg, rb);
    };

    // Overlay own LEDs whenever pose is known. In vision mode that means the
    // camera is currently reporting (or holding) a valid pose for that robot.
#ifdef USE_VISION
    bool draw_self_overlay  = vision_active ? vision_self_valid : true;
    bool draw_enemy_overlay = vision_active ? (vision_enemy_valid && target_robot.is_alive != 0)
                                            : (target_robot.is_alive != 0);
#else
    bool draw_self_overlay  = true;
    bool draw_enemy_overlay = (target_robot.is_alive != 0);
#endif

    if (draw_self_overlay)
    drawRobotLEDs(my_robot, 0,255,0, 255,0,0); // A: green front, red back
    if (draw_enemy_overlay)
		drawRobotLEDs(target_robot, 0,255,255, 255,0,255); // B: cyan front, magenta back

    // Turret direction lines
    auto drawTurretLine = [&](const Robot &R, bool is_my_robot) {
      float turret_angle = is_my_robot ? R.theta + R.turret + 3.1415926f / 2.0f : R.turret;
      float tx = std::cos(turret_angle);
      float ty = std::sin(turret_angle);
      drawLine(frame, (int)R.x, (int)R.y, (int)(R.x + tx * 50.0f), (int)(R.y + ty * 50.0f), 255,0,255, 1);
    };
    if (draw_self_overlay)
    drawTurretLine(my_robot, true);
    if (draw_enemy_overlay)
		drawTurretLine(target_robot, false);

    // Laser fire viz
    if (fired && t - fire_time < 0.2f) {
      float laser_angle = my_robot.theta + my_robot.turret + 3.1415926f / 2.0f;
      float laser_end_x = my_robot.x + 1000 * cos(laser_angle);
      float laser_end_y = my_robot.y + 1000 * sin(laser_angle);

      // Check for obstacle intersection
      for (float i = 0.0f; i < 1.0f; i += 0.01f) {
          int x = (int)(my_robot.x + i * (laser_end_x - my_robot.x));
          int y = (int)(my_robot.y + i * (laser_end_y - my_robot.y));

          bool stop_laser = false;
          for (auto& o : obstacles) {
              if (point_hits_obstacle(x, y, o)) {
                  laser_end_x = x;
                  laser_end_y = y;
                  stop_laser = true;
                  break;
              }
          }
          if (stop_laser) break;
      }

      drawLine(frame, (int)my_robot.x, (int)my_robot.y, (int)laser_end_x, (int)laser_end_y, 255,0,0, 2);
    }

    // 3) Output
    if (target == Target::SIMULATOR) {
        writeFrame(frame);
    }
#ifdef USE_IMAGE_VIEW
    pushFrameToImageView(frame, rgb);
#endif

    // ~30 FPS
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }

#ifdef USE_IMAGE_VIEW
  free_image(rgb);
  deactivate_vision();
#endif

    if (target == Target::ROBOT && robot_port) {
        // Send a final all-zero command so the robot stops moving when we
        // bail back to the menu instead of coasting on the last command.
        CmdPacket stopCmd{};
        stopCmd.seq = robot_cmd_seq++;
        stopCmd.chk = checksum_xor(stopCmd);
        sendRobotCommand(*robot_port, stopCmd);
    }

  if (robot_port) {
      delete robot_port;
        robot_port = nullptr;
  }

    std::cout << "\nScenario finished. Returning to main menu...\n";
  }  // end outer menu loop

  return 0;
}
# o2D_Sim — Codebase & software context

This document describes the **2D robot arena simulator + overhead-vision + Bluetooth hardware control** project under `Project/o2D_Sim/`. First principles: libraries, architecture, control flow, vision, AI strategies, mode behavior, and a history of notable fixes.

---

## 1. High-level purpose

The application is a **single Windows-centric C++17 program** (`program.cpp` plus modules) that can:

- Run a **2D BMP-based simulation** (two robots, obstacles, walls, composited output to `output.bmp`).
- Optionally drive a **real differential-drive robot with a turret** over **serial (Bluetooth module, e.g. HC-05)** using packed `CmdPacket` structures (`RobotIO.{h,cpp}`).
- Optionally use an **overhead USB webcam** and **OpenCV** to detect colored markers, estimate poses in a **perspective-warped arena**, and close the loop on the **real robot’s pose** while running AI or operator control.

Optional compile flags:

| Macro | Effect |
|--------|--------|
| `USE_VISION` | Compiles OpenCV vision pipeline (`VisionSystem`, warp, Mode 5 planner). |
| `USE_IMAGE_VIEW` | Uses `image_transfer.lib` shared-memory frames for live display in `image_view.exe` (MSVC x86 only per shipped `.lib`). |

---

## 2. Libraries and dependencies

### 2.1 Standard / platform

- **C++17** standard library (`<chrono>`, `<vector>`, `<iostream>`, etc.).
- **Windows (`_WIN32`)**: `windows.h`, `GetAsyncKeyState` for keyboard; `conio.h` for `_kbhit` / `_getch` menu input; **serial** (`serial.h`) for COM ports.
- **`user32.lib`**: MinGW link for keyboard APIs when building a minimal `program.exe`.

### 2.2 OpenCV (when `USE_VISION`)

- Used from **`opencv2/opencv.hpp`** in `VisionSystem`, calibration, perspective warp, and `program.cpp` (warped `cv::Mat`).
- Typical vcpkg install (see `build_msvc_vision.bat`): **`opencv4:x86-windows`** with separate libs: `opencv_core`, `opencv_imgproc`, `opencv_imgcodecs`, `opencv_videoio`, `opencv_highgui` (debug `...4d.lib` variants match the batch file’s `/MTd` + dynamic CRT linking trick).

### 2.3 Live display (`USE_IMAGE_VIEW`)

- **`image_transfer.h` / `image_transfer.lib`**: shared-memory transfer to **Week7-style `image_view.exe`** (must match **32-bit x86** toolchain; **not** linkable from MinGW per comments in `program.cpp`).
- Additional MSVC link libs in batch: DirectShow / media foundation stack (`strmbasd.lib`, `mfplat.lib`, etc.) as needed for the sample pipeline.

### 2.4 Project-local modules (no third-party beyond OpenCV)

| Module | Role |
|--------|------|
| `Parameters.{h,cpp}` | Global tunables via `params()` singleton. |
| `CoreTypes.h` | `Robot`, `Mode`, `Target`, `Image24`, `Obstacle`. |
| `Simulation.{h,cpp}` | BMP load/save, blit, rotate, `writeFrame` → `output.bmp`. |
| `Safety.{h,cpp}` | Arena polygon, `predicted_pose_safe`, `steer_avoid`, LOS helpers. |
| `Strategy.{h,cpp}` | Attack, defend, return-home, turret slew helper. |
| `RobotIO.{h,cpp}` | `CmdPacket`, XOR checksum, `vw_to_lr`, `sendRobotCommand`. |
| `DebugOverlay.{h,cpp}` | `AIDebug`, overlay drawing. |
| `vision/*` | `VisionSystem`, calibration dashboard, `VisionConfig.h` / HSV. |
| `map/*` | `Mode5AttackPlanner`, `GridPlanner`, `ArenaMap` (vision map attack). |
| `timer.{h,cpp}` | Used in MSVC vision build batch. |

---

## 3. Repository layout (relevant)

```
o2D_Sim/
  program.cpp           # Main loop, menu, sim + vision + robot I/O glue
  Parameters.h / Parameters.cpp
  CoreTypes.h
  Simulation.cpp / Simulation.h
  Safety.cpp / Safety.h
  Strategy.cpp / Strategy.h
  RobotIO.cpp / RobotIO.h
  DebugOverlay.cpp / DebugOverlay.h
  serial.h              # Windows serial port wrapper
  image_transfer.h      # Shared memory with image_view (MSVC)
  build_msvc_vision.bat # Full vision + image_view build (x86)
  vision/
    VisionSystem.cpp / VisionSystem.h
    VisionCalibration.cpp
    VisionConfig.h      # HSV ranges (tuned via mode 7 dashboard)
    VisionTypes.h       # GameState, poses, obstacles from vision
  map/
    Mode5AttackPlanner.cpp / .h
    GridPlanner.cpp / .h
    ArenaMap.cpp / .h
```

---

## 4. End-to-end program flow

### 4.1 Outer menu loop

The program prints a **scenario menu** and blocks until a key is pressed (`_kbhit` / `_getch` on Windows). Choices map to `Mode` + `Target` (`CoreTypes.h`):

- **`Mode`**: `USER_CONTROL`, `AI_ATTACK`, `AI_DEFEND`.
- **`Target`**: `SIMULATOR` or `ROBOT`.

After selection:

1. **Robot serial**: If `Target::ROBOT`, open `Parameters.robot_com_port` at `robot_baud`. On failure or exception, **`Target` falls back to `SIMULATOR`** (run continues without hardware).

2. **Vision (robot + `USE_VISION`)**: Prompt for **front marker color** (rear is fixed **blue** in messaging). Initialize `VisionSystem` with `camera_index`. Load or pick **four arena corners** → **homography** `warp_matrix` → rectified arena **640×480** (`warped_w` / `warped_h` in `Parameters`). Set `vision.setArenaOverride(...)` so the full warped image is the arena. **`arena_calibrated`** gates warp usage.

3. **Frame buffer**: If vision active, `Image24 frame` is resized to warped dimensions for overlays and optional `image_view`.

4. **Main scenario loop** (~60 s or until ESC): each iteration:
   - Compute `dt`, handle ESC → break to menu.
   - **Vision path**: Grab BGR → optional `cv::warpPerspective` → `VisionSystem::processFrame` → update `my_robot` / `target_robot` / `obstacles` from detections; latch spawn pose from first valid self pose.
   - **Motion**: Zero `v`,`w`, read keyboard; **`switch(mode)`** applies user or AI logic (see §6–§7).
   - **Turret**: `turret += turret_w * dt`, clamp to ±limit.
   - **Safety**: Graded OOB prediction clamp for **real robot** when applicable (see §8).
   - **Integration**: If not vision pose injection, integrate `theta`, `x`, `y` in sim/open-loop.
   - **Simulator**: Collision vs BMP obstacles + arena polygon revert.
   - **Robot**: Build `CmdPacket` → checksum → `sendRobotCommand`.
   - **Render**: Composite BMP, debug overlays, `writeFrame` / `pushFrameToImageView`.

### 4.2 Coordinate conventions

- **`Robot::theta`**: Body orientation; **forward** direction is **`theta + π/2`** (see comments in `program.cpp` / vision conversion `visionHeadingToRobotTheta`).
- **`Robot::turret`**: Radians **relative to body**, clamped to roughly ±85° (`170°` total span mapped to servo 0–180°).
- **Vision pose theta**: Converted once when injecting pose so AI, overlay, and Arduino agree.

---

## 5. How vision is used

### 5.1 Pipeline (`VisionSystem`)

Documented in `VisionSystem.h`:

1. **HSV threshold** five marker colors (black, blue, green, orange, red).
2. **Detect circles** per mask → **median diameter** → **pixels per inch** (markers ~3" diameter).
3. **Pair markers** ~9" apart: **blue + chosen front color** = **our robot**; best other pair = **enemy**; unassigned circles = **obstacles**.
4. **Pose hold**: Short dropout tolerance using previous pose timestamps.

Operator selects **front color** at scenario start (robot modes). Rear is **blue** by convention.

### 5.2 Perspective warp

Raw camera frames are warped so the physical arena maps to a **fixed rectangle** (`Parameters.warped_w` × `warped_h`). When calibrated, **`ArenaInfo`** for safety/AI is the **full warped frame polygon** (corners 0,0 — W-1,H-1).

### 5.3 Integration into `program.cpp`

- **`vision_active`**: Camera opened and pipeline running.
- **`arena_calibrated`**: Homography available (corners file or picker).
- **`latched_arena`**: Used by safety and AI **WorldEnv** for inside/outside tests.
- Self pose overwrites **`my_robot`** when valid; enemy updates **`target_robot`**; obstacles list rebuilt from vision disks (with filtering near self).

### 5.4 Vision-only menu modes (`USE_VISION`)

- **Mode 7**: `runCalibrationDashboard` — HSV tuning (`VisionConfig`), masks, no full game.
- **Mode 8**: Live camera → warp (if corners loaded) → `processFrame` → overlay → optional `image_view`; **C** re-picks corners; no robot/AI physics.

---

## 6. OpenCV usage (summary)

| Usage | Where |
|--------|--------|
| `cv::VideoCapture` | Camera frames (`VisionSystem::grabFrame`). |
| `cv::Mat`, HSV, morphology, contours | Thresholding, circle extraction. |
| `cv::warpPerspective` | Arena rectification in `program.cpp` and vision debug. |
| Drawing | Overlays for markers, arena, debug banners. |

HSV and morphological details live in **`VisionSystem.cpp`** and tunables in **`VisionConfig.h`** (edited via calibration UI).

---

## 7. Strategies: attack, defend, return-home, Mode 5

### 7.1 Shared environment: `WorldEnv` (`Safety.h`)

Contains obstacles pointer, optional other robot, **arena polygon** (`ArenaInfo`), **robot_radius**, **safety_margin**. Used for clearance, LOS (`has_clear_los` / `segment_clear`), and predicted safety.

### 7.2 `update_attack_ai` (`Strategy.cpp`) — **legacy 2D attack**

- **Debounced LOS** to vision noise (`AttackAiStability`).
- **If LOS**: Prefer **turret-only** aim when target fits turret arc; else **rotate body**. When aligned and not yet shot, marks **`shotUsed`** and hit (sim semantics).
- **If no LOS**: Samples arc positions to find exposed goal with clearance; **`steer_avoid`** for tangential approach around obstacles.

**Simulator**: Uses BMP obstacle list + synthetic enemy. **Robot + vision**: Obstacles/enemy poses come from vision when active.

### 7.3 `update_defend_ai`

Positions relative to enemy and cover/obstacles; defensive headings and retreat behaviors (see `Strategy.cpp`).

### 7.4 `update_return_home_ai`

Used **after the shot** in configurations that still use return-home (e.g. **simulator** AI attack post-shot):

- **Slews turret toward 0** rad (`slew_turret_toward_neutral_rad` — shared helper).
- Pops **breadcrumb trail** (recorded while `!shotUsed`) to retrace path.
- Final leg steers to **`start_x,start_y`** and aligns **`start_theta`**.

### 7.5 Mode 5 map planner (`Mode5AttackPlanner`, `#ifdef USE_VISION`)

Enabled only when:

```text
mode == AI_ATTACK && target == ROBOT && vision_active && arena_calibrated
```

**`tryInit` / `tickReconcile`**: Build/maintain an occupancy-style map from vision, plan waypoints, follow path, aim, fire with settle counters (`planner_fire_settle_frames` in `Parameters`). **Obstacles** for collision can come from the map module while LOS checks may still use vision disk list passed as `los_obstacles`.

If Mode 5 is not ready, code falls back to **`update_attack_ai`**.

---

## 8. Simulator vs robot: behavioral differences

| Aspect | Simulator (`Target::SIMULATOR`) | Robot (`Target::ROBOT`) |
|--------|--------------------------------|--------------------------|
| Pose source | Integrated from commanded `v,w` (+ collision revert) | Vision injects pose when active; else dead-reckoning without vision |
| Obstacles | BMP sprites + optional vision-off behavior | Vision circles (+ map obstacles in Mode 5) |
| **CmdPacket** | Not sent | Serial Bluetooth packets each frame |
| **Wheel commands** | N/A | `vw_to_lr` for AI modes, or direct **L/R** for user WASD / manual post-shot |
| **Safety clamp** | Real-time lookahead **skipped** for clamp math (latency not modeled the same way) | Graded **multi-horizon** clamp + optional edge-recover when AI clamps translation |
| **AI margin** | Often tighter (`ai_margin_other_px`) | Larger margin when **arena polygon** latched (`ai_margin_poly_px`) — real latency |

### 8.1 Bluetooth `CmdPacket` (`RobotIO.h`)

Packed binary: headers `0xAA 0x55`, `seq`, `flags`, `left`, `right` (-100…100), `turret_deg`, `laser`, checksum.

**Flags**:

- `FLAG_MODE_ATTACK` — attack semantics on firmware.
- `FLAG_MODE_DEF` — defend mode.
- `FLAG_FIRE_ARMED` — fire pulse window.

**`vw_to_lr`**: Converts simulator-style `v` (px/s) and `w` (rad/s) to differential **L/R**. **`Parameters.ai_invert_v/w`, `ai_swap_lr`** adjust only **AI** paths, not raw WASD user packets.

### 8.2 Keyboard mapping (`USER_CONTROL`)

- **Simulator**: `W/S` → `v`, `A/D` → `w`, `Q/E` → turret rate, `F` fire / hit logic vs synthetic enemy.
- **Robot**: Same **plus** **WASD mapped to `robot_L_cmd` / `robot_R_cmd`** at speed **70** (`apply_robot_wasd_lr`) for direct wheel commands.

---

## 9. Menu modes (operator reference)

| Key | Mode | Target | Summary |
|-----|------|--------|---------|
| **1** | User control | Simulator | 2D sim, keyboard drives `v,w`, turret, fire vs scripted enemy. |
| **2** | AI attack | Simulator | Legacy attack AI + post-shot **return-home** on sim. |
| **3** | AI defend | Simulator | Defensive AI vs orbiting enemy. |
| **4** | User control | Robot | WASD → L/R packets; vision pose if enabled; optional image_view. |
| **5** | AI attack | Robot | Mode 5 planner if vision+warp ready; else legacy attack AI. **Post-shot**: turret slew to neutral, then **manual WASD** (see §10). |
| **6** | AI defend | Robot | Defend AI + real hardware. |
| **7** *(vision build)* | Calibration | N/A | HSV / mask dashboard (`runCalibrationDashboard`). |
| **8** *(vision build)* | Vision debug | N/A | Live detection + overlay; optional warp; no robot loop. |
| **0** | Quit | — | Exit outer loop. |

**Note**: If COM port fails, **robot modes degrade to simulator** with a console warning.

---

## 10. Post-shot behavior (robot AI attack — implemented behavior)

Design goals that evolved in development:

1. **Post-shot WASD takeover**: On **robot + `AI_ATTACK`** after **`shotUsed`**, do **not** run full **`update_return_home_ai`** (no drive-home retracing). Match **mode 4** wheel mapping (**WASD → L/R**), and send packets like **user mode** when manual (**no `FLAG_MODE_ATTACK`** during manual phase).

2. **Wall safety**: **`Parameters.robot_user_disable_wall_clamp`** (default **true**) skips the graded OOB clamp for **robot user control** and for **post-shot manual** drive.

3. **Turret before WASD**: **`slew_turret_toward_neutral_rad`** (same rate as return-home: **3 rad/s**) runs until **`|turret| ≤ kPostShotTurretHomeTolRad` (0.02 rad)**. Debug detail strings: `post_shot_turret_return` then `manual_post_shot`.

4. **Simulator** post-shot: **`update_return_home_ai`** retained (return-home + turret slew + trail).

**Bluetooth gating**: `robot_manual_wheels` is true for mode 4 robot, or for mode 5 robot **only after** turret within tolerance; otherwise AI **`vw_to_lr`** with zero motion during turret slew.

---

## 11. Configuration hotspots (`Parameters.h`)

- **`camera_index`, `robot_com_port`, `robot_baud`**: Connectivity.
- **`warped_w/h`, `arena_w_in/h_in`**: Scale and warp output size.
- **Safety horizons/scales, `cmd_roundtrip_s`**: Real-robot OOB graded clamp.
- **`ai_margin_*`, `steer_lookahead_px`**: AI clearance.
- **`turret_servo_inverted`, `vision_flip_x/y`**: Hardware/image conventions.
- **`ai_invert_v/w`, `ai_swap_lr`**: AI-only drivetrain fixes before `vw_to_lr`.
- **`robot_user_disable_wall_clamp`**: Re-enable wall clamp for demos without logic edits.
- **Mode 5 planner**: `planner_*`, `planner_fire_settle_frames`, etc.

Vision HSV lives in **`VisionConfig.h`** (not `Parameters.h`).

---

## 12. Build recipes

- **MinGW (no vision, no image_view)**:  
  `g++ -O2 -std=c++17 program.cpp Simulation.cpp ... -o program.exe -luser32`  
  (Exact object list depends on minimal vs full project; the repo batch `build_g++.bat` may list sources.)

- **MSVC full vision + image_view**: Run **`build_msvc_vision.bat`** from **x86 Native Tools** command prompt; requires vcpkg OpenCV **x86** and correct CRT linking as scripted.

---

## 13. Issues fixed / lessons learned (project history summary)

This section aggregates problems encountered and addressed during development (including agent-assisted refactors), so future work does not repeat them.

1. **Post-shot control on real robot**: Originally **`update_return_home_ai`** ran after every shot, causing unwanted drive-home on hardware. **Fix**: On **robot + AI attack + post-shot**, skip return-home; use **WASD L/R** like mode 4; Bluetooth **manual** path without attack flag when driving manually.

2. **Operator wall clamp on robot**: The universal OOB clamp scaled **user** WASD near walls. **Fix**: Optional skip via **`robot_user_disable_wall_clamp`** for robot **USER_CONTROL** and **post-shot manual** (`AI_ATTACK` + `shotUsed`).

3. **`FLAG_MODE_ATTACK` during manual**: Firmware must not stay in “attack” when the operator drives manually. **Fix**: Clear attack flag when **`robot_manual_wheels`** is true (post-shot manual included).

4. **Simulator vs robot post-shot**: Plan explicitly kept **simulator** on **`update_return_home_ai`** for minimal behavior change while robot got manual drive.

5. **Safety prediction for optional re-enabled clamp**: When clamp is on, **post-shot manual** must map **L/R → check_v/check_w** like user mode for consistent prediction.

6. **Turret left off-straight after shot**: Skipping return-home removed automatic **turret centering**. **Fix**: Reuse **`slew_turret_toward_neutral_rad`** (extracted from **`update_return_home_ai`**) until within tolerance, **then** enable WASD and manual BT path.

7. **Toolchain / linking**: **`image_transfer.lib`** is **MSVC x86** only; MinGW builds omit **`USE_IMAGE_VIEW`**. OpenCV on Windows often installed per-architecture (**x86** vs **x64**); mismatch causes link/runtime failures.

8. **Parameter tuning**: AI margins, safety margins, and planner thresholds were iterated (comments in `Parameters.h` show several “was X → now Y” notes); vision noise required **LOS debouncing** and **Mode 5 fire settle** frames to avoid instant shot/return-home transitions.

9. **Vision / spawn latch**: **`start_x/y/theta`** re-latch from first valid vision self pose so return-home and AI reference the real spawn in camera coordinates.

---

## 14. Recreating the project (checklist)

1. Install **Visual Studio** C++ tools and/or **MinGW**; for vision use **vcpkg OpenCV x86** as in `build_msvc_vision.bat`.
2. Place BMP assets (`background.bmp`, robots, obstacles) in the working directory.
3. Configure **`Parameters.h`**: COM port, camera index, arena size, safety.
4. Build with **`USE_VISION`** / **`USE_IMAGE_VIEW`** as needed.
5. For live display, run **`image_view.exe`** before the app when using shared memory.
6. Run **`program.exe`**, pick menu mode; for robot modes, ensure markers and arena corners match the warp assumptions.

---

*Generated as project documentation for Industrial Automation (MECH6631) o2D_Sim. Update this file when architecture or behavior changes.*

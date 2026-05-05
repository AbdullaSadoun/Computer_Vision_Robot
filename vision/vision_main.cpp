/*
vision/vision_main.cpp
- Mode 9: legacy vision debug loop — live camera feed with channel-diff mask tuning, crosshair, and detection overlay
- Mode A: robot AI attack with legacy vision — BFS path planner, serial control, perspective warp
- Mode B: robot AI defend with legacy vision — serial control, no path planner
- runMode9: entry point for '9' (no serial, calibration + detection visualization only)
- runModeAB: entry point for 'A'/'B' (full robot AI + serial + legacy vision pipeline)
by: Abdulla Sadoun
Date: March 25, 2026
*/

#ifdef USE_LEGACY_VISION

#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include <conio.h>
#include <windows.h>

#include "../Parameters.h"
#include "../CoreTypes.h"
#include "../Simulation/Safety.h"
#include "../robot_command/RobotIO.h"
#include "../Simulation/Simulation.h"
#include "../Strategy/Strategy.h"
#include "../Simulation/DebugOverlay.h"
#include "VisionSystemLegacy.h"
#include "vision_helpers.h"
#include "LegacyVisionConfig.h"

static inline int kWarpedFrameW() { return params().warped_w; }
static inline int kWarpedFrameH() { return params().warped_h; }
static inline const char* kArenaCornersFile() { return params().arena_corners_file; }
static inline int kCameraIndex() { return params().camera_index; }
static constexpr float kPostShotTurretHomeTolRad = 0.02f;

void runMode9() {
    /*
    Legacy vision debug: live camera feed with per-color threshold tuning (Q/A), min-area tuning
    (W/S), color selector (1-5), corner re-pick (C), 180° flip toggle (R), and crosshair (arrows).
    No serial, no AI — purely for calibrating the channel-diff pipeline before using modes A/B.
    */
    activate_vision();
    const int W = params().warped_w;
    const int H = params().warped_h;
    image lv9_rgb{};
    lv9_rgb.type   = RGB_IMAGE;
    lv9_rgb.width  = (i2byte)W;
    lv9_rgb.height = (i2byte)H;
    if (allocate_image(lv9_rgb) != 0) {
        std::cerr << "allocate_image failed.\n";
        deactivate_vision();
        return;
    }

    VisionSystemLegacy lv9;
    lv9.config().camWidth  = W;
    lv9.config().camHeight = H;
    if (!lv9.initialize(kCameraIndex())) {
        std::cerr << "Camera open failed (mode 9). Plug in the overhead webcam.\n";
        free_image(lv9_rgb);
        deactivate_vision();
        return;
    }

    // Load arena corners if available.
    {
        FILE* f = fopen(kArenaCornersFile(), "r");
        if (f) {
            ArenaBoundary ab{}; bool ok = true;
            for (int k = 0; k < 4 && ok; ++k) {
                float x = 0.0f, y = 0.0f;
                if (fscanf(f, "%f %f", &x, &y) == 2) { ab.corners[k].x = x; ab.corners[k].y = y; }
                else ok = false;
            }
            fclose(f);
            if (ok) { ab.valid = true; lv9.setArenaOverride(ab); std::cout << "[mode9] Arena loaded.\n"; }
        }
    }

    // Front color prompt.
    MarkerColor lv9_front = MarkerColor::Red;
    std::cout << "\n[Mode 9] Front marker color? [b]lack/[g]reen/[o]range/[r]ed: " << std::flush;
    while (true) {
        if (_kbhit()) {
            char c = (char)std::tolower(_getch());
            switch (c) {
                case 'b': lv9_front = MarkerColor::Black;  break;
                case 'g': lv9_front = MarkerColor::Green;  break;
                case 'o': lv9_front = MarkerColor::Orange; break;
                case 'r': lv9_front = MarkerColor::Red;    break;
                default:  continue;
            }
            std::cout << markerColorName(lv9_front) << "\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    lv9.setSelfFrontColor(lv9_front);

    // Load persistent camera flip state.
    bool lv9_flip_x = false, lv9_flip_y = false;
    {
        FILE* ff = fopen("legacy_flip.txt", "r");
        if (ff) { int fx=0,fy=0; if(fscanf(ff,"%d %d",&fx,&fy)==2){lv9_flip_x=(fx!=0);lv9_flip_y=(fy!=0);} fclose(ff); }
        if (lv9_flip_x||lv9_flip_y) std::cout<<"[mode9] Camera flip loaded: x="<<lv9_flip_x<<" y="<<lv9_flip_y<<"\n";
    }

    std::cout << "[Mode 9] 1-5=color  Q/A=thresh  W/S=minArea  C=corners  R=rotate180  Arrows=crosshair  ESC=menu\n";

    MarkerColor lv9_activeColor = MarkerColor::Red;
    bool lv9_k1p=false,lv9_k2p=false,lv9_k3p=false,lv9_k4p=false,lv9_k5p=false;
    bool lv9_kQp=false,lv9_kAp=false,lv9_kWp=false,lv9_kSp=false,lv9_kCp=false,lv9_kRp=false;
    int  lv9_cx = W/2, lv9_cy = H/2;
    int  lv9_prev_cx = -1, lv9_prev_cy = -1;
    auto lv9_last_print = std::chrono::steady_clock::now();
    int  lv9_fc = 0;

    while (true) {
        if (GetAsyncKeyState(VK_ESCAPE) & 0x8000) break;

        bool k1=(GetAsyncKeyState('1')&0x8000)!=0, k2=(GetAsyncKeyState('2')&0x8000)!=0;
        bool k3=(GetAsyncKeyState('3')&0x8000)!=0, k4=(GetAsyncKeyState('4')&0x8000)!=0;
        bool k5=(GetAsyncKeyState('5')&0x8000)!=0;
        if (k1&&!lv9_k1p) { lv9_activeColor=MarkerColor::Black;  std::cout<<"Active: black\n"; }
        if (k2&&!lv9_k2p) { lv9_activeColor=MarkerColor::Blue;   std::cout<<"Active: blue\n"; }
        if (k3&&!lv9_k3p) { lv9_activeColor=MarkerColor::Green;  std::cout<<"Active: green\n"; }
        if (k4&&!lv9_k4p) { lv9_activeColor=MarkerColor::Orange; std::cout<<"Active: orange\n"; }
        if (k5&&!lv9_k5p) { lv9_activeColor=MarkerColor::Red;    std::cout<<"Active: red\n"; }
        lv9_k1p=k1; lv9_k2p=k2; lv9_k3p=k3; lv9_k4p=k4; lv9_k5p=k5;

        LegacyColorConfig* lv9cfgp = nullptr;
        switch (lv9_activeColor) {
            case MarkerColor::Black:  lv9cfgp = &lv9.config().black;  break;
            case MarkerColor::Blue:   lv9cfgp = &lv9.config().blue;   break;
            case MarkerColor::Green:  lv9cfgp = &lv9.config().green;  break;
            case MarkerColor::Orange: lv9cfgp = &lv9.config().orange; break;
            default:                  lv9cfgp = &lv9.config().red;    break;
        }
        LegacyColorConfig& lv9cfg = *lv9cfgp;

        bool kQ=(GetAsyncKeyState('Q')&0x8000)!=0, kAk=(GetAsyncKeyState('A')&0x8000)!=0;
        bool kW=(GetAsyncKeyState('W')&0x8000)!=0, kSk=(GetAsyncKeyState('S')&0x8000)!=0;
        if (kQ&&!lv9_kQp) { lv9cfg.channelThreshold=std::min(250,lv9cfg.channelThreshold+5); std::cout<<markerColorName(lv9_activeColor)<<" thresh="<<lv9cfg.channelThreshold<<"\n"; }
        if (kAk&&!lv9_kAp){ lv9cfg.channelThreshold=std::max(5,  lv9cfg.channelThreshold-5); std::cout<<markerColorName(lv9_activeColor)<<" thresh="<<lv9cfg.channelThreshold<<"\n"; }
        if (kW&&!lv9_kWp) { lv9cfg.minAreaPx+=20; std::cout<<markerColorName(lv9_activeColor)<<" minArea="<<lv9cfg.minAreaPx<<"\n"; }
        if (kSk&&!lv9_kSp){ lv9cfg.minAreaPx=std::max(10,lv9cfg.minAreaPx-20); std::cout<<markerColorName(lv9_activeColor)<<" minArea="<<lv9cfg.minAreaPx<<"\n"; }
        lv9_kQp=kQ; lv9_kAp=kAk; lv9_kWp=kW; lv9_kSp=kSk;

        bool kC=(GetAsyncKeyState('C')&0x8000)!=0;
        if (kC&&!lv9_kCp) {
            std::cout<<"\nEnter 4 corners TL TR BR BL (x y per line):\n";
            ArenaBoundary ab{}; bool ok=true;
            for (int k=0; k<4&&ok; ++k) {
                float x=0,y=0; std::cout<<"  corner "<<k+1<<": "<<std::flush;
                if (std::scanf("%f %f",&x,&y)==2) { ab.corners[k].x=x; ab.corners[k].y=y; } else ok=false;
            }
            if (ok) {
                ab.valid=true; lv9.setArenaOverride(ab);
                FILE* f=fopen(kArenaCornersFile(),"w");
                if (f) { for(int k=0;k<4;++k) fprintf(f,"%.3f %.3f\n",ab.corners[k].x,ab.corners[k].y); fclose(f); std::cout<<"Saved.\n"; }
            }
        }
        lv9_kCp=kC;

        bool kR=(GetAsyncKeyState('R')&0x8000)!=0;
        if (kR&&!lv9_kRp) {
            lv9_flip_x=!lv9_flip_x; lv9_flip_y=!lv9_flip_y;
            FILE* ff=fopen("legacy_flip.txt","w");
            if (ff) { fprintf(ff,"%d %d\n",(int)lv9_flip_x,(int)lv9_flip_y); fclose(ff); }
            std::cout<<"[mode9] 180° rotate "<<(lv9_flip_x?"ON":"OFF")<<" — re-pick corners if already set.\n";
        }
        lv9_kRp=kR;

        {
            int step = (GetAsyncKeyState(VK_SHIFT)&0x8000) ? 10 : 1;
            if (GetAsyncKeyState(VK_LEFT) &0x8000) lv9_cx=std::max(0,   lv9_cx-step);
            if (GetAsyncKeyState(VK_RIGHT)&0x8000) lv9_cx=std::min(W-1, lv9_cx+step);
            if (GetAsyncKeyState(VK_UP)   &0x8000) lv9_cy=std::max(0,   lv9_cy-step);
            if (GetAsyncKeyState(VK_DOWN) &0x8000) lv9_cy=std::min(H-1, lv9_cy+step);
            if (lv9_cx!=lv9_prev_cx||lv9_cy!=lv9_prev_cy) {
                std::cout<<"  Crosshair: x="<<lv9_cx<<"  y="<<lv9_cy<<"\n";
                lv9_prev_cx=lv9_cx; lv9_prev_cy=lv9_cy;
            }
        }

        if (!lv9.grabFrame()) { std::this_thread::sleep_for(std::chrono::milliseconds(33)); continue; }
        if (lv9_flip_x||lv9_flip_y) flipImageLegacy(lv9.getCurrentFrame(), lv9_flip_x, lv9_flip_y);
        GameState gs9 = lv9.processFrame();
        drawLegacyVisionOverlay(lv9.getCurrentFrame(), gs9);
        legacyDrawLine(lv9.getCurrentFrame(), 0, lv9_cy, W-1, lv9_cy, 255,255,0);
        legacyDrawLine(lv9.getCurrentFrame(), lv9_cx, 0, lv9_cx, H-1, 255,255,0);
        legacyDrawFilledCircle(lv9.getCurrentFrame(), lv9_cx, lv9_cy, 3, 255,0,0);

        if (lv9_rgb.pdata) {
            const size_t rb=(size_t)W*3;
            ibyte* dp=lv9_rgb.pdata, *sp=lv9.getCurrentFrame().pdata;
            for(int y=0;y<H;++y) memcpy(dp+(size_t)y*rb, sp+(size_t)(H-1-y)*rb, rb);
            view_rgb_image(lv9_rgb,1);
        }

        ++lv9_fc;
        auto lv9_now=std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(lv9_now-lv9_last_print).count()>=1.0f) {
            std::cout<<"  [legacy9] self:"<<(gs9.self.pose.valid?"OK":"--")
                     <<"  enemy:"<<(gs9.enemy.pose.valid?"OK":"--")
                     <<"  obs:"<<gs9.obstacles.size()<<"  fps~"<<lv9_fc<<"\n";
            lv9_last_print=lv9_now; lv9_fc=0;
        }
    }

    std::cout << "\n[mode 9] Stopping...\n";
    lv9.shutdown();
    free_image(lv9_rgb);
    deactivate_vision();
    std::cout << "[mode 9] Stopped. Returning to menu.\n";
}

void runModeAB(char choice) {
    /*
    Full robot AI loop using the legacy vision pipeline (no OpenCV). Grabs frames, applies
    optional flip and perspective warp, runs detection, runs AI (attack with BFS path planner
    or defend), applies safety clamp, builds and sends serial CmdPacket each frame.
    */
    const Mode lv_mode = (choice == 'A') ? Mode::AI_ATTACK : Mode::AI_DEFEND;

    Serial* lv_port = nullptr;
    try {
        lv_port = new Serial(params().robot_com_port, params().robot_baud);
        if (!lv_port->is_open()) {
            std::cerr << "Robot port not open: " << params().robot_com_port << ". Aborting.\n";
            delete lv_port; return;
        }
        std::cout << "Connected to robot on " << params().robot_com_port << ".\n";
    } catch (const std::exception& e) {
        std::cerr << "Serial error: " << e.what() << "\n"; return;
    }

    // Front color prompt.
    MarkerColor lv_front = MarkerColor::Red;
    std::cout << "\nOur robot REAR=BLUE. Front marker? [b]lack/[g]reen/[o]range/[r]ed: " << std::flush;
    while (true) {
        if (_kbhit()) {
            char c = (char)std::tolower(_getch());
            switch (c) {
                case 'b': lv_front = MarkerColor::Black;  break;
                case 'g': lv_front = MarkerColor::Green;  break;
                case 'o': lv_front = MarkerColor::Orange; break;
                case 'r': lv_front = MarkerColor::Red;    break;
                default:  continue;
            }
            std::cout << markerColorName(lv_front) << "\n"; break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Load persistent camera flip state (set via Mode 9).
    bool lv_flip_x = false, lv_flip_y = false;
    {
        FILE* ff = fopen("legacy_flip.txt", "r");
        if (ff) { int fx=0,fy=0; if(fscanf(ff,"%d %d",&fx,&fy)==2){lv_flip_x=(fx!=0);lv_flip_y=(fy!=0);} fclose(ff); }
        if (lv_flip_x||lv_flip_y) std::cout<<"[mode "<<choice<<"] Camera flip: x="<<lv_flip_x<<" y="<<lv_flip_y<<"\n";
    }

    activate_vision();
    const int lv_W = params().warped_w;
    const int lv_H = params().warped_h;
    image lv_disp{};
    lv_disp.type=RGB_IMAGE; lv_disp.width=(i2byte)lv_W; lv_disp.height=(i2byte)lv_H;
    if (allocate_image(lv_disp) != 0) {
        std::cerr << "allocate_image failed.\n"; deactivate_vision();
        delete lv_port; return;
    }

    VisionSystemLegacy lv;
    lv.config().camWidth=lv_W; lv.config().camHeight=lv_H;
    lv.setSelfFrontColor(lv_front);
    if (!lv.initialize(kCameraIndex())) {
        std::cerr << "Camera open failed (mode A/B).\n";
        free_image(lv_disp); deactivate_vision(); delete lv_port; return;
    }

    // Load arena corners and build homography.
    float lv_Hmat[9]{};
    bool  lv_warp_ok = false;
    ArenaBoundary lv_arena_ab{};
    {
        FILE* f = fopen(kArenaCornersFile(), "r");
        if (f) {
            float s4[4][2]{}, d4[4][2]{};
            bool ok=true;
            for (int k=0; k<4&&ok; ++k)
                if (fscanf(f,"%f %f",&s4[k][0],&s4[k][1])!=2) ok=false;
            fclose(f);
            if (ok) {
                d4[0][0]=0;              d4[0][1]=0;
                d4[1][0]=(float)lv_W-1; d4[1][1]=0;
                d4[2][0]=(float)lv_W-1; d4[2][1]=(float)lv_H-1;
                d4[3][0]=0;              d4[3][1]=(float)lv_H-1;
                if (computeHomography(s4, d4, lv_Hmat)) {
                    lv_warp_ok = true;
                    lv_arena_ab.corners[0]={0.0f,0.0f};
                    lv_arena_ab.corners[1]={(float)(lv_W-1),0.0f};
                    lv_arena_ab.corners[2]={(float)(lv_W-1),(float)(lv_H-1)};
                    lv_arena_ab.corners[3]={0.0f,(float)(lv_H-1)};
                    lv_arena_ab.valid=true; lv.setArenaOverride(lv_arena_ab);
                    std::cout << "Arena warp ready.\n";
                }
            }
        }
    }
    ArenaInfo lv_ai{};
    lv_ai.has_polygon=lv_arena_ab.valid; lv_ai.img_w=lv_W; lv_ai.img_h=lv_H;
    if (lv_arena_ab.valid)
        for (int k=0;k<4;++k) { lv_ai.corners[k][0]=lv_arena_ab.corners[k].x; lv_ai.corners[k][1]=lv_arena_ab.corners[k].y; }

    // Pre-allocate warped frame image.
    image lv_warped{};
    lv_warped.type=RGB_IMAGE; lv_warped.width=(i2byte)lv_W; lv_warped.height=(i2byte)lv_H;
    lv_warped.pdata=(ibyte*)malloc((size_t)lv_W*lv_H*3);
    if (lv_warped.pdata) memset(lv_warped.pdata,0,(size_t)lv_W*lv_H*3);

    // Robot state.
    Robot lv_my{}, lv_tgt{};
    lv_my.x=(float)lv_W*0.5f; lv_my.y=(float)lv_H*0.85f; lv_my.theta=-3.1415926f/2.0f;
    lv_tgt.x=(float)lv_W*0.5f; lv_tgt.y=(float)lv_H*0.15f; lv_tgt.is_alive=true;
    float lv_sx=lv_my.x, lv_sy=lv_my.y, lv_sth=lv_my.theta;
    bool  lv_start_latched=false;

    std::vector<Obstacle> lv_obs;
    bool   lv_fired=false, lv_hit=false;
    float  lv_t=0.0f, lv_fire_time=-1.0f, lv_hit_time=-1.0f;
    std::vector<std::pair<float,float>> lv_trail;
    uint8_t lv_seq=0;
    AttackAiStability lv_stab{};
    LegacyWaypoints   lv_path{};
    int lv_path_frame=0;
    const float lv_limit = 170.0f*3.1415926f/180.0f/2.0f;

    auto lv_t0=std::chrono::steady_clock::now(), lv_lp=lv_t0;
    int lv_fc=0;
    bool lv_prev_fired=false;
    std::cout << "[Mode " << choice << "] Running. ESC=stop.\n";
    std::cout << "  DBG: self default pos=(" << lv_my.x << "," << lv_my.y
              << ") tgt default pos=(" << lv_tgt.x << "," << lv_tgt.y << ")\n";
    std::cout << "  DBG: flip_x=" << lv_flip_x << " flip_y=" << lv_flip_y
              << " warp=" << lv_warp_ok << " pairGate=["
              << lv.config().pairMinPx << "," << lv.config().pairMaxPx << "]\n";

    while (true) {
        if (GetAsyncKeyState(VK_ESCAPE)&0x8000) break;

        auto lv_now=std::chrono::steady_clock::now();
        float lv_dt=std::min(0.05f, std::chrono::duration<float>(lv_now-lv_t0).count());
        lv_t0=lv_now; lv_t+=lv_dt;

        bool lv_active=lv.grabFrame();
        if (lv_active && (lv_flip_x||lv_flip_y))
            flipImageLegacy(lv.getCurrentFrame(), lv_flip_x, lv_flip_y);
        if (lv_active && lv_warp_ok && lv_warped.pdata) {
            warpImageLegacy(lv.getCurrentFrame(), lv_warped, lv_Hmat);
            lv.loadFrame(lv_warped);
        }

        GameState lv_gs{};
        bool lv_sv=false, lv_ev=false;
        if (lv_active) {
            lv_gs=lv.processFrame();
            if (lv_gs.self.pose.valid) {
                lv_my.x=lv_gs.self.pose.x; lv_my.y=lv_gs.self.pose.y;
                lv_my.theta=visionHeadingToRobotTheta(lv_gs.self.pose.theta); lv_sv=true;
                if (!lv_start_latched) { lv_sx=lv_my.x; lv_sy=lv_my.y; lv_sth=lv_my.theta; lv_start_latched=true; }
            }
            if (lv_gs.enemy.pose.valid) {
                lv_tgt.x=lv_gs.enemy.pose.x; lv_tgt.y=lv_gs.enemy.pose.y;
                lv_tgt.theta=visionHeadingToRobotTheta(lv_gs.enemy.pose.theta); lv_tgt.is_alive=true; lv_ev=true;
            }
            lv_obs.clear();
            const float sr=params().robot_radius_px;
            for (const auto& vo : lv_gs.obstacles) {
                if (!vo.valid) continue;
                if (lv_sv) { float dx=vo.x-lv_my.x, dy=vo.y-lv_my.y, cut=sr+vo.radius+6.0f; if (dx*dx+dy*dy<cut*cut) continue; }
                Obstacle o{}; o.x=(int)vo.x; o.y=(int)vo.y; o.img=nullptr; o.radius=vo.radius; lv_obs.push_back(o);
            }
        }

        // AI motion.
        lv_my.v=0; lv_my.w=0;
        AIDebug lv_dbg{};
        WorldEnv lv_env{};
        lv_env.obstacles=&lv_obs; lv_env.other_robot=lv_tgt.is_alive?&lv_tgt:nullptr;
        lv_env.robot_radius=params().robot_radius_px;
        lv_env.safety_margin=lv_ai.has_polygon?params().ai_margin_poly_px:params().ai_margin_other_px;
        lv_env.arena=lv_ai;

        if (!lv_my.shotUsed && lv_start_latched) {
            if (lv_mode==Mode::AI_ATTACK) {
                ++lv_path_frame;
                if (lv_path_frame%30==0 || !lv_path.valid)
                    lv_path=planPathLegacy(lv_my.x,lv_my.y,lv_tgt.x,lv_tgt.y,lv_obs,lv_ai,
                                           params().robot_radius_px,params().ai_margin_poly_px,lv_W,lv_H);
                update_attack_ai(lv_my, lv_tgt, lv_obs, lv_env, lv_dt, lv_fired, lv_t, lv_fire_time, lv_hit, lv_hit_time, &lv_dbg, &lv_stab);
                if (lv_fired && !lv_prev_fired)
                    std::cout << "  [FIRE] t=" << lv_t << " self_valid=" << (lv_sv?"Y":"N")
                              << " self=(" << lv_my.x << "," << lv_my.y << " th=" << lv_my.theta << ")"
                              << " tgt=(" << lv_tgt.x << "," << lv_tgt.y << ")"
                              << " AI_state=" << (int)lv_dbg.state << " detail=" << lv_dbg.detail << "\n";
            } else {
                update_defend_ai(lv_my, lv_tgt, lv_obs, lv_env, lv_dt, &lv_dbg);
            }
        } else {
            if (lv_mode==Mode::AI_ATTACK) {
                if (std::fabs(lv_my.turret)>kPostShotTurretHomeTolRad)
                    slew_turret_toward_neutral_rad(lv_my.turret, lv_dt);
                else
                    update_return_home_ai(lv_my, lv_sx, lv_sy, lv_sth, lv_trail, lv_obs, lv_dt, &lv_dbg);
            }
        }
        if (lv_hit) lv_tgt.is_alive=false;

        // Safety clamp.
        {
            WorldEnv se{}; se.obstacles=nullptr; se.other_robot=nullptr;
            se.robot_radius=params().robot_radius_px; se.arena=lv_ai;
            se.safety_margin=lv_ai.has_polygon?params().safety_margin_poly_px:params().safety_margin_fallback_px;
            const float coast=std::fabs(lv_my.v)*(params().cmd_roundtrip_s+params().brake_margin_s);
            se.safety_margin+=coast;
            if (!predicted_pose_safe(lv_my,lv_my.v,lv_my.w,params().safety_horizons_s[0],se)) {
                lv_my.v=0.0f;
                if (!predicted_pose_safe(lv_my,0.0f,lv_my.w,params().safety_horizons_s[0],se)) lv_my.w=0.0f;
            }
        }

        // Overlays + display.
        drawLegacyVisionOverlay(lv.getCurrentFrame(), lv_gs, lv_my.turret);
        drawLegacyDebugOverlay(lv.getCurrentFrame(), lv_gs, lv_env, lv_dbg, lv_mode==Mode::AI_ATTACK?&lv_path:nullptr);
        if (lv_disp.pdata) {
            const size_t rb=(size_t)lv_W*3;
            ibyte* dp=lv_disp.pdata, *sp=lv.getCurrentFrame().pdata;
            for (int y=0;y<lv_H;++y) memcpy(dp+(size_t)y*rb, sp+(size_t)(lv_H-1-y)*rb, rb);
            view_rgb_image(lv_disp,1);
        }

        // Serial packet.
        if (lv_port && lv_port->is_open()) {
            CmdPacket cmd{};
            cmd.seq=lv_seq++;
            cmd.flags=0;
            if (lv_mode==Mode::AI_ATTACK) cmd.flags|=FLAG_MODE_ATTACK;
            if (lv_mode==Mode::AI_DEFEND) cmd.flags|=FLAG_MODE_DEF;
            float vc=lv_my.v; if(params().lv_invert_v) vc=-vc;
            float wc=lv_my.w; if(params().lv_invert_w) wc=-wc;
            vw_to_lr(vc, wc, cmd.left, cmd.right);
            if (params().ai_swap_lr) { int8_t tmp=cmd.left; cmd.left=cmd.right; cmd.right=tmp; }
            float lv_turret=lv_my.turret; if(params().lv_invert_turret) lv_turret=-lv_turret;
            float td=(lv_turret/lv_limit)*90.0f+90.0f;
            cmd.turret_deg=(uint8_t)clampi((int)td,0,180);
            if (params().turret_servo_inverted) cmd.turret_deg=(uint8_t)(180-cmd.turret_deg);
            cmd.laser=0;
            if (lv_fired && lv_t-lv_fire_time<0.5f) { cmd.flags|=FLAG_FIRE_ARMED; cmd.laser=1; }
            cmd.chk=checksum_xor(cmd);
            sendRobotCommand(*lv_port, cmd);
        }

        // Breadcrumb trail for return-home.
        if (!lv_my.shotUsed) {
            const float ms=15.0f*15.0f;
            if (lv_trail.empty()) { lv_trail.push_back({lv_my.x,lv_my.y}); }
            else { float bx=lv_trail.back().first,by=lv_trail.back().second,ddx=lv_my.x-bx,ddy=lv_my.y-by; if(ddx*ddx+ddy*ddy>=ms) lv_trail.push_back({lv_my.x,lv_my.y}); }
        }

        lv_prev_fired = lv_fired;

        ++lv_fc;
        if (std::chrono::duration<float>(lv_now-lv_lp).count()>=1.0f) {
            std::cout<<"  [lv-"<<choice<<"] self:"<<(lv_sv?"OK":"--")<<" enemy:"<<(lv_ev?"OK":"--")
                     <<" obs:"<<lv_obs.size()<<" fired:"<<(lv_fired?"Y":"N")<<" shot:"<<(lv_my.shotUsed?"Y":"N")
                     <<" fps~"<<lv_fc<<"\n";
            std::cout<<"    my:("<<(int)lv_my.x<<","<<(int)lv_my.y<<" th="<<lv_my.theta<<")"
                     <<" tgt:("<<(int)lv_tgt.x<<","<<(int)lv_tgt.y<<")"
                     <<" AI:"<<(int)lv_dbg.state<<" "<<lv_dbg.detail<<"\n";
            lv_lp=lv_now; lv_fc=0;
        }
    }

    std::cout << "\n[mode " << choice << "] Stopping...\n";
    try {
        if (lv_port && lv_port->is_open()) {
            CmdPacket stop{}; stop.seq=lv_seq++; stop.chk=checksum_xor(stop);
            sendRobotCommand(*lv_port, stop);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } catch (...) {}
    if (lv_port) { try { delete lv_port; } catch (...) {} lv_port=nullptr; }
    if (lv_warped.pdata) { free(lv_warped.pdata); lv_warped.pdata=nullptr; }
    lv.shutdown();
    free_image(lv_disp);
    deactivate_vision();
    std::cout << "[mode " << choice << "] Stopped. Returning to menu.\n";
}

#endif // USE_LEGACY_VISION

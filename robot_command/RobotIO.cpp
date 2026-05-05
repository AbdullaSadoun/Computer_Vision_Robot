/*
RobotIO.cpp
- Implements the serial packet checksum and transmission for physical robot control
- checksum_xor: XORs all CmdPacket payload fields (seq through reserved) for integrity
- sendRobotCommand: raw byte write of CmdPacket to the Serial port (Windows only)
- vw_to_lr: normalizes v to [-1,1] and applies differential-drive kinematics to get L/R percentages
- apply_robot_wasd_lr: polls GetAsyncKeyState for WASD and sets L/R wheel speed bytes
by: Abdulla Sadoun
Date: March 1, 2026
*/
// RobotIO.cpp
#include "RobotIO.h"

#include "../Simulation/Simulation.h" // clampf

#ifdef _WIN32
#include <windows.h>
#endif
 
uint8_t checksum_xor(const CmdPacket& c) {
  /*
  XORs the 7 payload bytes (seq, flags, left, right, turret_deg, laser, reserved) together.
  Arduino firmware performs the same XOR on receipt to detect corrupted Bluetooth frames.
  */
  uint8_t x = 0;
  x ^= c.seq;
  x ^= c.flags;
  x ^= (uint8_t)c.left;
  x ^= (uint8_t)c.right;
  x ^= c.turret_deg;
  x ^= c.laser;
  x ^= c.reserved;
  return x;
}
 
#ifdef _WIN32
void sendRobotCommand(Serial& port, const CmdPacket& cmd) {
  /*
  Writes the full packed CmdPacket (sizeof(cmd) bytes) to the serial port in one call.
  The HC-05 Bluetooth module relays it to the Arduino which parses the 0xAA 0x55 header.
  */
  port.write((const char*)&cmd, sizeof(cmd));
}
#endif
 
void vw_to_lr(float v, float w, int8_t& L, int8_t& R) {
  /*
  Converts unicycle (v px/s, w rad/s) to differential-drive L/R wheel speeds in [-100, 100].
  Uses a fixed max_speed=150 and wheel_base=50 px; clamps output to int8 range.
  */
  const float max_speed = 150.0f; // px/s, should roughly match AI speed
  const float wheel_base = 50.0f; // px, distance between wheels
 
  float v_mps = v / max_speed; // normalize to -1..1
  float w_radps = w;
 
  float v_left = v_mps - (w_radps * wheel_base / 2.0f) / max_speed;
  float v_right = v_mps + (w_radps * wheel_base / 2.0f) / max_speed;
 
  L = (int8_t)clampf(v_left * 100.0f, -100.0f, 100.0f);
  R = (int8_t)clampf(v_right * 100.0f, -100.0f, 100.0f);
}

#ifdef _WIN32
void apply_robot_wasd_lr(int8_t& robot_L_cmd, int8_t& robot_R_cmd) {
  /*
  Maps WASD keys to left/right wheel speed bytes for real-robot manual control.
  W=forward, S=backward, A=left spin, D=right spin. Speed fixed at 70 of 100.
  */
  const int8_t wasd_speed = 70;
  if (GetAsyncKeyState('W') & 0x8000) { robot_L_cmd =  wasd_speed; robot_R_cmd =  wasd_speed; }
  if (GetAsyncKeyState('S') & 0x8000) { robot_L_cmd = -wasd_speed; robot_R_cmd = -wasd_speed; }
  if (GetAsyncKeyState('A') & 0x8000) { robot_L_cmd = -wasd_speed; robot_R_cmd =  wasd_speed; }
  if (GetAsyncKeyState('D') & 0x8000) { robot_L_cmd =  wasd_speed; robot_R_cmd = -wasd_speed; }
}
#endif

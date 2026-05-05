/*
RobotIO.h
- Defines the serial command packet (CmdPacket) sent to the physical robot over Bluetooth
- CmdPacket: packed 10-byte struct with header bytes, sequence, flags, L/R speeds, turret angle, laser, checksum
- Packet flag constants: FLAG_MODE_ATTACK, FLAG_MODE_DEF, FLAG_ESTOP, FLAG_FIRE_ARMED
- checksum_xor: XOR of all payload bytes for basic transmission integrity
- sendRobotCommand: writes the packet to a Windows serial port (WIN32 only)
- vw_to_lr: converts unicycle (v, w) commands to differential-drive left/right wheel percentages
- apply_robot_wasd_lr: maps WASD keys to differential left/right wheel speeds for manual robot control
- visionHeadingToRobotTheta: converts vision rear-to-front heading to Robot::theta (body forward = theta+pi/2)
by: Abdulla Sadoun
Date: March 1, 2026
*/
// RobotIO.h
#pragma once

#include <cstdint>
#include <cmath>

#ifdef _WIN32
  #include "serial.h"
#endif
 
#pragma pack(push, 1)
struct CmdPacket {
  uint8_t h1 = 0xAA;
  uint8_t h2 = 0x55;
  uint8_t seq = 0;
  uint8_t flags = 0;
  int8_t  left = 0;
  int8_t  right = 0;
  uint8_t turret_deg = 90;
  uint8_t laser = 0;
  uint8_t reserved = 0;
  uint8_t chk = 0;
};
#pragma pack(pop)
 
// Packet flags.
static constexpr uint8_t FLAG_MODE_ATTACK = (1 << 0);
static constexpr uint8_t FLAG_MODE_DEF    = (1 << 1);
static constexpr uint8_t FLAG_ESTOP       = (1 << 2);
static constexpr uint8_t FLAG_FIRE_ARMED  = (1 << 3);
 
uint8_t checksum_xor(const CmdPacket& c);
 
#ifdef _WIN32
void sendRobotCommand(Serial& port, const CmdPacket& cmd);
#endif
 
// Convert AI's v/w commands to left/right wheel speeds (-100..100).
void vw_to_lr(float v, float w, int8_t& L, int8_t& R);

// WASD keyboard → differential left/right wheel percentages for real-robot manual control.
// Non-WIN32 stub does nothing (key polling requires GetAsyncKeyState).
#ifdef _WIN32
void apply_robot_wasd_lr(int8_t& robot_L_cmd, int8_t& robot_R_cmd);
#else
inline void apply_robot_wasd_lr(int8_t& /*robot_L_cmd*/, int8_t& /*robot_R_cmd*/) {}
#endif

// Converts vision rear-to-front heading to Robot::theta.
// Vision pose.theta is the angle from rear marker to front marker in image coords.
// Robot forward direction is theta + pi/2 (matches kinematics + turret math).
inline float visionHeadingToRobotTheta(float pose_theta_rear_to_front) {
    constexpr float kPi = 3.1415926f;
    float th = pose_theta_rear_to_front - kPi / 2.0f;
    while (th >  kPi) th -= 2.0f * kPi;
    while (th < -kPi) th += 2.0f * kPi;
    return th;
}
 
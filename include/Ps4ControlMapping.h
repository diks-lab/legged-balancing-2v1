#pragma once
#include <stdint.h>
namespace ps4_control {
constexpr int DriveDeadzone=10,LegDeadzone=16,LegCenter=10;
constexpr int magnitude(int value){return value<0?-value:value;}
constexpr bool leftCentered(int x,int y){return magnitude(x)<=LegCenter&&magnitude(y)<=LegCenter;}
constexpr bool rightCentered(int x,int y){return magnitude(x)<=DriveDeadzone&&magnitude(y)<=DriveDeadzone;}
// Continuous at the deadzone boundary; signed PS4 endpoints are -128 / +127.
constexpr float legAxis(int value){
 const int v=value< -128?-128:value>127?127:value,mag=magnitude(v);
 return mag<=LegDeadzone?0.f:(v<0?-1.f:1.f)*float(mag-LegDeadzone)/float((v<0?128:127)-LegDeadzone);
}
constexpr uint8_t directions(float height,float roll){return uint8_t((height>0?1:height<0?2:0)|(roll>0?4:roll<0?8:0));}
struct StickObservation {
 uint32_t ms=0;
 float height=0,roll=0,applied_height=0,applied_roll=0,drive=0,yaw=0;
 int8_t lx=0,ly=0,rx=0,ry=0;
 uint8_t flags=0; // leg gate, left centered, right input active, drive gate, roll confirmed
};
}

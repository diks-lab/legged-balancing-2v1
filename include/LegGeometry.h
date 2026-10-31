#pragma once
#include <stdint.h>
namespace leg {
// Explicit four-point NVS adoption only. Percent is servo travel, NOT linear leg length.
struct Geometry { int16_t right_zero=0,right_full=0,left_zero=0,left_full=0; };
constexpr Geometry ConfirmedCandidate{2044,1697,2084,2408};
// Main task owns adoption. HTTP uses the Geometry copied into its Snapshot.
inline Geometry& activeGeometry(){
 static Geometry value{}; // No implicit usable calibration.
 return value;
}
constexpr int RightRetractedMarginTicks=10,RightExtendedMarginTicks=10;
constexpr int LeftRetractedMarginTicks=10,LeftExtendedMarginTicks=10;
constexpr float MinPercent=0,MaxPercent=30;
constexpr int lower(int a,int b){return a<b?a:b;}
constexpr int upper(int a,int b){return a>b?a:b;}
inline int zeroTick(bool right,const Geometry&g=activeGeometry()){return right?g.right_zero:g.left_zero;}
inline int fullTick(bool right,const Geometry&g=activeGeometry()){return right?g.right_full:g.left_full;}
inline int measuredMinTick(bool right,const Geometry&g=activeGeometry()){return lower(zeroTick(right,g),fullTick(right,g));}
inline int measuredMaxTick(bool right,const Geometry&g=activeGeometry()){return upper(zeroTick(right,g),fullTick(right,g));}
constexpr int retractMargin(bool right){return right?RightRetractedMarginTicks:LeftRetractedMarginTicks;}
constexpr int extendMargin(bool right){return right?RightExtendedMarginTicks:LeftExtendedMarginTicks;}
inline int safeMinTick(bool right,const Geometry&g=activeGeometry()){return measuredMinTick(right,g)+(right?extendMargin(right):retractMargin(right));}
inline int safeMaxTick(bool right,const Geometry&g=activeGeometry()){return measuredMaxTick(right,g)-(right?retractMargin(right):extendMargin(right));}
// Integer division rounds the 30% endpoint inward (not beyond 30%).
inline int normalEndTick(bool right,const Geometry&g=activeGeometry()){return zeroTick(right,g)+(fullTick(right,g)-zeroTick(right,g))*int(MaxPercent)/100;}
inline int normalMinTick(bool right,const Geometry&g=activeGeometry()){return upper(safeMinTick(right,g),lower(zeroTick(right,g),normalEndTick(right,g)));}
inline int normalMaxTick(bool right,const Geometry&g=activeGeometry()){return lower(safeMaxTick(right,g),upper(zeroTick(right,g),normalEndTick(right,g)));}
inline float percent(bool right,int16_t tick,const Geometry&g=activeGeometry()){return fullTick(right,g)==zeroTick(right,g)?__builtin_nanf(""):100.f*(tick-zeroTick(right,g))/(fullTick(right,g)-zeroTick(right,g));}
inline float normalLow(bool right,const Geometry&g=activeGeometry()){return percent(right,int16_t(right?normalMaxTick(right,g):normalMinTick(right,g)),g);}
inline float normalHigh(bool right,const Geometry&g=activeGeometry()){return percent(right,int16_t(right?normalMinTick(right,g):normalMaxTick(right,g)),g);}
inline bool inRange(bool right,int16_t tick,const Geometry&g=activeGeometry()){return tick>=normalMinTick(right,g)&&tick<=normalMaxTick(right,g);}
inline bool measured(bool right,int tick,const Geometry&g=activeGeometry()){return tick>=measuredMinTick(right,g)&&tick<=measuredMaxTick(right,g);}
inline bool validGeometry(const Geometry&g);
// Ordinary commands are always inside the margin AND the 0..30% window.
inline int16_t position(bool right,float pct,const Geometry&g=activeGeometry()){
 if(!validGeometry(g))return -1; // No implicit target when calibration is absent.
 const float p=pct<MinPercent?MinPercent:pct>MaxPercent?MaxPercent:pct;
 const float value=zeroTick(right,g)+(fullTick(right,g)-zeroTick(right,g))*p/100.f;
 const int rounded=int(value+.5f); // All measured ticks are positive.
 return int16_t(rounded<normalMinTick(right,g)?normalMinTick(right,g):rounded>normalMaxTick(right,g)?normalMaxTick(right,g):rounded);
}
static_assert(RightRetractedMarginTicks>0&&RightExtendedMarginTicks>0&&LeftRetractedMarginTicks>0&&LeftExtendedMarginTicks>0,"positive margins required");
inline bool validGeometry(const Geometry&g){
 return g.right_zero>=0&&g.right_zero<=4095&&g.right_full>=0&&g.right_full<g.right_zero&&
  g.left_zero>=0&&g.left_zero<=4095&&g.left_full>g.left_zero&&g.left_full<=4095&&
  normalMinTick(true,g)<normalMaxTick(true,g)&&normalMinTick(false,g)<normalMaxTick(false,g);
}
}

#pragma once
#include <stdint.h>
namespace leg {
// User-measured servo-position stroke (265 valid samples/side), NOT linear leg length.
constexpr int RightHome=2007,RightExtended=1675,LeftHome=2071,LeftExtended=2412;
constexpr int RightRetractedMarginTicks=10,RightExtendedMarginTicks=10;
constexpr int LeftRetractedMarginTicks=10,LeftExtendedMarginTicks=10;
constexpr float MinPercent=0,MaxPercent=30;
constexpr int lower(int a,int b){return a<b?a:b;}
constexpr int upper(int a,int b){return a>b?a:b;}
constexpr int zeroTick(bool right){return right?RightHome:LeftHome;}
constexpr int fullTick(bool right){return right?RightExtended:LeftExtended;}
constexpr int measuredMinTick(bool right){return lower(zeroTick(right),fullTick(right));}
constexpr int measuredMaxTick(bool right){return upper(zeroTick(right),fullTick(right));}
constexpr int retractMargin(bool right){return right?RightRetractedMarginTicks:LeftRetractedMarginTicks;}
constexpr int extendMargin(bool right){return right?RightExtendedMarginTicks:LeftExtendedMarginTicks;}
constexpr int safeMinTick(bool right){return measuredMinTick(right)+(right?extendMargin(right):retractMargin(right));}
constexpr int safeMaxTick(bool right){return measuredMaxTick(right)-(right?retractMargin(right):extendMargin(right));}
// Integer division rounds the 30% endpoint inward (not beyond 30%).
constexpr int normalEndTick(bool right){return zeroTick(right)+(fullTick(right)-zeroTick(right))*int(MaxPercent)/100;}
constexpr int normalMinTick(bool right){return upper(safeMinTick(right),lower(zeroTick(right),normalEndTick(right)));}
constexpr int normalMaxTick(bool right){return lower(safeMaxTick(right),upper(zeroTick(right),normalEndTick(right)));}
constexpr float percent(bool right,int16_t tick){return 100.f*(tick-zeroTick(right))/(fullTick(right)-zeroTick(right));}
constexpr float normalLow(bool right){return percent(right,int16_t(right?normalMaxTick(right):normalMinTick(right)));}
constexpr float normalHigh(bool right){return percent(right,int16_t(right?normalMinTick(right):normalMaxTick(right)));}
constexpr bool inRange(bool right,int16_t tick){return tick>=normalMinTick(right)&&tick<=normalMaxTick(right);}
constexpr bool measured(bool right,int tick){return tick>=measuredMinTick(right)&&tick<=measuredMaxTick(right);}
// Ordinary commands are always inside the margin AND the 0..30% window.
constexpr int16_t position(bool right,float pct){
 const float p=pct<MinPercent?MinPercent:pct>MaxPercent?MaxPercent:pct;
 const float value=zeroTick(right)+(fullTick(right)-zeroTick(right))*p/100.f;
 const int rounded=int(value+.5f); // All measured ticks are positive.
 return int16_t(rounded<normalMinTick(right)?normalMinTick(right):rounded>normalMaxTick(right)?normalMaxTick(right):rounded);
}
static_assert(RightRetractedMarginTicks>0&&RightExtendedMarginTicks>0&&LeftRetractedMarginTicks>0&&LeftExtendedMarginTicks>0,"positive margins required");
static_assert(normalMinTick(true)<normalMaxTick(true)&&normalMinTick(false)<normalMaxTick(false),"empty normal drive range");
static_assert(normalLow(true)>0&&normalHigh(true)<=30&&normalLow(false)>0&&normalHigh(false)<=30,"inward quantization required");
}

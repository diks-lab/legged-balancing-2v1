#pragma once
#include <stdint.h>
#include "LegGeometry.h"
namespace leg {
#ifndef LEG_ACCEPT_EXTRA_CONFIRMED
#define LEG_ACCEPT_EXTRA_CONFIRMED 0
#endif
static_assert(LEG_ACCEPT_EXTRA_CONFIRMED==0||LEG_ACCEPT_EXTRA_CONFIRMED==1,"acceptance confirmation must be 0 or 1");
#if LEG_ACCEPT_EXTRA_CONFIRMED
#if !defined(LEG_ACCEPT_RIGHT_MIN) || !defined(LEG_ACCEPT_RIGHT_MAX) || !defined(LEG_ACCEPT_LEFT_MIN) || !defined(LEG_ACCEPT_LEFT_MAX)
#error "Confirmed acceptance requires all four mechanically verified tick limits"
#endif
static_assert(LEG_ACCEPT_RIGHT_MIN>=measuredMinTick(true)&&LEG_ACCEPT_RIGHT_MAX<=measuredMaxTick(true)&&
 LEG_ACCEPT_LEFT_MIN>=measuredMinTick(false)&&LEG_ACCEPT_LEFT_MAX<=measuredMaxTick(false),"acceptance exceeds measured stroke");
static_assert(LEG_ACCEPT_RIGHT_MIN<=normalMinTick(true)&&LEG_ACCEPT_RIGHT_MAX>=normalMaxTick(true)&&
 LEG_ACCEPT_LEFT_MIN<=normalMinTick(false)&&LEG_ACCEPT_LEFT_MAX>=normalMaxTick(false),"acceptance must include normal operating tick limits");
#endif
struct Acceptance {
 int16_t right_min=normalMinTick(true),right_max=normalMaxTick(true),left_min=normalMinTick(false),left_max=normalMaxTick(false);
 bool extra=false;
 bool valid()const{return measured(true,right_min)&&measured(true,right_max)&&measured(false,left_min)&&measured(false,left_max)&&
  right_min<=normalMinTick(true)&&right_max>=normalMaxTick(true)&&left_min<=normalMinTick(false)&&left_max>=normalMaxTick(false);}
 bool contains(bool right,int16_t tick)const{
  if(!valid())return false;
  return tick>=(right?right_min:left_min)&&tick<=(right?right_max:left_max);
 }
};
#if LEG_ACCEPT_EXTRA_CONFIRMED
constexpr Acceptance ConfiguredAcceptance{LEG_ACCEPT_RIGHT_MIN,LEG_ACCEPT_RIGHT_MAX,LEG_ACCEPT_LEFT_MIN,LEG_ACCEPT_LEFT_MAX,true};
#else
constexpr Acceptance ConfiguredAcceptance{}; // no extra acceptance inferred from measured positions
#endif
struct ServoConfiguration {
 uint32_t sample_ms[4]{},started_ms=0,finished_ms=0,rejected_ms=0;
 uint16_t minimum[2]{},maximum[2]{};
 uint8_t mode[2]{},acquired=0,current=0,step=0,status=0,error=0,reject_reason=0;
 // status: never, reading, complete, failed; rejection does not cancel an existing read
};
}

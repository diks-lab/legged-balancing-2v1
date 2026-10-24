#pragma once
#include <stdint.h>
namespace leg {
#ifndef LEG_ACCEPT_EXTRA_CONFIRMED
#define LEG_ACCEPT_EXTRA_CONFIRMED 0
#endif
static_assert(LEG_ACCEPT_EXTRA_CONFIRMED==0||LEG_ACCEPT_EXTRA_CONFIRMED==1,"acceptance confirmation must be 0 or 1");
#if LEG_ACCEPT_EXTRA_CONFIRMED
#if !defined(LEG_ACCEPT_RIGHT_MIN) || !defined(LEG_ACCEPT_RIGHT_MAX) || !defined(LEG_ACCEPT_LEFT_MIN) || !defined(LEG_ACCEPT_LEFT_MAX)
#error "Confirmed acceptance requires all four mechanically verified tick limits"
#endif
static_assert(LEG_ACCEPT_RIGHT_MIN>=-32767&&LEG_ACCEPT_RIGHT_MAX<=32767&&
 LEG_ACCEPT_LEFT_MIN>=-32767&&LEG_ACCEPT_LEFT_MAX<=32767,"acceptance exceeds STS signed position representation");
static_assert(LEG_ACCEPT_RIGHT_MIN<=2010&&LEG_ACCEPT_RIGHT_MAX>=2061&&
 LEG_ACCEPT_LEFT_MIN<=2026&&LEG_ACCEPT_LEFT_MAX>=2081,"acceptance must include unchanged operating tick limits");
#endif
struct Acceptance {
 int16_t right_min=2010,right_max=2061,left_min=2026,left_max=2081;
 bool extra=false;
 bool valid()const{return right_min>=-32767&&left_min>=-32767&&right_min<=2010&&right_max>=2061&&left_min<=2026&&left_max>=2081;}
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

#pragma once
#include <stdint.h>
#include "LegGeometry.h"
namespace leg {
struct Acceptance {
 int16_t right_min=normalMinTick(true),right_max=normalMaxTick(true),left_min=normalMinTick(false),left_max=normalMaxTick(false);
 bool extra=false;
 bool valid(const Geometry&g=activeGeometry())const{return validGeometry(g)&&measured(true,right_min,g)&&measured(true,right_max,g)&&measured(false,left_min,g)&&measured(false,left_max,g)&&
  right_min<=normalMinTick(true,g)&&right_max>=normalMaxTick(true,g)&&left_min<=normalMinTick(false,g)&&left_max>=normalMaxTick(false,g);}
 bool contains(bool right,int16_t tick)const{
  if(!valid())return false;
  return tick>=(right?right_min:left_min)&&tick<=(right?right_max:left_max);
 }
};
inline Acceptance configuredAcceptance(){
 const auto&g=activeGeometry();return {int16_t(measuredMinTick(true,g)),int16_t(measuredMaxTick(true,g)),
  int16_t(measuredMinTick(false,g)),int16_t(measuredMaxTick(false,g)),true};
}
struct ServoConfiguration {
 uint32_t sample_ms[4]{},started_ms=0,finished_ms=0,rejected_ms=0;
 uint16_t minimum[2]{},maximum[2]{};
 uint8_t mode[2]{},acquired=0,current=0,step=0,status=0,error=0,reject_reason=0;
 // status: never, reading, complete, failed; rejection does not cancel an existing read
};
}

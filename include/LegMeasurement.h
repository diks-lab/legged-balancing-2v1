#pragma once
#include <stdint.h>
namespace leg {
// One explicit read-only session; no angle commands, history or inferred limits.
struct Measurement {
 uint32_t started_ms=0,finished_ms=0,rejected_ms=0,sample_ms[2]{},revision=0,count[2]{};
 int16_t latest[2]{},minimum[2]{},maximum[2]{};
 uint8_t status=0,acquired=0,valid=0,side=0,error=0,reject_reason=0;
 // status: never, measuring, stopped, failed; valid is protocol validity, not mechanical safety.
 void begin(uint32_t now){
  const uint32_t next=revision+1;*this=Measurement{};revision=next;started_ms=now;status=1;
 }
 void capture(unsigned index,int16_t tick,uint32_t now){
  const uint8_t bit=uint8_t(1u<<index);
  const bool changed=!(acquired&bit)||latest[index]!=tick;
  latest[index]=tick;sample_ms[index]=now;
  if(!(acquired&bit))minimum[index]=maximum[index]=tick;
  else {if(tick<minimum[index])minimum[index]=tick;if(tick>maximum[index])maximum[index]=tick;}
  acquired|=bit;valid|=bit;if(count[index]!=UINT32_MAX)++count[index];
  if(changed)++revision;
 }
};
constexpr uint32_t MeasurementPrintIntervalMs=500,MeasurementTxWaitMs=1000;
}

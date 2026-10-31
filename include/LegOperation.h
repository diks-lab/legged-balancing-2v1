#pragma once
#include <stdint.h>
namespace leg {
enum class Source:uint8_t { Input=1,Control,Bus,Guard,Stop,Legacy };
enum class Operation:uint8_t { Wait,Extend,Retract,Left,Right,Sync,Stopped,Failed,Release };
// Bit values are observations of existing gates, not new actuator conditions.
enum Gate:uint16_t { Ready=1,Balancing=2,Bench=4,Emergency=8,Moving=16,
 Imu=32,WheelPause=64,Calibration=128,Polarity=256,Connected=512 };
struct Transition {
 uint32_t ms=0,sequence=0;
 uint16_t gates=0;
 uint8_t raw=0,keys=0,reason=0,phase=0,source=0,flags=0,mode=0,from_reason=0,from_phase=0;
};
struct OperationObservation {
 Transition events[4],confirmed;
 uint32_t revision=0,overwritten=0,failure_ms=0,confirmed_ms=0;
 uint16_t gates=0;
 uint8_t count=0,next=0,raw_keys=0,phase=0,failure=255,failure_source=0;
 uint8_t mode=0,allowed=0,operation=0,blocker=0,source=0;
 bool committed=false,tx_wait=false;
 uint8_t guard=255;
};
}

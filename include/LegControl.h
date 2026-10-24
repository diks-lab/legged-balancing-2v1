#pragma once
#include <stdint.h>
#include <math.h>
#include "LegServoReply.h"
namespace leg {
constexpr uint8_t RightId=1,LeftId=2;
constexpr int RightHome=2061,RightExtended=1892,LeftHome=2026,LeftExtended=2208;
constexpr uint16_t ServoSpeed=150;
constexpr uint8_t ServoAcceleration=15;
constexpr float MinPercent=0,MaxPercent=30;
constexpr float HeightRate=4;             // percentage points/s, deliberately slow
constexpr float RollRate=2,MaxRoll=3;     // deg/s, deg
constexpr float RollGain=1;              // differential extension pp/deg
constexpr float DifferentialRate=4;      // pp/s, limits feedback-induced movement
// Enable only after measuring both the IMU sign and the leg extension directions.
#ifndef LEG_ROLL_DIRECTION_CONFIRMED
#define LEG_ROLL_DIRECTION_CONFIRMED 0
#endif
#ifndef LEG_IMU_ROLL_SIGN
#define LEG_IMU_ROLL_SIGN 1
#endif
static_assert(LEG_ROLL_DIRECTION_CONFIRMED == 0 || LEG_ROLL_DIRECTION_CONFIRMED == 1,
              "LEG_ROLL_DIRECTION_CONFIRMED must be 0 or 1");
static_assert(LEG_IMU_ROLL_SIGN == 1 || LEG_IMU_ROLL_SIGN == -1,
              "LEG_IMU_ROLL_SIGN must be +1 or -1");
constexpr bool RollDirectionConfirmed=LEG_ROLL_DIRECTION_CONFIRMED;
constexpr float ImuRollSign=LEG_IMU_ROLL_SIGN; // ignored for actuation until confirmed
constexpr uint32_t SendIntervalMs=50,ReplyTimeoutMs=10,SyncMaxAgeMs=250;
inline float clamp(float x,float lo,float hi){return fmaxf(lo,fminf(hi,x));}
inline float toward(float x,float target,float step){return x+clamp(target-x,-step,step);}
inline int16_t position(bool right,float percent){
 return int16_t(lroundf((right?RightHome:LeftHome)+
   ((right?RightExtended:LeftExtended)-(right?RightHome:LeftHome))*clamp(percent,0,30)/100));
}
inline float percent(bool right,int16_t pos){
 return 100.f*(pos-(right?RightHome:LeftHome))/
   ((right?RightExtended:LeftExtended)-(right?RightHome:LeftHome));
}
inline bool inRange(bool right,int16_t pos){
 const int home=position(right,0),end=position(right,30);
 return pos>=fminf(home,end)&&pos<=fmaxf(home,end);
}
// Latest valid protocol observation per side, not a position command/history.
struct PositionObservation {
 uint32_t received_ms[2]{},attempt_ms=0;
 uint16_t word[2]{};
 uint8_t acquired=0,current=0;
 bool read_only=false;
 void begin(uint32_t now,bool diagnostic){attempt_ms=now;current=0;read_only=diagnostic;}
 void capture(unsigned side,uint16_t raw,uint32_t now){
  word[side]=raw;received_ms[side]=now;acquired|=uint8_t(1u<<side);current|=uint8_t(1u<<side);
 }
};
enum class Reason:uint8_t { ReleaseRequired,Idle,Syncing,Accepted,Driving,Stopped,
 ImuInvalid,PositionRange,ReplyTimeout,ReplyInvalid,TxFailed,ClockGap,TxBusy,WheelRetry,RollUnconfirmed,Initializing,StartWaiting,ExplicitStop,Calibrating,PolarityTest };
struct State {
 float height=0,requested_roll=0,applied_roll=0,measured_roll=NAN;
 float right_percent=0,left_percent=0;
 int16_t right_command=0,left_command=0;
 uint32_t command_ms=0,sync_ms=0,tx_frames=0,tx_busy=0,max_service_us=0;
 Reason reason=Reason::ReleaseRequired;
 uint8_t torque=0; // unknown, OFF pending, OFF queued, ON queued; never physical feedback
 bool active=false,synced=false,limited=false,command_valid=false;
 uint8_t mode=0; // unavailable, supported stopped test, stationary balance
 RxFailure rx_failure;
 PositionObservation positions;
};
class Control {
 public:
 explicit Control(bool roll_confirmed=RollDirectionConfirmed):roll_confirmed_(roll_confirmed){}
 State state;
 uint8_t keys=0; // up/down/left/right
 bool input_gate=false,want_sync=false;
 void input(uint8_t value,bool released_gate){
  keys=released_gate?value:0;input_gate=released_gate;
  if(released_gate&&!value)armed_=true; // preserve release events within a FIFO batch
 }
 void disconnect(){keys=0;input_gate=false;} // active height holds, roll returns smoothly to zero
 void stop(Reason reason){
  state.active=state.synced=false;want_sync=false;armed_=false;state.reason=reason;
  state.requested_roll=state.applied_roll=0;state.limited=false;last_=0;
 }
 bool seed(int16_t right,int16_t left,uint32_t now){
  if(!inRange(true,right)||!inRange(false,left)){stop(Reason::PositionRange);return false;}
  const float r=clamp(percent(true,right),0,30),l=clamp(percent(false,left),0,30);
  state.height=(r+l)/2;difference_=(r-l)/2;
  state.right_percent=r;state.left_percent=l;state.requested_roll=state.applied_roll=0;
  state.synced=true;state.sync_ms=now;want_sync=false;last_=now;return true;
 }
 void activate(uint32_t now){state.active=true;state.reason=Reason::Accepted;last_=now;}
 void tick(uint32_t now,bool eligible,bool moving,bool imu_valid,float roll){
  state.measured_roll=imu_valid?roll:NAN;
  const uint32_t elapsed=last_?uint32_t(now-last_):0;last_=now;
  if(!eligible){stop(Reason::Stopped);return;}
  if(moving){stop(Reason::Driving);return;}
  if(!imu_valid){stop(Reason::ImuInvalid);return;}
  if(!keys&&input_gate)armed_=true;
  if(!state.active){
   // These reasons describe current external gates, not a latched operation failure.
   // The guards above have now passed (e.g. Share cleared explicit STOP). Refresh
   // the diagnosis without synchronizing, enabling torque or changing a target.
   switch(state.reason){
    case Reason::ReleaseRequired: case Reason::Stopped: case Reason::ExplicitStop:
    case Reason::Initializing: case Reason::StartWaiting: case Reason::Calibrating:
    case Reason::PolarityTest: case Reason::Driving: case Reason::ImuInvalid:
    case Reason::WheelRetry:
     state.reason=armed_?Reason::Idle:Reason::ReleaseRequired;break;
    default:break; // preserve position/UART failures until another explicit operation
   }
   if(!roll_confirmed_&&(keys&12)&&!(keys&3)){state.reason=Reason::RollUnconfirmed;return;}
   if(!state.synced&&!want_sync&&keys&&input_gate&&armed_){want_sync=true;state.reason=Reason::Syncing;armed_=false;}
   return;
  }
  if(elapsed>200){stop(Reason::ClockGap);return;} // never catch up a stale long command
  const float dt=elapsed*.001f;
  const int height_axis=((keys&1)!=0)-((keys&2)!=0);
  const int roll_axis=((keys&4)!=0)-((keys&8)!=0);
  state.height=clamp(state.height+height_axis*HeightRate*dt,0,30);
  const bool height_limited=(height_axis<0&&state.height<=0)||(height_axis>0&&state.height>=30);
  if(roll_confirmed_&&roll_axis)state.requested_roll=clamp(state.requested_roll+roll_axis*RollRate*dt,-MaxRoll,MaxRoll);
  else if(roll_confirmed_&&!(keys&12))state.requested_roll=toward(state.requested_roll,0,RollRate*dt);
  const float room=fminf(state.height,30-state.height);
  state.applied_roll=roll_confirmed_?clamp(state.requested_roll,-room/RollGain,room/RollGain):0;
  // Height-only commissioning holds the synchronized differential, never AngleX feedback.
  const float desired=roll_confirmed_?RollGain*(state.applied_roll-roll):difference_;
  difference_=clamp(toward(difference_,clamp(desired,-room,room),DifferentialRate*dt),-room,room);
  state.right_percent=state.height+difference_;state.left_percent=state.height-difference_;
  state.limited=height_limited||fabsf(state.applied_roll-state.requested_roll)>.0001f||fabsf(desired)>room;
  state.reason=(!roll_confirmed_&&(keys&12))?Reason::RollUnconfirmed:Reason::Accepted;
 }
 private:
 uint32_t last_=0;
 float difference_=0;
 bool armed_=false,roll_confirmed_=false;
};
}

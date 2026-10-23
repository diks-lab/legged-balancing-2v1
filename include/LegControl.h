#pragma once
#include <stdint.h>
#include <math.h>
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
constexpr float ImuRollSign=1;            // UNVERIFIED: AngleX positive = left lean
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
enum class Reason:uint8_t { ReleaseRequired,Idle,Syncing,Accepted,Driving,Stopped,
 ImuInvalid,PositionRange,ReplyTimeout,ReplyInvalid,TxFailed,ClockGap,TxBusy,WheelRetry };
struct State {
 float height=0,requested_roll=0,applied_roll=0,measured_roll=NAN;
 float right_percent=0,left_percent=0;
 int16_t right_command=0,left_command=0;
 uint32_t command_ms=0,sync_ms=0,tx_frames=0,tx_busy=0,max_service_us=0;
 Reason reason=Reason::ReleaseRequired;
 uint8_t torque=0; // unknown, OFF pending, OFF queued, ON queued; never physical feedback
 bool active=false,synced=false,limited=false,command_valid=false;
};
class Control {
 public:
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
   if(armed_&&state.reason==Reason::ReleaseRequired)state.reason=Reason::Idle;
   if(!state.synced&&!want_sync&&keys&&input_gate&&armed_){want_sync=true;state.reason=Reason::Syncing;armed_=false;}
   return;
  }
  if(elapsed>200){stop(Reason::ClockGap);return;} // never catch up a stale long command
  const float dt=elapsed*.001f;
  const int height_axis=((keys&1)!=0)-((keys&2)!=0);
  const int roll_axis=((keys&4)!=0)-((keys&8)!=0);
  state.height=clamp(state.height+height_axis*HeightRate*dt,0,30);
  const bool height_limited=(height_axis<0&&state.height<=0)||(height_axis>0&&state.height>=30);
  if(roll_axis)state.requested_roll=clamp(state.requested_roll+roll_axis*RollRate*dt,-MaxRoll,MaxRoll);
  else if(!(keys&12))state.requested_roll=toward(state.requested_roll,0,RollRate*dt);
  const float room=fminf(state.height,30-state.height);
  state.applied_roll=clamp(state.requested_roll,-room/RollGain,room/RollGain);
  const float desired=RollGain*(state.applied_roll-roll);
  difference_=clamp(toward(difference_,clamp(desired,-room,room),DifferentialRate*dt),-room,room);
  state.right_percent=state.height+difference_;state.left_percent=state.height-difference_;
  state.limited=height_limited||fabsf(state.applied_roll-state.requested_roll)>.0001f||fabsf(desired)>room;
  state.reason=Reason::Accepted;
 }
 private:
 uint32_t last_=0;
 float difference_=0;
 bool armed_=false;
};
}

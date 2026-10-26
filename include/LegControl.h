#pragma once
#include <stdint.h>
#include <math.h>
#include "LegServoReply.h"
#include "LegOperation.h"
#include "LegAcceptance.h"
#include "LegMeasurement.h"
#include "LegCalibration.h"
#include "Ps4ControlMapping.h"
namespace leg {
constexpr uint8_t RightId=1,LeftId=2;
constexpr uint16_t ServoSpeed=150;
constexpr uint8_t ServoAcceleration=15;
constexpr float HeightRate=12;            // maximum servo-travel percentage points/s
constexpr float RollRate=4,MaxRoll=3;     // deg/s, deg
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
// Common displacement preserves both legs' existing difference. Every interval contains zero.
inline void movementBounds(float rightPct,float leftPct,float&low,float&high,bool inward){
 low=-100000;high=100000;const float values[]={rightPct,leftPct};
 for(unsigned side=0;side<2;++side){
  const bool right=side==0;const float p=values[side],lo=normalLow(right),hi=normalHigh(right);
  float loDelta=lo-p,hiDelta=hi-p;
  if(inward&&p>hi)hiDelta=0;
  if(inward&&p<lo)loDelta=0;
  low=fmaxf(low,loDelta);high=fminf(high,hiDelta);
 }
}
inline uint8_t returnDirections(float rightPct,float leftPct){
 float lo,hi;movementBounds(rightPct,leftPct,lo,hi,true);
 return uint8_t((hi>.00001f?1:0)|(lo<-.00001f?2:0));
}
inline bool conflictingReturn(float rightPct,float leftPct){
 return (rightPct>normalHigh(true)||leftPct>normalHigh(false))&&
        (rightPct<normalLow(true)||leftPct<normalLow(false));
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
 ImuInvalid,PositionRange,ReplyTimeout,ReplyInvalid,TxFailed,ClockGap,TxBusy,WheelRetry,RollUnconfirmed,Initializing,StartWaiting,ExplicitStop,Calibrating,PolarityTest,AcceptanceInvalid,RecoveryDirection,RecoveryRelease,PairLimit,ConfigReadOnly,ConfigBusy,Measuring,MeasureStopRequired,ReturnConflict,CalibrationRequired };
// Diagnostic observations only. A successful UART write is NOT servo feedback.
struct TxDiagnostics {
 uint32_t goals=0,goal_wait=0,goal_failed=0,on_sent=0,on_wait=0,on_failed=0;
 uint32_t calculated_ms=0;
 int16_t calculated_right=0,calculated_left=0;
 bool calculated_valid=false;
 static void count(uint32_t&value){if(value!=UINT32_MAX)++value;}
};
// Diagnostic only: command-at-READ is not an acknowledgement from the servo.
struct Following {
 struct Side {
  uint32_t sample_ms=0,request_ms=0,command_ms=0;
  int16_t tick=0,command=0;
  uint8_t status=0,issues=0; // unread, waiting, valid, failed, cancelled, TX busy, partial TX
 } side[2];
};
constexpr uint32_t FollowIntervalMs=500,FollowFreshMs=1500;
struct State {
 Following following;

 float height=0,requested_roll=0,applied_roll=0,measured_roll=NAN;
 float right_percent=0,left_percent=0;
 int16_t right_command=0,left_command=0;
 uint32_t command_ms=0,sync_ms=0,tx_frames=0,tx_busy=0,max_service_us=0;
 Reason reason=Reason::ReleaseRequired;
 uint8_t torque=0; // unknown, OFF pending, OFF queued, ON queued; never physical feedback
 bool active=false,synced=false,limited=false,command_valid=false;
 uint8_t mode=0; // unavailable, supported stopped test, stationary balance
 TxDiagnostics tx;
 RxFailure rx_failure;
 PositionObservation positions;
 OperationObservation operation;
 Acceptance acceptance=configuredAcceptance();
 Geometry geometry=activeGeometry();
 LegCalibration calibration;
 ServoConfiguration configuration;
 Measurement measurement;
 ps4_control::StickObservation stick;
 uint8_t recovery=0,return_directions=0; // normal, holding until release, manual inward return
};
// Presentation-only hint: never used to relax a movement gate.
inline uint8_t recoveryHint(const State&s){
 if(!s.recovery||!validGeometry(s.geometry))return 0;
 const bool above=s.right_percent>normalHigh(true,s.geometry)||s.left_percent>normalHigh(false,s.geometry);
 const bool below=s.right_percent<normalLow(true,s.geometry)||s.left_percent<normalLow(false,s.geometry);
 return above?(below?3:1):below?2:0; // shorten, extend, conflict
}
class Control {
 public:
 explicit Control(bool roll_confirmed=RollDirectionConfirmed,Acceptance acceptance=configuredAcceptance()):roll_confirmed_(roll_confirmed){state.acceptance=acceptance;}
 State state;
 uint8_t keys=0; // up/down/left/right
 bool input_gate=false,want_sync=false;
 // Internal direction adapter retained for deterministic core/protocol tests.
 // Runtime PS4 input enters through stickInput; D-pad never calls this adapter.
 void input(uint8_t value,bool released_gate,uint32_t now=0){
  setInput(float(bool(value&1))-float(bool(value&2)),float(bool(value&4))-float(bool(value&8)),!value,released_gate,now);
  state.operation.raw_keys=value;keys=released_gate?value:0;observe(Source::Input,now,state.operation.phase);
 }
 void rawStick(int8_t lx,int8_t ly,int8_t rx,int8_t ry,uint32_t received){
  auto&s=state.stick;s.lx=lx;s.ly=ly;s.rx=rx;s.ry=ry;s.ms=received;
  // Measured PS4 axes: physical UP is positive LY. Convert once here.
  s.height=ps4_control::legAxis(ly);s.roll=-ps4_control::legAxis(lx);
  state.operation.raw_keys=ps4_control::directions(s.height,s.roll);
 }
 void stickInput(int8_t lx,int8_t ly,int8_t rx,int8_t ry,bool gate,uint32_t now,uint32_t received){
  rawStick(lx,ly,rx,ry,received);
  setInput(state.stick.height,state.stick.roll,ps4_control::leftCentered(lx,ly),gate,now);
 }
 void received(uint8_t value,uint32_t now){state.operation.raw_keys=value;observe(Source::Input,now,state.operation.phase);}
 void disconnect(){
  keys=0;input_gate=false;height_input_=roll_input_=0;
  state.requested_roll=state.applied_roll=0;roll_rearm_=true;
  state.stick.applied_height=state.stick.applied_roll=0;
  // Cancel an unsent ramp increment; hold the last position actually queued to UART.
  if(state.active&&state.command_valid){
   state.right_percent=percent(true,state.right_command);state.left_percent=percent(false,state.left_command);
   state.height=(state.right_percent+state.left_percent)/2;difference_=(state.right_percent-state.left_percent)/2;
  }
 }
 void stop(Reason reason){
  state.operation.tx_wait=false;height_input_=roll_input_=0;state.stick.applied_height=state.stick.applied_roll=0;state.recovery=state.return_directions=0;
  state.active=state.synced=false;want_sync=false;armed_=false;state.reason=reason;
  state.requested_roll=state.applied_roll=0;state.limited=false;last_=0;
 }
 bool accepted(bool right,int16_t tick)const{
  return state.acceptance.valid()&&(inRange(right,tick)||(state.acceptance.extra&&state.acceptance.contains(right,tick)));
 }
 int16_t target(bool right)const{
  const float pct=right?state.right_percent:state.left_percent;
  return int16_t(lroundf(zeroTick(right)+(fullTick(right)-zeroTick(right))*pct/100));
 }
 bool commandAllowed(int16_t right,int16_t left)const{
  return state.recovery?accepted(true,right)&&accepted(false,left):inRange(true,right)&&inRange(false,left);
 }
 bool seed(int16_t right,int16_t left,uint32_t now){
  if(!accepted(true,right)||!accepted(false,left)){stop(Reason::PositionRange);return false;}
  const bool outside=!inRange(true,right)||!inRange(false,left);
  const float r=percent(true,right),l=percent(false,left);
  state.height=(r+l)/2;difference_=(r-l)/2;
  state.right_percent=r;state.left_percent=l;state.requested_roll=state.applied_roll=0;
  state.recovery=outside?1:0;
  roll_rearm_=true;state.return_directions=0;
  state.synced=true;state.sync_ms=now;want_sync=false;last_=now;return true;
 }
 void activate(uint32_t now){state.active=true;state.reason=state.recovery?Reason::RecoveryRelease:Reason::Accepted;last_=now;}
 void tick(uint32_t now,bool eligible,bool moving,bool imu_valid,float roll){
  update(now,eligible,moving,imu_valid,roll);
  if(state.synced&&validGeometry(activeGeometry())){
   state.tx.calculated_right=target(true);state.tx.calculated_left=target(false);
   state.tx.calculated_ms=now;state.tx.calculated_valid=true;
  }
  observe(Source::Control,now,state.operation.phase);
 }
 void observe(Source source,uint32_t now,uint8_t phase){
  auto&o=state.operation;
  const uint8_t flags=(input_gate?1:0)|(armed_?2:0)|(state.active?4:0)|(state.synced?8:0)|(want_sync?16:0)|(state.operation.tx_wait?32:0);
  const Transition*previous=o.count?&o.events[(o.next+3)%4]:nullptr;
  if(previous&&previous->gates==o.gates&&previous->raw==o.raw_keys&&previous->keys==keys&&
     previous->reason==uint8_t(state.reason)&&previous->phase==phase&&previous->flags==flags&&previous->mode==state.mode)return;
  const uint8_t prior=previous?previous->reason:255;
  if((state.reason==Reason::PositionRange||state.reason==Reason::ReplyTimeout||state.reason==Reason::ReplyInvalid||
      state.reason==Reason::TxFailed||state.reason==Reason::ClockGap||state.reason==Reason::RollUnconfirmed||state.reason==Reason::RecoveryDirection||state.reason==Reason::PairLimit||state.reason==Reason::ReturnConflict)&&prior!=uint8_t(state.reason)){
   o.failure=uint8_t(state.reason);o.failure_ms=now;o.failure_source=uint8_t(source);
  }
  Transition event;event.ms=now;event.sequence=++o.revision;event.gates=o.gates;
  event.raw=o.raw_keys;event.keys=keys;event.reason=uint8_t(state.reason);event.phase=phase;
  event.source=uint8_t(source);event.flags=flags;event.mode=state.mode;
  event.from_reason=prior;event.from_phase=previous?previous->phase:255;
  if(o.count==4)++o.overwritten;else ++o.count;
  o.events[o.next]=event;o.next=(o.next+1)%4;o.phase=phase;
 }
 void commit(uint32_t now,bool healthy,Reason gate,uint8_t phase){
  observe(Source::Guard,now,phase);
  auto&o=state.operation;
  const bool new_press=(o.raw_keys&uint8_t(~o.confirmed.raw))!=0;
  const bool interrupted=(o.confirmed.flags&(4u|16u))!=0;
  if(!healthy&&gate!=Reason::ExplicitStop&&(o.raw_keys||interrupted)&&
      (o.failure==255||interrupted||new_press)&&(o.guard!=uint8_t(gate)||new_press)){
   o.failure=uint8_t(gate);o.failure_ms=now;o.failure_source=uint8_t(Source::Guard);
  }
  o.guard=healthy?255:uint8_t(gate);
  if(o.count)o.confirmed=o.events[(o.next+3)%4];
  o.confirmed.ms=now;o.confirmed.sequence=o.revision;
  o.committed=true;o.confirmed_ms=now;o.mode=healthy?state.mode:0;
  o.source=o.count?o.events[(o.next+3)%4].source:0;o.blocker=uint8_t(gate);o.allowed=0;
  if(!healthy){o.operation=uint8_t(Operation::Stopped);return;}
  o.blocker=255;
  if(phase==7){o.operation=uint8_t(Operation::Stopped);o.blocker=uint8_t(Reason::Stopped);return;}
  if(o.tx_wait){o.operation=uint8_t(Operation::Stopped);o.blocker=uint8_t(Reason::TxBusy);return;}
  if(phase>=9){o.operation=uint8_t(Operation::Stopped);o.blocker=uint8_t(phase>=11?Reason::Measuring:Reason::ConfigReadOnly);return;}
  if(want_sync||(phase>=1&&phase<=6)){o.operation=uint8_t(Operation::Sync);return;}
  if(!input_gate){o.operation=uint8_t(Operation::Release);o.blocker=uint8_t(Reason::ReleaseRequired);return;}
  if(!state.active&&!armed_){
   const Reason failure=Reason(o.failure);
   const bool failed=failure==Reason::PositionRange||failure==Reason::ReplyTimeout||
       failure==Reason::ReplyInvalid||failure==Reason::TxFailed||failure==Reason::ClockGap;
   o.operation=uint8_t(failed?Operation::Failed:Operation::Release);
   o.blocker=failed?o.failure:uint8_t(Reason::ReleaseRequired);return;
  }
  if(state.recovery==1){o.operation=uint8_t(Operation::Release);o.blocker=uint8_t(Reason::RecoveryRelease);return;}
  o.allowed=1;o.operation=uint8_t(Operation::Wait);
  if(state.recovery&&(state.reason==Reason::RecoveryDirection||state.reason==Reason::PairLimit||state.reason==Reason::ReturnConflict)){
   o.allowed=0;o.operation=uint8_t(Operation::Failed);o.blocker=uint8_t(state.reason);return;
  }
  if(!roll_confirmed_&&(keys&12)){
   o.blocker=uint8_t(Reason::RollUnconfirmed);
   if(!(keys&3)){o.allowed=0;o.operation=uint8_t(Operation::Wait);return;}
  }
  if(state.active){
   const int height=int(bool(keys&1))-int(bool(keys&2)),rollAxis=int(bool(keys&4))-int(bool(keys&8));
   o.operation=uint8_t(height>0?Operation::Extend:height<0?Operation::Retract:
      rollAxis>0?Operation::Left:rollAxis<0?Operation::Right:Operation::Wait);
  }
 }
 private:
 void update(uint32_t now,bool eligible,bool moving,bool imu_valid,float roll){
  state.measured_roll=imu_valid?roll:NAN;
  const uint32_t elapsed=last_?uint32_t(now-last_):0;last_=now;
  if(!eligible){stop(Reason::Stopped);return;}
  if(!validGeometry(activeGeometry())){stop(Reason::CalibrationRequired);return;}
  if(moving){stop(Reason::Driving);return;}
  if(!imu_valid){stop(Reason::ImuInvalid);return;}
  if(centered_&&input_gate)armed_=true;
  if(!state.active){
   state.stick.applied_height=state.stick.applied_roll=0;
   // These reasons describe current external gates, not a latched operation failure.
   // The guards above have now passed (e.g. Share cleared explicit STOP). Refresh
   // the diagnosis without synchronizing, enabling torque or changing a target.
   switch(state.reason){
    case Reason::ReleaseRequired: case Reason::Stopped: case Reason::ExplicitStop:
    case Reason::Initializing: case Reason::StartWaiting: case Reason::Calibrating:
    case Reason::PolarityTest: case Reason::Driving: case Reason::ImuInvalid:
    case Reason::WheelRetry: case Reason::ConfigReadOnly: case Reason::Measuring:
     state.reason=armed_?Reason::Idle:Reason::ReleaseRequired;break;
    default:break; // preserve position/UART failures until another explicit operation
   }
   if(!roll_confirmed_&&(keys&12)&&!(keys&3)){state.reason=Reason::RollUnconfirmed;return;}
   if(!state.synced&&!want_sync&&keys&&input_gate&&armed_){want_sync=true;state.reason=Reason::Syncing;armed_=false;}
   return;
  }
  if(elapsed>200){stop(Reason::ClockGap);return;} // never catch up a stale long command
  if(elapsed||state.recovery==1)state.stick.applied_height=state.stick.applied_roll=0;
  if(!height_input_)state.stick.applied_height=0;
  if(!roll_input_)state.stick.applied_roll=0;
  const float dt=elapsed*.001f;
  const float height_axis=height_input_,roll_axis=roll_input_;
  if(state.recovery){
   if(state.recovery==1){
    state.reason=Reason::RecoveryRelease;
    if(centered_&&input_gate){
     state.recovery=2;state.reason=conflictingReturn(state.right_percent,state.left_percent)?Reason::ReturnConflict:Reason::Accepted;float low,high;pairBounds(low,high,true);
     state.return_directions=(high>.00001f?1:0)|(low<-.00001f?2:0);
     if(!state.return_directions&&state.reason!=Reason::ReturnConflict)state.reason=Reason::PairLimit;
    }
    return;
   }
   float low,high;pairBounds(low,high,true);
   state.return_directions=(high>.00001f?1:0)|(low<-.00001f?2:0);
   if((keys&12)&&(roll_confirmed_||!(keys&3))){state.reason=roll_confirmed_?Reason::RecoveryDirection:Reason::RollUnconfirmed;return;}
   const float requested=height_axis*HeightRate*dt;
   const float delta=clamp(requested,low,high);
   state.limited=fabsf(delta-requested)>.00001f;
   if(height_axis&&fabsf(delta)<.000001f){
    const float rh=normalHigh(true),lh=normalHigh(false);
    const bool above=state.right_percent>rh||state.left_percent>lh;
    const bool below=state.right_percent<normalLow(true)||state.left_percent<normalLow(false);
    state.reason=above&&below?Reason::ReturnConflict:((height_axis<0&&above&&!below)||(height_axis>0&&below&&!above))?Reason::PairLimit:Reason::RecoveryDirection;
    return;
   }
   if(fabsf(delta)>.000001f)state.stick.applied_height=height_axis;
   state.right_percent+=delta;state.left_percent+=delta;
   state.height=(state.right_percent+state.left_percent)/2;difference_=(state.right_percent-state.left_percent)/2;
   state.reason=conflictingReturn(state.right_percent,state.left_percent)?Reason::ReturnConflict:!state.return_directions?Reason::PairLimit:(!roll_confirmed_&&(keys&12))?Reason::RollUnconfirmed:state.operation.tx_wait?Reason::TxBusy:Reason::Accepted;
   if(inRange(true,target(true))&&inRange(false,target(false))){
    // Adopt exactly the emitted ticks; never re-level at the mode transition.
    state.right_percent=percent(true,target(true));state.left_percent=percent(false,target(false));
    state.height=(state.right_percent+state.left_percent)/2;difference_=(state.right_percent-state.left_percent)/2;
    state.recovery=state.return_directions=0;roll_rearm_=true;
   }
   return;
  }
  float low,high;pairBounds(low,high,false);
  const float requested=height_axis*HeightRate*dt,delta=clamp(requested,low,high);
  if(fabsf(delta)>.000001f)state.stick.applied_height=height_axis;
  state.right_percent+=delta;state.left_percent+=delta;
  state.height=(state.right_percent+state.left_percent)/2;
  const bool height_limited=height_axis&&fabsf(delta-requested)>.00001f;
  const float prior_roll=state.requested_roll;
  if(roll_rearm_&&(keys&12)&&roll_confirmed_)roll_rearm_=false;
  if(roll_confirmed_&&!roll_rearm_&&roll_axis)state.requested_roll=clamp(state.requested_roll+roll_axis*RollRate*dt,-MaxRoll,MaxRoll);
  else if(roll_confirmed_&&!roll_rearm_&&!(keys&12))state.requested_roll=toward(state.requested_roll,0,RollRate*dt);
  if(state.requested_roll!=prior_roll&&roll_axis)state.stick.applied_roll=roll_axis;
  const float diffLo=fmaxf(normalLow(true)-state.height,state.height-normalHigh(false)),
              diffHi=fminf(normalHigh(true)-state.height,state.height-normalLow(false));
  const float room=fminf(fabsf(diffLo),fabsf(diffHi));
  state.applied_roll=roll_confirmed_&&!roll_rearm_?clamp(state.requested_roll,-room/RollGain,room/RollGain):0;
  const float desired=roll_confirmed_&&!roll_rearm_?RollGain*(state.applied_roll-roll):difference_;
  difference_=clamp(toward(difference_,clamp(desired,diffLo,diffHi),DifferentialRate*dt),diffLo,diffHi);
  state.right_percent=state.height+difference_;state.left_percent=state.height-difference_;
  state.limited=height_limited||fabsf(state.applied_roll-state.requested_roll)>.0001f||desired<diffLo||desired>diffHi;
  state.reason=(!roll_confirmed_&&(keys&12))?Reason::RollUnconfirmed:
      state.operation.tx_wait?Reason::TxBusy:Reason::Accepted;

 }
 private:
 uint32_t last_=0;
 float difference_=0;
 bool armed_=false,roll_confirmed_=false,roll_rearm_=false,centered_=true;
 float height_input_=0,roll_input_=0;
 void setInput(float height,float roll,bool centered,bool gate,uint32_t now){
  state.stick.height=height;state.stick.roll=roll;centered_=centered;
  state.operation.raw_keys=ps4_control::directions(height,roll);
  keys=gate?state.operation.raw_keys:0;input_gate=gate;
  height_input_=gate?height:0;roll_input_=gate?roll:0;
  if(gate&&centered)armed_=true;
  observe(Source::Input,now,state.operation.phase);
 }
 void pairBounds(float&low,float&high,bool inward)const{
  movementBounds(state.right_percent,state.left_percent,low,high,inward);
 }


};
}

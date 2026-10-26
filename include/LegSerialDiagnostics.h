#pragma once
#include "LegControl.h"
#include <stdio.h>
#include <string.h>
namespace leg {
// Single producer / single consumer; caller protects push/pop with its RTOS lock.
// No formatting, Serial access, allocation or waits in capture/input/pop.
class SerialDiagnostics {
 public:
 enum Type:uint8_t { Input,Decision,SyncStart,SyncDone,Range,Receive,Configuration,SyncFailed,AcceptanceOutside,Measure,MeasureRejected,Sticks,Calibration,Motion,FollowingRead };
 struct Sync { Geometry geometry;Acceptance accept;int16_t right,left;uint8_t acquired,recovery,directions,hint; };
 struct MotionState {
  TxDiagnostics tx;
  float height=0,roll=0;
  float measured=0,requested=0,applied=0;
  int16_t sent_right=0,sent_left=0;
  uint32_t sent_ms=0,coalesced=0;
  int8_t lx=0,ly=0;
  uint8_t torque=0,flags=0,reason=0,blocker=255;
 };
 static_assert(sizeof(MotionState)<=80,"bounded motion diagnostics");
 struct Record {
  uint32_t ms=0;
  Type type=Input;uint8_t keys=0,allowed=0,reason=0;
  union Payload { MotionState motion;Following following;Sync sync;RxFailure rx;ServoConfiguration config;Measurement measurement;ps4_control::StickObservation stick;struct { LegCalibration value;Geometry geometry; } calibration;struct { Measurement value;Geometry geometry; } measured;Payload():motion{}{} } data;
 };
 static_assert(sizeof(Record::Payload)==sizeof(MotionState),"initialize the largest payload completely");
 static constexpr unsigned Capacity=8,Chunk=48;
 void input(uint8_t keys,bool gate,uint32_t now){
  if(keys==raw_)return;
  raw_=keys;Record r;r.type=Input;r.ms=now;r.keys=keys;r.allowed=gate;
  latest_input_=r;input_pending_=true;
 }
 void capture(const State&s,uint8_t phase,uint32_t now){
  observed_now_=now;
  const auto&o=s.operation;
  if((s.stick.flags&32)?bool(s.stick.flags&2):!o.raw_keys)seen_refusals_=0;
  if(o.committed&&(o.allowed!=allowed_||o.operation!=operation_||o.blocker!=blocker_||o.raw_keys!=decision_keys_)){
   Record r;r.type=Decision;r.ms=now;r.keys=o.raw_keys;r.allowed=o.allowed;
   r.reason=o.blocker;r.data.sync.recovery=o.operation;r.data.sync.hint=recoveryHint(s);
   const bool changed=o.allowed!=allowed_||o.operation!=operation_||o.blocker!=blocker_;
   const bool important=changed&&o.blocker!=255&&o.blocker!=uint8_t(Reason::Idle)&&o.blocker!=uint8_t(Reason::ReleaseRequired);
   const uint32_t bit=o.blocker<32?1u<<o.blocker:0;
   if(important&&!(seen_refusals_&bit)){push(r);seen_refusals_|=bit;}
   else {latest_input_=r;input_pending_=true;}

  }
  if(s.positions.attempt_ms!=attempt_){
   attempt_=s.positions.attempt_ms;Record r;r.type=SyncStart;r.ms=now;r.allowed=s.positions.read_only;push(r);
  }
  const bool read_done=s.positions.read_only&&s.positions.current==3&&phase_>=1&&phase_<=4&&phase==0&&
   (s.reason==Reason::Idle||s.reason==Reason::PositionRange||s.reason==Reason::CalibrationRequired);
  if((s.synced&&!synced_)||read_done){
   Record r;r.type=SyncDone;r.ms=now;r.data.sync.accept=s.acceptance;r.data.sync.geometry=s.geometry;
   r.data.sync.right=decodePosition(s.positions.word[0]);r.data.sync.left=decodePosition(s.positions.word[1]);
   r.data.sync.acquired=s.positions.current;r.allowed=read_done;push(r);
  }
  if(s.recovery!=recovery_||s.return_directions!=directions_){
   Record r;r.type=Range;r.ms=now;r.data.sync.recovery=s.recovery;
   r.data.sync.directions=s.return_directions;r.data.sync.hint=recoveryHint(s);r.allowed=s.active||s.synced;
   if(s.recovery==1)r.data.sync.directions=returnDirections(s.right_percent,s.left_percent);
   push(r);
  }
  if((phase_>=1&&phase_<=6)&&(phase==0||phase==7)&&!s.synced&&!s.active&&!read_done){
   Record r;r.type=SyncFailed;r.ms=now;r.reason=uint8_t(s.reason);push(r);
   if(s.reason==Reason::PositionRange&&s.positions.current==3){
    r.type=AcceptanceOutside;r.data.sync.accept=s.acceptance;r.data.sync.geometry=s.geometry;
    r.data.sync.right=decodePosition(s.positions.word[0]);r.data.sync.left=decodePosition(s.positions.word[1]);push(r);
   }
  }
  if(s.rx_failure.issues&&s.rx_failure.ms!=rx_ms_){
   rx_ms_=s.rx_failure.ms;Record r;r.type=Receive;r.ms=now;r.data.rx=s.rx_failure;push(r);
  }
  const auto&c=s.configuration;
  if((c.status==2||c.status==3)&&(c.finished_ms!=config_ms_||c.status!=config_status_)){
   Record r;r.type=Configuration;r.ms=now;r.data.config=c;push(r);
  }
  const auto&m=s.measurement;
  if(m.revision!=measure_revision_&&(m.status!=measure_status_||now-measure_log_ms_>=MeasurementPrintIntervalMs)){
   Record r;r.type=Measure;r.ms=now;r.data.measured.value=m;r.data.measured.geometry=s.geometry;push(r);
   measure_revision_=m.revision;measure_status_=m.status;measure_log_ms_=now;
  }
  if(m.rejected_ms&&m.rejected_ms!=measure_rejected_ms_){
   Record r;r.type=MeasureRejected;r.ms=m.rejected_ms;r.reason=m.reject_reason;push(r);measure_rejected_ms_=m.rejected_ms;
  }
  if(!m.rejected_ms)measure_rejected_ms_=0;
  const auto&stick=s.stick;
  const uint32_t raw=uint32_t(uint8_t(stick.lx))|(uint32_t(uint8_t(stick.ly))<<8)|
      (uint32_t(uint8_t(stick.rx))<<16)|(uint32_t(uint8_t(stick.ry))<<24);
  if((stick.flags&32)||stick_seen_){
   if(!stick_seen_||raw!=stick_raw_||stick.flags!=stick_flags_||stick.applied_height!=last_height_||
      stick.applied_roll!=last_roll_||stick.drive!=last_drive_||stick.yaw!=last_yaw_){
    Record r;r.type=Sticks;r.ms=now;r.data.stick=stick;latest_stick_=r;stick_pending_=true;stick_seen_=true;
    stick_raw_=raw;stick_flags_=stick.flags;last_height_=stick.applied_height;last_roll_=stick.applied_roll;
    last_drive_=stick.drive;last_yaw_=stick.yaw;
   }
  }
  if(s.calibration.revision!=calibration_revision_){
   Record r;r.type=Calibration;r.ms=now;r.data.calibration.value=s.calibration;r.data.calibration.geometry=s.geometry;push(r);calibration_revision_=s.calibration.revision;
  }
  for(unsigned side=0;side<2;++side){
   const auto&f=s.following.side[side];
   if(f.status!=follow_status_[side]||f.sample_ms!=follow_sample_[side]){
    latest_feedback_=Record{};latest_feedback_.type=FollowingRead;latest_feedback_.ms=now;
    latest_feedback_.data.following=s.following;feedback_pending_=true;
    follow_status_[side]=f.status;follow_sample_[side]=f.sample_ms;
   }
  }
  captureMotion(s,now);
  synced_=s.synced;recovery_=s.recovery;directions_=s.return_directions;phase_=phase;
  allowed_=o.allowed;operation_=o.operation;blocker_=o.blocker;decision_keys_=o.raw_keys;
  config_ms_=c.finished_ms;config_status_=c.status;
 }
 bool pop(Record&r,uint32_t&lost){
  lost=lost_;lost_=0;
  if(!count_){
   if(uint32_t(observed_now_-continuous_ms_)<500)return false;
   if(feedback_pending_&&feedback_turn_){
    r=latest_feedback_;feedback_pending_=false;feedback_turn_=false;continuous_ms_=observed_now_;return true;
   }
   feedback_turn_=true;
   if(motion_count_){
    r=motion_edges_[motion_head_];motion_head_=(motion_head_+1)%2;--motion_count_;
    r.data.motion.coalesced=motion_lost_;motion_lost_=0;continuous_ms_=observed_now_;return true;
   }
   if(input_pending_&&(!stick_pending_||input_turn_)){r=latest_input_;input_pending_=false;input_turn_=false;}
   else if(stick_pending_){r=latest_stick_;stick_pending_=false;input_turn_=true;}
   else return false;
   continuous_ms_=observed_now_;return true;
  }
  r=records_[head_];head_=(head_+1)%Capacity;--count_;return true;
 }
 private:
 // Separate two-edge queue: noisy movement transitions cannot fill the critical FIFO.
 Record motion_edges_[2];
 uint8_t motion_head_=0,motion_count_=0,motion_direction_=0;
 bool final_pending_=false,input_turn_=true;
 uint32_t motion_lost_=0,final_failures_=0;
 void pushMotion(const Record&r){
  if(motion_count_==2){motion_edges_[(motion_head_+1)%2]=r;TxDiagnostics::count(motion_lost_);return;}
  motion_edges_[(motion_head_+motion_count_)%2]=r;++motion_count_;
 }
 void captureMotion(const State&s,uint32_t now){
  // Observe accepted/routed intent even when it produces no applied movement.
  // The packet observer bit selects the real PS4 path; core tests use applied rates.
  const uint8_t direction=(s.stick.flags&32)?
   ((s.stick.flags&65)==65?s.operation.raw_keys:0):
   (s.active?ps4_control::directions(s.stick.applied_height,s.stick.applied_roll):0);
  Record r;r.type=Motion;r.ms=now;
  auto&m=r.data.motion;m=MotionState{};m.tx=s.tx;m.height=s.stick.applied_height;m.roll=s.stick.applied_roll;
  m.measured=s.measured_roll;m.requested=s.requested_roll;m.applied=s.applied_roll;
  m.sent_right=s.right_command;m.sent_left=s.left_command;m.sent_ms=s.command_ms;
  m.lx=s.stick.lx;m.ly=s.stick.ly;m.torque=s.torque;m.reason=uint8_t(s.reason);
  m.blocker=s.operation.committed?s.operation.blocker:uint8_t(s.reason);
  m.flags=(s.tx.calculated_valid?1:0)|(s.command_valid?2:0)|(s.limited?4:0)|(s.synced?8:0)|(s.active?16:0)|(s.stick.flags&1?32:0);
  const bool pending=s.tx.calculated_valid&&(!s.command_valid||s.tx.calculated_right!=s.right_command||s.tx.calculated_left!=s.left_command);
  if(direction!=motion_direction_){
   r.allowed=direction?(motion_direction_?3:1):2;pushMotion(r);
   if(!direction){
    final_pending_=s.active&&pending;final_failures_=s.tx.goal_failed;
    if(latest_stick_.type==Motion)stick_pending_=false;
   }else final_pending_=false;
   motion_direction_=direction;
  }
  if(final_pending_&&(!pending||!s.active||s.tx.goal_failed!=final_failures_)){
   r.allowed=4;pushMotion(r);final_pending_=false;
   if(latest_stick_.type==Motion)stick_pending_=false;
  }
  if(direction||final_pending_){r.allowed=0;latest_stick_=r;stick_pending_=true;}
 }
 Record records_[Capacity],latest_input_,latest_stick_,latest_feedback_;
 uint32_t follow_sample_[2]{};
 uint8_t follow_status_[2]{};
 bool feedback_pending_=false,feedback_turn_=true;
 uint32_t observed_now_=0,continuous_ms_=0,calibration_revision_=0,seen_refusals_=0;
 bool input_pending_=false,stick_pending_=false;
 uint32_t attempt_=0,rx_ms_=0,config_ms_=0,lost_=0,measure_revision_=0,measure_log_ms_=0,measure_rejected_ms_=0;
 uint8_t head_=0,count_=0,raw_=0,allowed_=255,operation_=255,blocker_=255,
  recovery_=0,directions_=0,phase_=0,config_status_=0,decision_keys_=255,measure_status_=0;
 bool synced_=false,stick_seen_=false;
 uint32_t stick_raw_=0;uint8_t stick_flags_=0;
 float last_height_=0,last_roll_=0,last_drive_=0,last_yaw_=0;
 void push(const Record&r){
  if(count_==Capacity){if(lost_!=UINT32_MAX)++lost_;return;}
  records_[(head_+count_)%Capacity]=r;++count_;
 }
};
inline const char* serialReason(uint8_t n){
 static const char*const names[]={"LEFT_CENTER_REQUIRED","IDLE","SYNCING","ACCEPTED","RIGHT_INPUT_PRIORITY","STOPPED",
  "IMU_INVALID","POSITION_RANGE","REPLY_TIMEOUT","REPLY_INVALID","TX_FAILED","CLOCK_GAP","TX_BUSY",
  "WHEEL_RETRY","ROLL_UNCONFIRMED","INITIALIZING","START_WAITING","EXPLICIT_STOP","CALIBRATING",
  "POLARITY_TEST","ACCEPTANCE_INVALID","INWARD_ONLY","LEFT_CENTER_THEN_DEFLECT","PAIR_LIMIT","READ_ONLY","CONFIG_BUSY","MEASURE_READ_ONLY","MEASURE_STOP_REQUIRED","RETURN_DIRECTIONS_CONFLICT","LEG_CALIBRATION_REQUIRED"};
 return n<sizeof(names)/sizeof(*names)?names[n]:"NONE";
}
// Consumer-only formatter. A maximum 48-byte write, and no call when TX is full.
// Each record can yield several bounded lines. No raw data except failed RX.
class SerialOutput {
 public:
 static constexpr unsigned LineCapacity=192;
 bool idle()const{return size_==offset_&&!pending_;}
 void begin(const SerialDiagnostics::Record&r,uint32_t lost){record_=r;lost_=lost;pending_=true;part_=0;}
 void loss(uint32_t lost){lost_+=lost;}
 template<class Sink> void service(Sink&sink){
  if(offset_==size_){
   if(lost_){size_=snprintf(line_,sizeof(line_),"LEG LOG: dropped=%lu (diagnostic queue full)\n",(unsigned long)lost_);lost_=0;offset_=0;}
   else if(pending_){format();offset_=0;}
   else return;
   if(size_>=sizeof(line_))size_=sizeof(line_)-1;
  }
  const int room=sink.availableForWrite();if(room<=0)return;
  size_t n=size_-offset_;if(n>SerialDiagnostics::Chunk)n=SerialDiagnostics::Chunk;if(n>size_t(room))n=size_t(room);
  if(n)offset_+=sink.write(reinterpret_cast<const uint8_t*>(line_+offset_),n);
 }
 private:
 SerialDiagnostics::Record record_;
 char line_[LineCapacity]{};
 uint32_t lost_=0;
 size_t size_=0,offset_=0;
 uint8_t part_=0;
 bool pending_=false;
 const char* keys(){
  static const char*const names[]={"RELEASE","UP","DOWN","UP+DOWN","LEFT","UP+LEFT","DOWN+LEFT","UP+DOWN+LEFT",
   "RIGHT","UP+RIGHT","DOWN+RIGHT","UP+DOWN+RIGHT","LEFT+RIGHT","UP+LEFT+RIGHT","DOWN+LEFT+RIGHT","ALL"};
  return names[record_.keys&15];
 }
 void format(){
  using D=SerialDiagnostics;
  const auto&r=record_;const auto&s=r.data.sync;const auto&c=r.data.config;const auto&x=r.data.rx;
  pending_=false;
  switch(r.type){
   case D::Input:size_=snprintf(line_,sizeof(line_),"LEG INPUT: LEFT_STICK %s received; release_gate=%u ms=%lu\n",keys(),r.allowed,(unsigned long)r.ms);break;
   case D::Decision:
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG INPUT: LEFT_STICK %s result=%s reason=%s operation=%u ms=%lu\n",keys(),s.recovery==uint8_t(Operation::Sync)?"PENDING_SYNC":r.allowed&&r.reason==255?"ACCEPTED":"REJECTED",serialReason(r.reason),s.recovery,(unsigned long)r.ms);
    else size_=snprintf(line_,sizeof(line_),"LEG RETURN: %s; center LEFT XY, then NEW %s; paired limits remain active\n",s.hint==1?"短縮して通常範囲へ戻す必要あり":"伸長して通常範囲へ戻す必要あり",s.hint==1?"DOWN":"UP");
    pending_=(r.reason==uint8_t(Reason::RecoveryDirection)||r.reason==uint8_t(Reason::RecoveryRelease))&&s.hint>0&&s.hint<3&&part_++==0;break;
   case D::SyncStart:size_=snprintf(line_,sizeof(line_),"LEG %s: START ms=%lu\n",r.allowed?"READ_ONLY":"SYNC",(unsigned long)r.ms);break;
   case D::SyncFailed:size_=snprintf(line_,sizeof(line_),"LEG SYNC: FAILED reason=%s ms=%lu\n",serialReason(r.reason),(unsigned long)r.ms);break;
   case D::SyncDone:
    if(!part_&&validGeometry(s.geometry))size_=snprintf(line_,sizeof(line_),"LEG %s: right=%d (%.3f%%) left=%d (%.3f%%) result=%s mask=%u\n",r.allowed?"READ_ONLY":"SYNC",s.right,double(percent(true,s.right,s.geometry)),s.left,double(percent(false,s.left,s.geometry)),r.allowed?"OBSERVED_NO_MOTION":"ACCEPTED",s.acquired);
    else if(!part_)size_=snprintf(line_,sizeof(line_),"LEG READ_ONLY: right=%d left=%d tick; percent/ranges unavailable (leg calibration required); no motion\n",s.right,s.left);
    else if(part_==1&&!validGeometry(s.geometry))size_=snprintf(line_,sizeof(line_),"LEG LIMITS: UNAVAILABLE; explicit leg calibration required (not outside a measured range)\n");
    else if(part_==1)size_=snprintf(line_,sizeof(line_),"LEG LIMITS: initial R=%d..%d L=%d..%d; normal R=%d..%d L=%d..%d (servo travel 0..30%% intersect margins)\n",s.accept.right_min,s.accept.right_max,s.accept.left_min,s.accept.left_max,normalMinTick(true,s.geometry),normalMaxTick(true,s.geometry),normalMinTick(false,s.geometry),normalMaxTick(false,s.geometry));
    else if(part_==2)size_=snprintf(line_,sizeof(line_),"LEG GEOMETRY: measured 0/100%% R=%d/%d L=%d/%d; margins R=%d/%d L=%d/%d tick; not linear height\n",zeroTick(true,s.geometry),fullTick(true,s.geometry),zeroTick(false,s.geometry),fullTick(false,s.geometry),RightRetractedMarginTicks,RightExtendedMarginTicks,LeftRetractedMarginTicks,LeftExtendedMarginTicks);
    else if(validGeometry(s.geometry))size_=snprintf(line_,sizeof(line_),"LEG PROFILE: max_height=%.0f pp/s; normal_pct R=%.3f..%.3f L=%.3f..%.3f; UART=50 ms speed=150 acc=15\n",double(HeightRate),double(normalLow(true,s.geometry)),double(normalHigh(true,s.geometry)),double(normalLow(false,s.geometry)),double(normalHigh(false,s.geometry)));
    else size_=snprintf(line_,sizeof(line_),"LEG PROFILE: max_height=%.0f pp/s; normal range UNAVAILABLE (calibration required)\n",double(HeightRate));
    pending_=++part_<4;break;
   case D::Range:size_=snprintf(line_,sizeof(line_),s.recovery?
     "LEG STATE: HOLD_OUTSIDE_RANGE; center LEFT XY, then NEW inward deflection; UP=%u DOWN=%u; awaiting_release=%u\n":r.allowed?"LEG STATE: NORMAL_RANGE\n":"LEG STATE: INACTIVE\n",
     unsigned(bool(s.directions&1)),unsigned(bool(s.directions&2)),unsigned(s.recovery==1));break;
   case D::Receive:
    if(!part_){size_=snprintf(line_,sizeof(line_),"LEG RX FAILED: %s ID=%u reg=%u requested=%u received=%u issues=%02X%s%s%s%s%s%s elapsed=%lu ms\n",
     x.target==RightId?"RIGHT":x.target==LeftId?"LEFT":"UNKNOWN",x.target,x.reg,x.requested,x.received,x.issues,
     x.issues&RxHeader?" HEADER":"",x.issues&RxId?" ID":"",x.issues&RxLength?" LENGTH":"",x.issues&RxStatus?" STATUS":"",x.issues&RxChecksum?" CHECKSUM":"",x.issues&RxTimeout?" TIMEOUT":"",(unsigned long)x.elapsed_ms);}
    else if(part_==1){size_=snprintf(line_,sizeof(line_),"LEG RX EXPECT: id=%u len=%u status=0; actual id=%u len=%u status=%u; checksum expected=%02X actual=%02X valid=%u\n",
      x.target,x.requested+2,x.frame_size>2?x.frame[2]:255,x.frame_size>3?x.frame[3]:255,x.frame_size>4?x.frame[4]:255,x.expected_checksum,x.received_checksum,x.checksum_ready);}
    else {size_=snprintf(line_,sizeof(line_),"LEG RX RAW: ");for(unsigned i=0;i<x.raw_size&&i<sizeof(x.raw);++i)size_+=snprintf(line_+size_,sizeof(line_)-size_,"%02X",x.raw[i]);size_+=snprintf(line_+size_,sizeof(line_)-size_," truncated=%u\n",x.truncated);}
    pending_=++part_<3;break;
   case D::AcceptanceOutside:
    size_=snprintf(line_,sizeof(line_),"LEG ACCEPTANCE: 初回受付範囲外 (OUTSIDE_INITIAL_RANGE) right=%d (%.3f%%) limits=%d..%d left=%d (%.3f%%) limits=%d..%d; no preload/ON\n",
     s.right,double(percent(true,s.right,s.geometry)),s.accept.right_min,s.accept.right_max,s.left,double(percent(false,s.left,s.geometry)),s.accept.left_min,s.accept.left_max);break;
   case D::MeasureRejected:size_=snprintf(line_,sizeof(line_),"LEG MEASURE: REJECTED reason=%s ms=%lu\n",serialReason(r.reason),(unsigned long)r.ms);break;
   case D::Measure:{
    const auto&m=r.data.measured.value;const auto&g=r.data.measured.geometry;
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s start=%lu end=%lu reason=%s; READ ONLY, limits unchanged\n",
     m.status==1?"ACTIVE":m.status==2?"STOPPED":"FAILED",(unsigned long)m.started_ms,(unsigned long)m.finished_ms,m.status==3?serialReason(m.error):"NONE");
    else {const unsigned side=part_-1;const bool have=m.acquired&(1u<<side);
     if(have&&!validGeometry(g))size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s current=%d min=%d max=%d tick count=%lu; pct=UNAVAILABLE (leg calibration required)\n",side?"LEFT":"RIGHT",m.latest[side],m.minimum[side],m.maximum[side],(unsigned long)m.count[side]);
     else if(have)size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s current=%d min=%d max=%d tick pct=%.3f/%.3f/%.3f sample=%lu count=%lu valid=%u\n",
      side?"LEFT":"RIGHT",m.latest[side],m.minimum[side],m.maximum[side],double(percent(side==0,m.latest[side],g)),double(percent(side==0,m.minimum[side],g)),double(percent(side==0,m.maximum[side],g)),(unsigned long)m.sample_ms[side],(unsigned long)m.count[side],unsigned(bool(m.valid&(1u<<side))));
     else size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s UNREAD (not zero)\n",side?"LEFT":"RIGHT");
    }
    pending_=++part_<3;break;
   }
   case D::Sticks:{
    const auto&i=r.data.stick;
    if(!part_)size_=snprintf(line_,sizeof(line_),"PS4 AXES: LX=%d LY=%d RX=%d RY=%d mapped_leg=%+.3f/%+.3f DZ=%d center=%d flags=%02X ms=%lu\n",
     int(i.lx),int(i.ly),int(i.rx),int(i.ry),double(i.height),double(i.roll),ps4_control::LegDeadzone,ps4_control::LegCenter,unsigned(i.flags),(unsigned long)i.ms);
    else size_=snprintf(line_,sizeof(line_),"PS4 APPLIED: leg_height=%+.3f leg_roll=%+.3f drive_request=%+.3f m/s yaw_request=%+.3f V (before profile/limits)\n",
     double(i.applied_height),double(i.applied_roll),double(i.drive),double(i.yaw));
    pending_=part_++==0;break;
   }
   case D::Motion:{
    const auto&m=r.data.motion;const auto&t=m.tx;
    static const char*const event[]={"PROGRESS","START","END","CHANGE","FINAL_UART"};
    static const char*const torque[]={"UNKNOWN","OFF_PENDING","OFF_SENT_UNCONFIRMED","ON_SENT_UNCONFIRMED"};
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG MOTION: %s ms=%lu raw_LX_LY=%d/%d applied=%+.3f/%+.3f calc_RL=%d/%d valid=%u calc_ms=%lu\n",
     r.allowed<5?event[r.allowed]:"UNKNOWN",(unsigned long)r.ms,int(m.lx),int(m.ly),double(m.height),double(m.roll),t.calculated_right,t.calculated_left,unsigned(bool(m.flags&1)),(unsigned long)t.calculated_ms);
    else if(part_==1)size_=snprintf(line_,sizeof(line_),"LEG UART: last_RL=%d/%d valid=%u last_ms=%lu goal_sent=%lu wait=%lu failed=%lu flags=%02X\n",
     m.sent_right,m.sent_left,unsigned(bool(m.flags&2)),(unsigned long)m.sent_ms,(unsigned long)t.goals,(unsigned long)t.goal_wait,(unsigned long)t.goal_failed,unsigned(m.flags));
    else if(part_==2)size_=snprintf(line_,sizeof(line_),"LEG TORQUE: %s on_sent=%lu wait=%lu failed=%lu reason=%s blocker=%s edge_coalesced=%lu\n",
     m.torque<4?torque[m.torque]:"UNKNOWN",(unsigned long)t.on_sent,(unsigned long)t.on_wait,(unsigned long)t.on_failed,serialReason(m.reason),serialReason(m.blocker),(unsigned long)m.coalesced);
    if(part_==3)size_=snprintf(line_,sizeof(line_),"LEG ROLL: AngleX=%+.2f left_positive=%+.2f target_requested=%+.2f applied=%+.2f deg; limited=%u; LX negative=LEFT (physical direction to verify)\n",
     double(m.measured/ImuRollSign),double(m.measured),double(m.requested),double(m.applied),unsigned(bool(m.flags&4)));
    pending_=++part_<4;break;
   }
   case D::FollowingRead:{
    const auto&f=r.data.following.side[part_];
    static const char*const status[]={"UNREAD","WAITING","VALID","FAILED","CANCELLED","TX_BUSY","TX_FAILED"};
    const uint32_t age=r.ms-f.sample_ms;
    const bool valid=f.status==2&&age<=FollowFreshMs;
    char error[16];if(valid)snprintf(error,sizeof(error),"%ld",long(f.tick)-long(f.command));else strcpy(error,"UNKNOWN");
    size_=snprintf(line_,sizeof(line_),"LEG FEEDBACK: %s %s tick=%d ms=%lu read=%lu cmd_at_read=%d cmd_ms=%lu error=%s age=%lu issues=%02X\n",
     part_?"L":"R",f.status==2&&!valid?"STALE":f.status<7?status[f.status]:"UNKNOWN",f.tick,(unsigned long)f.sample_ms,(unsigned long)f.request_ms,
     f.command,(unsigned long)f.command_ms,error,(unsigned long)age,unsigned(f.issues));
    pending_=++part_<2;break;
   }
   case D::Calibration:{
    const auto&v=r.data.calibration.value;const auto&g=r.data.calibration.geometry;
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG CALIBRATION: phase=%u point=%u mask=%u result=%s generation=%lu source=%s ms=%lu\n",v.phase,v.point,v.mask,calResult(v.result),(unsigned long)v.generation,v.source?"NVS":"NONE",(unsigned long)v.event_ms);
    else if(part_==1)size_=snprintf(line_,sizeof(line_),"LEG CALIBRATION CANDIDATE: R0=%d R100=%d L0=%d L100=%d; save=%s; no motion\n",v.candidate.right_zero,v.candidate.right_full,v.candidate.left_zero,v.candidate.left_full,calResult(v.canSave()));
    else if(part_==2)size_=snprintf(line_,sizeof(line_),"LEG CALIBRATION ACTIVE: valid=%u R0=%d R100=%d L0=%d L100=%d; NVS=%s\n",unsigned(validGeometry(g)),g.right_zero,g.right_full,g.left_zero,g.left_full,calResult(v.storage));
    else if(validGeometry(v.candidate))size_=snprintf(line_,sizeof(line_),"LEG CALIBRATION REVIEW: R dir=- span=%d normal=%d..%d L dir=+ span=%d normal=%d..%d; margin=10 tick normal=0..30%%\n",v.candidate.right_zero-v.candidate.right_full,normalMinTick(true,v.candidate),normalMaxTick(true,v.candidate),v.candidate.left_full-v.candidate.left_zero,normalMinTick(false,v.candidate),normalMaxTick(false,v.candidate));
    else size_=snprintf(line_,sizeof(line_),"LEG CALIBRATION REVIEW: incomplete/invalid; %s\n",calResult(v.canSave()));
    pending_=++part_<4;break;
   }
   case D::Configuration:
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG CONFIG: %s reason=%s current_mask=%u; mode 0=POSITION 1=WHEEL\n",c.status==2?"COMPLETE":"FAILED",c.status==2?"NONE":serialReason(c.error),c.current);
    else {const unsigned side=part_-1;size_=snprintf(line_,sizeof(line_),"LEG CONFIG: %s mode=%u limits=%u..%u tick acquired=%u current=%u\n",side?"LEFT":"RIGHT",c.mode[side],c.minimum[side],c.maximum[side],unsigned((c.acquired>>(side*2))&3),unsigned((c.current>>(side*2))&3));}
    pending_=++part_<3;break;
  }
 }
};
}

#pragma once
#include "LegControl.h"
#include <stdio.h>
#include <string.h>
namespace leg {
// Single producer / single consumer; caller protects push/pop with its RTOS lock.
// No formatting, Serial access, allocation or waits in capture/input/pop.
class SerialDiagnostics {
 public:
 enum Type:uint8_t { Input,Decision,SyncStart,SyncDone,Range,Receive,Configuration,SyncFailed,AcceptanceOutside,Measure,MeasureRejected };
 struct Sync { Acceptance accept;int16_t right,left;uint8_t acquired,recovery,directions; };
 struct Record {
  uint32_t ms=0;
  Type type=Input;uint8_t keys=0,allowed=0,reason=0;
  union Payload { Sync sync;RxFailure rx;ServoConfiguration config;Measurement measurement;Payload():rx{}{} } data;
 };
 static constexpr unsigned Capacity=8,Chunk=48;
 void input(uint8_t keys,bool gate,uint32_t now){
  if(keys==raw_)return;
  raw_=keys;Record r;r.type=Input;r.ms=now;r.keys=keys;r.allowed=gate;
  push(r);
 }
 void capture(const State&s,uint8_t phase,uint32_t now){
  const auto&o=s.operation;
  if(o.committed&&(o.allowed!=allowed_||o.operation!=operation_||o.blocker!=blocker_||o.raw_keys!=decision_keys_)){
   Record r;r.type=Decision;r.ms=now;r.keys=o.raw_keys;r.allowed=o.allowed;
   r.reason=o.blocker;r.data.sync.recovery=o.operation;
   // Replace only an unsent input-only event from this same processing batch.
   if(count_&&records_[(head_+count_-1)%Capacity].type==Input&&records_[(head_+count_-1)%Capacity].keys==r.keys)
    records_[(head_+count_-1)%Capacity]=r;
   else push(r);
  }
  if(s.positions.attempt_ms!=attempt_){
   attempt_=s.positions.attempt_ms;Record r;r.type=SyncStart;r.ms=now;r.allowed=s.positions.read_only;push(r);
  }
  const bool read_done=s.positions.read_only&&s.positions.current==3&&phase_>=1&&phase_<=4&&phase==0&&
   (s.reason==Reason::Idle||s.reason==Reason::PositionRange);
  if((s.synced&&!synced_)||read_done){
   Record r;r.type=SyncDone;r.ms=now;r.data.sync.accept=s.acceptance;
   r.data.sync.right=decodePosition(s.positions.word[0]);r.data.sync.left=decodePosition(s.positions.word[1]);
   r.data.sync.acquired=s.positions.current;r.allowed=read_done;push(r);
  }
  if(s.recovery!=recovery_||s.return_directions!=directions_){
   Record r;r.type=Range;r.ms=now;r.data.sync.recovery=s.recovery;
   r.data.sync.directions=s.return_directions;r.allowed=s.active||s.synced;
   if(s.recovery==1)r.data.sync.directions=returnDirections(s.right_percent,s.left_percent);
   push(r);
  }
  if((phase_>=1&&phase_<=6)&&(phase==0||phase==7)&&!s.synced&&!s.active&&!read_done){
   Record r;r.type=SyncFailed;r.ms=now;r.reason=uint8_t(s.reason);push(r);
   if(s.reason==Reason::PositionRange&&s.positions.current==3){
    r.type=AcceptanceOutside;r.data.sync.accept=s.acceptance;
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
   Record r;r.type=Measure;r.ms=now;r.data.measurement=m;push(r);
   measure_revision_=m.revision;measure_status_=m.status;measure_log_ms_=now;
  }
  if(m.rejected_ms&&m.rejected_ms!=measure_rejected_ms_){
   Record r;r.type=MeasureRejected;r.ms=m.rejected_ms;r.reason=m.reject_reason;push(r);measure_rejected_ms_=m.rejected_ms;
  }
  if(!m.rejected_ms)measure_rejected_ms_=0;
  synced_=s.synced;recovery_=s.recovery;directions_=s.return_directions;phase_=phase;
  allowed_=o.allowed;operation_=o.operation;blocker_=o.blocker;decision_keys_=o.raw_keys;
  config_ms_=c.finished_ms;config_status_=c.status;
 }
 bool pop(Record&r,uint32_t&lost){
  lost=lost_;lost_=0;
  if(!count_)return false;
  r=records_[head_];head_=(head_+1)%Capacity;--count_;return true;
 }
 private:
 Record records_[Capacity];
 uint32_t attempt_=0,rx_ms_=0,config_ms_=0,lost_=0,measure_revision_=0,measure_log_ms_=0,measure_rejected_ms_=0;
 uint8_t head_=0,count_=0,raw_=0,allowed_=255,operation_=255,blocker_=255,
  recovery_=0,directions_=0,phase_=0,config_status_=0,decision_keys_=255,measure_status_=0;
 bool synced_=false;
 void push(const Record&r){
  if(count_==Capacity){if(lost_!=UINT32_MAX)++lost_;return;}
  records_[(head_+count_)%Capacity]=r;++count_;
 }
};
inline const char* serialReason(uint8_t n){
 static const char*const names[]={"RELEASE_REQUIRED","IDLE","SYNCING","ACCEPTED","DRIVING","STOPPED",
  "IMU_INVALID","POSITION_RANGE","REPLY_TIMEOUT","REPLY_INVALID","TX_FAILED","CLOCK_GAP","TX_BUSY",
  "WHEEL_RETRY","ROLL_UNCONFIRMED","INITIALIZING","START_WAITING","EXPLICIT_STOP","CALIBRATING",
  "POLARITY_TEST","ACCEPTANCE_INVALID","INWARD_ONLY","RELEASE_THEN_PRESS","PAIR_LIMIT","READ_ONLY","CONFIG_BUSY","MEASURE_READ_ONLY","MEASURE_STOP_REQUIRED","RETURN_DIRECTIONS_CONFLICT"};
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
   case D::Input:size_=snprintf(line_,sizeof(line_),"LEG INPUT: %s received; release_gate=%u ms=%lu\n",keys(),r.allowed,(unsigned long)r.ms);break;
   case D::Decision:size_=snprintf(line_,sizeof(line_),"LEG INPUT: %s result=%s reason=%s operation=%u ms=%lu\n",keys(),s.recovery==uint8_t(Operation::Sync)?"PENDING_SYNC":r.allowed&&r.reason==255?"ACCEPTED":"REJECTED",serialReason(r.reason),s.recovery,(unsigned long)r.ms);break;
   case D::SyncStart:size_=snprintf(line_,sizeof(line_),"LEG %s: START ms=%lu\n",r.allowed?"READ_ONLY":"SYNC",(unsigned long)r.ms);break;
   case D::SyncFailed:size_=snprintf(line_,sizeof(line_),"LEG SYNC: FAILED reason=%s ms=%lu\n",serialReason(r.reason),(unsigned long)r.ms);break;
   case D::SyncDone:
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG %s: right=%d (%.3f%%) left=%d (%.3f%%) result=%s mask=%u\n",r.allowed?"READ_ONLY":"SYNC",s.right,double(percent(true,s.right)),s.left,double(percent(false,s.left)),r.allowed?"OBSERVED_NO_MOTION":"ACCEPTED",s.acquired);
    else if(part_==1)size_=snprintf(line_,sizeof(line_),"LEG LIMITS: initial R=%d..%d L=%d..%d; normal R=%d..%d L=%d..%d (servo travel 0..30%% intersect margins)\n",s.accept.right_min,s.accept.right_max,s.accept.left_min,s.accept.left_max,normalMinTick(true),normalMaxTick(true),normalMinTick(false),normalMaxTick(false));
    else size_=snprintf(line_,sizeof(line_),"LEG GEOMETRY: measured 0/100%% R=%d/%d L=%d/%d; margins R=%d/%d L=%d/%d tick; not linear height\n",RightHome,RightExtended,LeftHome,LeftExtended,RightRetractedMarginTicks,RightExtendedMarginTicks,LeftRetractedMarginTicks,LeftExtendedMarginTicks);
    pending_=++part_<3;break;
   case D::Range:size_=snprintf(line_,sizeof(line_),s.recovery?
     "LEG STATE: HOLD_OUTSIDE_RANGE; release keys, then NEW inward press; UP=%u DOWN=%u; awaiting_release=%u\n":r.allowed?"LEG STATE: NORMAL_RANGE\n":"LEG STATE: INACTIVE\n",
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
     s.right,double(percent(true,s.right)),s.accept.right_min,s.accept.right_max,s.left,double(percent(false,s.left)),s.accept.left_min,s.accept.left_max);break;
   case D::MeasureRejected:size_=snprintf(line_,sizeof(line_),"LEG MEASURE: REJECTED reason=%s ms=%lu\n",serialReason(r.reason),(unsigned long)r.ms);break;
   case D::Measure:{
    const auto&m=r.data.measurement;
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s start=%lu end=%lu reason=%s; READ ONLY, limits unchanged\n",
     m.status==1?"ACTIVE":m.status==2?"STOPPED":"FAILED",(unsigned long)m.started_ms,(unsigned long)m.finished_ms,m.status==3?serialReason(m.error):"NONE");
    else {const unsigned side=part_-1;const bool have=m.acquired&(1u<<side);
     if(have)size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s current=%d min=%d max=%d tick pct=%.3f/%.3f/%.3f sample=%lu count=%lu valid=%u\n",
      side?"LEFT":"RIGHT",m.latest[side],m.minimum[side],m.maximum[side],double(percent(side==0,m.latest[side])),double(percent(side==0,m.minimum[side])),double(percent(side==0,m.maximum[side])),(unsigned long)m.sample_ms[side],(unsigned long)m.count[side],unsigned(bool(m.valid&(1u<<side))));
     else size_=snprintf(line_,sizeof(line_),"LEG MEASURE: %s UNREAD (not zero)\n",side?"LEFT":"RIGHT");
    }
    pending_=++part_<3;break;
   }
   case D::Configuration:
    if(!part_)size_=snprintf(line_,sizeof(line_),"LEG CONFIG: %s reason=%s current_mask=%u; mode 0=POSITION 1=WHEEL\n",c.status==2?"COMPLETE":"FAILED",c.status==2?"NONE":serialReason(c.error),c.current);
    else {const unsigned side=part_-1;size_=snprintf(line_,sizeof(line_),"LEG CONFIG: %s mode=%u limits=%u..%u tick acquired=%u current=%u\n",side?"LEFT":"RIGHT",c.mode[side],c.minimum[side],c.maximum[side],unsigned((c.acquired>>(side*2))&3),unsigned((c.current>>(side*2))&3));}
    pending_=++part_<3;break;
  }
 }
};
}

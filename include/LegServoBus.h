#pragma once
#include "LegControl.h"
namespace leg {
// IO is control-task-owned UART. No flush, response wait, heap or retry loop.
// At most 16 received bytes and one capacity-checked contiguous TX per service.
template<class IO> class ServoBus {
 public:
 uint8_t stage(const Control&c)const{return off_pending_?7:phase_==Idle&&c.state.active?8:uint8_t(phase_);}
 bool busy()const{return phase_!=Idle||off_pending_;}
 bool inspecting()const{return config_active_;}
 bool observing()const{return observe_only_||config_active_;}
 int16_t readPosition(bool right)const{return positions_[right?0:1];}
 bool observe(Control&c){
  if(busy())return false;
  c.stop(Reason::Syncing);c.want_sync=true;observe_only_=true;return true;
 }
 bool inspect(Control&c,uint32_t now){
  if(busy()||c.state.active||c.want_sync){c.state.configuration.rejected_ms=now;c.state.configuration.reject_reason=uint8_t(Reason::ConfigBusy);return false;}
  c.stop(Reason::ConfigReadOnly);auto&cfg=c.state.configuration;
  cfg.started_ms=now;cfg.current=0;cfg.step=0;cfg.status=1;cfg.error=0;
  config_active_=true;phase_=ConfigSend;reply_.clearFrame();return true;
 }
 void off(Control& c,uint32_t now=0){
  if(config_active_){c.state.configuration.status=3;c.state.configuration.error=uint8_t(Reason::Stopped);c.state.configuration.finished_ms=now;}
  c.stop(Reason::Stopped);phase_=Idle;reply_.clearFrame();observe_only_=false;config_active_=false;
  if(off_pending_||c.state.torque==2)return;
  off_pending_=true;c.state.torque=1;next_send_=0;
 }
 void cancel(){phase_=Idle;reply_.clearFrame();observe_only_=false;config_active_=false;}
 void service(IO& io,Control& c,uint32_t now){
  serviceOnce(io,c,now);c.observe(Source::Bus,now,stage(c));
 }
 private:
 void serviceOnce(IO& io,Control& c,uint32_t now){
  if(config_active_&&uint32_t(now-c.state.configuration.started_ms)>1000){fail(c,c.state.operation.tx_wait?Reason::TxBusy:Reason::ReplyTimeout,now);return;}
  // The original whole-request deadline is not extended by bytes or echoes.
  if(waiting()&&reply_.expired(now,ReplyTimeoutMs)){
   reply_.timeout();failReceive(c,now);
  }
  for(unsigned i=0;i<16&&io.available()>0;++i){
   const int byte=io.read();if(byte<0)break;
   if(!waiting())continue;
   const auto result=reply_.feed(uint8_t(byte));
   if(result==ServoReply::Bad){failReceive(c,now);break;}
   if(result!=ServoReply::Good)continue;
   if(phase_==ConfigWait){
    auto&cfg=c.state.configuration;const unsigned side=cfg.step/2;
    if(!(cfg.step&1)){
     cfg.minimum[side]=uint16_t(reply_.parameter(0))|(uint16_t(reply_.parameter(1))<<8);
     cfg.maximum[side]=uint16_t(reply_.parameter(2))|(uint16_t(reply_.parameter(3))<<8);
    }else cfg.mode[side]=reply_.parameter(0);
    cfg.sample_ms[cfg.step]=now;cfg.acquired|=uint8_t(1u<<cfg.step);cfg.current|=uint8_t(1u<<cfg.step);
    reply_.clearFrame();
    if(++cfg.step==4){cfg.status=2;cfg.finished_ms=now;phase_=Idle;config_active_=false;c.stop(Reason::ConfigReadOnly);}
    else phase_=ConfigSend;
    continue;
   }
   const unsigned side=phase_==WaitRight?0:1;
   c.state.positions.capture(side,reply_.positionWord(),now);
   const int16_t pos=reply_.position();
   reply_.clearFrame();
   if(phase_==WaitRight){positions_[0]=pos;first_ms_=now;phase_=ReadLeft;}
   else {
    positions_[1]=pos;
    if(uint32_t(now-first_ms_)>SyncMaxAgeMs){fail(c,Reason::ReplyTimeout,now);break;}
    if(!c.seed(positions_[0],positions_[1],now)){phase_=Idle;observe_only_=false;break;}
    if(observe_only_){observe_only_=false;c.stop(Reason::Idle);phase_=Idle;}
    else phase_=Seed;
   }
  }
  if(int32_t(now-next_send_)<0)return;
  if(off_pending_){if(torque(io,c,false,now))off_pending_=false;return;}
  if(phase_==ConfigSend){
   const unsigned step=c.state.configuration.step;
   uint8_t p[]={255,255,uint8_t(step<2?RightId:LeftId),4,2,uint8_t(step&1?33:9),uint8_t(step&1?1:4),0};
   checksum(p,8);
   if(send(io,c,p,8,now)){reply_.begin(p[2],now,p[5],p[6]);phase_=ConfigWait;}
   return;
  }
  if(c.want_sync&&phase_==Idle){c.state.positions.begin(now,observe_only_);phase_=ReadRight;reply_.clearFrame();}
  if(phase_==ReadRight||phase_==ReadLeft){
   uint8_t p[]={255,255,uint8_t(phase_==ReadRight?RightId:LeftId),4,2,56,2,0};
   checksum(p,8);
   if(send(io,c,p,8,now)){reply_.begin(p[2],now);phase_=phase_==ReadRight?WaitRight:WaitLeft;}
   return;
  }
  if((phase_==Seed||phase_==Enable)&&uint32_t(now-first_ms_)>SyncMaxAgeMs){fail(c,Reason::ReplyTimeout,now);return;}
  if(phase_==Seed){
   if(goal(io,c,positions_[0],positions_[1],now))phase_=Enable;
   return;
  }
  if(phase_==Enable){
   if(torque(io,c,true,now)){c.activate(now);phase_=Idle;}
   return;
  }
  if(c.state.active){
   const int16_t right=c.target(true),left=c.target(false);
   if(!c.state.command_valid||right!=c.state.right_command||left!=c.state.left_command)goal(io,c,right,left,now);
  }
 }
 private:
 enum Phase:uint8_t { Idle,ReadRight,WaitRight,ReadLeft,WaitLeft,Seed,Enable,ConfigSend=9,ConfigWait } phase_=Idle;
 ServoReply reply_;
 int16_t positions_[2]{};
 uint32_t first_ms_=0,next_send_=0;
 bool off_pending_=false,observe_only_=false,config_active_=false;
 bool waiting()const{return phase_==WaitRight||phase_==WaitLeft||phase_==ConfigWait;}
 void failReceive(Control&c,uint32_t now){
  c.state.rx_failure=reply_.failure(now);
  fail(c,(c.state.rx_failure.issues&RxTimeout)?Reason::ReplyTimeout:Reason::ReplyInvalid,now);
 }
 static void checksum(uint8_t*p,unsigned n){uint8_t sum=0;for(unsigned i=2;i+1<n;++i)sum+=p[i];p[n-1]=~sum;}
 void fail(Control&c,Reason reason,uint32_t now){
  if(config_active_){c.state.configuration.status=3;c.state.configuration.error=uint8_t(reason);c.state.configuration.finished_ms=now;}
  phase_=Idle;reply_.clearFrame();observe_only_=false;config_active_=false;c.stop(reason);}
 bool send(IO&io,Control&c,uint8_t*p,unsigned n,uint32_t now){
  next_send_=now+SendIntervalMs;
  if(io.availableForWrite()<int(n)){c.state.operation.tx_wait=true;++c.state.tx_busy;c.state.reason=Reason::TxBusy;return false;}
  if(io.write(p,n)!=n){
   const bool read_only=config_active_;fail(c,Reason::TxFailed,now);
   if(!read_only){off_pending_=true;c.state.torque=1;}return false;
  }
  c.state.operation.tx_wait=false;
  if(c.state.reason==Reason::TxBusy)c.state.reason=c.state.active?Reason::Accepted:
      c.want_sync||phase_!=Idle?Reason::Syncing:Reason::Idle;
  reply_.noteTransmit(p,n);++c.state.tx_frames;return true;
 }
 bool torque(IO&io,Control&c,bool on,uint32_t now){
  uint8_t p[]={255,255,254,8,131,40,1,RightId,uint8_t(on),LeftId,uint8_t(on),0};
  checksum(p,sizeof(p));if(!send(io,c,p,sizeof(p),now))return false;
  c.state.torque=on?3:2;return true;
 }
 bool goal(IO&io,Control&c,int16_t right,int16_t left,uint32_t now){
  if(!c.commandAllowed(right,left)){fail(c,Reason::PositionRange,now);return false;}
  uint8_t p[24]={255,255,254,20,131,41,7};
  const uint16_t positions[]={encodePosition(right),encodePosition(left)};
  for(unsigned i=0;i<2;++i){const unsigned at=7+i*8;p[at]=i?LeftId:RightId;p[at+1]=ServoAcceleration;
   p[at+2]=uint8_t(positions[i]);p[at+3]=uint8_t(positions[i]>>8);
   p[at+4]=p[at+5]=0;p[at+6]=uint8_t(ServoSpeed);p[at+7]=uint8_t(ServoSpeed>>8);}
  checksum(p,sizeof(p));if(!send(io,c,p,sizeof(p),now))return false;
  c.state.right_command=right;c.state.left_command=left;c.state.command_ms=now;c.state.command_valid=true;return true;
 }
};
}

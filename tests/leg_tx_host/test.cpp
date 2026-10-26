#include "LegServoBus.h"
#include "LegSerialDiagnostics.h"
#include "Servo_STS3032.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <string>
// The legacy header declares a non-pure base readSCS without a definition.
// Optimized firmware discards its base vtable; ASan retains it. This test-only
// link anchor must never execute; all real frame/SCSerial methods are unchanged.
int SCS::readSCS(unsigned char*,int){assert(false&&"unreachable base readSCS");return 0;}
using namespace leg;
struct IO {
 HardwareSerial reference;
 unsigned goals=0,ons=0;int room=128;bool partial_goal=false,partial_on=false;
 uint8_t rx[8]{};unsigned at=0,size=0;
 int available(){return size-at;}int read(){return rx[at++];}int availableForWrite(){return room;}
 size_t write(const uint8_t*p,size_t n){
  if(n==8){
   const uint16_t v=p[2]==1?1697:2408;uint8_t r[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};
   uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=r[i];r[7]=~sum;memcpy(rx,r,8);at=0;size=8;
  }else if(n==24){
   int16_t positions[]={decodePosition(uint16_t(p[9])|(uint16_t(p[10])<<8)),decodePosition(uint16_t(p[17])|(uint16_t(p[18])<<8))};
   uint8_t ids[]={1,2},acc[]={ServoAcceleration,ServoAcceleration};uint16_t speed[]={ServoSpeed,ServoSpeed};
   reference.bytes.clear();sms_sts.pSerial=&reference;
   // Compile and execute the existing driver, rather than copying its formula.
   sms_sts.SyncWritePosEx(ids,2,positions,speed,acc);
   assert(reference.bytes.size()==n&&!memcmp(p,reference.bytes.data(),n));
   if(partial_goal){partial_goal=false;return n-1;}++goals;
  }else if(n==12){
   reference.bytes.clear();sms_sts.pSerial=&reference;
   if(p[8])sms_sts.on_all_servo();else sms_sts.off_all_servo();
   assert(reference.bytes.size()==n&&!memcmp(p,reference.bytes.data(),n));
   if(p[8]&&partial_on){partial_on=false;return n-1;}
   if(p[8])++ons;
  }else assert(false);
  return n;
 }
};
struct Sink {
 std::string text;int room=48;unsigned writes=0;
 int availableForWrite(){return room;}
 size_t write(const uint8_t*p,size_t n){assert(n<=48&&n<=unsigned(room));text.append(reinterpret_cast<const char*>(p),n);++writes;return n;}
};
void setup(Control&c,ServoBus<IO>&bus,IO&io){
 sms_sts.pSerial=&io.reference;sms_sts.on_all_servo();io.reference.bytes.clear(); // Reference driver enable only, not the tested UART.
 c.input(0,true);c.tick(100,true,false,true,0);c.input(2,true);
 for(unsigned t=101;t<400;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(io.goals==1&&io.ons==1&&c.state.recovery==1);
 c.input(0,true);c.tick(400,true,false,true,0);assert(c.state.recovery==2);
}
int main(){
 activeGeometry()=ConfirmedCandidate;
 // All production UART frames are byte-for-byte compared with the actual old driver.
 Control c;ServoBus<IO>bus;IO io;setup(c,bus,io);c.input(2,true);
 for(unsigned t=401;t<=3400;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(c.target(true)==1822&&c.target(false)==2291&&io.goals>30&&io.ons==1);
 assert(c.state.tx.goals==io.goals&&c.state.tx.on_sent==io.ons);
 // Position TX capacity wait and partial write are distinct from a full UART send.
 io.room=0;const auto sent=c.state.tx.goals;const auto r=c.state.right_command,l=c.state.left_command;
 for(unsigned t=3401;t<=3550;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(c.state.tx.goals==sent&&c.state.tx.goal_wait>0&&c.state.right_command==r&&c.state.left_command==l);
 io.room=128;io.partial_goal=true;
 for(unsigned t=3551;t<=3610;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(c.state.tx.goal_failed==1&&!c.state.active&&c.state.tx.goals==sent);
 // ON capacity wait/short write: never mark a failed frame as ON sent.
 for(bool partial: {false,true}){
  Control x;ServoBus<IO>b;IO port;x.input(0,true);x.tick(100,true,false,true,0);x.input(2,true);
  for(unsigned t=101;t<400;++t){
   x.tick(t,true,false,true,0);
   if(b.stage(x)==6){port.room=partial?128:0;port.partial_on=partial;}
   b.service(port,x,t);
  }
  assert(x.state.tx.on_sent==0&&!port.ons&&!x.state.active);
  assert(partial?x.state.tx.on_failed==1:x.state.tx.on_wait>0);
 }
 // Movement-edge bursts cannot consume critical receive/calibration FIFO slots.
 SerialDiagnostics logs(true);State s;s.active=s.synced=true;s.stick.flags=97;s.tx.calculated_valid=true;
 for(unsigned t=1;t<10000;++t){s.stick.applied_height=t&1?1:0;s.stick.ly=t&1?-128:0;s.operation.raw_keys=t&1?1:0;logs.capture(s,8,t);}
 s.rx_failure.issues=RxTimeout;s.rx_failure.ms=10000;logs.capture(s,8,10000);
 SerialDiagnostics::Record record;uint32_t lost;bool receive=false,coalesced=false;
 for(unsigned cycle=0;cycle<4;++cycle){
 logs.capture(s,8,10000+cycle*500);
 while(logs.pop(record,lost)){assert(!lost);receive|=record.type==SerialDiagnostics::Receive;
  if(record.type==SerialDiagnostics::Motion){assert(receive);coalesced|=record.data.motion.coalesced>0;}
 }
 }
 assert(receive&&coalesced);
 // Formatter remains bounded, explicit about validity/ACK and never waits for USB.
 record={};record.type=SerialDiagnostics::Motion;record.allowed=2;record.reason=1;record.ms=UINT32_MAX;
 record.data.motion.tx=c.state.tx;record.data.motion.tx.goals=UINT32_MAX;record.data.motion.tx.goal_failed=UINT32_MAX;
 record.data.motion.torque=3;record.data.motion.flags=3;record.data.motion.coalesced=UINT32_MAX;
 Sink sink;SerialOutput output;output.begin(record,0);sink.room=0;for(unsigned i=0;i<100;++i)output.service(sink);assert(!sink.writes);
 sink.room=7;for(unsigned i=0;i<1000;++i)output.service(sink);
 assert(sink.text.find("LEG MOTION: END")!=std::string::npos&&sink.text.find("ON_SENT_UNCONFIRMED")!=std::string::npos);
 assert(sink.text.find("edge_coalesced=4294967295")!=std::string::npos);
 printf("PASS: real STS driver byte equivalence for ON/OFF/position and 3s shortening; distinct send/wait/failure; critical FIFO protected, bounded output. State=%zu log=%zu record=%zu TX=%zu\n",sizeof(State),sizeof(logs),sizeof(record),sizeof(TxDiagnostics));
}

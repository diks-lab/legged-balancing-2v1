#include "LegServoBus.h"
#include "LegSerialDiagnostics.h"
#include <cassert>
#include <vector>
#include <string>
#include <new>
#include <cstddef>
#include <cstdlib>
using namespace leg;
struct IO {
 std::vector<uint8_t> rx;unsigned at=0,budget=16,reads[2]{},goals=0,off=0;int capacity=128,mode=0;uint32_t now=0,lastGoal=0;
 int available(){return budget&&at<rx.size();}int read(){--budget;return rx[at++];}int availableForWrite(){return capacity;}
 size_t write(const uint8_t*p,size_t n){
  if(n==8){
   assert(p[5]==56&&p[6]==2);++reads[p[2]-1];rx.clear();at=0;
   if(mode==1)return n; // silent
   if(mode==6)return n-1; // bounded partial READ write
   const auto word=encodePosition(p[2]==1?1980:2144);
   rx={255,255,uint8_t(mode==2?3:p[2]),uint8_t(mode==5?3:4),uint8_t(mode==3?1:0),uint8_t(word),uint8_t(word>>8),0};
   if(mode==5)rx.erase(rx.end()-2);
   uint8_t sum=0;for(unsigned i=2;i+1<rx.size();++i)sum+=rx[i];rx.back()=uint8_t(~sum);
   if(mode==4)rx.back()^=1;
  }else if(n==24){if(goals)assert(now-lastGoal>=50);lastGoal=now;++goals;}
  else {assert(n==12);if(!p[8])++off;}
  return n;
 }
};
struct Sink {std::string text;int availableForWrite(){return 128;}size_t write(const uint8_t*p,size_t n){assert(n<=48);text.append((const char*)p,n);return n;}};
int main(){
 // Poison the exact record storage: old rx-only constructor left motion tail dirty.
 alignas(SerialDiagnostics::Record) unsigned char storage[sizeof(SerialDiagnostics::Record)];
 memset(storage,0xa5,sizeof storage);auto*record=new(storage)SerialDiagnostics::Record;
 assert(record->data.motion.coalesced==0&&record->data.motion.blocker==255);
 static_assert(offsetof(SerialDiagnostics::MotionState,coalesced)>=sizeof(RxFailure),"covers formerly uninitialized tail");
 for(int mode=0;mode<=6;++mode){
  Control c(false);ServoBus<IO>b;IO io;io.mode=mode;
  assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);c.state.torque=3;
  c.state.command_valid=true;c.state.right_command=c.target(true);c.state.left_command=c.target(false);c.state.command_ms=999;
  io.now=1000;b.service(io,c,1000);assert(c.state.following.side[0].status==(mode==6?6:1)&&b.stage(c)==8);
  // A due position command is sent even while a fragmented READ is outstanding.
  c.state.right_percent+=1;io.now=1001;io.budget=1;b.service(io,c,1001);assert(io.goals==1);
  for(unsigned t=1002;t<1020;++t){io.now=t;io.budget=1;b.service(io,c,t);assert(c.state.active&&c.state.synced);}
  const auto&f=c.state.following.side[0];
  if(!mode){assert(f.status==2&&f.tick==1980&&f.command_ms==999&&f.command!=c.state.right_command);}
  else if(mode==6){assert(f.status==6&&!f.sample_ms);}
  else {assert(f.status==3&&f.issues);assert(!f.sample_ms);} // no zero-as-success
  io.mode=0;
  for(unsigned t=1020;t<2600;++t){io.now=t;io.budget=16;b.service(io,c,t);}
  assert(io.reads[0]==2&&io.reads[1]==2&&c.state.following.side[1].tick==2144);
  assert(c.state.following.side[0].status==2&&c.state.active);
  // Stop preempts the pending READ; no wait for response deadline or next poll.
  io.now=3000;io.mode=1;b.service(io,c,3000);
  c.state.stick.flags=32;io.now=3001;b.service(io,c,3001);const unsigned count=io.reads[0]+io.reads[1];
  for(unsigned t=3002;t<3600;++t){io.now=t;b.service(io,c,t);}
  assert(io.reads[0]+io.reads[1]==count); // disconnected: no READ or cached success
  c.state.stick.flags=96;io.now=3600;b.service(io,c,3600);
  b.off(c,3601);io.now=3601;b.service(io,c,3601);
  assert(io.off==1&&!c.state.active&&c.state.torque==2);
  assert(c.state.following.side[0].status==4||c.state.following.side[1].status==4);
 }
 // Diagnostic TX congestion never blocks or changes control eligibility.
 {Control c(false);ServoBus<IO>b;IO io;assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.state.torque=3;c.state.command_valid=true;c.state.right_command=c.target(true);c.state.left_command=c.target(false);
  io.capacity=0;io.now=1000;b.service(io,c,1000);
  assert(c.state.following.side[0].status==5&&c.state.active&&!c.state.operation.tx_wait);
  io.capacity=128;for(unsigned t=1001;t<1500;++t){io.now=t;b.service(io,c,t);}
  assert(io.reads[0]+io.reads[1]==0);io.now=1500;b.service(io,c,1500);assert(io.reads[1]==1);
 }
 // Poisoned, mixed direction/motion/read records stay typed through copies and output.
 SerialDiagnostics log;Control c;Sink sink;SerialOutput out;
 for(unsigned k=1;k<50;++k){
  c.state.active=true;c.state.stick.flags=97;c.state.stick.lx=k&1?-127:0;c.state.stick.ly=k&1?0:127;
  c.state.operation.raw_keys=k&1?4:1;c.state.stick.applied_roll=k&1?1:0;c.state.stick.applied_height=k&1?0:1;
  c.state.requested_roll=.5f;c.state.measured_roll=1.25f;c.state.applied_roll=.4f;
  auto&f=c.state.following.side[k%2];f.status=2;f.tick=1900+k;f.sample_ms=k*1000;f.command=1900;f.command_ms=k*1000-1;
  log.capture(c.state,8,k*1000);
  for(unsigned i=0;i<100;++i){if(out.idle()){SerialDiagnostics::Record r;uint32_t lost;if(log.pop(r,lost))out.begin(r,lost);}out.service(sink);}
 }
 assert(sink.text.find("edge_coalesced=0")!=std::string::npos&&sink.text.find("LEG FEEDBACK:")!=std::string::npos);
 size_t offset=0;while((offset=sink.text.find("edge_coalesced=",offset))!=std::string::npos){
  offset+=15;assert(strtoul(sink.text.c_str()+offset,nullptr,10)<=50);
 }
 assert(sink.text.find("1073563328")==std::string::npos&&sink.text.find("left_positive=+1.25")!=std::string::npos);
 printf("PASS: poisoned union initialized, mixed typed diagnostics; alternating validated low-rate READ, split/ID/status/checksum/length/timeout, goals/OFF priority and non-fatal recovery\n");
}

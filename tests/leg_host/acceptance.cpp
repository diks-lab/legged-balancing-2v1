#include "LegServoBus.h"
#include "DiagnosticTelemetry.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using namespace leg;
// These are fictitious mechanically verified limits FOR HOST TESTS ONLY.
constexpr Acceptance verified{1675,2007,2071,2412,true};
struct IO {
 uint8_t rx[32]{},packets[512][24]{};unsigned head=0,tail=0,writes=0,sizes[512]{},at[512]{};
 uint16_t words[2]={1874,2207};int capacity=128,budget=16;bool echo=false,silent=false,partial=false,badMode=false;
 uint32_t now=0;
 int available(){return budget>0?int(tail-head):0;}
 int read(){--budget;return head<tail?rx[head++]:-1;}
 int availableForWrite(){return capacity;}
 size_t write(const uint8_t*p,unsigned n){
  assert(writes<512);memcpy(packets[writes],p,n);sizes[writes]=n;at[writes++]=now;
  if(n==8&&!silent){
   head=tail=0;if(echo){memcpy(rx,p,8);tail=8;}
   uint8_t response[10]={255,255,p[2],uint8_t(p[6]+2),0};
   if(p[5]==56){const auto w=words[p[2]==1?0:1];response[5]=w;response[6]=w>>8;}
   else if(p[5]==9){response[5]=p[2]==1?0x34:0;response[6]=p[2]==1?0x12:0;
    response[7]=p[2]==1?0xcd:0xff;response[8]=p[2]==1?0xab:0x0f;}
   else {assert(p[5]==33&&p[6]==1);response[5]=p[2]==1?0:1;}
   const unsigned length=p[6]+6;uint8_t sum=0;for(unsigned i=2;i+1<length;++i)sum+=response[i];
   response[length-1]=uint8_t(~sum)+uint8_t(badMode&&p[2]==2&&p[5]==33);
   memcpy(rx+tail,response,length);tail+=length;
  }
  if(n==24){words[0]=uint16_t(p[9])|(uint16_t(p[10])<<8);words[1]=uint16_t(p[17])|(uint16_t(p[18])<<8);}
  if(partial){partial=false;return n-1;}return n;
 }
};
void cycle(Control&c,ServoBus<IO>&b,IO&io,uint32_t t){io.now=t;io.budget=io.echo?2:16;c.tick(t,true,false,true,25);b.service(io,c,t);}
void begin(Control&c,ServoBus<IO>&b,IO&io,uint32_t start){
 c.input(0,true,start);cycle(c,b,io,start);c.input(1,true,start+1);
 for(uint32_t t=start+1;t<start+300;++t)cycle(c,b,io,t);
}
unsigned goals(const IO&io){unsigned n=0;for(unsigned i=0;i<io.writes;++i)n+=io.sizes[i]==24;return n;}
int main(){
 {Control c(false,Acceptance{});ServoBus<IO>b;IO io;begin(c,b,io,1000);
  assert(!c.state.active&&c.state.reason==Reason::PositionRange&&io.writes==2);}
 {Control c(false,verified);ServoBus<IO>b;IO io;begin(c,b,io,1000);
  assert(c.state.active&&c.state.recovery==1&&goals(io)==1);
  assert(io.words[0]==1874&&io.words[1]==2207); // no clamp to 30%, even with held Up
  assert(c.target(true)==1874&&c.target(false)==2207);
  c.input(0,true,1300);cycle(c,b,io,1300);assert(c.state.recovery==2&&c.state.return_directions==2);
  for(uint32_t t=1301;t<1500;++t)cycle(c,b,io,t);
  assert(io.words[0]==1874&&io.words[1]==2207&&goals(io)==1); // no automatic return
  c.input(1,true,1500);for(uint32_t t=1500;t<1600;++t)cycle(c,b,io,t);
  assert(c.state.reason==Reason::RecoveryDirection&&goals(io)==1);
  c.input(4,true,1600);cycle(c,b,io,1600);assert(goals(io)==1); // roll rejected
  c.input(2,true,1601);uint16_t prevR=1874,prevL=2207;
  uint32_t t=1601;
  for(;t<8000&&c.state.recovery;++t){
   cycle(c,b,io,t);assert(io.words[0]>=prevR&&io.words[1]<=prevL);
   assert(c.accepted(true,decodePosition(io.words[0]))&&c.accepted(false,decodePosition(io.words[1])));
   prevR=io.words[0];prevL=io.words[1];
  }
  assert(!c.state.recovery);
  const int16_t targetR=c.target(true),targetL=c.target(false);
  c.input(0,true,t);for(unsigned k=0;k<200;++k)cycle(c,b,io,t++);
  assert(io.words[0]==uint16_t(targetR)&&io.words[1]==uint16_t(targetL)); // final UART slot may follow mode transition
  assert(c.target(true)==targetR&&c.target(false)==targetL); // mode transition/release never re-level
  assert(inRange(true,targetR)&&inRange(false,targetL));
  c.input(2,true,t);
  for(unsigned k=0;k<7000;++k)cycle(c,b,io,t++);
  assert((c.target(true)==position(true,0)||c.target(false)==position(false,0))&&c.state.limited);
  const auto right=c.target(true),left=c.target(false);
  for(unsigned k=0;k<200;++k)cycle(c,b,io,t++);
  assert(c.target(true)==right&&c.target(false)==left); // no one-leg continuation
 }
 {Control c(true,verified);ServoBus<IO>b;IO io;begin(c,b,io,1000);
  assert(c.state.recovery==1&&goals(io)==1); // even confirmed roll cannot move recovery hold
  c.input(0,true,1300);cycle(c,b,io,1300);assert(goals(io)==1);}
 for(bool conflict:{false,true}){
  Control c(false,verified);ServoBus<IO>b;IO io;io.words[1]=conflict?2071:2081;begin(c,b,io,1000);
  assert(c.state.recovery==1);c.input(0,true,1300);cycle(c,b,io,1300);
  c.input(2,true,1301);for(uint32_t t=1301;t<1500;++t)cycle(c,b,io,t);
  assert(goals(io)==1&&io.words[0]==1874&&io.words[1]==(conflict?2071:2081));
  assert(c.state.reason==(conflict?Reason::ReturnConflict:Reason::PairLimit));
 }
 {Acceptance disabled=verified;disabled.extra=false;Control c(false,disabled);assert(!c.seed(1874,2207,1000));}
 {Acceptance broken{0,100,0,200,true};Control c(false,broken);assert(!c.seed(2036,2053,1000));}
 {Acceptance negative{-100,2007,2071,2412,true};Control c(false,negative);assert(!c.seed(-10,2122,1000));
  assert(encodePosition(-10)==0x800a);}
 // Pure configuration queries: all emitted frames are READ; no position or torque writes.
 for(unsigned failure=0;failure<5;++failure){
  Control c;ServoBus<IO>b;IO io;io.echo=failure==0;io.badMode=failure==1;io.silent=failure==2;
  io.partial=failure==3;if(failure==4)io.capacity=0;
  c.input(1,true,999);assert(b.inspect(c,1000));
  for(uint32_t t=1000;t<2200;++t){io.now=t;io.budget=io.echo?2:16;b.service(io,c,t);}
  for(unsigned i=0;i<io.writes;++i){assert(io.sizes[i]==8&&io.packets[i][4]==2);
   if(i)assert(io.at[i]-io.at[i-1]>=50);}
  assert(!c.state.active&&!c.state.synced&&c.state.torque==0);
  const auto&cfg=c.state.configuration;
  if(!failure){
   assert(io.writes==4&&cfg.status==2&&cfg.current==15&&cfg.acquired==15);
   assert(io.packets[0][5]==9&&io.packets[0][6]==4&&io.packets[1][5]==33&&io.packets[1][6]==1);
   assert(cfg.minimum[0]==0x1234&&cfg.maximum[0]==0xabcd&&cfg.maximum[1]==4095);
   assert(cfg.mode[0]==0&&cfg.mode[1]==1); // wheel-mode observation does not cause a mode write
  }else{
   assert(cfg.status==3);assert(cfg.finished_ms>=1000);
   if(failure==1){assert(cfg.current==7&&c.state.rx_failure.reg==33&&c.state.rx_failure.requested==1);}
  }
  const auto written=io.writes;
  for(uint32_t t=2200;t<2300;++t){io.now=t;io.budget=16;c.tick(t,true,false,true,0);b.service(io,c,t);}
  assert(io.writes==written); // Up held through read-only job cannot turn into motion afterwards
 }
 {Control c;ServoBus<IO>b;IO io;assert(b.inspect(c,1000));b.off(c,1001);
  assert(c.state.configuration.status==3&&c.state.configuration.finished_ms==1001);}
 printf("PASS: disabled/verified acceptance, exact hold, release/new press, inward-only paired return, limits/conflict, pure async mode/angle READ and failures; config=%zu acceptance=%zu\n",sizeof(ServoConfiguration),sizeof(Acceptance));
}

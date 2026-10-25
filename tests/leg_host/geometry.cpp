#include "LegServoBus.h"
#include "DiagnosticTelemetry.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>
using namespace leg;
constexpr Acceptance full{1675,2007,2071,2412,true};
struct IO {
 uint8_t rx[8]{};unsigned at=0,end=0,goals=0,ons=0,writes=0;
 int16_t right=2007,left=2071,firstRight=0,firstLeft=0;
 bool bad=false;
 int available(){return end-at;}int read(){return rx[at++];}int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,size_t n){
  ++writes;
  if(n==8){uint16_t v=encodePosition(p[2]==1?right:left);uint8_t response[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};
   uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=response[i];response[7]=uint8_t(~sum)+uint8_t(bad);
   for(unsigned i=0;i<8;++i)rx[i]=response[i];
   at=0;end=8;}
  if(n==24){if(!goals){firstRight=decodePosition(uint16_t(p[9])|uint16_t(p[10])<<8);firstLeft=decodePosition(uint16_t(p[17])|uint16_t(p[18])<<8);}++goals;}
  if(n==12&&p[8]){assert(goals);++ons;}return n;
 }
};
void tick(Control&c,ServoBus<IO>&b,IO&io,uint32_t t){c.tick(t,true,false,true,48);b.service(io,c,t);}
void start(Control&c,ServoBus<IO>&b,IO&io){
 c.input(0,true,1000);tick(c,b,io,1000);c.input(1,true,1001);tick(c,b,io,1001);
 c.input(0,true,1002);for(unsigned t=1002;t<1250;++t)tick(c,b,io,t);
}
int main(){
 assert(position(true,0)==1997&&position(true,30)==1908&&position(false,0)==2081&&position(false,30)==2173);
 assert(safeMinTick(true)==1685&&safeMaxTick(true)==1997&&safeMinTick(false)==2081&&safeMaxTick(false)==2402);
 assert(percent(true,2007)==0&&percent(true,1675)==100&&percent(false,2071)==0&&percent(false,2412)==100);
 assert(percent(true,1997)>3&&percent(false,2081)>2.9f&&normalHigh(true)<=30&&normalHigh(false)<=30);
 // Every accepted tick-pair round-trips exactly and never levels or moves without a request.
 unsigned pairs=0;
 for(int r=1675;r<=2007;++r)for(int l=2071;l<=2412;++l){
  Control c(false,full);assert(c.seed(r,l,1000));assert(c.target(true)==r&&c.target(false)==l);
  c.activate(1000);c.input(0,true,1000);c.tick(1050,true,false,true,48);c.tick(1100,true,false,true,-48);
  assert(c.target(true)==r&&c.target(false)==l);++pairs;
 }
 const int16_t rights[]={1674,1675,1907,1908,1957,1997,1998,2007,2008};
 const int16_t lefts[]={2070,2071,2080,2081,2122,2173,2174,2412,2413};
 for(auto r:rights)for(auto l:lefts){
  Control c(false,full);ServoBus<IO>b;IO io;io.right=r;io.left=l;start(c,b,io);
  if(measured(true,r)&&measured(false,l)){
   assert(c.state.active&&io.goals==1&&io.ons==1&&io.firstRight==r&&io.firstLeft==l);
   if(r==2007&&l==2071){
    if(const char*file=getenv("LEG_GEOMETRY_JSON_OUTPUT")){
     diagnostic::Snapshot snapshot{};snapshot.legs=c.state;snapshot.sample_ms=1250;
     char json[2048];assert(diagnostic::wheelReadMessage(json,sizeof(json),snapshot,"geometry-test"));
     FILE*f=fopen(file,"w");assert(f);fputs(json,f);fclose(f);
    }
   }
   const auto goals=io.goals;for(unsigned t=1250;t<1500;++t)tick(c,b,io,t);
   assert(io.goals==goals&&c.target(true)==r&&c.target(false)==l);
  }else assert(io.writes==2&&!io.goals&&!io.ons&&!c.state.active);
 }
 // Both margin-side and extension-side returns require release, then fresh intentional direction.
 for(bool extension:{false,true}){
  Control c(false,full);ServoBus<IO>b;IO io;
  io.right=extension?1675:2007;io.left=extension?2412:2071;
  c.input(0,true,1000);tick(c,b,io,1000);c.input(1,true,1001);
  for(unsigned t=1001;t<1400;++t)tick(c,b,io,t);
  assert(c.state.recovery==1&&io.goals==1);
  c.input(0,true,1400);tick(c,b,io,1400);
  assert(c.state.return_directions==(extension?2:1));
  const int r=c.target(true),l=c.target(false);
  c.input(extension?1:2,true,1401);tick(c,b,io,1451);
  assert(c.target(true)==r&&c.target(false)==l&&c.state.reason==Reason::RecoveryDirection);
  c.input(0,true,1452);tick(c,b,io,1452);c.input(extension?2:1,true,1453);
  for(unsigned t=1453;t<2000;++t)tick(c,b,io,t);
  assert(extension?c.target(true)>r:c.target(true)<r);assert(extension?c.target(false)<l:c.target(false)>l);
 }
 // Contradictory return directions and one leg at the travel-direction end never move either side.
 for(bool conflict:{false,true}){
  Control c(false,full);assert(c.seed(1675,conflict?2071:2081,1000));c.activate(1000);
  c.input(0,true,1001);c.tick(1001,true,false,true,0);assert(c.state.return_directions==0);
  uint32_t time=1050;
  for(uint8_t key:{uint8_t(1),uint8_t(2),uint8_t(3)}){
   c.input(key,true,time);c.tick(time,true,false,true,0);time+=50;
   assert(c.target(true)==1675&&c.target(false)==(conflict?2071:2081));
   assert(c.state.reason==(conflict?Reason::ReturnConflict:key==1?Reason::RecoveryDirection:Reason::PairLimit));
  }
 }
 for(auto r:{int16_t(1675),int16_t(2007)}){
  Control c(false,full);ServoBus<IO>b;IO io;io.right=r;io.bad=true;start(c,b,io);
  assert(!io.goals&&!io.ons&&!c.state.active);
 }
 printf("PASS: measured 0/100, margin/normal endpoints +/-1, %u exact hold pairs, 81 bus preload/ON cases, UP/DOWN recovery, differing legs/conflict/paired stop and bad response no ON\n",pairs);
}

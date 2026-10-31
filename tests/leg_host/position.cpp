#include "LegServoBus.h"
#include "DiagnosticTelemetry.h"
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
using namespace leg;
struct IO {
 uint8_t rx[8]{},packets[32][24]{};unsigned at=0,queued=0,writes=0,sizes[32]{};
 uint16_t words[2]={1957,2122};uint8_t corrupt=0;
 int available(){return int(queued-at);}int read(){return at<queued?rx[at++]:-1;}
 int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,unsigned n){
  assert(n<=24&&writes<32);memcpy(packets[writes],p,n);sizes[writes++]=n;
  if(n==8){
   const uint16_t word=words[p[2]==RightId?0:1];
   const uint8_t response[]={255,255,p[2],4,0,uint8_t(word),uint8_t(word>>8),0};
   memcpy(rx,response,8);uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=rx[i];
   rx[7]=uint8_t(~sum)+(corrupt==p[2]);at=0;queued=8;
  }
  return n;
 }
};
void run(Control&c,ServoBus<IO>&bus,IO&io,uint32_t start){
 c.input(0,true);c.tick(start,true,false,true,0);c.input(1,true);
 for(uint32_t now=start+1;now<start+210;++now){c.tick(now,true,false,true,0);bus.service(io,c,now);}
}
int main(){
 // Documented STS sign/magnitude semantics, not a two's-complement int16 cast.
 assert(decodePosition(0)==0&&decodePosition(2036)==2036&&decodePosition(0x7fff)==32767);
 assert(decodePosition(0x8000)==0&&decodePosition(0x8001)==-1&&decodePosition(0x800a)==-10);
 assert(decodePosition(0xffff)==-32767);
 struct Case {uint16_t right,left;int rightDecoded,leftDecoded;bool accepted;};
 const Case cases[]={
  {1957,2122,1957,2122,true}, {1997,2081,1997,2081,true}, {1908,2173,1908,2173,true},
  {1907,2122,1907,2122,false}, {1998,2122,1998,2122,false},
  {1957,2080,1957,2080,false}, {1957,2174,1957,2174,false},
  {0,2122,0,2122,false}, {0x800a,2122,-10,2122,false}, {0xffff,2122,-32767,2122,false},
  {0xf407,2122,-29703,2122,false}
 };
 for(const auto&test:cases){
  Control c(false,Acceptance{});ServoBus<IO>bus;IO io;io.words[0]=test.right;io.words[1]=test.left;run(c,bus,io,1000);
  const auto&seen=c.state.positions;
  assert(seen.acquired==3&&seen.current==3);
  assert(seen.word[0]==test.right&&seen.word[1]==test.left);
  assert(decodePosition(seen.word[0])==test.rightDecoded&&decodePosition(seen.word[1])==test.leftDecoded);
  assert(seen.received_ms[0]>=1000&&seen.received_ms[1]>seen.received_ms[0]);
  if(test.accepted){
   assert(c.state.active&&c.state.synced&&io.sizes[2]==24&&io.sizes[3]==12);
   assert((io.packets[2][9]|(uint16_t(io.packets[2][10])<<8))==test.right);
   assert((io.packets[2][17]|(uint16_t(io.packets[2][18])<<8))==test.left);
   assert(io.packets[3][5]==40&&io.packets[3][8]==1); // original positions precede ON
  }else{
   assert(!c.state.active&&!c.state.synced&&c.state.reason==Reason::PositionRange);
   assert(io.writes==2&&io.sizes[0]==8&&io.sizes[1]==8); // no clamp, preload or ON
   assert(!c.state.command_valid);
  }
  if(test.right==1907){
   const char*file=getenv("LEG_POSITION_JSON_OUTPUT");
   if(file){
    diagnostic::Snapshot snapshot{};snapshot.legs=c.state;snapshot.sample_ms=1210;
    char json[2048];assert(diagnostic::wheelReadMessage(json,sizeof(json),snapshot,"position-test"));
    FILE*f=fopen(file,"w");assert(f);fputs(json,f);fclose(f);
   }
  }
  const auto saved=seen;c.stop(Reason::Stopped);
  assert(c.state.positions.word[0]==saved.word[0]&&c.state.positions.word[1]==saved.word[1]);
 }
 // Inward quantization keeps the integer endpoint at or below 30%.
 assert(percent(true,1908)<30&&inRange(true,1908)&&!inRange(true,1907));
 for(uint8_t corrupt:{uint8_t(1),uint8_t(2)}){
  Control c(false,Acceptance{});ServoBus<IO>b;IO io;io.corrupt=corrupt;run(c,b,io,1000);
  assert(!c.state.active&&!c.state.synced);
  assert(c.state.positions.acquired==(corrupt==1?0:1));
  for(unsigned i=0;i<io.writes;++i)assert(io.sizes[i]==8);
 }
 // A later invalid reply never overwrites an earlier valid observation or mixes attempts.
 {Control c(false,Acceptance{});ServoBus<IO>b;IO io;run(c,b,io,1000);assert(c.state.active);
  const auto saved=c.state.positions;b.off(c);b.service(io,c,1300);
  const unsigned before=io.writes;io.words[0]=2040;io.words[1]=2080;io.corrupt=2;
  run(c,b,io,1400);assert(!c.state.active&&!c.state.synced);
  assert(c.state.positions.acquired==3&&c.state.positions.current==1);
  assert(c.state.positions.word[0]==2040&&c.state.positions.word[1]==saved.word[1]);
  assert(c.state.positions.received_ms[1]==saved.received_ms[1]);
  for(unsigned i=before;i<io.writes;++i)assert(io.sizes[i]==8);
 }
 printf("PASS: raw bytes/STS sign-magnitude, normal/endpoints/outside/refusal, exact preload, partial/invalid observations and attempt separation; position=%zu state=%zu\n",sizeof(PositionObservation),sizeof(State));
}

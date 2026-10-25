#include "LegServoBus.h"
#include "DiagnosticTelemetry.h"
#include <cstdlib>
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
using namespace leg;
void checksum(uint8_t*p,unsigned n){uint8_t s=0;for(unsigned i=2;i+1<n;++i)s+=p[i];p[n-1]=uint8_t(~s);}
void response(uint8_t*p,uint8_t id){const uint8_t v[]={255,255,id,4,0,244,7,0};memcpy(p,v,8);checksum(p,8);}
ServoReply::Result feed(ServoReply&p,const uint8_t*b,unsigned n){
 auto result=ServoReply::Pending;
 for(unsigned i=0;i<n;++i){result=p.feed(b[i]);if(result!=ServoReply::Pending){assert(i+1==n);break;}}
 return result;
}
struct IO {
 uint8_t rx[512]{},packets[32][24]{};unsigned head=0,tail=0,writes=0,length[32]{};
 int budget=16;unsigned mode=0;uint8_t bad_target=1; // 0 normal, 1 echo, 2 ID, 3 length, 4 status, 5 checksum, 6 no response
 int available(){return budget>0?int(tail-head):0;}
 int read(){--budget;return head<tail?rx[head++]:-1;}
 int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,unsigned n){
  assert(writes<32);memcpy(packets[writes],p,n);length[writes++]=n;
  if(n==8){
   if(mode==7){memcpy(rx+tail,packets[0],length[0]);tail+=length[0];}
   if(mode==1||mode==7){memcpy(rx+tail,p,8);tail+=8;}
   if(mode!=6){
    uint8_t answer[8];response(answer,p[2]);
    const auto pos=position(p[2]==RightId,15);answer[5]=uint8_t(pos);answer[6]=uint8_t(pos>>8);checksum(answer,8);
    if(p[2]==bad_target){
     if(mode==2)answer[2]=p[2]==1?2:1;
     if(mode==3)answer[3]=5;
     if(mode==4)answer[4]=32;
     checksum(answer,8);if(mode==5)++answer[7];
    }
    memcpy(rx+tail,answer,8);tail+=8;
   }
  }
  return n;
 }
};
int main(){
 uint8_t request[]={255,255,1,4,2,56,2,0};checksum(request,8);
 uint8_t normal[8];response(normal,1);
 // Receive every split point, without consuming a partial frame as invalid.
 for(unsigned split=0;split<8;++split){
  ServoReply p;p.begin(1,1000);
  assert(feed(p,normal,split)==ServoReply::Pending);
  assert(feed(p,normal+split,8-split)==ServoReply::Good);
  assert(p.position()==2036);
 }
 // Full/fragmented read echo is not a status=2 reply. Echo alone cannot succeed.
 for(unsigned split=0;split<=8;++split){
  ServoReply p;p.begin(1,1000);
  assert(feed(p,request,split)==ServoReply::Pending);
  assert(feed(p,request+split,8-split)==ServoReply::Pending);
  assert(feed(p,normal,8)==ServoReply::Good);
 }
 {ServoReply p;p.begin(1,1000);uint8_t almost[8];memcpy(almost,request,8);++almost[7];
  assert(feed(p,almost,8)==ServoReply::Bad);
  assert(p.failure(1001).issues==(RxStatus|RxChecksum));}
 // The last actually transmitted broadcast frame is recognized only in full.
 for(unsigned count:{12u,24u}){
  uint8_t broadcast[24]={255,255,254,0,131};broadcast[3]=count-4;checksum(broadcast,count);
  ServoReply p;p.noteTransmit(broadcast,count);p.begin(1,1000);
  assert(feed(p,broadcast,count)==ServoReply::Pending);
  assert(feed(p,normal,8)==ServoReply::Good);
  p.begin(1,1000);broadcast[4]^=1;
  assert(feed(p,broadcast,count)==ServoReply::Bad); // a merely similar frame is not discarded
 }
 // Noise and overlapping FF header retain the proper boundary, bounded to 10 discarded bytes.
 {ServoReply p;p.begin(1,1000);uint8_t noise[]={3,4,255};
  assert(feed(p,noise,3)==ServoReply::Pending);
  assert(feed(p,normal,8)==ServoReply::Good);}
 {ServoReply p;p.begin(1,1000);for(int i=0;i<10;++i)assert(p.feed(0)==ServoReply::Pending);
  assert(p.feed(0)==ServoReply::Bad);assert(p.failure(1002).issues&RxHeader);}
 // Exact mismatches, expected checksum and missing fields survive a timeout.
 for(unsigned mutation:{2u,3u,4u,7u}){
  ServoReply p;p.begin(1,1000);uint8_t bad[8];memcpy(bad,normal,8);++bad[mutation];
  if(mutation!=7)checksum(bad,8);
  const unsigned n=8;
  if(mutation==3){
   assert(feed(p,bad,n)==ServoReply::Pending); // advertised 9 B, only 8 arrived
   p.timeout();
  }else assert(feed(p,bad,n)==ServoReply::Bad);
  const auto r=p.failure(1002);
  assert(r.target==1&&r.received==n&&r.elapsed_ms==2&&r.frame_size==n);
  assert(r.issues==(mutation==2?RxId:mutation==3?(RxLength|RxTimeout):mutation==4?RxStatus:RxChecksum));
  if(mutation==7)assert(r.expected_checksum==normal[7]&&r.frame[7]==bad[7]);
 }
 {ServoReply p;p.begin(2,1000);uint8_t both[8];response(both,1);both[4]=32;++both[7];
  assert(feed(p,both,8)==ServoReply::Bad);
  assert(p.failure(1005).issues==(RxId|RxStatus|RxChecksum));}
 for(unsigned partial=0;partial<8;++partial){
  ServoReply p;p.begin(1,1000);assert(feed(p,normal,partial)==ServoReply::Pending);
  assert(!p.expired(1010,10)&&p.expired(1011,10));p.timeout();auto r=p.failure(1011);
  assert(r.issues==RxTimeout&&r.frame_size==partial&&r.received==partial);
 }
 {ServoReply p;p.begin(1,1000);for(int i=0;i<5;++i)assert(feed(p,request,8)==ServoReply::Pending);
  p.timeout();auto r=p.failure(1011);assert(r.echoes==5&&r.frame_size==0&&r.received==40&&r.raw_size==32&&r.truncated);}
 // Actual bus: echoes/fragmentation succeed; all failures on either side block goal and ON.
 for(unsigned mode=0;mode<=7;++mode)for(uint8_t target:{1,2}){
  Control c;ServoBus<IO>b;IO io;io.mode=mode;io.bad_target=target;
  if(mode==7){b.off(c);b.service(io,c,900);}
  c.input(0,true);c.tick(999,true,false,true,0);c.input(1,true);
  for(unsigned cycle=0;cycle<400;++cycle){
   uint32_t now=1000+cycle/2;io.budget=(mode==7?16:1+(mode==1)); // two-byte echo fragments fit the unchanged deadline
   const unsigned before=io.writes;
   c.tick(now,true,false,true,0);b.service(io,c,now);assert(io.writes-before<=1);
  }
  if(mode<=1||mode==7)assert(c.state.active&&c.state.synced);
  else {
   assert(!c.state.active&&!c.state.synced);
   const auto failure=c.state.rx_failure;
   // No-response mode necessarily fails on the first/right read.
   assert(failure.target==(mode==6?1:target));assert(failure.issues);
   if(mode==2&&target==2){
    const char* output=getenv("LEG_RX_JSON_OUTPUT");
    if(output){
     diagnostic::Snapshot snapshot{};snapshot.legs=c.state;snapshot.sample_ms=1200;
     char json[2048];assert(diagnostic::wheelReadMessage(json,sizeof(json),snapshot,"reply-test"));
     FILE*f=fopen(output,"w");assert(f);fputs(json,f);fclose(f);
    }
   }
   for(unsigned i=0;i<io.writes;++i)assert(io.length[i]==8); // neither preload nor ON
   c.stop(Reason::Stopped);assert(failure.ms==c.state.rx_failure.ms&&failure.issues==c.state.rx_failure.issues&&
    !memcmp(failure.raw,c.state.rx_failure.raw,sizeof(failure.raw)));
   c.input(0,true);c.tick(1300,true,false,true,0);assert(c.state.rx_failure.issues);
  }
 }
 // Short error-status replies retain status and the actual checksum at byte 5.
 {ServoReply p;p.begin(1,1000);uint8_t shortReply[]={255,255,1,2,32,0};checksum(shortReply,6);
  assert(feed(p,shortReply,6)==ServoReply::Bad);auto r=p.failure(1001);
  assert(r.issues==(RxLength|RxStatus)&&r.frame_size==6&&r.checksum_ready);
  assert(r.received_checksum==shortReply[5]&&r.expected_checksum==shortReply[5]);}
 for(uint8_t length:{0,1,21,255}){
  ServoReply p;p.begin(1,1000);uint8_t invalid[]={255,255,1,length};
  assert(feed(p,invalid,4)==ServoReply::Bad&&p.failure(1001).issues==RxLength);
 }
 // A complete late reply is not accepted; a cut frame never joins the next request.
 {Control c;ServoBus<IO>b;IO io;io.mode=6;c.input(0,true);c.tick(999,true,false,true,0);c.input(1,true);
  c.tick(1000,true,false,true,0);b.service(io,c,1000);io.mode=0;io.tail=8;memcpy(io.rx,normal,8);
  c.tick(1011,true,false,true,0);b.service(io,c,1011);assert(c.state.rx_failure.issues==RxTimeout&&!c.state.synced);}
 // A cut response expires, then a new explicit operation starts a clean frame.
 {Control c;ServoBus<IO>b;IO io;io.mode=6;c.input(0,true);c.tick(999,true,false,true,0);c.input(1,true);
  c.tick(1000,true,false,true,0);b.service(io,c,1000);
  memcpy(io.rx,normal,3);io.tail=3;io.budget=3;b.service(io,c,1001);
  b.service(io,c,1011);assert(c.state.rx_failure.frame_size==3&&!c.state.synced);
  io.mode=0;c.input(0,true);c.tick(1100,true,false,true,0);c.input(1,true);
  for(uint32_t now=1101;now<1400;++now){io.budget=16;c.tick(now,true,false,true,0);b.service(io,c,now);}
  assert(c.state.active&&c.state.synced&&c.state.rx_failure.ms==1011);
 }
 printf("PASS: STS read/broadcast echoes, all splits, noise/header, ID/length/status/checksum, timeout/late/truncation, both sides fail closed; record=%zu parser=%zu bus=%zu\n",sizeof(RxFailure),sizeof(ServoReply),sizeof(ServoBus<IO>));
}

#include "LegServoBus.h"
#include "LegSerialDiagnostics.h"
#include "DiagnosticTelemetry.h"
#include <cassert>
#include <cstdio>
#include <vector>
#include <string>
#include <map>
using namespace leg;
struct Store {
 std::map<std::string,std::vector<uint8_t>> bytes;
 bool fail_open=false,fail_read=false,fail_write=false,torn=false,corrupt_after_write=false;unsigned writes=0;
 bool begin(const char*ns,bool){assert(std::string(ns)=="leg_range");return !fail_open;}void end(){}
 size_t getBytesLength(const char*k){return bytes[k].size();}
 size_t getBytes(const char*k,void*p,size_t n){if(fail_read)return 0;if(bytes[k].size()!=n)return 0;memcpy(p,bytes[k].data(),n);return n;}
 size_t putBytes(const char*k,const void*p,size_t n){++writes;if(fail_write)return 0;const auto*b=static_cast<const uint8_t*>(p);bytes[k].assign(b,b+n-(torn?1:0));if(corrupt_after_write)bytes[k][31]^=1;return torn?n-1:n;}
 void record(unsigned slot,const LegRecord&r){const auto*b=reinterpret_cast<const uint8_t*>(&r);bytes[slot?"slot1":"slot0"].assign(b,b+sizeof(r));}
};
struct IO {
 uint8_t rx[16]{};unsigned at=0,count=0,goals=0,ons=0,reads=0;bool silent=false,bad=false,partial=false;
 int16_t values[2]={2044,2084},first_goal[2]{};
 int available(){return count-at;}int read(){return rx[at++];}int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,size_t n){
  if(n==24){if(!goals){first_goal[0]=decodePosition(uint16_t(p[9])|(uint16_t(p[10])<<8));first_goal[1]=decodePosition(uint16_t(p[17])|(uint16_t(p[18])<<8));}++goals;}
  if(n==12&&p[8])++ons;
  if(n==8){++reads;assert(p[4]==2&&p[5]==56&&p[6]==2);if(!silent){uint16_t v=encodePosition(values[p[2]==1?0:1]);
   uint8_t r[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=r[i];r[7]=uint8_t(~sum)+(bad?1:0);
   memcpy(rx,r,8);at=0;count=8;}}
  return partial?n-1:n;
 }
};
int main(){
 assert(!validGeometry(activeGeometry()));Control c;assert(!c.seed(2000,2100,1));
 c.input(0,true);c.tick(1,true,false,true,0);c.input(1,true);c.tick(2,true,false,true,0);assert(!c.want_sync);
 {Control uncalibrated;ServoBus<IO>bus;IO io;
  assert(bus.observe(uncalibrated));
  for(unsigned now=100;now<300;++now)bus.service(io,uncalibrated,now);
  assert(uncalibrated.state.positions.current==3&&io.goals==0&&io.ons==0&&uncalibrated.state.reason==Reason::CalibrationRequired);
 }
 const auto g=ConfirmedCandidate;assert(validateGeometry(g)==CalResult::Ok);
 assert(percent(true,2044,g)==0&&percent(true,1697,g)==100&&percent(false,2084,g)==0&&percent(false,2408,g)==100);
 assert(normalMinTick(true,g)==1940&&normalMaxTick(true,g)==2034&&normalMinTick(false,g)==2094&&normalMaxTick(false,g)==2181);
 assert(position(true,0,g)==2034&&position(false,0,g)==2094&&position(true,10,g)==2009&&position(false,10,g)==2116);
 for(auto bad: {Geometry{2044,2044,2084,2408},Geometry{1697,2044,2084,2408},Geometry{2044,1697,2408,2084},Geometry{-1,0,0,4096},Geometry{20,19,20,21}})assert(validateGeometry(bad)!=CalResult::Ok);
 Store store;assert(loadLegs(store).result==CalResult::NoSaved);
 auto saved=saveLegs(store,g);assert(saved.result==CalResult::Ok&&saved.slot==0&&saved.generation==1);
 auto original=store.bytes["slot0"];auto next=g;--next.right_full;
 store.fail_write=true;assert(saveLegs(store,next).result==CalResult::StorageWrite);assert(store.bytes["slot0"]==original);store.fail_write=false;
 store.torn=true;assert(saveLegs(store,next).result==CalResult::StorageWrite);assert(loadLegs(store).generation==1);store.torn=false;
 store.corrupt_after_write=true;assert(saveLegs(store,next).result==CalResult::StorageVerify);assert(loadLegs(store).generation==1);store.corrupt_after_write=false;
 store.fail_read=true;auto writes=store.writes;assert(saveLegs(store,next).result==CalResult::StorageVerify&&store.writes==writes);store.fail_read=false;
 store.fail_open=true;assert(saveLegs(store,next).result==CalResult::StorageOpen);store.fail_open=false;
 saved=saveLegs(store,next);assert(saved.result==CalResult::Ok&&saved.slot==1&&saved.generation==2&&loadLegs(store).generation==2);
 store.bytes["slot1"][0]^=1;assert(loadLegs(store).generation==1);
 store.bytes["slot0"][31]^=1;assert(loadLegs(store).result==CalResult::Corrupt);
 store.record(0,recordFor(g,0xffffffff));store.record(1,recordFor(next,0));assert(loadLegs(store).slot==1);
 auto mismatch=recordFor(g,5);mismatch.margin=11;mismatch.crc=legCrc(&mismatch,offsetof(LegRecord,crc));assert(!validRecord(mismatch));
 LegCalibration cal;assert(!cal.begin(false,1));assert(cal.begin(true,2));assert(cal.canSave()==CalResult::Missing);
 assert(cal.confirmed(3)&&cal.canSave()==CalResult::Ok);assert(cal.capture(0,4));cal.received(0,false,5);assert(!(cal.mask&1)&&cal.candidate.right_zero==2044&&cal.canSave()==CalResult::Missing);
 assert(cal.confirmed(6));assert(cal.abort(7)&&cal.phase==5);assert(!validGeometry(activeGeometry()));
 // Every capture goes through the production asynchronous UART parser. Neither
 // success, checksum failure, timeout nor partial TX can write a goal or ON.
 for(unsigned failure=0;failure<4;++failure){Control test;ServoBus<IO>bus;IO io;test.state.torque=2;
  assert(test.state.calibration.begin(true,100));
  for(unsigned point=0;point<4;++point){io.values[0]=point==1?1697:2044;io.values[1]=point==3?2408:2084;
   io.silent=failure==1;io.bad=failure==2;io.partial=failure==3;
   unsigned now=100+point*100;assert(bus.captureCalibration(test,point,now));
   for(unsigned t=now;t<now+80;++t)bus.service(io,test,t);
   assert(test.state.calibration.phase==1&&io.goals==0&&io.ons==0);
  }
  assert(test.state.calibration.mask==(failure?0:15));
  if(!failure)assert(test.state.calibration.canSave()==CalResult::Ok);
 }
 // New stroke exact initial hold over every pair including endpoints, then the
 // existing return gate, conflict and paired boundary behavior.
 activeGeometry()=g;
 struct SeedCase {int16_t r,l;bool valid;};
 for(const auto test: {SeedCase{2044,2084,true},{1697,2408,true},{2000,2100,true},{1940,2181,true},{2034,2094,true},{1696,2084,false},{2045,2084,false},{2000,2083,false},{2000,2409,false}}){
  Control control;ServoBus<IO>bus;IO io;io.values[0]=test.r;io.values[1]=test.l;
  control.input(0,true);control.tick(100,true,false,true,0);control.input(1,true);
  for(unsigned t=101;t<400;++t){control.tick(t,true,false,true,0);bus.service(io,control,t);if(io.goals)control.input(0,true);}
  if(test.valid){assert(io.goals==1&&io.ons==1&&io.first_goal[0]==test.r&&io.first_goal[1]==test.l);}
  else assert(io.goals==0&&io.ons==0&&!control.state.active);
 }
 for(int r=1697;r<=2044;++r)for(int l=2084;l<=2408;++l){Control test;assert(test.seed(r,l,100));assert(test.target(true)==r&&test.target(false)==l);test.activate(100);test.input(0,true);test.tick(101,true,false,true,0);assert(test.target(true)==r&&test.target(false)==l);}
 Control pair;assert(pair.seed(1697,2408,100));pair.activate(100);pair.input(1,true);pair.tick(101,true,false,true,0);assert(pair.state.recovery==1&&pair.target(true)==1697);
 pair.input(0,true);pair.tick(102,true,false,true,0);assert(pair.state.return_directions==2);pair.input(2,true);pair.tick(152,true,false,true,0);assert(pair.target(true)>1697&&pair.target(false)<2408);
 Control conflict;assert(conflict.seed(2044,2408,100));conflict.activate(100);conflict.input(0,true);conflict.tick(101,true,false,true,0);assert(conflict.state.reason==Reason::ReturnConflict);
 for(auto point: {std::pair<int,int>{1696,2084},{2045,2084},{2000,2083},{2000,2409}}){Control test;assert(!test.seed(point.first,point.second,1));}
 // Continuous axes cannot consume the important FIFO. Coalescing remains latest
 // value and bounded, and a receive failure/calibration event is retained.
 SerialDiagnostics log;State state;state.stick.flags=32;
 for(unsigned i=1;i<10000;++i){state.stick.lx=int8_t(i%128);log.input(i&15,true,i);log.capture(state,0,i);}
 state.rx_failure.issues=RxTimeout;state.rx_failure.ms=10000;state.calibration.begin(true,10000);log.capture(state,0,10000);
 SerialDiagnostics::Record record;uint32_t lost;bool rx=false,calEvent=false;unsigned total=0;
 while(log.pop(record,lost)){assert(!lost);rx|=record.type==SerialDiagnostics::Receive;calEvent|=record.type==SerialDiagnostics::Calibration;++total;}
 assert(rx&&calEvent&&total<=3);
 LegCalSerialParser parser;char action=0;std::string cmd=":leg candidate\n";for(char ch:cmd)assert(parser.feed(ch,action));assert(action=='b');
 for(char ch:std::string(":")+std::string(100,'u')+"\n")assert(parser.feed(ch,action));
 assert(action=='?');assert(!parser.feed('u',action));
 diagnostic::Snapshot snapshot;snapshot.legs.geometry=g;snapshot.legs.command_valid=true;snapshot.legs.right_command=2009;snapshot.legs.left_command=2116;
 activeGeometry()={2007,1675,2071,2412};
 char json[2048];assert(diagnostic::wheelReadMessage(json,sizeof(json),snapshot,"snapshot"));assert(strstr(json,"[2044,1697,10,10,1707,2034,1940,2034]"));assert(diagnostic::legCalibrationMessage(json,sizeof(json),snapshot,"test"));
 printf("PASS leg calibration: NVS slots/failure/CRC/generation, async capture, no implicit drive, 113100 exact initial pairs, boundaries, coalescing, parser; sizes State=%zu Control=%zu Bus=%zu Log=%zu Record=%zu Snapshot=%zu Calibration=%zu\n",sizeof(State),sizeof(Control),sizeof(ServoBus<IO>),sizeof(SerialDiagnostics),sizeof(SerialDiagnostics::Record),sizeof(snapshot),sizeof(LegCalibration));
}

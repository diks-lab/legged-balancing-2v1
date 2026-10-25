#include "LegServoBus.h"
#include "LegSerialDiagnostics.h"
#include <cassert>
#include <string>
#include <cstring>
#include <cstdio>
using namespace leg;
struct IO {
 uint8_t rx[24]{};unsigned at=0,end=0,writes=0,budget=16,maxRx=0;
 uint32_t now=0,previous=0;int capacity=128;
 int16_t values[2]{1977,2082};bool echo=false,silent=false,partial=false;uint8_t corrupt_id=0;
 int available(){return budget?end-at:0;}int read(){--budget;return rx[at++];}int availableForWrite(){return capacity;}
 size_t write(const uint8_t*p,size_t n){
  assert(n==8&&p[4]==2&&p[5]==56&&p[6]==2); // Every measurement TX is READ only.
  if(writes)assert(now-previous>=50);
  previous=now;++writes;at=end=0;
  if(!silent){
   if(echo){memcpy(rx,p,8);end=8;}
   const auto v=encodePosition(values[p[2]==1?0:1]);
   uint8_t reply[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};
   uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=reply[i];reply[7]=~sum;
   if(corrupt_id==p[2])reply[7]^=1;
   memcpy(rx+end,reply,8);end+=8;
  }
  if(partial){partial=false;return n-1;}return n;
 }
};
void step(ServoBus<IO>&bus,Control&c,IO&io,uint32_t t){io.now=t;io.budget=io.echo?2:16;bus.service(io,c,t);assert(io.budget<=16);}
struct Sink {
 std::string text;int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,size_t n){assert(n<=48);text.append((const char*)p,n);return n;}
};
int main(){
 Control c;ServoBus<IO>bus;IO io;
 assert(!bus.startMeasurement(c,1));assert(c.state.measurement.reject_reason==uint8_t(Reason::MeasureStopRequired)&&io.writes==0);
 c.state.torque=2;assert(bus.startMeasurement(c,100));
 const auto acceptance=c.state.acceptance;
 for(unsigned t=100;t<500;++t){
  if(t==200){io.values[0]=1900;io.values[1]=2200;}
  if(t==300){io.values[0]=2100;io.values[1]=1990;}
  step(bus,c,io,t);assert(!c.state.synced&&!c.state.active&&!c.want_sync&&c.state.torque==2);
 }
 auto&m=c.state.measurement;
 assert(m.acquired==3&&m.valid==3&&m.minimum[0]==1900&&m.maximum[0]==2100&&m.minimum[1]==1990&&m.maximum[1]==2200);
 assert(m.latest[0]==2100&&m.latest[1]==1990&&m.sample_ms[0]!=m.sample_ms[1]);
 assert(m.count[0]==4&&m.count[1]==4&&c.state.sync_ms==0&&c.state.positions.current==0);
 assert(c.state.acceptance.right_min==acceptance.right_min&&c.state.acceptance.left_max==acceptance.left_max);
 const auto count=io.writes;bus.stopMeasurement(c,500);assert(m.status==2&&m.finished_ms==500);
 for(unsigned t=500;t<600;++t)step(bus,c,io,t);
 assert(io.writes==count&&m.minimum[0]==1900); // Stop retains values, no writes, no implicit position hold.
 assert(bus.startMeasurement(c,600));assert(m.acquired==0&&m.count[0]==0);
 for(unsigned t=600;t<710;++t)step(bus,c,io,t);
 assert(m.minimum[0]==2100&&m.maximum[0]==2100);bus.stopMeasurement(c,710);
 // Both sides, checksum errors, absent response, partial TX and capacity failures.
 for(unsigned failure=0;failure<6;++failure){
  Control d;d.state.torque=2;ServoBus<IO>b;IO port;
  port.echo=failure==0;port.corrupt_id=failure==1?1:failure==2?2:0;
  port.silent=failure==3;port.partial=failure==4;if(failure==5)port.capacity=0;
  assert(b.startMeasurement(d,1000));
  for(unsigned t=1000;t<2200;++t)step(b,d,port,t);
  if(failure==0){assert(d.state.measurement.status==1&&d.state.measurement.acquired==3);b.stopMeasurement(d,2200);}
  else {assert(d.state.measurement.status==3&&!b.measuring()&&!b.busy());
   if(failure==1)assert(d.state.rx_failure.target==1&&d.state.measurement.acquired==0);
   if(failure==2)assert(d.state.rx_failure.target==2&&d.state.measurement.acquired==1&&d.state.measurement.minimum[0]==1977);
   if(failure==3)assert(d.state.measurement.error==uint8_t(Reason::ReplyTimeout));
   if(failure==4)assert(port.writes==1&&d.state.measurement.error==uint8_t(Reason::TxFailed));
   if(failure==5)assert(!port.writes&&d.state.measurement.error==uint8_t(Reason::TxBusy));
  }
  assert(d.state.torque==2&&!d.state.active&&!d.state.synced&&!d.want_sync);
 }
 // Transition reporting is bounded, changed-only, and is not used to drive servos.
 SerialDiagnostics log;SerialOutput out;Sink sink;
 auto emit=[&](){for(unsigned k=0;k<100;++k){if(out.idle()){SerialDiagnostics::Record r;uint32_t lost;
   if(log.pop(r,lost)){assert(!lost);out.begin(r,lost);}}out.service(sink);}};
 Control observed;observed.state.measurement.begin(10);log.capture(observed.state,11,10);emit();
 observed.state.measurement.capture(0,1977,20);observed.state.measurement.capture(1,2082,70);
 log.capture(observed.state,11,510);emit();const auto stable=sink.text;
 for(unsigned t=511;t<1511;++t)log.capture(observed.state,11,t);
 emit();assert(sink.text==stable);
 assert(sink.text.find("RIGHT current=1977 min=1977 max=1977")!=std::string::npos);
 // A separate acceptance diagnostic contains both positions and limits, not a UART failure.
 observed.state.positions.current=3;observed.state.positions.word[0]=encodePosition(1900);observed.state.positions.word[1]=encodePosition(2200);
 log.capture(observed.state,4,1600);observed.state.reason=Reason::PositionRange;log.capture(observed.state,0,1601);emit();
 assert(sink.text.find("LEG ACCEPTANCE: 初回受付範囲外 (OUTSIDE_INITIAL_RANGE) right=1900")!=std::string::npos);
 assert(sink.text.find("left=2200 (")!=std::string::npos&&sink.text.find("no preload/ON")!=std::string::npos);
 printf("PASS: explicit measurement pure READ both sides/outside acceptance, extrema/reset/end, split echo, failures/timeouts/partial/capacity, no seed/goal/ON or limit changes; measurement=%zu state=%zu bus=%zu records=%zu\n",sizeof(Measurement),sizeof(State),sizeof(bus),sizeof(log));
}

#include "LegSerialDiagnostics.h"
#include "LegServoBus.h"
#include <cassert>
#include <string>
#include <cstdio>
using namespace leg;
struct Sink {
 std::string text;int room=128;size_t maxWrite=0,cycles=0;
 int availableForWrite(){return room;}
 size_t write(const uint8_t*p,size_t n){assert(n<=48&&n<=size_t(room));text.append((const char*)p,n);if(n>maxWrite)maxWrite=n;++cycles;return n;}
};
struct IO {
 uint8_t rx[8]{};unsigned at=0,count=0,goals=0,ons=0;int16_t right=1675,left=2412;
 int available(){return count-at;}int read(){return rx[at++];}int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,size_t n){
  if(n==8){const uint16_t v=encodePosition(p[2]==1?right:left);uint8_t r[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};
   uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=r[i];r[7]=~sum;for(unsigned i=0;i<8;++i)rx[i]=r[i];at=0;count=8;}
  if(n==24){++goals;right=decodePosition(uint16_t(p[9])|uint16_t(p[10])<<8);left=decodePosition(uint16_t(p[17])|uint16_t(p[18])<<8);}
  if(n==12&&p[8])++ons;
  return n;
 }
};
int main(){
 assert(configuredAcceptance().extra&&configuredAcceptance().right_min==1675&&configuredAcceptance().right_max==2007&&
 configuredAcceptance().left_min==2071&&configuredAcceptance().left_max==2412);
 Control c;assert(!c.accepted(true,1674)&&!c.accepted(false,2413));
 ServoBus<IO>b;IO io;SerialDiagnostics records;SerialOutput out;Sink sink;
 auto drain=[&](){if(out.idle()){SerialDiagnostics::Record r;uint32_t lost;const bool have=records.pop(r,lost);assert(!lost);if(have)out.begin(r,lost);}out.service(sink);};
 auto cycle=[&](uint32_t t){c.tick(t,true,false,true,0);b.service(io,c,t);c.commit(t,true,Reason::Idle,b.stage(c));records.capture(c.state,b.stage(c),t);drain();};
 auto input=[&](uint8_t k,uint32_t t){c.input(k,true,t);records.input(k,true,t);};
 input(0,1000);cycle(1000);input(1,1001);
 for(unsigned t=1001;t<1600;++t)cycle(t);
 assert(io.goals==1&&io.ons==1&&io.right==1675&&io.left==2412&&c.state.recovery==1);
 assert(sink.text.find("right=1675 (100.000%) left=2412 (100.000%) result=ACCEPTED")!=std::string::npos);
 assert(sink.text.find("initial R=1675..2007 L=2071..2412")!=std::string::npos);
 assert(sink.text.find("UP=0 DOWN=1")!=std::string::npos);
 input(0,1600);cycle(1600);input(1,1601);cycle(1601);assert(c.state.reason==Reason::RecoveryDirection&&io.goals==1);
 input(4,1602);cycle(1602);assert(io.goals==1&&c.state.reason==Reason::RollUnconfirmed);
 input(0,1603);cycle(1603);input(2,1604);
 for(unsigned t=1604;t<2200;++t)cycle(t);
 assert(io.right>1675&&io.left<2412);
 input(0,2200);for(unsigned t=2200;t<2400;++t)cycle(t);
 const auto heldR=io.right,heldL=io.left;const auto goals=io.goals;
 for(unsigned t=2400;t<2600;++t)cycle(t);
 assert(io.right==heldR&&io.left==heldL&&io.goals==goals);
 input(2,2600);unsigned t=2600;
 while(c.state.recovery&&t<26000)cycle(t++);
 assert(!c.state.recovery&&inRange(true,c.target(true))&&inRange(false,c.target(false)));
 input(0,t++);for(unsigned k=0;k<400;++k)cycle(t++);
 assert(inRange(true,io.right)&&inRange(false,io.left));
 assert(sink.text.find("LEG STATE: NORMAL_RANGE")!=std::string::npos);
 const auto text=sink.text;for(unsigned k=0;k<1000;++k)cycle(t++);assert(sink.text==text); // no periodic held/idle spam
 // A disconnected/full USB sink never waits or modifies actuator state. Overflow is explicit.
 SerialDiagnostics full;State overflow;for(unsigned k=1;k<80;++k){overflow.rx_failure.issues=RxTimeout;overflow.rx_failure.ms=k;full.capture(overflow,0,k);}
 SerialDiagnostics::Record r;uint32_t lost;assert(full.pop(r,lost)&&lost>0);out.begin(r,lost);sink.room=0;
 const auto writes=sink.cycles;for(unsigned k=0;k<100;++k)out.service(sink);assert(sink.cycles==writes);
 sink.room=7;for(unsigned k=0;k<300;++k)out.service(sink);assert(sink.text.find("dropped=")!=std::string::npos);
 // Failure detail and bounded raw; consumer does not access UART or sensors.
 r=SerialDiagnostics::Record{};r.type=SerialDiagnostics::Receive;r.data.rx.target=1;r.data.rx.reg=56;r.data.rx.requested=2;
 r.data.rx.issues=RxId|RxChecksum;r.data.rx.frame_size=8;r.data.rx.frame[2]=2;r.data.rx.frame[3]=4;
 r.data.rx.expected_checksum=0xab;r.data.rx.received_checksum=0xcd;r.data.rx.checksum_ready=true;r.data.rx.raw_size=32;
 out.begin(r,0);sink.room=128;for(unsigned k=0;k<100;++k)out.service(sink);
 assert(sink.text.find("RIGHT ID=1 reg=56")!=std::string::npos&&sink.text.find("ID CHECKSUM")!=std::string::npos);
 assert(sink.text.find("expected=AB actual=CD")!=std::string::npos&&sink.text.find("LEG RX RAW:")!=std::string::npos);
 r=SerialDiagnostics::Record{};r.type=SerialDiagnostics::Configuration;r.data.config.status=2;r.data.config.current=r.data.config.acquired=15;
 r.data.config.maximum[0]=r.data.config.maximum[1]=4095;out.begin(r,0);for(unsigned k=0;k<100;++k)out.service(sink);
 assert(sink.text.find("LEG CONFIG: COMPLETE reason=NONE")!=std::string::npos&&sink.text.find("LEFT mode=0 limits=0..4095")!=std::string::npos);
 // Capture a real state transition to failed sync; repeated failure snapshots are silent.
 SerialDiagnostics failed;State failedState;failedState.positions.attempt_ms=500;
 failed.capture(failedState,2,500);failedState.reason=Reason::ReplyTimeout;
 failedState.rx_failure.ms=511;failedState.rx_failure.issues=RxTimeout;failedState.rx_failure.target=2;
 failed.capture(failedState,0,511);failed.capture(failedState,0,512);
 unsigned failureEvents=0;
 while(failed.pop(r,lost)){assert(!lost);failureEvents+=r.type==SerialDiagnostics::SyncFailed;
  if(r.type==SerialDiagnostics::SyncFailed)assert(r.reason==uint8_t(Reason::ReplyTimeout));}
 assert(failureEvents==1);
 SerialDiagnostics readonly;State observed;observed.positions.read_only=true;observed.positions.attempt_ms=600;
 readonly.capture(observed,4,600);observed.positions.current=3;observed.reason=Reason::PositionRange;
 observed.positions.word[0]=1675;observed.positions.word[1]=2412;readonly.capture(observed,0,601);
 unsigned readsDone=0;
 while(readonly.pop(r,lost)){assert(!lost&&r.type!=SerialDiagnostics::SyncFailed);
  if(r.type==SerialDiagnostics::SyncDone){assert(r.allowed);++readsDone;}}
 assert(readsDone==1); // Legacy read-only position query is not a failed motion synchronization.
 printf("PASS: production 1675/2412 exact hold/release/new DOWN/paired return and normal transition; bounded changed-only serial, full sink, loss, RX/config detail; records=%zu output=%zu record=%zu\n",sizeof(records),sizeof(out),sizeof(r));
}

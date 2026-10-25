#include "LegServoBus.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <initializer_list>
static bool forbid=false;
void*operator new(size_t n){assert(!forbid);if(auto*p=malloc(n))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{free(p);}void operator delete(void*p,size_t)noexcept{free(p);}
struct IO {
 uint8_t rx[1024]{},packets[256][24]{};unsigned head=0,tail=0,length[256]{},writes=0,reads=0;
 int read_budget=1024;int capacity=128;bool auto_reply=true,partial=false,bad_sum=false;int16_t right=1957,left=2122;
 int available(){return read_budget>0?tail-head:0;}int read(){++reads;--read_budget;return head<tail?rx[head++]:-1;}
 int availableForWrite(){return capacity;}
 void reply(uint8_t id,int16_t pos){
  uint8_t p[]={255,255,id,4,0,uint8_t(pos),uint8_t(pos>>8),0};
  uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=p[i];p[7]=uint8_t(~sum)+(bad_sum?1:0);
  assert(tail+8<sizeof(rx));memcpy(rx+tail,p,8);tail+=8;
 }
 size_t write(const uint8_t*p,unsigned n){
  assert(n<=24&&writes<256);length[writes]=n;memcpy(packets[writes++],p,n);
  uint8_t sum=0;for(unsigned i=2;i<n;++i)sum+=p[i];assert(sum==255);
  if(auto_reply&&n==8)reply(p[2],p[2]==leg::RightId?right:left);
  if(partial){partial=false;return n-1;}return n;
 }
};
void near(float a,float b){assert(fabsf(a-b)<.001f);}
void check(const leg::Control&c){
 assert(c.state.right_percent>=0&&c.state.right_percent<=30);
 assert(c.state.left_percent>=0&&c.state.left_percent<=30);
 near((c.state.right_percent+c.state.left_percent)/2,c.state.height);
}
int main(){
 using namespace leg;
 assert(position(true,0)==1997&&position(false,0)==2081);
 assert(position(true,30)==1908&&position(false,30)==2173);
 for(unsigned dt:{1u,5u,20u,50u}){
  Control c(true);c.input(0,true);c.tick(1000,true,false,true,0);c.input(1,true);c.tick(1001,true,false,true,0);
  assert(c.want_sync);assert(c.seed(position(true,15),position(false,15),1001));c.activate(1001);
  const float initial=c.state.height;
  for(uint32_t now=1001+dt;now<=2001;now+=dt){c.tick(now,true,false,true,0);check(c);}
  near(c.state.height,initial+HeightRate); // independent of packet/loop count
  c.input(3,true);c.tick(2051,true,false,true,0);near(c.state.height,initial+HeightRate);
  c.input(0,true);c.tick(2101,true,false,true,0);near(c.state.height,initial+HeightRate);
  c.input(4,true);for(uint32_t now=2151;now<=4101;now+=50)c.tick(now,true,false,true,0);
  near(c.state.requested_roll,MaxRoll);assert(c.state.right_percent>c.state.left_percent);
  const float height=c.state.height,target=c.state.requested_roll;
  c.input(12,true);c.tick(4151,true,false,true,0);near(c.state.requested_roll,target);
  c.input(8,true);for(uint32_t now=4201;now<=8151;now+=50)c.tick(now,true,false,true,0);
  near(c.state.requested_roll,-MaxRoll);assert(c.state.right_percent<c.state.left_percent);
  c.disconnect();for(uint32_t now=8201;now<=10151;now+=50)c.tick(now,true,false,true,0);
  near(c.state.requested_roll,0);near(c.state.height,height);
  c.input(0,true);c.tick(10201,true,false,true,2);assert(c.state.right_percent<c.state.left_percent); // error target-measured
 }
 for(float height:{0.f,1.f,29.f,30.f}){
  Control c(true);assert(c.seed(position(true,height),position(false,height),1000));c.activate(1000);c.input(4,true);
  for(uint32_t t=1050;t<5000;t+=50){c.tick(t,true,false,true,-10);check(c);}
  assert(c.state.limited);assert(fabsf(c.state.applied_roll)<=fminf(c.state.height,30-c.state.height)/RollGain+.001f);
 }
 for(bool high:{false,true}){
  Control x(true);x.seed(position(true,high?30:0),position(false,high?30:0),1000);x.activate(1000);
  x.input(high?1:2,true);x.tick(1050,true,false,true,0);assert(x.state.limited);
 }
 // Default height-only commissioning: no AngleX-driven differential, even tilted.
 {Control x;ServoBus<IO> b;IO uart;
  static_assert(!RollDirectionConfirmed,"default firmware must require measured roll direction");
  x.input(0,true);x.tick(1000,true,false,true,48);x.input(4,true);
  x.tick(1001,true,false,true,48);b.service(uart,x,1001);
  assert(!uart.writes&&!x.want_sync&&x.state.reason==Reason::RollUnconfirmed);
  x.input(1,true);
  for(uint32_t t=1002;t<1350;++t){x.tick(t,true,false,true,48);b.service(uart,x,t);}
  assert(x.state.active);const float h=x.state.height,d=x.state.right_percent-x.state.left_percent;
  x.input(5,true);for(uint32_t t=1350;t<2000;++t){x.tick(t,true,false,true,-48);b.service(uart,x,t);}
  assert(x.state.height>h);near(x.state.right_percent-x.state.left_percent,d);
  near(x.state.requested_roll,0);near(x.state.applied_roll,0);assert(x.state.reason==Reason::RollUnconfirmed);
  x.input(0,true);const float hold=x.state.height;
  for(uint32_t t=2000;t<2100;++t){x.tick(t,true,false,true,25);b.service(uart,x,t);}
  near(x.state.height,hold);near(x.state.right_percent-x.state.left_percent,d);
 }
 // A released external gate refreshes its diagnosis; it never starts an actuator.
 for(Reason prior:{Reason::ExplicitStop,Reason::Stopped,Reason::Initializing,
     Reason::ImuInvalid,Reason::Driving,Reason::WheelRetry}){
  Control x;x.stop(prior);x.input(0,true);
  x.tick(1000,false,false,true,0);assert(!x.want_sync&&!x.state.active);
  x.tick(1001,true,false,false,NAN);assert(x.state.reason==Reason::ImuInvalid);
  x.tick(1002,true,false,true,0);assert(x.state.reason==Reason::Idle);
  assert(!x.want_sync&&!x.state.active&&!x.state.synced);
 }
 for(Reason failure:{Reason::PositionRange,Reason::ReplyTimeout,Reason::ReplyInvalid,Reason::TxFailed}){
  Control x;x.stop(failure);x.input(0,true);x.tick(1000,true,false,true,0);
  assert(x.state.reason==failure&&!x.want_sync); // failures are not hidden by neutral packets
 }
 // Observations are bounded and change-driven; committed fields cannot mix with later input.
 {Control x;x.state.mode=1;x.state.operation.gates=Ready|Bench|Imu|Connected;
  x.input(0,true,1000);x.tick(1000,true,false,true,0);x.commit(1000,true,Reason::Stopped,0);
  const uint32_t stable=x.state.operation.revision;
  for(uint32_t t=1001;t<2000;++t){x.tick(t,true,false,true,0);x.commit(t,true,Reason::Stopped,0);}
  assert(x.state.operation.revision==stable&&x.state.operation.confirmed_ms==1999);
  x.stop(Reason::ReplyInvalid);x.observe(Source::Bus,2500,0);x.commit(2500,true,Reason::Stopped,0);
  assert(x.state.operation.failure==9&&x.state.operation.failure_ms==2500&&!x.state.operation.allowed);
  x.input(0,true,2501);x.tick(2501,true,false,true,0);x.commit(2501,true,Reason::Stopped,0);
  assert(x.state.operation.allowed&&x.state.operation.blocker==255&&x.state.operation.failure==9);
  const auto before=x.state.operation.confirmed;
  for(uint32_t t=2502;t<2520;++t)x.received(uint8_t(t&1),t);
  assert(x.state.operation.count==4&&x.state.operation.overwritten>0);
  assert(x.state.operation.confirmed.sequence==before.sequence&&x.state.operation.confirmed.raw==before.raw);
 }
 // Short release/press events in one FIFO batch retain the release gate.
 {Control x(true);x.input(0,true);x.input(1,true);x.tick(1000,true,false,true,0);assert(x.want_sync);}
 // Fragmented replies consume one byte per cycle without blocking.
 {Control x(true);ServoBus<IO> b;IO uart;x.input(0,true);x.tick(1000,true,false,true,0);x.input(1,true);
  for(uint32_t t=1001;t<1200;++t){uart.read_budget=1;x.tick(t,true,false,true,0);b.service(uart,x,t);}
  assert(x.state.active);}
 // A stale synchronized position cannot later enable an old goal.
 {Control x(true);ServoBus<IO> b;IO uart;x.input(0,true);x.tick(1000,true,false,true,0);x.input(1,true);
  for(uint32_t t=1001;t<1080;++t){x.tick(t,true,false,true,0);b.service(uart,x,t);}
  x.tick(2000,true,false,true,0);b.service(uart,x,2000);assert(!x.state.active);}
 {Control x(true);ServoBus<IO> b;IO uart;assert(b.observe(x));
  for(uint32_t t=1000;t<1200;++t)b.service(uart,x,t);
  assert(uart.writes==2&&!x.state.active&&!x.state.synced&&!b.observing());
  assert(b.readPosition(true)==uart.right&&b.readPosition(false)==uart.left);
  for(unsigned i=0;i<uart.writes;++i)assert(uart.length[i]==8);}
 // An initial held packet/reconnect cannot activate before release confirmation.
 Control c(true);ServoBus<IO> bus;IO io;
 c.input(1,false);c.tick(1000,true,false,true,0);bus.service(io,c,1000);assert(!io.writes&&!c.want_sync);
 c.input(0,true);c.tick(1001,true,false,true,0);c.input(1,true);
 for(uint32_t t=1002;t<1250;++t){
  const auto writes=io.writes,reads=io.reads;
  forbid=true;c.tick(t,true,false,true,0);bus.service(io,c,t);forbid=false;
  assert(io.writes-writes<=1&&io.reads-reads<=16);
 }
 assert(c.state.active&&c.state.synced&&io.writes>=4);
 assert(io.packets[0][4]==2&&io.packets[0][5]==56&&io.packets[0][2]==1);
 assert(io.packets[1][4]==2&&io.packets[1][2]==2);
 assert(io.length[2]==24&&io.packets[2][5]==41);
 assert((io.packets[2][9]|(io.packets[2][10]<<8))==io.right);
 assert((io.packets[2][17]|(io.packets[2][18]<<8))==io.left);
 assert(io.packets[3][5]==40&&io.packets[3][8]==1); // current-position seed precedes ON
 // Fall/explicit OFF is nonblocking; no old goal or HOME after recovery.
 bus.off(c);bus.service(io,c,1300);assert(!c.state.active&&c.state.torque==2);
 const auto written=io.writes;
 for(uint32_t t=1301;t<1400;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(io.writes==written); // held dpad does not rearm
 c.input(0,true);c.tick(1400,true,false,true,0);c.input(1,true);io.right=position(true,20);io.left=position(false,20);
 for(uint32_t t=1401;t<1650;++t){c.tick(t,true,false,true,0);bus.service(io,c,t);}
 assert(c.state.active&&c.state.height>19); // physical re-seed, never old 15% or HOME
 c.tick(1700,true,true,true,0);bus.cancel();assert(!c.state.active&&c.state.reason==Reason::Driving);
 c.tick(1750,true,false,false,NAN);assert(!c.state.active);
 for(unsigned failure=0;failure<4;++failure){
  Control x(true);ServoBus<IO> b;IO uart;
  uart.auto_reply=failure!=0;uart.bad_sum=failure==1;if(failure==2)uart.right=1900;if(failure==3)uart.partial=true;
  x.input(0,true);x.tick(1000,true,false,true,0);x.input(1,true);
  for(uint32_t t=1001;t<1250;++t){x.tick(t,true,false,true,0);b.service(uart,x,t);}
  assert(!x.state.active&&!x.state.synced);
  for(unsigned i=0;i<uart.writes;++i)assert(!(uart.packets[i][5]==40&&uart.packets[i][8]==1));
 }
 {Control x(true);ServoBus<IO> b;IO uart;uart.capacity=0;x.input(0,true);x.tick(1000,true,false,true,0);x.input(1,true);
  for(uint32_t t=1001;t<1200;++t){x.tick(t,true,false,true,0);b.service(uart,x,t);}assert(!uart.writes&&x.state.tx_busy<=4);
  b.off(x);uart.capacity=128;b.service(uart,x,1250);assert(x.state.torque==2&&!x.state.active);}
 {Control x(true);ServoBus<IO> b;IO uart;uart.capacity=0;
  for(uint32_t t=1000;t<1200;++t){b.off(x);b.service(uart,x,t);}
  assert(x.state.tx_busy<=4&&!uart.writes);uart.capacity=128;b.service(uart,x,1200);assert(x.state.torque==2);}
 printf("PASS: time-based axes/release/opposites/limits, error sign, sync-before-ON, fall/rearm, async timeout/OOM-TX/partial/checksum; control=%zu bus=%zu state=%zu\n",sizeof(Control),sizeof(ServoBus<IO>),sizeof(State));
}

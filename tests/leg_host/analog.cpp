#include "LegSerialDiagnostics.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <string>
using namespace leg;
int main(){
 static_assert(HeightRate==24&&RollRate==16.f/3&&MaxRoll==3,"preserve rates and roll limit");
 assert(ps4_control::legAxis(16)==0&&ps4_control::legAxis(-16)==0);
 assert(ps4_control::legAxis(127)==1&&ps4_control::legAxis(-128)==-1);
 assert(ps4_control::legAxis(17)>0&&ps4_control::legAxis(17)<.01f);
 assert(ps4_control::legAxis(-72)==-.5f);
 for(unsigned dt:{1u,5u,20u,50u})for(int raw:{-16,-17,-72,-128,16,17,72,127}){
  Control c;assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.stickInput(0,int8_t(raw),0,0,true,1000,1000);const float initial=c.state.height;float lo,hi;movementBounds(c.state.right_percent,c.state.left_percent,lo,hi,false);
  for(uint32_t t=1000+dt;t<=2000;t+=dt)c.tick(t,true,false,true,35);
  assert(fabsf(c.state.height-initial-(ps4_control::legAxis(raw)==0?0:clamp(heightTarget(heightInput(ps4_control::legAxis(raw)))-initial,lo,hi)))<.002f);
  const auto h=c.state.height;c.stickInput(0,0,0,0,true,2001,2001);c.tick(2051,true,false,true,-35);
  assert(fabsf(c.state.height-homeHeight())<=fabsf(h-homeHeight())+.00001f);
 }
 // Movement deadzone is wider than re-arm center. Zero mapped input in the band cannot arm.
 {Control c;c.stop(Reason::ReplyInvalid);
  for(uint32_t t=1000;t<1100;++t){c.stickInput(0,t&1?-17:-16,0,0,true,t,t);c.tick(t,true,false,true,0);assert(!c.want_sync);}
  c.stickInput(0,-11,0,0,true,1100,1100);c.tick(1100,true,false,true,0);assert(!c.want_sync);
  c.stickInput(0,-10,0,0,true,1101,1101);c.tick(1101,true,false,true,0);
  c.stickInput(0,-17,0,0,true,1102,1102);c.tick(1102,true,false,true,0);assert(c.want_sync);
 }
 {Control c;c.input(0,true);c.tick(1000,true,false,true,0);
  c.stickInput(-128,0,0,0,true,1001,1001);c.tick(1001,true,false,true,30);c.commit(1001,true,Reason::Idle,0);
  assert(!c.want_sync&&!c.state.operation.allowed&&c.state.operation.blocker==uint8_t(Reason::RollUnconfirmed));
  // X-only does not consume a previously confirmed center; adding Y permits height only.
  c.stickInput(-128,127,0,0,true,1002,1002);c.tick(1002,true,false,true,30);assert(c.want_sync);
  assert(c.seed(position(true,15),position(false,15),1002));c.activate(1002);c.tick(1052,true,false,true,30);
  c.commit(1052,true,Reason::Idle,8);
  assert(c.state.stick.applied_height==1&&c.state.stick.applied_roll==0&&c.state.requested_roll==0);
  assert(c.state.operation.allowed&&c.state.operation.operation==uint8_t(Operation::Extend));
 }
 {Control c(true);assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.stickInput(-72,0,0,0,true,1000,1000);for(uint32_t t=1050;t<=2000;t+=50)c.tick(t,true,false,true,0);
  assert(fabsf(c.state.requested_roll-.9375f)<.0001f);
  c.stickInput(0,0,0,0,true,2001,2001);for(uint32_t t=2051;t<2700;t+=50)c.tick(t,true,false,true,0);
  assert(c.state.requested_roll==0);
 }
 // Raw observation is independent of routing; stale input cannot change command rates.
 {Control c;assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.stickInput(0,0,0,0,true,1000,1000);const auto h=c.state.height;
  c.rawStick(0,127,127,-128,999);c.tick(1050,true,false,true,0);assert(c.state.height==h&&c.state.stick.height==1);
 }
 // Stable raw input, accepted rate and changing timestamps alone never produce repeated records.
 {SerialDiagnostics log(true);Control c;assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.stickInput(0,-72,0,0,true,1000,1000);c.state.stick.flags=97;c.tick(1050,true,false,true,0);
  log.capture(c.state,8,1050);SerialDiagnostics::Record record;uint32_t lost;while(log.pop(record,lost))assert(!lost);
  for(uint32_t t=1051;t<1200;++t){c.tick(t,true,false,true,0);log.capture(c.state,8,t);}
  assert(!log.pop(record,lost)&&!lost);
  c.stickInput(0,-72,0,0,true,1200,1200);c.tick(1200,true,false,true,0);log.capture(c.state,8,1200);
  assert(!log.pop(record,lost));
 }
 printf("PASS: continuous analog DZ16/endpoints/rate-time scaling, center10 hysteresis, unconfirmed X+Y height only, roll target/ramp, stale observation cannot route\n");
}

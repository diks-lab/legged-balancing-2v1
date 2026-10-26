#include "LegControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace leg;
static_assert(HeightRate==24&&RollGain==3&&MaxRoll==3&&DifferentialRate==16&&RollRate==16.f/3);
static_assert(ServoSpeed==150&&ServoAcceleration==15&&SendIntervalMs==50&&TargetHysteresisTicks==.25f);
int main(){
 activeGeometry()=ConfirmedCandidate;
 assert(fabsf(homeHeight()-16.512346f)<.0001f);
 assert(heightInput(.5f)==.3125f&&rollInput(.5f)==.3125f);
 float previous=-2;
 for(int raw=-128;raw<=127;++raw){
  const auto normalized=ps4_control::legAxis(raw);const auto shaped=heightInput(normalized);
  assert(shaped>=previous&&shaped>=-1&&shaped<=1);previous=shaped;
  if(raw>=-16&&raw<=16)assert(shaped==0);else assert(shaped*normalized>0);
  assert(fabsf(shaped)<=fabsf(normalized));
 }
 assert(heightInput(1)==1&&heightInput(-1)==-1);
 for(unsigned dt:{1u,2u,5u,10u,20u,50u})for(int raw:{-17,-40,-72,-128,17,40,72,127}){
  Control c(true);assert(c.seed(position(true,16),position(false,16),1000));c.activate(1000);
  c.stickInput(0,0,0,0,true,1000,1000);c.tick(1000,true,false,true,2);
  const float r0=c.state.right_percent,l0=c.state.left_percent;
  c.tick(1000+dt,true,false,true,2);assert(c.state.right_percent==r0&&c.state.left_percent==l0); // sync/center alone cannot start compensation
  const auto height=c.state.height;const uint32_t start=1000+dt;
  c.stickInput(int8_t(raw),0,0,0,true,start,start);
  auto advance=[&](uint32_t t){
   const float angle=c.state.requested_roll,diff=(c.state.right_percent-c.state.left_percent)/2;
   c.tick(t,true,false,true,0);
   assert(fabsf(c.state.requested_roll-angle)<=RollRate*dt*.001f+.00001f);
   assert(fabsf((c.state.right_percent-c.state.left_percent)/2-diff)<=16*dt*.001f+.00001f);
   assert(c.state.height==height&&inRange(true,c.target(true))&&inRange(false,c.target(false)));
  };
  for(unsigned t=dt;t<=2000;t+=dt)advance(start+t);
  const float wanted=3*rollInput(-ps4_control::legAxis(raw));
  assert(fabsf(c.state.requested_roll-wanted)<.0001f);
  for(unsigned t=2000+dt;t<=4000;t+=dt)advance(start+t);
  assert(fabsf(c.state.requested_roll-wanted)<.0001f); // holding never integrates further
  c.stickInput(raw<0?127:-128,0,0,0,true,start+4000,start+4000);
  for(unsigned t=4000+dt;t<=7000;t+=dt)advance(start+t);
  assert(c.state.requested_roll==(raw<0?-3:3));
  c.stickInput(0,0,0,0,true,start+7000,start+7000);
  for(unsigned t=7000+dt;t<=8500;t+=dt)advance(start+t);
  assert(c.state.requested_roll==0);
  // Y is a HOME-centered position demand, with the same bounded rate.
  c.stickInput(0,int8_t(raw),0,0,true,start+8500,start+8500);const float h=c.state.height;
  for(unsigned t=8500+dt;t<=9500;t+=dt)c.tick(start+t,true,false,true,0);
  assert(fabsf(c.state.height-h-clamp(heightTarget(heightInput(ps4_control::legAxis(raw)))-h,-24.f,24.f))<.002f);
  c.stickInput(0,0,0,0,true,start+9500,start+9500);const auto held=c.state.height;
  for(unsigned t=9500+dt;t<=10500;t+=dt)c.tick(start+t,true,false,true,0);
  assert(fabsf(c.state.height-homeHeight())<=fabsf(held-homeHeight()));
  for(unsigned t=10500+dt;t<=11500;t+=dt)c.tick(start+t,true,false,true,0);
  assert(fabsf(c.state.height-homeHeight())<.0001f);
 }
 puts("PASS: monotone soft curve/DZ/endpoints; direct position roll holds without accumulation, reversal/center <=16 pp/s at 1/2/5/10/20/50ms, Y HOME-position/return, sync-only compensation gate");
}

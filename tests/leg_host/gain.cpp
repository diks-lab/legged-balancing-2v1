#include "LegControl.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
using namespace leg;
static_assert(RollGain==EXPECTED_GAIN,"compile actual old/new control header");
static_assert(RollRate*RollGain==DifferentialRate&&MaxRoll==3&&DifferentialRate==8&&HeightRate==12,"preserve rates/target limit");
static_assert(SendIntervalMs==50&&ServoSpeed==150&&ServoAcceleration==15,"preserve servo settings");
int main(){
 activeGeometry()=ConfirmedCandidate;
 for(unsigned dt:{1u,5u,10u,20u,50u})for(float error:{-.25f,.25f,-.5f,.5f,-1.f,1.f}){
  Control c(true);assert(c.seed(position(true,15),position(false,15),1000));c.activate(1000);
  c.input(4,true);c.tick(1050,true,false,true,0);c.input(0,true);c.tick(1100,true,false,true,0);
  for(unsigned t=1150;t<=1300;t+=50)c.tick(t,true,false,true,0);
  const float height=c.state.height;
  for(unsigned t=1300+dt;t<=2300;t+=dt){
   const float before=(c.state.right_percent-c.state.left_percent)/2;
   c.tick(t,true,false,true,error);
   const float after=(c.state.right_percent-c.state.left_percent)/2;
   assert(fabsf(after-before)<=8*dt*.001f+.00001f);
   assert(c.state.height==height&&!c.state.limited);
   assert(inRange(true,c.target(true))&&inRange(false,c.target(false)));
  }
  const float diff=(c.state.right_percent-c.state.left_percent)/2;
  assert(error*diff<0);printf("%.7g ",double(diff));
  // Large errors hit the original range protection, without bypassing the slew cap.
  for(unsigned t=2300+dt;t<=7300;t+=dt){
   const float before=(c.state.right_percent-c.state.left_percent)/2;
   c.tick(t,true,false,true,error>0?100:-100);
   const float after=(c.state.right_percent-c.state.left_percent)/2;
   assert(fabsf(after-before)<=8*dt*.001f+.00001f);
   assert(c.state.height==height&&inRange(true,c.target(true))&&inRange(false,c.target(false)));
  }
  assert(c.state.limited);
 }
 puts("");
}

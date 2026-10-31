#include "LegServoBus.h"
#include <cassert>
#include <cstdio>
#include <cmath>
#include <initializer_list>
using namespace leg;
struct IO {
 uint8_t rx[8]{};unsigned at=0,end=0,goals=0;int lastR=1992,lastL=2133,travel=0;
 int available(){return end-at;}int read(){return rx[at++];}int availableForWrite(){return 128;}
 size_t write(const uint8_t*p,size_t n){
  if(n==8){const auto v=encodePosition(p[2]==1?lastR:lastL);uint8_t r[]={255,255,p[2],4,0,uint8_t(v),uint8_t(v>>8),0};uint8_t sum=0;for(unsigned i=2;i<7;++i)sum+=r[i];r[7]=uint8_t(~sum);for(unsigned i=0;i<8;++i)rx[i]=r[i];at=0;end=8;}
  else {assert(n==24&&p[5]==41&&p[6]==7);assert(p[8]==15&&p[16]==15&&p[13]==150&&p[21]==150);
   assert(p[11]==0&&p[12]==0&&p[19]==0&&p[20]==0);
   const int r=decodePosition(uint16_t(p[9])|(uint16_t(p[10])<<8)),l=decodePosition(uint16_t(p[17])|(uint16_t(p[18])<<8));
   assert(inRange(true,r)&&inRange(false,l));travel+=abs(r-lastR)+abs(l-lastL);lastR=r;lastL=l;++goals;
  }return n;
 }
};
float pct(bool right,float tick){return 100*(tick-zeroTick(right))/float(fullTick(right)-zeroTick(right));}
void ready(Control&c){assert(c.seed(1992,2133,1000));c.activate(1000);c.state.command_valid=true;c.state.torque=3;c.state.right_command=1992;c.state.left_command=2133;}
int main(){
 activeGeometry()=ConfirmedCandidate;
 Control c(true);ServoBus<IO>b;IO io;ready(c);
 // Identical 50ms desired positions on either side of a rounding boundary.
 for(unsigned i=0;i<100;++i){
  const float delta=i&1?.55f:.45f;
  c.state.right_percent=pct(true,1992+delta);c.state.left_percent=pct(false,2133+delta);
  const float r=c.state.right_percent,l=c.state.left_percent;
  b.service(io,c,1000+i*50);assert(c.state.right_percent==r&&c.state.left_percent==l);
  assert(fabsf(1992+delta-c.state.right_command)<=.5f+TargetHysteresisTicks+.001f);
 }
 printf("%u %d\n",io.goals,io.travel);
 // Both directions, then a continuous return, keep sub-tick intent and range.
 for(int direction:{-1,1}){
  Control x(true);ServoBus<IO>bus;IO port;ready(x);
  const float r0=x.state.right_percent,l0=x.state.left_percent;
  for(unsigned phase=0;phase<2;++phase)for(unsigned i=0;i<=20;++i){
   const float elapsed=(phase?20-i:i)*.05f;
   x.state.right_percent=r0+direction*4*elapsed;x.state.left_percent=l0-direction*4*elapsed;
   const float rt=zeroTick(true)+(fullTick(true)-zeroTick(true))*x.state.right_percent/100;
   const float lt=zeroTick(false)+(fullTick(false)-zeroTick(false))*x.state.left_percent/100;
   const int r=port.lastR,l=port.lastL;bus.service(port,x,1000+(phase*21+i)*50);
   assert(fabsf(rt-x.state.right_command)<=.5f+TargetHysteresisTicks+.001f&&fabsf(lt-x.state.left_command)<=.5f+TargetHysteresisTicks+.001f);
   const int sign=direction*(phase?-1:1);
   if(!((port.lastR-r)*sign<=0&&(port.lastL-l)*sign<=0))fprintf(stderr,"reverse hyst=%g dir=%d phase=%u i=%u prev=%d/%d now=%d/%d raw=%f/%f\n",double(TargetHysteresisTicks),direction,phase,i,r,l,port.lastR,port.lastL,double(rt),double(lt));
   assert((port.lastR-r)*sign<=0&&(port.lastL-l)*sign<=0);
  }
 }
 // Initial/outer-range coordinates never use hysteresis or clamp acquired ticks.
 {Control x(true);assert(x.seed(1697,2408,1000));x.activate(1000);x.state.command_valid=true;x.state.right_command=1697;x.state.left_command=2408;
  x.state.right_percent=pct(true,1697.55f);assert(x.target(true)==1698);
  x.state.recovery=0;x.state.active=false;assert(x.target(true)==1698);
 }
}

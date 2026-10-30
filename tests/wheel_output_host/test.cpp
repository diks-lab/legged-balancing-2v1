#include "BLDCMotor.h"
#include "drivers/BLDCDriver3PWM.h"
#include "WheelOutputObservation.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <mutex>
struct Lock {std::mutex m;void lock(){m.lock();}void unlock(){m.unlock();}};
struct TestSensor:Sensor {using Sensor::init;int reads=0;float raw=1;float getSensorAngle() override{++reads;return raw;}};
float written[2][3];
void* _configure3PWM(long,int a,int,int){return &written[a==26];}
void _writeDutyCycle3PWM(float a,float b,float c,void* p){float* v=(float*)p;v[0]=a;v[1]=b;v[2]=c;}
void near(float a,float b){assert(std::abs(a-b)<1e-5f);}
int main(){
 TestSensor s[2];BLDCMotor r(7),l(7);BLDCMotor* motors[]={&r,&l};
 BLDCDriver3PWM dr(13,12,14,27),dl(26,25,33,32);BLDCDriver3PWM* drivers[]={&dr,&dl};
 wheeloutput::Observation<Lock> obs;
 for(unsigned i=0;i<2;++i){auto&m=*motors[i];auto&d=*drivers[i];
   s[i].init();m.linkSensor(&s[i]);d.voltage_power_supply=d.voltage_limit=8.3f;assert(d.init());
   m.linkDriver(&d);m.controller=MotionControlType::torque;m.torque_controller=TorqueControlType::voltage;
   m.voltage_limit=1;m.sensor_direction=Direction::CCW;m.zero_electric_angle=i?5.8521371f:1.3360963f;
   assert(m.init());m.motor_status=FOCMotorStatus::motor_ready;
 }
 for(float sign:{-1.f,1.f}){
   assert(obs.request(test_us));assert(!obs.request(test_us));
   const float commands[]={sign,-sign};
   r.loopFOC();l.loopFOC();const float previous=r.voltage.q;
   for(unsigned i=0;i<2;++i)motors[i]->move(commands[i]);
   near(r.voltage.q,previous); // move has NOT applied the new command.
   obs.moved(test_us,test_us+1,{commands[0],r.target,r.current_sp},{commands[1],l.target,l.current_sp});
   const uint32_t sensor_us=test_us+=1000;uint32_t starts[2],ends[2];
   for(unsigned i=0;i<2;++i){starts[i]=test_us++;motors[i]->loopFOC();ends[i]=test_us++;}
   const int reads=s[0].reads+s[1].reads;
   auto ro=wheeloutput::cached(r,dr,s[0],1),lo=wheeloutput::cached(l,dl,s[1],1);
   assert(reads==s[0].reads+s[1].reads);
   obs.foc(sensor_us,starts,ends,ro,lo,true);auto record=obs.copy();
   assert(record.state==3&&record.applied);near(record.move[1].command,-sign);
   for(unsigned i=0;i<2;++i){auto&m=*motors[i];auto&d=*drivers[i];const auto&o=record.output[i];
     near(m.current_sp,commands[i]);near(m.voltage.q,commands[i]);near(m.voltage.d,0);
     near(o.duty[0],written[i][0]);near(o.duty[1],written[i][1]);near(o.duty[2],written[i][2]);
     assert(o.duty[0]>=0&&o.duty[0]<=1);assert(std::max({d.dc_a,d.dc_b,d.dc_c})>std::min({d.dc_a,d.dc_b,d.dc_c}));
   }
   char text[1900];assert(wheeloutput::format(record,text,sizeof(text))<int(sizeof(text)));
   assert(strstr(text,"NOT measured pin waveforms")&&strstr(text,"LEFT")&&strstr(text,"RIGHT"));
 }
 // Velocity input is rad/s and passes through PID: never equate it to volts.
 l.controller=MotionControlType::velocity;l.PID_velocity.P=.075f;l.PID_velocity.I=0;l.PID_velocity.D=0;
 l.PID_velocity.output_ramp=0;l.PID_velocity.limit=1;l.move(6);near(l.current_sp,.45f);
 l.loopFOC();near(l.voltage.q,.45f);
 // Clamping, disabled move/FOC, stop between the two cycles and no auto-retry.
 r.move(5);r.loopFOC();near(r.voltage.q,1);
 assert(obs.request(test_us));obs.moved(10,11,{1,1,1},{-1,-1,-1});obs.interrupt();
 uint32_t times[]={20,21};obs.foc(19,times,times,{}, {},true);assert(obs.copy().state==4&&!obs.copy().applied);
 assert(obs.request(test_us));obs.moved(10,11,{1,1,1},{-1,-1,-1});
 obs.foc(19,times,times,{}, {},false);assert(!obs.copy().applied);
 // Extreme finite values must fit the bounded HTTP text buffer.
 auto extreme=obs.copy();
 for(auto&o:extreme.output){o.q=o.d=o.electrical=o.mechanical=o.shaft_angle=o.shaft_velocity=-3.402823e38f;
  for(unsigned i=0;i<3;++i)o.phase[i]=o.duty[i]=-3.402823e38f;}
 char bounded[1900];assert(wheeloutput::format(extreme,bounded,sizeof(bounded))<int(sizeof(bounded)));
 r.disable();r.move(-1);r.loopFOC();near(r.current_sp,5); // target updated, setpoint unchanged while disabled.
 std::cout<<"PASS: real SimpleFOC move N / FOC N+1, both signs, saved CCW zeros, phase/PWM caches, voltage vs velocity, limits/disabled/interrupted capture; recorder="<<sizeof(obs)<<" record="<<sizeof(wheeloutput::Record)<<" B\n";
}
// Existing alignment-observation hooks are not used by this output-path test.
#include "FocAlignmentObservation.h"
void observeFocLibraryDisable(unsigned int){}
void observeFocAlignPoint(const void*,FocAlignPoint,float,unsigned int){}
void observeFocAlignPp(const void*,float,bool){}

"""Compare real SimpleFOC alignment before/after observation and run the app wrapper."""
from pathlib import Path
import importlib.util
import hashlib
import re
import subprocess
import sys
import tempfile
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('foc_patch','scripts/patch_foc_observation.py')
patch=importlib.util.module_from_spec(spec);spec.loader.exec_module(patch)
lib=Path('.pio/libdeps/esp32dev/Arduino-FOC/src/common/base_classes/FOCMotor.cpp').read_text()
if hashlib.sha256(lib.encode()).hexdigest()==patch.PATCHED_SHA256:
    original=lib
    for before,after in reversed(patch.CHANGES):original=original.replace(after,before,1)
else:original=lib
assert hashlib.sha256(original.encode()).hexdigest()==patch.ORIGINAL_SHA256
patched=original
for before,after in patch.CHANGES:patched=patched.replace(before,after,1)
assert hashlib.sha256(patched.encode()).hexdigest()==patch.PATCHED_SHA256

def function(source,name):
    m=re.search(r'^(?:void|bool|int)\s+'+re.escape(name)+r'\([^;]*?\)\s*\{',source,re.M)
    assert m,name
    i=m.end();depth=1
    while depth:
        depth+=(source[i]=='{')-(source[i]=='}');i+=1
    return source[m.start():i]

app=Path('src/main.cpp').read_text()
prelude=r'''
#include "FocAlignmentObservation.h"
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
using portMUX_TYPE=std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
void portENTER_CRITICAL(portMUX_TYPE* m){m->lock();}
void portEXIT_CRITICAL(portMUX_TYPE* m){m->unlock();}
uint32_t now_us=100;
uint32_t micros(){return now_us;}
uint32_t _micros(){return now_us;}
bool en[2]={true,true},bus_ok=true;
int fault_mode=0,timer_start_result=0,timer_stop_result=0;
std::vector<unsigned long> delays;
std::vector<float> commands;
std::atomic<bool> wheelAlignExpired{false};
void* wheelAlignTimer=reinterpret_cast<void*>(1);
constexpr int ESP_OK=0;
void gateWheelOutputsOffFrom(FocGateSource);
class Print {
 public:
 virtual ~Print()=default;
 virtual size_t write(uint8_t)=0;
};
'''
start=app.index('struct FocObservationLock {');end=app.index('std::atomic<const char*> foc_deferred_http_failure',start)
prelude+=app[start:end]
prelude+=r'''
struct {
 std::string output;
 void print(const char* text){assert(!foc_alignment_observer.active() && !en[0] && !en[1]);output+=text;}
 size_t write(uint8_t byte){char text[]={static_cast<char>(byte),0};print(text);return 1;}
} Serial;
struct SimpleFOCDebug {
 static Print* sink;
 static void enable(Print* print){sink=print;}
 template<class T> static void enable(T*){} // Serial restore only, no raw alignment output.
};
Print* SimpleFOCDebug::sink=nullptr;
void debug(const char* text){if(SimpleFOCDebug::sink)while(*text)SimpleFOCDebug::sink->write(*text++);}
#define SIMPLEFOC_MOTOR_DEBUG(msg,...) debug(msg)
#define SIMPLEFOC_MOTOR_ERROR(msg,...) debug(msg)
#define SIMPLEFOC_MOTOR_WARN(msg,...) debug(msg)
constexpr float _2PI=6.28318530718f,_3PI_2=4.71238898038f,NOT_SET=-12345.0f;
#define MIN_ANGLE_DETECT_MOVEMENT (_2PI/101.0f)
bool _isset(float value){return value!=NOT_SET;}
void _delay(unsigned long ms){
 delays.push_back(ms);now_us+=ms*1000;
 if(fault_mode && now_us>400000 && en[1]){
   const auto source=fault_mode==1?FocGateSource::Timer:(fault_mode==2?FocGateSource::WebAbort:FocGateSource::LeftSensor);
   gateWheelOutputsOffFrom(source);
   if(fault_mode==1)wheelAlignExpired.store(true);
   if(fault_mode==3)bus_ok=false;
 }
}
struct FakeSensor {
 const char* error="OK";float angle=0,command=_3PI_2,estimated_pp=7;int updates=0;
 bool search=false;bool needsSearch(){return search;}
 void update(){++updates;if(en[1])angle=-command/estimated_pp;}
 float getAngle(){return angle;}float getMechanicalAngle(){return angle;}
} sensor1,sensor2;
enum class Direction {UNKNOWN=0,CW=1,CCW=-1};
enum class FOCMotorStatus {motor_calibrating,motor_ready,motor_calib_failed};
enum class MotionControlType {angle_openloop,velocity_openloop,torque};
struct FakeCurrent {bool initialized=true;};
class FOCMotor {
 public:
 FakeSensor* sensor=&sensor2;FakeCurrent* current_sense=nullptr;
 Direction sensor_direction=Direction::UNKNOWN;int pole_pairs=7;
 float voltage_sensor_align=2,zero_electric_angle=NOT_SET,shaft_angle=0;
 bool pp_check_result=false;
 MotionControlType controller=MotionControlType::torque;
 FOCMotorStatus motor_status=FOCMotorStatus::motor_calibrating;
 int initFOC();int alignSensor();
 int absoluteZeroSearch(){return 0;}
 int alignCurrentSense(){return 1;}
 float shaftAngle(){return sensor->getAngle();}
 float electricalAngle(){return static_cast<int>(sensor_direction)*pole_pairs*sensor->getMechanicalAngle();}
 void setPhaseVoltage(float,float,float angle){commands.push_back(angle);if(sensor)sensor->command=angle;}
 void disable(){en[0]=en[1]=false;}
};
using BLDCMotor=FOCMotor;
BLDCMotor motor1,motor2;
bool wheelSensorsOK(){return bus_ok;}
int esp_timer_start_once(void*,unsigned long us){assert(us==6000000);return timer_start_result;}
int esp_timer_stop(void*){return timer_stop_result;}
using gpio_num_t=int;
constexpr int motor1EnablePin=27,motor2EnablePin=32,LOW=0;
void gpio_set_level(int pin,int level){assert(level==LOW);en[pin==27?0:1]=false;}

void observeFocAlignPoint(const void* m,FocAlignPoint p,float v,uint32_t now){foc_alignment_observer.point(m,p,v,now);}
void observeFocAlignPp(const void* m,float moved,bool ok){foc_alignment_observer.pp(m,moved,ok);}
void observeFocLibraryDisable(uint32_t now){foc_alignment_observer.gate(FocGateSource::LibraryDisable,now);}
'''
checks=r'''
void reset(){
 now_us=100;en[0]=en[1]=bus_ok=true;fault_mode=0;wheelAlignExpired=false;
 timer_start_result=timer_stop_result=0;wheelAlignTimer=reinterpret_cast<void*>(1);
 sensor1=FakeSensor{};sensor2=FakeSensor{};motor1=BLDCMotor{};motor2=BLDCMotor{};
 delays.clear();commands.clear();Serial.output.clear();foc_quiet_print.clear();
}
int main(){
 for(float estimate:{7.0f,7.64f,7.66f,100000.0f}){
  reset();sensor2.estimated_pp=estimate;BaselineMotor baseline;
  int raw=baseline.initFOC();auto base_delays=delays;auto base_commands=commands;
  const int base_updates=sensor2.updates;const bool base_pp=baseline.pp_check_result;
  reset();sensor2.estimated_pp=estimate;initializeWheelFoc(motor2);
  assert(foc_alignment_observer.records[1].raw_result==raw && motor2.pp_check_result==base_pp);
  assert(delays==base_delays && commands==base_commands && sensor2.updates==base_updates);
 }
 reset();assert(initializeWheelFoc(motor2));
 auto r=foc_alignment_observer.records[1];
 assert(r.raw_result==1 && r.pp_observed && r.pp_pass && r.valid==31 && r.accepted);
 assert(sensor2.updates==1006); // 2*501 sweep + mid/end + zero + post-init.
 assert(delays.size()==1006 && commands.size()==1004);
 assert(r.point_us[1]-r.point_us[0]==1002000);
 assert(r.point_us[2]-r.point_us[1]==1002000);
 assert(r.point_us[4]-r.point_us[3]==920000 && r.total_us==3124000);
 assert(r.gates[static_cast<unsigned>(FocGateSource::Cleanup)].count==1);
 assert(Serial.output.find("raw_initFOC=1")!=std::string::npos);
 for(float estimate:{7.64f,7.66f}){
  reset();sensor2.estimated_pp=estimate;assert(!initializeWheelFoc(motor2));
  r=foc_alignment_observer.records[1];assert(r.raw_result==1 && !r.pp_pass && r.pp_observed && r.valid==31);
  assert(std::fabs(r.moved-_2PI/estimate)<0.00001f);
 }
 reset();sensor2.estimated_pp=100000;assert(!initializeWheelFoc(motor2));
 r=foc_alignment_observer.records[1];assert(r.raw_result==0 && !r.pp_observed && r.valid==7);
 assert(r.gates[static_cast<unsigned>(FocGateSource::LibraryDisable)].count==1);
 for(int fault:{1,2,3}){
  reset();fault_mode=fault;assert(!initializeWheelFoc(motor2));r=foc_alignment_observer.records[1];
  unsigned source=static_cast<unsigned>(fault==1?FocGateSource::Timer:(fault==2?FocGateSource::WebAbort:FocGateSource::LeftSensor));
  assert(r.gates[source].count==1 && r.gates[source].first_us>0);
  assert((fault!=1 || r.expired) && (fault!=3 || !r.sensors_ok));
 }
 reset();timer_start_result=99;assert(!initializeWheelFoc(motor2));
 r=foc_alignment_observer.records[1];assert(r.raw_result==-1 && r.timer_start==99 && r.valid==0 && !en[0] && !en[1]);
 reset();bus_ok=false;assert(!initializeWheelFoc(motor2));assert(!en[0] && !en[1]);
 reset();wheelAlignTimer=nullptr;assert(!initializeWheelFoc(motor2));assert(!en[0] && !en[1]);
 reset();foc_quiet_print.clear();for(int i=0;i<700;++i)foc_quiet_print.write('x');
 assert(foc_quiet_print.used==512 && foc_quiet_print.dropped==188);
 // Multiple producers retain exact counts; first_us=0 is a valid first event.
 reset();foc_alignment_observer.begin(0,&motor1,now_us);
 auto producer=[]{for(int i=0;i<10000;++i)foc_alignment_observer.gate(FocGateSource::Timer,100);};
 std::thread a(producer),b(producer);a.join();b.join();
 foc_alignment_observer.end(1,0,0,false,true,true,true,200);
 r=foc_alignment_observer.records[0];assert(r.gates[1].count==20000 && r.gates[1].first_us==0);
 reset();foc_alignment_observer.begin(0,&motor1,0xfffffff0u);
 foc_alignment_observer.point(&motor1,FocAlignPoint::ForwardBegin,0,0xfffffff5u);
 foc_alignment_observer.point(&motor1,FocAlignPoint::ForwardEnd,1,0x15u);
 foc_alignment_observer.end(1,0,0,false,true,true,true,0x30u);
 r=foc_alignment_observer.records[0];assert(r.total_us==64 && r.point_us[1]-r.point_us[0]==32);
 puts("PASS: real alignment/raw-vs-PP, exact update/command/delay counts, timing, gate sources, early failures, post-gate output, bounded capture and concurrent counters");
}
'''
with tempfile.TemporaryDirectory(prefix='foc-observe-') as directory:
    path=Path(directory)/'FOCMotor.cpp';path.write_text(original)
    assert patch.patch(path);assert not patch.patch(path);assert path.read_text()==patched
    path.write_text(original+'\n// drift');before=path.read_bytes()
    try:patch.patch(path);raise AssertionError('accepted mismatched source')
    except RuntimeError:pass
    assert path.read_bytes()==before
    # Exact reverse transformation proves that no library algorithm text changed.
    reverse=patched
    for before,after in reversed(patch.CHANGES):reverse=reverse.replace(after,before,1)
    assert reverse==original
    cpp=Path(directory)/'test.cpp';binary=Path(directory)/'test'
    definition=prelude[prelude.index('class FOCMotor {'):prelude.index('using BLDCMotor=')]
    baseline=re.sub(r'\bFOCMotor\b','BaselineMotor',definition+'\n'+'\n'.join(function(original,n) for n in ['FOCMotor::initFOC','FOCMotor::alignSensor']))
    code=prelude+'\n'+function(app,'gateWheelOutputsOffFrom')+'\n'+baseline+'\n'+'\n'.join(function(patched,n) for n in ['FOCMotor::initFOC','FOCMotor::alignSensor'])
    code+='\n'+function(app,'printFocAlignmentObservation')+'\n'+function(app,'initializeWheelFoc')+checks
    cpp.write_text(code)
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-pthread','-Iinclude',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
# Hooks only consume cached getters or local values. No extra sensor update or wait.
for n in ['FOCMotor::initFOC','FOCMotor::alignSensor']:
    a,b=function(original,n),function(patched,n)
    assert a.count('sensor->update()')==b.count('sensor->update()')
    assert re.findall(r'_delay\([^)]*\)',a)==re.findall(r'_delay\([^)]*\)',b)
assert 'sensor->getAngle()' in patch.CHANGES[2][1] # Cached getter, not update/I2C.

gate=function(app,'gateWheelOutputsOffFrom')
assert gate.index('gpio_set_level(static_cast<gpio_num_t>(motor2EnablePin)')<gate.index('foc_alignment_observer.gate(')

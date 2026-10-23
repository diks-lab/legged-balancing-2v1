from pathlib import Path
import subprocess, tempfile
code=r'''
#include "HealthDiagnostics.h"
#include <cassert>
#include <cstring>
#include <cstdio>
struct Bus {
 int step=0,fail=-1,reg=0,pos=0,mismatch=-1;
 void beginTransmission(int a){assert(a==0x68);pos=0;}
 void write(uint8_t b){if(!pos++)reg=b;}
 int endTransmission(bool){return step++==fail?4:0;}
 int requestFrom(uint8_t,uint8_t){return step++==fail?0:1;}
 int read(){if(step++==fail)return -1;return (reg==0x1a?3:reg==0x1b?8:0)+(reg==mismatch);}
};
int main(){
 using namespace imu_config;
 Result r;assert(r.failure==Failure::NotRun);
 for(int fail=0;fail<13;++fail){
  Bus b;b.fail=fail;assert(!apply(b,r));
  const int index=fail?((fail-1)/3):0;
  const uint8_t regs[]={0x1a,0x19,0x1b,0x1c},expected[]={3,0,8,0};
  assert(r.reg==regs[index]&&r.expected==expected[index]&&r.actual==-1);
  if(!fail)assert(r.failure==Failure::Write&&r.tx==4&&r.rx==-1&&r.read==-1);
  else if((fail-1)%3==0)assert(r.failure==Failure::Register&&r.tx==4&&r.rx==-1&&r.read==-1);
  else if((fail-1)%3==1)assert(r.failure==Failure::Receive&&r.tx==0&&r.rx==0&&r.read==-1);
  else assert(r.failure==Failure::Read&&r.tx==0&&r.rx==1&&r.read==-1);
 }
 for(int reg:{0x1a,0x19,0x1b,0x1c}){
  Bus b;b.mismatch=reg;assert(!apply(b,r));
  assert(r.failure==Failure::Mismatch&&r.reg==reg&&r.actual==r.expected+1&&r.read==r.actual&&r.tx==0&&r.rx==1);
 }
 Bus ok;assert(apply(ok,r)&&r.failure==Failure::Ok);
 diagnostic::HealthDiagnostics h;
 for(auto s:h.report.stages)assert(s==diagnostic::NotRun);
 h.phase("WHEEL_SENSORS");h.report.stages[diagnostic::RightSensor]=diagnostic::Failed;
 h.report.stages[diagnostic::LeftSensor]=diagnostic::Passed;h.first("RIGHT_SENSOR",42);
 h.phase("WHEEL_SENSORS");h.failed("WHEEL_SENSORS",50);
 assert(h.report.stages[diagnostic::RightSensor]==diagnostic::Failed);
 assert(h.report.stages[diagnostic::LeftSensor]==diagnostic::Passed);
 assert(h.report.stages[diagnostic::RightFoc]==diagnostic::Blocked);
 h.first("IMU_LOOP_STALE",500);assert(!strcmp(h.report.first,"RIGHT_SENSOR")&&h.report.first_ms==42);
 h.beginAttempt();h.phase("MPU6050");h.failed("IMU_CONFIG_FAILED",600);
 assert(h.report.stages[diagnostic::Imu]==diagnostic::Failed&&h.report.imu_failure==2);
 assert(h.report.stages[diagnostic::RightFoc]==diagnostic::Blocked&&h.report.barrier==5);
 Bus bad;bad.fail=2;assert(!apply(bad,r));h.mpuResult(r);const auto first=h.report.mpu;
 Bus good;assert(apply(good,r));h.mpuResult(r);
 assert(h.report.mpu.failure==first.failure&&h.report.mpu.reg==first.reg&&h.report.mpu.rx==0);
 h.failed("WIFI_DISCONNECTED",900);assert(h.report.barrier==5&&!strcmp(h.report.first,"RIGHT_SENSOR"));
 h.beginAttempt();h.phase("RIGHT_FOC");h.failed("WIFI_DISCONNECTED",950);
 assert(h.report.stages[diagnostic::RightFoc]==diagnostic::Running);
 h.report.stages[diagnostic::RightFoc]=diagnostic::Failed;h.failed("RIGHT_FOC",960);
 assert(h.report.stages[diagnostic::RightFoc]==diagnostic::Failed);
 h.beginAttempt();h.failed("RIGHT_DRIVER",1000);assert(h.report.stages[diagnostic::RightFoc]==diagnostic::Blocked);
 h.beginAttempt();h.phase("CALIBRATION_REQUIRED");assert(h.report.barrier==7&&h.report.stages[diagnostic::RightFoc]==diagnostic::Blocked);
 puts("PASS: all MPU failure categories/regs/expected/actual/tx/rx/read; unrun vs blocked vs failed; first fault and configuration failure survive secondary faults/retries");
 printf("HealthReport=%zu Result=%zu bytes\n",sizeof(h.report),sizeof(r));
}
'''.replace('#include <cstdio>','#include <cstdio>\n#include <initializer_list>')
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.cpp').write_text(code)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)

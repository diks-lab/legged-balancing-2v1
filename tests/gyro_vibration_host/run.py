"""Production fixed window and checked initialization; no simulated filter."""
from pathlib import Path
import subprocess, tempfile
code = r'''
#include "GyroVibration.h"
#include "Mpu6050Config.h"
#include "DiagnosticTelemetry.h"
#include <cassert>
#include <cstdio>
#include <cstring>
#include <initializer_list>
struct Bus {
 int step=0, fail=-1, writes=0, reads=0, reg=0, position=0;
 int mismatch=-1;
 void beginTransmission(int address){assert(address==0x68);position=0;}
 void write(uint8_t v){if(position++==0)reg=v;else {assert(reg==0x1a&&v==imu_config::DlpfConfig);++writes;}}
 int endTransmission(bool stop){assert(stop==(writes==1&&step==0));return step++==fail?4:0;}
 int requestFrom(uint8_t address,uint8_t n){assert(address==0x68&&n==1);return step++==fail?0:1;}
 int read(){++reads;if(step++==fail)return -1;return (reg==0x1a?imu_config::DlpfConfig:reg==0x1b?8:0)+(reg==mismatch?1:0);}
};
int main(){
 {Bus b;imu_config::Result r;assert(imu_config::apply(b,r));assert(b.writes==1&&b.reads==4&&r.config==imu_config::DlpfConfig&&r.gyro==8);}
 for(int fail=0;fail<13;++fail){Bus b;b.fail=fail;imu_config::Result r;assert(!imu_config::apply(b,r));}
 for(int reg:{0x1a,0x19,0x1b,0x1c}){Bus b;b.mismatch=reg;imu_config::Result r;assert(!imu_config::apply(b,r));}
 diagnostic::GyroVibration g;
 for(unsigned i=0;i<=100;++i)g.add(i*2000,i*2,i%2?-3:3,true);
 assert(g.latest.count==101&&g.latest.invalid==0&&g.latest.duration_us==200000);
 assert(g.latest.rms==3&&g.latest.peak==3&&g.latest.min_dt_us==2000&&g.latest.max_dt_us==2000);
 for(unsigned i=101;i<=201;++i)g.add(i*2000,i*2,0,true);
 assert(g.latest.rms==0&&g.latest.peak==0&&g.latest.count==101);
 for(unsigned i=202;i<=302;++i)g.add(i*2000,i*2,i==250?NAN:5,i!=240);
 assert(g.latest.invalid==2&&g.latest.count==99&&isnan(g.latest.rms)&&isnan(g.latest.peak));
 g.add(606000,606,4,true);g.add(906001,906,4,true);assert(isnan(g.latest.rms));
 diagnostic::GyroVibration wrap;uint32_t start=UINT32_MAX-100000;
 for(unsigned i=0;i<=100;++i)wrap.add(start+i*2000,i*2,4,true);
 assert(wrap.latest.rms==4&&wrap.latest.duration_us==200000);
 diagnostic::Snapshot s;s.gyro_window=wrap.latest;s.imu_valid=true;s.imu_config=imu_config::DlpfConfig;
 char json[2048];assert(diagnostic::sample(json,sizeof(json),s,"boot",1,1,0,1u<<3,200));
 assert(strstr(json,"\"gyro_window\"")&&strstr(json,",4,4]"));
 assert(diagnostic::sample(json,sizeof(json),s,"boot",1,1,0,1u<<3,701));assert(strstr(json,"null,null]"));
 assert(diagnostic::sample(json,sizeof(json),s,"boot",1,1,0,1u<<0,200));assert(!strstr(json,"gyro_window"));
 s.imu_valid=false;assert(diagnostic::sample(json,sizeof(json),s,"boot",1,1,0,1u<<3,200));assert(strstr(json,"null,null]"));
 printf("PASS: register ACK/readback/range failure; 200 ms RMS/peak, all control samples, invalid/gap/wrap, stale/selected-only JSON; accumulator=%zu result=%zu snapshot=%zu\n",sizeof(g),sizeof(g.latest),sizeof(s));
}
'''
with tempfile.TemporaryDirectory() as td:
    root=Path(td); (root/'test.cpp').write_text(code)
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(root/'test.cpp'),'-o',str(root/'test')],check=True)
    subprocess.run([str(root/'test')],check=True)
source=Path('src/main.cpp').read_text()
init=source[source.index('  mpu6050.begin();'):source.index('  driver1.voltage_power_supply')]
assert init.index('imu_config::apply') < init.index('calibrateMpu6050();')
assert 'disableBothDrivers();publishStartupPhase("IMU_CONFIG_FAILED")' in init
assert source.count('gyro_vibration.add(')==1
assert 'gyro_vibration.add(micros(),millis(),LQR_gyro,diagnostic_imu_finite)' in source

"""Pinned observational patch and actual update: no extra IO/filter change."""
from pathlib import Path
import importlib.util, tempfile, subprocess
spec=importlib.util.spec_from_file_location('imu_patch','scripts/patch_imu_observation.py')
p=importlib.util.module_from_spec(spec);spec.loader.exec_module(p)
installed=Path('.pio/libdeps/esp32dev/MPU6050_tockn/src/MPU6050_tockn.cpp').read_bytes().replace(b'\r\n',b'\n').decode()
if p.sha(installed.encode())==p.PATCHED:
 for before,after in reversed(p.CHANGES):installed=installed.replace(after,before,1)
assert p.sha(installed.encode())==p.ORIGINAL
for newline in ['\n','\r\n']:
 with tempfile.TemporaryDirectory() as td:
  root=Path(td);(root/'src').mkdir();f=root/'src/MPU6050_tockn.cpp'
  f.write_bytes(installed.replace('\n',newline).encode());(root/'library.properties').write_text('version=1.5.2\n')
  assert p.patch(f);out=f.read_bytes();assert not p.patch(f) and out==f.read_bytes()
  assert p.sha(out.replace(b'\r\n',b'\n'))==p.PATCHED
  f.write_bytes(out+b'// unexpected drift\n');before=f.read_bytes()
  try:p.patch(f);assert False
  except RuntimeError:pass
  assert f.read_bytes()==before
  f.write_bytes(installed.replace('\n',newline).encode());(root/'library.properties').write_text('version=9\n')
  try:p.patch(f);assert False
  except RuntimeError:pass
p.verify_link('00000001 <MPU6050::update()>:\n 00001: call8 123 <observeImuTransaction>\n00000010 <MPU6050::calcGyroOffsets(bool, unsigned short, unsigned short)>:\n 00010: call8 123 <observeImuCalibration>\n00000020 <next>:\n')
try:p.verify_link('00000001 <MPU6050::update()>:\n 00001: nop\n');assert False
except RuntimeError:pass
oldcal=installed[installed.index('void MPU6050::calcGyroOffsets('):installed.index('void MPU6050::update(){')]
original=installed[installed.index('void MPU6050::update(){'):]
patched=p.transform(installed);newcal=patched[patched.index('void MPU6050::calcGyroOffsets('):patched.index('void MPU6050::update(){')];patched=patched[patched.index('void MPU6050::update(){'):]
# update is the final method in this exact source. All original calculations remain verbatim.
assert original.count('wire->read()')==patched.count('wire->read()')==14
pre=r'''
#include <cmath>
#include <cassert>
#include <cstdint>
#include <cstdio>
#define PI 3.14159265358979323846
#define MPU6050_ADDR 0x68
unsigned long millis(){return 1234;}
void delay(unsigned){}
struct Console {template<class T>void print(T){} template<class T>void println(T){}void println(){}}Serial;
unsigned txSeen=999,rxSeen=999,events=0,calEvents=0;
extern "C" void observeImuCalibration(unsigned char,unsigned char){++calEvents;}
extern "C" void observeImuTransaction(unsigned char tx,unsigned char rx){txSeen=tx;rxSeen=rx;++events;}
struct Wire {
 unsigned tx=0,rx=14,ends=0,requests=0,reads=0;
 void beginTransmission(int){} void write(int){}
 unsigned endTransmission(bool){++ends;return tx;}
 unsigned requestFrom(int,int n){assert(n==14||n==6);++requests;return rx;}
 int read(){return (++reads)%2?0x10:0x20;}
};
struct MPU6050 {
 Wire*wire;int16_t rawAccX=0,rawAccY=0,rawAccZ=0,rawTemp=0,rawGyroX=0,rawGyroY=0,rawGyroZ=0;
 float temp=0,accX=0,accY=0,accZ=0,gyroX=0,gyroY=0,gyroZ=0;
 float angleAccX=0,angleAccY=0,angleGyroX=0,angleGyroY=0,angleGyroZ=0,angleX=0,angleY=0,angleZ=0;
 float gyroXoffset=0,gyroYoffset=0,gyroZoffset=0,interval=0,preInterval=0,accCoef=.02f,gyroCoef=.98f;
 void update();void calcGyroOffsets(bool,uint16_t,uint16_t);
};
'''
code=pre+'namespace original {\nusing ::Wire;\n'+pre[pre.index('struct MPU6050 {'):]+oldcal+original+'}\n'+newcal+patched+r'''
int main(){for(unsigned tx:{0u,2u})for(unsigned rx:{14u,13u,0u}){
 Wire a,b;a.tx=b.tx=tx;a.rx=b.rx=rx;original::MPU6050 before;MPU6050 after;before.wire=&a;after.wire=&b;
 const auto n=events;before.update();after.update();assert(events==n+1&&txSeen==tx&&rxSeen==rx);
 assert(a.ends==b.ends&&a.requests==b.requests&&a.reads==b.reads&&b.reads==14);
 assert(before.angleY==after.angleY&&before.gyroY==after.gyroY&&before.preInterval==after.preInterval);
 }
 for(unsigned tx:{0u,2u})for(unsigned rx:{6u,5u,0u}){
 Wire a,b;a.tx=b.tx=tx;a.rx=b.rx=rx;original::MPU6050 before;MPU6050 after;before.wire=&a;after.wire=&b;
 auto count=calEvents;before.calcGyroOffsets(false,0,0);after.calcGyroOffsets(false,0,0);
 assert(calEvents==count+3000&&a.requests==3000&&b.requests==3000&&a.reads==18000&&b.reads==18000);
 assert(before.gyroYoffset==after.gyroYoffset&&before.gyroXoffset==after.gyroXoffset);
 }puts("PASS: IMU exact-source LF/CRLF/idempotence/drift/version/link guard; real update normal/short/NACK, identical read count and filter values");}
'''
code=code.replace('#include <cmath>','#include <cmath>\n#include <initializer_list>',1)
with tempfile.TemporaryDirectory() as td:
 f=Path(td)/'test.cpp';f.write_text(code);exe=Path(td)/'test'
 subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',str(f),'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)

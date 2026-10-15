"""Run production control/state-machine functions with real SimpleFOC PID/Sensor.

Only hardware and log sinks are doubled. The main.cpp declarations and selected
functions are compiled verbatim so control equations and transition conditions
are not reimplemented in the test.
"""
from pathlib import Path
import os
import subprocess
import tempfile

source = Path('src/main.cpp').read_text()
def function(name):
    import re
    match = re.search(r'^(?:void|bool|const char\*|float)\s+' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start, pos, depth = match.start(), match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos] + '\n'

prefix = source[source.index('namespace {'):source.index('}  // namespace\n\nvoid setup()')]
prefix = prefix.replace('BLDCMotor motor', 'DriveTestMotor motor').replace('BLDCDriver3PWM driver', 'DriveTestDriver driver')
prelude = r'''
#include "Arduino.h"
#include "CheckedWheelSensor.h"
#include "LqrMotorVoltage.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "common/pid.h"
#include <atomic>
#include <algorithm>
#include <cassert>
#include <iostream>
using std::min; using std::max; using std::abs;
template<class T, class L, class H> T constrain(T x,L lo,H hi) {
  return x < lo ? T(lo) : (x > hi ? T(hi) : x);
}
struct DriveSerial : FakeSerial {
  void println(const char* s="") { output += std::string(s)+"\n"; }
  template<class T> void print(T) {}
} driveSerial;
#define Serial driveSerial
struct WebServer { explicit WebServer(int) {} };
struct DriveTestMotor {
  explicit DriveTestMotor(int) {}
  float shaft_angle=0,shaft_velocity=0,target=0,current_sp=0;
  struct {float q=0,d=0;} voltage;
  void move(float v){target=current_sp=v;}
};
struct DriveTestDriver {
  int en; DriveTestDriver(int,int,int,int e):en(e){}
  void disable(){digitalWrite(en,LOW);}
  void setPwm(float,float,float){}
};
struct MPU6050 {
  explicit MPU6050(TwoWire&){}
  float getAngleY(){return 0;} float getGyroY(){return 0;}
};
unsigned long _micros(){return micros();}
'''
names = [
    'updateTestLqrOutput','lqrTestModeLogName','lqrTestModeDisplayName','lqrSpeedSourceName',
    'requestLqrDrive','requestLqrSoftStop','startPendingLqrDrive','updateLqrDiagnostic',
    'stopLqrDriveOnly','terminateLqrDriveSession','stopBalanceTest','resetLqrTestState',
    'startBalanceTest','cycleLqrTestMode','updateBalanceWait','cancelBalanceWait',
    'confirmActualFall','detectDriveSafetyFault','wheelSensorsOK','stopOnWheelFault',
    'setBothTargetsToZero','stopBothMotorsAtZero','disableBothDrivers',
    'clearVelocityTrace','clearLoopTimingTrace','clearLqrControlTrace',
    'updateLqrControlTrace','captureLqrDriveSignTrace','captureLqrStopTrace',
    'resetLqrDriveSignTrace','resetLqrStopTrace','startLqrStopTrace','setStatusLed','printLqrStatus',
]
# No control behavior is stubbed: these functions only print or store logs.
stubs = r'''
void printLqrDriveSnapshot(const char*){}
void printLqrDriveSignTrace(const char*){}
void printLqrStopTrace(const char*){}
void printAndResetLqrRunTime(){}
void printVelocityTrace(){}
void printLoopTimingTrace(){}
void printLqrControlTrace(){}
void setPs4DriveState(Ps4DriveState state){ps4_drive_state=state;}
const char* ps4DriveStateName(Ps4DriveState){return "TEST";}
void beginPs4DriveStopping(uint32_t){ps4_drive_state=Ps4DriveState::Stopping;}
'''
checks = r'''
void near(float x,float y){assert(std::fabs(x-y)<1e-5f);}
void setupCase(LqrTestMode mode=LqrTestMode::Full){
  nowMs+=1000;
  resetLqrTestState(); cancelBalanceWait();
  lqr_test_mode=mode;
  test_system_ready=true; emergency_stop_active=false;
  lqr_diagnostic_armed=true; lqr_direction_test_armed=true;
  LQR_angle=LQR_gyro=LQR_speed=LQR_distance=0;
  angle_zeropoint=distance_zeropoint=0;
  motor1.shaft_velocity=motor2.shaft_velocity=0;
  ps4_drive_state=Ps4DriveState::Neutral;
  driver1_initialized=driver2_initialized=true;
  digitalWrite(27,HIGH);digitalWrite(32,HIGH);
  Serial.output.clear();
}
void step(float speed=0){
  delay(10);
  LQR_speed=diag_LQR_speed_10ms=speed;
  motor1.shaft_velocity=motor2.shaft_velocity=speed;
  control_dt_ms=10;
  updateLqrDiagnostic();
}
void start(float sign){
  assert(requestLqrDrive(sign*kLqrDriveTestSpeed,sign>0?'>':'<'));
  assert(!lqr_auto_drive_active && lqr_target_speed==0);
  for(int i=0;i<100 && !lqr_auto_drive_active;++i) step();
  assert(lqr_auto_drive_active && !lqr_drive_start_pending);
  near(lqr_target_speed,sign*kLqrDriveTestSpeed);
  assert(sign*lqr_drive_angle_bias>0); // forward/backward body-tilt target
  near(angle_control,-lqr_drive_angle_bias); // create lean, not a voltage step
  assert(speed_direct_control==0); // preserve the old tilt-only drive profile
}
int main(){
  // AUTO START still requires an explicit t-equivalent and starts at rest.
  setupCase(); lqr_direction_test_armed=false; lqr_diagnostic_armed=false;
  assert(!requestLqrDrive(.3f,'>'));
  assert(startBalanceTest());
  for(int i=0;i<500 && !lqr_direction_test_armed;++i){
    step(); updateBalanceWait();
  }
  assert(lqr_direction_test_armed && !lqr_auto_drive_active);
  assert(lqr_target_speed==0 && !lqr_drive_start_pending);
  // Attitude polarity is preserved in every selectable mode.
  for(auto mode:{LqrTestMode::Full,LqrTestMode::AttitudeSpeed,
                 LqrTestMode::AttitudeDistance,LqrTestMode::AttitudeOnly}){
    setupCase(mode);LQR_angle=.25f;LQR_gyro=.5f;step();
    near(test_LQR_u,.29f);near(LQR_u,test_LQR_u);
    assert(lqrMotorVoltage(test_LQR_u,1,1)>0);
  }
  setupCase();step(.2f);assert(speed_control<0 && test_LQR_u<0);
  setupCase();LQR_distance=.1f;step();near(distance_control,-.04f);assert(test_LQR_u<0);
  setupCase();LQR_distance=-.1f;step();assert(test_LQR_u>0);
  // Re-selecting AttitudeOnly exactly recovers c4ba29b, irrespective of x/v.
  setupCase(LqrTestMode::AttitudeOnly);LQR_angle=.25f;LQR_gyro=.5f;LQR_distance=8;
  step(12);near(test_LQR_u,.29f);assert(distance_control==0 && speed_control==0);
  assert(!requestLqrDrive(.3f,'>'));
  lqr_direction_test_armed=false;
  assert(cycleLqrTestMode() && lqr_test_mode==LqrTestMode::Full);
  assert(cycleLqrTestMode() && cycleLqrTestMode() && cycleLqrTestMode());
  assert(lqr_test_mode==LqrTestMode::AttitudeOnly);
  // Existing drive-start conditions reject moving/tilting starts and time out.
  setupCase();LQR_gyro=3;assert(requestLqrDrive(.3f,'>'));
  for(int i=0;i<510;++i)step();
  assert(!lqr_auto_drive_active && !lqr_drive_start_pending);
  assert(drive_recovery_state==DriveRecoveryState::AttitudeOnly);
  // Pending request cancellation keeps stationary balance armed.
  setupCase();assert(requestLqrDrive(.3f,'>'));requestLqrSoftStop();
  assert(!lqr_drive_start_pending && !lqr_auto_drive_active && lqr_direction_test_armed);
  for(float sign:{1.0f,-1.0f}){
    setupCase();start(sign);
    LQR_distance=sign*.04f;step(sign*.2f);
    near(lqr_target_speed,sign*.15f); // original 0.03 -> 0.05 rad decel window
    LQR_distance=sign*.051f;step(sign*.2f);
    assert(!lqr_auto_drive_active && lqr_soft_stop_active && lqr_target_speed==0);
    assert(sign*speed_direct_control<0 && sign*test_LQR_u<0);
    for(int i=0;i<200 && lqr_soft_stop_active;++i)step();
    assert(!lqr_soft_stop_active);near(distance_zeropoint,LQR_distance);
    setupCase();start(sign);requestLqrSoftStop();step(sign*.2f);
    assert(lqr_soft_stop_active && sign*speed_direct_control<0);
  }
  // Preserve both zero-crossing and sustained-low-speed stop completions.
  setupCase();start(1);step(.2f);requestLqrSoftStop();
  for(int i=0;i<50 && lqr_soft_stop_active;++i)step(-.2f);
  assert(!lqr_soft_stop_active);
  // Original travel and wrong-way limits still terminate the drive request.
  setupCase();start(1);LQR_distance=kLqrDriveTravelLimit;step(.2f);
  assert(!lqr_auto_drive_active && lqr_target_speed==0);
  setupCase();start(1);LQR_distance=-.04f;
  for(int i=0;i<50 && lqr_auto_drive_active;++i)step(-.2f);
  assert(!lqr_auto_drive_active && drive_recovery_state==DriveRecoveryState::AttitudeOnly);
  setupCase();start(1);assert(stopBalanceTest());
  assert(!lqr_direction_test_armed && !lqr_diagnostic_armed);
  assert(motor1.current_sp==0 && motor2.current_sp==0);
  // Wait brake opposes forward motion, but comparison mode has no brake.
  setupCase();balance_wait_active=true;balance_prepare_active=true;
  balance_prepare_start_ms=millis();balance_brake_enabled=true;
  step(.2f);updateBalanceWait();assert(balance_brake_right<0 && balance_brake_left<0);
  lqr_test_mode=LqrTestMode::AttitudeOnly;step(.2f);updateBalanceWait();
  assert(balance_brake_right==0 && balance_brake_left==0);
  // e and the existing drive-only fault path keep attitude control alive.
  setupCase();start(1);stopLqrDriveOnly("MANUAL_DRIVE_STOP",true);
  LQR_angle=.2f;LQR_distance=10;step(1);
  assert(!lqr_auto_drive_active && !lqr_soft_stop_active && !lqr_drive_start_pending);
  assert(lqr_direction_test_armed && pins[27] && pins[32]);near(test_LQR_u,.2f);
  assert(distance_control==0 && speed_direct_control==0);
  // Existing drive-envelope and fall conditions are still exercised.
  setupCase();start(1);LQR_angle=3;assert(detectDriveSafetyFault(0,0));
  setupCase();LQR_angle=21;bool fell=false;
  for(int i=0;i<12;++i){delay(10);fell=confirmActualFall()||fell;}
  assert(fell);
  // A fault on either checked sensor still latches a full stop and blocks t.
  sensor1.begin(19,18);sensor2.begin(23,5);
  setupCase();sensor2.fault=true;gateWheelOutputsOff();assert(stopOnWheelFault());
  assert(!pins[27] && !pins[32] && !test_system_ready && !lqr_direction_test_armed);
  assert(!startBalanceTest());
  // Trace sum and status report selected contributions, not unused speed PID.
  setupCase();start(1);near(speed_direct_control,0);
  updateLqrControlTrace(0,0,true);
  auto& sample=lqr_control_trace[0];
  near(sample.angle_control+sample.gyro_control+sample.distance_control+sample.speed_control,sample.lqr_u);
  Serial.output.clear();printLqrStatus();
  assert(Serial.output.find("speed_control=0.000000")!=std::string::npos);
  std::cout<<"PASS: production start/gates, terms/signs, both drive directions, deceleration, cancel/recovery, faults/fall, mode return, logs\n";
}
'''
foc = Path(os.environ.get('SIMPLEFOC_SRC','.pio/libdeps/esp32dev/Arduino-FOC/src'))
with tempfile.TemporaryDirectory(prefix='drive-control-') as directory:
    cpp=Path(directory)/'drive.cpp';binary=Path(directory)/'drive'
    bodies = [function(n) for n in names]
    prototypes = '\n'.join(body[:body.index('{')]+';' for body in bodies)
    cpp.write_text(prelude+prefix+prototypes+stubs+'\n'.join(bodies)+'}\n'+checks)
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',
                    '-Iinclude','-Itests/wheel_host','-I'+str(foc),str(cpp),
                    str(foc/'common/pid.cpp'),str(foc/'common/base_classes/Sensor.cpp'),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

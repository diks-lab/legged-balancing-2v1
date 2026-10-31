"""Compile the production status function with recording output and fixed state."""
from pathlib import Path
import subprocess
import tempfile

source = Path('src/main.cpp').read_text()
start = source.index('void printLqrStatus() {')
end = source.index('\nvoid printDistanceDiagnostic()', start)
status = source[start:end]
drive_start = source.index('  if (lqr_direction_test_armed) {\n    const float motor1_output_sign')
drive_end = source.index('\n  applyPs4Steering(', drive_start)
# Balance output mapping is checked here; production steering has separate
# gate/headroom tests in tests/drive_host/run.py.
drive = 'void driveOutputs(float& right_output, float& left_output) {\n' + source[drive_start:drive_end] + '\n}\n'
harness = r'''
#include "Arduino.h"
#include "LqrMotorVoltage.h"
#include <cassert>
struct Motor { float shaft_angle=8, shaft_velocity=12; } motor1, motor2;
struct Imu { float getAngleY(){return 99;} float getGyroY(){return 88;} } mpu6050;
enum class LqrTestMode { AttitudeOnly, Full, AttitudeSpeed, AttitudeDistance };
enum class LqrSpeedSource { SimpleFoc, Diagnostic10ms };
LqrTestMode lqr_test_mode=LqrTestMode::AttitudeOnly;
LqrSpeedSource lqr_speed_source=LqrSpeedSource::SimpleFoc;
float m1_direction=.5f,m2_direction=.5f,diag_LQR_speed_10ms=12;
float diag_right_velocity_10ms=12,diag_left_velocity_10ms=12;
float angle_control=.2f,gyro_control=.05f,distance_control=7,speed_control=8;
float speed_direct_control=0, lqr_speed_for_control=12, lqr_drive_angle_bias=0, lqr_ps4_target_tilt_deg=0;
float LQR_u=.25f,test_LQR_u=.25f,angle_zeropoint=1,distance_zeropoint=2,control_dt_ms=1;
bool lqr_zero_set=true,lqr_diagnostic_armed=true,lqr_direction_test_armed=true;
constexpr float kLqrTestVoltageLimit=1;
const char* lqrSpeedSourceName(){return "SIMPLEFOC";}
const char* lqrTestModeLogName(){return "ATTITUDE_ONLY";}
'''
checks = r'''
int main() {
  float right=0, left=0;
  test_LQR_u=5.93f;
  driveOutputs(right,left);
  assert(right==1.0f && left==1.0f);
  test_LQR_u=-9.87f;
  driveOutputs(right,left);
  assert(right==-1.0f && left==-1.0f);
  test_LQR_u=.25f;
  driveOutputs(right,left);
  assert(right==.25f && left==.25f);
  printLqrStatus();
  assert(Serial.output.find("distance_control=0.000000 speed_control=0.000000")!=std::string::npos);
  assert(Serial.output.find("test_LQR_u=0.250000")!=std::string::npos);
  assert(Serial.output.find("LQR_u=0.250000 diagnostic=")!=std::string::npos);
  assert(Serial.output.find("right_output[V]=0.250000 left_output[V]=0.250000")!=std::string::npos);
  assert(LQR_u==.25f && test_LQR_u==.25f); // display has no controller side effects
  Serial.output.clear();
  LQR_u=test_LQR_u=-9.87f;
  printLqrStatus();
  assert(Serial.output.find("right_output[V]=-1.000000 left_output[V]=-1.000000")!=std::string::npos);
  assert(test_LQR_u==-9.87f);
  Serial.output.clear();
  lqr_test_mode=LqrTestMode::Full;
  distance_control=.1f;speed_control=99;speed_direct_control=-.05f;
  LQR_u=test_LQR_u=.3f;
  printLqrStatus();
  assert(Serial.output.find("distance_control=0.100000 speed_control=-0.050000")!=std::string::npos);
  assert(Serial.output.find("speed_pid_raw=99.000000 speed_direct_term=-0.050000")!=std::string::npos);
  assert(Serial.output.find("LQR_u=0.300000 diagnostic=")!=std::string::npos);
  std::puts("PASS: production status zero terms, cached control sum, polarity, no mutation");
}
'''
with tempfile.TemporaryDirectory(prefix='attitude-status-') as directory:
    cpp = Path(directory)/'status.cpp'
    cpp.write_text(harness + drive + status + checks)
    binary=Path(directory)/'status'
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude','-Itests/wheel_host',str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)

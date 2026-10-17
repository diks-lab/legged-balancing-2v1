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
#include <vector>
#include <cstring>
using std::min; using std::max; using std::abs;
template<class T, class L, class H> T constrain(T x,L lo,H hi) {
  return x < lo ? T(lo) : (x > hi ? T(hi) : x);
}
std::function<void()> onSerialOutput;
struct DriveSerial : FakeSerial {
  void println(const char* s="") {
    if(onSerialOutput) onSerialOutput();
    output += std::string(s)+"\n";
  }
  template<class... A> void printf(const char* s,A... args) {
    if(onSerialOutput) onSerialOutput();
    FakeSerial::printf(s,args...);
  }
  template<class T> void print(T) {}
} driveSerial;
#define Serial driveSerial
using String=std::string;
constexpr int WL_CONNECTED=1, CONTENT_LENGTH_UNKNOWN=-1;
struct { int status(){return WL_CONNECTED;}
  struct IP { String toString(){return "127.0.0.1";} };
  IP localIP(){return {};}
} WiFi;
struct File {
  bool exists=false, directory=false;
  explicit operator bool() const {return exists;}
  bool isDirectory() const {return directory;}
};
struct { File file{}; int opens=0;
  File open(const char*,const char*) {++opens;return file;}
} LittleFS;
struct WebServer {
  explicit WebServer(int) {}
  int status=0;std::string body;
  struct Client {bool connected(){return true;}};
  Client client(){return {};}
  void sendHeader(const char*,const String&){}
  void setContentLength(int){}
  void send(int code,const char*,const char* data){status=code;body=data;}
  void sendContent(const char* data){body+=data;}
  void streamFile(File,const char*) {status=200;body="asset";}
};
struct DriveTestMotor {
  explicit DriveTestMotor(int) {}
  float shaft_angle=0,shaft_velocity=0,target=0,current_sp=0;
  int enabled=0,motor_status=0;
  struct {float q=0,d=0;} voltage;
  void move(float v){target=current_sp=v;}
};
struct DriveTestDriver {
  int en; DriveTestDriver(int,int,int,int e):en(e){}
  void disable(){digitalWrite(en,LOW);}
  void enable(){digitalWrite(en,HIGH);}
  void setPwm(float,float,float){}
};
struct MPU6050 {
  explicit MPU6050(TwoWire&){}
  float getAngleY(){return 0;} float getGyroY(){return 0;}
};
unsigned long _micros(){return micros();}
'''
prelude += r'''
constexpr int pdTRUE=1;
struct HostQueue { std::vector<unsigned char> bytes; size_t item; bool full=false; };
using QueueHandle_t=HostQueue*;
QueueHandle_t xQueueCreate(int,size_t item){return new HostQueue{{},item,false};}
int xQueueOverwrite(QueueHandle_t q,const void* data){
  q->bytes.assign((const unsigned char*)data,(const unsigned char*)data+q->item);q->full=true;return 1;
}
int xQueueSend(QueueHandle_t q,const void* data,int){if(q->full)return 0;return xQueueOverwrite(q,data);}
int xQueuePeek(QueueHandle_t q,void* data,int){if(!q->full)return 0;memcpy(data,q->bytes.data(),q->item);return 1;}
int xQueueReceive(QueueHandle_t q,void* data,int t){int r=xQueuePeek(q,data,t);q->full=false;return r;}
#define pdMS_TO_TICKS(x) (x)
std::function<void()> onHttpYield;
void vTaskDelay(int ms){delay(ms);if(onHttpYield)onHttpYield();}
'''
bridge_start = source.index('struct HttpStatus {')
bridge_end = source.index('void captureHttpStatus();', bridge_start)
prefix += source[bridge_start:bridge_end]
names = [
    'updateTestLqrOutput','lqrTestModeLogName','lqrTestModeDisplayName','lqrSpeedSourceName',
    'requestLqrDrive','requestLqrSoftStop','startPendingLqrDrive','updateLqrDiagnostic',
    'stopLqrDriveOnly','terminateLqrDriveSession','stopBalanceTest','resetLqrTestState',
    'startBalanceTest','cycleLqrTestMode','updateBalanceWait','cancelBalanceWait',
    'confirmActualFall','detectDriveSafetyFault','wheelSensorsOK','stopOnWheelFault',
    'setBothTargetsToZero','stopBothMotorsAtZero','disableBothDrivers',
    'clearVelocityTrace','clearLoopTimingTrace','clearLqrControlTrace',
    'captureFocOutputSnapshot','updateLqrControlTrace','captureLqrDriveSignTrace','captureLqrStopTrace',
    'resetLqrDriveSignTrace','resetLqrStopTrace','startLqrStopTrace','setStatusLed','printLqrStatus',
    'stopForActualFall','traceDownloadBlocked','beginCsvResponse','sendTraceBusyResponse',
    'handleVelocityTraceDownload','handleLoopTimingTraceDownload','handleLqrControlTraceDownload',
    'captureHttpStatus','serviceHttpRequests','handleWifiStatusPage','handleStartBalanceTest','handleStopBalanceTest','enqueueHttpCommand',
    'serveWebFile','cycleLqrSpeedSource','resetPs4MotionInput','executeOperatorCommand',
    'applyPs4Steering','ps4ButtonsReleased','handlePs4Connection','processPs4Input',
    'ps4InputStatesEqual','ps4DriveSpeedRequestFromLeftStickY',
    'updateWheelVelocityDiagnostic',
]
# No control behavior is stubbed: these functions only print or store logs.
stubs = r'''
void formatHttpResources(char* text,size_t capacity){snprintf(text,capacity,"\nHTTP resources: host stub\n");}

float legCommand=-1;int legWrites=0;
void moveBothLegsToPercent(float x){legCommand=x;++legWrites;}
void printPs4InputDiagnostic(const Ps4InputState&){}
void printPs4DriveStopDiagnostic(const Ps4InputState&,bool,bool,uint32_t){}
void printPs4DriveStopSummary(uint32_t){}
void printLqrDriveSnapshot(const char*){}
void printLqrDriveSignTrace(const char*){}
void printLqrStopTrace(const char*){}
void printAndResetLqrRunTime(){}
void disableLegServoTorqueForTestEnd(const char*){}
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
  motor1.enabled=motor2.enabled=1;motor1.motor_status=motor2.motor_status=4;
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
  http_status_queue=xQueueCreate(1,sizeof(HttpStatus));
  http_command_queue=xQueueCreate(1,sizeof(char));
  onHttpYield=[]{serviceHttpRequests();};
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
  setupCase();step(.2f);assert(speed_control<0 && speed_direct_control>0 && test_LQR_u>0);
  setupCase();LQR_distance=.1f;step();near(distance_control,.01f);assert(test_LQR_u>0);
  setupCase();LQR_distance=-.1f;step();assert(test_LQR_u<0);
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
    assert(sign*speed_direct_control>0 && sign*test_LQR_u>0);
    for(int i=0;i<200 && lqr_soft_stop_active;++i)step();
    assert(!lqr_soft_stop_active);near(distance_zeropoint,LQR_distance);
    setupCase();start(sign);requestLqrSoftStop();step(sign*.2f);
    assert(lqr_soft_stop_active && sign*speed_direct_control>0);
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

  // Normal trace observes last FOC values before this loop's move stages
  // a different command. Merely recording must not alter cached motor state.
  setupCase();clearLqrControlTrace();
  motor1.voltage.q=.125f;motor2.voltage.q=-.25f;
  motor1.current_sp=.125f;motor2.current_sp=-.25f;
  captureFocOutputSnapshot();
  motor1.current_sp=motor1.voltage.q=0; // control code can clear live fields
  updateLqrControlTrace(.75f,-.5f,true);
  const auto observed=lqr_control_trace[0];
  near(observed.right_output,.75f);near(observed.left_output,-.5f);
  near(observed.right_voltage_q,.125f);near(observed.left_voltage_q,-.25f);
  near(observed.right_current_sp,.125f);near(observed.left_current_sp,-.25f);
  assert(observed.right_motor_enabled && observed.left_motor_enabled);
  assert(observed.right_driver_en && observed.left_driver_en && !observed.after_disable);
  motor1.move(.75f);near(motor1.current_sp,.75f);near(motor1.voltage.q,0);
  near(lqr_control_trace[0].right_current_sp,.125f); // immutable before-move snapshot
  lqr_direction_test_armed=false;
  handleLqrControlTraceDownload();
  assert(wifi_log_server.body.find("right_voltage_q_V,left_voltage_q_V,right_current_sp_V")!=String::npos);
  assert(wifi_log_server.body.find("0.750000,-0.500000,0.125000,-0.250000,0.125000,-0.250000,1,1,1,1,BEFORE_MOVE")!=String::npos);
  // CSV has 24 columns (existing 14 plus 10 appended), with no truncated row.
  const auto newline=wifi_log_server.body.find('\n');
  assert(std::count(wifi_log_server.body.begin(),wifi_log_server.body.begin()+newline,',')==23);
  assert(std::count(wifi_log_server.body.begin()+newline+1,wifi_log_server.body.end(),',')==23);
  LQR_angle=5.93f;angle_zeropoint=1;captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("Cached AngleY: 5.930000 deg")!=String::npos);
  assert(wifi_log_server.body.find("Reference: 1.000000 deg")!=String::npos);
  assert(wifi_log_server.body.find("Delta: 4.930000 deg")!=String::npos);
  assert(wifi_log_server.body.find("driver_init=1")!=String::npos);
  assert(wifi_log_server.body.find("motor_status=4 motor_enabled=1 EN=1")!=String::npos);
  // No USB: gating must already be complete whenever any stop-path Serial
  // output occurs, including the startup diagnostic before the trace dumps.
  setupCase();
  velocity_trace_count=velocity_trace_write_index=1;velocity_trace[0].elapsed_ms=42;
  loop_timing_count=loop_timing_write_index=1;loop_timing_trace[0].elapsed_ms=43;
  lqr_control_trace_count=lqr_control_trace_write_index=1;
  lqr_control_trace[0].elapsed_ms=44;
  handleVelocityTraceDownload();assert(wifi_log_server.status==409);
  motor1.current_sp=motor2.current_sp=.8f;
  lqr_startup_diagnostic_active=true;
  int serialChecks=0;
  onSerialOutput=[&]{
    ++serialChecks;
    assert(!pins[27] && !pins[32] && emergency_stop_active);
    assert(motor1.current_sp==0 && motor2.current_sp==0);
    assert(!lqr_direction_test_armed);
  };
  float right=.8f,left=.8f;
  stopForActualFall(right,left);
  onSerialOutput={};
  assert(serialChecks>0 && right==0 && left==0);
  assert(std::string(last_stop_reason)=="TILT_STOP" && !traceDownloadBlocked(1));
  assert(velocity_trace_count==1 && loop_timing_count==1 && lqr_control_trace_count==2);
  assert(lqr_control_trace[1].right_output==0 && lqr_control_trace[1].left_output==0);
  assert(lqr_control_trace[1].after_disable);
  assert(lqr_control_trace[1].right_voltage_q==0 && lqr_control_trace[1].right_current_sp==0);
  assert(lqr_control_trace[1].right_motor_enabled && !lqr_control_trace[1].right_driver_en);
  for(int i=0;i<20;++i){delay(20);updateWheelVelocityDiagnostic();updateLqrControlTrace(0,0,false);}
  assert(velocity_trace_count==1 && loop_timing_count==1 && lqr_control_trace_count==2);
  captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.status==200);
  assert(wifi_log_server.body.find("TILT_STOP")!=String::npos);
  assert(wifi_log_server.body.find("Velocity=1")!=String::npos);
  handleVelocityTraceDownload();assert(wifi_log_server.status==200);
  const String csv=wifi_log_server.body;
  assert(std::count(csv.begin(),csv.end(),'\n')==2);
  assert(csv.find("42,")!=String::npos);
  handleVelocityTraceDownload();assert(csv==wifi_log_server.body);
  handleLoopTimingTraceDownload();assert(wifi_log_server.status==200);
  assert(std::count(wifi_log_server.body.begin(),wifi_log_server.body.end(),'\n')==2);
  handleLqrControlTraceDownload();assert(wifi_log_server.status==200);
  assert(std::count(wifi_log_server.body.begin(),wifi_log_server.body.end(),'\n')==3);
  assert(wifi_log_server.body.find("AFTER_DISABLE")!=String::npos);
  handleStopBalanceTest();serviceHttpRequests(); // a redundant Web STOP must preserve the fall
  assert(std::string(last_stop_reason)=="TILT_STOP" && velocity_trace_count==1);
  handleStartBalanceTest();serviceHttpRequests(); // rejected while disabled: no erase
  assert(wifi_log_server.status==202 && lqr_control_trace_count==2);
  assert(std::string(last_stop_reason)=="TILT_STOP");
  emergency_stop_active=false; // same zero-output re-enable state as u
  assert(startBalanceTest());
  assert(velocity_trace_count==0 && loop_timing_count==0 && lqr_control_trace_count==0);
  assert(std::string(last_stop_reason)=="NONE" && balance_wait_active);
  assert(stopBalanceTest());assert(std::string(last_stop_reason)=="MANUAL_STOP");

  // A fault on either checked sensor still latches a full stop and blocks t.
  sensor1.begin(19,18);sensor2.begin(23,5);
  setupCase();sensor2.fault=true;gateWheelOutputsOff();assert(stopOnWheelFault());
  assert(!pins[27] && !pins[32] && !test_system_ready && !lqr_direction_test_armed);
  assert(!startBalanceTest());
  captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("System initialized: no")!=String::npos);
  assert(wifi_log_server.body.find("fault=1")!=String::npos);

  // Missing/unmounted Web assets do not mutate motor or balance state.
  setupCase();http_filesystem_ready=false;
  serveWebFile("/index.html","text/html");
  assert(wifi_log_server.status==503 && LittleFS.opens==0);
  assert(lqr_direction_test_armed && !emergency_stop_active && pins[27] && pins[32]);
  http_filesystem_ready=true;LittleFS.file={false,false};
  serveWebFile("/index.html","text/html");assert(wifi_log_server.status==404);
  assert(lqr_direction_test_armed && lqr_target_speed==0);
  LittleFS.file={true,false};serveWebFile("/index.html","text/html");assert(wifi_log_server.status==200);
  for(auto mode:{LqrTestMode::Full,LqrTestMode::AttitudeSpeed}){
    for(float sign:{-1.0f,1.0f}){
      setupCase(mode);start(sign);requestLqrSoftStop();step(sign*.2f);
      assert(sign*speed_direct_control>0 && lqr_direction_test_armed);
      near(speed_direct_control,-speed_control);
    }
  }
  // Mode selection is available on PS4, preserving the existing boot mode.
  setupCase(LqrTestMode::AttitudeOnly);stopBalanceTest();ps4_was_connected=false;
  Ps4InputState modePacket{};handlePs4Connection(true);
  processPs4Input(modePacket,nowMs+=50);modePacket.triangle=true;
  processPs4Input(modePacket,nowMs+=50);assert(lqr_test_mode==LqrTestMode::Full);
  processPs4Input(modePacket,nowMs+=50);assert(lqr_test_mode==LqrTestMode::Full);
  // PS4 startup: connection and held buttons/sticks never start balancing.
  setupCase();stopBalanceTest();ps4_was_connected=false;
  Ps4InputState packet{};
  handlePs4Connection(true);packet.options=true;packet.left_stick_y=-127;
  for(int i=0;i<5;++i)processPs4Input(packet,nowMs+=50);
  assert(!balance_wait_active && !lqr_direction_test_armed && !lqr_drive_start_pending);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.options=true;processPs4Input(packet,nowMs+=50);
  assert(balance_wait_active && !lqr_drive_start_pending);
  const auto prepare=balance_prepare_start_ms;
  for(int i=0;i<5;++i)processPs4Input(packet,nowMs+=50);
  assert(balance_prepare_start_ms==prepare); // held Options did not restart prepare
  // Stick held during prepare cannot become a request at AUTO START.
  packet.options=false;packet.left_stick_y=-127;
  processPs4Input(packet,nowMs+=50);
  lqr_direction_test_armed=true;balance_wait_active=false;
  processPs4Input(packet,nowMs+=50);assert(!lqr_drive_start_pending);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.left_stick_y=-127;processPs4Input(packet,nowMs+=50);
  assert(lqr_drive_start_pending && ps4_drive_state==Ps4DriveState::DriveRequested);
  // Neutral cancels pending requests, preserving balance.
  packet={};processPs4Input(packet,nowMs+=50);
  assert(!lqr_drive_start_pending && lqr_direction_test_armed);
  processPs4Input(packet,nowMs+=50);
  // Neutral/deadzone and held buttons cannot initiate a drive.
  setupCase();handlePs4Connection(false);handlePs4Connection(true);
  packet={};packet.left_stick_y=10;packet.right_stick_x=-10;
  processPs4Input(packet,nowMs+=50);assert(!lqr_drive_start_pending);
  packet.circle=true;packet.left_stick_y=-127;
  processPs4Input(packet,nowMs+=50);assert(!lqr_drive_start_pending);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.left_stick_y=-127;processPs4Input(packet,nowMs+=50);
  assert(lqr_drive_start_pending);
  // A rejected repeated start must not change an active drive's state/ownership.
  startPendingLqrDrive();
  assert(!executeOperatorCommand(OperatorCommand::Start));
  assert(lqr_auto_drive_active && ps4_drive_state==Ps4DriveState::DriveRequested);
  assert(!executeOperatorCommand(OperatorCommand::EnableDrivers));
  handlePs4Connection(false);
  assert(!lqr_auto_drive_active && !lqr_drive_start_pending && lqr_soft_stop_active);
  // Pending cancellation on disconnect, including a serial-origin request.
  setupCase();handlePs4Connection(true);assert(requestLqrDrive(.05f,'>'));
  handlePs4Connection(false);assert(!lqr_drive_start_pending && !lqr_auto_drive_active);
  // Both drive signs use the common gate and neutral performs a soft stop.
  for(int direction:{-127,127}){
    setupCase();handlePs4Connection(false);handlePs4Connection(true);
    packet={};processPs4Input(packet,nowMs+=50);
    packet.left_stick_y=direction;processPs4Input(packet,nowMs+=50);
    assert(lqr_drive_start_pending);
    startPendingLqrDrive();near(lqr_target_speed,direction<0?.05f:-.05f);
    lqr_speed_for_control=direction<0?.2f:-.2f;
    packet={};processPs4Input(packet,nowMs+=50);
    assert(lqr_soft_stop_active && lqr_target_speed==0 && lqr_direction_test_armed);
    near(ps4_yaw_output,0);assert(pins[27] && pins[32]);
    step(direction<0?.2f:-.2f);
    assert((direction<0 ? 1:-1)*speed_direct_control>0); // retain verified feedback sign
  }
  // Pure yaw shares the stable-start gate; saturation preserves common torque.
  setupCase();handlePs4Connection(false);handlePs4Connection(true);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.right_stick_x=127;processPs4Input(packet,nowMs+=50);
  assert(lqr_drive_start_pending && lqr_drive_pending_speed==0);
  startPendingLqrDrive();
  for(int i=0;i<20;++i)processPs4Input(packet,nowMs+=50);
  near(ps4_yaw_output,.1f);
  float turnRight=0,turnLeft=0;
  test_LQR_u=.2f;applyPs4Steering(turnRight,turnLeft);
  near(turnRight,.1f);near(turnLeft,.3f);
  test_LQR_u=.98f;applyPs4Steering(turnRight,turnLeft);
  near(turnRight,.96f);near(turnLeft,1);near((turnRight+turnLeft)/2,.98f);
  test_LQR_u=1.5f;applyPs4Steering(turnRight,turnLeft);near(turnRight,1);near(turnLeft,1);
  packet.right_stick_x=-127;
  for(int i=0;i<25;++i)processPs4Input(packet,nowMs+=50);
  assert(ps4_yaw_output<0);
  // Disconnect releases both axes and pending/active commands; reconnect held is inert.
  handlePs4Connection(false);near(ps4_drive_speed_request,0);near(ps4_yaw_output,0);
  assert(!lqr_auto_drive_active && !lqr_drive_start_pending);
  handlePs4Connection(true);
  processPs4Input(packet,nowMs+=50);assert(!lqr_drive_start_pending);
  // Explicit full stop gates outputs, disables drivers, and needs separate re-enable.
  packet={};processPs4Input(packet,nowMs+=50);
  packet.cross=true;processPs4Input(packet,nowMs+=50);
  assert(emergency_stop_active && !pins[27] && !pins[32] && !lqr_direction_test_armed);
  assert(std::string(last_stop_reason)=="DISARM");
  packet.options=true;processPs4Input(packet,nowMs+=50);assert(!balance_wait_active);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.options=true;processPs4Input(packet,nowMs+=50);assert(!balance_wait_active);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.share=true;processPs4Input(packet,nowMs+=50);
  assert(!emergency_stop_active && pins[27] && pins[32] && !balance_wait_active);
  near(right_target_voltage,0);near(left_target_voltage,0);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.options=true;processPs4Input(packet,nowMs+=50);assert(balance_wait_active);
  // Leg actions are edges, retain the original bounds, and are blocked in motion.
  setupCase();handlePs4Connection(false);handlePs4Connection(true);
  packet={};processPs4Input(packet,nowMs+=50);ps4_leg_extension_percent=0;legWrites=0;
  packet.dpad_up=true;
  for(int i=0;i<5;++i)processPs4Input(packet,nowMs+=50);
  assert(legWrites==1 && legCommand==10);
  for(int i=0;i<5;++i){packet={};processPs4Input(packet,nowMs+=50);packet.dpad_up=true;processPs4Input(packet,nowMs+=50);}
  assert(legCommand==30 && legWrites==3);
  packet={};processPs4Input(packet,nowMs+=50);packet.dpad_down=true;processPs4Input(packet,nowMs+=50);
  assert(legCommand==20);
  lqr_soft_stop_active=true;packet={};processPs4Input(packet,nowMs+=50);
  packet.dpad_down=true;processPs4Input(packet,nowMs+=50);assert(legCommand==20);
  // Stable CSV copies remain independent if the robot starts after capture.
  setupCase();stopBalanceTest();velocity_trace_count=1;velocity_trace[0].elapsed_ms=876;
  assert(!traceDownloadBlocked(1));velocity_trace[0].elapsed_ms=999;
  assert(http_trace_snapshot.velocity[0].elapsed_ms==876);
  loop_timing_trace[0].elapsed_ms=432;loop_timing_count=1;
  assert(!traceDownloadBlocked(2));loop_timing_trace[0].elapsed_ms=999;
  assert(http_trace_snapshot.timing[0].elapsed_ms==432);
  lqr_control_trace[0].elapsed_ms=654;lqr_control_trace_count=1;
  assert(!traceDownloadBlocked(3));lqr_control_trace[0].elapsed_ms=999;
  assert(http_trace_snapshot.control[0].elapsed_ms==654);
  http_trace_request.store(1);http_trace_kind=2;
  assert(traceDownloadBlocked(3));assert(http_trace_kind==2);
  http_trace_request.store(0);

  // Trace sum and status report selected contributions, not unused speed PID.
  setupCase();start(1);near(speed_direct_control,0);
  updateLqrControlTrace(0,0,true);
  auto& sample=lqr_control_trace[0];
  near(sample.angle_control+sample.gyro_control+sample.distance_control+sample.speed_control,sample.lqr_u);
  Serial.output.clear();printLqrStatus();
  assert(Serial.output.find("speed_control=0.000000")!=std::string::npos);
  std::cout<<"PASS: production PS4 release/edges/reconnect, start/neutral/disconnect/full-stop, leg bounds, yaw headroom, feedback signs, recovery/fall, immutable CSV and Web status\n";
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

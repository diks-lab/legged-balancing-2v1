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
    match = re.search(r'^(?:void|bool|uint8_t|const char\*|float)\s+' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    start, pos, depth = match.start(), match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos] + '\n'

prefix = source[source.index('namespace {\n//=================================================='):source.index('}  // namespace\n\nvoid setup()')]
prefix = prefix.replace('BLDCMotor motor', 'DriveTestMotor motor').replace('BLDCDriver3PWM driver', 'DriveTestDriver driver')
prelude = r'''
#include "Arduino.h"
#include "CheckedWheelSensor.h"
#include "LqrMotorVoltage.h"
#include "Ps4Diagnostics.h"
#include "Ps4EventInbox.h"
#include "BoundedHttpResponse.h"
#include "DiagnosticTelemetry.h"
#include "Mpu6050Config.h"
#include "BootAutoStart.h"
#include "WheelCalibration.h"
#include "FallAutoRecovery.h"
#include "DiagnosticWebSocket.h"
#define SERIAL_DIAGNOSTICS 1
#include "ContinuingHttpFile.h"
#include "ReleaseMemoryObservation.h"
#include "AllocationResponseObserver.h"
#include <memory>
AllocationResponseObserver liveAllocations;
AllocationResponseSummary finishAllocationResponse(){return liveAllocations.finish();}
ReleaseMemoryRecord memoryEvents[64]{};unsigned memoryEventCount=0;
extern "C" void observeReleaseMemory(uint16_t e,uint16_t cid,uint32_t arg){
 if(memoryEventCount<64)memoryEvents[memoryEventCount++]={0,0,0,e,cid,arg};
}
void observeHttpStatusSend(bool){}
bool release_memory_serial_active=false;
bool first_http_request=false;
bool http_response_pending=false;
uint16_t first_http_route=0;
using PGM_P=const char*;
#include "FocAlignmentObservation.h"
class Print { public: virtual ~Print()=default;virtual size_t write(uint8_t)=0; };
#include "driver/gpio.h"
#include "esp_timer.h"
#include "common/pid.h"
#include <atomic>
std::atomic<bool> http_task_ready{false};
#include <algorithm>
#include <cassert>
#include <iostream>
#include <vector>
#include <map>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <cstdlib>
struct Preferences {
 inline static std::map<std::string,std::vector<uint8_t>> data;
 inline static bool failOpen=false,failWrite=false,failRead=false;
 bool begin(const char*,bool){return !failOpen;}
 size_t getBytesLength(const char* k){return data[k].size();}
 size_t getBytes(const char*k,void*p,size_t n){if(failRead||data[k].size()!=n)return 0;memcpy(p,data[k].data(),n);return n;}
 size_t putBytes(const char*k,const void*p,size_t n){if(failWrite)return 0;const auto*b=static_cast<const uint8_t*>(p);data[k].assign(b,b+n);return n;}
};
using portMUX_TYPE=std::mutex;
ReleaseMemoryObserver release_memory_observer;
portMUX_TYPE release_memory_lock;
#define portMUX_INITIALIZER_UNLOCKED {}
void portENTER_CRITICAL(portMUX_TYPE* m){m->lock();}
void portEXIT_CRITICAL(portMUX_TYPE* m){m->unlock();}
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
  struct IP { String toString(){return "127.0.0.1";} int operator[](int i) const {return i==0?127:i==3?1:0;} };
  IP localIP(){return {};}
} WiFi;
struct File {
  bool exists=false, directory=false;
  explicit operator bool() const {return exists;}
  bool isDirectory() const {return directory;}
  size_t size() const {return 5;}
  size_t position=0;
  int read(uint8_t* out,size_t n){if(n>5-position)n=5-position;memcpy(out,"asset"+position,n);position+=n;return int(n);}
  void close(){}
};
struct { File file{}; int opens=0;
  File open(const char*,const char*) {++opens;return file;}
} LittleFS;
bool csvClientConnected=true;
std::function<void()> onCsvContent,onCsvCopy;
bool failCsvAllocation=false;
int csvAllocs=0,csvFrees=0;
void* csvMalloc(size_t size){if(failCsvAllocation)return nullptr;++csvAllocs;return std::malloc(size);}
void csvFree(void* p){if(p)++csvFrees;std::free(p);}
void copyCsvSnapshot(void* dst,const void* src,size_t n){if(onCsvCopy)onCsvCopy();memcpy(dst,src,n);}
struct NetworkClient {
  std::shared_ptr<int> _rxBuffer;
  int fd(){return 1;}
  void stop(){}
  bool connected(){return csvClientConnected;}
};
#include "HttpTransmitClient.h"
std::string* rawStatusBody=nullptr;
bool rawStatusFirstWrite=true;
int* rawStatusCode=nullptr;
std::function<void()> onStatusSend;
int forcedHttpEagain=0,httpSendCalls=0;
int send(int,const char* data,size_t size,int){
  ++httpSendCalls;if(forcedHttpEagain){--forcedHttpEagain;liveAllocations.record(0,1492,0x1000);errno=EAGAIN;return -1;}
  if(onStatusSend)onStatusSend();
  if(rawStatusFirstWrite){rawStatusBody->clear();rawStatusFirstWrite=false;sscanf(data,"HTTP/1.1 %d",rawStatusCode);}
  rawStatusBody->append(data,size);return static_cast<int>(size);
}
constexpr int MSG_DONTWAIT=1;
struct WebServer {
  bool _server=false;
  explicit WebServer(int) {}
  int status=0;std::string body;
  struct Client {bool connected(){return csvClientConnected;}};
  NetworkClient client(){rawStatusBody=&body;rawStatusCode=&status;rawStatusFirstWrite=true;return {};}
  void sendHeader(const char*,const String&){}
  void setContentLength(int){}
  void send(int code,const char*,const char* data){status=code;body=data;}
  void sendContent(const char* data){body+=data;if(onCsvContent)onCsvContent();}
  size_t streamFile(File,const char*) {status=200;body="asset";return 5;}
  virtual size_t _currentClientWrite(const char*,size_t n){return n;}
  virtual size_t _currentClientWrite_P(const char*,size_t n){return n;}
};
enum class Ps4ClassicStartup { Ready, AlreadyInitialized, StartFailed, NotEnabled };
Ps4ClassicStartup classicStartup=Ps4ClassicStartup::Ready;
Ps4ClassicStartup preparePs4ClassicBluetooth(){return classicStartup;}
struct {
  bool ok=false;int begins=0;
  void (*connect_cb)()=nullptr;void (*disconnect_cb)()=nullptr;void (*input_cb)()=nullptr;
  Ps4InputState input{};
  void attachOnConnect(void (*cb)()){connect_cb=cb;}
  void attachOnDisconnect(void (*cb)()){disconnect_cb=cb;}
  void attach(void (*cb)()){input_cb=cb;}
  bool begin(){++begins;return ok;} // No pairing-argument overload is allowed here.
} PS4;
enum class FOCMotorStatus { motor_ready=4, motor_calib_failed=5 };
struct DriveTestMotor {
  explicit DriveTestMotor(int) {}
  Direction sensor_direction=Direction::CCW;int pole_pairs=7;float zero_electric_angle=1;
  float shaft_angle=0,shaft_velocity=0,target=0,current_sp=0;
  int enabled=0;FOCMotorStatus motor_status=FOCMotorStatus::motor_ready;
  struct {float q=0,d=0;} voltage;
  float shaftAngle(){return shaft_angle;}
  float shaftVelocity(){return shaft_velocity;}
  int enablePin=-1;
  void enable(){enabled=1;if(enablePin>=0)digitalWrite(enablePin,HIGH);}
  void move(float v){target=current_sp=v;}
};
struct DriveTestDriver {
  int en,pwmA,pwmB,pwmC; DriveTestDriver(int a,int b,int c,int e):en(e),pwmA(a),pwmB(b),pwmC(c){}
  void disable(){digitalWrite(en,LOW);}
  void enable(){digitalWrite(en,HIGH);}
  void setPwm(float,float,float){}
};
struct MPU6050 {
  explicit MPU6050(TwoWire&){}
  int updates=0;float angle=0,gyro=0;
  std::function<void()> onUpdate;
  void update(){++updates;if(onUpdate)onUpdate();}
  float getAngleY(){return angle;} float getGyroY(){return gyro;}
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
bridge_start = source.index('diagnostic::Snapshot diagnostic_latest;')
bridge_end = source.index('void captureHttpStatus();', bridge_start)
prefix += source[bridge_start:bridge_end]
prefix = prefix.replace('void printHttpStartupMemory(const char* stage);', 'void printHttpStartupMemory(const char*) {}')
prefix += source[source.index('struct HttpResponseTransport {'):source.index('void serviceHttpFileResponse()')]
prefix += source[source.index('struct CalibrationNvs {'):source.index('void loadWheelCalibration(){')]
prefix += source[source.index('extern "C" void observeImuTransaction('):source.index('void observeFocAlignPoint(', source.index('extern "C" void observeImuTransaction('))]
names = [
    'handleWheelCalibrationRequest','handleWheelCalibrationAbort','loadWheelCalibration','saveWheelCalibration','requestWheelCalibration','fallRecoveryChecks','serviceFallAutoRecovery','serviceWheelFaultDiagnostics','updateStoppedMeasurements','bootAutoMeasurementsReady','serviceBootAutoStart','serviceHttpFileResponse','recordBoundedHttpResult','sendSmallHttpResponse',
    'updateTestLqrOutput','lqrTestModeLogName','lqrTestModeDisplayName','lqrSpeedSourceName',
    'requestLqrDrive','requestLqrSoftStop','startPendingLqrDrive','updateLqrDiagnostic',
    'stopLqrDriveOnly','terminateLqrDriveSession','stopBalanceTest','resetLqrTestState',
    'startBalanceTest','cycleLqrTestMode','updateBalanceWait','cancelBalanceWait',
    'confirmActualFall','detectDriveSafetyFault','wheelSensorsOK','stopOnWheelFault',
    'setBothTargetsToZero','stopBothMotorsAtZero','disableBothDrivers',
    'clearVelocityTrace','clearLoopTimingTrace','clearLqrControlTrace',
    'captureFocOutputSnapshot','updateLqrControlTrace','captureLqrDriveSignTrace','captureLqrStopTrace',
    'resetLqrDriveSignTrace','resetLqrStopTrace','startLqrStopTrace','setStatusLed','printLqrStatus',
    'stopForActualFall',
    'captureHttpStatus','serviceHttpRequests','handleWifiStatusPage','handleStartBalanceTest','handleStopBalanceTest','enqueueHttpCommand',
    'serveWebFile','cycleLqrSpeedSource','resetPs4MotionInput','executeOperatorCommand',
    'applyPs4Steering','ps4ButtonsReleased','handlePs4Connection','processPs4Input',
    'ps4InputStatesEqual','ps4DriveSpeedRequestFromLeftStickY',
    'onPs4Connect','onPs4Disconnect','onPs4Input','raisePs4ControlBarrier',
    'observePs4InputOnly','rearmPs4AfterFoc','updatePs4Controller',
    'setupPs4Controller','servicePs4DiagnosticSerial','serviceHttpPs4DiagnosticSerial','drainPs4DiagnosticSerial','executePs4Command','ps4CommandName','ps4CommandRejectionReason',
    'updateWheelVelocityDiagnostic','updateLqrState',
]
# No control behavior is stubbed: these functions only print or store logs.
stubs = r'''
Ps4InputState readPs4Input(){return PS4.input;}
void formatHttpResources(char* text,size_t capacity){snprintf(text,capacity,"\nHTTP resources: host stub\n");}

float legCommand=-1;int legWrites=0;
void moveBothLegsToPercent(float x){legCommand=x;++legWrites;}
void printPs4InputDiagnostic(const Ps4InputState&){}
void printPs4DriveStopDiagnostic(const Ps4InputState&,bool,bool,uint32_t){}
void printPs4DriveStopSummary(uint32_t){}
void printLqrDriveSnapshot(const char*){}
void printLqrDriveSignTrace(const char*){}
void printLqrStopTrace(const char*){}
void publishStartupPhase(const char* stage){diagnostic_startup_phase.store(stage);}
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
struct ParkedWsIO {
 void observe(const diagnostic::WsObservation&){}
 bool open=true;unsigned sends=0;std::string input;
 uint32_t now(){return nowMs;}bool connected(){return open;}void close(){open=false;}
 int read(char*p,size_t n){if(input.empty())return -EAGAIN;size_t k=std::min(n,input.size());memcpy(p,input.data(),k);input.erase(0,k);return k;}
 int write(const char*,size_t){++sends;return -EAGAIN;}
 bool acceptKey(const char*,char*out){strcpy(out,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");return true;}
 void snapshot(diagnostic::Snapshot&s){s=diagnostic_latest;}
};
void nearAt(float x,float y,int line){if(!(std::fabs(x-y)<1e-5f))fprintf(stderr,"near line=%d x=%f y=%f\n",line,x,y);assert(std::fabs(x-y)<1e-5f);}
#define near(x,y) nearAt(x,y,__LINE__)
void setupCase(LqrTestMode mode=LqrTestMode::Full){
  nowMs+=1000;
  boot_auto_start=BootAutoStart{false};
  fall_auto_recovery.cancel();calibration_loaded=true;calibration_active=false;calibration_abort=false;calibration_commit_phase=0;
  resetLqrTestState(); cancelBalanceWait();
  lqr_test_mode=mode;
  test_system_ready=true; startup_foc_ready.store(true); emergency_stop_active=false;
  sensor1.fault=sensor2.fault=false;
  diagnostic_calibration_done.store(true);diagnostic_calibration_ok.store(true);
  motor1.enablePin=motor1EnablePin;motor2.enablePin=motor2EnablePin;
  motor1.enabled=motor2.enabled=1;motor1.motor_status=motor2.motor_status=FOCMotorStatus::motor_ready;
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
  motor1.shaft_velocity=speed*kRightForwardSign;
  motor2.shaft_velocity=speed*kLeftForwardSign;
  control_dt_ms=10;
  diagnostic_imu_initialized.store(true);diagnostic_imu_io_ok=diagnostic_imu_io_seen=true;noteImuMeasurement();
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
  assert(lqr_test_mode==LqrTestMode::Full); // Real production global default.
  health_diagnostics=diagnostic::HealthDiagnostics{};
  gateRightSensorFault();
  diagnostic::Snapshot first_health;copyHealthReport(first_health);
  assert(first_health.health.stages[diagnostic::RightSensor]==diagnostic::Failed);
  assert(!strcmp(first_health.health.first,"RIGHT_SENSOR"));
  firstHealthFault("IMU_I2C_FAILED");copyHealthReport(first_health);
  assert(!strcmp(first_health.health.first,"RIGHT_SENSOR"));
  health_diagnostics=diagnostic::HealthDiagnostics{};
  // Measured forward rotation: right shaft decreases, left increases.
  static_assert(kRightForwardSign==-1 && kLeftForwardSign==1);
  static_assert(m1_direction==-0.5f && m2_direction==0.5f);
  for(float direction:{-1.0f,1.0f}){
    setupCase();
    motor1.shaft_angle=-2*direction;motor2.shaft_angle=2*direction;
    motor1.shaft_velocity=-3*direction;motor2.shaft_velocity=3*direction;
    updateLqrState();near(LQR_distance,2*direction);near(LQR_speed,3*direction);
    previous_velocity_diagnostic_us=micros();
    previous_right_angle=motor1.shaft_angle;previous_left_angle=motor2.shaft_angle;
    delay(10);motor1.shaft_angle-=.03f*direction;motor2.shaft_angle+=.03f*direction;
    updateWheelVelocityDiagnostic();near(diag_LQR_speed_10ms,3*direction);
    const float right=lqrMotorVoltage(.4f*direction,kRightForwardSign,1);
    const float left=lqrMotorVoltage(.4f*direction,kLeftForwardSign,1);
    near(right,-.4f*direction);near(left,.4f*direction);
    // Same vehicle-frame direction on both sides; no second averaging.
    near(right*kRightForwardSign,left*kLeftForwardSign);
  }
  // Resource reservation and post-Web sensor sampling stay separate.
  {
    TwoWire prepared_bus(0);
    CheckedWheelSensor prepared_sensor(prepared_bus,gateWheelOutputsOff);
    assert(prepared_bus.begin(19,18,400000));
    assert(prepared_bus.requests==0 && !prepared_sensor.ready);
    prepared_sensor.beginPreparedBus();
    assert(prepared_sensor.ready && prepared_bus.begins==1 && prepared_bus.requests==4);
    TwoWire bad_bus(1);bad_bus.nack=1;
    CheckedWheelSensor bad_sensor(bad_bus,gateWheelOutputsOff);
    assert(bad_bus.begin(23,5,400000));
    bad_sensor.beginPreparedBus();
    assert(!bad_sensor.ready && bad_sensor.fault && bad_bus.begins==1);
  }
  http_status_queue=xQueueCreate(1,sizeof(HttpStatus));
  http_command_queue=xQueueCreate(1,sizeof(char));
  onHttpYield=[]{serviceHttpRequests();};
  // AUTO START still requires an explicit t-equivalent and starts at rest.
  setupCase(); lqr_direction_test_armed=false; lqr_diagnostic_armed=false;
  assert(!requestLqrDrive(.3f,'>'));
  assert(startBalanceTest(false));
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
  step(.2f);updateBalanceWait();assert(balance_brake_right>0 && balance_brake_left<0);
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

  // Latest diagnostic output/cache observes values without storing history.
  setupCase();motor1.voltage.q=.125f;motor2.voltage.q=-.25f;
  captureFocOutputSnapshot();updateLqrControlTrace(.75f,-.5f,true);
  near(diagnostic_right_command,.75f);near(diagnostic_left_command,-.5f);
  near(last_foc_output.right_voltage_q,.125f);
  motor1.move(.75f);near(last_foc_output.right_voltage_q,.125f);
  LQR_angle=5.93f;angle_zeropoint=1;captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("Cached AngleY: 5.930000 deg")!=String::npos);
  assert(wifi_log_server.body.find("Reference: 1.000000 deg")!=String::npos);
  assert(wifi_log_server.body.find("Delta: 4.930000 deg")!=String::npos);
  assert(wifi_log_server.body.find("driver_init=1")!=String::npos);
  assert(wifi_log_server.body.find("motor_status=4 motor_enabled=1 EN=1")!=String::npos);
  // No USB: gating must already be complete whenever any stop-path Serial
  // output occurs, including the startup diagnostic before the trace dumps.
  setupCase();
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
  assert(std::string(last_stop_reason)=="TILT_STOP");
  assert(std::string(diagnostic_stop_reason)=="TILT_STOP" && diagnostic_stop_ms==nowMs);
  near(diagnostic_right_command,0);near(diagnostic_left_command,0);
  assert(last_foc_output.right_voltage_q==0 && !last_foc_output.right_driver_en);
  captureHttpStatus();handleWifiStatusPage();assert(wifi_log_server.status==200);
  handleStopBalanceTest();serviceHttpRequests();assert(std::string(last_stop_reason)=="TILT_STOP");
  handleStartBalanceTest();serviceHttpRequests();assert(std::string(last_stop_reason)=="TILT_STOP");
  emergency_stop_active=false;assert(startBalanceTest(false));
  assert(std::string(diagnostic_stop_reason)=="TILT_STOP"); // restart keeps latest event
  assert(std::string(last_stop_reason)=="NONE" && balance_wait_active);
  assert(stopBalanceTest());assert(std::string(last_stop_reason)=="MANUAL_STOP");

  // A fault on either checked sensor still latches a full stop and blocks t.
  sensor1.begin(19,18);sensor2.begin(23,5);
  setupCase();sensor2.fault=true;gateWheelOutputsOff();assert(stopOnWheelFault());
  assert(!pins[27] && !pins[32] && !test_system_ready && !lqr_direction_test_armed);
  assert(!startBalanceTest(false));
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
  LittleFS.file={true,false};first_http_request=true;first_http_route=1;memoryEventCount=0;
  serveWebFile("/index.html","text/html");
  while(http_response_pending){++nowMs;serviceHttpFileResponse();}
  assert(wifi_log_server.status==200);
  assert(memoryEventCount==2 && memoryEvents[0].event==MemHttpFirstFile && memoryEvents[0].cid==1 && memoryEvents[0].arg==5);
  assert(memoryEvents[1].event==MemHttpFirstResult && memoryEvents[1].arg==(200u<<8));
  first_http_request=false;
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
    assert(lqr_drive_start_pending&&lqr_test_mode==LqrTestMode::Full);
    startPendingLqrDrive();near(lqr_target_speed,direction<0?.05f:-.05f);
    lqr_speed_for_control=direction<0?.2f:-.2f;
    packet={};processPs4Input(packet,nowMs+=50);
    assert(lqr_soft_stop_active && lqr_target_speed==0 && lqr_direction_test_armed);
    near(ps4_yaw_output,0);assert(pins[27] && pins[32]);
    step(direction<0?.2f:-.2f);
    assert((direction<0 ? 1:-1)*speed_direct_control>0); // retain verified feedback sign
    near(speed_control,.25f*(lqr_target_speed-diagnostic_control_inputs.speed)*1.25f);
    near(speed_direct_control,-speed_control);
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
  near(turnRight,-.1f);near(turnLeft,.3f);
  test_LQR_u=.98f;applyPs4Steering(turnRight,turnLeft);
  near(turnRight,-.96f);near(turnLeft,1);near((turnRight*kRightForwardSign+turnLeft*kLeftForwardSign)/2,.98f);
  test_LQR_u=1.5f;applyPs4Steering(turnRight,turnLeft);near(turnRight,-1);near(turnLeft,1);
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
  // PS4 library init result and actual connection are distinct diagnostic facts.
  setupCase();ps4_diagnostics=Ps4Diagnostics{};PS4.ok=false;
  setupPs4Controller();assert(ps4_diagnostics.init_attempted && !ps4_diagnostics.init_ok);
  assert(!ps4_diagnostics.connected && PS4.begins==1);
  PS4.ok=true;setupPs4Controller();assert(ps4_diagnostics.init_ok && !ps4_diagnostics.connected);
  const int begins_before_mode_failure=PS4.begins;
  for (auto failure : {Ps4ClassicStartup::AlreadyInitialized, Ps4ClassicStartup::StartFailed, Ps4ClassicStartup::NotEnabled}) {
    classicStartup=failure;setupPs4Controller();
    assert(!ps4_diagnostics.init_ok && PS4.begins==begins_before_mode_failure);
  }
  classicStartup=Ps4ClassicStartup::Ready;setupPs4Controller();
  stopBalanceTest();test_system_ready=false;startup_foc_ready.store(false);
  ps4_was_connected=false;handlePs4Connection(true);
  assert(ps4_diagnostics.connected && ps4_diagnostics.connects==1);
  packet={};processPs4Input(packet,nowMs+=50);
  packet.options=true;processPs4Input(packet,nowMs+=50);
  assert(!balance_wait_active && !lqr_direction_test_armed);
  assert(std::string(ps4_diagnostics.action)=="START");
  assert(std::string(ps4_diagnostics.result)=="REJECTED");
  assert(std::string(ps4_diagnostics.reason)=="FOC_NOT_INITIALIZED");
  assert(ps4_diagnostics.last_press & (1u<<10));
  packet={};processPs4Input(packet,nowMs+=50);
  assert(ps4_diagnostics.last_release & (1u<<10));
  packet.left_stick_x=-128;packet.left_stick_y=-127;
  packet.right_stick_x=127;packet.right_stick_y=-11;
  packet.l2=64;packet.r2=255;processPs4Input(packet,nowMs+=50);
  captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("PS4 init=OK connected=1")!=String::npos);
  assert(wifi_log_server.body.find("LX=-128 LY=-127 RX=127 RY=-11")!=String::npos);
  assert(wifi_log_server.body.find("neutral_left/right=0/0")!=String::npos);
  assert(wifi_log_server.body.find("reason=FOC_NOT_INITIALIZED")!=String::npos);
  assert(wifi_log_server.body.find("last_release=Options")!=String::npos);
  // Disconnection events retain counters and release the raw buttons/state.
  handlePs4Connection(false);
  assert(!ps4_diagnostics.connected && ps4_diagnostics.disconnects==1);
  captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("event=DISCONNECTED")!=String::npos);
  // New observations must never block on UART capacity or write an unbounded burst.
  ps4_diagnostic_serial_offset=ps4_diagnostic_serial_size=0;
  ps4_diagnostic_serial_last_ms=0;nowMs+=1000;Serial.output.clear();Serial.room=0;
  ps4_diagnostics.dirty=true;servicePs4DiagnosticSerial();assert(Serial.output.empty());
  const auto staged=ps4_diagnostic_serial_size;
  Serial.room=5;servicePs4DiagnosticSerial();assert(Serial.output.size()==(SERIAL_DIAGNOSTICS?5u:0u));
  Serial.room=128;servicePs4DiagnosticSerial();assert(Serial.output.size()<=53);
  for(int i=0;i<30;++i)servicePs4DiagnosticSerial();
  assert(Serial.output.size()==staged);assert((Serial.output.find("event=DISCONNECTED")!=String::npos)==bool(SERIAL_DIAGNOSTICS));
  ps4_diagnostics.dirty=true;const auto count=Serial.output.size();
  servicePs4DiagnosticSerial();assert(Serial.output.size()==count); // Still inside 500 ms limit.
  Serial.room=128;

  // After HTTP task publication, the control loop performs no UART diagnostic I/O.
  http_task_ready.store(true);ps4_diagnostics.dirty=true;nowMs+=1000;
  const auto before_io=Serial.output.size();servicePs4DiagnosticSerial();
  assert(Serial.output.size()==before_io);
  captureHttpStatus();serviceHttpPs4DiagnosticSerial();
  assert(Serial.output.size()<=before_io+48);assert((Serial.output.size()>before_io)==bool(SERIAL_DIAGNOSTICS));
  for(int i=0;i<30;++i)serviceHttpPs4DiagnosticSerial();
  assert((Serial.output.find("PS4 init=OK connected=0")!=String::npos)==bool(SERIAL_DIAGNOSTICS));
  http_task_ready.store(false);

  // Longest named button sets and decision data must fit in the Web status copy.
  setupCase();ps4_diagnostics=Ps4Diagnostics{};ps4_diagnostics.init_attempted=ps4_diagnostics.init_ok=true;
  packet={};packet.cross=packet.circle=packet.square=packet.triangle=true;
  packet.dpad_up=packet.dpad_down=packet.dpad_left=packet.dpad_right=true;
  packet.l1=packet.r1=packet.options=packet.share=packet.ps_button=true;
  ps4_diagnostics.observe(packet,nowMs);ps4_diagnostics.observe({},nowMs+1);
  ps4_diagnostics.observe(packet,nowMs+2);
  ps4_diagnostics.decision("ENABLE_DRIVERS",false,"ACTIVE_TEST_OR_SAFETY_GATE");
  startup_phase="FOC_INHIBITED_FULL_STOP";
  captureHttpStatus();handleWifiStatusPage();
  assert(wifi_log_server.body.find("last_press=Cross,Circle,Square,Triangle,Up,Down,Left,Right,L1,R1,Options,Share,PS")!=String::npos);
  assert(wifi_log_server.body.find("reason=ACTIVE_TEST_OR_SAFETY_GATE accepted/rejected=0/1\n")!=String::npos);

  // Callback wiring: callbacks only publish, with no Serial or actuator work.
  setupCase();stopBalanceTest();ps4_was_connected=false;ps4_diagnostics=Ps4Diagnostics{};
  PS4.ok=true;setupPs4Controller();
  assert(PS4.connect_cb && PS4.disconnect_cb && PS4.input_cb);
  const auto before_callback_output=Serial.output;
  PS4.connect_cb();
  assert(!ps4_was_connected && Serial.output==before_callback_output);
  updatePs4Controller();assert(ps4_was_connected && ps4_diagnostics.connects==1);
  // Press/release within less than the old 50ms poll interval are both retained.
  PS4.input={};PS4.input_cb();
  PS4.input.options=true;PS4.input_cb();
  PS4.input.options=false;PS4.input_cb();
  assert(!balance_wait_active);
  updatePs4Controller();assert(balance_wait_active);
  assert(ps4_diagnostics.last_press&(1u<<10));
  assert(ps4_diagnostics.last_release&(1u<<10));
  const auto unchanged_revision=ps4_diagnostics.revision;
  const auto unchanged_sample=ps4_diagnostics.sample_ms;
  const auto unchanged_decisions=ps4_diagnostics.decisions;
  for(int i=0;i<30;++i){nowMs+=50;PS4.input_cb();updatePs4Controller();}
  assert(ps4_was_connected && ps4_diagnostics.revision==unchanged_revision);
  assert(ps4_diagnostics.sample_ms==unchanged_sample && ps4_diagnostics.decisions==unchanged_decisions);
  assert(ps4_diagnostics.fifo_duplicates>=30);
  // Hold forward+yaw with no new callbacks: ramps continue, no implicit disconnect.
  setupCase();PS4.input={};PS4.input_cb();updatePs4Controller();
  PS4.input.left_stick_y=-127;PS4.input.right_stick_x=127;PS4.input_cb();updatePs4Controller();
  assert(lqr_drive_start_pending);startPendingLqrDrive();lqr_target_speed=0;
  const auto held_revision=ps4_diagnostics.revision, held_sample=ps4_diagnostics.sample_ms;
  for(int i=0;i<15;++i){nowMs+=50;updatePs4Controller();}
  near(lqr_target_speed,.05f);near(ps4_yaw_output,.1f);
  assert(ps4_was_connected && lqr_auto_drive_active);
  assert(ps4_diagnostics.revision==held_revision && ps4_diagnostics.sample_ms==held_sample);
  // Even changing transport duplicate counters cannot create periodic Serial output.
  http_task_ready.store(true);ps4_serial_seen_revision=ps4_diagnostics.revision;
  ps4_diagnostic_serial_offset=ps4_diagnostic_serial_size=0;Serial.output.clear();
  for(int i=0;i<20;++i){nowMs+=500;PS4.input_cb();updatePs4Controller();captureHttpStatus();serviceHttpPs4DiagnosticSerial();}
  assert(Serial.output.empty());http_task_ready.store(false);
  // An actual neutral report still starts normal soft stop; maintenance completes it.
  lqr_speed_for_control=.2f;PS4.input={};PS4.input_cb();updatePs4Controller();
  assert(lqr_soft_stop_active && lqr_target_speed==0 && lqr_direction_test_armed);
  for(int i=0;i<200 && ps4_drive_state==Ps4DriveState::Stopping;++i){step();nowMs+=50;updatePs4Controller();}
  assert(!lqr_soft_stop_active && ps4_drive_state==Ps4DriveState::Neutral);
  // Ordered disconnect+reconnect survives one drain; a held reconnect cannot drive.
  const auto old_connections=ps4_diagnostics.connects, old_disconnections=ps4_diagnostics.disconnects;
  PS4.disconnect_cb();PS4.connect_cb();PS4.input.left_stick_y=-127;PS4.input_cb();
  updatePs4Controller();
  assert(ps4_diagnostics.connects==old_connections+1 && ps4_diagnostics.disconnects==old_disconnections+1);
  assert(ps4_was_connected && !lqr_drive_start_pending && !lqr_auto_drive_active);
  // Stop priority: older Share/Options must never re-enable after a short Cross.
  PS4.input={};PS4.input_cb();
  PS4.input.share=true;PS4.input_cb();PS4.input.share=false;PS4.input_cb();
  PS4.input.options=true;PS4.input_cb();PS4.input.options=false;PS4.input_cb();
  PS4.input.cross=true;PS4.input_cb();PS4.input.cross=false;PS4.input_cb();
  updatePs4Controller();
  assert(emergency_stop_active && !pins[27] && !pins[32] && !lqr_direction_test_armed && !balance_wait_active);
  // Overflow preserves stop/link latches and press/release evidence, not unsafe replay.
  const auto previous_overflows=ps4_diagnostics.fifo_overflows;
  for(int i=0;i<80;++i){PS4.input.right_stick_x=(i%2)?40:50;PS4.input_cb();}
  PS4.input.options=true;PS4.input_cb();PS4.input.options=false;PS4.input_cb();
  PS4.input.cross=true;PS4.input_cb();PS4.input.cross=false;PS4.input_cb();
  PS4.disconnect_cb();PS4.connect_cb();
  updatePs4Controller();
  assert(ps4_diagnostics.fifo_overflows>previous_overflows && ps4_event_rearm_required);
  assert(ps4_diagnostics.overflow_presses&(1u<<10));
  assert(ps4_diagnostics.overflow_releases&(1u<<10));
  assert(ps4_diagnostics.overflow_presses&1u);assert(ps4_diagnostics.overflow_releases&1u);
  assert(std::string(ps4_diagnostics.reason)=="QUEUE_OVERFLOW_REARM");
  assert(emergency_stop_active && !pins[27] && !pins[32] && ps4_was_connected);
  // Recovery needs a real fresh neutral callback, even if it repeats latest values.
  PS4.input={};PS4.input_cb();updatePs4Controller();assert(!ps4_event_rearm_required);
  PS4.input.share=true;PS4.input_cb();updatePs4Controller();
  assert(!emergency_stop_active && pins[27] && pins[32] && !balance_wait_active);
  // FOC handoff ignores queued startup motion but never loses queued Cross.
  setupCase();PS4.input={};PS4.input_cb();PS4.input.options=true;PS4.input_cb();
  rearmPs4AfterFoc();updatePs4Controller();assert(lqr_direction_test_armed && !balance_wait_active && !lqr_drive_start_pending);
  PS4.input={};PS4.input_cb();updatePs4Controller();
  PS4.input.cross=true;PS4.input_cb();rearmPs4AfterFoc();updatePs4Controller();
  assert(emergency_stop_active && !pins[27] && !pins[32]);
  // Overflow itself does not add a disconnect/full stop; actual neutral still stops.
  setupCase();PS4.input={};PS4.input_cb();updatePs4Controller();
  PS4.input.left_stick_y=-127;PS4.input_cb();updatePs4Controller();
  assert(lqr_drive_start_pending);startPendingLqrDrive();
  for(int i=0;i<80;++i){PS4.input.right_stick_x=(i%2)?40:50;PS4.input_cb();}
  updatePs4Controller();assert(lqr_auto_drive_active && !emergency_stop_active && ps4_was_connected);
  // Keep the FIFO full again, then put neutral into the overflow summary.
  for(int i=0;i<80;++i){PS4.input.right_stick_x=(i%2)?60:70;PS4.input_cb();}
  lqr_speed_for_control=.2f;PS4.input={};PS4.input_cb();updatePs4Controller();
  assert(lqr_soft_stop_active && lqr_target_speed==0 && !emergency_stop_active);

  // The first forwarded packet after reconnect must not treat held Options as a new start.
  PS4.disconnect_cb();PS4.connect_cb();PS4.input={};PS4.input.options=true;PS4.input_cb();
  updatePs4Controller();assert(!balance_wait_active && !lqr_drive_start_pending);
  PS4.input={};PS4.input_cb();PS4.input.cross=true;PS4.input_cb();PS4.input.cross=false;PS4.input_cb();
  updatePs4Controller();assert(emergency_stop_active && !pins[27] && !pins[32]);

  // A stalled HTTP sender owns only its copied status. Actual PS4 callbacks and
  // control-side draining remain runnable while that sender is parked.
  setupCase();PS4.ok=true;setupPs4Controller();captureHttpStatus();
  std::mutex statusMutex;std::condition_variable statusCv;bool entered=false,resume=false;
  onStatusSend=[&]{std::unique_lock<std::mutex> lock(statusMutex);
    if(!entered){entered=true;statusCv.notify_one();statusCv.wait(lock,[&]{return resume;});}};
  std::thread statusSender([]{handleWifiStatusPage();});
  {std::unique_lock<std::mutex> lock(statusMutex);statusCv.wait(lock,[&]{return entered;});}
  PS4.connect_cb();PS4.input={};PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.disconnect_cb();updatePs4Controller();assert(!ps4_was_connected);
  PS4.connect_cb();PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.input.cross=true;PS4.input_cb();updatePs4Controller();assert(emergency_stop_active);
  {std::lock_guard<std::mutex> lock(statusMutex);resume=true;}statusCv.notify_one();
  statusSender.join();onStatusSend={};

  // An actual EAGAIN backoff does not gate PS4 events, diagnostics or full stop.
  setupCase();PS4.ok=true;setupPs4Controller();http_filesystem_ready=true;
  LittleFS.file={true,false};csvClientConnected=true;forcedHttpEagain=1;liveAllocations.begin();
  serveWebFile("/index.html","text/html");assert(http_response_pending);
  serviceHttpFileResponse();assert(http_response_pending);
  const int sendsBeforeBackoff=httpSendCalls;
  PS4.connect_cb();PS4.input={};PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.disconnect_cb();updatePs4Controller();assert(!ps4_was_connected);
  PS4.connect_cb();PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.input.cross=true;PS4.input_cb();updatePs4Controller();assert(emergency_stop_active);
  ps4_diagnostic_serial_offset=ps4_diagnostic_serial_size=0;
  ps4_diagnostic_serial_last_ms=nowMs-1000;Serial.room=128;
  captureHttpStatus();const auto diagnosticBefore=Serial.output.size();
  serviceHttpPs4DiagnosticSerial();assert((Serial.output.size()>diagnosticBefore)==bool(SERIAL_DIAGNOSTICS));
  ++nowMs;serviceHttpFileResponse();assert(httpSendCalls==sendsBeforeBackoff&&http_response_pending);
  while(http_response_pending){++nowMs;serviceHttpFileResponse();}

  bool haveAllocation=false;ReleaseMemoryRecord allocationRecord{};
  while(release_memory_observer.pop(allocationRecord)){
    if((allocationRecord.event&255)==MemHttpAllocCore0&&allocationRecord.arg==1&&allocationRecord.free==1492&&allocationRecord.largest==0x1000)haveAllocation=true;
  }
  assert(haveAllocation); // Actual async completion emits this window's failed request.

  // Actual WebSocket engine parked on EAGAIN: PS4 delivery/full-stop still proceeds.
  setupCase();PS4.ok=true;setupPs4Controller();
  ParkedWsIO wsio;diagnostic::WebSocket<ParkedWsIO> ws;
  ws.setBoot("0123456789abcdef");ws.connected(wsio);
  wsio.input="GET /diagnostics HTTP/1.1\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";
  ws.poll(wsio);ws.poll(wsio);ws.poll(wsio);assert(ws.active()&&wsio.sends>0);
  PS4.connect_cb();PS4.input={};PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.disconnect_cb();updatePs4Controller();assert(!ps4_was_connected);
  PS4.connect_cb();PS4.input_cb();updatePs4Controller();assert(ps4_was_connected);
  PS4.input.cross=true;PS4.input_cb();updatePs4Controller();assert(emergency_stop_active&&!pins[27]&&!pins[32]);
  ws.disconnect(wsio);assert(!wsio.open);

  // Boot starts immediately with fixed zero reference, using shared start reset.
  auto bootFixture=[&](){
    setupCase(LqrTestMode::AttitudeOnly);
    boot_auto_start=BootAutoStart{};
    sensor1.begin(19,18);sensor2.begin(23,5);
    lqr_diagnostic_armed=lqr_direction_test_armed=false;lqr_zero_set=false;
    last_stop_reason="NONE";diagnostic_startup_failed.store(false);
    diagnostic_imu_initialized.store(true);diagnostic_imu_calibrating.store(false);
    diagnostic_imu_io_ok=diagnostic_imu_io_seen=true;
    LQR_angle=1.9f;LQR_gyro=3;LQR_speed=1;LQR_distance=4.25f;
    noteImuMeasurement();
  };
  bootFixture();test_system_ready=false;serviceBootAutoStart();assert(!lqr_direction_test_armed);
  test_system_ready=true;startup_foc_ready.store(false);serviceBootAutoStart();assert(!lqr_direction_test_armed);
  startup_foc_ready.store(true);diagnostic_imu_calibrating.store(true);serviceBootAutoStart();assert(!lqr_direction_test_armed);
  diagnostic_imu_calibrating.store(false);diagnostic_calibration_done.store(false);serviceBootAutoStart();assert(!lqr_direction_test_armed);
  diagnostic_calibration_done.store(true);
  // Tilted boot stays gated off, with fixed reference visible and live attitude.
  LQR_angle=15;noteImuMeasurement();
  for(int n=0;n<10;++n){serviceBootAutoStart();assert(!lqr_direction_test_armed&&!pins[27]&&!pins[32]);}
  publishDiagnosticSnapshot(true);assert(diagnostic_latest.state==8&&diagnostic_latest.reference_valid);
  near(diagnostic_latest.values[0],15);near(diagnostic_latest.values[1],0);
  LQR_distance=6.25f;LQR_angle=1.9f;noteImuMeasurement();
  // First in-range call: no hold or stationary-speed gate, no time advancement.
  const auto immediateMs=millis();serviceBootAutoStart();updateLqrDiagnostic();
  assert(millis()==immediateMs && lqr_direction_test_armed && lqr_zero_set);
  assert(pins[27]&&pins[32]);
  assert(!balance_wait_active&&!balance_prepare_active);
  near(angle_zeropoint,0);near(distance_zeropoint,6.25f);
  near(lqr_drive_origin_distance,6.25f);near(lqr_drive_start_distance,6.25f);
  assert(boot_auto_start.phase==BootAutoStart::Phase::Finished);
  assert(test_LQR_u>0); // Uses actual 1.9 deg error, not the tilted startup pose.
  setBothTargetsToZero();nowMs+=50;noteImuMeasurement();publishDiagnosticSnapshot(false);
  near(diagnostic_latest.values[0],1.9f);near(diagnostic_latest.values[1],0);
  stopBalanceTest();
  for(int n=0;n<1000;++n){nowMs+=10;noteImuMeasurement();serviceBootAutoStart();}
  assert(!lqr_direction_test_armed);
  assert(executeOperatorCommand(OperatorCommand::Start));assert(balance_wait_active); // Manual path unchanged.
  bootFixture();assert(executeOperatorCommand(OperatorCommand::FullStop));emergency_stop_active=false;
  serviceBootAutoStart();assert(!lqr_direction_test_armed);
  bootFixture();serviceBootAutoStart();float br=.1f,bl=.1f;stopForActualFall(br,bl);
  emergency_stop_active=false;serviceBootAutoStart();assert(!lqr_direction_test_armed);
  for(int fault=0;fault<4;++fault){
    bootFixture();
    if(fault==0)diagnostic_imu_io_ok=false;
    if(fault==1)diagnostic_calibration_ok.store(false);
    if(fault==2)diagnostic_imu_finite=false;
    if(fault==3){sensor2.fault=true;assert(stopOnWheelFault());}
    serviceBootAutoStart();assert(!lqr_direction_test_armed);
    assert(boot_auto_start.phase==BootAutoStart::Phase::Cancelled);
    diagnostic_imu_io_ok=true;diagnostic_calibration_ok.store(true);diagnostic_imu_finite=true;sensor2.fault=false;
    serviceBootAutoStart();assert(!lqr_direction_test_armed);
  }

  // Stop/fault during tilted boot wait cannot be revived by raising the body.
  for(int blocked=0;blocked<3;++blocked){
    bootFixture();LQR_angle=15;noteImuMeasurement();serviceBootAutoStart();
    if(blocked==0)executeOperatorCommand(OperatorCommand::FullStop);
    if(blocked==1){diagnostic_imu_io_ok=false;serviceBootAutoStart();}
    if(blocked==2){motor1.motor_status=FOCMotorStatus::motor_calib_failed;serviceBootAutoStart();}
    diagnostic_imu_io_ok=true;motor1.motor_status=FOCMotorStatus::motor_ready;
    emergency_stop_active=false;LQR_angle=1;noteImuMeasurement();serviceBootAutoStart();
    assert(!lqr_direction_test_armed&&!boot_auto_start.pending());
  }

  // Fault probes are bounded, gated and cannot clear the fault/start latch.
  setupCase();sensor1.setDiagnosticContext(3,42);I2Cone.count=0;sensor1.update();
  assert(sensor1.fault);assert(stopOnWheelFault());
  const auto fault_ms=sensor1.diagnostics.first.ms;
  I2Cone.count=2;nowMs+=1000;serviceWheelFaultDiagnostics();
  assert(sensor1.diagnostics.probes&&sensor1.diagnostics.probe.result==1);
  assert(sensor1.fault&&!test_system_ready&&emergency_stop_active&&!pins[27]&&!pins[32]);
  const auto probes=sensor1.diagnostics.probes;
  serviceWheelFaultDiagnostics();assert(sensor1.diagnostics.probes==probes);
  LQR_angle=0;noteImuMeasurement();serviceFallAutoRecovery();serviceBootAutoStart();
  assert(!lqr_direction_test_armed&&sensor1.diagnostics.first.ms==fault_ms);
  I2Cone.count=0;nowMs+=1000;serviceWheelFaultDiagnostics();
  assert(sensor1.diagnostics.probe.result==3&&sensor1.fault);
  I2Cone.count=2;
  // Automatic entry always resets FULL, origin, targets, PID and speed-filter state.
  bootFixture();lqr_test_mode=LqrTestMode::AttitudeDistance;
  LQR_angle=1;LQR_distance=4.5f;noteImuMeasurement();
  lqr_target_speed=.04f;lqr_ps4_target_tilt_deg=.3f;lqr_drive_angle_bias=.2f;
  serviceBootAutoStart();
  assert(lqr_test_mode==LqrTestMode::Full&&lqr_direction_test_armed);
  near(angle_zeropoint,0);near(distance_zeropoint,4.5f);
  near(lqr_target_speed,0);near(lqr_ps4_target_tilt_deg,0);near(lqr_drive_angle_bias,0);
  assert(!lqr_control_speed_filter_initialized);
  near(pid_speed.P,.25f);near(pid_distance.P,.1f);
  step(.2f);near(speed_control,-.05f);near(speed_direct_control,.05f);
  LQR_distance=4.6f;step(.2f);near(distance_control,.01f);
  near(test_LQR_u,angle_control+gyro_control+distance_control+speed_direct_control);
  // A reduced diagnostic selection cannot silently persist through a fall restart.
  lqr_test_mode=LqrTestMode::AttitudeOnly;float fullR=0,fullL=0;stopForActualFall(fullR,fullL);
  LQR_angle=-1;LQR_distance=-3;noteImuMeasurement();serviceFallAutoRecovery();
  assert(lqr_test_mode==LqrTestMode::Full&&lqr_direction_test_armed);
  near(angle_zeropoint,0);near(distance_zeropoint,-3);
  near(lqr_target_speed,0);near(lqr_ps4_target_tilt_deg,0);near(lqr_drive_angle_bias,0);
  // Explicit manual tuning remains available and does not overwrite the auto policy.
  stopBalanceTest();emergency_stop_active=false;lqr_test_mode=LqrTestMode::AttitudeOnly;
  assert(startBalanceTest(false)&&lqr_test_mode==LqrTestMode::AttitudeOnly);
  // First in-range sample, even while rotating/moving; no time advancement.
  bootFixture();serviceBootAutoStart();br=bl=0;stopForActualFall(br,bl);
  assert(fall_auto_recovery.pending && emergency_stop_active);
  LQR_angle=2.01f;LQR_gyro=20;LQR_speed=10;
  auto recoverFrame=[&](unsigned ms){nowMs+=ms;noteImuMeasurement();serviceFallAutoRecovery();};
  recoverFrame(0);assert(!lqr_direction_test_armed);
  publishDiagnosticSnapshot(true);assert(diagnostic_latest.recovery_checks==14);
  LQR_angle=1.9f;LQR_distance=6.5f;const auto entryMs=millis();
  recoverFrame(0);assert(millis()==entryMs&&lqr_direction_test_armed&&!emergency_stop_active);
  near(angle_zeropoint,0);near(distance_zeropoint,6.5f);
  nowMs+=50;noteImuMeasurement();publishDiagnosticSnapshot(false);
  assert(diagnostic_latest.state==3 && std::string(diagnostic_latest.stop_reason)=="TILT_STOP");
  // A new confirmed fall after recovery arms a new immediate attempt.
  stopForActualFall(br,bl);LQR_angle=-2;recoverFrame(0);assert(lqr_direction_test_armed);
  for(float boundary:{-2.0f,2.0f}){
    stopForActualFall(br,bl);LQR_angle=boundary;recoverFrame(0);
    assert(lqr_direction_test_armed);
  }
  // Real stopped measurement path: no FOC dependency, no hidden hold, no recalibration.
  bootFixture();serviceBootAutoStart();stopForActualFall(br,bl);
  health_diagnostics=diagnostic::HealthDiagnostics{};
  for(auto&stage:health_diagnostics.report.stages)stage=diagnostic::Passed;
  mpu6050.angle=15;mpu6050.gyro=20;mpu6050.onUpdate=[](){observeImuTransaction(0,14);};
  motor1.enabled=motor2.enabled=0;
  const auto savedCalibration=saved_wheel_calibration;
  for(int i=0;i<100;++i){
    nowMs+=10;const int r=I2Cone.requests,l=I2Ctwo.requests,m=mpu6050.updates;
    updateStoppedMeasurements();serviceFallAutoRecovery();
    assert(I2Cone.requests==r+1&&I2Ctwo.requests==l+1&&mpu6050.updates==m+1);
    assert(!digitalRead(motor1EnablePin)&&!digitalRead(motor2EnablePin));
    assert(test_system_ready&&startup_foc_ready.load()&&diagnostic_calibration_ok.load());
    assert((fallRecoveryChecks()&14)==14&&fall_auto_recovery.pending);
  }
  assert(!memcmp(&savedCalibration,&saved_wheel_calibration,sizeof(savedCalibration)));
  assert(health_diagnostics.report.first[0]==0);
  for(auto stage:health_diagnostics.report.stages)assert(stage==diagnostic::Passed);
  mpu6050.angle=1.9f;motor1.shaft_angle=-7;motor2.shaft_angle=7;
  const auto stoppedEntryMs=millis();updateStoppedMeasurements();serviceFallAutoRecovery();
  assert(millis()==stoppedEntryMs&&lqr_direction_test_armed&&motor1.enabled&&motor2.enabled);
  assert(digitalRead(motor1EnablePin)&&digitalRead(motor2EnablePin));near(angle_zeropoint,0);near(distance_zeropoint,7);
  // A real NACK cancels recovery and gates off. A healthy IMU stays independently valid.
  for(int side=0;side<2;++side){
    bootFixture();serviceBootAutoStart();stopForActualFall(br,bl);
    health_diagnostics=diagnostic::HealthDiagnostics{};
    mpu6050.angle=0;TwoWire&bus=side?I2Ctwo:I2Cone;bus.nack=2;
    updateStoppedMeasurements();assert(stopOnWheelFault());publishDiagnosticSnapshot(true);
    assert(!test_system_ready&&!fall_auto_recovery.pending&&diagnostic_latest.imu_valid&&diagnostic_latest.foc_ready);
    assert(!strcmp(health_diagnostics.report.first,side?"LEFT_SENSOR":"RIGHT_SENSOR"));
    assert(!strcmp(side?sensor2.error:sensor1.error,"I2C_ADDRESS/NACK"));
    assert(!digitalRead(motor1EnablePin)&&!digitalRead(motor2EnablePin));bus.nack=0;
    serviceFallAutoRecovery();assert(!lqr_direction_test_armed);
  }
  bootFixture();serviceBootAutoStart();stopForActualFall(br,bl);
  mpu6050.onUpdate=[](){observeImuTransaction(2,0);};mpu6050.angle=0;
  updateStoppedMeasurements();serviceFallAutoRecovery();publishDiagnosticSnapshot(true);
  assert(!lqr_direction_test_armed&&!fall_auto_recovery.pending&&!diagnostic_latest.imu_valid);
  assert(diagnostic_latest.imu_tx==2&&diagnostic_latest.imu_rx==0);
  assert(!digitalRead(motor1EnablePin)&&!digitalRead(motor2EnablePin));
  mpu6050.onUpdate={};mpu6050.angle=mpu6050.gyro=0;diagnostic_imu_io_ok=diagnostic_imu_finite=true;
  // Traversing between samples never invents an unseen in-range measurement.
  stopForActualFall(br,bl);LQR_angle=2.1f;recoverFrame(0);
  LQR_angle=-2.1f;recoverFrame(0);assert(!lqr_direction_test_armed);
  LQR_angle=-1.99f;recoverFrame(0);assert(lqr_direction_test_armed);
  // Invalid/latest failed/stale measurement cannot restart; recovering health alone does not re-arm.
  for(int invalid=0;invalid<4;++invalid){
    bootFixture();serviceBootAutoStart();stopForActualFall(br,bl);LQR_angle=0;
    if(invalid==0)diagnostic_imu_io_ok=false;
    if(invalid==1)diagnostic_imu_finite=false;
    if(invalid==2){nowMs+=251;}
    if(invalid==3)LQR_angle=NAN;
    serviceFallAutoRecovery();assert(!lqr_direction_test_armed&&!fall_auto_recovery.pending);
    LQR_angle=0;diagnostic_imu_io_ok=diagnostic_imu_finite=true;noteImuMeasurement();
    serviceFallAutoRecovery();assert(!lqr_direction_test_armed);
  }
  for(int kind=0;kind<5;++kind){
    bootFixture();serviceBootAutoStart();stopForActualFall(br,bl);
    if(kind==0)executeOperatorCommand(OperatorCommand::FullStop);
    if(kind==1)stopBalanceTest();
    if(kind==2)diagnostic_imu_io_ok=false;
    if(kind==3)calibration_loaded=false;
    if(kind==4)motor1.motor_status=FOCMotorStatus::motor_calib_failed;
    LQR_angle=LQR_gyro=LQR_speed=0;recoverFrame(0);recoverFrame(2000);
    assert(!lqr_direction_test_armed&&!fall_auto_recovery.pending);
    publishDiagnosticSnapshot(true);
    if(kind==0)assert(diagnostic_latest.state==4);
    if(kind==4)assert(diagnostic_latest.state==6 && !(diagnostic_latest.recovery_checks&8));
  }
  // Explicit calibration never queues from running control; success path is
  // exercised through the real FOC wrapper + two-slot storage suite.
  setupCase();requestWheelCalibration();assert(!calibration_active.load());
  setupCase();lqr_diagnostic_armed=lqr_direction_test_armed=false;
  startup_web_ready.store(true);requestWheelCalibration();
  assert(calibration_active.load()&&!test_system_ready&&!boot_auto_start.pending());
  assert(!fall_auto_recovery.pending);
  calibration_active=false;

  // Production persistence admission: never save unsuccessful/aborted records.
  Preferences::data.clear();Preferences::failWrite=false;
  calibration_loaded=false;loadWheelCalibration();assert(!calibration_loaded);
  setupCase();calibration_active=true;calibration_commit_phase=0;
  for(auto&r:foc_alignment_observer.records){r={};r.accepted=r.pp_observed=r.pp_pass=true;}
  saved_calibration_slot=-1;
  assert(saveWheelCalibration());assert(calibration_loaded);
  const auto oldStorage=Preferences::data;
  for(int failure=0;failure<7;++failure){
    calibration_commit_phase=0;calibration_abort=false;
    for(auto&r:foc_alignment_observer.records){r.accepted=r.pp_observed=r.pp_pass=true;r.expired=false;}
    if(failure==0)foc_alignment_observer.records[0].accepted=false;
    if(failure==1)foc_alignment_observer.records[1].pp_pass=false;
    if(failure==2)foc_alignment_observer.records[0].expired=true;
    if(failure==3)calibration_abort=true;
    if(failure==4)Preferences::failWrite=true;
    if(failure==5)Preferences::failOpen=true;
    if(failure==6)Preferences::failRead=true;
    assert(!saveWheelCalibration());assert(Preferences::data==oldStorage);
    Preferences::failWrite=false;Preferences::failOpen=false;Preferences::failRead=false;
  }
  calibration_active=false;calibration_abort=false;calibration_loaded=false;
  saved_wheel_calibration={};saved_calibration_slot=-1;
  loadWheelCalibration();assert(calibration_loaded && saved_calibration_slot>=0);
  // Calibration request cancels automatic boot intent; completion stays stopped.
  setupCase();lqr_direction_test_armed=lqr_diagnostic_armed=false;
  requestWheelCalibration();test_system_ready=true;startup_foc_ready=true;
  calibration_active=false;emergency_stop_active=true;serviceBootAutoStart();
  assert(!lqr_direction_test_armed&&!boot_auto_start.pending());

  // Cancellation covers queued requests and running scans without closing shared I2C.
  setupCase();http_command_queue->full=false;calibration_request_pending=false;
  handleWheelCalibrationRequest();assert(calibration_request_pending);
  handleWheelCalibrationAbort();serviceHttpRequests();
  assert(!calibration_request_pending&&!calibration_active);
  assert(!strcmp(calibration_status,"ABORTED"));
  setupCase();lqr_direction_test_armed=lqr_diagnostic_armed=false;requestWheelCalibration();
  calibration_commit_phase=0;handleWheelCalibrationAbort();
  assert(calibration_abort && calibration_commit_phase==1&&!pins[27]&&!pins[32]);
  setupCase();calibration_active=true;calibration_commit_phase=2;
  handleWheelCalibrationAbort();assert(!calibration_abort&&calibration_commit_phase==2);
  calibration_active=false;

  // Latest telemetry reports selected contributions, not unused speed PID.
  setupCase();start(1);near(speed_direct_control,0);
  nowMs+=50;publishDiagnosticSnapshot(false);
  assert(std::isnan(diagnostic_latest.values[11])); // Drive disables position feedback.
  near(diagnostic_latest.values[9]+diagnostic_latest.values[10]+diagnostic_latest.values[12],diagnostic_latest.values[13]);
  // Exact PID inputs, signed/filtered speed selection and no second averaging.
  setupCase();LQR_angle=0.4f;LQR_gyro=-0.2f;LQR_distance=0.3f;
  step(0.6f);nowMs+=50;publishDiagnosticSnapshot(false);
  near(diagnostic_latest.values[26],diagnostic_control_inputs.angle_error);
  near(diagnostic_latest.values[26],LQR_angle-angle_zeropoint);
  near(diagnostic_latest.values[3],LQR_gyro);
  near(diagnostic_latest.values[8],lqr_speed_for_control);
  near(diagnostic_latest.values[6],motor1.shaft_velocity*kRightForwardSign);
  near(diagnostic_latest.values[7],motor2.shaft_velocity*kLeftForwardSign);
  near(diagnostic_latest.values[27],distance_zeropoint-LQR_distance);
  near(diagnostic_latest.values[28],lqr_target_speed);
  near(diagnostic_latest.values[31],lqr_speed_raw_for_control);
  setupCase(LqrTestMode::AttitudeOnly);step();nowMs+=50;publishDiagnosticSnapshot(false);
  assert(std::isnan(diagnostic_latest.values[8]));
  assert(std::isnan(diagnostic_latest.values[11])&&std::isnan(diagnostic_latest.values[12]));
  setupCase();start(1);ps4_drive_state=Ps4DriveState::DriveRequested;
  lqr_ps4_window_speed=0.27f;step(0.9f);nowMs+=50;publishDiagnosticSnapshot(false);
  near(diagnostic_latest.values[8],0.27f);
  near(diagnostic_latest.values[26],LQR_angle-angle_zeropoint-lqr_ps4_target_tilt_deg-lqr_drive_angle_bias);
  near(diagnostic_latest.values[29],lqr_ps4_target_tilt_deg+lqr_drive_angle_bias);
  assert(std::isnan(diagnostic_latest.values[12]));
  assert(lqr_test_mode==LqrTestMode::Full);
  near(speed_control,.25f*(lqr_target_speed-.27f)*1.25f);
  near(distance_control,0);near(speed_direct_control,0);
  near(test_LQR_u,angle_control+gyro_control); // Existing active-PS4 profile, not four simultaneous terms.
  assert(stopBalanceTest());
  LQR_angle=12.5f;LQR_gyro=-3.2f;
  angle_zeropoint=-0.8f;step();nowMs+=50;publishDiagnosticSnapshot(false);
  near(diagnostic_latest.values[0],12.5f);
  near(diagnostic_latest.values[1],-0.8f);
  near(diagnostic_latest.values[3],-3.2f);
  assert(!diagnostic_latest.running);
  LQR_angle=-8.25f;LQR_gyro=1.6f;step();nowMs+=50;publishDiagnosticSnapshot(false);
  near(diagnostic_latest.values[0],-8.25f);
  near(diagnostic_latest.values[1],-0.8f);
  near(diagnostic_latest.values[3],1.6f);
  assert(std::isnan(diagnostic_latest.values[26])&&std::isnan(diagnostic_latest.values[8]));
  const auto measurement_ms=diagnostic_imu_ms,measurement_seq=diagnostic_imu_seq;
  nowMs+=300;publishDiagnosticSnapshot(false);
  assert(!diagnostic_latest.imu_valid&&diagnostic_latest.imu_reason==8);
  assert(diagnostic_latest.imu_ms==measurement_ms&&diagnostic_latest.imu_seq==measurement_seq);
  assert(std::isnan(diagnostic_latest.values[0]));
  sensor2.fault=true;nowMs+=50;publishDiagnosticSnapshot(false);
  assert(diagnostic_latest.state==6&&diagnostic_latest.imu_reason==8);
  updateStoppedMeasurements();publishDiagnosticSnapshot(true);
  assert(diagnostic_latest.state==6&&diagnostic_latest.imu_valid&&diagnostic_latest.imu_reason==0);
  assert(diagnostic_latest.foc_ready&&(diagnostic_latest.recovery_checks&8));
  sensor2.fault=false;
  Serial.output.clear();printLqrStatus();
  assert(Serial.output.find("speed_control=0.000000")!=std::string::npos);
  std::cout<<"PASS: production PS4 release/edges/reconnect, start/neutral/disconnect/full-stop, leg bounds, yaw headroom, feedback signs, recovery/fall, latest telemetry/stop event and Web status\n";
}
'''
foc = Path(os.environ.get('SIMPLEFOC_SRC','.pio/libdeps/esp32dev/Arduino-FOC/src'))
with tempfile.TemporaryDirectory(prefix='drive-control-') as directory:
    cpp=Path(directory)/'drive.cpp';binary=Path(directory)/'drive'
    bodies = [function(n).replace('malloc(sizeof(HttpTraceSnapshot))','csvMalloc(sizeof(HttpTraceSnapshot))')
              .replace('free(http_trace_snapshot)','csvFree(http_trace_snapshot)')
              .replace('memcpy(http_trace_snapshot->','copyCsvSnapshot(http_trace_snapshot->') for n in names]
    prototypes = '\n'.join(body[:body.index('{')]+';' for body in bodies)
    prelude=prelude.replace('#define SERIAL_DIAGNOSTICS 1','#define SERIAL_DIAGNOSTICS '+os.environ.get('DRIVE_SERIAL_DIAGNOSTICS','1'))
    cpp.write_text(prelude+prefix+prototypes+stubs+'\n'.join(bodies)+'}\n'+checks)
    sanitizer_flags = ['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('CSV_TEST_SANITIZERS') else []
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-variable',
                    '-pthread','-Iinclude','-Itests/wheel_host','-I'+str(foc),str(cpp),
                    str(foc/'common/pid.cpp'),str(foc/'common/base_classes/Sensor.cpp'),'-o',str(binary)]+sanitizer_flags,check=True)
    subprocess.run([str(binary)],check=True)

for term in ['malloc(', 'free(', 'releaseHttpTraceSnapshot(', 'reapHttpTraceSnapshot(']:
    assert term not in function('serviceHttpRequests')

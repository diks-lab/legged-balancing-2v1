"""Run production resource and HTTP startup functions with RTOS/network doubles."""
from pathlib import Path
import re
import subprocess
import tempfile

source = Path('src/main.cpp').read_text()
def function(name):
    match = re.search(r'^(?:void|bool) ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos]

code = r'''
#include <cassert>
#include <atomic>
std::atomic<bool> diagnostic_startup_failed{false};
std::atomic<const char*> diagnostic_startup_phase{"WEB_WAIT"};
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <string>
#include <vector>
using String=std::string;
struct { bool measuring=false;bool active(){return measuring;} } foc_alignment_observer;
std::atomic<const char*> foc_deferred_http_failure{nullptr};
bool foc_deferred_http_started=false,foc_deferred_ip_announcement=false;
enum class FocGateSource {WebAbort};
using QueueHandle_t = int*;
using TaskHandle_t = void*;
using StackType_t = uint8_t;
struct StaticTask_t { uint8_t storage[64]; };
bool kEnableWifiLogServer = true;
constexpr uint32_t kWifiStatusCheckIntervalMs=1000,kWifiConnectTimeoutMs=5000;
uint32_t now_ms=0;
uint32_t millis(){return now_ms;}
constexpr uint32_t kHttpTaskStackBytes = 8192;
uint8_t http_task_stack[kHttpTaskStackBytes]; StaticTask_t http_task_tcb;
TaskHandle_t http_task_handle = nullptr;
std::atomic<bool> http_task_ready{false};
struct HttpStatus { char text[2048]; uint32_t ps4_revision=0; };
struct HttpTraceSnapshot {char data[10776];};
QueueHandle_t http_status_queue = nullptr, http_command_queue = nullptr;
bool http_filesystem_ready = false;
bool fail_task = false, fail_mount = false, fail_mode=false;
int fail_queue = 0, queues = 0, deletes = 0, creates = 0, wifi_calls = 0;
std::vector<std::string> stages;
constexpr uint32_t MALLOC_CAP_INTERNAL=1, MALLOC_CAP_8BIT=2;
struct { unsigned getFreeHeap(){return 20000;} } ESP;
unsigned heap_caps_get_free_size(uint32_t caps){assert(caps==3);return 18000;}
unsigned heap_caps_get_largest_free_block(uint32_t caps){assert(caps==3);return 6000;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t){return 4000;}
struct {
  std::string text;
  void print(const char* s){text+=s;}
  void println(const char* s){text+=s;text+='\n';}
  template<class... A> void printf(const char* f,A... args){
    char buf[1024];snprintf(buf,sizeof(buf),f,args...);text+=buf;
  }
} Serial;
QueueHandle_t xQueueCreate(int n,size_t size){
  assert(n==1);assert(size==sizeof(HttpStatus)||size==1);
  return ++queues==fail_queue ? nullptr : new int;
}
void vQueueDelete(QueueHandle_t q){assert(q);++deletes;delete q;}
void httpTask(void*){}
TaskHandle_t xTaskCreateStaticPinnedToCore(void(*entry)(void*),const char*,
    uint32_t bytes,void*,int priority,StackType_t* stack,StaticTask_t* tcb,int core){
  ++creates;stages.push_back("task");assert(wifi_calls==0);
  assert(!http_task_ready.load());assert(entry==httpTask);
  assert(bytes==8192 && stack==http_task_stack && tcb==&http_task_tcb);
  assert(priority==1 && core==0);
  return fail_task ? nullptr : tcb;
}
struct {bool begin(bool format){assert(!format);stages.push_back("fs");return !fail_mount;}} LittleFS;
constexpr int WIFI_STA=1,WL_CONNECTED=3;
constexpr char SSID_NAME[]="test",SSID_KEY[]="not-a-real-key";
struct IP {
  uint32_t value=0;
  explicit operator uint32_t() const {return value;}
  String toString() const {
    char ip[16];snprintf(ip,sizeof(ip),"%u.%u.%u.%u",
      (value>>24)&255,(value>>16)&255,(value>>8)&255,value&255);
    return ip;
  }
};
struct {
  int connection_status=0; IP ip{};
  int status(){return connection_status;}
  IP localIP(){return ip;}
  bool mode(int){++wifi_calls;stages.push_back("wifi");assert(!http_task_ready.load());return !fail_mode;}
  void setAutoReconnect(bool){}
  void begin(const char*,const char*){assert(http_task_handle);}
} WiFi;
std::atomic<bool> calibration_active{false},calibration_abort{false};
bool calibration_load_attempted=true;
const char* calibration_status="RESTORED";
void loadWheelCalibration(){calibration_load_attempted=true;}
void publishDiagnosticSnapshot(bool){}
constexpr uint8_t kWheelRightSda=19,kWheelRightScl=18,kWheelLeftSda=23,kWheelLeftScl=5;
bool http_response_pending=false;
bool fail_listen=false;
bool test_system_ready=false, emergency_stop_active=false, fail_foc=false, lose_preflight_during_foc=false;
int foc_calls=0, gates=0, snapshots=0;
bool wheel_outputs_enabled=false;
void gateWheelOutputsOff(){++gates;wheel_outputs_enabled=false;}
void gateWheelOutputsOffFrom(FocGateSource){gateWheelOutputsOff();}
void disableBothDrivers(){gateWheelOutputsOff();}
int listen_attempts=0, route_count=0, handled=0, help_count=0;
constexpr int HTTP_GET=1,HTTP_POST=2;
using PGM_P=const char*;
struct HttpMemoryRequest {explicit HttpMemoryRequest(uint16_t=0){} ~HttpMemoryRequest(){}};
void sendSmallHttpResponse(int,const char*){}
void observeHttpStatusSend(bool){}
struct ObserverStub {void httpAfter(uint32_t,uint32_t){}}release_memory_observer;
int release_memory_lock;
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
struct WebServer {
 virtual size_t _currentClientWrite(const char*,size_t n){return n;}
 virtual size_t _currentClientWrite_P(const char*,size_t n){return n;}
  explicit WebServer(int){}
  bool _server=false;
  template<class F> void on(const char*,int,F){++route_count;}
  template<class F> void onNotFound(F){++route_count;}
  void begin(){++listen_attempts;_server=!fail_listen;}
  void close(){_server=false;}
  void handleClient(){++handled;}
  void send(int,const char*,const char*){}
};
void printHelp(){++help_count;Serial.println("NORMAL OPERATION MENU");}
void serveWebFile(const char*,const char*){}
void handleWifiStatusPage(){}
void handleVelocityTraceDownload(){}
void handleLoopTimingTraceDownload(){}
void handleLqrControlTraceDownload(){}
void handleWheelCalibrationRequest(){}
void handleWheelCalibrationAbort(){}
void handleCycleLqrTestMode(){}
void handleCycleLqrSpeedSource(){}
void handleStartBalanceTest(){}
void handleStopBalanceTest(){}
'''
# Include the production listener-check subclass and cross-task publication state.
start = source.index('class CheckedHttpServer :')
end = source.index('BLDCMotor motor1(', start)
code += source[start:end]
code += r'''
constexpr unsigned I2C_BUFFER_LENGTH=128;
struct Bus {
  int index, calls=0; bool fail=false; unsigned timeout=0;
  bool begin(int sda,int scl,unsigned hz){
    assert(!wheel_outputs_enabled && wifi_calls==0 && creates==0);
    assert(hz==400000 && sda==(index?23:19) && scl==(index?5:18));
    stages.push_back(index?"i2c1":"i2c0");++calls;return !fail;
  }
  void setTimeOut(unsigned ms){timeout=ms;}
} I2Cone{0},I2Ctwo{1};
void captureHttpStatus(){++snapshots;}
void enforceStartupOutputGate();
void reportStartupFailure(const char*);
void initializeRobotHardware(){
  assert(startup_web_ready.load() && !emergency_stop_active);
  ++foc_calls;
  if(lose_preflight_during_foc){
    startup_web_ready.store(false);wheel_outputs_enabled=true;
    enforceStartupOutputGate();
    assert(!wheel_outputs_enabled && startup_foc_gate==StartupFocGate::Aborted);
    startup_web_ready.store(true); // Even immediate recovery cannot undo an abort.
  }
  if(!fail_foc)test_system_ready=true;
}
'''

code += '\n'.join(function(n) for n in
                  ['printStartupMemoryResult','printHttpStartupMemory','prepareI2cResources','publishStartupPhase','enforceStartupOutputGate','startupHardwareStepAllowed','advanceStartup','formatHttpResources', 'reportStartupFailure',
                   'maybePrintStartupMenu', 'setupWifiLogServer', 'startWifiHttpServer',
                   'updateWifiLogServer'])
code += r'''
void reset(){
  calibration_active=false;calibration_abort=false;calibration_load_attempted=true;
  foc_alignment_observer.measuring=false;foc_deferred_http_failure.store(nullptr);foc_deferred_http_started=foc_deferred_ip_announcement=false;
  if(http_status_queue)delete http_status_queue;
  if(http_command_queue)delete http_command_queue;
  http_status_queue=http_command_queue=nullptr;http_task_handle=nullptr;
  http_task_ready.store(false);fail_task=fail_mount=fail_mode=false;
  fail_queue=queues=deletes=creates=wifi_calls=0;stages.clear();Serial.text.clear();
  kEnableWifiLogServer=true;startup_foc_ready.store(false);startup_menu_printed=false;
  startup_web_ready.store(false);startup_http_ready.store(false);
  foc_initialization_attempted=foc_initialization_failed=false;startup_phase="WEB_WAIT";
  i2c_resources_attempted=false;i2c0_resources_ready=i2c1_resources_ready=true;
  I2Cone.calls=I2Ctwo.calls=0;I2Cone.fail=I2Ctwo.fail=false;I2Cone.timeout=I2Ctwo.timeout=0;
  test_system_ready=emergency_stop_active=fail_foc=lose_preflight_during_foc=false;
  startup_foc_gate.store(StartupFocGate::Waiting);
  foc_calls=gates=snapshots=0;wheel_outputs_enabled=false;
  http_routes_registered=http_listener_failure_reported=false;
  wifi_connection_ready=wifi_timeout_reported=false;wifi_announced_ip.clear();
  now_ms=wifi_wait_started_ms=wifi_status_last_check_ms=0;
  wifi_log_server_started=false;
  wifi_log_server.close();WiFi.connection_status=0;WiFi.ip.value=0;
  fail_listen=false;listen_attempts=route_count=handled=help_count=0;
}
void tick(){
  now_ms+=1000;updateWifiLogServer();advanceStartup();maybePrintStartupMenu();
}
void connect(uint32_t ip=0xc0a8012a){WiFi.connection_status=WL_CONNECTED;WiFi.ip.value=ip;}
int main(){
  reset();foc_alignment_observer.measuring=true;
  printHttpStartupMemory("during_alignment");reportStartupFailure("WEB_LOST_DURING_FOC");
  assert(Serial.text.empty() && String(foc_deferred_http_failure.load())=="WEB_LOST_DURING_FOC");
  calibration_active=false;calibration_abort=false;calibration_load_attempted=true;
  foc_alignment_observer.measuring=false;
  reportStartupFailure(foc_deferred_http_failure.exchange(nullptr));
  assert(Serial.text.find("WEB_LOST_DURING_FOC")!=String::npos);
  for (int failed=0;failed<=2;++failed){
    reset();i2c0_resources_ready=i2c1_resources_ready=false;
    I2Cone.fail=failed==1;I2Ctwo.fail=failed==2;wheel_outputs_enabled=true;
    prepareI2cResources();
    assert(!wheel_outputs_enabled && i2c_resources_attempted);
    assert(I2Cone.calls==1 && I2Ctwo.calls==1);
    assert(i2c0_resources_ready==(failed!=1) && i2c1_resources_ready==(failed!=2));
    assert(I2Cone.timeout==(failed==1?0:5) && I2Ctwo.timeout==(failed==2?0:5));
    assert((stages==std::vector<std::string>{"i2c0","i2c1"}));
    for(const char* stage:{"before_i2c0","after_i2c0","before_i2c1","after_i2c1"})
      assert((Serial.text.find(String("stage=")+stage)!=String::npos)==bool(HTTP_MEMORY_STAGE_DIAGNOSTICS));
    assert(Serial.text.find("rx_bytes=128 tx_bytes=128")!=String::npos);
    prepareI2cResources();assert(I2Cone.calls==1 && I2Ctwo.calls==1);
    setupWifiLogServer();connect();tick();
    if(failed){
      assert(foc_calls==0 && help_count==0 && !wheel_outputs_enabled);
      assert(wifi_log_server_started); // Web diagnostics survive bus reservation failure.
      assert(!startupHardwareStepAllowed());
      assert(String(startup_phase)==(failed==1?"I2C0_RESOURCES_FAILED":"I2C1_RESOURCES_FAILED"));
      for(int i=0;i<3;++i) { tick(); }
      assert(foc_calls==0);
    }else assert(foc_calls==1 && help_count==int(SERIAL_DIAGNOSTICS));
  }
  reset();
  // A 6 KiB largest heap block is smaller than the reserved 8 KiB stack.
  setupWifiLogServer();assert(http_task_handle && http_task_ready.load());
  assert((stages==std::vector<std::string>{"task","fs","wifi"}));
  assert(Serial.text.find("internal_largest=6000")!=String::npos || !HTTP_MEMORY_STAGE_DIAGNOSTICS);
  assert(Serial.text.find("api=static result=OK")!=String::npos);
  setupWifiLogServer();assert(creates==1);
  assert(help_count==0 && !wifi_log_server_started); // Task creation is not HTTP success.
  char resources[280];formatHttpResources(resources,sizeof(resources));
  assert(String(resources).find("stack_free_min_bytes=4000")!=String::npos);
  for(int failed=1;failed<=2;++failed){
    reset();fail_queue=failed;setupWifiLogServer();
    assert(!http_task_ready && !http_status_queue && !http_command_queue);
    assert(deletes==1 && creates==0 && wifi_calls==0);
  }
  reset();fail_task=true;setupWifiLogServer();
  assert(!http_task_ready && !http_status_queue && !http_command_queue);
  assert(deletes==2 && wifi_calls==0);
  assert(Serial.text.find("api=static result=NULL")!=String::npos);
  reset();fail_mount=true;setupWifiLogServer();
  assert(http_task_ready && !http_filesystem_ready && wifi_calls==1);
  // A mounted filesystem and created task alone never trigger the menu.
  reset();setupWifiLogServer();
  for(int i=0;i<4;++i)tick();
  assert(help_count==0 && listen_attempts==0 && foc_calls==0 && !wheel_outputs_enabled);
  tick();assert(Serial.text.find("stage=WIFI_TIMEOUT")!=String::npos);
  const auto timeout_log=Serial.text;
  tick();assert(Serial.text==timeout_log); // No repeated timeout spam.
  // Recovery after a startup timeout can complete startup exactly once.
  connect();tick();
  assert(help_count==int(SERIAL_DIAGNOSTICS) && startup_menu_printed && wifi_log_server_started && handled>0);
  assert(foc_calls==1 && startup_foc_ready && test_system_ready);
  assert(Serial.text.find("STARTUP memory result=READY heap_free=20000 internal_free=18000 internal_largest=6000")!=String::npos);
  const auto done=Serial.text.find("接続完了");
  const auto ip=Serial.text.find("WiFi DHCP IP: 192.168.1.42");
  const auto url=Serial.text.find("HTTP access URL: http://192.168.1.42/");
  const auto listener=Serial.text.find("WiFi trace HTTP server started.");
  const auto menu=Serial.text.find("NORMAL OPERATION MENU");
  assert(done!=String::npos && ip!=String::npos && url!=String::npos);
  assert(listener!=String::npos);assert((menu!=String::npos)==bool(SERIAL_DIAGNOSTICS));
  assert(done<ip && ip<url && url<listener);if(SERIAL_DIAGNOSTICS)assert(listener<menu);
  for(int i=0;i<4;++i)tick();
  assert(help_count==int(SERIAL_DIAGNOSTICS) && listen_attempts==1);
  // A continuing file response gates only new requests, not Wi-Fi monitoring.
  http_response_pending=true;const int before_pending=handled;tick();
  assert(handled==before_pending && startup_web_ready.load());
  WiFi.connection_status=0;WiFi.ip.value=0;tick();
  assert(handled==before_pending && !startup_web_ready.load());
  http_response_pending=false;connect();tick();assert(handled>before_pending);
  // A same-IP reconnect during observation defers diagnostics until afterwards.
  WiFi.connection_status=0;WiFi.ip.value=0;tick();
  foc_alignment_observer.measuring=true;Serial.text.clear();connect();tick();
  assert(Serial.text.empty() && foc_deferred_ip_announcement);
  calibration_active=false;calibration_abort=false;calibration_load_attempted=true;
  foc_alignment_observer.measuring=false;tick();
  assert(Serial.text.find("http://192.168.1.42/")!=String::npos && !foc_deferred_ip_announcement);
  // IP change and reconnect announce the numeric URL again, without another menu.
  connect(0xc0a8012b);tick();assert(help_count==int(SERIAL_DIAGNOSTICS));
  assert(Serial.text.find("http://192.168.1.43/")!=String::npos);
  WiFi.connection_status=0;WiFi.ip.value=0;tick();
  assert(!wifi_log_server_started && !wifi_connection_ready && help_count==int(SERIAL_DIAGNOSTICS));
  connect();tick();assert(wifi_log_server_started && help_count==int(SERIAL_DIAGNOSTICS));
  WiFi.ip.value=0;tick();
  assert(!wifi_log_server_started && help_count==int(SERIAL_DIAGNOSTICS));
  assert(Serial.text.find("stage=IP_LOST")!=String::npos);
  for(int i=0;i<5;++i)tick();
  assert(Serial.text.find("stage=IP_TIMEOUT")!=String::npos);
  connect();tick();assert(wifi_log_server_started && help_count==int(SERIAL_DIAGNOSTICS));
  // A cached IP without link association is not readiness either.
  reset();setupWifiLogServer();WiFi.ip.value=0xc0a8012a;
  for(int i=0;i<5;++i)tick();
  assert(help_count==0 && listen_attempts==0 && foc_calls==0 && !wheel_outputs_enabled);
  assert(Serial.text.find("stage=WIFI_TIMEOUT")!=String::npos);
  assert(Serial.text.find("STARTUP memory result=WIFI_TIMEOUT")!=String::npos);
  assert(Serial.text.find("HTTP access URL:")==String::npos);
  // Link association with no DHCP address is not readiness.
  reset();setupWifiLogServer();WiFi.connection_status=WL_CONNECTED;
  for(int i=0;i<5;++i)tick();
  assert(help_count==0 && listen_attempts==0 && foc_calls==0 && !wheel_outputs_enabled);
  assert(Serial.text.find("stage=IP_TIMEOUT")!=String::npos);
  assert(Serial.text.find("http://0.0.0.0/")==String::npos);
  connect();tick();assert(help_count==int(SERIAL_DIAGNOSTICS));
  // A failed listen must not set the started flag; retries register routes once.
  reset();setupWifiLogServer();connect();fail_listen=true;tick();
  assert(help_count==0 && !wifi_log_server_started && !wifi_log_server.listening() && foc_calls==0);
  assert(Serial.text.find("stage=HTTP_LISTEN")!=String::npos);
  const auto routes=route_count;tick();assert(listen_attempts==2 && route_count==routes);
  fail_listen=false;tick();assert(help_count==int(SERIAL_DIAGNOSTICS) && wifi_log_server_started);
  assert(route_count==routes);
  // Missing filesystem, failed queues/task or failed FOC cannot produce a menu.
  reset();fail_mount=true;setupWifiLogServer();connect();tick();
  assert(wifi_log_server_started && help_count==0 && foc_calls==0 && !wheel_outputs_enabled);
  assert(Serial.text.find("stage=LITTLEFS_MOUNT")!=String::npos);
  for(int failed=1;failed<=2;++failed){
    reset();fail_queue=failed;setupWifiLogServer();
    assert(help_count==0 && Serial.text.find("stage=HTTP_QUEUES")!=String::npos);
  }
  reset();fail_task=true;setupWifiLogServer();
  assert(help_count==0 && Serial.text.find("stage=HTTP_TASK")!=String::npos);
  reset();fail_foc=true;setupWifiLogServer();connect();tick();
  assert(help_count==0 && foc_calls==1 && foc_initialization_failed && !test_system_ready);
  for(int i=0;i<3;++i)tick();
  assert(foc_calls==1 && !wheel_outputs_enabled); // No automatic retry after hardware failure.
  reset();setupWifiLogServer();emergency_stop_active=true;connect();tick();
  assert(foc_calls==0 && !wheel_outputs_enabled && help_count==0);
  assert(String(startup_phase)=="FOC_INHIBITED_FULL_STOP");
  reset();fail_mode=true;setupWifiLogServer();
  assert(help_count==0 && Serial.text.find("stage=WIFI_MODE")!=String::npos);
  reset();setupWifiLogServer();lose_preflight_during_foc=true;connect();tick();
  assert(foc_calls==1 && !test_system_ready && !startup_foc_ready && foc_initialization_failed);
  assert(!wheel_outputs_enabled && help_count==0);
  tick();assert(foc_calls==1); // No reinitialization after a latched startup abort.
  // A disconnect after startup commit does not introduce a runtime stop condition.
  reset();setupWifiLogServer();connect();tick();
  assert(startup_foc_gate==StartupFocGate::Ready);
  wheel_outputs_enabled=true;startup_web_ready.store(false);enforceStartupOutputGate();
  assert(wheel_outputs_enabled && startup_foc_gate==StartupFocGate::Ready);
  reset();kEnableWifiLogServer=false;setupWifiLogServer();
  assert(help_count==0 && Serial.text.find("stage=WIFI_DISABLED")!=String::npos);
  reset();puts("PASS: HTTP static stack/memory diagnostics; FOC/FS/queues/task/link/IP/listen gates, timeouts, Web before FOC, no mDNS, recovery, one-shot startup menu");
}
'''
with tempfile.TemporaryDirectory(prefix='http-startup-') as directory:
    cpp = Path(directory) / 'test.cpp'
    binary = Path(directory) / 'test'
    cpp.write_text(code)
    for detailed in [0,1]:
        subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                        '-DSERIAL_DIAGNOSTICS='+str(detailed),'-DHTTP_MEMORY_STAGE_DIAGNOSTICS='+str(detailed),str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)

setup = function('setup')
assert 'printHelp();' not in setup and 'setupWifiLogServer();' in setup
assert 'initFOC' not in setup and 'sensor1.begin' not in setup
assert setup.index('prepareI2cResources();') < setup.index('setupPs4Controller();') < setup.index('setupWifiLogServer();')
prepare=function('prepareI2cResources')
assert 'Sensor' not in prepare and 'initFOC' not in prepare and 'beginTransmission' not in prepare
assert 'setBufferSize' not in prepare and '.end()' not in prepare
assert 'sensor1.beginPreparedBus();' in function('initializeRobotHardware')
assert 'sensor2.beginPreparedBus();' in function('initializeRobotHardware')
assert 'sensor1.begin(' not in function('initializeRobotHardware')
assert 'sensor2.begin(' not in function('initializeRobotHardware')
assert 'after_both_wheels_initialized' in function('initializeRobotHardware')
hardware = function('initializeRobotHardware')
# Three stopped exits are deliberate: require calibration, IMU calibration failure, save failure.
assert hardware.count('return;') == hardware.count('reportStartupFailure(startup_stage);') + hardware.count('if (!startupHardwareStepAllowed()) return;') + 3
assert hardware.index('if(!calibration_loaded&&!calibration_active.load())') < hardware.index('driver1.init()')
assert hardware.index('saveWheelCalibration()') < hardware.index('test_system_ready = true')
assert 'startup_foc_ready.store(true)' not in hardware
assert function('advanceStartup').index('initializeRobotHardware();') < function('advanceStartup').index('startup_foc_ready.store(true)')
assert 'MDNS.begin' not in source and 'ESPmDNS.h' not in source
assert '.local/' not in source
loop = function('loop')
assert loop.index('updatePs4Controller();') < loop.index('advanceStartup();')
assert loop.index('serviceHttpRequests();') < loop.index('advanceStartup();')
assert 'PS4' not in function('advanceStartup') # No PS4-connection prerequisite.

# Waiting/retrying remains outside the control loop and stop handlers.
update = function('updateWifiLogServer')
assert 'while (' not in update and 'delay(' not in update
# HTTP observes queued copies and atomics, never controller objects or I2C.
for name in ['httpTask', 'handleWifiStatusPage', 'serviceHttpPs4DiagnosticSerial']:
    text = function(name)
    assert 'initializeRobotHardware(' not in text
    assert 'ps4_diagnostics.' not in text
    for hardware_access in ['I2Cone', 'I2Ctwo', 'sensor1.', 'sensor2.', 'motor1.', 'motor2.', 'mpu6050.']:
        assert hardware_access not in text, (name, hardware_access)
for hardware_read in ['getAngle(', 'getVelocity(', 'beginTransmission(', 'requestFrom(', 'loopFOC(', 'initFOC(']:
    assert hardware_read not in function('captureHttpStatus')

# Memory/failure reporting must still work when Print::printf cannot malloc.
for name in ['printHttpStartupMemory', 'reportStartupFailure', 'prepareI2cResources']:
    assert 'Serial.printf' not in function(name)
    assert 'String(' not in function(name)
    assert 'snprintf(' in function(name)

assert '#define HTTP_MEMORY_STAGE_DIAGNOSTICS 0' in source

# Pending asset responses must not gate monitoring/diagnostic task services.
http_task=function('httpTask')
assert 'serviceHttpPs4DiagnosticSerial();' in http_task
assert 'serviceHttpFileResponse();' in http_task
assert http_task.index('updateWifiLogServer();') < http_task.index('serviceHttpFileResponse();')
assert 'if(!http_response_pending)wifi_log_server.handleClient();' in function('updateWifiLogServer')
assert 'http_response_pending' not in function('serviceHttpPs4DiagnosticSerial')

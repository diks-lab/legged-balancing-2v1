"""Exercise production HTTP resource setup with failing/successful RTOS doubles."""
from pathlib import Path
import re
import subprocess
import tempfile

source = Path('src/main.cpp').read_text()
def function(name):
    match = re.search(r'^void ' + name + r'\([^;]*?\)\s*\{', source, re.M)
    assert match, name
    pos, depth = match.end(), 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[match.start():pos]

code = r'''
#include <cassert>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>
using QueueHandle_t = int*;
using TaskHandle_t = void*;
using StackType_t = uint8_t;
struct StaticTask_t { uint8_t storage[64]; };
constexpr bool kEnableWifiLogServer = true;
constexpr uint32_t kHttpTaskStackBytes = 8192;
uint8_t http_task_stack[kHttpTaskStackBytes]; StaticTask_t http_task_tcb;
TaskHandle_t http_task_handle = nullptr;
std::atomic<bool> http_task_ready{false};
struct HttpStatus { char text[1536]; };
char http_trace_snapshot[20480];
QueueHandle_t http_status_queue = nullptr, http_command_queue = nullptr;
bool http_filesystem_ready = false;
bool fail_task = false, fail_mount = false;
int fail_queue = 0, queues = 0, deletes = 0, creates = 0, wifi_calls = 0;
std::vector<std::string> stages;
constexpr uint32_t MALLOC_CAP_INTERNAL=1, MALLOC_CAP_8BIT=2;
struct { unsigned getFreeHeap(){return 20000;} } ESP;
unsigned heap_caps_get_free_size(uint32_t caps){assert(caps==3);return 18000;}
unsigned heap_caps_get_largest_free_block(uint32_t caps){assert(caps==3);return 6000;}
unsigned uxTaskGetStackHighWaterMark(TaskHandle_t){return 4000;}
struct {
  std::string text;
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
constexpr int WIFI_STA=1;
constexpr char SSID_NAME[]="test",SSID_KEY[]="not-a-real-key";
struct {
  void mode(int){++wifi_calls;stages.push_back("wifi");assert(!http_task_ready.load());}
  void setAutoReconnect(bool){}
  void begin(const char*,const char*){assert(http_task_handle);}
} WiFi;
'''
code += '\n'.join(function(n) for n in
                  ['formatHttpResources', 'printHttpStartupMemory', 'setupWifiLogServer'])
code += r'''
void reset(){
  if(http_status_queue)delete http_status_queue;
  if(http_command_queue)delete http_command_queue;
  http_status_queue=http_command_queue=nullptr;http_task_handle=nullptr;
  http_task_ready.store(false);fail_task=fail_mount=false;
  fail_queue=queues=deletes=creates=wifi_calls=0;stages.clear();Serial.text.clear();
}
int main(){
  // A 6 KiB largest heap block is smaller than the reserved 8 KiB stack.
  setupWifiLogServer();assert(http_task_handle && http_task_ready.load());
  assert((stages==std::vector<std::string>{"task","fs","wifi"}));
  assert(Serial.text.find("internal_largest=6000")!=std::string::npos);
  assert(Serial.text.find("api=static result=OK")!=std::string::npos);
  setupWifiLogServer();assert(creates==1);
  char resources[280];formatHttpResources(resources,sizeof(resources));
  assert(std::string(resources).find("stack_free_min_bytes=4000")!=std::string::npos);
  for(int failed=1;failed<=2;++failed){
    reset();fail_queue=failed;setupWifiLogServer();
    assert(!http_task_ready && !http_status_queue && !http_command_queue);
    assert(deletes==1 && creates==0 && wifi_calls==0);
  }
  reset();fail_task=true;setupWifiLogServer();
  assert(!http_task_ready && !http_status_queue && !http_command_queue);
  assert(deletes==2 && wifi_calls==0);
  assert(Serial.text.find("api=static result=NULL")!=std::string::npos);
  reset();fail_mount=true;setupWifiLogServer();
  assert(http_task_ready && !http_filesystem_ready && wifi_calls==1);
  reset();puts("PASS: HTTP static stack, startup ordering, diagnostics, failure cleanup");
}
'''
with tempfile.TemporaryDirectory(prefix='http-startup-') as directory:
    cpp = Path(directory) / 'test.cpp'
    binary = Path(directory) / 'test'
    cpp.write_text(code)
    subprocess.run(['c++', '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)

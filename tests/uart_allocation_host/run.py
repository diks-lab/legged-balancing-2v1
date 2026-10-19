"""Production allocation observer/console setup, with reboot and OOM injection."""
from pathlib import Path
import os,subprocess,tempfile
source=Path('src/UartAllocationDiagnostics.cpp').read_text()
# The only target-specific instruction: CPU PS register read.
source=source.replace('asm volatile("rsr.ps %0":"=a"(r.ps));','r.ps=mockPs;')
assert 'asm volatile' not in source
common=r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstdio>
#include <cstdarg>
#include <cassert>
#include <cstring>
#include <string>
#include <new>
#include <atomic>
#include <thread>
#include <unistd.h>
#define IRAM_ATTR
#define RTC_NOINIT_ATTR
using esp_err_t=int;
enum esp_reset_reason_t {ESP_RST_UNKNOWN,ESP_RST_POWERON,ESP_RST_EXT,ESP_RST_SW,ESP_RST_PANIC,ESP_RST_INT_WDT,ESP_RST_TASK_WDT,ESP_RST_WDT,ESP_RST_DEEPSLEEP,ESP_RST_BROWNOUT};
constexpr int MALLOC_CAP_INTERNAL=0x800,MALLOC_CAP_8BIT=4,ESP_LOG_ERROR=1;
thread_local unsigned mockCore=0;thread_local bool inCallback=false;
unsigned mockPs=0,mockCycles=999,mockMs=100;
bool mockIsr=false;
esp_reset_reason_t resetReason=ESP_RST_POWERON;
int consoleWrites=0,logWrites=0,flushes=0;bool wireless=false;
using esp_alloc_failed_hook_t=void(*)(size_t,uint32_t,const char*);
esp_alloc_failed_hook_t hook=nullptr;
void* operator new(size_t n){assert(!inCallback);if(void*p=malloc(n))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{free(p);}void operator delete(void*p,size_t)noexcept{free(p);}
extern "C" void* __real_malloc(size_t);extern "C" void* __real_calloc(size_t,size_t);extern "C" void* __real_realloc(void*,size_t);
extern "C" void* __wrap_malloc(size_t n){assert(!inCallback);return __real_malloc(n);}
extern "C" void* __wrap_calloc(size_t n,size_t z){assert(!inCallback);return __real_calloc(n,z);}
extern "C" void* __wrap_realloc(void*p,size_t n){assert(!inCallback);return __real_realloc(p,n);}
struct {std::string text;void printf(const char*fmt,...){assert(!inCallback);char b[512];va_list a;va_start(a,fmt);vsnprintf(b,sizeof b,fmt,a);va_end(a);text+=b;}}Serial;
esp_reset_reason_t esp_reset_reason(){assert(!inCallback);return resetReason;}
int heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t f){assert(!inCallback);hook=f;return 0;}
unsigned heap_caps_get_free_size(unsigned){assert(!inCallback);return 9000;}
unsigned heap_caps_get_largest_free_block(unsigned){assert(!inCallback);return 7000;}
unsigned millis(){assert(!inCallback);return mockMs;}
unsigned xPortGetCoreID(){return mockCore;}
bool xPortInIsrContext(){return mockIsr;}
unsigned esp_cpu_get_cycle_count(){return mockCycles;}
ssize_t mockWrite(int fd,const void*,size_t n){assert(!inCallback&&!wireless&&fd==STDOUT_FILENO);++consoleWrites;return n;}
void esp_log_write(int,const char*,const char*){assert(!inCallback&&!wireless&&hook);++logWrites;}
int mockFlush(FILE*){assert(!inCallback&&!wireless);++flushes;return 0;}
int mockError(FILE*){return 0;}
#define write mockWrite
#define fflush mockFlush
#define ferror mockError
'''
checks=r'''
void fail(unsigned core=0,unsigned size=84,unsigned caps=0x804){mockCore=core;inCallback=true;hook(size,caps,reinterpret_cast<const char*>(1));inCallback=false;}
void reset(esp_reset_reason_t reason){wireless=false;resetReason=reason;Serial.text.clear();initializeUartAllocationDiagnostics();}
int main(){
 static_assert(sizeof(AllocationFailureRtc)==124);
 reset(ESP_RST_POWERON);assert(hook&&consoleWrites==1&&logWrites==1&&flushes==1);
 assert(Serial.text.find("previous_session=invalid")!=std::string::npos);
 // ISR/task callbacks never output, allocate, walk heaps, or wait. First per core wins.
 sampleAllocationDiagnosticHeap(200,1234,987);wireless=true;auto text=Serial.text;
 fail();assert(Serial.text==text);auto first=allocation_failure_rtc.records[0];
 assert(allocationRecordValid(first,allocation_boot,0)&&first.requested==84&&first.caps==0x804);
 assert(first.sampleValid&&first.sampleMs==200&&first.sampleFree==1234&&first.sampleLargest==987);
 fail(0,4112,0x1000);assert(allocation_failure_rtc.records[0].requested==84);
 mockIsr=true;mockPs=3;fail(1,4112,0x1000);mockIsr=false;mockPs=0;
 assert(allocation_failure_rtc.records[1].isr==1&&allocation_failure_rtc.records[1].ps==3);
 reset(ESP_RST_PANIC);
 assert(Serial.text.find("requested=84 caps=0x804")!=std::string::npos);
 assert(Serial.text.find("requested=4112 caps=0x1000")!=std::string::npos);
 assert(Serial.text.find("preceding_sample valid=1 sample_ms=200")!=std::string::npos);
 assert(Serial.text.find("not measured at failure")!=std::string::npos);
 assert(Serial.text.find("cause not proven")!=std::string::npos);
 reset(ESP_RST_PANIC);assert(Serial.text.find("requested=")==std::string::npos); // Consumed.
 // Every non-crash reset discards otherwise valid data; never replays it later.
 for(auto reason:{ESP_RST_POWERON,ESP_RST_EXT,ESP_RST_SW,ESP_RST_DEEPSLEEP,ESP_RST_BROWNOUT,ESP_RST_UNKNOWN}){
  fail();reset(reason);assert(Serial.text.find("requested=")==std::string::npos);
  assert(Serial.text.find("discarded core=0")!=std::string::npos);
  reset(ESP_RST_PANIC);assert(Serial.text.find("requested=")==std::string::npos);
 }
 for(auto reason:{ESP_RST_INT_WDT,ESP_RST_TASK_WDT,ESP_RST_WDT}){fail();reset(reason);assert(Serial.text.find("requested=84")!=std::string::npos);}
 // Incomplete marker, bad checksum/version/session/core and header are rejected.
 for(unsigned kind=0;kind<7;++kind){
  fail();auto& r=allocation_failure_rtc.records[0];
  if(kind==0)r.marker=0;if(kind==1)++r.checksum;if(kind==2)++r.version;
  if(kind==3)++r.boot;if(kind==4)r.core=1;if(kind==5)++allocation_failure_rtc.check;
  if(kind==6)allocation_failure_rtc.marker=0;
  reset(ESP_RST_PANIC);assert(Serial.text.find("requested=")==std::string::npos);
 }
 // A callback sees an interrupted sample once, marks it invalid, and never retries.
 allocation_sample_seq.fetch_add(1);fail();assert(!allocation_failure_rtc.records[0].sampleValid);
 assert(allocation_failure_rtc.records[0].sampleFree==0);allocation_sample_seq.fetch_add(1);
 reset(ESP_RST_PANIC);
 // Independent per-core slots; same-core duplicate calls never overwrite first failure.
 std::thread a([]{for(int i=0;i<1000;++i)fail(0);});std::thread b([]{for(int i=0;i<1000;++i)fail(1,4112,0x1000);});a.join();b.join();
 assert(allocationRecordValid(allocation_failure_rtc.records[0],allocation_boot,0));
 assert(allocationRecordValid(allocation_failure_rtc.records[1],allocation_boot,1));
 reset(ESP_RST_PANIC);
 allocation_sample_seq=0;fail();assert(!allocation_failure_rtc.records[0].sampleValid);reset(ESP_RST_PANIC);
 // Generation wrap remains nonzero. Record belongs only to immediately prior boot.
 allocation_failure_rtc.boot=UINT32_MAX;allocation_failure_rtc.check=UINT32_MAX^kAllocRtcMagic;
 reset(ESP_RST_PANIC);assert(allocation_boot==1);
 puts("PASS: actual observer/preflight, no callback allocation/output/wait/heap walk, per-core first OOM, concurrent slots, checksums/session/reset gating/consume, interrupted samples, exact request vs preceding heap, public UART/SDK log before wireless");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'common.h').write_text(common)
 for h in ['Arduino.h','esp_attr.h','esp_heap_caps.h','esp_system.h','esp_log.h','esp_cpu.h','freertos/FreeRTOS.h','freertos/task.h']:
  f=p/h;f.parent.mkdir(exist_ok=True,parents=True);f.write_text('#include "common.h"\n')
 cpp=p/'test.cpp';cpp.write_text(source+'\n'+checks);exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('UART_ALLOCATION_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I'+str(p),'-Iinclude','-pthread',str(cpp),'-o',str(exe),'-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc']+flags,check=True)
 subprocess.run([str(exe)],check=True)
main=Path('src/main.cpp').read_text()
setup=main[main.index('void setup()'):main.index('void prepareI2cResources() {')]
assert setup.index('initializeUartAllocationDiagnostics();') < setup.index('setupPs4Controller();') < setup.index('setupWifiLogServer();')
callback=source[source.index('void IRAM_ATTR allocationFailed'):source.index('\nbool diagnosticReset')]
for forbidden in ['Serial.','printf(','malloc(','delay(','yield(','portENTER','while(','heap_caps_get_']:
 assert forbidden not in callback,forbidden

"""Actual fixed recorder, callback hook, HTTP counters and bounded output service."""
from pathlib import Path
import re,subprocess,tempfile,os
source=Path('src/main.cpp').read_text()
def fn(name):
 m=re.search(r'^(?:extern "C" )?void '+name+r'\([^;]*?\)\s*\{',source,re.M);assert m,name
 i=m.end();depth=1
 while depth:depth+=(source[i]=='{')-(source[i]=='}');i+=1
 return source[m.start():i]
prefix=r'''
#include "ReleaseMemoryObservation.h"
#include "AllocationResponseObserver.h"
int allocationBegins=0,allocationEnds=0;
void beginAllocationResponse(){++allocationBegins;}
AllocationResponseSummary finishAllocationResponse(){++allocationEnds;return {};}
#include <cassert>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>
#include <atomic>
using portMUX_TYPE=std::mutex;
#define portENTER_CRITICAL(x) ((x)->lock())
#define portEXIT_CRITICAL(x) ((x)->unlock())
void sampleAllocationDiagnosticHeap(uint32_t,uint32_t,uint32_t){}
ReleaseMemoryObserver release_memory_observer;
bool release_memory_serial_active=false;
bool first_http_request=false;
bool http_response_pending=false;
uint16_t first_http_route=0;
size_t ps4_diagnostic_serial_offset=0,ps4_diagnostic_serial_size=0;
portMUX_TYPE release_memory_lock;
constexpr int MALLOC_CAP_INTERNAL=1,MALLOC_CAP_8BIT=2;
uint32_t clock_ms=0;uint32_t millis(){return clock_ms;}
uint32_t heap_caps_get_free_size(int){return 9000;}
uint32_t heap_caps_get_largest_free_block(int){return 7000;}
struct {bool measuring=false;bool active(){return measuring;}}foc_alignment_observer;
struct {
 char output[16384]{};size_t used=0,max_write=0;int room=0;
 int availableForWrite(){return room;}
 size_t write(const uint8_t*p,size_t n){assert(n<=48&&n<=size_t(room));if(n>max_write)max_write=n;assert(used+n<sizeof(output));memcpy(output+used,p,n);used+=n;return n;}
} Serial;
'''
a=source.index('struct HttpMemoryRequest {');b=source.index('\n}\nextern "C"',a)
code=prefix+fn('finishHttpAllocationObservation')+'\n'+source[a:b]+'\n'+fn('observeHttpRequestBegin')+'\n'+fn('observeHttpRequestEnd')+'\n'+fn('observeReleaseMemory')+'\n'+fn('serviceReleaseMemoryObservation')+r'''
int main(){
 static_assert(sizeof(ReleaseMemoryObserver)<768,"bounded observer");
 ReleaseMemoryObserver local;ReleaseMemoryRecord r{};
 for(unsigned i=0;i<40;++i)local.push({i,8000-i,6000,MemHidBefore,uint16_t(i),244});
 assert(local.count==32&&local.dropped==8);
 for(unsigned i=0;i<32;++i){assert(local.pop(r));assert(r.ms==i&&r.cid==i);}
 assert(!local.pop(r));
 for(unsigned i=0;i<70;++i){local.push({i,1,1,MemConnected,0,0});assert(local.pop(r)&&r.ms==i);}
 local.push({UINT32_MAX-50,1,1,MemDisconnected,0,0});uint32_t delay=0;
 assert(!local.due(48,delay));assert(local.due(49,delay)&&delay==100);
 assert(local.due(449,delay)&&delay==500);assert(local.due(1949,delay)&&delay==2000);
 assert(local.due(9949,delay)&&delay==10000&&!local.due(20000,delay));
 local.push({30000,1,1,MemDisconnected,0,0});local.push({30001,1,1,MemConnectStart,10,17});
 assert(!local.due(50000,delay));
 // Parser allocation windows are separate from response/request handler counters.
 observeHttpRequestBegin();clock_ms=200;observeHttpRequestEnd(2,19,200);
 assert(allocationBegins==1&&allocationEnds==1&&release_memory_observer.requests==0);
 assert(release_memory_observer.pop(r)&&r.event==MemHttpRequestEnd&&r.arg==2&&r.free==19&&r.largest==200);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore0&&r.cid==0);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore1&&r.cid==0);
 allocationBegins=allocationEnds=0;
 // HTTP request active state and failed writes are counted without per-request records.
 {HttpMemoryRequest request(1);assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpFirstRoute&&r.cid==1);assert(release_memory_observer.requests==1&&release_memory_observer.active==1);
  observeHttpStatusSend(true);observeHttpStatusSend(false);
  observeReleaseMemory(MemHidBefore,10,244);assert(release_memory_observer.pop(r)&&(r.event>>8)==1);}
 assert(release_memory_observer.active==0&&release_memory_observer.send_failures==1&&release_memory_observer.count==2);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore0);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore1);
 assert(allocationBegins==1&&allocationEnds==1);
 {HttpMemoryRequest pending(2);http_response_pending=true;}
 assert(allocationBegins==2&&allocationEnds==1&&release_memory_observer.active==1);
 finishHttpAllocationObservation(2);http_response_pending=false;release_memory_observer.httpEnd();
 assert(allocationEnds==2&&release_memory_observer.active==0);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore0&&r.cid==2);
 assert(release_memory_observer.pop(r)&&(r.event&255)==MemHttpAllocCore1&&r.cid==2);
 observeReleaseMemory(MemChannelReleased,42,9);assert(Serial.used==0);
 assert(release_memory_observer.pop(r)&&r.cid==42&&r.arg==9&&r.free==9000&&r.largest==7000);
 // A callback never outputs; overflow does not wait for UART or allocate.
 for(int i=0;i<40;++i)observeReleaseMemory(MemHidAfter,10,i);
 assert(Serial.used==0&&release_memory_observer.dropped==8);
 clock_ms=100;serviceReleaseMemoryObservation();assert(Serial.used==0); // backpressure
 Serial.room=20;
 for(int i=0;i<500;++i){clock_ms+=10;serviceReleaseMemoryObservation();}
 assert(Serial.max_write<=20&&strstr(Serial.output,"MEM event=")&&strstr(Serial.output,"lost=8"));
 for(int i=0;i<500;++i){clock_ms+=10;serviceReleaseMemoryObservation();}
 assert(strstr(Serial.output,"MEM HTTP")&&strstr(Serial.output,"requests=2 active=0 send_failed=1"));
 // Terminal summaries decode 128/64 counters and timing/progress without heap labels.
 release_memory_observer.push({clock_ms,500,11,MemHttpTxEnd,1,11u|(3u<<8)|(128u<<16)|(64u<<24)});
 release_memory_observer.push({clock_ms,101,30,MemHttpTxTiming,1,200});
 release_memory_observer.push({clock_ms,1492,0x1000,MemHttpAllocCore0,1,2u|kAllocationDropped});
 release_memory_observer.push({clock_ms,0,0,MemHttpAllocCore1,1,kAllocationUnavailable|kAllocationSaturated});
 release_memory_observer.push({clock_ms,10,450,MemHttpTxProgress,1,50});
 for(int i=0;i<300;++i){clock_ms+=10;serviceReleaseMemoryObservation();}
 assert(strstr(Serial.output,"reason=11 phase=3 elapsed=500 calls=128 retry=64 errno=11"));
 assert(strstr(Serial.output,"polls=101 max_gap_ms=30 retry_wait_ms=200"));
 assert(strstr(Serial.output,"first_ms=10 last_ms=450 idle_ms=50"));
 assert(strstr(Serial.output,"core=0 count=2 first_size=1492 first_caps=0x1000 saturated=0 unavailable=0 dropped=1"));
 assert(strstr(Serial.output,"core=1 count=0 first_size=0 first_caps=0x0 saturated=1 unavailable=1 dropped=0"));
 const size_t before=Serial.used;foc_alignment_observer.measuring=true;
 observeReleaseMemory(MemHidBefore,10,244);clock_ms+=10000;serviceReleaseMemoryObservation();assert(Serial.used==before);
 // The shared recorder is exercised with the same caller-side lock discipline.
 std::atomic<bool> done=false;std::thread producer([&]{for(int i=0;i<10000;++i)observeReleaseMemory(MemHidBefore,10,i);done=true;});
 unsigned read=0;while(!done){std::lock_guard<std::mutex> lock(release_memory_lock);if(release_memory_observer.pop(r))++read;}
 producer.join();assert(read+release_memory_observer.count+release_memory_observer.dropped>=10000);
 printf("PASS: fixed ring/drop accounting, HTTP aggregates, callback no-output, delayed disconnect/rollover/reconnect, UART backpressure/FOC defer, concurrent store; model=%zu B\n",sizeof(ReleaseMemoryObserver));
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);cpp=p/'test.cpp';cpp.write_text(code);exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('MEMORY_OBSERVATION_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude','-pthread',str(cpp),'-o',str(exe)]+flags,check=True)
 subprocess.run([str(exe)],check=True)

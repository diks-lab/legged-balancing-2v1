#include <Arduino.h>
#include <esp_attr.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_log.h>
#include <esp_cpu.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <atomic>
#include <stdio.h>
#include <unistd.h>
#include "UartAllocationDiagnostics.h"
#define ALLOC_RECORD_IRAM IRAM_ATTR
#include "AllocationFailureRecord.h"

namespace {
RTC_NOINIT_ATTR AllocationFailureRtc allocation_failure_rtc;
std::atomic<uint32_t> allocation_claimed[2]{};
// One producer (setup before HTTP creation, then HTTP only). Callback reads once,
// never spins on an in-progress sample. Atomic fields avoid data races.
std::atomic<uint32_t> allocation_sample_seq{0},allocation_sample_ms{0};
std::atomic<uint32_t> allocation_sample_free{0},allocation_sample_largest{0};
uint32_t allocation_boot=0;
AllocationResponseObserver allocation_response_observer;
static_assert(ATOMIC_INT_LOCK_FREE==2,"failure callback requires lock-free word atomics");

void IRAM_ATTR allocationFailed(size_t size,uint32_t caps,const char* function){
  (void)function; // Do not dereference flash strings or preserve pointers in RTC.
  const unsigned core=xPortGetCoreID();
  if(core>=2)return;
  allocation_response_observer.record(core,uint32_t(size),caps);
  uint32_t expected=0;
  if(!allocation_claimed[core].compare_exchange_strong(expected,1,std::memory_order_relaxed))return;
  AllocationFailureRecord r{};
  r.version=kAllocRecordVersion;r.boot=allocation_boot;r.requested=uint32_t(size);r.caps=caps;
  r.cycles=esp_cpu_get_cycle_count();r.core=core;r.isr=xPortInIsrContext()?1:0;
  asm volatile("rsr.ps %0":"=a"(r.ps));
  const uint32_t before=allocation_sample_seq.load(std::memory_order_acquire);
  r.sampleMs=allocation_sample_ms.load(std::memory_order_relaxed);
  r.sampleFree=allocation_sample_free.load(std::memory_order_relaxed);
  r.sampleLargest=allocation_sample_largest.load(std::memory_order_relaxed);
  const uint32_t after=allocation_sample_seq.load(std::memory_order_acquire);
  r.sampleValid=before && !(before&1) && before==after;
  if(!r.sampleValid)r.sampleMs=r.sampleFree=r.sampleLargest=0;
  r.checksum=allocationRecordChecksum(r);
  // Only this core's first callback owns this slot. Publish marker LAST.
  auto& out=allocation_failure_rtc.records[core];
  out.marker=0;out.version=r.version;out.boot=r.boot;out.requested=r.requested;out.caps=r.caps;
  out.cycles=r.cycles;out.core=r.core;out.isr=r.isr;out.ps=r.ps;out.sampleValid=r.sampleValid;
  out.sampleMs=r.sampleMs;out.sampleFree=r.sampleFree;out.sampleLargest=r.sampleLargest;
  out.checksum=r.checksum;
  __atomic_store_n(&out.marker,kAllocRecordMagic,__ATOMIC_RELEASE);
}

bool diagnosticReset(esp_reset_reason_t reason){
  return reason==ESP_RST_PANIC || reason==ESP_RST_INT_WDT ||
      reason==ESP_RST_TASK_WDT || reason==ESP_RST_WDT;
}
}

void beginAllocationResponse(){allocation_response_observer.begin();}
AllocationResponseSummary finishAllocationResponse(){return allocation_response_observer.finish();}

void sampleAllocationDiagnosticHeap(uint32_t ms,uint32_t free,uint32_t largest){
  allocation_sample_seq.fetch_add(1,std::memory_order_acq_rel);
  allocation_sample_ms.store(ms,std::memory_order_relaxed);
  allocation_sample_free.store(free,std::memory_order_relaxed);
  allocation_sample_largest.store(largest,std::memory_order_relaxed);
  allocation_sample_seq.fetch_add(1,std::memory_order_release);
}

void initializeUartAllocationDiagnostics(){
  const auto reason=esp_reset_reason();
  const bool valid=allocationRtcValid(allocation_failure_rtc);
  const uint32_t oldBoot=valid?allocation_failure_rtc.boot:0;
  // Invalidate the old session before copying/consuming any slot.
  // A reset midway through consumption must not replay a remaining old slot.
  __atomic_store_n(&allocation_failure_rtc.marker,0,__ATOMIC_SEQ_CST);
  // Consume before any output: a reset during reporting cannot replay old slots.
  AllocationFailureRecord previous[2];
  for(unsigned i=0;i<2;++i){previous[i]=allocation_failure_rtc.records[i];allocation_failure_rtc.records[i].marker=0;}
  allocation_boot=oldBoot+1;if(!allocation_boot)allocation_boot=1;
  allocation_failure_rtc.boot=allocation_boot;
  allocation_failure_rtc.check=allocation_boot^kAllocRtcMagic;
  __atomic_store_n(&allocation_failure_rtc.marker,kAllocRtcMagic,__ATOMIC_RELEASE);
  for(auto& claimed:allocation_claimed)claimed.store(0,std::memory_order_relaxed);
  const auto hook=heap_caps_register_failed_alloc_callback(allocationFailed);
  Serial.printf("ALLOC RTC reset=%u previous_session=%s hook=%d\n",unsigned(reason),
      valid?"valid":"invalid",int(hook));
  for(unsigned i=0;i<2;++i){
    if(!valid || !allocationRecordValid(previous[i],oldBoot,i))continue;
    if(!diagnosticReset(reason)){
      Serial.printf("ALLOC RTC discarded core=%u reset=%u (not a crash reset)\n",i,unsigned(reason));continue;
    }
    const auto& r=previous[i];
    Serial.printf("ALLOC previous_boot=%lu reset=%u requested=%lu caps=0x%lx core=%lu isr=%lu ps=0x%lx cycles=%lu (observed before reset; cause not proven)\n",
        (unsigned long)oldBoot,unsigned(reason),(unsigned long)r.requested,(unsigned long)r.caps,
        (unsigned long)r.core,(unsigned long)r.isr,(unsigned long)r.ps,(unsigned long)r.cycles);
    Serial.printf("ALLOC preceding_sample valid=%lu sample_ms=%lu internal_free=%lu internal_largest=%lu (not measured at failure)\n",
        (unsigned long)r.sampleValid,(unsigned long)r.sampleMs,(unsigned long)r.sampleFree,(unsigned long)r.sampleLargest);
  }
  constexpr uint32_t caps=MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT;
  sampleAllocationDiagnosticHeap(millis(),heap_caps_get_free_size(caps),heap_caps_get_largest_free_block(caps));
  // Public APIs only; Serial uses a different UART path and does not prime VFS.
  static const char ready[]="SDK UART VFS preflight\n";
  const ssize_t sent=write(STDOUT_FILENO,ready,sizeof(ready)-1);
  esp_log_write(ESP_LOG_ERROR,"startup","SDK UART log preflight\n");
  const int flushed=fflush(stdout);
  Serial.printf("SDK UART preflight write=%d/%u flush=%d stream_error=%d (before wireless)\n",
      int(sent),unsigned(sizeof(ready)-1),flushed,ferror(stdout));
  sampleAllocationDiagnosticHeap(millis(),heap_caps_get_free_size(caps),heap_caps_get_largest_free_block(caps));
}

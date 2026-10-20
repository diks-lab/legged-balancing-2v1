#pragma once
#include <atomic>
#include <stdint.h>
struct AllocationResponseCore {
  uint32_t countFlags=0,firstSize=0,firstCaps=0;
};
struct AllocationResponseSummary { AllocationResponseCore core[2]; };
constexpr uint32_t kAllocationCountMask=0x1fffffff;
constexpr uint32_t kAllocationSaturated=1u<<29;
constexpr uint32_t kAllocationUnavailable=1u<<30;
constexpr uint32_t kAllocationDropped=1u<<31;
// HTTP owns begin/finish. Callbacks on either core TRY a slot once, never wait.
// Attribution is the active window when the callback acquires its slot; it is
// system-wide temporal correlation, NOT proof that HTTP/TCP caused the failure.
class AllocationResponseObserver {
  struct Slot {
    std::atomic<uint32_t> guard{0},dropped{0};
    uint32_t epoch=0,countFlags=0,firstSize=0,firstCaps=0;
  } slots_[2];
  std::atomic<uint32_t> active_{0};
  uint32_t next_=0;
 public:
  static_assert(ATOMIC_INT_LOCK_FREE==2,"callback needs lock-free word atomics");
  void begin(){
    active_.store(0,std::memory_order_release);
    if(!++next_)++next_;
    for(auto& slot:slots_){
      slot.dropped.store(0,std::memory_order_release);uint32_t expected=0;
      if(slot.guard.compare_exchange_strong(expected,1,std::memory_order_acquire)){
        slot.epoch=next_;slot.countFlags=slot.firstSize=slot.firstCaps=0;
        slot.guard.store(0,std::memory_order_release);
      }else slot.dropped.store(1,std::memory_order_release);
    }
    active_.store(next_,std::memory_order_release);
  }
  __attribute__((always_inline)) void record(unsigned core,uint32_t size,uint32_t caps){
    if(core>=2)return;
    auto& slot=slots_[core];uint32_t expected=0;
    if(!slot.guard.compare_exchange_strong(expected,1,std::memory_order_acquire)){
      if(active_.load(std::memory_order_acquire))slot.dropped.store(1,std::memory_order_release);
      return;
    }
    const uint32_t epoch=active_.load(std::memory_order_acquire);
    if(epoch){
      if(slot.epoch!=epoch){slot.epoch=epoch;slot.countFlags=slot.firstSize=slot.firstCaps=0;}
      if(!(slot.countFlags&kAllocationCountMask)){slot.firstSize=size;slot.firstCaps=caps;}
      if((slot.countFlags&kAllocationCountMask)==kAllocationCountMask)slot.countFlags|=kAllocationSaturated;
      else ++slot.countFlags;
    }
    slot.guard.store(0,std::memory_order_release);
  }
  AllocationResponseSummary finish(){
    AllocationResponseSummary result;
    const uint32_t epoch=active_.exchange(0,std::memory_order_acq_rel);
    if(!epoch)return result;
    for(unsigned i=0;i<2;++i){
      auto& slot=slots_[i];uint32_t expected=0;
      if(slot.guard.compare_exchange_strong(expected,1,std::memory_order_acquire)){
        if(slot.epoch==epoch)result.core[i]={slot.countFlags,slot.firstSize,slot.firstCaps};
        slot.guard.store(0,std::memory_order_release);
      }else result.core[i].countFlags=kAllocationUnavailable;
      if(slot.dropped.load(std::memory_order_acquire))result.core[i].countFlags|=kAllocationDropped;
    }
    return result;
  }
};
static_assert(sizeof(AllocationResponseObserver)==56,"bounded live allocation observer");

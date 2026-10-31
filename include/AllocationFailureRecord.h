#pragma once
#include <stdint.h>
#ifndef ALLOC_RECORD_IRAM
#define ALLOC_RECORD_IRAM
#endif
// Plain words only: RTC_NOINIT storage has no constructors or saved pointers.
struct AllocationFailureRecord {
  uint32_t marker,version,boot,requested,caps,cycles,core,isr,ps;
  uint32_t sampleValid,sampleMs,sampleFree,sampleLargest,checksum;
};
struct AllocationFailureRtc {
  uint32_t marker,boot,check;
  AllocationFailureRecord records[2];
};
constexpr uint32_t kAllocRtcMagic=0x414c4c32,kAllocRecordMagic=0x46414932;
constexpr uint32_t kAllocRecordVersion=1;
static inline __attribute__((always_inline)) ALLOC_RECORD_IRAM uint32_t allocationRecordChecksum(const AllocationFailureRecord& r){
  const uint32_t words[]={r.version,r.boot,r.requested,r.caps,r.cycles,r.core,r.isr,r.ps,
      r.sampleValid,r.sampleMs,r.sampleFree,r.sampleLargest};
  uint32_t h=2166136261u;
  for(uint32_t w:words)h=(h^w)*16777619u;
  return h;
}
inline bool allocationRecordValid(const AllocationFailureRecord& r,uint32_t boot,unsigned core){
  return r.marker==kAllocRecordMagic && r.version==kAllocRecordVersion && r.boot==boot &&
      r.core==core && r.isr<=1 && r.sampleValid<=1 && r.checksum==allocationRecordChecksum(r);
}
inline bool allocationRtcValid(const AllocationFailureRtc& r){
  return r.marker==kAllocRtcMagic && r.boot!=0 && r.check==(r.boot^kAllocRtcMagic);
}
static_assert(sizeof(AllocationFailureRecord)==56,"fixed RTC allocation record");
static_assert(sizeof(AllocationFailureRtc)==124,"fixed RTC allocation storage");

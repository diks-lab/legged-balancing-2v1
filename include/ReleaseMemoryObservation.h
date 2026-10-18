#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum ReleaseMemoryEvent {
  MemPs4Before=1, MemPs4Ready, MemConnectStart, MemConfigReady, MemConnected,
  MemDisconnectNotice, MemDisconnectAck, MemChannelReleased, MemDisconnected,
  MemHidBefore, MemHidAfter, MemHidAllocationFailed, MemAfterDisconnect,
  MemHttpFirstRoute, MemHttpFirstFile, MemHttpFirstResult
};
void observeReleaseMemory(uint16_t event, uint16_t cid, uint32_t arg);
#ifdef __cplusplus
}
#include <stddef.h>
struct ReleaseMemoryRecord { uint32_t ms, free, largest; uint16_t event,cid; uint32_t arg; };
static_assert(sizeof(ReleaseMemoryRecord)==20,"fixed record");
// Caller serializes access. No allocation/output or controller changes here.
struct ReleaseMemoryObserver {
  ReleaseMemoryRecord records[32]{};
  uint32_t dropped=0, requests=0, active=0, send_failures=0;
  uint32_t after_free=0, after_largest=0, min_free=UINT32_MAX;
  uint32_t disconnect_ms=0, disconnect_next=4;
  uint8_t head=0,count=0;
  void push(ReleaseMemoryRecord r){
    if(r.event==MemDisconnected || r.event==MemHidAllocationFailed){disconnect_ms=r.ms;disconnect_next=0;}
    if(r.event==MemConnected || r.event==MemConnectStart) disconnect_next=4;
    if(count==32){++dropped;return;}
    // High event byte snapshots HTTP activity without growing the record.
    r.event=uint16_t(r.event | ((active>255?255:active)<<8));
    records[(head+count)%32]=r;++count;
  }
  bool pop(ReleaseMemoryRecord& r){if(!count)return false;r=records[head];head=(head+1)%32;--count;return true;}
  bool due(uint32_t now,uint32_t& delay){
    static constexpr uint32_t waits[]={100,500,2000,10000};
    if(disconnect_next>=4 || uint32_t(now-disconnect_ms)<waits[disconnect_next])return false;
    delay=waits[disconnect_next++];return true;
  }
  void httpBegin(){++requests;++active;}
  void httpEnd(){if(active)--active;}
  void httpAfter(uint32_t f,uint32_t l){after_free=f;after_largest=l;if(f<min_free)min_free=f;}
};
#endif

#pragma once
#include <stdint.h>
// Boot-only intent. No motor operations, allocation, timers or controller gains.
class BootAutoStart {
 public:
  enum class Phase : uint8_t { Initializing, WaitingPose, Preparing, Finished, Cancelled };
  Phase phase=Phase::Initializing;
  bool invoking=false;
  bool tick(bool ready,bool allowed,uint32_t now,uint32_t hold_ms){
    if(phase==Phase::Preparing||phase==Phase::Finished||phase==Phase::Cancelled)return false;
    if(!ready){phase=Phase::Initializing;holding=false;return false;}
    phase=Phase::WaitingPose;
    if(!allowed){holding=false;return false;}
    if(!holding){since=now;holding=true;return false;}
    if(uint32_t(now-since)<hold_ms)return false;
    phase=Phase::Preparing;holding=false;return true;
  }
  void cancel(){phase=Phase::Cancelled;holding=false;}
  void complete(){if(phase==Phase::Preparing)phase=Phase::Finished;}
  bool pending()const{return phase==Phase::Initializing||phase==Phase::WaitingPose||phase==Phase::Preparing;}
 private:
  uint32_t since=0;
  bool holding=false;
};

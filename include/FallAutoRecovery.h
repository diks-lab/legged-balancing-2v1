#pragma once
#include <cstdint>
// Only a confirmed fall may arm this intent. Other stops cancel it.
struct FallAutoRecovery {
  bool pending=false,holding=false;uint32_t since=0;
  const char* reason="NONE";
  void cancel(){pending=holding=false;reason="NONE";}
  void arm(){pending=true;holding=false;reason="UPRIGHT_REQUIRED";}
  bool tick(bool healthy,bool upright,uint32_t now){
    if(!pending)return false;
    if(!healthy){cancel();reason="HEALTH_FAULT";return false;}
    if(!upright){holding=false;reason="UPRIGHT_REQUIRED";return false;}
    reason="HOLD_1500_MS";
    if(!holding){holding=true;since=now;return false;}
    if(uint32_t(now-since)<1500)return false;
    pending=holding=false;reason="RECOVERED";return true;
  }
};

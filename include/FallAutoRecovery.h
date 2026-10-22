#pragma once
// Only a confirmed fall may arm this intent. No timer or stationary-speed gate.
struct FallAutoRecovery {
  bool pending=false;
  const char* reason="NONE";
  void cancel(){pending=false;reason="NONE";}
  void arm(){pending=true;reason="UPRIGHT_REQUIRED";}
  bool tick(bool healthy,bool upright){
    if(!pending)return false;
    if(!healthy){cancel();reason="HEALTH_FAULT";return false;}
    if(!upright){reason="UPRIGHT_REQUIRED";return false;}
    pending=false;reason="RECOVERED";return true;
  }
};

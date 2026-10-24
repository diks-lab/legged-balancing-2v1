#pragma once
#include <atomic>
#include <cstdint>

// Observation only: no bus access, allocation, logging or actuator calls.
enum class FocGateSource : uint8_t { Other, Timer, WebAbort, RightSensor, LeftSensor, Cleanup, LibraryDisable, Count };
enum class FocAlignPoint : uint8_t { ForwardBegin, ForwardEnd, BackwardEnd, ZeroBegin, ZeroEnd, Count };
struct FocAlignmentRecord {
  struct Gate { uint32_t count=0, first_us=0; } gates[static_cast<unsigned>(FocGateSource::Count)];
  uint32_t start_us=0, total_us=0, point_us[5]={}, valid=0;
  float angles[5]={}, moved=0;
  int raw_result=-1, timer_start=-1, timer_stop=-1;
  bool pp_observed=false, pp_pass=false, expired=false, sensors_ok=false, accepted=false;
  bool started=false, ended=false;
};
template<class Lock>
class FocAlignmentObservation {
 public:
  bool active() const { return active_slot_.load()>=0; }
  void begin(unsigned slot, const void* motor, uint32_t now) {
    Guard guard(lock_); records[slot]={};records[slot].started=true;
    records[slot].start_us=now; motor_=motor;active_slot_.store(static_cast<int>(slot));
  }
  void point(const void* motor,FocAlignPoint point,float angle,uint32_t now) {
    Guard guard(lock_);int slot=active_slot_.load();if(slot<0 || motor!=motor_)return;
    auto& r=records[slot];unsigned p=static_cast<unsigned>(point);
    r.point_us[p]=now-r.start_us;r.angles[p]=angle;r.valid|=1u<<p;
    if(point==FocAlignPoint::BackwardEnd){float d=r.angles[1]-angle;r.moved=d<0?-d:d;}
  }
  void pp(const void* motor,float moved,bool passed) {
    Guard guard(lock_);int slot=active_slot_.load();if(slot<0 || motor!=motor_)return;
    records[slot].moved=moved;records[slot].pp_pass=passed;records[slot].pp_observed=true;
  }
  void gate(FocGateSource source,uint32_t now) {
    Guard guard(lock_);int slot=active_slot_.load();if(slot<0)return;
    auto& r=records[slot];auto& g=r.gates[static_cast<unsigned>(source)];
    if(g.count++==0)g.first_us=now-r.start_us;
  }
  void end(int raw,int start,int stop,bool expired,bool pp,bool sensors,bool accepted,uint32_t now) {
    Guard guard(lock_);int slot=active_slot_.load();if(slot<0)return;
    auto& r=records[slot];r.raw_result=raw;r.timer_start=start;r.timer_stop=stop;
    r.expired=expired;r.pp_pass=pp;r.sensors_ok=sensors;r.accepted=accepted;
    r.total_us=now-r.start_us;r.ended=true;active_slot_.store(-1);motor_=nullptr;
  }
  // Records are read by the Arduino task after end; asynchronous gates use the lock.
  FocAlignmentRecord records[2]{};
 private:
  struct Guard { Lock& lock; explicit Guard(Lock& l):lock(l){lock.lock();} ~Guard(){lock.unlock();} };
  Lock lock_;
  std::atomic<int> active_slot_{-1};
  const void* motor_=nullptr;
};
// Hooks are defined by the application. Exact-source build patch inserts calls
// at the library's existing cached values; the hook never reads a sensor.
void observeFocAlignPoint(const void*,FocAlignPoint,float,uint32_t);
void observeFocAlignPp(const void*,float,bool);
void observeFocLibraryDisable(uint32_t);

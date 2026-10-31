#pragma once
#include "Ps4Diagnostics.h"
#include "Ps4ControlMapping.h"
#include <cstddef>

// Lock supplies short task-context critical sections (portMUX on ESP32).
// No allocation, waiting for space, overwrite of queued events, or hardware I/O.
template<class Lock, size_t Capacity = 64>
class Ps4EventInbox {
  static_assert(Capacity>0,"PS4 FIFO capacity must be nonzero");
 public:
  enum class Kind { Connect, Disconnect, Input };
  struct Event { Kind kind; uint32_t sequence, time_ms; Ps4InputState input; };
  struct Summary {
    uint32_t sequence=0,received=0,duplicates=0,overflows=0;
    uint32_t connects=0,disconnects=0,presses=0,releases=0;
    uint32_t cross_sequence=0,disconnect_sequence=0,neutral_sequence=0;
    bool cross_pending=false,disconnect_pending=false,neutral_pending=false;
    bool overflowed=false,connected=false,input_valid=false;
    uint16_t lost_presses=0,lost_releases=0;
    Ps4InputState latest_input{};
  };
  struct Batch : Summary { Event events[Capacity]; size_t count=0; };
  static bool after(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) > 0; }
  static bool same(const Ps4InputState& a, const Ps4InputState& b) {
    return a.left_stick_x==b.left_stick_x && a.left_stick_y==b.left_stick_y &&
      a.right_stick_x==b.right_stick_x && a.right_stick_y==b.right_stick_y &&
      a.l2==b.l2 && a.r2==b.r2 && Ps4Diagnostics::buttons(a)==Ps4Diagnostics::buttons(b);
  }
  void connect(uint32_t now) { publish(Kind::Connect, {}, now); }
  void disconnect(uint32_t now) { publish(Kind::Disconnect, {}, now); }
  void input(const Ps4InputState& snapshot, uint32_t now) { publish(Kind::Input, snapshot, now); }
  uint32_t barrier() {
    Guard guard(lock_);
    force_next_input_ = true; // A real fresh report may confirm neutral even if unchanged.
    return state_.sequence;
  }
  void drain(Batch& out) {
    Guard guard(lock_);
    // Bounded copy while locked; callbacks never execute application logic here.
    static_cast<Summary&>(out) = state_;
    for (size_t i=0;i<size_;++i) out.events[i]=events_[(head_+i)%Capacity];
    out.count=size_;
    head_=size_=0;
    if(state_.overflowed) force_next_input_=true;
    state_.cross_pending=state_.disconnect_pending=state_.neutral_pending=false;
    state_.overflowed=false;
    state_.lost_presses=state_.lost_releases=0;
  }
 private:
  struct Guard { Lock& lock; explicit Guard(Lock& value):lock(value){lock.lock();} ~Guard(){lock.unlock();} };
  Lock lock_;
  Event events_[Capacity]{};
  Summary state_{};
  size_t head_=0,size_=0;
  bool force_next_input_=false;
  static unsigned bits(uint16_t value) {
    unsigned count=0;while(value){count+=value&1u;value>>=1;}return count;
  }
  static bool driveNeutral(const Ps4InputState& value) {
    // Same used-axis deadzone as existing forward/yaw commands (not a new stop rule).
    return ps4_control::rightCentered(value.right_stick_x,value.right_stick_y);
  }
  void publish(Kind kind,const Ps4InputState& value,uint32_t now) {
    Guard guard(lock_);
    ++state_.received;
    const bool was_valid=state_.input_valid;
    const Ps4InputState previous=state_.latest_input;
    if(kind==Kind::Input && was_valid && same(previous,value) && !force_next_input_){
      ++state_.duplicates;return;
    }
    const uint32_t seq=++state_.sequence;
    uint16_t pressed=0,released=0;
    if(kind==Kind::Connect){++state_.connects;state_.connected=true;state_.input_valid=false;}
    else if(kind==Kind::Disconnect){
      ++state_.disconnects;state_.connected=false;state_.input_valid=false;
      state_.disconnect_pending=true;state_.disconnect_sequence=seq;
    } else {
      const uint16_t before=was_valid?Ps4Diagnostics::buttons(previous):0;
      const uint16_t next=Ps4Diagnostics::buttons(value);
      pressed=next&~before;released=before&~next;
      state_.presses+=bits(pressed);state_.releases+=bits(released);
      if(value.cross && (!was_valid || !previous.cross)){
        state_.cross_pending=true;state_.cross_sequence=seq;
      }
      if(was_valid && !driveNeutral(previous) && driveNeutral(value)){
        state_.neutral_pending=true;state_.neutral_sequence=seq;
      }
      state_.latest_input=value;state_.input_valid=true;force_next_input_=false;
    }
    if(size_==Capacity){
      ++state_.overflows;state_.overflowed=true;
      state_.lost_presses|=pressed;state_.lost_releases|=released;
      return; // Existing events are never overwritten. Urgent latches survive.
    }
    events_[(head_+size_)%Capacity]={kind,seq,now,value};++size_;
  }
};

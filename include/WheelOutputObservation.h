#pragma once
#include <stdint.h>
#include <stdio.h>

// One requested pair only. Move N is joined to FOC N+1; no history or I/O
// in the control path. Lock protects the HTTP/control-task handoff.
namespace wheeloutput {
struct Move {
  float command=0, target=0, current_sp=0;
};
struct Output {
  float target=0, current_sp=0, q=0, d=0, electrical=0;
  float phase[3]={}, duty[3]={};
  float mechanical=0, shaft_angle=0, shaft_velocity=0;
  float motor_limit=0, driver_limit=0, supply=0, zero=0;
  int controller=0, torque=0, status=0, enabled=0, driver_init=0, en=0, direction=0, poles=0;
};
struct Record {
  uint32_t id=0, request_us=0, command_us=0, move_us=0, sensor_us=0;
  uint32_t foc_start_us[2]={}, foc_end_us[2]={};
  Move move[2]; Output output[2];
  // 0 empty, 1 requested, 2 move staged, 3 FOC captured, 4 interrupted.
  unsigned state=0; bool applied=false;
};
template<class Lock> class Observation {
 public:
  bool request(uint32_t now) {
    lock_.lock();
    if(record_.state==1||record_.state==2){lock_.unlock();return false;}
    const uint32_t id=record_.id+1;record_=Record{};
    record_.id=id;record_.request_us=now;record_.state=1;
    lock_.unlock();return true;
  }
  void moved(uint32_t command_us,uint32_t now,const Move& right,const Move& left) {
    lock_.lock();
    if(record_.state==1){record_.command_us=command_us;record_.move_us=now;
      record_.move[0]=right;record_.move[1]=left;record_.state=2;}
    lock_.unlock();
  }
  bool waitingFoc(){lock_.lock();const bool b=record_.state==2;lock_.unlock();return b;}
  void foc(uint32_t sensor_us,const uint32_t* starts,const uint32_t* ends,
           const Output& right,const Output& left,bool ran) {
    lock_.lock();
    if(record_.state==2){
      record_.sensor_us=sensor_us;record_.output[0]=right;record_.output[1]=left;
      for(unsigned i=0;i<2;++i){record_.foc_start_us[i]=starts[i];record_.foc_end_us[i]=ends[i];}
      record_.applied=ran&&right.enabled&&left.enabled&&right.en&&left.en&&
        right.current_sp==record_.move[0].current_sp&&left.current_sp==record_.move[1].current_sp;
      record_.state=3;
    }
    lock_.unlock();
  }
  void interrupt(){lock_.lock();if(record_.state==1||record_.state==2){record_.state=4;record_.applied=false;}lock_.unlock();}
  Record copy(){lock_.lock();Record r=record_;lock_.unlock();return r;}
 private:
  Lock lock_;Record record_;
};

// Read public software caches only. phase[] is BLDCMotor's pre-driver voltage;
// duty[] is the clamped value handed to the hardware backend, not pin waveforms.
template<class Motor,class Driver,class Sensor>
Output cached(const Motor& m,const Driver& d,Sensor& s,int en){
  Output o;
  o.target=m.target;o.current_sp=m.current_sp;o.q=m.voltage.q;o.d=m.voltage.d;
  o.electrical=m.electrical_angle;
  o.phase[0]=m.Ua;o.phase[1]=m.Ub;o.phase[2]=m.Uc;
  o.duty[0]=d.dc_a;o.duty[1]=d.dc_b;o.duty[2]=d.dc_c;
  o.mechanical=s.getMechanicalAngle();o.shaft_angle=m.shaft_angle;o.shaft_velocity=m.shaft_velocity;
  o.motor_limit=m.voltage_limit;o.driver_limit=d.voltage_limit;o.supply=d.voltage_power_supply;
  o.zero=m.zero_electric_angle;o.direction=int(m.sensor_direction);o.poles=m.pole_pairs;
  o.controller=int(m.controller);o.torque=int(m.torque_controller);o.status=int(m.motor_status);
  o.enabled=m.enabled;o.driver_init=d.initialized;o.en=en;return o;
}
inline int format(const Record& r,char* text,size_t size){
  int n=snprintf(text,size,"id=%lu state=%u applied=%u request_us=%lu command_us=%lu move_us=%lu sensor_us=%lu\nSOFTWARE PWM CACHES; NOT measured pin waveforms. shaft angle/velocity and mechanical cached at sensor_us.\n",
    (unsigned long)r.id,r.state,unsigned(r.applied),(unsigned long)r.request_us,
    (unsigned long)r.command_us,(unsigned long)r.move_us,(unsigned long)r.sensor_us);
  for(unsigned i=0;i<2&&n>=0&&size_t(n)<size;++i){const auto&m=r.move[i];const auto&o=r.output[i];
    const int k=snprintf(text+n,size-size_t(n),
      "%s foc_us=%lu..%lu command_V=%.7g move_target=%.7g move_current_sp=%.7g foc_target=%.7g foc_current_sp=%.7g q=%.7g d=%.7g electrical=%.7g phase_V=%.7g,%.7g,%.7g duty=%.7g,%.7g,%.7g mechanical=%.7g shaft_angle=%.7g shaft_velocity=%.7g motor_limit=%.7g driver_limit=%.7g supply=%.7g zero=%.7g direction=%d poles=%d mode=%d torque=%d status=%d enabled=%d driver_init=%d EN=%d\n",
      i?"LEFT":"RIGHT",(unsigned long)r.foc_start_us[i],(unsigned long)r.foc_end_us[i],
      m.command,m.target,m.current_sp,o.target,o.current_sp,o.q,o.d,o.electrical,
      o.phase[0],o.phase[1],o.phase[2],o.duty[0],o.duty[1],o.duty[2],
      o.mechanical,o.shaft_angle,o.shaft_velocity,o.motor_limit,o.driver_limit,o.supply,
      o.zero,o.direction,o.poles,o.controller,o.torque,o.status,o.enabled,o.driver_init,o.en);
    if(k<0)return k;
    n+=k;
  }
  return n;
}
} // namespace wheeloutput

#pragma once
#include <stdint.h>
#include <math.h>

namespace diagnostic {
// One completed 200 ms window, not a sample history. Units: deg/s, us, ms.
struct GyroWindow {
  uint32_t end_ms=0, duration_us=0, count=0, invalid=0;
  uint32_t min_dt_us=0, max_dt_us=0;
  float rms=NAN, peak=NAN;
};
class GyroVibration {
  uint32_t start_=0, previous_=0, count_=0, invalid_=0;
  uint32_t min_=UINT32_MAX, max_=0;
  double squares_=0;
  float peak_=0;
  bool active_=false;
public:
  GyroWindow latest;
  void add(uint32_t us, uint32_t ms, float gyro, bool valid) {
    if (!active_) { active_=true; start_=previous_=us; }
    else {
      const uint32_t dt=us-previous_;
      if (dt<min_) min_=dt;
      if (dt>max_) max_=dt;
      previous_=us;
    }
    if (valid && isfinite(gyro)) {
      squares_+=double(gyro)*double(gyro);
      if (fabsf(gyro)>peak_) peak_=fabsf(gyro);
      ++count_;
    } else ++invalid_;
    if (uint32_t(us-start_)<200000) return;
    latest={ms,us-start_,count_,invalid_,min_==UINT32_MAX?0:min_,max_,NAN,NAN};
    // An invalid acquisition or a stale gap invalidates the entire window.
    if (count_ && !invalid_ && max_<=250000) {
      latest.rms=float(sqrt(squares_/count_)); latest.peak=peak_;
    }
    active_=false; count_=invalid_=max_=0; min_=UINT32_MAX;
    squares_=0; peak_=0;
  }
};
}

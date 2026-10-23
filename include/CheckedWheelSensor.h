#pragma once

#include <SimpleFOC.h>
#include <Wire.h>
#include "WheelReadDiagnostics.h"

// Same RAW_ANGLE checks and stopped-velocity correction as wheel_diagnostic.
// The callback only gates EN low; normal control code performs PWM cleanup.
class CheckedWheelSensor : public Sensor {
 public:
  CheckedWheelSensor(TwoWire& bus, void (*gateOff)())
      : bus_(bus), gateOff_(gateOff) {}

  WheelReadDiagnostics diagnostics;
  void setDiagnosticContext(uint8_t flags, uint32_t fall_ms) {
    context_ = flags; fall_ms_ = fall_ms;
  }
  // Caller guarantees outputs remain gated and owns this bus (control task only).
  // A probe never touches Sensor caches, fault, ready or the first-fault latch.
  void probeWhileGated() {
    WheelReadRecord r;
    readRaw(r);
    diagnostics.probe = r;
    if (diagnostics.probes != UINT32_MAX) ++diagnostics.probes;
  }

  bool ready = false;
  bool fault = false;
  const char* error = "NOT_INITIALIZED";

  void begin(int sda, int scl) {
    if (!bus_.begin(sda, scl, 400000)) { fail("BUS_INIT"); return; }
    bus_.setTimeOut(5);
    beginPreparedBus();
  }

  // Caller must have checked TwoWire::begin() before entering this path.
  // Keep bus buffers/driver allocated; perform sensor reads only after Web is ready.
  void beginPreparedBus() {
    Sensor::init();
    ready = !fault;
  }

  float getVelocity() override {
    const float elapsed = (angle_prev_ts - vel_angle_prev_ts) * 1e-6f;
    if (elapsed >= min_elapsed_time && full_rotations == vel_full_rotations &&
        fabsf(angle_prev - vel_angle_prev) <= 1e-8f) {
      velocity = 0.0f;
    }
    return Sensor::getVelocity();
  }

 protected:
  float getSensorAngle() override {
    if (fault) return -1.0f;
    WheelReadRecord r;
    const float angle = readRaw(r);
    if (r.result != 1) {
      // Gate before saving the fixed-size record or producing any diagnostics.
      fail(r.result == 2 ? "I2C_ADDRESS/NACK" : r.result == 3 ?
           "I2C_SHORT_READ/TIMEOUT" : r.result == 4 ? "INVALID_DATA" : "INVALID_ANGLE");
      if (!diagnostics.latched) { diagnostics.first = r; diagnostics.latched = true; }
      return -1.0f;
    }
    error = "OK";
    return angle;
  }

 private:
  uint8_t context_=0;
  uint32_t fall_ms_=0;
  float readRaw(WheelReadRecord& r) {
    r.ms=millis(); r.context=context_; r.fall_ms=fall_ms_;
    const uint32_t start=micros();
    bus_.beginTransmission(0x36); bus_.write(r.reg);
    r.tx=bus_.endTransmission(false);
    float angle=-1.0f;
    if(r.tx!=0) r.result=2;
    else {
      r.received=bus_.requestFrom(uint8_t(0x36),uint8_t(r.requested));
      r.available=bus_.available();
      if(r.received!=2||r.available!=2) r.result=3;
      else {
        r.high=bus_.read(); r.low=bus_.read();
        if(r.high<0||r.high>0x0f||r.low<0||r.low>0xff) r.result=4;
        else {
          angle=((r.high<<8)|r.low)*(6.283185307179586f/4096.0f);
          r.result=isfinite(angle)&&angle>=0&&angle<6.283185307179586f ? 1 : 5;
        }
      }
    }
    r.elapsed_us=micros()-start;
    return angle;
  }
  TwoWire& bus_;
  void (*gateOff_)();
  float fail(const char* reason) {
    fault = true;
    error = reason;
    gateOff_();
    return -1.0f;
  }
};

#pragma once

#include <SimpleFOC.h>
#include <Wire.h>

// Same RAW_ANGLE checks and stopped-velocity correction as wheel_diagnostic.
// The callback only gates EN low; normal control code performs PWM cleanup.
class CheckedWheelSensor : public Sensor {
 public:
  CheckedWheelSensor(TwoWire& bus, void (*gateOff)())
      : bus_(bus), gateOff_(gateOff) {}

  bool ready = false;
  bool fault = false;
  const char* error = "NOT_INITIALIZED";

  void begin(int sda, int scl) {
    if (!bus_.begin(sda, scl, 400000)) { fail("BUS_INIT"); return; }
    bus_.setTimeOut(5);
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
    bus_.beginTransmission(0x36);
    bus_.write(0x0C);
    if (bus_.endTransmission(false) != 0) return fail("I2C_ADDRESS/NACK");
    if (bus_.requestFrom(uint8_t(0x36), uint8_t(2)) != 2 || bus_.available() != 2)
      return fail("I2C_SHORT_READ/TIMEOUT");
    const int high = bus_.read(), low = bus_.read();
    if (high < 0 || high > 0x0F || low < 0 || low > 0xFF)
      return fail("INVALID_DATA");
    const float angle = ((high << 8) | low) * (6.283185307179586f / 4096.0f);
    if (!isfinite(angle) || angle < 0 || angle >= 6.283185307179586f)
      return fail("INVALID_ANGLE");
    error = "OK";
    return angle;
  }

 private:
  TwoWire& bus_;
  void (*gateOff_)();
  float fail(const char* reason) {
    fault = true;
    error = reason;
    gateOff_();
    return -1.0f;
  }
};

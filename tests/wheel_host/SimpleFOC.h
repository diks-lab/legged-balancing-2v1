#pragma once
#include "Arduino.h"
// Hardware/FOC doubles: tests exercise the actual diagnostic source, not
// electrical calibration. Real Sensor base is included via the runner's -I.
#include "common/base_classes/Sensor.h"
enum class MotionControlType { torque };
enum class TorqueControlType { voltage };
inline bool driverInitOK = true, motorInitOK = true, focInitOK = true, ppOK = true;
inline std::function<void()> duringFOC;
class BLDCDriver3PWM {
 public:
  int en, initCalls = 0;
  float voltage_power_supply = 0, voltage_limit = 0;
  float pwm = 0;
  BLDCDriver3PWM(int, int, int, int e) : en(e) {}
  bool init() { ++initCalls; return driverInitOK; }
  void disable() { pwm = 0; digitalWrite(en, LOW); }
  void enable() { pwm = 0; digitalWrite(en, HIGH); }
};
class BLDCMotor {
 public:
  explicit BLDCMotor(int) {}
  BLDCDriver3PWM* driver = nullptr;
  Sensor* sensor = nullptr;
  int initCalls = 0, focCalls = 0;
  float target = 0, voltage_limit = 0, voltage_sensor_align = 0, zero_electric_angle = 0.1f;
  struct { float q = 0, d = 0; } voltage;
  bool pp_check_result = false;
  Direction sensor_direction = CW;
  MotionControlType controller;
  TorqueControlType torque_controller;
  void linkDriver(BLDCDriver3PWM* d) { driver = d; }
  void linkSensor(Sensor* s) { sensor = s; }
  bool init() { ++initCalls; delay(500); enable(); delay(500); return motorInitOK; }
  bool initFOC() {
    ++focCalls; if (duringFOC) duringFOC(); sensor->update();
    pp_check_result = ppOK; return focInitOK;
  }
  void enable() { driver->enable(); }
  void disable() { driver->disable(); }
  void loopFOC() { sensor->update(); }
  void move(float v) { target = v; voltage.q = v; driver->pwm = v; }
};

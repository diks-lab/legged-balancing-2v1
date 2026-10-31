#pragma once
#include "Arduino.h"
// Hardware/FOC doubles: tests exercise the actual diagnostic source, not
// electrical calibration. Real Sensor base is included via the runner's -I.
#include "common/base_classes/Sensor.h"
enum class MotionControlType { torque, velocity };
enum class TorqueControlType { voltage };
inline bool driverInitOK = true, motorInitOK = true, focInitOK = true, ppOK = true;
inline std::function<void()> duringFOC;
inline std::vector<int> driverInitEvents;
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)
#include "drivers/hardware_specific/esp32/esp32_driver_mcpwm.h"
#endif
class BLDCDriver3PWM {
 public:
  int en, pwmA, pwmB, pwmC, initCalls = 0;
  float voltage_power_supply = 0, voltage_limit = 0;
  float pwm = 0;
  void* params = nullptr;
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)
  ESP32MCPWMDriverParams allocation{};
#endif
  BLDCDriver3PWM(int a, int b, int c, int e) : en(e), pwmA(a), pwmB(b), pwmC(c) {}
  bool init() {
    ++initCalls; driverInitEvents.push_back(en);
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)
    // Deliberately non-default assignments distinguish backend readout from
    // a guessed group0/timer0 based on initialization order.
    allocation.group_id = en == 27 ? 1 : 0;
    allocation.timers[0] = timers[allocation.group_id][2];
    params = &allocation;
#endif
    return driverInitOK;
  }
  void disable() { pwm = 0; digitalWrite(en, LOW); }
  void enable() { pwm = 0; digitalWrite(en, HIGH); }
};
class BLDCMotor {
 public:
  explicit BLDCMotor(int) {}
  BLDCDriver3PWM* driver = nullptr;
  Sensor* sensor = nullptr;
  int initCalls = 0, focCalls = 0, loopCalls = 0, moveCalls = 0;
  float current_sp = 0, shaft_velocity = 0, velocity_limit = 0;
  struct { float P=0, I=0, D=0, output_ramp=0, limit=0; int resets=0; void reset() { ++resets; } } PID_velocity;
  struct { float Tf=0; } LPF_velocity;
  void useMonitoring(FakeSerial&) {}
  void updateVoltageLimit(float v) { voltage_limit = PID_velocity.limit = v; }
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
  void loopFOC() { ++loopCalls; sensor->update(); shaft_velocity = sensor->getVelocity(); }
  void move(float v) { ++moveCalls; target = v; voltage.q = std::fmax(-voltage_limit, std::fmin(v, voltage_limit)); driver->pwm = voltage.q; }
};

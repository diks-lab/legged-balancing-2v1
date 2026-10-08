#include "../../src/wheel_diagnostic.cpp"
#include <cassert>
#include <iostream>

// Definitions for the real SimpleFOC Sensor base used by our subclass. These
// getters are identical in behavior to 2.4.0; update() under test is our override.
float Sensor::getMechanicalAngle() { return angle_prev; }
float Sensor::getAngle() { return full_rotations * kTau + angle_prev; }
double Sensor::getPreciseAngle() { return double(full_rotations) * kTau + angle_prev; }
int32_t Sensor::getFullRotations() { return full_rotations; }
float Sensor::getVelocity() { return velocity; }
int Sensor::needsSearch() { return 0; }
void Sensor::init() {}
void Sensor::update() {}

void input(const std::string& s) {
  Serial.feed(s);
  while (Serial.available()) loop();
}
void disabled() { assert(!pins[27] && !pins[32]); }
void onlyLeft() { assert(pins[32] && !pins[27]); assert(rightDriver.initCalls == 0); }
void onlyRight() { assert(pins[27] && !pins[32]); assert(leftDriver.initCalls == 0); }
int main(int argc, char** argv) {
  assert(argc == 2);
  const std::string scenario = argv[1];
  if (scenario == "missing") rightBus.nack = 2;
  if (scenario == "bus_init") leftBus.busOK = false;
  if (scenario == "timer_create") createTimerOK = false;
  setup();
  disabled();
  assert(rightDriver.initCalls == 0 && leftDriver.initCalls == 0);
  assert(rightMotor.initCalls == 0 && leftMotor.initCalls == 0);
  assert(gpioEvents[0] == std::make_pair(27, LOW));
  assert(gpioEvents[1] == std::make_pair(32, LOW));
  assert(gpioEvents[2] == std::make_pair(27, OUTPUT + 10));
  assert(gpioEvents[3] == std::make_pair(32, OUTPUT + 10));
  if (scenario == "invalid") {
    for (const std::string s : {"motor", "motor both +", "motor left ++", "motor left + ",
         " motor right +", "motor left +x", "motor left 1", "motor left +\rX", "MOTOR left +"}) {
      input(s + "\n"); disabled();
    }
    input(std::string(100, 'x') + "motor left +\n");
    input(std::string("motor left +\0", 13) + "\n");
    assert(leftMotor.initCalls == 0 && rightMotor.initCalls == 0);
    Serial.feed("motor left +"); loop(); disabled(); // newline required
    input("\r\n"); onlyLeft();
  } else if (scenario == "wrap") {
    rightBus.raw = 4090; rightSensor.update();
    const double before = rightSensor.getPreciseAngle();
    rightBus.raw = 5; rightSensor.update();
    assert(rightSensor.getPreciseAngle() > before);
    assert(rightSensor.getPreciseAngle() - before < 0.02);
    rightBus.raw = 4090; rightSensor.update();
    assert(std::abs(rightSensor.getPreciseAngle() - before) < 0.00001);
    disabled();
  } else if (scenario == "missing" || scenario == "bus_init" || scenario == "timer_create") {
    input("motor left +\n"); disabled(); assert(leftMotor.initCalls == 0);
  } else if (scenario == "driver_fail" || scenario == "motor_fail" || scenario == "foc_fail" || scenario == "pp_fail") {
    driverInitOK = scenario != "driver_fail"; motorInitOK = scenario != "motor_fail";
    focInitOK = scenario != "foc_fail"; ppOK = scenario != "pp_fail";
    input("motor left +\n"); disabled(); assert(left.initFault && !running);
    int n = leftMotor.initCalls; input("motor left +\n"); assert(leftMotor.initCalls == n);
  } else if (scenario == "init_stop" || scenario == "init_motor" || scenario == "init_partial") {
    duringFOC = [&] { Serial.feed(scenario == "init_stop" ? "stop\n" :
                                  scenario == "init_motor" ? "motor right +\n" : "motor right"); };
    input("motor left +\n"); disabled(); assert(!running && rightMotor.initCalls == 0);
    if (scenario == "init_partial") { input("motor right +\n"); disabled(); assert(rightMotor.initCalls == 0); }
  } else if (scenario == "init_timeout") {
    duringFOC = [] { delay(kAlignMs); disabled(); };
    input("motor left +\n"); disabled(); assert(left.initFault && !running);
  } else if (scenario == "init_sensor") {
    duringFOC = [] { leftBus.nack = 2; };
    input("motor left +\n"); disabled(); assert(leftSensor.fault && left.initFault);
  } else if (scenario == "right") {
    input("motor right -\n"); onlyRight(); assert(commandVoltage == -kDriveVoltage);
    input("stop\n"); disabled(); assert(!running && rightDriver.pwm == 0);
  } else {
    if (scenario == "clock_wrap") nowMs = UINT32_MAX - 500;
    input("motor left +\n"); onlyLeft(); assert(commandVoltage == kDriveVoltage);
    input("motor right -\nmotor left -\n"); onlyLeft(); assert(commandVoltage == kDriveVoltage);
    if (scenario == "during_control") {
      int reads = 0;
      leftBus.onRequest = [&] { if (++reads == 2) leftBus.count = 2; };
      loop(); disabled(); assert(leftSensor.fault && !running && leftDriver.pwm == 0);
    } else if (scenario == "other_sensor") {
      rightBus.nack = 2; loop(); disabled(); assert(rightSensor.fault && !running);
    } else if (scenario == "nack" || scenario == "short" || scenario == "magnet" ||
               scenario == "weak" || scenario == "strong" || scenario == "data") {
      if (scenario == "nack") leftBus.nack = 2;
      if (scenario == "short") leftBus.count = 2;
      if (scenario == "magnet") leftBus.magnet = 0;
      if (scenario == "weak") leftBus.magnet = 0x30;
      if (scenario == "strong") leftBus.magnet = 0x28;
      if (scenario == "data") leftBus.raw = 0xFFFF;
      loop(); disabled(); assert(leftSensor.fault && !running && leftDriver.pwm == 0);
      leftBus.nack = 0; leftBus.count = 3; leftBus.magnet = 0x20; leftBus.raw = 0;
      input("read left\nmotor left +\n"); disabled(); // latched, no stale OK/retry
      while (txRead != txWrite) pumpLog();
      assert(Serial.output.find("angle/continuous/turns=INVALID") != std::string::npos);
    } else if (scenario == "timer_start") {
      input("stop\n"); startTimerOK = false;
      input("motor left -\n"); disabled(); assert(!running);
    } else if (scenario == "loop_timeout") {
      fakeTimer.armed = false; nowMs += kRunMs; loop(); disabled(); assert(!running);
    } else if (scenario == "blocked_tx" || scenario == "clock_wrap") {
      Serial.room = 0; input("stream on\n");
      input(std::string(3000, 'x') + "\n");
      for (int i = 0; i < 30; ++i) input("status\n");
      assert(droppedLogs > 0);
      // No loop invocation: timer gates off independently of blocked printing.
      delay(kRunMs); disabled(); loop(); assert(!running);
    } else { input("stop\n"); disabled(); assert(!running && leftDriver.pwm == 0); }
  }
  // At no point may both EN pins be HIGH, including initialization/cleanup.
  int r = 0, l = 0;
  for (auto e : gpioEvents) {
    if (e.second > 1) continue;
    if (e.first == 27) r = e.second;
    if (e.first == 32) l = e.second;
    assert(!(r && l));
  }
  std::cout << "PASS " << scenario << '\n';
}

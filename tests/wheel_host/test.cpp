#include "../../src/wheel_diagnostic.cpp"
#include <cassert>
#include <iostream>

unsigned long _micros() { return micros(); }

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
  if (scenario.rfind("both_", 0) == 0) {
    int initializations = 0;
    const bool secondFailure = scenario.find("second_") != std::string::npos;
    const bool firstFailure = scenario.find("first_") != std::string::npos;
    if (firstFailure && scenario == "both_first_driver") driverInitOK = false;
    if (scenario == "both_missing") rightBus.nack = 2;
    if (scenario == "both_timer_create") timerReady = false;
    duringFOC = [&] {
      ++initializations;
      // Sequential alignment: the other enable must be LOW throughout.
      if (initializations == 1) assert(!pins[27]);
      else { assert(!pins[32]); assert(leftDriver.pwm == 0); }
      if ((firstFailure && initializations == 1) || (secondFailure && initializations == 2)) {
        if (scenario.find("foc") != std::string::npos) focInitOK = false;
        if (scenario.find("pp") != std::string::npos) ppOK = false;
        if (scenario.find("sensor") != std::string::npos) rightBus.nack = 2;
        if (scenario.find("timeout") != std::string::npos) { delay(kAlignMs); disabled(); }
        if (scenario.find("stop") != std::string::npos) Serial.feed("stop\n");
        if (scenario.find("partial") != std::string::npos) Serial.feed("motor both");
        if (scenario.find("queued") != std::string::npos) Serial.feed("motor both -\n");
      }
      if (initializations == 1 && scenario == "both_second_driver") driverInitOK = false;
      if (initializations == 1 && scenario == "both_second_motor") motorInitOK = false;
      if (initializations == 2 && scenario == "both_timer_start") startTimerOK = false;
    };
    input(scenario == "both_negative" ? "motor both -\n" : "motor both +\n");
    if (firstFailure || secondFailure || scenario == "both_missing" ||
        scenario == "both_timer_create" || scenario == "both_timer_start") {
      disabled(); assert(!running && !runningBoth);
      assert(leftMotor.moveCalls == 0 && rightMotor.moveCalls == 0);
      if (firstFailure) assert(rightMotor.initCalls == 0);
      if (scenario.find("partial") != std::string::npos) { input(" +\n"); disabled(); }
    } else {
      assert(pins[27] && pins[32] && runningBoth);
      assert(left.calibrated && right.calibrated);
      const float target = scenario == "both_negative" ? -6.0f : 6.0f;
      assert(leftMotor.target == target && rightMotor.target == target);
      for (Wheel* w : {&left, &right}) {
        assert(w->motor.PID_velocity.P == 0.075f && w->motor.PID_velocity.I == 0.4f);
        assert(w->motor.PID_velocity.D == 0 && w->motor.PID_velocity.output_ramp == 5);
        assert(w->motor.LPF_velocity.Tf == 0.05f);
        assert(w->motor.voltage_limit == 1 && w->motor.voltage_sensor_align == 2);
        assert(w->motor.PID_velocity.resets == 2); // setup plus run
      }
      const int l = leftBus.requests, r = rightBus.requests;
      loop(); assert(leftBus.requests == l + 1 && rightBus.requests == r + 1);
      input("motor left -\nmotor both -\n"); assert(commandVelocity == target);
      if (scenario == "both_left_sensor") leftBus.nack = 2;
      else if (scenario == "both_right_sensor") rightBus.count = 1;
      else if (scenario == "both_loop_timeout") { fakeTimer.armed = false; nowMs += kRunMs; }
      else if (scenario == "both_timer") { Serial.room = 0; delay(kRunMs); disabled(); }
      else if (scenario == "both_restart") {
        input("stop\n"); disabled();
        input("motor both -\n"); assert(runningBoth);
        assert(leftMotor.initCalls == 1 && rightMotor.initCalls == 1);
        assert(leftMotor.PID_velocity.resets == 3 && rightMotor.PID_velocity.resets == 3);
        input("stop\n");
      } else input("stop\n");
      loop(); disabled(); assert(!running && !runningBoth);
      assert(leftDriver.pwm == 0 && rightDriver.pwm == 0);
      assert(leftMotor.target == 0 && rightMotor.target == 0);
    }
  } else if (scenario == "invalid") {
    for (const std::string s : {"motor", "motor both ++", "motor left ++", "motor left + ",
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
    input("motor right -\n"); onlyRight(); assert(commandVelocity == -kTargetVelocity);
    input("stop\n"); disabled(); assert(!running && rightDriver.pwm == 0);
  } else {
    if (scenario == "clock_wrap") nowMs = UINT32_MAX - 500;
    input("motor left +\n"); onlyLeft(); assert(commandVelocity == kTargetVelocity);
    input("motor right -\nmotor left -\n"); onlyLeft(); assert(commandVelocity == kTargetVelocity);
    if (scenario == "during_control") {
      int reads = 0;
      leftBus.onRequest = [&] { if (++reads == 1) leftBus.count = 1; };
      loop(); disabled(); assert(leftSensor.fault && !running && leftDriver.pwm == 0);
    } else if (scenario == "other_sensor") {
      rightBus.nack = 2; loop(); disabled(); assert(rightSensor.fault && !running);
    } else if (scenario == "nack" || scenario == "short" || scenario == "data") {
      if (scenario == "nack") leftBus.nack = 2;
      if (scenario == "short") leftBus.count = 1;
      if (scenario == "data") leftBus.raw = 0xFFFF;
      loop(); disabled(); assert(leftSensor.fault && !running && leftDriver.pwm == 0);
      leftBus.nack = 0; leftBus.count = 2; leftBus.magnet = 0x20; leftBus.raw = 0;
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
  // Single-wheel runs and cancelled dual runs must never enable both pins.
  int r = 0, l = 0;
  for (auto e : gpioEvents) {
    if (e.second > 1) continue;
    if (e.first == 27) r = e.second;
    if (e.first == 32) l = e.second;
    if (scenario.rfind("both_", 0) != 0 ||
        scenario.find("first_") != std::string::npos ||
        scenario.find("second_") != std::string::npos ||
        scenario == "both_missing" || scenario == "both_timer_create" ||
        scenario == "both_timer_start") assert(!(r && l));
  }
  std::cout << "PASS " << scenario << '\n';
}

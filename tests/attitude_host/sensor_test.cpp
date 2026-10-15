#include "CheckedWheelSensor.h"
#include <cassert>
#include <iostream>

unsigned long _micros() { return micros(); }
void gateBoth() { digitalWrite(27, LOW); digitalWrite(32, LOW); }
void enableBoth() { digitalWrite(27, HIGH); digitalWrite(32, HIGH); }
void expectBothOff() { assert(!pins[27] && !pins[32]); }

int main() {
  TwoWire rightBus(0), leftBus(1);
  CheckedWheelSensor right(rightBus, gateBoth), left(leftBus, gateBoth);
  right.begin(19, 18);
  left.begin(23, 5);
  assert(right.ready && left.ready);
  delay(10);
  rightBus.raw = 10;
  right.update();
  assert(right.getVelocity() > 0);
  // A stationary sample clears the previous nonzero speed, using real 2.4.0
  // Sensor timestamps and minimum sample interval (not a fake velocity API).
  delay(10);
  right.update();
  assert(right.getVelocity() == 0);
  assert(right.getAngle() > 0);
  // Zero-crossing tracks a forward wrap, then a reverse wrap.
  TwoWire wrapBus(0);
  wrapBus.raw = 4090;
  CheckedWheelSensor wrap(wrapBus, gateBoth);
  wrap.begin(19, 18);
  delay(10);
  wrapBus.raw = 5;
  wrap.update();
  assert(wrap.getFullRotations() == 1 && wrap.getVelocity() > 0);
  delay(10);
  wrapBus.raw = 4090;
  wrap.update();
  assert(wrap.getFullRotations() == 0 && wrap.getVelocity() < 0);
  // Either wheel's communication failure immediately gates BOTH outputs.
  enableBoth();
  rightBus.nack = 2;
  right.update();
  expectBothOff();
  assert(right.fault && !left.fault);
  const int reads = rightBus.requests;
  rightBus.nack = 0;
  right.update();
  assert(right.fault && rightBus.requests == reads); // reboot-only latch
  enableBoth();
  leftBus.count = 1;
  left.update();
  expectBothOff();
  assert(left.fault);
  TwoWire invalidBus(0);
  CheckedWheelSensor invalid(invalidBus, gateBoth);
  invalidBus.raw = 0xFFFF;
  enableBoth();
  invalid.begin(19, 18);
  expectBothOff();
  assert(invalid.fault && !invalid.ready);
  TwoWire missingBus(0);
  CheckedWheelSensor missing(missingBus, gateBoth);
  missingBus.busOK = false;
  enableBoth();
  missing.begin(19, 18);
  expectBothOff();
  assert(missing.fault && !missing.ready);
  std::cout << "PASS: stopped velocity, wraps, either-wheel gate-off, latched faults, init failures\n";
}

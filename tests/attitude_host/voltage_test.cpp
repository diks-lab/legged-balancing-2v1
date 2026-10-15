#include "LqrMotorVoltage.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  // Recorded forward/backward tilt errors, with zero gyro contribution.
  for (float forwardSign : {1.0f, 1.0f}) {
    assert(lqrMotorVoltage(5.93f, forwardSign, true, 1.0f) == 1.0f);
    assert(lqrMotorVoltage(-9.87f, forwardSign, true, 1.0f) == -1.0f);
    assert(lqrMotorVoltage(0.25f, forwardSign, true, 1.0f) == 0.25f);
    assert(lqrMotorVoltage(-0.25f, forwardSign, true, 1.0f) == -0.25f);
    assert(lqrMotorVoltage(0.0f, forwardSign, true, 1.0f) == 0.0f);
    // Modes outside this hardware verification retain their previous polarity.
    assert(lqrMotorVoltage(0.25f, forwardSign, false, 1.0f) == -0.25f);
  }
  assert(lqrMotorVoltage(0.25f, -1.0f, true, 1.0f) == -0.25f);
  std::cout << "PASS: recorded tilts, unsaturated/zero voltage, mode polarity\n";
}

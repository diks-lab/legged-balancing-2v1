#include "LqrMotorVoltage.h"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
  // Recorded forward/backward tilt errors, with zero gyro contribution.
  for (float forwardSign : {1.0f, 1.0f}) {
    assert(lqrMotorVoltage(5.93f, forwardSign, 1.0f) == 1.0f);
    assert(lqrMotorVoltage(-9.87f, forwardSign, 1.0f) == -1.0f);
    assert(lqrMotorVoltage(0.25f, forwardSign, 1.0f) == 0.25f);
    assert(lqrMotorVoltage(-0.25f, forwardSign, 1.0f) == -0.25f);
    assert(lqrMotorVoltage(0.0f, forwardSign, 1.0f) == 0.0f);
    // All selected modes sum signed physical voltage contributions.
    assert(lqrMotorVoltage(0.25f, forwardSign, 1.0f) == 0.25f);
  }
  assert(lqrMotorVoltage(0.25f, -1.0f, 1.0f) == -0.25f);
  std::cout << "PASS: recorded tilts, unsaturated/zero voltage, mode polarity\n";
}

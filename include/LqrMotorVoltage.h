#pragma once

// Vehicle-forward signs are independent of sensor_direction. Hardware testing
// confirmed positive AngleY error is forward tilt. Control terms are signed
// individually in vehicle coordinates before summation; no mode-wide inversion.
inline float lqrMotorVoltage(float control, float forwardSign,
                             float limit) {
  const float voltage = control * forwardSign;
  return voltage > limit ? limit : (voltage < -limit ? -limit : voltage);
}

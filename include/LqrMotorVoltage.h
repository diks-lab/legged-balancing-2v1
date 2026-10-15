#pragma once

// Vehicle-forward signs are independent of sensor_direction. Hardware testing
// confirmed positive AngleY error is forward tilt: AttitudeOnly needs +Uq.
inline float lqrMotorVoltage(float control, float forwardSign,
                             bool attitudeOnly, float limit) {
  const float voltage = (attitudeOnly ? control : -control) * forwardSign;
  return voltage > limit ? limit : (voltage < -limit ? -limit : voltage);
}

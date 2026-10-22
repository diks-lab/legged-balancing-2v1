#pragma once
#include <stdint.h>
namespace imu_config {
// Change only this value for the independent DLPF comparison commit.
constexpr uint8_t DlpfConfig=0x00;
struct Result { uint8_t config=255, divider=255, gyro=255, accel=255; };
// Only during initialization, with motors disabled and before gyro calibration.
// Do not use the library's unchecked byte-returning read helper.
template<class Wire> bool apply(Wire& bus, Result& result) {
  bus.beginTransmission(0x68); bus.write(uint8_t(0x1a)); bus.write(DlpfConfig);
  if (bus.endTransmission(true)!=0) return false;
  const uint8_t regs[]={0x1a,0x19,0x1b,0x1c};
  uint8_t* values[]={&result.config,&result.divider,&result.gyro,&result.accel};
  for (unsigned i=0;i<4;++i) {
    bus.beginTransmission(0x68); bus.write(regs[i]);
    if (bus.endTransmission(false)!=0) return false;
    if (bus.requestFrom(uint8_t(0x68),uint8_t(1))!=1) return false;
    const int value=bus.read(); if (value<0) return false;
    *values[i]=uint8_t(value);
  }
  return result.config==DlpfConfig && result.divider==0 &&
         result.gyro==0x08 && result.accel==0;
}
}

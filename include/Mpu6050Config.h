#pragma once
#include <stdint.h>
namespace imu_config {
constexpr uint8_t DlpfConfig=0x03;
enum class Failure:uint8_t { NotRun, Ok, Write, Register, Receive, Read, Mismatch };
inline const char* failureName(Failure f) {
  static const char* const names[]={"NOT_RUN","OK","WRITE_FAILED","REGISTER_FAILED",
    "SHORT_RECEIVE","READ_FAILED","VALUE_MISMATCH"};
  return unsigned(f)<7?names[unsigned(f)]:"UNKNOWN";
}
inline const char* registerName(uint8_t reg) {
  switch(reg){case 0x1a:return "CONFIG";case 0x19:return "SMPLRT_DIV";
    case 0x1b:return "GYRO_CONFIG";case 0x1c:return "ACCEL_CONFIG";default:return "NONE";}
}
struct Result {
  uint8_t config=255, divider=255, gyro=255, accel=255;
  Failure failure=Failure::NotRun;
  uint8_t reg=0, expected=0;
  int16_t actual=-1, tx=-1, rx=-1, read=-1;
};
// Initialization only. -1 means not obtained/not executed, never a received byte.
template<class Wire> bool apply(Wire& bus, Result& result) {
  result=Result{}; result.reg=0x1a;result.expected=DlpfConfig;
  bus.beginTransmission(0x68); bus.write(uint8_t(0x1a)); bus.write(DlpfConfig);
  result.tx=bus.endTransmission(true);
  if (result.tx!=0) {result.failure=Failure::Write;return false;}
  const uint8_t regs[]={0x1a,0x19,0x1b,0x1c};
  const uint8_t expected[]={DlpfConfig,0,8,0};
  uint8_t* values[]={&result.config,&result.divider,&result.gyro,&result.accel};
  // Preserve the old transaction order and read every register before comparing.
  for (unsigned i=0;i<4;++i) {
    result.reg=regs[i]; result.expected=expected[i];
    result.actual=result.rx=result.read=-1;
    bus.beginTransmission(0x68); bus.write(regs[i]);
    result.tx=bus.endTransmission(false);
    if (result.tx!=0) {result.failure=Failure::Register;return false;}
    result.rx=bus.requestFrom(uint8_t(0x68),uint8_t(1));
    if (result.rx!=1) {result.failure=Failure::Receive;return false;}
    result.read=bus.read();
    if (result.read<0) {result.failure=Failure::Read;return false;}
    result.actual=result.read;*values[i]=uint8_t(result.read);
  }
  for(unsigned i=0;i<4;++i)if(*values[i]!=expected[i]) {
    result.failure=Failure::Mismatch;result.reg=regs[i];result.expected=expected[i];
    result.actual=result.read=*values[i];result.tx=0;result.rx=1;return false;
  }
  result.failure=Failure::Ok;return true;
}
}

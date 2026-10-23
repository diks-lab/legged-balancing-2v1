#pragma once
#include <stdint.h>

// No history, allocation or bus ownership: one first fault and one latest probe.
struct WheelReadRecord {
  uint32_t ms=0, elapsed_us=0, fall_ms=0;
  int16_t tx=-1, received=-1, available=-1, high=-1, low=-1;
  uint8_t reg=0x0c, requested=2, context=0, result=0;
};
struct WheelReadDiagnostics {
  WheelReadRecord first, probe;
  uint32_t probes=0;
  bool latched=false;
};

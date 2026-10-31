#pragma once
#include <stdint.h>

// Single immutable result after gate-off. No history, allocation or I/O.
struct PolarityComparison {
  struct Wheel {
    float requested=0, command=0, q=0, start_angle=0, end_angle=0;
    float velocity=0, mean_velocity=0, peak_velocity=0;
  } wheels[2];
  uint32_t start_us=0, end_us=0, start_sample_us=0, end_sample_us=0;
  uint32_t first_move_us=0, foc_sample_us=0, timer_dispatch_us=0;
  uint32_t samples=0, rejected=0;
  uint8_t motor_enabled=0, driver_en=0, stopped_en=0, faults=0;
  const char* reason=nullptr;
  bool started=false, pending=false;
};

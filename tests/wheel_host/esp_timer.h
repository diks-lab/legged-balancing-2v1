#pragma once
#include "Arduino.h"
constexpr int ESP_OK = 0;
struct Timer { void (*callback)(void*) = nullptr; bool armed = false; uint32_t start = 0, duration = 0; };
using esp_timer_handle_t = Timer*;
struct esp_timer_create_args_t { void (*callback)(void*); const char* name; };
inline bool createTimerOK = true, startTimerOK = true;
inline Timer fakeTimer;
inline int esp_timer_create(esp_timer_create_args_t* a, Timer** timer) {
  if (!createTimerOK) return -1;
  fakeTimer.callback = a->callback; *timer = &fakeTimer;
  tickTimer = [] {
    if (fakeTimer.armed && uint32_t(nowMs - fakeTimer.start) >= fakeTimer.duration) {
      fakeTimer.armed = false; fakeTimer.callback(nullptr);
    }
  };
  return 0;
}
inline int esp_timer_stop(Timer* t) { t->armed = false; return 0; }
inline int esp_timer_start_once(Timer* t, uint64_t us) {
  if (!startTimerOK) return -1;
  t->start = nowMs; t->duration = us / 1000; t->armed = true; return 0;
}

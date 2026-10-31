#pragma once
// Test-only backend metadata. Does not allocate or model real ESP32 resources.
using mcpwm_timer_handle_t = void*;
struct ESP32MCPWMDriverParams {
  int group_id = -1;
  mcpwm_timer_handle_t timers[2] = {};
};
inline int timerTokens[2][3];
inline mcpwm_timer_handle_t timers[2][3] = {
  {&timerTokens[0][0], &timerTokens[0][1], &timerTokens[0][2]},
  {&timerTokens[1][0], &timerTokens[1][1], &timerTokens[1][2]}
};

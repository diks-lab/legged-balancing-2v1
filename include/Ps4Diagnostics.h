#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

struct Ps4InputState {
  int8_t left_stick_x, left_stick_y, right_stick_x, right_stick_y;
  uint8_t l2, r2;
  bool cross, circle, square, triangle;
  bool dpad_up, dpad_down, dpad_left, dpad_right;
  bool l1, r1, options, share, ps_button;
};

// Main-loop-owned observations. HTTP receives formatted queue copies only.
// This object never reads a controller, bus or actuator and never logs itself.
struct Ps4Diagnostics {
  bool init_attempted = false, init_ok = false, connected = false, dirty = true;
  Ps4InputState input{};
  uint16_t down = 0, last_press = 0, last_release = 0;
  uint16_t pending_press = 0, pending_release = 0;
  uint32_t connects = 0, disconnects = 0, button_events = 0, sample_ms = 0;
  uint32_t decisions = 0, accepted = 0, rejected = 0;
  const char* event = "NONE";
  const char* action = "NONE";
  const char* result = "NONE";
  const char* reason = "NO_INPUT";

  static uint16_t buttons(const Ps4InputState& p) {
    const bool values[] = {p.cross,p.circle,p.square,p.triangle,p.dpad_up,p.dpad_down,
      p.dpad_left,p.dpad_right,p.l1,p.r1,p.options,p.share,p.ps_button};
    uint16_t mask = 0;
    for (unsigned i = 0; i < 13; ++i) if (values[i]) mask |= 1u << i;
    return mask;
  }
  static void names(uint16_t mask, char* out, size_t capacity) {
    const char* labels[] = {"Cross","Circle","Square","Triangle","Up","Down",
      "Left","Right","L1","R1","Options","Share","PS"};
    size_t used = 0;
    if (!capacity) return;
    out[0] = '\0';
    for (unsigned i = 0; i < 13; ++i) if (mask & (1u << i)) {
      const int count = snprintf(out + used, capacity - used, "%s%s", used ? "," : "", labels[i]);
      if (count < 0 || static_cast<size_t>(count) >= capacity - used) return;
      used += count;
    }
    if (!used) snprintf(out, capacity, "none");
  }
  void connection(bool value) {
    if (connected == value) return;
    connected = value;
    event = value ? "CONNECTED" : "DISCONNECTED";
    if (value) ++connects; else ++disconnects;
    if (!value) {
      last_release = down;
      pending_release |= down;
      down = 0;
      input = {};
    }
    dirty = true;
  }
  void observe(const Ps4InputState& value, uint32_t now) {
    const uint16_t next = buttons(value);
    const uint16_t pressed = next & ~down, released = down & ~next;
    if (pressed || released) ++button_events;
    if (pressed) last_press = pressed;
    if (released) last_release = released;
    pending_press |= pressed;
    pending_release |= released;
    if (next != down || value.left_stick_x != input.left_stick_x ||
        value.left_stick_y != input.left_stick_y || value.right_stick_x != input.right_stick_x ||
        value.right_stick_y != input.right_stick_y || value.l2 != input.l2 || value.r2 != input.r2) dirty = true;
    down = next;
    input = value;
    sample_ms = now;
  }
  void decision(const char* name, bool ok, const char* why) {
    action = name; result = ok ? "ACCEPTED" : "REJECTED"; reason = why;
    ++decisions;
    if (ok) ++accepted; else ++rejected;
    dirty = true;
  }
  void format(char* out, size_t capacity, int deadzone, bool pending = false) const {
    char held[96], pressed[96], released[96];
    names(down, held, sizeof(held));
    names(pending ? pending_press : last_press, pressed, sizeof(pressed));
    names(pending ? pending_release : last_release, released, sizeof(released));
    const bool left_neutral = input.left_stick_x >= -deadzone && input.left_stick_x <= deadzone &&
      input.left_stick_y >= -deadzone && input.left_stick_y <= deadzone;
    const bool right_neutral = input.right_stick_x >= -deadzone && input.right_stick_x <= deadzone &&
      input.right_stick_y >= -deadzone && input.right_stick_y <= deadzone;
    snprintf(out, capacity,
      "PS4 init=%s connected=%d event=%s connect/disconnect=%lu/%lu sample_ms=%lu\n"
      "PS4 raw LX=%d LY=%d RX=%d RY=%d L2=%u R2=%u neutral_left/right=%d/%d\n"
      "PS4 buttons held=%s last_press=%s last_release=%s events=%lu\n"
      "PS4 decision=%lu action=%s result=%s reason=%s accepted/rejected=%lu/%lu\n",
      !init_attempted ? "NOT_ATTEMPTED" : (init_ok ? "OK" : "FAILED"), connected, event,
      static_cast<unsigned long>(connects), static_cast<unsigned long>(disconnects),
      static_cast<unsigned long>(sample_ms), input.left_stick_x,input.left_stick_y,
      input.right_stick_x,input.right_stick_y,input.l2,input.r2,left_neutral,right_neutral,
      held,pressed,released,static_cast<unsigned long>(button_events),
      static_cast<unsigned long>(decisions),action,result,reason,
      static_cast<unsigned long>(accepted),static_cast<unsigned long>(rejected));
  }
};

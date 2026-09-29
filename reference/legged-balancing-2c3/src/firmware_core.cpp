#include "firmware_core.h"

#include <stdio.h>
#include <string.h>

FirmwareCore::FirmwareCore(FiringOutput& firing, AudioTask& audio)
    : firing_(firing), audio_(audio), state_(State::kDisarmed) {}

void FirmwareCore::safeStop() { firing_.stop(); state_ = State::kDisarmed; }

void FirmwareCore::firingCompleted(bool success) {
  firing_.stop();
  state_ = success ? State::kArmed : State::kError;
}

void FirmwareCore::processCommand(const char* command, ResponseSink& out) {
  if (!command) { out.line("ERR COMMAND"); return; }
  if (!strcmp(command, "PING")) { out.line("OK PONG"); return; }
  if (!strcmp(command, "HELP")) {
    out.line("OK HELP PING STATUS ARM DISARM FIRE TEST STOP RESET SOUND"); return;
  }
  if (!strcmp(command, "STATUS")) {
    const char* names[] = {"DISARMED", "ARMED", "FIRING", "ERROR"};
    char text[32];
    snprintf(text, sizeof(text), "OK STATUS %s", names[static_cast<int>(state_)]);
    out.line(text); return;
  }
  if (!strcmp(command, "ARM")) {
    if (state_ == State::kFiring) { out.line("ERR BUSY"); return; }
    state_ = State::kArmed; out.line("OK ARMED"); return;
  }
  if (!strcmp(command, "DISARM") || !strcmp(command, "STOP")) {
    safeStop(); audio_.stop(); out.line(!strcmp(command, "STOP") ? "OK STOPPED" : "OK DISARMED"); return;
  }
  if (!strcmp(command, "RESET")) {
    safeStop(); audio_.stop(); out.line("OK RESET"); return;
  }
  if (!strcmp(command, "SOUND STOP")) { audio_.stop(); out.line("OK SOUND STOPPED"); return; }
  unsigned value = 0; char tail = 0;
  if (sscanf(command, "SOUND %u%c", &value, &tail) == 1 && value <= 2) {
    out.line(audio_.play(static_cast<uint8_t>(value)) ? "OK SOUND" : "ERR SOUND"); return;
  }
  bool test = !strncmp(command, "TEST ", 5);
  bool fire = !strncmp(command, "FIRE ", 5);
  if ((test || fire) && sscanf(command + 5, "%u%c", &value, &tail) == 1 && value >= 1 && value <= 9) {
    if (state_ != State::kArmed) { out.line("ERR NOT ARMED"); return; }
    if (!firing_.start(static_cast<uint8_t>(value), test)) {
      firing_.stop(); state_ = State::kError; out.line("ERR FIRE START"); return;
    }
    state_ = State::kFiring; out.line(test ? "OK TEST" : "OK FIRE"); return;
  }
  out.line("ERR COMMAND");
}

#pragma once

#include <stddef.h>
#include <stdint.h>

class ResponseSink {
 public:
  virtual ~ResponseSink() = default;
  virtual bool line(const char* text) = 0;
};

class FiringOutput {
 public:
  virtual ~FiringOutput() = default;
  virtual bool start(uint8_t status, bool test) = 0;
  virtual void stop() = 0;
};

class AudioTask {
 public:
  virtual ~AudioTask() = default;
  virtual bool play(uint8_t sound) = 0;
  virtual void stop() = 0;
};

class FirmwareCore {
 public:
  enum class State { kDisarmed, kArmed, kFiring, kError };
  FirmwareCore(FiringOutput& firing, AudioTask& audio);
  void processCommand(const char* command, ResponseSink& response);
  void firingCompleted(bool success);
  State state() const { return state_; }
 private:
  void safeStop();
  FiringOutput& firing_;
  AudioTask& audio_;
  State state_;
};

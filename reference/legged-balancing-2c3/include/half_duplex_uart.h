#pragma once
#ifdef ARDUINO
#include <Arduino.h>
#include "command_bridge.h"

class HalfDuplexUart {
 public:
  HalfDuplexUart(HardwareSerial& uart, FirmwareCore& core);
  void begin();
  void update();
 private:
  enum class LogKind : uint8_t { kRx, kEcho, kPingResponse, kStatus };
  struct LogRecord {
    LogKind kind;
    uint16_t size;
    uint8_t data[sts::kMaxPacketSize];
  };
  void send(const uint8_t* data, size_t size);
  void discardEcho(size_t size);
  void captureRx(uint8_t byte, uint32_t now_us);
  void finishRxLog();
  void queueLog(LogKind kind, const uint8_t* data = nullptr, size_t size = 0);
  void serviceUsbLog(uint32_t now_us);
  char nextLogCharacter();
  HardwareSerial& uart_; FirmwareCore& core_; sts::Parser parser_;
  uint8_t response_[sts::kMaxPacketSize];
  uint8_t rx_log_[sts::kMaxPacketSize];
  size_t rx_log_size_ = 0;
  uint32_t last_bus_activity_us_ = 0;
  static constexpr size_t kLogQueueSize = 4;
  LogRecord logs_[kLogQueueSize];
  size_t log_head_ = 0, log_tail_ = 0, log_count_ = 0;
  size_t log_output_position_ = 0;
  char log_header_[48];
  size_t log_header_size_ = 0;
};
#endif

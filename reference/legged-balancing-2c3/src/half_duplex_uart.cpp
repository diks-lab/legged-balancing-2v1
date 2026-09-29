#ifdef ARDUINO
#include "half_duplex_uart.h"
#include "hardware_config.h"
#include <stdio.h>

HalfDuplexUart::HalfDuplexUart(HardwareSerial& uart, FirmwareCore& core) : uart_(uart), core_(core) {}

void HalfDuplexUart::begin() {
  digitalWrite(HALF_DUPLEX_OE_PIN, HIGH);
  pinMode(HALF_DUPLEX_OE_PIN, OUTPUT);
  uart_.begin(HALF_DUPLEX_BAUD, SERIAL_8N1, HALF_DUPLEX_RX_PIN, HALF_DUPLEX_TX_PIN);
}

void HalfDuplexUart::discardEcho(size_t size) {
  const uint32_t deadline = micros() + static_cast<uint32_t>(size * 12 + 30);
  uint8_t echo[sts::kMaxPacketSize];
  size_t echo_size = 0;
  while (static_cast<int32_t>(deadline - micros()) > 0) {
    while (uart_.available()) {
      const uint8_t byte = static_cast<uint8_t>(uart_.read());
      if (echo_size < sizeof(echo)) echo[echo_size++] = byte;
      last_bus_activity_us_ = micros();
    }
    yield();
  }
  while (uart_.available()) {
    const uint8_t byte = static_cast<uint8_t>(uart_.read());
    if (echo_size < sizeof(echo)) echo[echo_size++] = byte;
    last_bus_activity_us_ = micros();
  }
  if (HALF_DUPLEX_RX_LOG_ENABLED && echo_size)
    queueLog(LogKind::kEcho, echo, echo_size);
  parser_.reset();
}

void HalfDuplexUart::send(const uint8_t* data, size_t size) {
  digitalWrite(HALF_DUPLEX_OE_PIN, LOW);
  delayMicroseconds(3);
  uart_.write(data, size);
  uart_.flush();  // waits through the final stop bit on ESP32 Arduino
  digitalWrite(HALF_DUPLEX_OE_PIN, HIGH);
  discardEcho(size);
}

void HalfDuplexUart::captureRx(uint8_t byte, uint32_t now_us) {
  if (!HALF_DUPLEX_RX_LOG_ENABLED) return;
  if (rx_log_size_ == sizeof(rx_log_)) finishRxLog();
  rx_log_[rx_log_size_++] = byte;
  last_bus_activity_us_ = now_us;
}

void HalfDuplexUart::finishRxLog() {
  if (!rx_log_size_) return;

  // STSパケットの先頭 FF FF に続くIDが3の場合だけ記録する。
  // 不完全な受信列でも、ここまで判別できれば表示する。
  if (HALF_DUPLEX_RX_LOG_ENABLED &&
      rx_log_size_ >= 3 &&
      rx_log_[0] == 0xFF &&
      rx_log_[1] == 0xFF &&
      rx_log_[2] == sts::kDeviceId) {
    queueLog(LogKind::kRx, rx_log_, rx_log_size_);
  }

  rx_log_size_ = 0;
}

void HalfDuplexUart::queueLog(LogKind kind, const uint8_t* data, size_t size) {
  if (log_count_ == kLogQueueSize) {
    return;
  }
  LogRecord& record = logs_[log_tail_];
  record.kind = kind;
  record.size = static_cast<uint16_t>(
      size > sizeof(record.data) ? sizeof(record.data) : size);
  for (size_t i = 0; i < record.size; ++i) {
    record.data[i] = data[i];
  }
  log_tail_ = (log_tail_ + 1) % kLogQueueSize;
  ++log_count_;
}

char HalfDuplexUart::nextLogCharacter() {
  LogRecord& record = logs_[log_head_];
  if (!log_output_position_) {
    const char* label = record.kind == LogKind::kRx ? "RX" : "TX ECHO";
    if (record.kind == LogKind::kPingResponse)
      log_header_size_ = snprintf(log_header_, sizeof(log_header_), "ID3 PING response sent\r\n");
    else if (record.kind == LogKind::kStatus)
      log_header_size_ = snprintf(log_header_, sizeof(log_header_), "BUS RX logging: %s\r\n",
                                  HALF_DUPLEX_RX_LOG_ENABLED ? "enabled" : "disabled");
    else
      log_header_size_ = snprintf(log_header_, sizeof(log_header_), "%s %u bytes:", label,
                                  static_cast<unsigned>(record.size));
  }
  if (log_output_position_ < log_header_size_)
    return log_header_[log_output_position_++];
  if (record.kind == LogKind::kPingResponse || record.kind == LogKind::kStatus) return '\0';
  const size_t data_position = log_output_position_ - log_header_size_;
  const size_t encoded_data_size = static_cast<size_t>(record.size) * 3;
  if (data_position < encoded_data_size) {
    const size_t phase = data_position % 3;
    const uint8_t byte = record.data[data_position / 3];
    ++log_output_position_;
    if (phase == 0) return ' ';
    const uint8_t nibble = phase == 1 ? byte >> 4 : byte & 0x0f;
    return nibble < 10 ? '0' + nibble : 'A' + nibble - 10;
  }
  if (data_position == encoded_data_size) {
    ++log_output_position_;
    return '\r';
  }
  if (data_position == encoded_data_size + 1) {
    ++log_output_position_;
    return '\n';
  }
  return '\0';
}

void HalfDuplexUart::serviceUsbLog(uint32_t now_us) {
  while (Serial.available()) {
    if (Serial.read() == '?') queueLog(LogKind::kStatus);
  }
  // Wait for a complete bus quiet interval, then emit only a small non-blocking
  // slice. update() gets another chance to drain the 1 Mbps UART before more USB.
  if (!log_count_ || uart_.available() ||
      static_cast<uint32_t>(now_us - last_bus_activity_us_) <= sts::kReceiveTimeoutUs)
    return;
  int writable = Serial.availableForWrite();
  if (writable > 24) writable = 24;
  while (writable-- > 0 && log_count_) {
    const char character = nextLogCharacter();
    if (character) {
      Serial.write(character);
      continue;
    }
    log_head_ = (log_head_ + 1) % kLogQueueSize;
    --log_count_;
    log_output_position_ = log_header_size_ = 0;
  }
}

void HalfDuplexUart::update() {
  const uint32_t start_us = micros();
  parser_.update(start_us);
  if (rx_log_size_ && static_cast<uint32_t>(start_us - last_bus_activity_us_) > sts::kReceiveTimeoutUs)
    finishRxLog();
  while (uart_.available()) {
    const uint32_t now_us = micros();
    const uint8_t byte = static_cast<uint8_t>(uart_.read());
    captureRx(byte, now_us);
    sts::Packet packet;
    const auto result = parser_.push(byte, now_us, packet);
    if (result != sts::ParseResult::kPacket) continue;
    finishRxLog();
    const size_t count = handlePacket(packet, core_, response_, sizeof(response_));
    if (count) {
      send(response_, count);
      if (packet.id == sts::kDeviceId && packet.instruction == sts::kPingInstruction)
        queueLog(LogKind::kPingResponse);
    }
  }
  serviceUsbLog(micros());
}
#endif

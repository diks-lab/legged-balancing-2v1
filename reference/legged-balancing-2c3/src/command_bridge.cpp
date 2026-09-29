#include "command_bridge.h"
#include <string.h>

FixedResponseSink::FixedResponseSink(uint8_t* buffer, size_t capacity)
    : buffer_(buffer), capacity_(capacity), size_(0), overflowed_(false) {}

bool FixedResponseSink::line(const char* text) {
  const size_t n = strlen(text);
  const size_t separator = size_ ? 1 : 0;
  if (n + separator > capacity_ - size_) { overflowed_ = true; return false; }
  if (separator) buffer_[size_++] = '\n';
  memcpy(buffer_ + size_, text, n); size_ += n; return true;
}

size_t handlePacket(const sts::Packet& request, FirmwareCore& core,
                    uint8_t* response, size_t response_capacity) {
  if (request.id != sts::kDeviceId) return 0;
  if (request.instruction == sts::kPingInstruction && request.parameter_count == 0)
    return sts::makeStatusPacket(0, nullptr, 0, response, response_capacity);
  if (request.instruction != sts::kAsciiInstruction || request.parameter_count == 0 ||
      request.parameter_count >= sts::kMaxParameters) return sts::makeStatusPacket(2, nullptr, 0, response, response_capacity);
  char command[sts::kMaxParameters];
  for (size_t i = 0; i < request.parameter_count; ++i) {
    if (request.parameters[i] == '\r' || request.parameters[i] == '\n' ||
        request.parameters[i] == 0) return sts::makeStatusPacket(2, nullptr, 0, response, response_capacity);
    command[i] = static_cast<char>(request.parameters[i]);
  }
  command[request.parameter_count] = 0;
  uint8_t payload[sts::kMaxParameters];
  FixedResponseSink sink(payload, sizeof(payload));
  core.processCommand(command, sink);
  if (sink.overflowed()) return sts::makeStatusPacket(3, nullptr, 0, response, response_capacity);
  return sts::makeStatusPacket(0, payload, sink.size(), response, response_capacity);
}

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace sts {

constexpr uint8_t kDeviceId = 0x03;
constexpr uint8_t kBroadcastId = 0xfe;
constexpr uint8_t kPingInstruction = 0x01;
constexpr uint8_t kAsciiInstruction = 0xa0;
constexpr size_t kMaxParameters = 253;  // LENGTH includes instruction/error + checksum.
constexpr size_t kMaxPacketSize = kMaxParameters + 6;
constexpr uint32_t kReceiveTimeoutUs = 3000;

uint8_t checksum(uint8_t id, uint8_t length, uint8_t command,
                 const uint8_t* parameters, size_t parameter_count);

// Returns encoded byte count, or zero if the supplied fields cannot fit.
size_t makeStatusPacket(uint8_t error, const uint8_t* parameters,
                        size_t parameter_count, uint8_t* output,
                        size_t output_capacity);

struct Packet {
  uint8_t id;
  uint8_t instruction;
  uint8_t parameters[kMaxParameters];
  size_t parameter_count;
};

enum class ParseResult { kNone, kPacket, kRejected };

// A bounded, allocation-free streaming parser. Timestamps are monotonic µs.
class Parser {
 public:
  Parser();
  ParseResult push(uint8_t byte, uint32_t now_us, Packet& packet);
  void update(uint32_t now_us);
  void reset();

 private:
  enum class State { kHeader1, kHeader2, kId, kLength, kBody };
  State state_;
  uint8_t id_;
  uint8_t length_;
  uint8_t body_[kMaxParameters + 2];
  size_t body_count_;
  uint32_t last_byte_us_;
  bool receiving_;
};

}  // namespace sts

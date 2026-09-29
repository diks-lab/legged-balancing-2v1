#include "sts_packet.h"

namespace sts {

uint8_t checksum(uint8_t id, uint8_t length, uint8_t command,
                 const uint8_t* parameters, size_t parameter_count) {
  uint16_t sum = id + length + command;
  for (size_t i = 0; i < parameter_count; ++i) sum += parameters[i];
  return static_cast<uint8_t>(~sum);
}

size_t makeStatusPacket(uint8_t error, const uint8_t* parameters,
                        size_t parameter_count, uint8_t* output,
                        size_t output_capacity) {
  if (parameter_count > kMaxParameters || parameter_count + 2 > 255) return 0;
  const size_t total = parameter_count + 6;
  if (!output || output_capacity < total) return 0;
  const uint8_t length = static_cast<uint8_t>(parameter_count + 2);
  output[0] = output[1] = 0xff;
  output[2] = kDeviceId;
  output[3] = length;
  output[4] = error;
  for (size_t i = 0; i < parameter_count; ++i) output[5 + i] = parameters[i];
  output[5 + parameter_count] = checksum(kDeviceId, length, error, parameters,
                                          parameter_count);
  return total;
}

Parser::Parser() { reset(); }

void Parser::reset() {
  state_ = State::kHeader1;
  id_ = length_ = 0;
  body_count_ = 0;
  last_byte_us_ = 0;
  receiving_ = false;
}

void Parser::update(uint32_t now_us) {
  if (receiving_ && static_cast<uint32_t>(now_us - last_byte_us_) > kReceiveTimeoutUs)
    reset();
}

ParseResult Parser::push(uint8_t byte, uint32_t now_us, Packet& packet) {
  update(now_us);
  last_byte_us_ = now_us;
  receiving_ = true;
  switch (state_) {
    case State::kHeader1:
      if (byte == 0xff) state_ = State::kHeader2;
      else receiving_ = false;
      return ParseResult::kNone;
    case State::kHeader2:
      if (byte == 0xff) state_ = State::kId;
      else { state_ = State::kHeader1; receiving_ = false; }
      return ParseResult::kNone;
    case State::kId:
      // A third FF can be the first byte of a new header.
      if (byte == 0xff) return ParseResult::kNone;
      id_ = byte;
      state_ = State::kLength;
      return ParseResult::kNone;
    case State::kLength:
      length_ = byte;
      if (length_ < 2 || length_ > kMaxParameters + 2) {
        reset();
        return ParseResult::kRejected;
      }
      body_count_ = 0;
      state_ = State::kBody;
      return ParseResult::kNone;
    case State::kBody:
      body_[body_count_++] = byte;
      if (body_count_ != length_) return ParseResult::kNone;
      const size_t parameter_count = length_ - 2;
      const uint8_t expected = checksum(id_, length_, body_[0], body_ + 1,
                                        parameter_count);
      if (body_[length_ - 1] != expected) {
        reset();
        return ParseResult::kRejected;
      }
      packet.id = id_;
      packet.instruction = body_[0];
      packet.parameter_count = parameter_count;
      for (size_t i = 0; i < parameter_count; ++i) packet.parameters[i] = body_[i + 1];
      reset();
      return ParseResult::kPacket;
  }
  reset();
  return ParseResult::kRejected;
}

}  // namespace sts

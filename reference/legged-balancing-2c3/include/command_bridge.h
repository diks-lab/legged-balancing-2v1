#pragma once
#include "firmware_core.h"
#include "sts_packet.h"

class FixedResponseSink : public ResponseSink {
 public:
  FixedResponseSink(uint8_t* buffer, size_t capacity);
  bool line(const char* text) override;
  size_t size() const { return size_; }
  bool overflowed() const { return overflowed_; }
 private:
  uint8_t* buffer_; size_t capacity_; size_t size_; bool overflowed_;
};

// Returns response packet length. Zero means that no reply is permitted.
size_t handlePacket(const sts::Packet& request, FirmwareCore& core,
                    uint8_t* response, size_t response_capacity);

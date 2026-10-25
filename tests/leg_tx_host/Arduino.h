#pragma once
#include <stdint.h>
#include <stddef.h>
#include <vector>
inline uint32_t millis(){return 0;}
class HardwareSerial {
 public:
 std::vector<uint8_t> bytes;
 int read(){return -1;}
 size_t write(const uint8_t*p,size_t n){bytes.insert(bytes.end(),p,p+n);return n;}
};

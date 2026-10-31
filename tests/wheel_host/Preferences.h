#pragma once
#include "Arduino.h"
#include <array>
#include <cassert>
inline std::array<std::vector<uint8_t>,2> nvsSlots;
inline bool nvsOpenOK=true,nvsReadOK=true;
inline unsigned nvsOpens=0;
class Preferences {
 public:
  bool begin(const char* name,bool readOnly) {
    assert(std::string(name)=="wheel-foc" && readOnly); ++nvsOpens; return nvsOpenOK;
  }
  void end() {}
  size_t getBytesLength(const char* key) { return nvsSlots[std::string(key)=="slot0"?0:1].size(); }
  size_t getBytes(const char* key,void* p,size_t n) {
    if(!nvsReadOK) return 0;
    const auto& bytes=nvsSlots[std::string(key)=="slot0"?0:1];
    assert(bytes.size()==n); memcpy(p,bytes.data(),n); return n;
  }
  // Deliberately no write API: production diagnostic must compile read-only.
};

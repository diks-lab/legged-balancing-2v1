#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

namespace wheelcal {
// Fixed ABI: two wheels are one atomic NVS blob; no partial wheel updates.
struct Wheel { int32_t direction=0; uint32_t poles=0; float zero=0; uint32_t pp=0; };
struct Record {
  uint32_t magic=0x434f4632, version=1, config=0, generation=0;
  Wheel wheels[2]; uint32_t checksum=0;
};
static_assert(sizeof(Record)==52,"calibration storage ABI");
inline uint32_t crc(const Record& r) {
  uint32_t v=0xffffffffu;
  const auto* p=reinterpret_cast<const uint8_t*>(&r);
  for(size_t i=0;i<offsetof(Record,checksum);++i){v^=p[i];for(int b=0;b<8;++b)v=(v>>1)^(0xedb88320u&uint32_t(-int32_t(v&1)));}
  return ~v;
}
inline bool valid(const Record&r,uint32_t config){
  if(r.magic!=0x434f4632||r.version!=1||r.config!=config||r.checksum!=crc(r))return false;
  for(const auto&w:r.wheels)if((w.direction!=1&&w.direction!=-1)||w.poles!=7||w.pp!=1||!std::isfinite(w.zero)||w.zero<0||w.zero>=6.28318530718f)return false;
  return true;
}
enum class Load { Valid, Missing, Invalid };
// Backend returns actual byte count. Alternate slots preserve the last good blob
// on write failure/power loss; a readback must match before accepting a save.
template<class Backend> Load load(Backend& b,uint32_t config,Record& out,int& slot){
  bool found=false,any=false;Record r;slot=-1;
  for(int i=0;i<2;++i){const size_t n=b.read(i,&r,sizeof(r));any|=n!=0;
    if(n==sizeof(r)&&valid(r,r.config)&&(!found||int32_t(r.generation-out.generation)>0)){out=r;slot=i;found=true;}}
  return found?(out.config==config?Load::Valid:Load::Invalid):(any?Load::Invalid:Load::Missing);
}
template<class Backend> bool save(Backend&b,uint32_t config,Record candidate,Record&current,int&slot){
  candidate.config=config;candidate.generation=slot<0?1:current.generation+1;candidate.checksum=crc(candidate);
  if(!valid(candidate,config))return false;
  const int next=slot==0?1:0;
  if(!b.write(next,&candidate,sizeof(candidate)))return false;
  Record verify;
  if(b.read(next,&verify,sizeof(verify))!=sizeof(verify)||memcmp(&candidate,&verify,sizeof(verify))||!valid(verify,config))return false;
  current=verify;slot=next;return true;
}
}

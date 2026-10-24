#pragma once
#include <stdint.h>
#include <string.h>
namespace leg {
// SMS_STS::ReadPos uses bit 15 as a sign bit with a 15-bit magnitude.
inline int16_t decodePosition(uint16_t word){
 const int16_t magnitude=int16_t(word&0x7fff);
 return word&0x8000?int16_t(-magnitude):magnitude;
}
enum RxIssue : uint8_t { RxHeader=1, RxId=2, RxLength=4, RxStatus=8,
 RxChecksum=16, RxTimeout=32 };
struct RxFailure {
 uint32_t ms=0,elapsed_ms=0;
 uint16_t received=0;
 uint8_t target=0,issues=0,frame_size=0,raw_size=0,echoes=0,skipped=0;
 bool truncated=false;
 uint8_t expected_checksum=0,raw[32]{},frame[8]{},received_checksum=0;
 bool checksum_ready=false;
};
// Fixed parser for a two-byte STS position reply. The 24-byte candidate/echo
// slots fit the largest frame we transmit (two-servo SyncWrite), not arbitrary RX.
class ServoReply {
 public:
 enum Result { Pending,Good,Bad };
 void noteTransmit(const uint8_t*p,unsigned n){
  if(n<=sizeof(echo_)&&!(n==8&&p[4]==2)){
   memcpy(echo_,p,n);echo_size_=uint8_t(n);
  }
 }
 void begin(uint8_t target,uint32_t now){
  record_=RxFailure{};record_.target=target;start_=now;clearFrame();
 }
 void clearFrame(){used_=0;}
 bool expired(uint32_t now,uint32_t limit)const{return uint32_t(now-start_)>limit;}
 void timeout(){record_.issues|=RxTimeout;}
 Result feed(uint8_t byte){
  if(record_.received!=UINT16_MAX)++record_.received;
  if(record_.raw_size<sizeof(record_.raw))record_.raw[record_.raw_size++]=byte;
  else record_.truncated=true;
  if(!used_&&byte!=255)return skip();
  if(used_==1&&byte!=255){used_=0;skip();return skip();}
  // Preserve the last two FF bytes of a longer header run; FF is not a servo ID.
  if(used_==2&&byte==255)return skip();
  frame_[used_++]=byte;
  if(used_<4)return Pending;
  const bool broadcast_prefix=echo_size_&&used_<=echo_size_&&!memcmp(frame_,echo_,used_);
  if(broadcast_prefix&&used_==echo_size_)return echo();
  if(frame_[3]!=4){
   if(broadcast_prefix)return Pending;
   record_.issues|=RxLength;
   if(frame_[2]!=record_.target)record_.issues|=RxId;
   // Inspect a bounded, declared error packet as well (e.g. length=2 -> 6 B).
   // A wrong length is never accepted, even with a correct checksum.
   if(frame_[3]<2||frame_[3]>sizeof(frame_)-4)return Bad;
  }
  const unsigned total=unsigned(frame_[3])+4;
  if(used_<total)return Pending;
  uint8_t request[]={255,255,record_.target,4,2,56,2,0};
  request[7]=uint8_t(~uint8_t(request[2]+request[3]+request[4]+request[5]+request[6]));
  if(total==sizeof(request)&&!memcmp(frame_,request,sizeof(request)))return echo();
  if(frame_[2]!=record_.target)record_.issues|=RxId;
  if(frame_[4])record_.issues|=RxStatus;
  uint8_t sum=0;for(unsigned i=2;i+1<total;++i)sum+=frame_[i];
  record_.expected_checksum=uint8_t(~sum);
  record_.received_checksum=frame_[total-1];record_.checksum_ready=true;
  if(record_.received_checksum!=record_.expected_checksum)record_.issues|=RxChecksum;
  return record_.issues?Bad:Good;
 }
 uint16_t positionWord()const{return uint16_t(frame_[5])|(uint16_t(frame_[6])<<8);}
 int16_t position()const{return decodePosition(positionWord());}
 RxFailure failure(uint32_t now)const{
  RxFailure result=record_;
  result.ms=now;result.elapsed_ms=now-start_;
  result.frame_size=used_;
  memcpy(result.frame,frame_,used_>sizeof(result.frame)?sizeof(result.frame):used_);
  return result;
 }
 private:
 RxFailure record_{};
 uint32_t start_=0;
 uint8_t frame_[24]{},echo_[24]{},used_=0,echo_size_=0;
 Result skip(){
  if(record_.skipped!=UINT8_MAX)++record_.skipped;
  if(record_.skipped>10){record_.issues|=RxHeader;return Bad;}
  return Pending;
 }
 Result echo(){
  if(record_.echoes!=UINT8_MAX)++record_.echoes;
  clearFrame();return Pending; // An echo is never evidence of a valid servo reply.
 }
};
}

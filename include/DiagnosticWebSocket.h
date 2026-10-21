#pragma once
#include "DiagnosticTelemetry.h"
#include <errno.h>
namespace diagnostic {
enum class WsEvent:uint8_t { Accepted,Upgraded,Subscribed,FirstPrepared,FirstWrite,FirstSent,Closed };
enum class WsReason:uint8_t { None,LocalClose,WiFiLost,IoClosed,HandshakeDeadline,HandshakeIdle,HandshakeInvalid,FrameInvalid,FrameTooBig,RxCapacity,RxDeadline,RxIdle,ReadError,PeerClosed,WriteError,WriteDeadline,WriteIdle,CallLimit,RetryLimit,JsonError };
inline const char* wsEventName(unsigned n){static const char*const names[]={"ACCEPTED","UPGRADED","SUBSCRIBED","FIRST_PREPARED","FIRST_WRITE","FIRST_SENT","CLOSED"};return n<7?names[n]:"UNKNOWN";}
inline const char* wsReasonName(unsigned n){static const char*const names[]={"NONE","LOCAL_CLOSE","WIFI_LOST","IO_CLOSED","HANDSHAKE_DEADLINE","HANDSHAKE_IDLE","HANDSHAKE_INVALID","FRAME_INVALID","FRAME_TOO_BIG","RX_CAPACITY","RX_DEADLINE","RX_IDLE","READ_ERROR","PEER_CLOSED","WRITE_ERROR","WRITE_DEADLINE","WRITE_IDLE","CALL_LIMIT","RETRY_LIMIT","JSON_ERROR"};return n<20?names[n]:"UNKNOWN";}
struct WsObservation { WsEvent event;WsReason reason;uint32_t ms,generation,seq,sent,total,elapsed;int error;uint16_t calls,retries; };

// One transport/one slot. IO exposes nonblocking raw socket read/write, close,
// millis, handshake SHA1/base64, and a coherent latest snapshot; no heap here.
template<class IO> class WebSocket {
 char rx_[1025]{},tx_[2052]{};size_t rxSize_=0,txSize_=0,txOffset_=0;
 uint32_t connectedAt_=0,rxProgress_=0,frameAt_=0,txAt_=0,txProgress_=0,nextTry_=0;
 uint32_t due_=0,statusDue_=0,mask_=0,generation_=0,seq_=0,dropped_=0;
 uint16_t calls_=0,retries_=0;bool active_=false,upgraded_=false,handshakeSent_=false,txSample_=false;
 uint32_t pendingSeq_=0;int lastError_=0;
 bool firstPrepared_=false,firstWritten_=false,txFirst_=false;
 char boot_[17]{};
 void emit(IO&io,WsEvent e,WsReason reason=WsReason::None){io.observe({e,reason,io.now(),generation_,txSample_?pendingSeq_:seq_,uint32_t(txOffset_),uint32_t(txSize_),txSize_?uint32_t(io.now()-txAt_):0,lastError_,calls_,retries_});}
 void resetReceive(){rxSize_=0;frameAt_=0;}
 void end(IO&io,WsReason reason=WsReason::LocalClose){if(active_)emit(io,WsEvent::Closed,reason);if(txSample_&&txSize_)++dropped_;txSample_=false;io.close();active_=upgraded_=handshakeSent_=false;mask_=0;rxSize_=txSize_=txOffset_=0;}
 bool frame(IO&io,unsigned opcode,size_t length){
  if(txSize_||length>2048)return false;
  txSample_=false;txFirst_=false;lastError_=0;
  const size_t h=length<126?2:4;memmove(tx_+h,tx_,length);tx_[0]=char(0x80|opcode);tx_[1]=length<126?char(length):126;
  if(h==4){tx_[2]=char(length>>8);tx_[3]=char(length);}txSize_=length+h;txOffset_=0;calls_=retries_=0;txAt_=txProgress_=io.now();nextTry_=txAt_;return true;
 }
 bool upgrade(IO&io){
  rx_[rxSize_]=0;
  if(strncmp(rx_,"GET /diagnostics HTTP/1.1\r\n",27))return false;
  char key[25]{};bool upgrade=false,connection=false,version=false;unsigned keys=0;
  char*line=strstr(rx_,"\r\n")+2;
  while(*line){char*end=strstr(line,"\r\n");if(!end)return false;if(end==line)break;*end=0;char*colon=strchr(line,':');if(!colon)return false;*colon=0;char*value=colon+1;while(*value==' '||*value=='\t')++value;
   if(!strcasecmp(line,"Sec-WebSocket-Key")){if(++keys!=1||strlen(value)!=24)return false;memcpy(key,value,25);}
   if(!strcasecmp(line,"Upgrade"))upgrade=!strcasecmp(value,"websocket");
   if(!strcasecmp(line,"Connection")){for(char*p=value;*p;++p)if(*p>='A'&&*p<='Z')*p+=32;connection=strstr(value,"upgrade");}
   if(!strcasecmp(line,"Sec-WebSocket-Version"))version=!strcmp(value,"13");
   line=end+2;
  }
  char accept[29];if(!keys||!upgrade||!connection||!version||!io.acceptKey(key,accept))return false;
  int n=snprintf(tx_,sizeof(tx_),"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: %s\r\n\r\n",accept);
  if(n<=0||size_t(n)>=sizeof(tx_))return false;
  txSize_=n;txOffset_=0;calls_=retries_=0;txAt_=txProgress_=nextTry_=io.now();upgraded_=true;resetReceive();return true;
 }
 void incoming(IO&io){
  if(rxSize_<2)return;
  const uint8_t a=rx_[0],b=rx_[1];unsigned op=a&15;
  if((a&0xf0)!=0x80||!(b&128)||(op!=1&&op!=8&&op!=9&&op!=10)){end(io,WsReason::FrameInvalid);return;}
  size_t length=b&127,header=2;
  if(length==127){end(io,WsReason::FrameInvalid);return;}
  if(length==126){if(rxSize_<4)return;length=(uint8_t(rx_[2])<<8)|uint8_t(rx_[3]);header=4;if(length<126){end(io,WsReason::FrameInvalid);return;}}
  if(length>1000||(op>=8&&length>125)){end(io,WsReason::FrameTooBig);return;}
  if(rxSize_<header+4+length)return;
  for(size_t i=0;i<length;++i)rx_[header+4+i]^=rx_[header+(i%4)];
  char*payload=rx_+header+4;payload[length]=0;
  if(op==8){end(io,WsReason::PeerClosed);return;}
  if(op==9){memcpy(tx_,payload,length);frame(io,10,length);}
  if(op==1){Subscription s;const bool valid=!memchr(payload,0,length)&&SubscriptionParser(payload).parse(s);
   if(!valid){strcpy(tx_,"{\"type\":\"error\",\"reason\":\"invalid_subscription\"}");frame(io,1,strlen(tx_));}
   else {++generation_;mask_=s.stop?0:s.mask;due_=io.now()+50;
    if(!ack(tx_,2048,s,generation_,boot_)){end(io,WsReason::JsonError);return;}frame(io,1,strlen(tx_));firstPrepared_=firstWritten_=false;emit(io,WsEvent::Subscribed);}
  }
  // recv is limited to precisely this frame, so no next-frame bytes are lost.
  resetReceive();
 }
 public:
 void setBoot(const char* id){memcpy(boot_,id,16);boot_[16]=0;}
 bool active()const{return active_;}bool ready()const{return handshakeSent_;}
 uint32_t dropped()const{return dropped_;}uint32_t generation()const{return generation_;}
 uint32_t mask()const{return mask_;}
 void connected(IO&io){active_=true;upgraded_=handshakeSent_=false;rxSize_=txSize_=txOffset_=0;mask_=0;connectedAt_=rxProgress_=io.now();statusDue_=io.now();firstPrepared_=firstWritten_=txFirst_=false;lastError_=0;emit(io,WsEvent::Accepted);}
 void disconnect(IO&io,WsReason reason=WsReason::LocalClose){end(io,reason);}
 void poll(IO&io){
  if(!active_)return;
  uint32_t now=io.now();
  if(!io.connected()){end(io,WsReason::IoClosed);return;}
  if(!upgraded_&&(uint32_t(now-connectedAt_)>=1000||uint32_t(now-rxProgress_)>=200)){end(io,uint32_t(now-connectedAt_)>=1000?WsReason::HandshakeDeadline:WsReason::HandshakeIdle);return;}
  if(handshakeSent_&&mask_&&int32_t(now-due_)>=0){
   uint32_t slots=uint32_t(now-due_)/50+1;due_+=slots*50;seq_+=slots;
   if(txSize_)dropped_+=slots;
   else {dropped_+=slots-1;Snapshot s;io.snapshot(s);if(!sample(tx_,2048,s,boot_,generation_,seq_,dropped_,mask_,io.now())){end(io,WsReason::JsonError);return;}frame(io,1,strlen(tx_));txSample_=true;pendingSeq_=seq_;txFirst_=!firstPrepared_;if(txFirst_){firstPrepared_=true;emit(io,WsEvent::FirstPrepared);}}
  }
  // JSON formatting, snapshot capture and observation may advance millis().
  // Compare against txAt_ only after taking a fresh timestamp (also wrap-safe).
  now=io.now();
  if(txSize_){
   if(uint32_t(now-txAt_)>=2000||uint32_t(now-txProgress_)>=500||calls_>=128||retries_>=64){end(io,uint32_t(now-txAt_)>=2000?WsReason::WriteDeadline:uint32_t(now-txProgress_)>=500?WsReason::WriteIdle:calls_>=128?WsReason::CallLimit:WsReason::RetryLimit);return;}
   if(int32_t(now-nextTry_)<0)return;
   ++calls_;int n=io.write(tx_+txOffset_,txSize_-txOffset_>256?256:txSize_-txOffset_);
   if(n>0){if(size_t(n)>txSize_-txOffset_){end(io,WsReason::WriteError);return;}txOffset_+=n;txProgress_=io.now();lastError_=0;if(txFirst_&&!firstWritten_){firstWritten_=true;emit(io,WsEvent::FirstWrite);}
    if(txOffset_==txSize_){if(txFirst_)emit(io,WsEvent::FirstSent);txFirst_=false;txSample_=false;txSize_=txOffset_=0;if(!handshakeSent_){handshakeSent_=true;resetReceive();emit(io,WsEvent::Upgraded);}}}
   else if(n==-EAGAIN||n==-EWOULDBLOCK||n==-EINTR||n==-ENOMEM){lastError_=-n;++retries_;nextTry_=io.now()+10;}
   else {lastError_=n<0?-n:0;end(io,WsReason::WriteError);return;}return;
  }
  // Receive at most 128 bytes per iteration, never parse a following frame inline.
  size_t wanted=128;
  if(handshakeSent_){
   if(rxSize_<2)wanted=2-rxSize_;
   else {size_t length=uint8_t(rx_[1])&127,h=2;
    if(length==127){end(io,WsReason::FrameInvalid);return;}
    if(length==126){if(rxSize_<4)wanted=4-rxSize_;else {length=(uint8_t(rx_[2])<<8)|uint8_t(rx_[3]);h=4;wanted=h+4+length-rxSize_;}}
    else wanted=h+4+length-rxSize_;
    if(length>1000){end(io,WsReason::FrameTooBig);return;}
   }
  }
  if(wanted>128)wanted=128;
  if(wanted>1024-rxSize_){end(io,WsReason::RxCapacity);return;}
  int n=io.read(rx_+rxSize_,wanted);
  now=io.now();
  if(n>0){if(!rxSize_)frameAt_=now;rxSize_+=n;rxProgress_=now;rx_[rxSize_]=0;
   if(!upgraded_){if(rxSize_>=4&&!memcmp(rx_+rxSize_-4,"\r\n\r\n",4)){if(!upgrade(io))end(io,WsReason::HandshakeInvalid);}}
   else incoming(io);
  }else if(n==0||(n!=-EAGAIN&&n!=-EWOULDBLOCK&&n!=-EINTR)){lastError_=n<0?-n:0;end(io,n==0?WsReason::PeerClosed:WsReason::ReadError);return;}
  if(handshakeSent_&&rxSize_&&(uint32_t(now-frameAt_)>=1000||uint32_t(now-rxProgress_)>=200)){end(io,uint32_t(now-frameAt_)>=1000?WsReason::RxDeadline:WsReason::RxIdle);return;}
  if(handshakeSent_&&!mask_&&!rxSize_&&int32_t(now-statusDue_)>=0){Snapshot s;io.snapshot(s);if(!sample(tx_,2048,s,boot_,generation_,seq_,dropped_,0,io.now())){end(io,WsReason::JsonError);return;}frame(io,1,strlen(tx_));statusDue_=now+1000;}
 }
};
}

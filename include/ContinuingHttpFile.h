#pragma once
#include "BoundedHttpResponse.h"

enum class HttpFilePhase : uint8_t { Idle, Header, Read, Body, Finished };
enum class HttpFileEnd : uint8_t {
  Complete, Deadline, CallLimit, RetryLimit, Disconnected, SocketError,
  ReadError, Invalid, SizeLimit, WifiLost, Cancelled
};
struct HttpFileMetrics {
  uint32_t elapsed=0;
  uint16_t bodySent=0,bodyTotal=0,wireSent=0,wireTotal=0;
  int16_t lastErrno=0;
  uint8_t calls=0,retries=0;
  HttpFileEnd reason=HttpFileEnd::Invalid;
  HttpFilePhase stage=HttpFilePhase::Idle;
};

// One fixed context owns shared client/file references across handler returns.
// One poll performs at most one nonblocking send OR one bounded file read.
template<class Client,class File> class ContinuingHttpFile {
  Client client_;
  File file_;
  char buffer_[256]{}; // Header then body; never two payload buffers.
  uint32_t start_=0;
  uint16_t length_=0,offset_=0;
  HttpFilePhase phase_=HttpFilePhase::Idle;
  HttpFileMetrics metrics_{};
 public:
  uint16_t route=0;
  bool first=false;
  ContinuingHttpFile()=default;
  ContinuingHttpFile(const ContinuingHttpFile&)=delete;
  ContinuingHttpFile& operator=(const ContinuingHttpFile&)=delete;
  ~ContinuingHttpFile(){if(active())finish(HttpFileEnd::Cancelled,start_);}
  bool active()const{return phase_==HttpFilePhase::Header||phase_==HttpFilePhase::Read||phase_==HttpFilePhase::Body;}
  bool finished()const{return phase_==HttpFilePhase::Finished;}
  Client& client(){return client_;}
  const HttpFileMetrics& metrics()const{return metrics_;}
  void finish(HttpFileEnd reason,uint32_t now){
    if(!active())return;
    metrics_.reason=reason;metrics_.stage=phase_;metrics_.elapsed=uint32_t(now-start_);
    // Capture terminal metrics BEFORE cleanup; no budget restart on progress.
    client_.stop();file_.close();
    client_=Client{};file_=File{};
    phase_=HttpFilePhase::Finished;
  }
  void acknowledge(){if(finished())phase_=HttpFilePhase::Idle;}
  bool begin(const Client& client,const File& file,const char* mime,uint32_t now,uint16_t id,bool isFirst){
    if(active()||finished())return false; // Leave existing ownership untouched.
    client_=client;file_=file;route=id;first=isFirst;start_=now;
    metrics_=HttpFileMetrics{};length_=offset_=0;phase_=HttpFilePhase::Header;
    const size_t size=file_.size();
    if(size>kMaxHttpWebFileBytes){finish(HttpFileEnd::SizeLimit,now);return true;}
    metrics_.bodyTotal=uint16_t(size);
    if(!mime){finish(HttpFileEnd::Invalid,now);return true;}
    const int n=snprintf(buffer_,192,
        "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n",mime,unsigned(size));
    if(n<0||n>=192){finish(HttpFileEnd::Invalid,now);return true;}
    length_=uint16_t(n);metrics_.wireTotal=uint16_t(n+size);
    return true;
  }
  template<class Transport> void poll(Transport& io){
    if(!active())return;
    const uint32_t now=io.now();
    if(uint32_t(now-start_)>=50){finish(HttpFileEnd::Deadline,now);return;}
    if(metrics_.calls>=64){finish(HttpFileEnd::CallLimit,now);return;}
    if(!io.connected()){finish(HttpFileEnd::Disconnected,now);return;}
    if(phase_==HttpFilePhase::Read){
      const size_t remaining=metrics_.bodyTotal-metrics_.bodySent;
      if(!remaining){finish(HttpFileEnd::Complete,now);return;}
      const size_t wanted=remaining>sizeof(buffer_)?sizeof(buffer_):remaining;
      const int got=file_.read(reinterpret_cast<uint8_t*>(buffer_),wanted);
      if(got<=0||size_t(got)>wanted){finish(HttpFileEnd::ReadError,io.now());return;}
      length_=uint16_t(got);offset_=0;phase_=HttpFilePhase::Body;
      return; // Yield to the HTTP task, no read-ahead or synchronous send loop.
    }
    ++metrics_.calls;
    const int sent=io.writeSome(buffer_+offset_,length_-offset_);
    if(sent>0){
      if(sent>int(length_-offset_)){finish(HttpFileEnd::SocketError,io.now());return;}
      offset_+=uint16_t(sent);metrics_.wireSent+=uint16_t(sent);
      if(phase_==HttpFilePhase::Body)metrics_.bodySent+=uint16_t(sent);
      if(offset_==length_){
        if(metrics_.bodySent==metrics_.bodyTotal){finish(HttpFileEnd::Complete,io.now());return;}
        phase_=HttpFilePhase::Read;
      }
      return;
    }
    if(sent<0)metrics_.lastErrno=int16_t(-sent);
    if(sent==-EAGAIN||sent==-EWOULDBLOCK||sent==-EINTR){
      if(++metrics_.retries>=3)finish(HttpFileEnd::RetryLimit,io.now());
      return; // Retain offset/buffer; retry only on a future task iteration.
    }
    finish(sent==0||sent==-ENOTCONN?HttpFileEnd::Disconnected:HttpFileEnd::SocketError,io.now());
  }
};

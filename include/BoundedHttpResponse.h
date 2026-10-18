#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>

// Transport: writeSome is nonblocking and returns negative errno on failure.
// No allocation, String, UART, controller locks or control-side waits.
struct HttpResponsePart { const char* data; size_t size; };
template<class Transport>
bool sendBoundedHttpResponse(Transport& io, int code,
                             const HttpResponsePart* parts, size_t count) {
  struct Close { Transport& io; ~Close(){io.stop();} } close{io};
  size_t bytes=0;
  if (!parts || count>8) return false;
  for(size_t i=0;i<count;++i){
    if ((!parts[i].data && parts[i].size) || parts[i].size>4096-bytes) return false;
    bytes+=parts[i].size;
  }
  char header[192];
  int n=snprintf(header,sizeof(header),
      "HTTP/1.1 %d %s\r\nContent-Type: text/plain; charset=utf-8\r\n"
      "Content-Length: %u\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n",
      code,code==200?"OK":"Service Unavailable",static_cast<unsigned>(bytes));
  if(n<0 || static_cast<size_t>(n)>=sizeof(header)) return false;
  const uint32_t start=io.now();
  unsigned calls=0,stalls=0;
  for(size_t i=0;i<=count;++i){
    const char* data=i==0?header:parts[i-1].data;
    size_t left=i==0?static_cast<size_t>(n):parts[i-1].size;
    while(left){
      if(calls++>=64 || static_cast<uint32_t>(io.now()-start)>=50) return false;
      const size_t chunk=left>512?512:left;
      const int sent=io.writeSome(data,chunk);
      if(sent>0){
        if(static_cast<size_t>(sent)>chunk) return false;
        data+=sent;left-=sent;
      }else if(sent==-EAGAIN || sent==-EWOULDBLOCK || sent==-EINTR){
        if(++stalls>=3) return false; // Total, not reset by partial progress.
        io.yieldOnce();
      }else return false;
    }
  }
  return true;
}

#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cerrno>

struct HttpResponsePart { const char* data; size_t size; };
enum class HttpSendResult : uint8_t { Complete, Invalid, HeaderFailed, ReadFailed, BodyFailed, FileTooLarge };
constexpr size_t kMaxHttpWebFileBytes=8192;

// One budget for header AND all body chunks. Transport writes are nonblocking.
// Destruction releases the connection on every path; no heap or String use.
template<class Transport> class BoundedHttpWriter {
  Transport& io;
  const uint32_t start;
  unsigned calls=0,stalls=0;
 public:
  explicit BoundedHttpWriter(Transport& transport):io(transport),start(io.now()){}
  ~BoundedHttpWriter(){io.stop();}
  bool available(){return calls<64 && uint32_t(io.now()-start)<50;}
  bool write(const char* data,size_t left){
    if(!data && left)return false;
    while(left){
      if(!available())return false;
      ++calls;
      const size_t chunk=left>512?512:left;
      const int sent=io.writeSome(data,chunk);
      if(sent>0){
        if(size_t(sent)>chunk)return false;
        data+=sent;left-=sent;
      }else if(sent==-EAGAIN || sent==-EWOULDBLOCK || sent==-EINTR){
        if(++stalls>=3)return false;
        io.yieldOnce();
      }else return false;
    }
    return true;
  }
  bool header(int code,const char* mime,size_t bytes){
    if(!mime)return false;
    const char* reason=code==200?"OK":code==202?"Accepted":code==404?"Not Found":
        code==409?"Conflict":"Service Unavailable";
    char text[192];
    int n=snprintf(text,sizeof(text),
        "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %u\r\n"
        "Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        code,reason,mime,static_cast<unsigned>(bytes));
    return n>=0 && size_t(n)<sizeof(text) && write(text,size_t(n));
  }
};

template<class Transport>
bool sendBoundedHttpResponse(Transport& io,int code,const HttpResponsePart* parts,size_t count,
                             HttpSendResult* result=nullptr){
  BoundedHttpWriter<Transport> writer(io);
  auto finish=[&](HttpSendResult r){if(result)*result=r;return r==HttpSendResult::Complete;};
  size_t bytes=0;
  if(!parts || count>8)return finish(HttpSendResult::Invalid);
  for(size_t i=0;i<count;++i){
    if((!parts[i].data&&parts[i].size)||parts[i].size>4096-bytes)return finish(HttpSendResult::Invalid);
    bytes+=parts[i].size;
  }
  if(!writer.header(code,"text/plain; charset=utf-8",bytes))return finish(HttpSendResult::HeaderFailed);
  for(size_t i=0;i<count;++i)
    if(!writer.write(parts[i].data,parts[i].size))return finish(HttpSendResult::BodyFailed);
  return finish(HttpSendResult::Complete);
}

template<class Transport,class File>
HttpSendResult sendBoundedHttpFile(Transport& io,File& file,const char* mime){
  struct CloseFile {File& file;~CloseFile(){file.close();}} closeFile{file};
  BoundedHttpWriter<Transport> writer(io);
  const size_t size=file.size();
  if(size>kMaxHttpWebFileBytes)return HttpSendResult::FileTooLarge;
  if(!writer.header(200,mime,size))return HttpSendResult::HeaderFailed;
  uint8_t buffer[256];
  size_t remaining=size;
  while(remaining){
    if(!writer.available())return HttpSendResult::BodyFailed;
    const size_t wanted=remaining>sizeof(buffer)?sizeof(buffer):remaining;
    const int got=file.read(buffer,wanted);
    if(got<=0 || size_t(got)>wanted)return HttpSendResult::ReadFailed;
    // Keep this buffer and offset until fully sent. Never read ahead.
    if(!writer.write(reinterpret_cast<const char*>(buffer),size_t(got)))return HttpSendResult::BodyFailed;
    remaining-=size_t(got);
  }
  return HttpSendResult::Complete;
}

#pragma once
#include <stdint.h>
#include <stdlib.h>
// Project WebServer parser overlay only; no SDK/TCP or global Stream patch.
extern "C" void observeHttpRequestBegin();
extern "C" void observeHttpRequestEnd(uint32_t reason,uint32_t bytes,uint32_t elapsed);
enum class HttpRequestEnd:uint8_t {
  Complete,Deadline,NoProgress,ReadFailed,Disconnected,StringAllocation,
  LineLimit,TotalLimit,LineCount,BadSyntax,BodyLimit,BodyAllocation,Unsupported,ParseRejected
};
template<class Client> class BoundedHttpRequest {
  Client& client_;
  uint32_t start_=millis(),progress_=start_,yieldAt_=start_,bytes_=0;
  uint16_t lineBytes_=0,lines_=0,burst_=0;
  uint32_t contentLength_=0;
  bool haveLength_=false,headersDone_=false;
  HttpRequestEnd reason_=HttpRequestEnd::ParseRejected;
  bool budget(){
    const uint32_t now=millis();
    if(uint32_t(now-start_)>=1000)return reject(HttpRequestEnd::Deadline);
    if(uint32_t(now-progress_)>=200)return reject(HttpRequestEnd::NoProgress);
    return true;
  }
  int byte(){
    while(budget()){
      // Blocking/yielding progress is explicit; read() is never wrapped in timedRead.
      if(burst_>=32 || uint32_t(millis()-yieldAt_)>=2){delay(1);yieldAt_=millis();burst_=0;if(!budget())return -1;}
      const int available=client_.available();
      if(available<0){reject(HttpRequestEnd::ReadFailed);return -1;}
      if(available>0){
        const int c=client_.read();
        if(c<0){reject(HttpRequestEnd::ReadFailed);return -1;}
        if(++bytes_>12288){reject(HttpRequestEnd::TotalLimit);return -1;}
        progress_=millis();++burst_;return c;
      }
      if(!client_.connected()){reject(HttpRequestEnd::Disconnected);return -1;}
      delay(1);yieldAt_=millis();burst_=0;
    }
    return -1;
  }
  template<class Text> bool validateHeader(const Text& line){
    if(lines_==1)return line.length()!=0 || reject(HttpRequestEnd::BadSyntax);
    if(!line.length()){headersDone_=true;return true;}
    const int colon=line.indexOf(':');
    if(colon<=0)return reject(HttpRequestEnd::BadSyntax);
    const char* p=line.c_str();
    for(int i=0;i<colon;++i){
      const char c=p[i];
      if(!((c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='-'||c=='_'))return reject(HttpRequestEnd::BadSyntax);
    }
    auto nameIs=[&](const char* name){
      int i=0;for(;i<colon&&name[i];++i){char c=p[i];if(c>='A'&&c<='Z')c+=32;if(c!=name[i])return false;}
      return i==colon&&!name[i];
    };
    if(nameIs("transfer-encoding"))return reject(HttpRequestEnd::Unsupported);
    if(nameIs("content-length")){
      if(haveLength_)return reject(HttpRequestEnd::BadSyntax);
      haveLength_=true;unsigned i=unsigned(colon)+1;
      while(p[i]==' '||p[i]=='\t')++i;
      if(p[i]<'0'||p[i]>'9')return reject(HttpRequestEnd::BadSyntax);
      for(;p[i]>='0'&&p[i]<='9';++i){contentLength_=contentLength_*10+unsigned(p[i]-'0');if(contentLength_>4096)return reject(HttpRequestEnd::BodyLimit);}
      while(p[i]==' '||p[i]=='\t')++i;
      if(p[i])return reject(HttpRequestEnd::BadSyntax);
    }
    return true;
  }
 public:
  explicit BoundedHttpRequest(Client& client):client_(client){observeHttpRequestBegin();}
  ~BoundedHttpRequest(){
    if(reason_!=HttpRequestEnd::Complete)client_.stop();
    observeHttpRequestEnd(uint32_t(reason_),bytes_,uint32_t(millis()-start_));
  }
  bool reject(HttpRequestEnd reason){reason_=reason;return false;}
  template<class Text> bool line(Text& out){
    if(headersDone_)return reject(HttpRequestEnd::BadSyntax);
    if(++lines_>66)return reject(HttpRequestEnd::LineCount);
    out="";lineBytes_=0;
    for(;;){
      const int c=byte();if(c<0)return false;
      if(bytes_>8192)return reject(HttpRequestEnd::TotalLimit);
      if(c=='\r'){
        const int lf=byte();if(lf<0)return false;
        if(lf!='\n')return reject(HttpRequestEnd::BadSyntax);
        if(bytes_>8192)return reject(HttpRequestEnd::TotalLimit);
        return validateHeader(out);
      }
      if(c=='\n'||c==0||(c<32&&c!='\t'))return reject(HttpRequestEnd::BadSyntax);
      if(++lineBytes_>1024)return reject(HttpRequestEnd::LineLimit);
      if(!out.concat(char(c)))return reject(HttpRequestEnd::StringAllocation);
    }
  }
  bool headers(bool bodyAllowed){
    if(!headersDone_ || (!bodyAllowed&&contentLength_))return reject(HttpRequestEnd::BadSyntax);
    return true;
  }
  char* body(size_t length,size_t& got){
    got=0;
    if(length!=contentLength_ || length>4096){reject(HttpRequestEnd::BodyLimit);return nullptr;}
    if(!length)return nullptr;
    char* data=static_cast<char*>(malloc(length+1));
    if(!data){reject(HttpRequestEnd::BodyAllocation);return nullptr;}
    while(got<length){const int c=byte();if(c<0){free(data);return nullptr;}data[got++]=char(c);}
    data[length]=0;return data;
  }
  bool complete(){if(reason_!=HttpRequestEnd::ParseRejected || !budget())return false;reason_=HttpRequestEnd::Complete;return true;}
};

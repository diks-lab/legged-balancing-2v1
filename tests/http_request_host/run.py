"""Actual checked _parseRequest plus actual RX cache, cooperative fault injection."""
from pathlib import Path
import importlib.util,tempfile,subprocess,os,hashlib,json
spec=importlib.util.spec_from_file_location('overlay','scripts/bounded_http_request_overlay.py');overlay=importlib.util.module_from_spec(spec);spec.loader.exec_module(overlay)
root=Path(os.environ.get('PLATFORMIO_CORE_DIR','/workspace/pio-core'))/'packages/framework-arduinoespressif32'
raw=(root/'libraries/WebServer/src/Parsing.cpp').read_bytes()
patched=overlay.transform(raw).decode()
assert overlay.transform(raw).decode()==patched  # repeat generation is deterministic
assert overlay.transform(raw.replace(b'\r\n',b'\n').replace(b'\n',b'\r\n')).decode()==patched
try:overlay.transform(raw+b' drift');raise AssertionError('accepted unknown parser')
except RuntimeError:pass
assert (root/'libraries/WebServer/src/Parsing.cpp').read_bytes()==raw
start=patched.index('bool WebServer::_parseRequest');end=patched.index('bool WebServer::_collectHeader',start)
parse=patched[start:end]
rx=(root/'libraries/Network/src/NetworkClient.cpp').read_text();rx=rx[rx.index('class NetworkClientRxBuffer {'):rx.index('class NetworkClientSocketHandle {')]
pre=r'''
#include <cassert>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <algorithm>
#include <stdint.h>
#include <string>
uint32_t now=0,idle=0,bytesSinceYield=0,maxBurst=0;
uint32_t millis(){return now;}
void delay(unsigned n){assert(n>=1);now+=n;++idle;maxBurst=std::max(maxBurst,bytesSinceYield);bytesSinceYield=0;}
bool failRx=false,failBody=false;unsigned rxFailures=0;
extern "C" void* __real_malloc(size_t);
extern "C" void* __wrap_malloc(size_t n){if(failRx&&n==1436){++rxFailures;return nullptr;}if(failBody&&n!=1436)return nullptr;return __real_malloc(n);}
int failConcat=-1;
struct String {
 std::string s;
 String()=default;String(const char*p):s(p?p:""){}
 bool concat(char c){if(failConcat==0)return false;if(failConcat>0)--failConcat;s+=c;return true;}
 unsigned length()const{return s.size();}const char*c_str()const{return s.c_str();}
 int indexOf(char c,int a=0)const{if(a<0)return -1;auto n=s.find(c,a);return n==std::string::npos?-1:int(n);}
 String substring(int a,int b=100000)const{String r;if(a<0)a=0;if(unsigned(a)<s.size())r.s=s.substr(a,std::max(0,std::min(int(s.size()),b)-a));return r;}
 bool operator==(const char*p)const{return s==p;}bool operator!=(const char*p)const{return s!=p;}
 String&operator+=(char c){s+=c;return *this;}String&operator+=(const String&r){s+=r.s;return *this;}String&operator+=(const char*p){s+=p;return *this;}
 bool equalsIgnoreCase(const char*p)const{if(strlen(p)!=s.size())return false;for(unsigned i=0;i<s.size();++i)if(tolower(s[i])!=tolower(p[i]))return false;return true;}
 bool startsWith(const char*p)const{return s.find(p)==0;}
 void trim(){auto a=s.find_first_not_of(" \t");if(a==std::string::npos){s.clear();return;}s=s.substr(a,s.find_last_not_of(" \t")-a+1);}
 long toInt()const{return strtol(s.c_str(),nullptr,10);}
 void replace(const char*a,const char*b){auto p=s.find(a);if(p!=std::string::npos)s.replace(p,strlen(a),b);}
};
#define F(x) x
#define FPSTR(x) x
#define log_e(...) ((void)0)
#define log_v(...) ((void)0)
#define ESP_IDF_VERSION_MAJOR 5
constexpr int FIONREAD=1,MSG_DONTWAIT=1;
struct Peer {std::string data;size_t pos=0;unsigned interval=0;bool open=true,readError=false;size_t chunk=1436,failAt=SIZE_MAX;}peer;
int lwip_ioctl(int,int,int* count){*count=peer.pos<peer.data.size()&&now>=peer.pos*peer.interval?int(peer.data.size()-peer.pos):0;return 0;}
int recv(int,uint8_t*out,size_t n,int){if(peer.readError||peer.pos>=peer.failAt)return -1;n=std::min(n,peer.chunk);size_t ready=0;while(ready<n&&peer.pos+ready<peer.data.size()&&now>=(peer.pos+ready)*peer.interval)++ready;memcpy(out,peer.data.data()+peer.pos,ready);peer.pos+=ready;return ready;}
'''
post=r'''
struct NetworkClient {
 NetworkClientRxBuffer rx{1};bool stopped=false;unsigned closes=0;
 int available(){return stopped?0:int(rx.available());}
 int read(){uint8_t c;int n=rx.read(&c,1);++bytesSinceYield;return n==1?int(c):-1;}
 bool connected(){return !stopped&&peer.open;}
 void stop(){if(!stopped)++closes;stopped=true;}
 void flush(){}
};
uint32_t reason=99,observedBytes=0,elapsed=0;unsigned begins=0,ends=0;
extern "C" void observeHttpRequestBegin(){++begins;}
extern "C" void observeHttpRequestEnd(uint32_t r,uint32_t b,uint32_t ms){++ends;reason=r;observedBytes=b;elapsed=ms;}
#include "BoundedHttpRequest.h"
enum HTTPMethod{HTTP_GET,HTTP_HEAD,HTTP_POST,HTTP_PUT,HTTP_PATCH,HTTP_DELETE,HTTP_ANY=255};
const char*_http_method_str[]={"GET","HEAD","POST","PUT","PATCH","DELETE"};
const char*Content_Type="Content-Type";
namespace mime {enum{txt};struct Entry {const char*mimeType;};Entry mimeTable[]={{"text/plain"}};}
struct RequestHandler {RequestHandler*next(){return nullptr;}bool canHandle(...){return true;}bool canRaw(...){return false;}};
struct RequestArgument {String key,value;};
struct WebServer {
 unsigned _headerKeysCount=0;RequestArgument _currentHeaders[8];
 int _currentVersion=0,_clientContentLength=0,_currentArgCount=0;bool _chunked=false;
 String _currentUri,_hostHeader;HTTPMethod _currentMethod=HTTP_ANY;
 RequestHandler handler,*_firstHandler=&handler,*_currentHandler=nullptr;
 RequestArgument _currentArgs[20];unsigned argsCalls=0;
 void _parseArguments(const String&){++argsCalls;}
 void _collectHeader(const char*,const char*){}
 bool _parseRequest(NetworkClient&client);
};
'''
checks=r'''
void reset(std::string data,unsigned interval=0){now=idle=bytesSinceYield=maxBurst=0;peer={data,0,interval,true,false};failRx=failBody=false;rxFailures=0;failConcat=-1;reason=99;begins=ends=0;}
void bad(HttpRequestEnd expected){NetworkClient client;WebServer server;assert(!server._parseRequest(client));assert(client.stopped&&client.closes==1);assert(reason==uint32_t(expected)&&begins==1&&ends==1);assert(elapsed<=1000);}
int main(){
 reset("GET / HTTP/1.1\r\nHost: esp\r\n\r\n");{NetworkClient c;WebServer w;assert(w._parseRequest(c));assert(!c.stopped&&reason==0&&w._currentUri=="/"&&w.argsCalls==1);}
 reset("POST /control/stop HTTP/1.1\r\nContent-Length: 0\r\n\r\n");{NetworkClient c;WebServer w;assert(w._parseRequest(c)&&w._currentMethod==HTTP_POST);}
 reset("POST /control/mode?x=1 HTTP/1.1\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: 3\r\n\r\na=1");{NetworkClient c;WebServer w;assert(w._parseRequest(c)&&w.argsCalls==1);}
 reset("GET / HTTP/1.1\r\nHost: esp\r\n\r\n");failRx=true;bad(HttpRequestEnd::ReadFailed);assert(rxFailures==1&&elapsed==0); // actual malloc(1436) failure, no repeated reads
 reset("GET / HTTP/1.1\r\n\r\n");peer.readError=true;bad(HttpRequestEnd::ReadFailed);
 reset("GET / HTTP/1.1\r\n\r\n");peer.chunk=8;peer.failAt=8;bad(HttpRequestEnd::ReadFailed);assert(observedBytes==8);
 for(const char*p:{"G","GET / HTTP/1.1","GET / HTTP/1.1\r","GET / HTTP/1.1\r\nHost: esp","GET / HTTP/1.1\r\nHost: esp\r\n"}){reset(p);bad(HttpRequestEnd::NoProgress);assert(idle>0&&elapsed>=200);}
 reset("");bad(HttpRequestEnd::NoProgress);assert(idle>=200);
 reset("");peer.open=false;bad(HttpRequestEnd::Disconnected);
 reset("GET / HTTP/1.1\r\n\r\n",100);bad(HttpRequestEnd::Deadline);assert(idle>=1000);
 reset("GET / HTTP/1.1\n\n");bad(HttpRequestEnd::BadSyntax);
 reset("GET / HTTP/1.1\rX");bad(HttpRequestEnd::BadSyntax);
 reset("GET / HTTP/1.1\r\ninvalid\r\n\r\n");bad(HttpRequestEnd::BadSyntax);
 reset("GET / HTTP/1.1\r\n\r\n");failConcat=0;bad(HttpRequestEnd::StringAllocation);
 reset("GET /"+std::string(1024,'x')+" HTTP/1.1\r\n\r\n");bad(HttpRequestEnd::LineLimit);assert(idle>0&&maxBurst<=32);
 std::string headers="GET / HTTP/1.1\r\n";for(int i=0;i<10;++i)headers+="X: "+std::string(900,'x')+"\r\n";reset(headers+"\r\n");bad(HttpRequestEnd::TotalLimit);assert(maxBurst<=32);
 headers="GET / HTTP/1.1\r\n";for(int i=0;i<66;++i)headers+="X: x\r\n";reset(headers+"\r\n");bad(HttpRequestEnd::LineCount);
 for(const char*length:{"-1","abc","1x",""}){reset(std::string("POST /control/start HTTP/1.1\r\nContent-Length: ")+length+"\r\n\r\n");bad(HttpRequestEnd::BadSyntax);}
 reset("POST /control/start HTTP/1.1\r\nContent-Length: 4097\r\n\r\n");bad(HttpRequestEnd::BodyLimit);
 reset("POST /control/start HTTP/1.1\r\nContent-Length: 1\r\nContent-Length: 1\r\n\r\nx");bad(HttpRequestEnd::BadSyntax);
 reset("POST /control/start HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n");bad(HttpRequestEnd::Unsupported);
 reset("POST /control/start HTTP/1.1\r\nContent-Type: multipart/form-data; boundary=x\r\nContent-Length: 0\r\n\r\n");bad(HttpRequestEnd::Unsupported);
 reset("GET / HTTP/1.1\r\nContent-Length: 1\r\n\r\nx");bad(HttpRequestEnd::BadSyntax);
 reset("POST /control/start HTTP/1.1\r\nContent-Length: 3\r\n\r\na");bad(HttpRequestEnd::NoProgress);assert(idle>0);
 reset("POST /control/start HTTP/1.1\r\nContent-Length: 3\r\n\r\nabc");failBody=true;bad(HttpRequestEnd::BodyAllocation);
 reset("GET / HTTP/1.1\r\n\r\n");now=UINT32_MAX-10;{NetworkClient c;WebServer w;assert(w._parseRequest(c));}
 // A failed/closed request cannot poison a subsequent accepted connection.
 reset("GET /api/status HTTP/1.1\r\nHost: esp\r\n\r\n");{NetworkClient c;WebServer w;assert(w._parseRequest(c)&&!c.stopped);}
 puts("PASS: real overlaid parser + real RX cache: RX/String/body OOM, read error, stalled/slow/incomplete/malformed requests, total/idle/size budgets, Idle opportunities, valid GET/POST and close/recovery");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);cpp=p/'test.cpp';cpp.write_text(pre+rx+post+parse+checks);exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('HTTP_REQUEST_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-sign-compare','-Iinclude',str(cpp),'-o',str(exe),'-Wl,--wrap=malloc']+flags,check=True)
 subprocess.run([str(exe)],check=True)

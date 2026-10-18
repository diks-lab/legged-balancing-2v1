"""Actual state machine and installed core ownership methods, with fault injection."""
from pathlib import Path
import os,re,subprocess,tempfile
core=Path(os.environ.get('PLATFORMIO_CORE_DIR','/workspace/pio-core'))/'packages/framework-arduinoespressif32'
net=(core/'libraries/Network/src/NetworkClient.cpp').read_text()
methods=net[net.index('NetworkClient::NetworkClient()'):net.index('int NetworkClient::connect(IPAddress')]
fs=(core/'libraries/FS/src/FS.cpp').read_text()
a=fs.index('void File::close()');b=fs.index('\nFile::operator bool()',a)
close=fs[a:b]
header=(core/'libraries/Network/src/NetworkClient.h').read_text()
assert 'std::shared_ptr<NetworkClientSocketHandle> clientSocketHandle' in header
assert 'std::shared_ptr<NetworkClientRxBuffer> _rxBuffer' in header
assets=[]
for name in ['index.html','style.css','app.js']:
 raw=(Path('data')/name).read_bytes().replace(b'\r\n',b'\n')
 for data in [raw,raw.replace(b'\n',b'\r\n')]:
  assets.append('"'+''.join('\\x%02x'%v for v in data)+'"')
code=r'''
#include "ContinuingHttpFile.h"
#include <cassert>
#include <cstdlib>
#include <new>
#include <memory>
#include <cstdio>
#include <initializer_list>
bool failAllocation=false;
extern "C" void* __real_malloc(size_t);
extern "C" void* __real_calloc(size_t,size_t);
extern "C" void* __real_realloc(void*,size_t);
extern "C" void* __wrap_malloc(size_t n){return failAllocation?nullptr:__real_malloc(n);}
extern "C" void* __wrap_calloc(size_t n,size_t s){return failAllocation?nullptr:__real_calloc(n,s);}
extern "C" void* __wrap_realloc(void*p,size_t n){return failAllocation?nullptr:__real_realloc(p,n);}
void* operator new(size_t n){if(failAllocation)throw std::bad_alloc();if(void*p=malloc(n))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{free(p);}void operator delete(void*p,size_t)noexcept{free(p);}
struct Stats {unsigned socketCloses=0,socketDeaths=0,rxDeaths=0,fileCloses=0,fileDeaths=0,reads=0;} stats;
struct NetworkClientSocketHandle {
 int socket;explicit NetworkClientSocketHandle(int fd):socket(fd){}
 void close(){if(socket>=0){socket=-1;++stats.socketCloses;}}
 ~NetworkClientSocketHandle(){close();++stats.socketDeaths;}
 int fd(){return socket;}
};
struct NetworkClientRxBuffer {explicit NetworkClientRxBuffer(int){}~NetworkClientRxBuffer(){++stats.rxDeaths;}};
constexpr int WIFI_CLIENT_DEF_CONN_TIMEOUT_MS=3000;
struct NetworkClient {
 std::shared_ptr<NetworkClientSocketHandle> clientSocketHandle;
 std::shared_ptr<NetworkClientRxBuffer> _rxBuffer;
 bool _connected=false,_sse=false;int _timeout=0,_lastWriteTimeout=0,_lastReadTimeout=0;
 NetworkClient*next=nullptr;
 NetworkClient();NetworkClient(int);~NetworkClient();void stop();
 int fd(){return clientSocketHandle?clientSocketHandle->fd():-1;}
};
'''+methods+r'''
struct IOState {
 char output[20000]{};size_t used=0;unsigned sends=0;
 uint32_t tick=0;bool peer=true;int maxWrite=512;
 int script[80]{};unsigned scriptSize=0,scriptPosition=0;
 size_t bodyBytes()const{const char*p=strstr(output,"\r\n\r\n");return p?used-size_t(p-output)-4:0;}
} ;
struct FileImpl {
 const char* data;size_t size,pos=0;IOState*io;bool closed=false;
 int failRead=0;bool oversize=false;size_t shortRead=256;
 FileImpl(const char*p,size_t n,IOState*i):data(p),size(n),io(i){}
 void close(){if(!closed){closed=true;++stats.fileCloses;}}
 ~FileImpl(){close();++stats.fileDeaths;}
 int read(uint8_t*out,size_t n){
  assert(!closed&&io->bodyBytes()==pos);++stats.reads;
  if(failRead&&stats.reads==unsigned(failRead))return 0;
  if(oversize)return int(n+1);
  if(n>shortRead)n=shortRead;
  if(n>size-pos)n=size-pos;
  memcpy(out,data+pos,n);pos+=n;return int(n);
 }
};
struct File {
 std::shared_ptr<FileImpl> _p;
 File()=default;File(const char*p,size_t n,IOState*i):_p(std::make_shared<FileImpl>(p,n,i)){}
 size_t size()const{return _p?_p->size:0;}
 int read(uint8_t*p,size_t n){return _p?_p->read(p,n):-1;}
 void close();
};
'''+close+r'''
using Response=ContinuingHttpFile<NetworkClient,File>;
struct Transport {
 Response& response;IOState& io;
 uint32_t now(){return io.tick;}
 bool connected(){return io.peer&&response.client().fd()>=0;}
 int writeSome(const char*p,size_t n){
  ++io.sends;int result=io.maxWrite;
  if(io.scriptPosition<io.scriptSize)result=io.script[io.scriptPosition++];
  if(result<=0)return result;
  size_t take=n<size_t(result)?n:size_t(result);
  assert(io.used+take<sizeof(io.output));memcpy(io.output+io.used,p,take);io.used+=take;
  return int(take);
 }
};
const char* assets[]={
'''+',\n'.join(assets)+r'''
};
void start(Response&r,IOState&io,const char*data,const char*mime="text/html; charset=utf-8",size_t size=0){
 assert(!failAllocation);stats=Stats{};
 NetworkClient server(7);File local(data,size?size:strlen(data),&io);
 assert(r.begin(server,local,mime,io.tick,1,true));
 // Actual core copy ownership: WebServer drops its reference after handler return.
 server=NetworkClient();local=File();
 if(r.active())assert(stats.socketCloses==0&&stats.fileCloses==0&&stats.rxDeaths==0);
 failAllocation=true;assert(malloc(1)==nullptr); // Continuation itself must allocate nothing.
}
void step(Response&r,IOState&io){
 Transport t{r,io};const auto sends=io.sends,reads=stats.reads;
 r.poll(t);assert(io.sends-sends<=1&&stats.reads-reads<=1);
 assert(!((io.sends-sends)&&(stats.reads-reads))); // No send/read loop in one task iteration.
}
void pump(Response&r,IOState&io,unsigned increment=1){for(int i=0;i<1000&&r.active();++i){step(r,io);io.tick+=increment;}assert(r.finished());}
void released(){assert(stats.socketCloses==1&&stats.socketDeaths==1&&stats.rxDeaths==1&&stats.fileCloses==1&&stats.fileDeaths==1);failAllocation=false;}
int main(){
 for(unsigned i=0;i<6;++i){Response r;IOState io;start(r,io,assets[i]);pump(r,io);
  assert(r.metrics().reason==HttpFileEnd::Complete&&r.metrics().bodySent==strlen(assets[i]));
  assert(io.bodyBytes()==strlen(assets[i])&&!memcmp(strstr(io.output,"\r\n\r\n")+4,assets[i],strlen(assets[i])));
  assert(r.metrics().wireSent==r.metrics().wireTotal&&io.used==r.metrics().wireTotal);released();
  r.acknowledge();assert(!r.active()&&!r.finished());
 }
 assert(strlen(assets[0])==1243&&strlen(assets[1])==1253);
 {Response r;IOState io;io.maxWrite=127;start(r,io,assets[1]);pump(r,io);assert(r.metrics().reason==HttpFileEnd::Complete);released();}
 {Response r;IOState io;io.script[0]=-EAGAIN;io.script[1]=17;io.script[2]=512;io.script[3]=-EAGAIN;io.scriptSize=4;
  start(r,io,assets[0]);step(r,io);assert(r.active()&&io.used==0&&io.sends==1); // EAGAIN returns, does not retry inline.
  pump(r,io);assert(r.metrics().reason==HttpFileEnd::Complete&&r.metrics().retries==2);released();}
 {Response r;IOState io;int script[]={-EAGAIN,512,-EAGAIN,256,-EAGAIN};memcpy(io.script,script,sizeof(script));io.scriptSize=5;
  start(r,io,assets[0]);pump(r,io);assert(r.metrics().reason==HttpFileEnd::RetryLimit&&r.metrics().retries==3&&r.metrics().calls==5&&r.metrics().bodySent==256);released();}
 {Response r;IOState io;start(r,io,assets[0]);step(r,io);io.tick=50;step(r,io);
  assert(r.metrics().reason==HttpFileEnd::Deadline&&r.metrics().elapsed==50&&r.metrics().bodySent==0&&r.metrics().calls==1);released();}
 {Response r;IOState io;io.script[0]=512;io.script[1]=17;io.scriptSize=2;start(r,io,assets[0]);step(r,io);step(r,io);step(r,io);
  io.tick=51;step(r,io);assert(r.metrics().reason==HttpFileEnd::Deadline&&r.metrics().bodySent==17&&r.metrics().stage==HttpFilePhase::Body);released();}
 {Response r;IOState io;io.maxWrite=1;start(r,io,assets[0]);pump(r,io,0);assert(r.metrics().reason==HttpFileEnd::CallLimit&&r.metrics().calls==64);released();}
 {Response r;IOState io;start(r,io,assets[0]);io.peer=false;step(r,io);assert(r.metrics().reason==HttpFileEnd::Disconnected&&stats.reads==0);released();}
 for(int error:{-ENOMEM,-ECONNRESET,-EPIPE,0}){Response r;IOState io;io.script[0]=512;io.script[1]=error;io.scriptSize=2;
  start(r,io,assets[0]);pump(r,io);assert(r.metrics().reason==(error?HttpFileEnd::SocketError:HttpFileEnd::Disconnected));
  assert(r.metrics().stage==HttpFilePhase::Body&&r.metrics().lastErrno==-error);released();}
 {Response r;IOState io;start(r,io,assets[0]);step(r,io);io.peer=false;step(r,io);assert(r.metrics().reason==HttpFileEnd::Disconnected&&stats.reads==0);released();}
 {Response r;IOState io;start(r,io,assets[0]);r.finish(HttpFileEnd::WifiLost,12);assert(r.metrics().reason==HttpFileEnd::WifiLost);released();}
 {Response r;IOState io;stats=Stats{};NetworkClient server(7);File f(assets[0],strlen(assets[0]),&io);f._p->failRead=2;
  assert(r.begin(server,f,"text/html",0,1,true));server=NetworkClient();f=File();failAllocation=true;
  pump(r,io);assert(r.metrics().reason==HttpFileEnd::ReadError&&r.metrics().bodySent==256&&stats.reads==2);released();}
 {Response r;IOState io;start(r,io,"");pump(r,io);assert(r.metrics().reason==HttpFileEnd::Complete&&stats.reads==0&&r.metrics().bodyTotal==0);released();}
 {Response r;IOState io;stats=Stats{};NetworkClient server(7);File f(assets[0],strlen(assets[0]),&io);f._p->shortRead=100;
  assert(r.begin(server,f,"text/html",0,1,true));server=NetworkClient();f=File();failAllocation=true;
  pump(r,io);assert(r.metrics().reason==HttpFileEnd::Complete&&stats.reads>5);released();}
 {Response r;IOState io;stats=Stats{};NetworkClient server(7);File f(assets[0],strlen(assets[0]),&io);f._p->oversize=true;
  assert(r.begin(server,f,"text/html",0,1,true));server=NetworkClient();f=File();failAllocation=true;
  pump(r,io);assert(r.metrics().reason==HttpFileEnd::ReadError);released();}
 {Response r;IOState io;char mime[300];memset(mime,'x',sizeof(mime));mime[299]=0;start(r,io,assets[0],mime);
  assert(r.finished()&&r.metrics().reason==HttpFileEnd::Invalid);released();}
 {Response r;IOState io;start(r,io,assets[0],"text/html",8193);assert(r.finished()&&r.metrics().reason==HttpFileEnd::SizeLimit);released();}
 {Response r;IOState io;start(r,io,assets[0],nullptr);assert(r.finished()&&r.metrics().reason==HttpFileEnd::Invalid);released();}
 {IOState io;{Response r;start(r,io,assets[0]);}released();} // destruction cancels owned resources
 {Response r;IOState io;start(r,io,assets[0]);r.finish(HttpFileEnd::Cancelled,1);released();
  r.acknowledge();IOState second;start(r,second,assets[0]);pump(r,second);assert(r.metrics().reason==HttpFileEnd::Complete);released();}
 {Response r;IOState io;start(r,io,assets[0]);failAllocation=false;NetworkClient extra(8);File f(assets[0],1,&io);
  assert(!r.begin(extra,f,"text/html",0,2,false)&&r.route==1&&r.active()&&stats.socketCloses==0);
  extra.stop();f.close();r.finish(HttpFileEnd::Cancelled,2);assert(stats.socketCloses==2&&stats.fileCloses==2);}
 {Response r;IOState io;io.tick=UINT32_MAX-8;start(r,io,assets[0]);pump(r,io);assert(r.metrics().reason==HttpFileEnd::Complete);released();}
 puts("PASS: LF/CRLF assets, actual core stop/copy lifetime after WebServer reference release, no allocation/read-ahead, partial/EAGAIN continuation, all budgets/reasons, disconnect/WiFi/cancel, single slot/reuse and release");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);src=p/'test.cpp';src.write_text(code,encoding='utf-8');exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('HTTP_CONTINUATION_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(src),'-o',str(exe),'-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc']+flags,check=True)
 subprocess.run([str(exe)],check=True)

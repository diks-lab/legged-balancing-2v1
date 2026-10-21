"""Production raw socket adapter/listener with syscall failures; no core wrappers."""
from pathlib import Path
import subprocess,tempfile
source=Path('src/main.cpp').read_text()
body=source[source.index('struct DiagnosticSocketIO {'):source.rindex('\n}')].replace('::close(', '::fakeClose(')
pre=r'''
#include "DiagnosticWebSocket.h"
#include "ReleaseMemoryObservation.h"
#include <cassert>
#include <cstring>
#include <cstdio>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <string>
uint32_t tick=0;uint32_t millis(){return tick;}
bool wifi_connection_ready=true,wifi_log_server_started=true;
int lock=0,diagnostic_snapshot_lock=0;
#define portENTER_CRITICAL(x) (++lock)
#define portEXIT_CRITICAL(x) (--lock)
diagnostic::Snapshot diagnostic_latest;
int release_memory_lock=0;ReleaseMemoryObserver release_memory_observer;
const unsigned MALLOC_CAP_INTERNAL=1,MALLOC_CAP_8BIT=2;
unsigned heap_caps_get_free_size(unsigned){assert(!lock);return 10000;}
unsigned heap_caps_get_largest_free_block(unsigned){assert(!lock);return 8000;}
unsigned esp_random(){return 12345678;}
int failure=0,offers=-1,closes=0,creates=0,reads=0,writes=0;bool listening=false;
int fakeClose(int){++closes;return 0;}
int fakeSocket(int,int,int){++creates;return failure==1?-1:50;}
int fakeSetSockOpt(int,int,int,const void*,unsigned){return 0;}
int fakeFcntl(int,int,int){return failure==2?-1:0;}
int fakeBind(int,const sockaddr*,unsigned){return failure==3?-1:0;}
int fakeListen(int,int n){assert(n==1);listening=failure!=4;return failure==4?-1:0;}
int fakeAccept(int,sockaddr*,unsigned*){int result=offers;offers=-1;return result;}
int fakeRecv(int,void*,size_t,int flags){assert(flags==MSG_DONTWAIT);++reads;errno=EAGAIN;return -1;}
int fakeSend(int,const void*,size_t,int flags){assert(flags==MSG_DONTWAIT);++writes;errno=ENOMEM;return -1;}
#define socket fakeSocket
#define setsockopt fakeSetSockOpt
#define fcntl fakeFcntl
#define bind fakeBind
#define listen fakeListen
#define accept fakeAccept
#define recv fakeRecv
#define send fakeSend
int mbedtls_base64_decode(unsigned char*,size_t,size_t*n,const unsigned char*p,size_t size){assert(size==24);if(memcmp(p,"dGhlIHNhbXBsZSBub25jZQ==",24))return -1;*n=16;return 0;}
int mbedtls_sha1(const unsigned char*p,size_t n,unsigned char*){assert(n==60);assert(!memcmp(p,"dGhlIHNhbXBsZSBub25jZQ==258EAFA5-E914-47DA-95CA-C5AB0DC85B11",60));return 0;}
int mbedtls_base64_encode(unsigned char*out,size_t cap,size_t*n,const unsigned char*,size_t size){assert(cap==29&&size==20);strcpy((char*)out,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");*n=28;return 0;}
'''
checks=r'''
int main(){
 for(failure=1;failure<=4;++failure){tick+=6000;int before=closes;serviceDiagnosticWebSocket();assert(diagnostic_listener<0);assert(closes-before==(failure==1?0:1));}
 failure=0;tick+=6000;serviceDiagnosticWebSocket();assert(diagnostic_listener==50&&listening);
 auto created=creates;offers=60;serviceDiagnosticWebSocket();assert(diagnostic_socket_io.fd==60&&diagnostic_websocket.active());
 int before=closes;offers=61;serviceDiagnosticWebSocket();assert(closes==before+1&&diagnostic_socket_io.fd==60); // second session rejected
 diagnostic::Snapshot s;diagnostic_socket_io.snapshot(s);assert(s.values[24]==10000&&s.values[25]==8000);
 char acceptValue[29];assert(diagnostic_socket_io.acceptKey("dGhlIHNhbXBsZSBub25jZQ==",acceptValue));assert(!strcmp(acceptValue,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));
 assert(!diagnostic_socket_io.acceptKey("invalidinvalidinvalidinv",acceptValue));
 wifi_connection_ready=false;serviceDiagnosticWebSocket();assert(diagnostic_socket_io.fd<0&&diagnostic_listener<0&&!diagnostic_websocket.active());
 before=closes;serviceDiagnosticWebSocket();assert(closes==before); // exactly-once cleanup, no motor/stop calls
 wifi_connection_ready=true;tick+=6000;offers=62;serviceDiagnosticWebSocket();assert(diagnostic_websocket.active()&&creates==created+1);
 diagnostic_socket_io.close();diagnostic_websocket.poll(diagnostic_socket_io);assert(!diagnostic_websocket.active());
 puts("PASS: production listener socket/fcntl/bind/listen failures, 1 client/reject second, raw nonblocking IO, Wi-Fi cleanup/retry, snapshot lock and RFC handshake adapter");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.cpp').write_text(pre+body+checks)
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(p/'test.cpp'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)

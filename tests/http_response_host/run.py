"""Actual bounded HTTP writer: heap exhaustion, partial writes and deadlines."""
from pathlib import Path
import os, subprocess, tempfile
code=r'''
#include "BoundedHttpResponse.h"
#include <cassert>
#include <cstdlib>
#include <new>
#include <cstdio>
bool failAllocation=false;
extern "C" void* __real_malloc(size_t);
extern "C" void* __real_calloc(size_t,size_t);
extern "C" void* __real_realloc(void*,size_t);
extern "C" void* __wrap_malloc(size_t n){return failAllocation?nullptr:__real_malloc(n);}
extern "C" void* __wrap_calloc(size_t n,size_t s){return failAllocation?nullptr:__real_calloc(n,s);}
extern "C" void* __wrap_realloc(void*p,size_t n){return failAllocation?nullptr:__real_realloc(p,n);}
void* operator new(size_t n){if(failAllocation)throw std::bad_alloc();if(void*p=malloc(n))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{free(p);}
void operator delete(void*p,size_t)noexcept{free(p);}
struct Transport {
 char output[8192]{};size_t used=0;unsigned calls=0,closed=0,yields=0;
 uint32_t tick=0,step=0;int result=512;bool intermittent=false;
 uint32_t now(){uint32_t t=tick;tick+=step;return t;}
 void stop(){++closed;}
 void yieldOnce(){++yields;++tick;}
 int writeSome(const char*p,size_t n){++calls;
   if(intermittent && calls%2) return -EAGAIN;
   if(result<=0)return result;
   size_t take=n>static_cast<size_t>(result)?result:n;
   assert(used+take<sizeof(output));memcpy(output+used,p,take);used+=take;return take;}
};
int main(){
 char data[2600];memset(data,'x',sizeof(data));
 HttpResponsePart parts[]={{"prefix\n",7},{data,sizeof(data)},{"tail\n",5}};
 failAllocation=true;assert(malloc(32)==nullptr && calloc(1,32)==nullptr && realloc(nullptr,32)==nullptr); // All paths must work even when C++ heap allocation fails.
 Transport ok;assert(sendBoundedHttpResponse(ok,200,parts,3));assert(ok.closed==1);
 assert(strstr(ok.output,"Content-Length: 2612\r\n"));assert(strstr(ok.output,"Connection: close\r\n"));
 Transport partial;partial.result=127;assert(sendBoundedHttpResponse(partial,200,parts,3));
 assert(partial.used==ok.used&&!memcmp(ok.output,partial.output,ok.used));
 Transport tiny;tiny.result=1;assert(!sendBoundedHttpResponse(tiny,200,parts,3)&&tiny.calls==64&&tiny.closed==1);
 Transport busy;busy.result=-EAGAIN;assert(!sendBoundedHttpResponse(busy,200,parts,3)&&busy.calls==3&&busy.yields==2&&busy.closed==1);
 Transport oscillating;oscillating.intermittent=true;assert(!sendBoundedHttpResponse(oscillating,200,parts,3)&&oscillating.calls==5&&oscillating.closed==1);
 Transport cut;cut.result=-ECONNRESET;assert(!sendBoundedHttpResponse(cut,200,parts,3)&&cut.calls==1&&cut.closed==1);
 Transport oom;oom.result=-ENOMEM;assert(!sendBoundedHttpResponse(oom,200,parts,3)&&oom.calls==1&&oom.closed==1);
 Transport zero;zero.result=0;assert(!sendBoundedHttpResponse(zero,200,parts,3)&&zero.calls==1&&zero.closed==1);
 Transport slow;slow.step=25;assert(!sendBoundedHttpResponse(slow,200,parts,3)&&slow.calls==1&&slow.closed==1);
 Transport wrap;wrap.tick=UINT32_MAX-10;assert(sendBoundedHttpResponse(wrap,200,parts,3)&&wrap.closed==1);
 HttpResponsePart invalid{nullptr,1};Transport null;assert(!sendBoundedHttpResponse(null,200,&invalid,1)&&null.calls==0&&null.closed==1);
 HttpResponsePart large{data,4097};Transport big;assert(!sendBoundedHttpResponse(big,200,&large,1)&&big.closed==1&&big.calls==0);
 HttpResponsePart empty{nullptr,0};Transport unavailable;assert(sendBoundedHttpResponse(unavailable,503,&empty,1));
 assert(strstr(unavailable.output,"503 Service Unavailable"));
 failAllocation=false;puts("PASS: allocation-free HTTP, content length, partial send, heap/send failure, EAGAIN total retry cap, disconnect, close on every path, call/deadline bounds, rollover");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);src=p/'test.cpp';src.write_text(code);exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('HTTP_RESPONSE_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Iinclude',str(src),'-o',str(exe),'-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc']+flags,check=True)
 subprocess.run([str(exe)],check=True)

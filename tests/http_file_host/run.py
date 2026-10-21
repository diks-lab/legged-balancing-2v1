"""Actual Web assets through the fixed reader/writer with failure injection."""
from pathlib import Path
import subprocess,tempfile,os
assets=[]
for name in ['index.html','style.css','app.js','signals.js','history.js','plot.js','subscribe.js','dashboard.js','panels.css']:
 data=(Path('data')/name).read_text(encoding='utf-8')
 assert ')ASSET"' not in data
 assets.append('R"ASSET('+data+')ASSET"')
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
void operator delete(void*p)noexcept{free(p);}void operator delete(void*p,size_t)noexcept{free(p);}
struct Transport {
 char output[32768]{};size_t used=0;unsigned calls=0,closed=0,yields=0;
 uint32_t tick=0;int maximum=512,result=512;bool body_fail=false,intermittent=false;int body_result=-EAGAIN,header_progress=512;
 bool headerDone()const{return strstr(output,"\r\n\r\n")!=nullptr;}
 size_t bodyBytes()const{const char*p=strstr(output,"\r\n\r\n");return p?used-size_t(p-output)-4:0;}
 uint32_t now(){return tick;}
 void stop(){++closed;}
 void yieldOnce(){++yields;++tick;}
 int writeSome(const char*p,size_t n){++calls;
  if(result<=0)return result;
  if(intermittent && calls%2)return -EAGAIN;
  if(body_fail&&headerDone())return body_result;
  size_t limit=headerDone()?size_t(maximum):size_t(header_progress);
  size_t take=n<limit?n:limit;assert(used+take<sizeof(output));memcpy(output+used,p,take);used+=take;return int(take);}
};
struct File {
 const char* data;size_t length;Transport& io;size_t offset=0;unsigned reads=0,closed=0;
 int fail_read=0;size_t short_read=256;uint32_t read_time=0;
 size_t size(){return length;}
 int read(uint8_t*out,size_t n){
  // A subsequent read is forbidden until EVERY byte of the previous read was sent.
  assert(io.bodyBytes()==offset);++reads;io.tick+=read_time;
  if(fail_read==int(reads))return 0;
  if(n>short_read)n=short_read;
  if(n>length-offset)n=length-offset;
  memcpy(out,data+offset,n);offset+=n;return int(n);}
 void close(){++closed;}
};
const char* assets[]={
'''+',\n'.join(assets)+r'''
};
int main(){
 failAllocation=true;assert(malloc(1)==nullptr);
 const char*mimes[]={"text/html; charset=utf-8","text/css","application/javascript","application/javascript","application/javascript","application/javascript","application/javascript","application/javascript","text/css"};
 for(int i=0;i<9;++i){
  Transport t;File f{assets[i],strlen(assets[i]),t};
  assert(sendBoundedHttpFile(t,f,mimes[i])==HttpSendResult::Complete);
  assert(f.closed==1&&t.closed==1&&t.bodyBytes()==f.length&&!memcmp(strstr(t.output,"\r\n\r\n")+4,assets[i],f.length));
  char contentLength[64];snprintf(contentLength,sizeof(contentLength),"Content-Length: %u\r\n",unsigned(f.length));
  assert(strstr(t.output,contentLength)&&strstr(t.output,mimes[i]));
 }
 Transport partial;partial.maximum=128;partial.header_progress=37;File pf{assets[0],strlen(assets[0]),partial};
 assert(sendBoundedHttpFile(partial,pf,mimes[0])==HttpSendResult::Complete);
 assert(partial.bodyBytes()==pf.length&&!memcmp(strstr(partial.output,"\r\n\r\n")+4,assets[0],pf.length));
 // The retired synchronous helper retains its 50ms/64-call bounds.
 // Larger formatted HTML with 100-byte reads must fail finitely, not relax them.
 Transport smallReads;File large{assets[0],strlen(assets[0]),smallReads};large.short_read=100;
 assert(sendBoundedHttpFile(smallReads,large,mimes[0])==HttpSendResult::BodyFailed);
 assert(smallReads.bodyBytes()<large.length&&large.closed==1&&smallReads.closed==1);
 Transport shortIO;File sf{assets[1],strlen(assets[1]),shortIO};sf.short_read=100;
 assert(sendBoundedHttpFile(shortIO,sf,mimes[1])==HttpSendResult::Complete);
 Transport headerFail;headerFail.result=-EAGAIN;File hf{assets[0],strlen(assets[0]),headerFail};
 assert(sendBoundedHttpFile(headerFail,hf,mimes[0])==HttpSendResult::HeaderFailed);
 assert(hf.reads==0&&hf.closed==1&&headerFail.closed==1&&headerFail.calls==3);
 Transport bodyFail;bodyFail.body_fail=true;File bf{assets[0],strlen(assets[0]),bodyFail};
 assert(sendBoundedHttpFile(bodyFail,bf,mimes[0])==HttpSendResult::BodyFailed);
 assert(bf.reads==1&&bf.offset==256&&bodyFail.bodyBytes()==0&&bf.closed==1&&bodyFail.closed==1);
 Transport cut;cut.result=-ECONNRESET;File cf{assets[0],strlen(assets[0]),cut};
 assert(sendBoundedHttpFile(cut,cf,mimes[0])==HttpSendResult::HeaderFailed&&cf.reads==0&&cf.closed==1&&cut.closed==1);
 Transport bodyCut;bodyCut.body_fail=true;bodyCut.body_result=-ECONNRESET;File bcf{assets[0],strlen(assets[0]),bodyCut};
 assert(sendBoundedHttpFile(bodyCut,bcf,mimes[0])==HttpSendResult::BodyFailed&&bcf.reads==1&&bcf.closed==1&&bodyCut.closed==1);
 Transport mixed;mixed.intermittent=true;File mf{assets[0],strlen(assets[0]),mixed};
 assert(sendBoundedHttpFile(mixed,mf,mimes[0])==HttpSendResult::BodyFailed&&mixed.calls==5&&mixed.yields==2&&mf.reads==2&&mf.closed==1&&mixed.closed==1);
 Transport readFail;File rf{assets[0],strlen(assets[0]),readFail};rf.fail_read=2;
 assert(sendBoundedHttpFile(readFail,rf,mimes[0])==HttpSendResult::ReadFailed&&rf.reads==2&&readFail.bodyBytes()==256&&rf.closed==1&&readFail.closed==1);
 Transport deadline;File df{assets[0],strlen(assets[0]),deadline};df.read_time=12;
 assert(sendBoundedHttpFile(deadline,df,mimes[0])==HttpSendResult::BodyFailed&&deadline.tick>=50&&df.closed==1&&deadline.closed==1);
 Transport callLimit;callLimit.maximum=16;File lf{assets[0],strlen(assets[0]),callLimit};
 assert(sendBoundedHttpFile(callLimit,lf,mimes[0])==HttpSendResult::BodyFailed&&callLimit.calls==64&&lf.closed==1&&callLimit.closed==1);
 Transport over;File of{assets[0],8193,over};
 assert(sendBoundedHttpFile(over,of,mimes[0])==HttpSendResult::FileTooLarge&&of.reads==0&&of.closed==1&&over.closed==1&&over.calls==0);
 Transport zero;File zf{"",0,zero};assert(sendBoundedHttpFile(zero,zf,mimes[0])==HttpSendResult::Complete&&zf.reads==0&&zf.closed==1&&zero.closed==1);
 failAllocation=false;puts("PASS: actual HTML/CSS/JS, allocation failure, MIME/length, partial/short IO, no read-ahead, header/body/read failure, disconnect, shared deadline/call budget, size cap and close on all paths");
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);cpp=p/'test.cpp';cpp.write_text(code,encoding='utf-8');exe=p/'test'
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('HTTP_FILE_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-Wno-missing-field-initializers','-Iinclude',str(cpp),'-o',str(exe),'-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc']+flags,check=True)
 subprocess.run([str(exe)],check=True)

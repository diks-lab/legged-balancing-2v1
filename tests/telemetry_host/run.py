from pathlib import Path
import subprocess,tempfile,os,json,re
# Browser choices/presets must use exactly the firmware's published schema.
header=Path('include/DiagnosticTelemetry.h').read_text()
firmware_fields=re.findall(r'"([a-z0-9_]+)"',header.split('fields[FieldCount]={',1)[1].split('};',1)[0])
browser_fields=json.loads(subprocess.check_output(['node','-e',
 'console.log(JSON.stringify(require("./data/signals.js").SIGNALS.map(s=>s[0])))'],text=True))
assert firmware_fields==browser_fields and len(firmware_fields)==32

code=r'''
#include "DiagnosticWebSocket.h"
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <new>
#include <string>
#include <algorithm>
bool forbid=false;
extern "C" void* __real_malloc(size_t);
extern "C" void* __real_calloc(size_t,size_t);
extern "C" void* __real_realloc(void*,size_t);
extern "C" void* __wrap_malloc(size_t n){assert(!forbid);return __real_malloc(n);}
extern "C" void* __wrap_calloc(size_t n,size_t s){assert(!forbid);return __real_calloc(n,s);}
extern "C" void* __wrap_realloc(void*p,size_t n){assert(!forbid);return __real_realloc(p,n);}

void*operator new(size_t n){assert(!forbid);if(auto*p=malloc(n))return p;throw std::bad_alloc();}
void operator delete(void*p)noexcept{free(p);}void operator delete(void*p,size_t)noexcept{free(p);}
struct IO{
 diagnostic::WsObservation events[64]{};unsigned eventCount=0;unsigned snapshotCost=0,clockStep=0;
 void observe(const diagnostic::WsObservation&r){assert(eventCount<64);events[eventCount++]=r;}
 bool wheelFault=false;
 uint32_t tick=0;bool open=true;unsigned closes=0,reads=0,writes=0;
 char input[16384]{},output[200000]{};size_t in=0,pos=0,out=0;int limit=256,readLimit=128,error=0,readError=0;
 uint32_t now(){auto n=tick;tick+=clockStep;return n;}bool connected(){return open;}
 void close(){if(open)++closes;open=false;}
 int read(char*p,size_t n){++reads;if(readError)return readError;if(pos==in)return -EAGAIN;size_t amount=std::min({n,in-pos,size_t(readLimit)});memcpy(p,input+pos,amount);pos+=amount;return amount;}
 int write(const char*p,size_t n){++writes;if(error)return error;size_t amount=std::min(n,size_t(limit));assert(out+amount<sizeof(output));memcpy(output+out,p,amount);out+=amount;return amount;}
 bool acceptKey(const char*key,char*out){if(strcmp(key,"dGhlIHNhbXBsZSBub25jZQ=="))return false;strcpy(out,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo=");return true;}
 void snapshot(diagnostic::Snapshot&s){tick+=snapshotCost;s.wheel_reads[0].latched=wheelFault;s.wheel_reads[0].probes=3;s.sample_ms=tick;s.source_seq=tick/50;s.values[2]=-2.42;s.values[6]=.15;s.values[7]=.14;s.values[24]=12345;}
 void raw(const char*p,size_t n){assert(in+n<sizeof(input));memcpy(input+in,p,n);in+=n;}
 void message(const std::string&msg,unsigned op=1,bool mask=true){size_t n=msg.size();unsigned char h[8]={uint8_t(0x80|op),uint8_t((mask?128:0)|(n<126?n:126)),0,0,1,2,3,4};size_t at=2;if(n>=126){h[2]=n>>8;h[3]=n;at=4;}if(mask){h[at]=1;h[at+1]=2;h[at+2]=3;h[at+3]=4;}raw((char*)h,at+(mask?4:0));for(size_t i=0;i<n;++i){char c=msg[i]^(mask?char(i%4+1):0);raw(&c,1);}}
};
using WS=diagnostic::WebSocket<IO>;
void poll(WS&w,IO&i,unsigned times=1,unsigned gap=2){for(unsigned k=0;k<times;++k){auto r=i.reads,s=i.writes;forbid=true;w.poll(i);forbid=false;assert(i.reads-r<=1&&i.writes-s<=1&&!(i.reads-r&&i.writes-s));i.tick+=gap;}}
void handshake(WS&w,IO&i){w.setBoot("0123456789abcdef");w.connected(i);const char*req="GET /diagnostics HTTP/1.1\r\nHost: esp:81\r\nUpgrade: websocket\r\nConnection: keep-alive, Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n";i.raw(req,strlen(req));poll(w,i,15);assert(w.ready()&&i.open);assert(strstr(i.output,"s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));i.out=0;}
const char*sub="{\"type\":\"subscribe\",\"request_id\":1,\"period_ms\":50,\"fields\":[\"angle_error_deg\",\"right_velocity_rad_s\"]}";
std::string payload(const IO&i,size_t&pos){assert(pos+2<=i.out);size_t n=uint8_t(i.output[pos+1])&127,h=2;if(n==126){n=(uint8_t(i.output[pos+2])<<8)|uint8_t(i.output[pos+3]);h=4;}assert(pos+h+n<=i.out);std::string result(i.output+pos+h,n);pos+=h+n;return result;}
int main(){
 diagnostic::Subscription s;assert(diagnostic::SubscriptionParser(sub).parse(s)&&s.mask==((1<<2)|(1<<6)));
 for(auto text:{"{}","[]","null","{\"type\":\"stop\",\"request_id\":1,}","{\"type\":\"stop\",\"request_id\":1,\"request_id\":2}","{\"type\":\"stop\",\"request_id\":-1}","{\"type\":\"stop\",\"request_id\":01}","{\"type\":\"stop\",\"request_id\":4294967296}"})assert(!diagnostic::SubscriptionParser(text).parse(s));
 std::string bad=sub;bad.replace(bad.find("50"),2,"10");assert(!diagnostic::SubscriptionParser(bad.c_str()).parse(s));
 bad=sub;bad.replace(bad.find("right_velocity_rad_s"),20,"angle_error_deg");assert(!diagnostic::SubscriptionParser(bad.c_str()).parse(s));
 diagnostic::Snapshot snap;for(auto&v:snap.values)v=NAN;strcpy(snap.stop_reason,"quote\"\\\n");char json[2048];assert(diagnostic::sample(json,sizeof(json),snap,"0123456789abcdef",1,1,0,UINT32_MAX,0));assert(strstr(json,"null")&&strstr(json,"quote\\\"\\\\\\u000a"));char tiny[8];assert(!diagnostic::sample(tiny,sizeof(tiny),snap,"x",1,1,0,1,0));
 {IO i;WS w;handshake(w,i);i.message(sub);poll(w,i,25);assert(w.generation()==1&&w.mask()==(diagnostic::PermanentMask|(1u<<2)|(1u<<6)));size_t at=0;auto ack=payload(i,at);assert(ack.find("subscribed")!=ack.npos);poll(w,i,40);bool saw=false;while(at<i.out){auto msg=payload(i,at);if(msg.find("sample")!=msg.npos){assert(msg.find("angle_error_deg")!=msg.npos&&msg.find("right_velocity_rad_s")!=msg.npos&&msg.find("left_velocity_rad_s")==msg.npos&&msg.find("internal_free_b")==msg.npos);puts(msg.c_str());saw=true;}}assert(saw);
  auto generation=w.generation();i.message("{\"type\":\"pid\",\"value\":99}");poll(w,i,30);assert(w.generation()==generation&&w.mask()==(diagnostic::PermanentMask|(1u<<2)|(1u<<6)));
  i.message("{\"type\":\"subscribe\",\"request_id\":2,\"period_ms\":50,\"fields\":[\"gyro_deg_s\"]}");poll(w,i,30);assert(w.generation()==2&&w.mask()==(diagnostic::PermanentMask|(1u<<3)));
  i.message("{\"type\":\"stop\",\"request_id\":3}");poll(w,i,30);assert(w.generation()==3&&w.mask()==0);w.disconnect(i);assert(i.closes==1);i.open=true;i.in=i.pos=i.out=0;handshake(w,i);assert(w.mask()==0&&w.generation()==3);i.message(sub);poll(w,i,25);assert(w.generation()==4&&w.mask()==(diagnostic::PermanentMask|(1u<<2)|(1u<<6)));w.disconnect(i);assert(i.closes==2);
 }
 // Fault diagnostics use the same bounded slot, independent of sample sequence.
 {IO i;WS w;handshake(w,i);i.wheelFault=true;i.message(sub);poll(w,i,650);
  assert(w.active());size_t at=0;unsigned wheel=0,samples=0;
  while(at+4<i.out){auto msg=payload(i,at);wheel+=msg.find("wheel_diagnostics")!=msg.npos;samples+=msg.find("\"sample\"")!=msg.npos;}
  assert(wheel>=1&&samples>=10);}
 // Reproduce real time advancing during snapshot/JSON and timer reads.
 for(unsigned cost:{1u,5u})for(unsigned step:{0u,1u}){
  IO i;WS w;handshake(w,i);i.snapshotCost=cost;i.clockStep=step;i.message(sub);poll(w,i,100);
  assert(w.active());unsigned prepared=0,written=0,sent=0;
  for(unsigned k=0;k<i.eventCount;++k){auto&e=i.events[k];
   if(e.event==diagnostic::WsEvent::FirstPrepared)++prepared;
   if(e.event==diagnostic::WsEvent::FirstWrite)++written;
   if(e.event==diagnostic::WsEvent::FirstSent){++sent;assert(e.sent==e.total&&e.seq>0);}
   assert(e.event!=diagnostic::WsEvent::Closed);
  }
  assert(prepared==1&&written==1&&sent==1);
  size_t offset=0;while(offset<i.out){auto text=payload(i,offset);if(text.find("\"type\":\"sample\"")!=text.npos){puts(text.c_str());break;}}
 }
 // All fields fit the single frame even with worst finite float text and escaped stop reason.
 {diagnostic::Snapshot all;for(auto&v:all.values)v=3.4028234e38f;memset(all.stop_reason,'\\',63);all.stop_reason[63]=0;memset(all.startup_reason,'R',31);all.startup_reason[31]=0;memset(all.right_error,'R',23);all.right_error[23]=0;memset(all.left_error,'L',23);all.left_error[23]=0;all.imu_ms=all.imu_seq=all.command_ms=UINT32_MAX;all.imu_reason=10;all.state=7;
  all.imu_valid=true;all.gyro_window={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,3.4028234e38f,3.4028234e38f};all.imu_config=255;
  all.health.first_ms=UINT32_MAX;strcpy(all.health.first,"IMU_CALIBRATION_FAILED");
  for(auto&state:all.health.stages)state=diagnostic::Blocked;
  all.health.barrier=18;all.health.foc_reason[0]=all.health.foc_reason[1]=9;
  all.health.imu_failure=6;all.health.mpu.failure=imu_config::Failure::Mismatch;
  all.health.mpu.reg=0x1b;all.health.mpu.expected=8;all.health.mpu.actual=all.health.mpu.read=255;all.health.mpu.tx=5;all.health.mpu.rx=0;
  for(auto&d:all.wheel_reads){d.latched=true;d.probes=UINT32_MAX;
   for(auto*r:{&d.first,&d.probe}){r->ms=r->elapsed_us=r->fall_ms=UINT32_MAX;r->tx=5;r->received=2;r->available=2;r->high=15;r->low=255;r->context=31;r->result=3;}}
  char buffer[2048];assert(diagnostic::sample(buffer,sizeof(buffer),all,"0123456789abcdef",UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX));
  char wheel[1024];assert(diagnostic::wheelReadMessage(wheel,sizeof(wheel),all,"0123456789abcdef"));
  assert(strstr(wheel,"\"wheel0\":[")&&strstr(wheel,"\"wheel1\":["));
  const char* start=strstr(buffer,"\"gyro_window\":");assert(start);const char* end=strchr(start,']');assert(end&&size_t(end-start+2)<=128);
  std::string request="{\"type\":\"subscribe\",\"request_id\":9,\"period_ms\":50,\"fields\":[";
  for(unsigned i=0;i<diagnostic::FieldCount;++i){if(i)request+=",";request+="\"";request+=diagnostic::fields[i];request+="\"";}request+="]}";
  diagnostic::Subscription accepted;assert(request.size()<=1000&&diagnostic::SubscriptionParser(request.c_str()).parse(accepted)&&accepted.mask==UINT32_MAX);
  IO i;WS w;handshake(w,i);i.message(request);poll(w,i,80);assert(w.mask()==UINT32_MAX&&w.generation()==1);
 }
 {IO i;WS w;i.tick=UINT32_MAX-100;handshake(w,i);i.snapshotCost=5;i.message(sub);poll(w,i,100);assert(w.active());}
 // Deterministic malformed bytes exercise bounded JSON scanning under sanitizers.
 {uint32_t seed=1;char input[1001];for(unsigned k=0;k<10000;++k){size_t n=k%1000;for(size_t i=0;i<n;++i){seed=seed*1664525+1013904223;input[i]=char(1+(seed%127));}input[n]=0;diagnostic::Subscription unused;diagnostic::SubscriptionParser(input).parse(unused);}}
 // Partial sends + temporary EAGAIN keep frame offsets; no heap or output queue growth.
 {IO i;WS w;handshake(w,i);i.limit=7;i.message(sub);poll(w,i,400);assert(i.open&&w.dropped()>0);size_t at=0;while(at<i.out){size_t saved=at;unsigned n=uint8_t(i.output[at+1])&127;size_t h=n==126?4:2;if(h==4)n=(uint8_t(i.output[at+2])<<8)|uint8_t(i.output[at+3]);if(saved+h+n>i.out)break;auto msg=payload(i,at);assert(msg.front()=='{'&&msg.back()=='}');}}
 {IO i;WS w;handshake(w,i);i.message(sub);poll(w,i,25);i.error=-EAGAIN;poll(w,i,400);assert(!w.active()&&i.closes==1&&w.dropped()>0);}
 {IO i;WS w;handshake(w,i);i.message(sub);poll(w,i,25);i.error=-ENOMEM;poll(w,i,400);assert(!w.active()&&i.closes==1);}
 {IO i;WS w;handshake(w,i);i.readError=-ENOMEM;poll(w,i);assert(!w.active()&&i.closes==1);}
 {IO i;WS w;w.connected(i);poll(w,i,120);assert(!w.active()&&i.tick<1000&&i.closes==1);}
 {IO i;WS w;handshake(w,i);i.message(std::string(1001,'x'));poll(w,i,10);assert(!w.active());}
 {IO i;WS w;handshake(w,i);i.message(sub,1,false);poll(w,i,10);assert(!w.active());}
 {IO i;WS w;handshake(w,i);char h=char(0x81);i.raw(&h,1);poll(w,i,120);assert(!w.active());}
 {IO i;WS w;handshake(w,i);i.message("test",9);poll(w,i,10);assert(uint8_t(i.output[0])==0x8a);w.disconnect(i);}
 printf("PASS: strict JSON/atomic subscription, selection-only sample, partial/EAGAIN/OOM, slow drop, parser bounds/timeouts, masking/ping, cleanup/reconnect; WS=%zu snapshot=%zu bytes\n",sizeof(WS),sizeof(diagnostic::Snapshot));
}
'''
with tempfile.TemporaryDirectory() as td:
 p=Path(td);(p/'test.cpp').write_text(code)
 flags=['-fsanitize=address,undefined','-fno-omit-frame-pointer'] if os.environ.get('TELEMETRY_SANITIZERS') else []
 subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I'+os.environ.get('TELEMETRY_HEADER_DIR','include'),'-Iinclude',str(p/'test.cpp'),'-o',str(p/'test'),'-Wl,--wrap=malloc,--wrap=calloc,--wrap=realloc']+flags,check=True)
 result=subprocess.run([str(p/'test')],capture_output=True,text=True)
 if result.returncode:print(result.stderr);result.check_returncode()
 samples=[]
 for line in result.stdout.splitlines():
  if line.startswith('{'):
   sample=json.loads(line);assert set(sample['values'])=={'angle_error_deg','right_velocity_rad_s','angle_deg','reference_deg','right_command_v','left_command_v'};assert sample['age_ms']<1000;samples.append(sample)
  else:print(line)

 if os.environ.get("TELEMETRY_SAMPLE_OUTPUT"):Path(os.environ["TELEMETRY_SAMPLE_OUTPUT"]).write_text(json.dumps(samples))

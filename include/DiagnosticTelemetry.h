#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdarg.h>
namespace diagnostic {
constexpr unsigned FieldCount=26;
static const char* const fields[FieldCount]={
 "angle_deg","reference_deg","angle_error_deg","gyro_deg_s",
 "right_position_rad","left_position_rad","right_velocity_rad_s","left_velocity_rad_s","mean_velocity_rad_s",
 "angle_term_v","gyro_term_v","position_term_v","speed_term_v","total_output_v",
 "right_command_v","left_command_v","right_foc_q_v","left_foc_q_v",
 "control_dt_ms","loop_us","right_foc_us","left_foc_us","imu_us","move_us","internal_free_b","internal_largest_b"};
struct Snapshot {
 uint32_t sample_ms=0,source_seq=0,foc_capture_us=0,stop_ms=0;
 uint32_t faults=0; bool running=false,disabled=true,ps4=false,foc_ready=false;
 char stop_reason[64]="NONE";
 float values[FieldCount]{};
};
struct Json {
 char* data;size_t capacity,used=0;bool ok=true;
 Json(char*p,size_t n):data(p),capacity(n){if(n)data[0]=0;else ok=false;}
 void add(const char* fmt,...){if(!ok)return;va_list ap;va_start(ap,fmt);int n=vsnprintf(data+used,capacity-used,fmt,ap);va_end(ap);if(n<0||size_t(n)>=capacity-used){ok=false;return;}used+=n;}
 void string(const char*p){add("\"");for(;*p&&ok;++p){unsigned char c=*p;if(c=='"'||c=='\\')add("\\%c",c);else if(c<32)add("\\u%04x",c);else add("%c",c);}add("\"");}
};
// Strict small JSON grammar. Unknown/duplicate keys, escapes, fractional periods,
// oversized strings/arrays and trailing input are rejected atomically.
struct Subscription { uint32_t request=0,mask=0;bool stop=false; };
class SubscriptionParser {
 const char*p;void ws(){while(*p==' '||*p=='\r'||*p=='\n'||*p=='\t')++p;}
 bool token(char c){ws();if(*p!=c)return false;++p;return true;}
 bool text(char*out,size_t size){if(!token('"'))return false;size_t n=0;while(*p&&*p!='"'){if(*p=='\\'||unsigned(*p)<32||n+1>=size)return false;out[n++]=*p++;}if(*p!='"')return false;++p;out[n]=0;return true;}
 bool number(uint32_t& v){ws();v=0;if(*p<'0'||*p>'9')return false;if(*p=='0'&&p[1]>='0'&&p[1]<='9')return false;while(*p>='0'&&*p<='9'){unsigned d=*p++-'0';if(v>(UINT32_MAX-d)/10)return false;v=v*10+d;}return true;}
 public:
 explicit SubscriptionParser(const char* input):p(input){}
 bool parse(Subscription& out){Subscription s;unsigned seen=0;char type[12]="";if(!token('{'))return false;
  do{char key[16];if(!text(key,sizeof(key))||!token(':'))return false;
   unsigned bit=0;
   if(!strcmp(key,"type")){bit=1;if(!text(type,sizeof(type)))return false;}
   else if(!strcmp(key,"request_id")){bit=2;if(!number(s.request))return false;}
   else if(!strcmp(key,"period_ms")){bit=4;uint32_t period;if(!number(period)||period!=50)return false;}
   else if(!strcmp(key,"fields")){bit=8;if(!token('['))return false;ws();if(*p!=']')do{char name[40];if(!text(name,sizeof(name)))return false;unsigned i=0;while(i<FieldCount&&strcmp(name,fields[i]))++i;if(i==FieldCount||(s.mask&(1u<<i)))return false;s.mask|=1u<<i;ws();if(*p!=',')break;++p;}while(true);if(!token(']'))return false;}
   else return false;
   if(seen&bit)return false;
   seen|=bit;ws();if(*p!=',')break;++p;
  }while(true);
  if(!token('}'))return false;
  ws();if(*p)return false;
  if(!strcmp(type,"subscribe")){if(seen!=15||!s.mask)return false;}
  else if(!strcmp(type,"stop")){if(seen!=3)return false;s.stop=true;}
  else return false;
  out=s;return true;
 }
};
inline bool sample(char*dest,size_t capacity,const Snapshot&s,const char*boot,uint32_t generation,uint32_t seq,uint32_t dropped,uint32_t mask,uint32_t now){
 Json j(dest,capacity);j.add("{\"type\":\"%s\",\"boot_id\":\"%s\",\"subscription_id\":%lu,\"seq\":%lu,\"source_seq\":%lu,\"sample_ms\":%lu,\"foc_capture_us\":%lu,\"age_ms\":%lu,\"dropped\":%lu,\"stop_ms\":%lu,\"stop_reason\":",mask?"sample":"status",boot,(unsigned long)generation,(unsigned long)seq,(unsigned long)s.source_seq,(unsigned long)s.sample_ms,(unsigned long)s.foc_capture_us,(unsigned long)(now-s.sample_ms),(unsigned long)dropped,(unsigned long)s.stop_ms);j.string(s.stop_reason);
 j.add(",\"faults\":%lu,\"running\":%s,\"disabled\":%s,\"ps4\":%s,\"foc_ready\":%s,\"values\":{",(unsigned long)s.faults,s.running?"true":"false",s.disabled?"true":"false",s.ps4?"true":"false",s.foc_ready?"true":"false");
 bool first=true;for(unsigned i=0;i<FieldCount;++i)if(mask&(1u<<i)){j.add("%s\"%s\":",first?"":",",fields[i]);if(isfinite(s.values[i]))j.add("%.7g",double(s.values[i]));else j.add("null");first=false;}
 j.add("}}");return j.ok;
}
inline bool ack(char*dest,size_t cap,const Subscription&s,uint32_t generation,const char*boot){Json j(dest,cap);j.add("{\"type\":\"subscribed\",\"boot_id\":\"%s\",\"request_id\":%lu,\"subscription_id\":%lu,\"period_ms\":50,\"streaming\":%s,\"fields\":[",boot,(unsigned long)s.request,(unsigned long)generation,s.stop?"false":"true");bool first=true;for(unsigned i=0;i<FieldCount;++i)if(s.mask&(1u<<i)){j.add("%s\"%s\"",first?"":",",fields[i]);first=false;}j.add("]}");return j.ok;}
}
